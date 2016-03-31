//******************************************************************************
//
//  Modem.c: Module Title
//
//      Copyright (c) 2001-2010, Aeromechanical Services Ltd.
//      ALL RIGHTS RESERVED
//
//  This module provides interfaces to the higher level ModemAPI
//  and the lower level ModemSerial.  The modem command language
//  (ATtention code) is sent when the API requires an event to occur.
//  The command is formed and sent to the serial interface.  When
//  the modem responds, it is sent back through ModemSerial and
//  parsed here, to determine if the command was executed successfully
//  or not.  The result is then reported back to ModemAPI.
//
//  In the case where a response was not completed successfully,
//  the upper level will be in charge of re-submitting the command
//  (i.e.: there is no retry capability at this time from the middle layer).
//  
//  This state machine that manages the modem interface must be updated
//  periodically in order to retrieve the current status of the Iridium 
//  modem. 
//
//  Date            Author      Comment
//  ----------------------------------------------------------------
//  2008-Sept-2     ZJ          Code Review
//
//******************************************************************************
  

//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#if defined( __BORLANDC__ ) || defined( WIN32 )
    #include "artl.h"
    #include "artlx.h"
    #ifdef __BORLANDC__
        #include "DebugOut.h"
        #include "Main.h"
        #include "comm32.h"
        #include "PcmciaAPIStub.h"
    #endif
#else
    #include "ARINC573_717.h"
    #include "CISAPI.h"
    #include "debug.h"
    #include "FaultHandler.h"
    #include "FileTransfer.h"
    #include "FileUtils.h"
    #include "HwWatchdog.h"
    #include "Modem.h"
    #include "ModemSerial.h"
    #include "ModemAPI.h"
    #include "ModemLog.h"
    #include "MsgHandler.h"
    #include "pcmciaAPI.h"
    #include "PowerManager.h"
    #include "rs422TxtMsg.h"
    #include "rulesbin.h"
    #include "SystemCfg.h"
    #include "SystemLog.h"
    #include "timer.h"
    #include "utils.h"
#endif

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


#define     STANDARD_RSP_TIMEOUT        5000

#define     CARRIAGE_RETURN             '\r'
#define     LINE_FEED                   '\n'
                               
#define     STR_SIZE                    10      // " 65535\0" = max size of rspnse field
#define     SATELLITE_RSP_TIMEOUT       65000   // in ms
#define     MAX_RX_SIZE                 sizeof( MODEM_RX_STRUCT )


// While sending a short burst data packet, there are several 
// command/response levels to the procedure.  Below is the state
// machine we must adhere to in order to successfully execute
// a "short burst data" message.
typedef BYTE    SUB_STATES;
enum sub_states
{
    SUBSTATE_NONE = 0,            // Initialize to.
    SEND_IMEI_CMD,                // Get the IMEI for the system information (done on init)
    SEND_MT_ALERT_CMD,            // Sets up the system to ensure the RI line is used only for incoming calls.
    SEND_MT_ALERT_RSP,            // Gets response from command.
    SEND_SBD_AUTOREG_CMD,         // Makes the modem query the satellite and update the system's geo location at regular intervals (for the MT-RA feature)
    SEND_SBD_AUTOREG_RSP,         // Gets the response from the command.
    SEND_SBD_DOWNLOAD_CMD,        // Makes the modem query the satellite and update the system's geo location and download any MT data
    SEND_TEXT_MSG,                // Waits for the response to a send text msg cmd
    SEND_READY_CMD,               // Waits for the rsp after a send sbd cmd
    SEND_DATA,                    // Waits for the rsp after binary data is sent
    SEND_INITIATE_TRANSFER_CMD,   // Initiates the modem to send the buffer to the gateway
    SEND_CLEAR_BUF_CMD,           // Clears the modem's send buffer
    SEND_STATUS_CMD,              // Waits for the sbd buffer status rsp
    SEND_CREG_CMD,                // Waits for the network registration rsp
    SEND_CSQ_CMD,                 // gets the signal quality of the link
    SEND_MAILBOX_CHECK_CMD,       // Clears the MO buffer and sends an initiate transfer cmd.
    GET_MAILBOX_CHECK_RSP,        // Looks again for response (0/4) and sends an initiate transfer cmd.
    SEND_MODEM_STATE_CMD,         // Checks the call status of the modem.
    SEND_HANGUP_CALL_CMD,         // Hangs up the current call
    SEND_MODEM_VER_CMD,           // Gets the modem software version.
    HANDLE_FINAL_RSP,             // Get the stray 0 or 4 from the modem after a command.
    SEND_CIS_PORT_CMD,            // Provides enough time to switch between CIS programming/data port
    SEND_CIS_RINGER_STATE_CMD,    // Checks the status of the ringer
    SEND_CIS_RELAY_STATE_CMD,     // Checks the status of relay 1 or 2 (depending on currrelaynbr val)
    SEND_CIS_DOWNLOAD_CONFIG_CMD, // Sends the 'download config' command for when the local AFIRS config is not valid.
    CIS_DOWNLOAD_CONFIG,          // Receives the data from the CIS.
    SEND_CIS_VERSION_QUERY_CMD,   // Sends the '~' to verify configuration before proceeding.
    START_CIS_PGMING_CMD,         // Starts the programming procedure to the CIS.
    CIS_PGMING_CMD,               // Continues to send the CIS configuration, line-by-line
    CIS_PGMING_RSP,               // Analyzes the response from the CIS and reacts accordingly.
    GET_DATA                      // Transfers the file size + actual SBD message + CRC
};


// This is the AT command set.  Add commands as necessary, but
// be sure to keep the order of the string command in AT_CMDS
typedef BYTE        AT_CMD_LIST;


enum at_cmd_list
{
    AT_CMD_SBD_ALERT,
    AT_CMD_SBD_AUTO_REG,
    AT_CMD_NETWORK_REG,
    AT_CMD_SIGNAL_STRENGTH,
    AT_CMD_SERIAL_NBR,
    AT_CMD_SBD_WRITE_TEXT,
    AT_CMD_SBD_WRITE_BIN,
    AT_CMD_SBD_READ_BIN,
    AT_CMD_SBD_CURRENT_CALL_STATUS,
    AT_CMD_SBD_CLEAR_MO_BUFF,
    AT_CMD_REVISION, // expects 145 bytes in its response
    AT_CMD_HANGUP,
    AT_CMD_SBD_STATUS,
    AT_CMD_SBD_INITIATE_SESSION, // satellite commands from here down.
    AT_CMD_SBD_INITIATE_ALERT_SESSION,
    AT_CMD_NBR_CODE               // MUST BE LAST ENUM
};


// String commands as enumerated in AT_CMD_LIST
static BYTE AT_CMDS[AT_CMD_NBR_CODE][MAX_CMD_LINE_LEN] = 
{
    "AT+SBDMTA=0\r",
    "AT+SBDAREG=1\r",
    "AT+CREG?\r",   
    "AT+CSQF\r",    
    "AT+CGSN\r",    
    "AT+SBDWT=",    
    "AT+SBDWB=",    
    "AT+SBDRB\r",   
    "AT+CLCC\r",    
    "AT+SBDD0\r",   
    "AT+CGMR\r",    
    "AT+CHUP\r",
    "AT+SBDSX\r",// not exactly sat - piggy-backs off +SBDIX
    "AT+SBDIX\r\n",// sat
    "AT+SBDIXA\r\n",// sat
};

// Length of each of the above commands, calc'd at init time.
static BYTE AT_CMDLEN[AT_CMD_NBR_CODE];



// This is a list of valid responses for each AT command.
// Add as necessary, but be sure to keep the order of the 
// string command in AT_RSPS
typedef BYTE    AT_RSP_LIST;
enum at_rsp_list
{
    AT_RSP_SBD_WRITE_BIN_READY,
    AT_RSP_SBD_READ_TEXT_OK,
    AT_RSP_SBD_READ_BIN_OK,
    AT_RSP_SBD_INITIATE_SESSION,
    AT_RSP_SBD_STATUS,
    AT_RSP_SBD_XFER_BUFF_MO_TO_MT_OK,
    AT_RSP_CSQ,
    AT_RSP_CALL_STATUS,
    AT_RSP_CREG,
    AT_RSP_SBDREG,
    AT_RSP_REVISION,
    AT_RSP_NBR_CODE
};

// String commands as enumerated in AT_RSP_LIST;
static BYTE AT_RSPS[AT_RSP_NBR_CODE][MAX_CMD_LINE_LEN] = 
{
    "READY\r",
    "+SBDRT:\r\n",                // text is followed by "0\r"
    "0\r",
    "+SBDIX:",                     // fields followed by "OK\r\n"
    "+SBDSX:",                     // fields followed by "0\r"
    "SBMT: Outbound SBM Copied to Inbound SBM: Osize = 5, Isize = 5\r\n0\r",
    "+CSQF:",
    "+CLCC:",
    "+CREG:",
    "+SBDREG:",
    "Call Processor Version: "
};


// This is the CIS command set.  Add commands as necessary, but
// be sure to keep the order of the string command in CIS_CMDS
typedef BYTE    CIS_CMD_LIST;
enum cis_cmd_list
{
    CIS_CMD_RELAY_1_OFF,
    CIS_CMD_RELAY_1_ON,
    CIS_CMD_RELAY_1_STATUS,
    CIS_CMD_RELAY_2_OFF,
    CIS_CMD_RELAY_2_ON,
    CIS_CMD_RELAY_2_STATUS,
    CIS_CMD_RINGER_OFF,
    CIS_CMD_RINGER_ON,
    CIS_CMD_RINGER_STATUS,
    CIS_CMD_RESET,
    CIS_CMD_DOWNLOAD_CONFIG,
    CIS_CMD_VERSION_CHECK,
    CIS_CMD_LOAD_FLASH,
    CIS_CMD_CANCEL_LOAD_FLASH,
    CIS_CMD_F1,
    CIS_CMD_F4,
    CIS_CMD_NBR_CODE            // Must be last enum
};


// String commands as enumerated in AT_CMD_LIST
static BYTE CIS_CMDS[CIS_CMD_NBR_CODE][MAX_CMD_LINE_LEN] = 
{
    "set relay 0 0",  // relays are zero based, while hardware labeling is 1 based!
    "set relay 0 1",
    "set relay 0\r",
    "set relay 1 0",
    "set relay 1 1",
    "set relay 1\r",
    "set ringer 1",
    "set ringer 0",   // reverse polarity with ringer 0 = ON
    "set ringer\r",
    "reset",
    "download config\r\n",
    "~",
    "reload flash",
    "c\r",
    {0x1B, 0x4F, 0x50}, // F1, CIS style.
    {0x1B, 0x4F, 0x53} // F4, CIS style.
};


// Length of each of the above commands, calc'd at init time.
static BYTE CIS_CMDLEN[CIS_CMD_NBR_CODE];


// This is a list of valid responses for each CIS command.
// Add as necessary, but be sure to keep the order of the 
// string command in CIS_RSPS
typedef BYTE    CIS_RSP_LIST;
enum cis_rsp_list
{
    CIS_RSP_RINGER_OFF,
    CIS_RSP_RINGER_ON,
    CIS_RSP_RELAY_1_OFF,
    CIS_RSP_RELAY_1_ON,
    CIS_RSP_RELAY_2_OFF,
    CIS_RSP_RELAY_2_ON,
    CIS_RSP_VERSION_CHECK,
    CIS_RSP_NBR_CODE
};

// String commands as enumerated in AT_RSP_LIST;
static BYTE CIS_RSPS[CIS_RSP_NBR_CODE][MAX_CMD_LINE_LEN] = 
{
    "Ringer(s) Off",
    "Ringer(s) On",
    "Relay[0] Off", // relays are zero based in the response as well
    "Relay[0] On",
    "Relay[1] Off",
    "Relay[1] On",
    "20400000 1B010000"
};

static BYTE CIS_RSPLEN[CIS_CMD_NBR_CODE];


typedef struct
{
    BYTE   byMOStatus;
    MAILBOXCHECK_RSP byMTStatus;
    char   szMOMSN[STR_SIZE];
    char   szMTMSN[STR_SIZE];
    WORD   wMTLength;
    BYTE   byMTQueueNbr;
    BYTE   byRAFlag;
    DWORD  dwTxMsgLen;               // Size of current file to send
    short  iSignalStrength;
    CIS_CMD_LIST cmdEnum;
    CALL_STATUS_RSP byCallStatus;
    BOOL   bRingersOn;
    BOOL   bRelayOn[NBR_RELAYS];
    BYTE   byCurrRelayNbr;

} MODEM_INFO_STRUCT;

typedef struct
{
    WORD  wMsgCheckSum;
    WORD  wMTType;
    BYTE  byBuffer[MAX_RX_FILE_LEN];
} MT_MSG_FORMAT;

typedef struct
{
    WORD  wRxMsgLen;
    MT_MSG_FORMAT MTMessage;
} MODEM_RX_STRUCT;


typedef union
{
    MODEM_RX_STRUCT rxMsg;
    BYTE  pbyRxMsg[MAX_RX_SIZE];
} U_MODEM_RX_STRUCT;


// Generic responses
#define     AT_RSP_OK                               '0'
#define     AT_RSP_ERROR                            '4'
#define     AT_RSP_ERROR_STR                        "4"
#define     AT_RSP_HW_ERROR                         127  // unsolicited result code! - other AT commands will return 4 - the RF is turned off!

// AT command specific
#define     AT_RSP_SBD_CLEAR_FAIL                   '1'  // Clear buffer/MOMSN rsp
#define     AT_RSP_SBD_WRITE_BIN_TIMEOUT            '1'  // Happens after 60 seconds - Iridium local timer
#define     AT_RSP_SBD_WRITE_BIN_BAD_CHECKSUM       '2'
#define     AT_RSP_SBD_WRITE_BIN_BAD_SIZE           '3'

#define     AT_RSP_SBDI_SUCCESS                     0
#define     AT_RSP_SBDI_SUCCESS_TRUNC               1   // MT message was too big
#define     AT_RSP_SBDI_SUCCESS_NO_LU               2   // No location update performed
#define     AT_RSP_SBDI_SUCCESS_RFU_1               3
#define     AT_RSP_SBDI_SUCCESS_RFU_2               4
#define     AT_RSP_SBDI_FAILURE_RFU_1               5
#define     AT_RSP_SBDI_FAILURE_RFU_2               6
#define     AT_RSP_SBDI_FAILURE_RFU_3               7
#define     AT_RSP_SBDI_FAILURE_RFU_4               8
#define     AT_RSP_SBDI_FAILURE_RFU_5               9   // does not exist in documentation
#define     AT_RSP_SBDI_FAILURE_TIMEOUT             10
#define     AT_RSP_SBDI_FAILURE_Q_FULL              11  // The queue at the GSS is full
#define     AT_RSP_SBDI_FAILURE_MO_ERROR            12  // ok this one says "MO message has too many segments" ???
#define     AT_RSP_SBDI_FAILURE_INCOMPLETE_SESSION  13
#define     AT_RSP_SBDI_FAILURE_INVALID_SIZE        14  // "Invalid segment size" ?
#define     AT_RSP_SBDI_FAILURE_ACCESS_DENIED       15
#define     AT_RSP_SBDI_FAILURE_SBD_BLOCKED         16  // THIS SHOULD TRIGGER A HARDWARE ERROR
#define     AT_RSP_SBDI_FAILURE_NO_RSP              17  // No response from gateway - local session timed out
#define     AT_RSP_SBDI_FAILURE_NO_CONNECTION       18  // RF drop
#define     AT_RSP_SBDI_FAILURE_NO_LINK             19  // "protocol error cause termination of the call"
#define     AT_RSP_SBDI_FAILURE_RFU_6               20
#define     AT_RSP_SBDI_FAILURE_RFU_7               21
#define     AT_RSP_SBDI_FAILURE_RFU_8               22
#define     AT_RSP_SBDI_FAILURE_RFU_9               23
#define     AT_RSP_SBDI_FAILURE_RFU_10              24
#define     AT_RSP_SBDI_FAILURE_RFU_11              25
#define     AT_RSP_SBDI_FAILURE_RFU_12              26
#define     AT_RSP_SBDI_FAILURE_RFU_13              27
#define     AT_RSP_SBDI_FAILURE_RFU_14              28
#define     AT_RSP_SBDI_FAILURE_RFU_15              29
#define     AT_RSP_SBDI_FAILURE_RFU_16              30
#define     AT_RSP_SBDI_FAILURE_RFU_17              31
#define     AT_RSP_SBDI_FAILURE_NO_NETWORK          32
#define     AT_RSP_SBDI_FAILURE_RFU_18              33
#define     AT_RSP_SBDI_FAILURE_RFU_19              34
#define     AT_RSP_SBDI_FAILURE_BUSY                35
#define     AT_RSP_SBDI_FAILURE_RFU_20              36

#define     AT_RSP_SBD_STATUS_RA                    1

#define     AT_RSP_CREG_NOT_REG                     0
#define     AT_RSP_CREG_REG_HOME                    1
#define     AT_RSP_CREG_SEARCHING                   2
#define     AT_RSP_CREG_DENIED                      3
#define     AT_RSP_CREG_UNKNOWN                     4
#define     AT_RSP_CREG_REG_ROAMING                 5

#define     AT_RSP_CSQ_LEVEL_0                      0
#define     AT_RSP_CSQ_LEVEL_1                      1
#define     AT_RSP_CSQ_LEVEL_2                      2
#define     AT_RSP_CSQ_LEVEL_3                      3
#define     AT_RSP_CSQ_LEVEL_4                      4
#define     AT_RSP_CSQ_LEVEL_5                      5
                                                    
#define     PROG_CIS_MANUFACTURER_ERROR             'M' // No recovery
#define     PROG_CIS_OUT_OF_FLASH_ERROR             'O'
#define     PROG_CIS_PAGE_ERASE_ERROR               'E'
#define     PROG_CIS_PROG_PAGE_ERROR                'e'
#define     PROG_CIS_BAD_FORMAT_ERROR               'F'
#define     PROG_CIS_BAD_HW_ID_ERROR                'H'
#define     PROG_CIS_BAD_CHECKSUM_ERROR             'N'
#define     PROG_CIS_BAD_CHAR_RXD_ERROR             'n'
#define     PROG_CIS_BLOCK_PASSED                   'a'
#define     PROG_CIS_UPLOAD_SUCCESSFUL              'C'
                                                    
typedef BYTE    MTMDIR_RETURN_TYPE;
enum mtmdir_return_type
{                                                   
    BUFFER_ONLY,                                    
    COPY_PORT3,                                     
    SAVE_TO_FILE                                    
};       
                                                    

//------------------------------------------------------------------------------
//  GLOBAL DECLARATIONS
//------------------------------------------------------------------------------


static  AT_CMD_STATES        ATCmdState;    // Holds the current state
static  SUB_STATES           subState;      // Substate holder
static  MODEM_ERROR_CODE_RSP errorCodeRsp;  // Send error code holder

// Buffer must be wide enough to hold any and all responses from the modem.
// It will not be used as a circular buffer, as string operations are necessary
// in order to parse the responses. On overflow however, the buffer index wraps.
static  BYTE                byRxBuffer[MAX_CMD_LINE_LEN];
static  WORD                wRxIndex;
static  BOOL                bPrevVoiceState;

static  TIMERHANDLE         thRespTimeOut;
static  TIMERHANDLE         thCISRespTimeOut;

// vars used for rx'ing buffers.
static  U_MODEM_RX_STRUCT   RxMsg;
static  WORD                wCalculatedCheckSum = 0;
static  char                szRxFilename[MAX_FILENAME_LEN];
static  char                szRxPathFilename[EMAXPATH];

static  MODEM_INFO_STRUCT   modemInfo;
static  BYTE                byBinMsgBuffer[MAX_FILE_LEN]; // Buffer to hold the incomming binary message.

static  char                szIMEI[IMEI_SIZE]; 
static  char                szModemSWVersion[MODEM_SW_VER_SIZE];
static  BOOL                bHaveIMEI;

static  WORD                wSatelliteTimeout;
static char                 szErrString[MAX_SYSTEM_LOG_STR];

#ifdef __BORLANDC__
AnsiString                  sErrorStr;
static  COMMINFO_TYPE       ciModemCommPort;
#endif

//------------------------------------------------------------------------------
//  PRIVATE FUNCTION PROTOTYPES
//------------------------------------------------------------------------------


// Command helper functions
static void                 SendCommand( AT_CMD_LIST cmd );
static void                 SendWriteBinaryMsgCmd( void );
static void                 SendBinaryDataBuffer( void );
static BOOL                 SendCISPortCmd( void );
static BOOL                 SendCISLoadConfigLineCmd( void );
static void                 RecoverFromBadCISCmd( void );


// Response helper functions
static BOOL                 GetResponseBuffer( BYTE EOL );
static MODEM_RESPONSES      GetLastRsp( void );
static MODEM_RESPONSES      GetWriteBinaryMsgRsp( void );
static MODEM_RESPONSES      GetInitiateSBDSessionRsp( void );
static MODEM_RESPONSES      GetReqCurrCallStatusRsp( void );
static MODEM_RESPONSES      GetSBDStatusRsp( void );
static MODEM_RESPONSES      GetCREGRsp( void );
static MODEM_RESPONSES      GetCSQRsp( void );
static MODEM_RESPONSES      GetIMEIRsp( void );
static MODEM_RESPONSES      GetModemVerRsp( void );
static MODEM_RESPONSES      GetRxBinaryDataBufferRsp( void );
static BOOL                 GetDualResponse( BYTE FirstEOL, BYTE SecondEOL );
static MODEM_RESPONSES      GetCISPortRsp( void );
static MODEM_RESPONSES      GetRingerStatusRsp( void );
static MODEM_RESPONSES      GetRelayStatusRsp( void );
static BOOL                 CaptureCISOutput( void );
static MODEM_RESPONSES      GetCISVersionStatusRsp( void );

static MTMDIR_RETURN_TYPE   DefineMsgTypeDestPath( WORD* pwMsg, DEVICE_DIR* pDeviceDir, SUBDIR_NAME* pSubDir );
static void                 ClearBuffers( CIS_PORT portState );
static void                 ClearModemInfo( void );
static void                 ClearRxBinaryDataVars( void );


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: InitModem
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Initializes all local states/variables.  Hardware 
//               (lower layer) must be initialized before this
//               is called.  Upper API layer can then be called
//               after this.
//
//******************************************************************************
void InitModem( void )
{
    WORD wCmdIndex;
                       
    // Calculate the length of each command string. These don't change
    // through the course of execution.
    for( wCmdIndex = 0; wCmdIndex < AT_CMD_NBR_CODE; wCmdIndex++ )
    {
        AT_CMDLEN[wCmdIndex] = (BYTE)StringLen( (char*)AT_CMDS[wCmdIndex] );
    }

    for( wCmdIndex = 0; wCmdIndex < CIS_CMD_NBR_CODE; wCmdIndex++ )
    {
        CIS_CMDLEN[wCmdIndex] = (BYTE)StringLen( (char*)CIS_CMDS[wCmdIndex] );
    }

    for( wCmdIndex = 0; wCmdIndex < CIS_RSP_NBR_CODE; wCmdIndex++ )
    {
        CIS_RSPLEN[wCmdIndex] = (BYTE)StringLen( (char*)CIS_RSPS[wCmdIndex] );
    }

    // State init.
    subState        = SUBSTATE_NONE;
    errorCodeRsp    = MEC_NONE;
    bPrevVoiceState = FALSE;
    modemInfo.bRingersOn        = TRUE; // by default, the ringers are on.
    modemInfo.bRelayOn[RELAY_1] = NO_FAULT; // trick the firmware, so it will trigger a relay-1-on condition
    modemInfo.bRelayOn[RELAY_2] = FALSE; // by default, relay 2 is off

    // Initialize all local variables.
    ClearBuffers( DATA_PORT );
    ClearModemInfo();
    ClearRxBinaryDataVars();
    MemSet( szModemSWVersion, 0, MODEM_SW_VER_SIZE );
    StringCpy( szIMEI, ERROR_IMEI );
    bHaveIMEI = FALSE;
    wSatelliteTimeout = SATELLITE_RSP_TIMEOUT;

    // Initialize command response timer
    thRespTimeOut    = RegisterTimer();
    thCISRespTimeOut = RegisterTimer();

    // Always set initial state to powered down. State machine will bring
    // us out when the modem power is good.
    ATCmdState    = AT_CMD_POWERED_DOWN;
}


#ifdef __BORLANDC__
BOOL ConnectToModem( void )
{
    AnsiString sPort = "1";

    if( InputQuery( "Select Comm Port", "Enter device comm port: ", sPort ) )
    {
        if( sPort.ToIntDef( -1 ) < 1 )
        {
            MessageDlg( "You have entered an invalid comm port number!", mtError, TMsgDlgButtons() << mbOK, 0 );
            return FALSE;
        }

        InitCommPort( &ciModemCommPort );
        AnsiString sBitRate = IntToStr( MODEM_PORT_BPS );

        if( InputQuery( "Select Port Bit Rate", "Enter Bit Rate: ", sBitRate ) )
        {
            try
            {
                COMM_RESULT commResult = ConnectPort( &ciModemCommPort, sPort.ToInt(), sBitRate.ToInt() );

                if( commResult != ERR_NONE )
                {
                    MessageDlg( "Could not open port " + sPort + "!", mtError, TMsgDlgButtons() << mbOK, 0 );
                    return FALSE;
                }
            }
            catch( ... )
            {
                MessageDlg( "You have entered an invalid port number!", mtError, TMsgDlgButtons() << mbOK, 0 );
                return FALSE;
            }
        }
        else
        {
            return FALSE;
        }
    }
    else
    {
        return FALSE;
    }

    return TRUE;
}


void DisconnectModem( void )
{
    Disconnect( &ciModemCommPort );
    ATCmdState = AT_CMD_POWERED_DOWN;
}


void SetIMEI( AnsiString ModemSN )
{
    StringNCpy( szIMEI, ModemSN.c_str(), IMEI_SIZE );
}


void SendTransparentStr( AnsiString sBuffer )
{
    CommSend( &ciModemCommPort, sBuffer.c_str(), sBuffer.Length() );
}


DWORD ReceiveTransparentStr( BYTE* pbyStr, DWORD dwMaxBytes )
{
    return CommRecv( &ciModemCommPort, pbyStr, dwMaxBytes );
}

void ModemPortSendBuffer( BYTE* pBuffer, WORD wLength )
{
    CommSend( &ciModemCommPort, pBuffer, wLength );
}

BOOL GetModemPortChar( BYTE* byData )
{
    CommRecv( &ciModemCommPort, byData, 1 );
}

BOOL IsModemRunning( void )
{
    return IsPortConnected( &ciModemCommPort );
}
#endif


//******************************************************************************
//
//  Function: ResetModem
//
//  Arguments: void.
//
//  Returns: TRUE if modem was successfully power cycled.
//           FALSE if the modem was unsuccessfully power cycled.
//
//  Description: This power cycles the modem at the power manager level.
//               Since we are using profiles, profile 0 will be loaded
//               automatically after power cycle is complete.
//
//******************************************************************************
BOOL ResetModem( void )
{
    if( !InVoiceCall() )
    {
        // Now power cycle the modem.
        if( PowerCycleModem() )
        {
            StopTimer( thRespTimeOut );

            // Initialize all local variables.
            ClearBuffers( DATA_PORT );
            ClearModemInfo();

            // Alwasy set initial state to powered down. State machine will bring
            // us out when the modem power is good.
            ATCmdState    = AT_CMD_POWERED_DOWN;

            // State init.
            subState      = SUBSTATE_NONE;
            errorCodeRsp  = MEC_NONE;

            return TRUE;
        }
    }

    // Could not power cycle the modem - don't change state
    return FALSE;
}


//******************************************************************************
//
//  Function: SendWriteTextMsgCmd
//
//  Arguments:
//    szDataBuf - pointer to text message sent by user
//
//  Returns: TRUE if we're idle and can send the msg, 
//           FALSE when not in AT_CMD_IDLE
//
//  Description: This function sends the AT command to send an SBD text
//               message. It will only send the message if we're in 
//               idle mode. This function requires that the data buffer
//               not exceed MAX_CMD_LINE_LEN characters. It also
//               assumes only ascii characters are sent, and that there
//               are no CARRIAGE_RETURN <CR> or LINE_FEED <LF> characters included
//               within the stream.
//
//******************************************************************************
BOOL SendWriteTextMsgCmd( const char *szDataBuf )
{
    static char szTextMsg[MAX_CMD_LINE_LEN];

    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    StringCpy( szTextMsg, (char*)AT_CMDS[AT_CMD_SBD_WRITE_TEXT] );

    // Trucate the message in case it exceeds max text length.
    // Be sure to leave room for the carriage return.
    StringNCat( szTextMsg, szDataBuf, MAX_CMD_LINE_LEN-1 );

    // Message is terminated when a carriage return is entered.
    StringCat( szTextMsg, "\r" );

    // Clear the receive buffer before we send a command.
    ClearBuffers( DATA_PORT );

    // Output text message to modem
    ModemPortSendBuffer( (BYTE*)szTextMsg, StringLen( szTextMsg ) );

    ATCmdState = AT_CMD_SENDING;
    subState = SEND_TEXT_MSG;

    StartTimer( thRespTimeOut, STANDARD_RSP_TIMEOUT );

    return TRUE;
}


//******************************************************************************
//
//  Function: SendBinaryFile
//
//  Arguments:
//    szPathfileName - string value of the file name
//
//  Returns: TRUE if we are in idle state, we could open the file 
//           successfully, and the size of the file is acceptable.
//           FALSE if we are not in AT_CMD_IDLE state, or the file is
//           corrupted or open by another process, or we have exceeded
//           the modem's max buffer size.
//
//  Description: There are no restrictions as to the contents of the 
//               file, only that it does not exceed the modem's maximum
//               buffer size.  We must also be in AT_CMD_IDLE state to 
//               execute this cmd.
//
//******************************************************************************
BOOL SendBinaryFile( char *szPathfileName )
{
    PCFD   fd;

    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    modemInfo.dwTxMsgLen = FileLength( szPathfileName );

    // make sure the file length does not exceed the max size. 
    // Trucate the message if it exceeds max length
    if( modemInfo.dwTxMsgLen > MAX_FILE_LEN )
    {
        modemInfo.dwTxMsgLen = MAX_FILE_LEN;
        errorCodeRsp = MEC_TRUNCATED_FILE;
    }
    else if( modemInfo.dwTxMsgLen <= 0 )
    {
        errorCodeRsp = MEC_TX_BIN_DATA_BAD_SIZE;
        return FALSE;
    }

    // Clear the buffer before using it.
    MemSet( byBinMsgBuffer, 0, MAX_FILE_LEN );

    // Try to open the file
    // FOR RULESIM ONLY - PCMCIA driver ignores this tag.
    fd = fileOpen( szPathfileName, PO_RDONLY | PO_TEXT, PS_IREAD | PS_IWRITE );

    if( fd == -1 )
    {
        // Could not open report file for some reason!
        errorCodeRsp = MEC_FILE_OPEN_ERR;
        StringCpy( szErrString, szPathfileName );
        StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_OPENED_OR_CREATED ), MAX_SYSTEM_LOG_STR );
        SystemLog( szErrString );
        return FALSE;
    }

    // Copy rpt file into a buffer
    if( fileRead( fd, byBinMsgBuffer, (WORD)modemInfo.dwTxMsgLen ) != (WORD)modemInfo.dwTxMsgLen )
    {
        errorCodeRsp = MEC_FILE_READ_ERR;
        StringCpy( szErrString, szPathfileName );
        StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_READ ), MAX_SYSTEM_LOG_STR );
        SystemLog( szErrString );
        
        fileClose( fd );
        return FALSE;
    }

    fileClose( fd );

    SendWriteBinaryMsgCmd();

    ATCmdState = AT_CMD_SENDING;
    subState = SEND_READY_CMD;

    StartTimer( thRespTimeOut, STANDARD_RSP_TIMEOUT );

    return TRUE;
}


//******************************************************************************
//
//  Function: SendBinaryBuffer
//
//  Arguments:
//    pbyDataBuf - pointer to binary message being sent.
//
//  Returns: TRUE if we're idle and can send the msg, 
//           FALSE when not in AT_CMD_IDLE
//
//  Description: There are no restrictions as to the contents of the 
//               buffer, only that it does not exceed the modem's maximum
//               buffer size.  We must also be in AT_CMD_IDLE state to 
//               execute this cmd.
//
//******************************************************************************
BOOL SendBinaryBuffer( const BYTE *pbyDataBuf, WORD wMsgSize )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    modemInfo.dwTxMsgLen = wMsgSize;

    // make sure the file length does not exceed the max size. 
    // Trucate the message if it exceeds max length
    if( modemInfo.dwTxMsgLen > MAX_FILE_LEN )
    {
        modemInfo.dwTxMsgLen = MAX_FILE_LEN;
        errorCodeRsp = MEC_TRUNCATED_FILE;
    }
    else if( modemInfo.dwTxMsgLen <= 0 )
    {
        errorCodeRsp = MEC_TX_BIN_DATA_BAD_SIZE;
        return FALSE;
    }

    // Clear the buffer before copying the actual data in.
    MemSet( byBinMsgBuffer, 0, MAX_FILE_LEN );
    MemCpy( byBinMsgBuffer, pbyDataBuf, (WORD)modemInfo.dwTxMsgLen );

    SendWriteBinaryMsgCmd();

    ATCmdState = AT_CMD_SENDING;
    subState = SEND_READY_CMD;

    StartTimer( thRespTimeOut, STANDARD_RSP_TIMEOUT );

    return TRUE;
}



//******************************************************************************
//
//  Function: CheckGateway
//
//  Arguments: void
//
//  Returns: TRUE if we're idle and can send the cmd,
//           FALSE if ATCmdState not in idle mode, or buffer was not
//                 cleared successfully.
//
//  Description: Perform a Gateway Check by sending +SBDSX. If an MT message 
//               is waiting for the ISU at the ESS, the ring alert field
//               will be TRUE and we then need to perform a mailbox check
//               to download it.
//
//******************************************************************************
BOOL CheckGateway( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    SendCommand( AT_CMD_SBD_STATUS );
                                                                    
    ATCmdState = AT_CMD_SENDING;                                    
    subState = SEND_STATUS_CMD;

    return TRUE;
}


//******************************************************************************
//
//  Function: CheckMailbox
//
//  Arguments: void
//
//  Returns: TRUE if we're idle and can send the cmd,
//           FALSE if ATCmdState not in idle mode, or buffer was not
//                 cleared successfully.
//
//  Description: Perform a Mailbox Check by initiating an SBD session with
//               an empty MO buffer. If an MT message is waiting for the
//               ISU at the ESS, the MT message is transmitted to the ISU.
//
//******************************************************************************
BOOL CheckMailbox( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    // Sets timer in command.
    SendCommand( AT_CMD_SBD_CLEAR_MO_BUFF );

    // Change substate and continue with SBDI command
    ATCmdState = AT_CMD_SENDING;
    subState = SEND_MAILBOX_CHECK_CMD;

    return TRUE;
}


//******************************************************************************
//
//  Function: SendCSQCmd
//
//  Arguments: void.
//
//  Returns: TRUE  if we're idle and can send the cmd. 
//           FALSE when not in AT_CMD_IDLE.
//
//  Description: Queries the modem for its current signal strength.
//
//******************************************************************************
BOOL SendCSQCmd( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    SendCommand( AT_CMD_SIGNAL_STRENGTH );

    ATCmdState = AT_CMD_SENDING;
    subState = SEND_CSQ_CMD;

    return TRUE;
}


//******************************************************************************
//
//  Function: SendReadBinaryFileCmd
//
//  Arguments: void.
//
//  Returns: TRUE if we are in idle state.
//           FALSE if we are not in AT_CMD_IDLE state.
//
//  Description: This command is sent to retrieve a message from 
//               the modem buffer (already received from the gateway, stored
//               locally onboard the modem's buffer).
//
//******************************************************************************
BOOL SendReadBinaryFileCmd( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    SendCommand( AT_CMD_SBD_READ_BIN );
    
    ClearRxBinaryDataVars();
    
    ATCmdState = AT_CMD_RCVING;
    subState = GET_DATA;

    return TRUE;
}


//******************************************************************************
//
//  Function: SendCLCCCmd
//
//  Arguments: void
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Queries the modem for its current call status.
//               Should only be called if upper layer has been turned off.
//
//******************************************************************************
BOOL SendCLCCCmd( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    SendCommand( AT_CMD_SBD_CURRENT_CALL_STATUS );

    // Reset the call status...
    modemInfo.byCallStatus = AT_RSP_INVALID_CALL_STATUS;

    ATCmdState = AT_CMD_SENDING;
    subState = SEND_MODEM_STATE_CMD;

    return TRUE;
}


//******************************************************************************
//
//  Function: SendCallHangupCmd
//
//  Arguments: void
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Sends the hangup command to the modem.
//
//******************************************************************************
BOOL SendCallHangupCmd( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    SendCommand( AT_CMD_HANGUP );

    ATCmdState = AT_CMD_SENDING;
    subState = SEND_HANGUP_CALL_CMD;

    return TRUE;
}


//******************************************************************************
//
//  Function: SendCREGCmd
//
//  Arguments: void
//
//  Returns: TRUE  if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Queries the modem if it is registered on to the network.
//
//******************************************************************************
BOOL SendCREGCmd( void )
{
    // Ensure the modem is not currently busy first
    if( ATCmdState != AT_CMD_IDLE )
    {
        return FALSE;
    }

    SendCommand( AT_CMD_NETWORK_REG );
           
    ATCmdState = AT_CMD_SENDING;
    subState = SEND_CREG_CMD;
    return TRUE;
}


//******************************************************************************
//
//  Function: SendDownloadCISCmd
//
//  Arguments: void.
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Instructs the CIS board to download the current configuration.
//
//******************************************************************************
BOOL SendDownloadCISCmd( void )
{
    BOOL bRet_val = TRUE;

    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            print("\r\nat cmd state: " );
            print( GetATCmdText( ATCmdState ) );
            return FALSE;
    }

    print( " starting download config" );
    modemInfo.cmdEnum = CIS_CMD_DOWNLOAD_CONFIG;
    bRet_val = SendCISPortCmd();
    subState = SEND_CIS_DOWNLOAD_CONFIG_CMD;

    return bRet_val;
}


//******************************************************************************
//
//  Function: SendProgramCISCmd
//
//  Arguments: void.
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Begins the CIS configuration process.
//
//******************************************************************************
BOOL SendProgramCISCmd( void )
{
    BOOL bRet_val = TRUE;

    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            print("\r\nat cmd state: " );
            print( GetATCmdText( ATCmdState ) );
            return FALSE;
    }

    modemInfo.cmdEnum = CIS_CMD_VERSION_CHECK;
    bRet_val = SendCISPortCmd();
    subState = SEND_CIS_VERSION_QUERY_CMD;

    // Don't clear the current (cached) state of the ringer until
    // we have a valid response.
    return bRet_val;
}


//******************************************************************************
//
//  Function: SendCISResetCmd
//
//  Arguments: void.
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Instructs the CIS board to reset itself.
//
//******************************************************************************
BOOL SendCISResetCmd( void )
{
    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            return FALSE;
    }

    print( " sending reset cmd to CIS:" );
    modemInfo.cmdEnum = CIS_CMD_RESET;

    return SendCISPortCmd();
}


//******************************************************************************
//
//  Function: SendSetRingerCmd
//
//  Arguments:
//    IN bRingerState - TRUE to turn ringer on
//                      FALSE to turn ringer off
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Instructs the CIS board to turn on/off ringers.
//
//******************************************************************************
BOOL SendSetRingerCmd( BOOL bRingerState )
{
    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            print("\r\nat cmd state: " );
            print( GetATCmdText( ATCmdState ) );
            return FALSE;
    }

    if( bRingerState )
    {
        // ringer is on
        modemInfo.cmdEnum = CIS_CMD_RINGER_ON;
        modemInfo.bRingersOn = TRUE;
    }
    else
    {
        // ringer is off
        modemInfo.cmdEnum = CIS_CMD_RINGER_OFF;
        modemInfo.bRingersOn = FALSE;
    }

    // Cache value for quick retrieval
    return SendCISPortCmd();
}


//******************************************************************************
//
//  Function: SendGetRingerStatusCmd
//
//  Arguments: void.
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Queries the CIS for its current ringer status.
//               Returns the value and stores it locally, for retrival from
//               command GetRingerStatus().
//
//******************************************************************************
BOOL SendGetRingerStatusCmd( void )
{
    BOOL bRet_val = TRUE;

    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            print("\r\nat cmd state: " );
            print( GetATCmdText( ATCmdState ) );
            return FALSE;
    }

//    print( "\r\nTx: " );
//    print( CIS_CMDS[CIS_CMD_RINGER_STATUS] );
    modemInfo.cmdEnum = CIS_CMD_RINGER_STATUS;
    bRet_val = SendCISPortCmd();
    subState = SEND_CIS_RINGER_STATE_CMD;

    // Don't clear the current (cached) state of the ringer until
    // we have a valid response.
    return bRet_val;
}


//******************************************************************************
//
//  Function: SendSetRelayCmd
//
//  Arguments:
//    IN bRelayState - TRUE to open relay
//                     FALSE to close relay
//    IN byRelayNbr  - Relay number to open (FALSE) or close (TRUE).
//                     Should be RELAY_1 or RELAY_2.
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Instructs the CIS board to turn on/off relay
//               (given by byRelayNbr).
//
//******************************************************************************
BOOL SendSetRelayCmd( BYTE byRelayNbr, BOOL bRelayState )
{
    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            print("\r\nat cmd state: " );
            print( GetATCmdText( ATCmdState ) );
            return FALSE;
    }

    switch( byRelayNbr )
    {
        case RELAY_1:

            if( bRelayState )
            {
                // turn relay 0 on
                modemInfo.cmdEnum = CIS_CMD_RELAY_1_ON;
            }
            else
            {
                // turn relay 0 off
                modemInfo.cmdEnum = CIS_CMD_RELAY_1_OFF;
            }

            break;

        case RELAY_2:

            if( bRelayState )
            {
                // turn relay 1 on
                modemInfo.cmdEnum = CIS_CMD_RELAY_2_ON;
            }
            else
            {
                // turn relay 1 off
                modemInfo.cmdEnum = CIS_CMD_RELAY_2_OFF;
            }

            break;

        default:
            return FALSE;
    }

    // Cache value for quick retrieval
    modemInfo.bRelayOn[byRelayNbr] = bRelayState;

    return SendCISPortCmd();
}


//******************************************************************************
//
//  Function: SendGetRelayStatusCmd
//
//  Arguments:
//    IN byRelayNbr  - Relay number from which to query to the status.
//                     Should be 0 or 1.
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if ATCmdState not in idle mode.
//
//  Description: Queries the CIS for its current relay status (given the
//               relay number). Returns the value and stores it locally,
//               for retrival from command GetRelayStatus( byRelayNbr ).
//
//******************************************************************************
BOOL SendGetRelayStatusCmd( BYTE byRelayNbr )
{
    CIS_CMD_LIST cmdEnum;
    BOOL bRet_val = TRUE;

    // Ensure the modem is not currently busy first
    switch( ATCmdState )
    {
        case AT_CMD_IDLE:
        case AT_CMD_POWERED_DOWN:
            break;

        default:
            print("\r\nat cmd state: " );
            print( GetATCmdText( ATCmdState ) );
            return FALSE;
    }

//    print( "\r\nTx: " );
//    print( CIS_CMDS[cmdEnum] );

    switch( byRelayNbr )
    {
        case RELAY_1:

            // Check relay 1
            cmdEnum  = CIS_CMD_RELAY_1_STATUS;
            modemInfo.byCurrRelayNbr = RELAY_1; 
            break;

        case RELAY_2:

            // Check relay 2
            cmdEnum  = CIS_CMD_RELAY_2_STATUS;
            modemInfo.byCurrRelayNbr = RELAY_2; 
            break;

        default:
            return FALSE;
    }

    modemInfo.cmdEnum = cmdEnum;
    bRet_val = SendCISPortCmd();
    subState = SEND_CIS_RELAY_STATE_CMD;

    // Don't clear the current (cached) state of the relay until
    // we have a valid response.
    return bRet_val;
}


//******************************************************************************
//
//  Function: SetATCmdStateInit
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Allows the upper layer to place the middle layer into init
//               just in case there was an issue after a power cycle.
//               The upper layer should only send the modem into init state
//               if there was a problem resetting the modem. This also clears the
//               modem error code.
//
//******************************************************************************
void SetATCmdStateInit( void )
{
    // Need to send the modem into powered down state, in order to send
    // the IMEI command.
    ATCmdState = AT_CMD_POWERED_DOWN;
    subState = SUBSTATE_NONE;

    // Just in case there is anything residual in the buffer,
    // clear it.
    ClearBuffers( DATA_PORT );
    StopTimer( thRespTimeOut );
    StopTimer( thCISRespTimeOut );
}


//******************************************************************************
//
//  Function: SetATCmdStateIdle
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Allows the upper layer to acknowledge that this middle layer
//               failed or successfully sent a message.  Only the upper layer
//               should send the modem into idle state.  This also clears the
//               modem error code.
//
//******************************************************************************
void SetATCmdStateIdle( void )
{
    ATCmdState = AT_CMD_IDLE;
    subState = SUBSTATE_NONE;

    // Just in case there is anything residual in the buffer,
    // clear it.
    ClearBuffers( DATA_PORT );
    StopTimer( thRespTimeOut );
    StopTimer( thCISRespTimeOut );
}


//******************************************************************************
//
//  Function: SetModemCmdRspTime
//
//  Arguments:
//    IN *pdwTimeoutTime - DWORD array expressing the times for each consecutive
//                         time out wait cycle.
//
//  Returns: TRUE if pdwTimeoutTime is not NULL.
//           FALSE if array is NULL.
//
//  Description: Allows Rules the option to set a range of values the modem
//               driver uses to wait before declaring a response has timed out.
//               Each modem time out event, the next value in the list is
//               used as the timeout time, until a successful response is
//               received from the modem (after which, the index goes back
//               to the first value).
//
//               Once all the values in the list have been exhausted, the
//               final value is retained for each time out reset thereafter.
//
//               The list should only be exhausted in a major failure of the
//               modem, as only consecutive resets will increment the index.
//
//               OBSOLETE IN 200v54+
//
//******************************************************************************
BOOL SetModemCmdRspTime( DWORD* pdwTimeoutTime )
{
    return TRUE;
}


//******************************************************************************
//
//  Function: SetModemCmdRspTimeInSeconds
//
//  Arguments:
//    IN *byTimeoutTime_in_seconds - BYTE value provides time out for satellite
//                                   commands to the modem (in seconds).
//
//  Returns: void.
//
//  Description: Allows Rules the option to set a value the modem
//               driver uses to wait before declaring a response has timed out.
//
//               Default is 65 seconds.
//
//******************************************************************************
BOOL SetModemCmdRspTimeInSeconds( BYTE byTimeoutTime_in_seconds )
{
    wSatelliteTimeout = (WORD)byTimeoutTime_in_seconds * 1000UL;
    return TRUE;
}


//******************************************************************************
//
//  Function: SetFaultLightOn
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This function is only called when a CIS is not powered error
//               is reported by Power Manager.
//
//******************************************************************************
void SetFaultLightOn( void )
{
    modemInfo.bRelayOn[RELAY_1] = REPORT_FAULT; // CIS is powered down, change the value of the flag
}


//******************************************************************************
//
//  Function: ClearModemSignalStrength
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This function is only called when an Iridium Error is reported
//               by the upper layer.
//
//******************************************************************************
void ClearModemSignalStrength( void )
{
    modemInfo.iSignalStrength = -1;
}


//******************************************************************************
//
//  Function: UpdateModemState
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This function manages the modem state machine.  It must be
//               be called periodically in order to analyze responses
//               received from the modem, and execute commands sent by the user
//               or the system.
//
//******************************************************************************
void UpdateModemState( void )
{
    MODEM_RESPONSES CmdResponse;
    BYTE byRxByte;
#if(DEBUG)
    WORD wTempIndex;
#endif

    // If at any time we loose power to the modem, then immediately
    // go into the power down state.
    if( !IsModemRunning() )
    {
        // Modem and CIS are separate - we need to keep the communication
        // going through the CIS board (if possible)
        if( ATCmdState != AT_CMD_PGMING )
        {
            // Clear signal strength - we have lost the signal
            ClearBuffers( DATA_PORT );
            ClearModemInfo();
            ClearRxBinaryDataVars();

            StopTimer( thRespTimeOut );

            // Always set initial state to powered down. State machine will bring
            // us out when the modem power is good.
            ATCmdState   = AT_CMD_POWERED_DOWN;

            // State init.
            subState     = SUBSTATE_NONE;
            errorCodeRsp = MEC_NONE;
        }
    }

    // thRespTimeOut timer is only set when a command is sent.
    // Therefore, if it does expire, it means we have not yet obtained
    // our response. Automatically fail, and give error response code.
    if( TimerExpired( thRespTimeOut ) )
    {
        // Switch the port back to data (just in case)
        SetCISPort( DATA_PORT );

        print( " TIMER EXPIRED - " );
        output_int( subState );
        print( " wRxIndex=" );
        output_int( wRxIndex );
        print( " buff=*" );
        print( byRxBuffer );
        print( "* -->" );
#if(DEBUG)
        for( wTempIndex = 0; wTempIndex < wRxIndex; wTempIndex++ )
        {
            output_hex( byRxBuffer[wTempIndex], 2 );
        }
        print( "<-- " );
#endif
        if( subState == SEND_STATUS_CMD )
        {
            modemInfo.byMTStatus = FAILED_MSG;
        }

        if( ( subState != SEND_STATUS_CMD ) 
            &&
            ( subState != SEND_CSQ_CMD ) )
        {
            // only report timed out if it's not SBDSX or CSQF
            errorCodeRsp = MEC_RSP_TIMED_OUT;
        }

        ATCmdState = AT_CMD_TIMED_OUT;

        StopTimer( thRespTimeOut );
    }

    // If at any time we lose power to the CIS, then immediately
    // go into the power down state.
    if( !CISPowered() )
    {
        // Communications are down, prepare the state machine.
        if( ATCmdState == AT_CMD_PGMING )
        {
            // Clear signal strength - we have lost the signal
            ClearBuffers( DATA_PORT );
            ClearModemInfo();
            ClearRxBinaryDataVars();

            StopTimer( thCISRespTimeOut );

            // Always set initial state to powered down. State machine will bring
            // us out when the modem power is good.
            ATCmdState   = AT_CMD_POWERED_DOWN;

            // State init.
            subState     = SUBSTATE_NONE;
            errorCodeRsp = MEC_NONE;
        }
    }

    if( TimerExpired( thCISRespTimeOut ) )
    {
        print( " CIS TIMER EXPIRED - " );
        output_int( modemInfo.cmdEnum );
        errorCodeRsp = MEC_RSP_TIMED_OUT;
        ATCmdState = AT_CMD_TIMED_OUT;

        // Switch the port back to data (just in case)
        SetCISPort( DATA_PORT );

        StopTimer( thCISRespTimeOut );
    }

    // Update modem states.

    switch( ATCmdState )
    {
        case AT_CMD_POWERED_DOWN:

            // Check to see if the modem has powered up. If so,
            // we can initialize it.
            if( IsModemRunning() )
            {
                if( InVoiceCall() )
                {
                    if( !bPrevVoiceState )
                    {
                        RecordModemLogError( MODEMLOG_PHONE_OFF_HOOK );
                        bPrevVoiceState = TRUE;
                    }

                    // pause initialization.
                    break;
                }
                else if( bPrevVoiceState )
                {
                    // If we were previously off hook - report we're back to normal now.
                    RecordModemLogError( MODEMLOG_PHONE_BACK_ON_HOOK );
                    bPrevVoiceState = FALSE;
                }

                // Get IMEI on startup
                SendCommand( AT_CMD_SERIAL_NBR );

                ATCmdState = AT_CMD_INITTING;
                subState   = SEND_IMEI_CMD;
            }

            break;

        case AT_CMD_INITTING:

            switch( subState )
            {
                case SEND_IMEI_CMD:

                    // Check if we're ready to receive data.
                    CmdResponse = GetIMEIRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            StopTimer( thRespTimeOut );
                            subState = SEND_MT_ALERT_CMD;
                            break;

                        case MR_FAILED:
                            // Let the upper layer handle this.
                            StopTimer( thRespTimeOut );
                            ATCmdState = AT_CMD_FAILED;
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_MT_ALERT_CMD:

                    if( InVoiceCall() )
                    {
                        if( !bPrevVoiceState )
                        {
                            RecordModemLogError( MODEMLOG_PHONE_OFF_HOOK );
                            bPrevVoiceState = TRUE;
                        }

                        // pause initialization.
                        break;
                    }
                    else if( bPrevVoiceState )
                    {
                        // If we were previously off hook - report we're back to normal now.
                        RecordModemLogError( MODEMLOG_PHONE_BACK_ON_HOOK );
                        bPrevVoiceState = FALSE;
                    }

                    // Get the stray 0 or 4
                    CmdResponse = GetLastRsp();

                    // Finish setting up the modem.
                    SendCommand( AT_CMD_SBD_ALERT );
                    subState = SEND_MT_ALERT_RSP;
                    break;

                case SEND_MT_ALERT_RSP:
                     // Check if we're ready to receive data.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            StopTimer( thRespTimeOut );
                            subState = SEND_SBD_AUTOREG_CMD;
                            break;

                        case MR_FAILED:
                            // Resend the command.
                            StopTimer( thRespTimeOut );
                            subState = SEND_MT_ALERT_CMD;
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_SBD_AUTOREG_CMD:

                    if( InVoiceCall() )
                    {
                        if( !bPrevVoiceState )
                        {
                            RecordModemLogError( MODEMLOG_PHONE_OFF_HOOK );
                            bPrevVoiceState = TRUE;
                        }

                        // pause initialization.
                        break;
                    }
                    else if( bPrevVoiceState )
                    {
                        // If we were previously off hook - report we're back to normal now.
                        RecordModemLogError( MODEMLOG_PHONE_BACK_ON_HOOK );
                        bPrevVoiceState = FALSE;
                    }

                    // Finish setting up the modem.
                    SendCommand( AT_CMD_SBD_AUTO_REG );
                    subState = SEND_SBD_AUTOREG_RSP;
                    break;

                case SEND_SBD_AUTOREG_RSP:

                     // Check if we're ready to receive data.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            StopTimer( thRespTimeOut );
                            subState = SEND_SBD_DOWNLOAD_CMD;
                            break;

                        case MR_FAILED:
                            // Resend the command.
                            StopTimer( thRespTimeOut );
                            subState = SEND_SBD_AUTOREG_CMD;
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_SBD_DOWNLOAD_CMD:

                    if( InVoiceCall() )
                    {
                        if( !bPrevVoiceState )
                        {
                            RecordModemLogError( MODEMLOG_PHONE_OFF_HOOK );
                            bPrevVoiceState = TRUE;
                        }

                        // pause initialization.
                        break;
                    }
                    else if( bPrevVoiceState )
                    {
                        // If we were previously off hook - report we're back to normal now.
                        RecordModemLogError( MODEMLOG_PHONE_BACK_ON_HOOK );
                        bPrevVoiceState = FALSE;
                    }

                    // Is there anything waiting at the gateway to be downloaded?
                    SendCommand( AT_CMD_SBD_INITIATE_SESSION );
                    subState = SEND_INITIATE_TRANSFER_CMD;
                    break;

                case SEND_INITIATE_TRANSFER_CMD:

                    // Check if we're ready to receive data.
                    CmdResponse = GetInitiateSBDSessionRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:
                            // Resend the failed command.
                            StopTimer( thRespTimeOut );
                            subState = SEND_SBD_DOWNLOAD_CMD;
                            break;

                        default:
                            break;
                    }

                    break;

                case HANDLE_FINAL_RSP:

                    // Check the response.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            StopTimer( thRespTimeOut );
                            SendCommand( AT_CMD_REVISION );
                            subState = SEND_MODEM_VER_CMD;
                            break;

                        case MR_FAILED:
                            // Resend the failed command.
                            StopTimer( thRespTimeOut );
                            subState = SEND_SBD_DOWNLOAD_CMD;
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_MODEM_VER_CMD:

                    CmdResponse = GetModemVerRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            StopTimer( thRespTimeOut );
                            ATCmdState = AT_CMD_SUCCESS;
                            break;

                        case MR_FAILED:
                            // Resend the failed command.
                            StopTimer( thRespTimeOut );
                            SendCommand( AT_CMD_REVISION );
                            subState = SEND_MODEM_VER_CMD;
                            break;

                        default:
                            break;

                    }

                    break;

                default:
                    break;
            }

            break;

        case AT_CMD_IDLE:
            // Do nothing.  Waiting for upper layer to send a command.
            break;

        case AT_CMD_SENDING:

            switch( subState )
            {
                case SUBSTATE_NONE:
                    // Set on initialization.  Spin here until
                    // we have a command to send.
                    break;

                case SEND_TEXT_MSG:
                    // Check if response is "OK" or "ERROR".
                    // If successful, send to MR_SUCCESS state,
                    // send to MR_FAILED otherwise.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            // Sent binary data - now tell the modem to
                            // send the buffer to the ESS.

                            if( InVoiceCall() )
                            {
                                if( !bPrevVoiceState )
                                {
                                    RecordModemLogError( MODEMLOG_PHONE_OFF_HOOK );
                                    bPrevVoiceState = TRUE;
                                }
                            }
                            else
                            {
                                if( bPrevVoiceState )
                                {
                                    // If we were previously off hook - report we're back to normal now.
                                    RecordModemLogError( MODEMLOG_PHONE_BACK_ON_HOOK );
                                    bPrevVoiceState = FALSE;
                                }

                                SendCommand( AT_CMD_SBD_INITIATE_SESSION );
                                subState = SEND_INITIATE_TRANSFER_CMD;
                                break;
                            }


                            //if( InitiateSBDSession() )
                            //{
                            //    subState = SEND_INITIATE_TRANSFER_CMD;
                            //    break;
                            //}

                            // else fall through...

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case SEND_READY_CMD:
                    // Check if we're ready to receive data.
                    CmdResponse = GetWriteBinaryMsgRsp();

                    // Get 'READY' or numeric response before moving on:
                    // 1 = timeout
                    // 2 = bad checksum
                    // 3 = bad length
                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // If there is an error opening the file,
                            // trash this session and put it in failed 
                            // to send mode.
                            SendBinaryDataBuffer();
                            subState = SEND_DATA;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case SEND_DATA:
                    // Check if we're ready to receive data.
                    CmdResponse = GetLastRsp();

                    // Response should be '0' (0x30)<CR><LF>
                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            // Get OK or ERROR before continuing
                            subState = SEND_TEXT_MSG;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case SEND_INITIATE_TRANSFER_CMD:

                    // Check if we're ready to receive data.
                    CmdResponse = GetInitiateSBDSessionRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            // File sent successfully!
                            // Wait for upper layer to ack. and reset
                            // us back to idle. 
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case SEND_MODEM_STATE_CMD:

                    // Check the response.
                    CmdResponse = GetReqCurrCallStatusRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // SBDI definately failed - blame it on the satellites
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:

                            // We're busy dialing...could mean trouble. Let upper layer know.
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_CLEAR_BUF_CMD:

                    // Check the response.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );

                        default:
                            break;

                    }

                    break;

                case SEND_STATUS_CMD:
                    // Check if we're ready to receive data.
                    CmdResponse = GetSBDStatusRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;


                case SEND_CREG_CMD:
                    // Check if we're ready to receive data.
                    CmdResponse = GetCREGRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case SEND_CSQ_CMD:
                    // Check if we're ready to receive data.
                    CmdResponse = GetCSQRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = HANDLE_FINAL_RSP;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;                                                                  
                                                                                            
                case SEND_MAILBOX_CHECK_CMD:

                    // Check the response.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = SEND_TEXT_MSG;
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case SEND_HANGUP_CALL_CMD:

                    // Check if response is "OK" or "ERROR".
                    // If successful, send to MR_SUCCESS state,
                    // send to MR_FAILED otherwise.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            ATCmdState = AT_CMD_SUCCESS;
                            StopTimer( thRespTimeOut );
                            break;

                        case MR_FAILED:

                            // Could not get response!
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                case HANDLE_FINAL_RSP:

                    // Check the response.
                    CmdResponse = GetLastRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            ATCmdState = AT_CMD_SUCCESS;
                            StopTimer( thRespTimeOut );
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                default:
                    break;

            }

            break;

        case AT_CMD_RCVING:

            switch( subState )
            {
                case SUBSTATE_NONE:
                    // Set on initialization.  Spin here until
                    // we have a command to send.
                    break;

                case GET_DATA:
                    // Check if we're ready to receive data.
                    CmdResponse = GetRxBinaryDataBufferRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            ATCmdState = AT_CMD_SUCCESS;
                            StopTimer( thRespTimeOut );
                            break;

                        case MR_FAILED:
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thRespTimeOut );
                            break;

                        default:
                            break;

                    }

                    break;

                default:
                    break;

            }

            break;

        case AT_CMD_PGMING:

            switch( subState )
            {
                case SEND_CIS_PORT_CMD:

                    // Check the response.
                    CmdResponse = GetCISPortRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_SUCCESS;
                            StopTimer( thCISRespTimeOut );
                            break;

                        case MR_FAILED:

                            // Could not get response!
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
                            break;

                        default:
                            break;
                    }

                    break;


                case SEND_CIS_RINGER_STATE_CMD:

                    // Check the response.
                    CmdResponse = GetRingerStatusRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_SUCCESS;
                            StopTimer( thCISRespTimeOut );
                            break;

                        case MR_FAILED:

                            // Could not get response!
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_CIS_RELAY_STATE_CMD:

                    // Check the response.
                    CmdResponse = GetRelayStatusRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_SUCCESS;
                            StopTimer( thCISRespTimeOut );
                            break;

                        case MR_FAILED:

                            // Could not get response!
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
                            break;

                        default:
                            break;
                    }

                    break;

                case SEND_CIS_DOWNLOAD_CONFIG_CMD:
                    // Verify the version is correct.

                    CmdResponse = GetCISPortRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // Then, send the programming command - needs 25 seconds to download.
                            ResetTimer( thCISRespTimeOut, SATELLITE_RSP_TIMEOUT );
                            subState = CIS_DOWNLOAD_CONFIG;
                            break;

                        case MR_FAILED:

                            // INVALID RESPONSE!!! Hardware error with the CIS.
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
//                            print( " ERROR!!!" );
                            break;

                        default:
                            break;
                    }

                    break;

                case CIS_DOWNLOAD_CONFIG:

                    if( CaptureCISOutput() )
                    {
                        SetCISPort( DATA_PORT );
	                    CommitCISCfg();
                        ATCmdState = AT_CMD_SUCCESS;
                        StopTimer( thCISRespTimeOut );
                    }

                    break;

                case SEND_CIS_VERSION_QUERY_CMD:
                    // Verify the version is correct.

                    CmdResponse = GetCISVersionStatusRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:

                            // Then, send the programming command.
                            modemInfo.cmdEnum = CIS_CMD_LOAD_FLASH;
                            SendCISPortCmd();
                            subState = START_CIS_PGMING_CMD;
//                            print( " -> " );
                            break;

                        case MR_FAILED:

                            // INVALID RESPONSE!!! Hardware error with the CIS.
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ReportSystemLogError( SYS_LOG_CIS_FW_ERROR );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
//                            print( " ERROR1!!!" );
                            break;

                        default:
                            break;
                    }

                    break;

                case START_CIS_PGMING_CMD:
                    // Double check the buffer matches the expected version.

                    // Check the response.
                    CmdResponse = GetCISPortRsp();

                    switch( CmdResponse )
                    {
                        case MR_SUCCESS:
                            subState = CIS_PGMING_CMD;
                            break;

                        case MR_FAILED:

                            // Could not get response!
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ReportSystemLogError( SYS_LOG_REMOTE_CONFIG_FAILED_CISCFG );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
//                            print( " ERROR2!!!" );
                            break;

                        default:
                            break;
                    }

                    break;

                case CIS_PGMING_CMD:

//                    print( " + " );

                    if( !SendCISLoadConfigLineCmd() )
                    {
                        // We're done! Looks like we programmed the CIS successfully!
                        SetCISPort( DATA_PORT );
                        ATCmdState = AT_CMD_SUCCESS;
                        StopTimer( thCISRespTimeOut );
//                        print( " no more lines!!!" );
                    }

                    break;

                case CIS_PGMING_RSP:

                    // Double check the response for errors/success, and react accordingly.
                    if( !GetModemPortChar( &byRxByte ) )
                    {
                        break;
                    }

                    switch( byRxByte )
                    {
                        case PROG_CIS_BAD_CHECKSUM_ERROR:
                        case PROG_CIS_BAD_CHAR_RXD_ERROR:
                        case PROG_CIS_BAD_FORMAT_ERROR:

                            // Recover from these types of errors:
                            RecoverFromBadCISCmd();
                            modemInfo.cmdEnum = CIS_CMD_LOAD_FLASH;
                            SendCISPortCmd();
                            subState = START_CIS_PGMING_CMD;
                            break;

                        case PROG_CIS_MANUFACTURER_ERROR:
                        case PROG_CIS_OUT_OF_FLASH_ERROR:
                        case PROG_CIS_PAGE_ERASE_ERROR:
                        case PROG_CIS_PROG_PAGE_ERROR:
                        case PROG_CIS_BAD_HW_ID_ERROR:
                            // Log it to the system log (and send a FWACK2)
                            // Switch the port back to data
                            SetCISPort( DATA_PORT );
                            ReportSystemLogError( SYS_LOG_REMOTE_CONFIG_FAILED_CISCFG );
                            ATCmdState = AT_CMD_FAILED;
                            StopTimer( thCISRespTimeOut );
//                            print( " ERROR3!!!" );
//                            print( " = " );
//                            print( &byRxByte );
//                            print( " " );
                            break;

                        case PROG_CIS_BLOCK_PASSED:
                        
                            if( GetModemPortChar( &byRxByte ) )
                            {
                                if( byRxByte == PROG_CIS_UPLOAD_SUCCESSFUL )
                                {
                                    // Switch the port back to data
                                    SetCISPort( DATA_PORT );
                                    ATCmdState = AT_CMD_SUCCESS;
                                    StopTimer( thCISRespTimeOut );
//                                    print( " DONE!!!" );
                                    break;
                                }
                            }

                            subState = CIS_PGMING_CMD;

                            break;

                        default:
                            break;
                    }

                    break;

                default:
                    break;
           }

        case AT_CMD_FAILED:
        case AT_CMD_SUCCESS:
            // Waiting for upper level to send us to idle
            break;

        default:
            break;
    }

}


//******************************************************************************
//
//  Function: GetModemAtState
//
//  Arguments: void.
//
//  Returns: AT_CMD_STATES the modem's current state.
//
//  Description: Notifies caller of the modem's current state.
//
//******************************************************************************
AT_CMD_STATES GetModemAtState( void )
{
    return ATCmdState;
}


//******************************************************************************
//
//  Function: GetErrorCodeRsp
//
//  Arguments: void.
//
//  Returns: MODEM_ERROR_CODE_RSP reason for modem failure.
//
//  Description: Provides modemlog with a reason why the modem failed
//               or if there were any problems during transmission.
//               Resets the errorCodeRsp variable after it has been
//               retrieved by modemlog.
//
//******************************************************************************
MODEM_ERROR_CODE_RSP GetErrorCodeRsp( void )
{
    MODEM_ERROR_CODE_RSP err = errorCodeRsp;

    errorCodeRsp = MEC_NONE;

    return err;
}


//******************************************************************************
//
//  Function: GetIMEI
//
//  Arguments: void
//
//  Returns: char * to IMEI string.  
//           NULL if the string has no value.
//
//  Description: Obtains from the lower layer the modem IMEI.
//
//******************************************************************************
const char* GetIMEI( void )
{
    if( !bHaveIMEI )
    {
        return GetIMEICopy();
    }

    return szIMEI;
}


//******************************************************************************
//
//  Function: GetSBDStatus
//
//  Arguments: void.
//
//  Returns: MTStatus - 0: No message in MT buffer
//                      1: Message in MT buffer
//                      2: An error occured while attempting to do a 
//                         mailbox check or rx a msg
//
//  Description: Gets the status information returned by SBDIX.
//               MT status value is cleared after reading.
//
//               Note MTQueueNbr is only retrieved after an SBDIX or SBDREG call.
//
//******************************************************************************
MAILBOXCHECK_RSP GetSBDStatus( void )
{
    MAILBOXCHECK_RSP MTStatus = modemInfo.byMTStatus;

    if( modemInfo.byMTStatus == FAILED_MSG )
    {
        if( modemInfo.byRAFlag )
        {
            modemInfo.byMTQueueNbr++; // increment this in order to save the RA flag.
        }
    }

    modemInfo.byMTStatus  = NO_MSG;
    modemInfo.byRAFlag    = 0;

    // Preserve the length, as it is needed to compare against the read value.
    // We need this information each time this function is polled, so keep it.
//    modemInfo.wMTLength    = 0;
//    modemInfo.byMTQueueNbr = 0;
//    modemInfo.byMOStatus   = NO_MSG;

    return MTStatus;
}


//******************************************************************************
//
//  Function: GetModemSignalStrength
//
//  Arguments: void.
//
//  Returns: int value of signal strenght quality:
//           -1 failure/no value,
//           0-5 success.
//
//  Description: Provides a signal strength value, updated every poll cycle
//               (as given by rules).
//
//******************************************************************************
short GetModemSignalStrength( void )
{
    return modemInfo.iSignalStrength;
}


//******************************************************************************
//
//  Function: GetCallStatus
//
//  Arguments: void.
//
//  Returns: CALL_STATUS_RSP enum value.
//
//  Description: Provides the current call status.
//
//******************************************************************************
CALL_STATUS_RSP GetCallStatus( void )
{
    return modemInfo.byCallStatus;
}


//******************************************************************************
//
//  Function: GetRingerStatus
//
//  Arguments: void.
//
//  Returns: TRUE if ringers are on
//           FALSE otherwise.
//
//  Description: Call this command to get the current ringer status, as updated
//               by the firmware, or followed by a call to
//               SendPhoneRingerStatusCmd() then GetPhoneRingerStatus() 
//               in order to get the value directly from the CIS board.
//
//******************************************************************************
BOOL GetRingerStatus( void )
{
    return modemInfo.bRingersOn;
}


//******************************************************************************
//
//  Function: GetRelayStatus
//
//  Arguments:
//    IN byRelayNbr - Relay number to set state (RELAY_1 or RELAY_2).
//
//  Returns: TRUE if relay is Closed (enunciator ON)
//           FALSE otherwise.
//
//  Description: Call this command to get the current relay status, as updated
//               by the firmware, or followed by a call to
//               SendOutputRelayStatusCmd(...) then GetOutputRelayStatus(...)
//               this this function in order to get the value directly from
//               the CIS board.
//
//******************************************************************************
BOOL GetRelayStatus( const BYTE byRelayNbr )
{
    return modemInfo.bRelayOn[byRelayNbr];
}


//******************************************************************************
//
//  Function: GetModemCmdRspTime
//
//  Arguments: void.
//
//  Returns: DEFAULT_CMD_RESPONSE_TIME (in seconds).
//
//  Description: Allows Rules the option of getting the current set of values
//               the modem driver uses to wait before declaring a response has
//               timed out. Each modem time out event, the next value in the
//               list is used as the timeout time, until a successful response
//               is received (after which, the index goes back to the first value).
//
//               Once all the values in the list have been exhausted, the
//               final value is retained for each time out reset thereafter.
//
//               The list should only be exhausted in a major failure of the
//               modem, as only consecutive resets will increment the index.
//
//               OBSOLETE IN 200v54+
//
//******************************************************************************
DWORD GetModemCmdRspTime( void )
{
    return STANDARD_RSP_TIMEOUT;
}


//******************************************************************************
//
//  Function: GetModemCmdRspTimeInSeconds
//
//  Arguments: void.
//
//  Returns: BYTE value indicating the current time out time (in seconds).
//
//  Description: Returns 8-bit value (seconds)
//               that the system waits for satellite commands from the modem
//               (as programmed by function SetModemCmdRspTimeInSeconds).
//
//******************************************************************************
BYTE GetModemCmdRspTimeInSeconds( void )
{
    return (BYTE)(wSatelliteTimeout/1000UL);
}


//******************************************************************************
//
//  Function: GetTimeoutCount
//
//  Arguments: void.
//
//  Returns: BYTE value indicating the current time out count (max 10).
//
//  Description: Index value indicating where we are within the ratching time-
//               out list. OBSOLETE IN 200v54+
//
//******************************************************************************
BYTE GetTimeoutCount( void )
{
    return 0;
}


//******************************************************************************
//
//  Function: GetModemSWVersion
//
//  Arguments: void
//
//  Returns: char * to modem version string.  
//           NULL if the string has no value.
//
//  Description: Obtains from the lower layer the modem software version.
//
//******************************************************************************
const char* GetModemSWVersion( void )
{
    return szModemSWVersion;
}


//******************************************************************************
//
//  Function: GetMOMSN
//
//  Arguments: void.
//
//  Returns: String containing unique identifier to mobile originated message,
//           to be included in modem log file.
//
//  Description: Gets the MOMSN associated with the message sent from the modem.
//
//******************************************************************************
char* GetMOMSN( void )
{
    return modemInfo.szMOMSN;
}


//******************************************************************************
//
//  Function: GetMTMSN
//
//  Arguments: void.
//
//  Returns: String contaiing unique identifier to mobile terminated message,
//           to be included in modem log file.
//
//  Description: Gets the MTMSN associated with the message received from the modem.
//
//******************************************************************************
char* GetMTMSN( void )
{
    return modemInfo.szMTMSN;
}


//------------------------------------------------------------------------------
//  PRIVATE FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: SendCommand
//
//  Arguments:
//    IN cmd - AT command to send to the modem.
//
//  Returns: void.
//
//  Description: Clears the buffers, sets port to Data, sends a command 
//               to the modem and starts the response timer. 
//               Does not change states.
//
//******************************************************************************
void SendCommand( AT_CMD_LIST cmd )
{
    DWORD dwTimerLapse;

    // Clear the receive buffer before we send a command.
    ClearBuffers( DATA_PORT );

    if( cmd < AT_CMD_SBD_INITIATE_SESSION )
    {
        dwTimerLapse = STANDARD_RSP_TIMEOUT;
    }
    else
    {
        if( modemInfo.byRAFlag == AT_RSP_SBD_STATUS_RA )
        {
            cmd = AT_CMD_SBD_INITIATE_ALERT_SESSION;
        }
        else
        {
            cmd = AT_CMD_SBD_INITIATE_SESSION;
        }

        dwTimerLapse = wSatelliteTimeout;
    }

    // Output message to modem
    ModemPortSendBuffer( AT_CMDS[cmd], AT_CMDLEN[cmd] );
    StartTimer( thRespTimeOut, dwTimerLapse );
    print( AT_CMDS[cmd] );

#ifdef __BORLANDC__
    MainForm->OutputModemText( (char*)AT_CMDS[cmd] );
#endif
}


//******************************************************************************
//
//  Function: SendWriteBinaryMsgCmd
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Sends the AT command that starts to send the mobile 
//               orginated (MO) message. The size of the file is required 
//               for this cmd. Short burst data binary messages are limited 
//               to 1960 bytes, excluding the 2 byte checksum.
//
//******************************************************************************
void SendWriteBinaryMsgCmd( void )
{
    static BYTE szMsgLen[STR_SIZE]; // max buffer size is 1960 = 4 chars + NULL
    static BYTE szRptMsg[MAX_CMD_LINE_LEN];

    StringCpy( (char*)szRptMsg, (char*)AT_CMDS[AT_CMD_SBD_WRITE_BIN] );

    // Convert the size (in decimal) to a string, to be entered on the
    // command line.  
    IntToString( (char*)szMsgLen, modemInfo.dwTxMsgLen, 1 );

    // Enter the size of the buffer to the command
    StringNCat( (char*)szRptMsg, (char*)szMsgLen, MAX_CMD_LINE_LEN ); 

    // Message is terminated when a carriage return is entered.
    StringNCat( (char*)szRptMsg, "\r", MAX_CMD_LINE_LEN );

    // Clear the receive buffer before we send a command.
    ClearBuffers( DATA_PORT );

    // Output message to modem
    ModemPortSendBuffer( szRptMsg, StringLen( (char*)szRptMsg ) );
}


//******************************************************************************
//
//  Function: SendBinaryDataBuffer 
//
//  Arguments:
//   fileName   - A pointer to a NULL terminated string containing the
//                file name of a summary report that is to be sent.
//
//  Returns: void.
//
//  Description: This function copies each byte of a specified 
//               file into a buffer and sends that buffer to 
//               the satellite modem port. 
//
//******************************************************************************
void SendBinaryDataBuffer( void )
{
    WORD   wIndex;
    WORD   wCheckSum = 0;
    static BYTE   byChecksum[CHECKSUM_SIZE];             // String conversion of check sum

    // Calculate the checksum word and copy the buffer.
    for( wIndex = 0; wIndex < (WORD)modemInfo.dwTxMsgLen; wIndex++ )
    {
        wCheckSum += byBinMsgBuffer[wIndex];
    }

    // Clear the receive buffer before we send a command.
    ClearBuffers( DATA_PORT );

    // Add the checksum word to the end (MSB->LSB):
    byChecksum[0] = HIBYTE( wCheckSum );
    byChecksum[1] = LOBYTE( wCheckSum );

    // Send the buffer to the ISU
    ModemPortSendBuffer( byBinMsgBuffer, (WORD)modemInfo.dwTxMsgLen );

    ModemPortSendBuffer( byChecksum, 2 );

    StartTimer( thRespTimeOut, STANDARD_RSP_TIMEOUT );
}


//******************************************************************************
//
//  Function: SendCISPortCmd
//
//  Arguments: void
//
//  Returns: TRUE if we're idle and can send the cmd, 
//           FALSE if CIS is not powered.
//
//  Description: Sends the CIS port command populated by the function,
//               and sets the correct state (AT and sub).
//
//******************************************************************************
BOOL SendCISPortCmd( void )
{
    // Ensure the CIS is not currently busy first
    if( !CISPowered() )
    {
        return FALSE;
    }

    // Clear the receive buffer before we send a command.
    ClearBuffers( PROGRAMMING_PORT );

    // Output message to port
    ModemPortSendBuffer( (BYTE*)"\r", 1 );
    ModemPortSendBuffer( CIS_CMDS[modemInfo.cmdEnum], CIS_CMDLEN[modemInfo.cmdEnum] );
    ModemPortSendBuffer( (BYTE*)"\r", 1 );

//    print( "\r\nTx: " );
//    print( CIS_CMDS[modemInfo.cmdEnum] );

    StartTimer( thCISRespTimeOut, STANDARD_RSP_TIMEOUT );

    ATCmdState = AT_CMD_PGMING;
    subState   = SEND_CIS_PORT_CMD;
    return TRUE;
}


//******************************************************************************
//
//  Function: SendCISLoadConfigLineCmd
//
//  Arguments: void
//
//  Returns: void.
//
//  Description: Sends the "reload flash" command to the CIS
//               and sets the correct state (AT and sub).
//
//******************************************************************************
BOOL SendCISLoadConfigLineCmd( void )
{
    // Afterwards, send the file, 1 line at a time.
    char* szCmd = GetCISConfigLine();

    if( szCmd == NULL )
    {
        // End of file!
        return FALSE;
    }

    // Need a safe wait cycle before sending the data.
    DelayMS( 60 );

    // Clear the receive buffer before we send a command.
    ClearBuffers( PROGRAMMING_PORT );

    // Output message to port
    ModemPortSendBuffer( (BYTE*)szCmd, StringLen( szCmd ) );

//    print( "\r\nTx: " );
//    print( CIS_CMDS[modemInfo.cmdEnum] );

    StartTimer( thCISRespTimeOut, STANDARD_RSP_TIMEOUT );

    ATCmdState = AT_CMD_PGMING;
    subState   = CIS_PGMING_RSP;

    return TRUE;
}


//******************************************************************************
//
//  Function: RecoverFromBadCISCmd
//
//  Arguments: void
//
//  Returns: void.
//
//  Description: Sends the "c" command, then resets the CIS command buffer,
//               and the CIS configuration buffer index.
//
//******************************************************************************
void RecoverFromBadCISCmd( void )
{
    // Recover from these types of errors:
    ModemPortSendBuffer( CIS_CMDS[CIS_CMD_CANCEL_LOAD_FLASH], CIS_CMDLEN[CIS_CMD_CANCEL_LOAD_FLASH] ); // cancel command
    ResetTimer( thCISRespTimeOut, STANDARD_RSP_TIMEOUT );

    // Now we're able to start again from scratch.
    ResetCISConfigIndex();
}


//******************************************************************************
//
//  Function: GetResponseBuffer
//
//  Arguments:
//    EOL     - BYTE value indicating the end of line character.
//
//  Returns: TRUE if this is not the previous response, or if there
//           is data received from the port.
//           FALSE if the previous command is the response and there is
//           no more data in the buffer, if no data was received,
//
//  Description: This function fills up the byRxBuffer and updates the
//               wRxIndex.  This will constantly poll the lower layer
//               for data, until the eol character is received.  If
//               the buffer overflows, the index is reset and the
//               buffer continues to fill up.  When the EOL character is found,
//               the function returns with the response in the byRxbuffer.
//
//******************************************************************************
BOOL GetResponseBuffer( BYTE EOL )
{
#ifdef __BORLANDC__
    wRxIndex = CommRecv( &ciModemCommPort, byRxBuffer, MAX_CMD_LINE_LEN );

    if( wRxIndex == 0 )
    {
        return FALSE;
    }

    for( int i = 0; i < wRxIndex; i++ )
    {
        if( byRxBuffer[i] == EOL )
        {
            // Replace the end-of-line indicator
            byRxBuffer[i] = NULL;
            break;
        }
    }

    MainForm->OutputModemText( (char*)byRxBuffer );

    return TRUE;

#else
    BYTE byRxByte;

    // Read any pending bytes adding them to the end of the
    // modem response buffer.
    while( GetModemPortChar( &byRxByte ) )
    {
        output_raw( byRxByte );

        // This should never happen as the response buffer should be large enough
        // to accomodate a stream of rx text.
        if( wRxIndex >= MAX_CMD_LINE_LEN )
        {
            // Notify error response that we have a buffer overflow.
            errorCodeRsp = MEC_RX_BUFFER_OVERFLOW;
            wRxIndex = 0;
        }

        // Test for eol character
        if( byRxByte == EOL )
        {
            // We have a full response, break out
            byRxBuffer[wRxIndex++] = NULL;
            return TRUE;
        }

        // DO NOT Parse out any <CR> or <LF> characters as they are used
        // during the response comparison

        byRxBuffer[wRxIndex++] = byRxByte;
    }

    // Assume the packet is incomplete or empty
    return FALSE;
#endif
}


//******************************************************************************
//
//  Function: GetLastRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail, or waiting).
//
//  Description: This function looks for OK ('0'), clear fail ('1') and
//               ERROR ('4') return codes. This is the generic response
//               analyzer (doesn't suppport many of the SBD cmds).
//
//******************************************************************************
MODEM_RESPONSES GetLastRsp( void )
{
    BYTE byResponse;

    // Actual end of response character (<CR>)

    if( GetModemPortChar( &byResponse ) )
    {
#ifdef __BORLANDC__
        MainForm->OutputModemText( (char*)&byResponse );
#endif
        switch( byResponse )
        {
        case AT_RSP_OK:
            return MR_SUCCESS;

        case AT_RSP_SBD_CLEAR_FAIL:
            errorCodeRsp = MEC_CLEAR_MODEM_BUFFER_ERROR;
            return MR_FAILED;

        case AT_RSP_ERROR:
            errorCodeRsp = MEC_ERROR;
            return MR_FAILED;

        //case AT_RSP_HW_ERROR:
        //    errorCodeRsp = MEC_HW_ERROR;
        //    return MR_FAILED;

        default:
            break;
        }
    }
    
    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetWriteBinaryMsgRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail, or waiting).
//
//  Description: Checks for "Ready\r\n".  Once we've received the "Ready"
//               response, AFIRS can begin sending binary data.
//
//               Expected response:
//               "Ready<CR><LF>" or
//              '1' = timeout
//              '2' = bad checksum
//              '3' = bad length
//
//******************************************************************************
MODEM_RESPONSES GetWriteBinaryMsgRsp( void )
{
    // Actual end of response character (<CR><LF>)
    if( !GetResponseBuffer( LINE_FEED ) )
    {
        return MR_WAITING;
    }

    // Note that the response should have the <CR>.
    if( FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_SBD_WRITE_BIN_READY], wRxIndex ) > -1 )
    {
        return MR_SUCCESS;
    }

    print( "waiting for ready. got: *" );
    print( byRxBuffer );
    print( "*" );

    // Assume the first character is the numeric response
    switch( byRxBuffer[0] )
    {
        case AT_RSP_OK:
            // Successful!
            return MR_SUCCESS;

        case AT_RSP_SBD_WRITE_BIN_TIMEOUT:
            errorCodeRsp = MEC_TX_BIN_DATA_TIMEOUT;
            return MR_FAILED;

        case AT_RSP_SBD_WRITE_BIN_BAD_CHECKSUM:
            errorCodeRsp = MEC_TX_BIN_DATA_BAD_CHECKSUM;
            return MR_FAILED;

        case AT_RSP_SBD_WRITE_BIN_BAD_SIZE:
            errorCodeRsp = MEC_TX_BIN_DATA_BAD_SIZE;
            return MR_FAILED;

        default:
            break;
    }

    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetInitiateSBDSessionRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail, or waiting).
//
//  Description: This response has 6 fields, delimited by ','.
// 
//               We should see (max size = 41):
//               "+SBDI: a, bbbbb, c, ddddd, eeee, ff<CR><LF>OK<CR><LF>"
//               Where:
//                a - MO Status ( 0 No MO-SBD message to transmit 
//                                1 MO-SBD message successfully sent from the ISU to the ESS
//                                2 Error while transmitting MO-SBD msg )
//                b - MOMSN (0 - 65535)
//                c - MT Status ( 0 No SBD message from ESS
//                                1 SBD message successfully received from the ESS
//                                2 Error while receiving SBD msg or checking mailbox )
//                d - MTMSN (0 - 65535)
//                e - MT Length (0 - 1890)
//                f - MT Queued Number queued msgs at ESS (0 - 50)
//
//               Expected response:
//               "+SBDIX: 1, bbbbb, c, ddddd, eeee, ff<CR><LF>"
//
//******************************************************************************
MODEM_RESPONSES GetInitiateSBDSessionRsp( void )
{
    static char szRspBuffer[MAX_CMD_LINE_LEN];
    WORD wRspHdrSize = StringLen( (char*)AT_RSPS[AT_RSP_SBD_INITIATE_SESSION] );
    int  iHeaderIndex;

    // Parse out all fields
    static char szMOStatus[STR_SIZE];
    static char szMTStatus[STR_SIZE];
    static char szMTLength[STR_SIZE];
    static char szMTQueueNbr[STR_SIZE];


    // Acutal eol response <CR><LF>
    if( !GetResponseBuffer( LINE_FEED ) )
    {
        return MR_WAITING;
    }

    // Check if the response is what we expect first:
    // (i.e.: look for the heading)
    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_SBD_INITIATE_SESSION], wRxIndex );

    if( iHeaderIndex > -1 )
    {
        // We know the response is valid.  Remove the "heading" (+SBDIX:)
        // from the reponse
        StringMidCpy( szRspBuffer, (char*)byRxBuffer, wRspHdrSize + (WORD)iHeaderIndex, wRxIndex );

        // Now parse out the 6 fields.
        StringTok( szRspBuffer, ",", NULL );

        StringTok( "", ",", szMOStatus );
        StringTok( "", ",", modemInfo.szMOMSN );
        StringTok( "", ",", szMTStatus );
        StringTok( "", ",", modemInfo.szMTMSN );
        StringTok( "", ",", szMTLength );
        StringTok( "", ",", szMTQueueNbr );

        // These values are then reported by GetSBDStatus(...)
        modemInfo.byMOStatus   = (BYTE)StringToInt( szMOStatus );
        modemInfo.byMTStatus   = (MAILBOXCHECK_RSP)StringToInt( szMTStatus );

        switch( modemInfo.byMOStatus )
        {
            case AT_RSP_SBDI_SUCCESS:
            case AT_RSP_SBDI_SUCCESS_TRUNC:
            case AT_RSP_SBDI_SUCCESS_NO_LU:
            case AT_RSP_SBDI_SUCCESS_RFU_1:
            case AT_RSP_SBDI_SUCCESS_RFU_2:
                // ONly populate these on success.
                modemInfo.wMTLength    = (WORD)StringToInt( szMTLength );
                modemInfo.byMTQueueNbr = (BYTE)StringToInt( szMTQueueNbr );
                return MR_SUCCESS;

            case AT_RSP_SBDI_FAILURE_TIMEOUT:
                errorCodeRsp = MEC_SBDI_GSS_TIMEOUT;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_Q_FULL:
                errorCodeRsp = MEC_SBDI_GSS_Q_FULL;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_MO_ERROR:
                errorCodeRsp = MEC_SBDI_MO_SEGMENT_ERR;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_INCOMPLETE_SESSION:
                errorCodeRsp = MEC_SBDI_INCOMPLETE_SESSION;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_INVALID_SIZE:
                errorCodeRsp = MEC_SBDI_SEGMENT_SIZE_ERR;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_ACCESS_DENIED:
                errorCodeRsp = MEC_SBDI_GSS_ACCESS_DENIED;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_SBD_BLOCKED:
                // CONSIDERED A HARDWARE ERROR
                errorCodeRsp = MEC_SBDI_SBD_BLOCKED;
                ReportSystemLogError( SYS_LOG_SBD_BLOCKED );
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_NO_RSP:
                errorCodeRsp = MEC_SBDI_ISU_TIMEOUT;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_NO_CONNECTION:
                errorCodeRsp = MEC_SBDI_RF_DROP;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_NO_LINK:
                errorCodeRsp = MEC_SBDI_PROTOCOL_ERR;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_RFU_1:
            case AT_RSP_SBDI_FAILURE_RFU_2:
            case AT_RSP_SBDI_FAILURE_RFU_3:
            case AT_RSP_SBDI_FAILURE_RFU_4:
            case AT_RSP_SBDI_FAILURE_RFU_5:
            case AT_RSP_SBDI_FAILURE_RFU_6:
            case AT_RSP_SBDI_FAILURE_RFU_7:
            case AT_RSP_SBDI_FAILURE_RFU_8:
            case AT_RSP_SBDI_FAILURE_RFU_9:
            case AT_RSP_SBDI_FAILURE_RFU_10:
            case AT_RSP_SBDI_FAILURE_RFU_11:
            case AT_RSP_SBDI_FAILURE_RFU_12:
            case AT_RSP_SBDI_FAILURE_RFU_13:
            case AT_RSP_SBDI_FAILURE_RFU_14:
            case AT_RSP_SBDI_FAILURE_RFU_15:
            case AT_RSP_SBDI_FAILURE_RFU_16:
            case AT_RSP_SBDI_FAILURE_RFU_17:
            case AT_RSP_SBDI_FAILURE_RFU_18:
            case AT_RSP_SBDI_FAILURE_RFU_19:
            case AT_RSP_SBDI_FAILURE_RFU_20:
                // An error occured while attempting to transmit 
                // the MO-SBD message from ISU to ESS
                errorCodeRsp = MEC_SBDI_FAIL;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_NO_NETWORK:
                errorCodeRsp = MEC_SBDI_NO_NETWORK_SERVICE;
                return MR_FAILED;

            case AT_RSP_SBDI_FAILURE_BUSY:
                errorCodeRsp = MEC_SBDI_ISU_BUSY;
                return MR_FAILED;
        }
    }

    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetReqCurrCallStatusRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail or waiting).
//
//  Description: Gets the current modem state.
//
//               Expected Response:
//               +CLCC:006<CR><LF>0<CR>  
//
//******************************************************************************
MODEM_RESPONSES GetReqCurrCallStatusRsp( void )
{
    static char szRspBuffer[MAX_CMD_LINE_LEN];
    WORD wRspHdrSize = StringLen( (char*)AT_RSPS[AT_RSP_CALL_STATUS] );
    int iHeaderIndex;
    BYTE byResponse;

    // Acutal eol response <CR><LF>
    if( !GetResponseBuffer( LINE_FEED ) )
    {
        return MR_WAITING;
    }

    // Check if the response is what we expect first
    // (i.e.: look for the heading)
    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_CALL_STATUS], wRxIndex );

    if( iHeaderIndex > -1 )
    {
        // We know the response is valid.  Remove the "heading" (+CLCC:)
        // from the reponse
        StringMidCpy( szRspBuffer, (char*)byRxBuffer, wRspHdrSize + (WORD)iHeaderIndex, wRxIndex );

        byResponse = (BYTE)StringToInt( szRspBuffer );

        switch( byResponse )
        {
            case AT_RSP_ACTIVE_CALL_STATUS:  
                // Reason why call failed was because
                // the modem is currently in a call!
                errorCodeRsp = MEC_ACTIVE_CALL_STATUS;
                modemInfo.byCallStatus = AT_RSP_ACTIVE_CALL_STATUS;
                return MR_SUCCESS;

            case AT_RSP_HELD_CALL_STATUS:
                // Modem failed to send the message as
                // the modem is on hold
                errorCodeRsp = MEC_HELD_CALL_STATUS;
                modemInfo.byCallStatus = AT_RSP_HELD_CALL_STATUS;
                return MR_SUCCESS;

            case AT_RSP_DIALING_CALL_STATUS:
                // The modem is dialing into a call.
                errorCodeRsp = MEC_DIALING_CALL_STATUS;
                modemInfo.byCallStatus = AT_RSP_DIALING_CALL_STATUS;
                return MR_FAILED;

            case AT_RSP_INCOMING_CALL_STATUS:
                // Modem handset is ringing
                errorCodeRsp = MEC_INCOMING_CALL_STATUS;
                modemInfo.byCallStatus = AT_RSP_INCOMING_CALL_STATUS;
                return MR_SUCCESS;

            case AT_RSP_WAITING_CALL_STATUS:
                // Call waiting
                errorCodeRsp = MEC_WAITING_CALL_STATUS;
                modemInfo.byCallStatus = AT_RSP_WAITING_CALL_STATUS;
                return MR_SUCCESS;

            case AT_RSP_IDLE_CALL_STATUS:
                // Nothing happening on the phone - lets get busy!
                modemInfo.byCallStatus = AT_RSP_IDLE_CALL_STATUS;
                return MR_SUCCESS;
        }
    }

    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetSBDStatusRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES success if there is a ring alert
//                           failure if there is nothing at the gateway.
//                           waiting if no response received yet.
//
//  Description: This response has 4 fields, delimited by ','.
// 
//               We should see (max size = 31):
//               "+SBDSX: a, bbbbb, c, ddddd, e, f<CR><LF>0<CR>"
//               Where:
//                a - MO Flag ( 0 No message in MO buffer 
//                              1 Message exists in MO buffer )
//                b - MOMSN (0 - 65535)
//                c - MT Flag ( 0 No message in MT buffer 
//                              1 Message exists in MT buffer )
//                d - MTMSN (-1 - 65535)
//                e - RA Flag ( 0 No SBD ring alert
//                              1 SBD ring alert has been received and needs to be answered )
//                f - MT Queued Number queued msgs at ESS (0 - 50)
//                    ** NOTE ** Queued number is only updated on an +SBDIX or +AREG!
//
//               Expected response:
//               "+SBDSX: 0, 123, 0, -1, 0, 0<CR><LF>0<CR>"
//
//******************************************************************************
MODEM_RESPONSES GetSBDStatusRsp( void )
{
    static char szRspBuffer[MAX_CMD_LINE_LEN];
    WORD wRspHdrSize = StringLen( (char*)AT_RSPS[AT_RSP_SBD_STATUS] );
    int iHeaderIndex;
    BYTE byQueuedMsgs;

    // Parse out all fields and save them to our structure.
    static char szMOFlag[STR_SIZE];
    static char szMTFlag[STR_SIZE];
    static char szRAFlag[STR_SIZE];
    static char szQueued[STR_SIZE];

    // Actual end of response character (<CR><LF>)
    if( !GetResponseBuffer( LINE_FEED ) )
    {
        return MR_WAITING;
    }

    // Check if the response is what we expect first:
    // (i.e.: look for the heading)
    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_SBD_STATUS], wRxIndex );

    if( iHeaderIndex > -1 )
    {
        // We know the response is valid.  Remove the "heading" (+SBDSX:)
        // from the reponse
        StringMidCpy( szRspBuffer, (char*)byRxBuffer, wRspHdrSize + (WORD)iHeaderIndex, wRxIndex );

        // Now parse out the 4 fields.
        StringTok( szRspBuffer, ",", NULL );

        StringTok( "", ",", szMOFlag );
        StringTok( "", ",", modemInfo.szMOMSN );
        StringTok( "", ",", szMTFlag );
        StringTok( "", ",", modemInfo.szMTMSN );
        StringTok( "", ",", szRAFlag );
        StringTok( "", ",", szQueued );

        // Do not populate MO status as it indicates a different enum value ( 0 - no message in MO buffer, 1 - message in MO buffer)
        // Do not populate any of the other fields (MOMSN, MT status, MTMSN, Queued), as they return the previous value from an +SBDI!!!!
        modemInfo.byRAFlag = (BYTE)StringToInt( szRAFlag );
        byQueuedMsgs       = (BYTE)StringToInt( szQueued );

        if( ( modemInfo.byRAFlag == AT_RSP_SBD_STATUS_RA )
            ||
            ( modemInfo.byMTQueueNbr != 0 ) )
        {
            // MT-SBD message received at ESS
            return MR_SUCCESS;
        }
        else if( byQueuedMsgs != 0 )
        {
            // MT-SBD message queued at ESS - returned from +SBDREG on startup.
            modemInfo.byMTQueueNbr = byQueuedMsgs;
            return MR_SUCCESS;
        }   
        else
        {
            // No MT-SBD message to receive by ISU
            return MR_FAILED;
        }
    }
    
    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetCREGRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (failed - this function is called when the csq or
//           SBDI commands have failed.  Therefore, we must return what the
//           modem should rightfully expect.).
//
//  Description: This response has 2 fields, delimited by ','.
//
//               We should see (max size = 13):
//               "+CREG:aaa,bbb<CR><LF>"
//               Where:
//                a - Setting ( 0 disable network reg. unsolicited rsp
//                              1 enable network reg. unsolicited rsp
//                              2 enable unsolicited + location info (lac) )
//                b - Status  ( 0 not registered - searching for new operator
//                              1 registered, home network
//                              2 not registered, but searching
//                              3 registration denied
//                              4 unknown
//                              5 registered, roaming )
//
//               Expected response:
//               "+CREG:000,001<CR><LF>"
//
//******************************************************************************
MODEM_RESPONSES GetCREGRsp( void )
{
    static char szRspBuffer[MAX_CMD_LINE_LEN];
    WORD wRspHdrSize = StringLen( (char*)AT_RSPS[AT_RSP_CREG] );
    int iHeaderIndex;

    // We only need MO flag, the remaining fields are RFU AFIRS220.
    static char szSetting[STR_SIZE];
    static char szStatus[STR_SIZE];
    WORD wSetting;
    WORD wStatus;

    // Actual end of response character (<CR><LF>)
    if( !GetResponseBuffer( LINE_FEED ) )
    {
        return MR_WAITING;
    }

    // Check if the response is what we expect first:
    // (i.e.: look for the heading)
    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_CREG], wRxIndex );

    if( iHeaderIndex > -1 )
    {

        // We know the response is valid.  Remove the "heading" (+CREG:)
        // from the reponse
        StringMidCpy( szRspBuffer, (char*)byRxBuffer, wRspHdrSize + (WORD)iHeaderIndex, wRxIndex );

        // Now parse out the 4 fields.
        StringTok( szRspBuffer, ",", NULL );

        StringTok( "", ",", szSetting );
        StringTok( "", ",", szStatus );

        wSetting  = (WORD)StringToInt( szSetting );
        wStatus   = (WORD)StringToInt( szStatus );

        switch( wStatus )
        {
            case AT_RSP_CREG_NOT_REG:
                // Iridium error
                errorCodeRsp = MEC_CREG_NOT_REGISTERED;
                return MR_FAILED;

            case AT_RSP_CREG_REG_HOME:
                errorCodeRsp = MEC_CREG_REGISTERED_HOME;
                return MR_SUCCESS;

            case AT_RSP_CREG_SEARCHING:
                errorCodeRsp = MEC_CREG_SEARCHING;
                return MR_SUCCESS;

            case AT_RSP_CREG_DENIED:
                // Iridium error
                errorCodeRsp = MEC_CREG_DENIED;
                return MR_SUCCESS;
                
            case AT_RSP_CREG_UNKNOWN:
                // Iridium error
                errorCodeRsp = MEC_CREG_UNKNOWN;
                return MR_SUCCESS;

            case AT_RSP_CREG_REG_ROAMING:
                errorCodeRsp = MEC_CREG_REGISTERED_ROAMING;
                return MR_SUCCESS;
        }
    }
    
    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetCSQRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail or waiting).
//
//  Description: Gets the CSQ response. Fills in iSignalStrength with
//               -1 if the modem is not registered or 0-5 for signal strength
//               quality value returned by modem.
//
//               Expected Response:
//               +CSQF:5<CR><LF>  
//
//******************************************************************************
MODEM_RESPONSES GetCSQRsp( void )
{
    static char szRspBuffer[MAX_CMD_LINE_LEN];
    WORD wRspHdrSize = StringLen( (char*)AT_RSPS[AT_RSP_CSQ] );
    int iHeaderIndex;
    BYTE byResponse;

    // Actual end of response character (<CR><LF>)
    // However, if there is an error, the eol char is <CR>!!
    // So, we don't know for certain we've obtained a response
    if( !GetResponseBuffer( LINE_FEED ) )
    {
        if( wRxIndex > 0 )
        {
            if( StringCmp( (char*)byRxBuffer, AT_RSP_ERROR_STR ) == 0 )
            {
                errorCodeRsp = MEC_ERROR;
                return MR_FAILED;
            }
        }

        return MR_WAITING;
    }

    // Check if the response is what we expect first
    // (i.e.: look for the heading)
    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_CSQ], wRxIndex );

    if( iHeaderIndex > -1 )
    {
        // We know the response is valid.  Remove the "heading" (+CSQ:)
        // from the reponse
        StringMidCpy( szRspBuffer, (char*)byRxBuffer, wRspHdrSize + (WORD)iHeaderIndex, wRxIndex );

        // Buffer must have something in order to decode out.

        byResponse = (BYTE)StringToInt( szRspBuffer );

        switch( byResponse )
        {
            case AT_RSP_CSQ_LEVEL_0:
                // Signal strength is 0
                modemInfo.iSignalStrength = 0;
                return MR_FAILED;

            case AT_RSP_CSQ_LEVEL_1:
                // Signal strength is 1
                modemInfo.iSignalStrength = 1;
                return MR_SUCCESS;

            case AT_RSP_CSQ_LEVEL_2:
                // Signal strength is 2
                modemInfo.iSignalStrength = 2;
                return MR_SUCCESS;

            case AT_RSP_CSQ_LEVEL_3:
                // Signal strength is 3
                modemInfo.iSignalStrength = 3;
                return MR_SUCCESS;

            case AT_RSP_CSQ_LEVEL_4:
                // Signal strength is 4
                modemInfo.iSignalStrength = 4;
                return MR_SUCCESS;

            case AT_RSP_CSQ_LEVEL_5:
                // Signal strength is 5
                modemInfo.iSignalStrength = 5;
                return MR_SUCCESS;
        }

        // else, let it timeout.
    }

    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetIMEIRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success or waiting).
//
//  Description: Gets the modem serial number.
//
//               Expected Response:
//               {16-digit IMEI}<CR><LF>  
//
//******************************************************************************
MODEM_RESPONSES GetIMEIRsp( void )
{
    // Actual end of response character (<CR><LF>)
    if( !GetResponseBuffer( CARRIAGE_RETURN ) )
    {
        return MR_WAITING;
    }

    // We have to assume that the IMEI is the only thing in the
    // buffer.  Copy it out and save it to the local global string.
    if( wRxIndex >= IMEI_SIZE )
    {
        if( StringNCpy( szIMEI, (char*)byRxBuffer, IMEI_SIZE ) != NULL )
        {
            if( StringCmp( GetIMEICopy(), szIMEI ) != 0 )
            {
                // Save the new value to the EEPROM
                print("\r\n->IMEI changed in eeprom to: ");
                print( szIMEI );

                SetIMEICopy( szIMEI );
            }

            bHaveIMEI = TRUE;
            return MR_SUCCESS;
        }
    }

    // Fall through here means there was an issue with either the number
    // of characters we received, or in copying the string over.
    StringCpy( szIMEI, ERROR_IMEI );

    return MR_FAILED;
}


//******************************************************************************
//
//  Function: GetModemVerRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success or waiting).
//
//  Description: Gets the modem software version number.
//
//               Expected Response:
//               {Call Processor Version: IS0xxxx}<CR><LF>  
//
//******************************************************************************
MODEM_RESPONSES GetModemVerRsp( void )
{
    int  iHeaderIndex;
    BYTE byRxByte;
    WORD wRspHdrSize = StringLen( (char*)AT_RSPS[AT_RSP_REVISION] );

    if( !GetResponseBuffer( CARRIAGE_RETURN ) )
    {
        // Assume the packet is incomplete or empty
        return MR_WAITING;
    }

    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)AT_RSPS[AT_RSP_REVISION], wRxIndex );

    if( iHeaderIndex > -1 )
    {
        // We know the response is valid.  Remove the "heading" (Call Processor Version: )
        // from the reponse. Do not exceed the buffer size!
        StringMidCpy( szModemSWVersion, (char*)byRxBuffer, wRspHdrSize + (WORD)iHeaderIndex, wRspHdrSize + (WORD)iHeaderIndex + MODEM_SW_VER_SIZE );

        // finish receiving the remaining characters
        for( ; wRxIndex < 145; wRxIndex++ )
        {
            if( !GetModemPortChar( &byRxByte ) ) break;
            output_raw( byRxByte );
        }

        return MR_SUCCESS;
    }

    return MR_FAILED;
}


//******************************************************************************
//
//  Function: GetRxBinaryDataBufferRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success if file length is not zero, 
//                            fail if file length is zero).
//
//  Description: Format of response is:
//               {2-byte message length} + 
//               {binary SBD message} + 
//               {2-byte checksum} + 
//               {0}<CR>
//
//******************************************************************************
MODEM_RESPONSES GetRxBinaryDataBufferRsp( void )
{
    DEVICE_DIR  deviceDir;
    SUBDIR_NAME subDir;
    MTMDIR_RETURN_TYPE mtmReturn;
    PCFD   fd;
    MODEM_RESPONSES modemResponse = MR_SUCCESS;
    static char szDestPathFileName[EMAXPATH];
    static WORD wRxDataCount = 0;
    BYTE    byRxData = 0;
    WORD_BUFF rxChecksum;
    
    // Keep on receiving until size(2) + msg(modemInfo.wMTLength) + checksum(2) is received
    while( wRxDataCount < ( modemInfo.wMTLength + WORD_SIZE * 2 ) )
    {
        if( !GetModemPortChar( &byRxData ) )
        {
            return MR_WAITING;
        }

        // Calculate the checksum on the message only (minus size and checksum)
        if( ( wRxDataCount >= WORD_SIZE )
            &&
            ( wRxDataCount < ( modemInfo.wMTLength + WORD_SIZE ) ) )
        {
            wCalculatedCheckSum += byRxData;
        }

        RxMsg.pbyRxMsg[wRxDataCount++] = byRxData;
    }

    // Get the stray 0 or 4
    byRxData = GetLastRsp();

    rxChecksum.byData[0] = RxMsg.pbyRxMsg[wRxDataCount-2];
    rxChecksum.byData[1] = RxMsg.pbyRxMsg[wRxDataCount-1];

    if( RxMsg.rxMsg.wRxMsgLen == 0 )
    {
        // Notify upper layer
        errorCodeRsp = MEC_RX_NO_MSG_WAITING;
        modemResponse = MR_FAILED;
        RecordModemLogError( MODEMLOG_RECEIVE_FAILURE );
    }

    // The modem should check for this; this is an unlikely event, but we must
    // be sure
    else if( RxMsg.rxMsg.wRxMsgLen > MAX_RX_FILE_LEN )
    {
        errorCodeRsp = MEC_RX_BAD_FILELENGTH;
        RxMsg.rxMsg.wRxMsgLen = modemInfo.wMTLength;
    }

    if( wCalculatedCheckSum != rxChecksum.wData )
    {
        errorCodeRsp = MEC_RX_BAD_CHECKSUM;
        print( "\r\n(GetRxBinaryDataBufferRsp) Bad checksum calc'd: " );
        output_hex( wCalculatedCheckSum, 2 );
        print( " expected: " );
        output_hex( rxChecksum.wData, 2 );
        modemResponse = MR_FAILED;
    }

    wRxDataCount = 0;

    // Final check - verify the information sent from the mailbox check
    if( RxMsg.rxMsg.wRxMsgLen != modemInfo.wMTLength )
    {
        RxMsg.rxMsg.wRxMsgLen = modemInfo.wMTLength;
    }

    // **Write the buffer to a file, even if it has an error, but only if we have data to write out**
    if( modemInfo.wMTLength != 0 )
    {
        // Define, based on message type ranges, where the file will be moved to
        // at the upper layer.
        mtmReturn = DefineMsgTypeDestPath( (WORD*)&RxMsg.rxMsg.MTMessage, &deviceDir, &subDir );

        switch( mtmReturn )
        {
            case COPY_PORT3:
            case SAVE_TO_FILE:

                if( modemResponse == MR_FAILED )
                {
                    // File should be sent to the error directory.
                    deviceDir = MODEM_DIR;
                    subDir = ERROR_SUBDIR;
                }

                // We've received a buffer, now save it to a file.
                switch( RxMsg.rxMsg.MTMessage.wMTType )
                {
                case DELETE_MODEM_DIR_FILES:
                case DELETE_ELA_DIR_FILES:
                case DELETE_RS422_2_DIR_FILES:
                case DELETE_RS422_3_DIR_FILES:
                case DELETE_COMPRESS_DIR_FILES:
                case DELETE_DECOMP_DIR_FILES:
                case DELETE_FIRMWARE_DIR_FILES:
                case DELETE_SYSTEM_DIR_FILES:
                case EEPROM_CFG_MSG_TYPE:
                case PCMCIA_STATUS_MSG_TYPE:

                    CreateNewSystemFileName( szRxPathFilename,             // pathfilename
                                             szRxFilename,                 // filename
                                             GetPCMCIAPath( deviceDir, subDir ),// build dir
                                             RxMsg.rxMsg.MTMessage.wMTType );        // MT type
                    break;

                default:

                    CreateNewFileName( szRxPathFilename,             // pathfilename
                                       szRxFilename,                 // filename
                                       GetPCMCIAPath( deviceDir, subDir ),// build dir
                                       GetPCMCIAPath( deviceDir, subDir ),// search dir
                                       FALSE,                      // no time adjustment
                                       0 );                        // 0 adjust time
                    break;
                }

                fd = fileOpen( szRxPathFilename, PO_CREAT|PO_TRUNC|PO_WRONLY|PO_BINARY, PS_IREAD|PS_IWRITE );

                if( fd == -1 )
                {
                    // Could not open report file for some reason!
                    errorCodeRsp = MEC_FILE_OPEN_ERR;
                    modemResponse = MR_FAILED;
                    StringCpy( szErrString, szRxPathFilename );
                    StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_OPENED_OR_CREATED ), MAX_SYSTEM_LOG_STR );
                    SystemLog( szErrString );
                }

                // Write the buffer to a file (sans the filesize and checksum).
                else if( fileWrite( fd, (BYTE*)&RxMsg.rxMsg.MTMessage, modemInfo.wMTLength ) != modemInfo.wMTLength )
                {
                    errorCodeRsp = MEC_FILE_WRITE_ERR;
                    modemResponse = MR_FAILED;
                    fileClose( fd );
                    StringCpy( szErrString, szRxPathFilename );
                    StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_WRITTEN ), MAX_SYSTEM_LOG_STR );
                    SystemLog( szErrString );
                    MarkFileAsError( MODEM_DIR, szRxPathFilename ); // despite where it lands, it is not accurate - move it to the error dir.
                }
                else
                {
                    fileClose( fd );
                }

                if( modemResponse == MR_SUCCESS )
                {
                    ModemLog( ConvertMTMToType( szRxPathFilename, RxMsg.rxMsg.MTMessage.wMTType, EMAXPATH ), MODEMLOG_RECEIVE_SUCCESSFUL );

                    if( mtmReturn == COPY_PORT3 )
                    {
                        szDestPathFileName[0] = NULL;
                        BuildPath( szDestPathFileName, GetPCMCIAPath( RS422_PORT_3_DIR, subDir ), szRxFilename );

                        if( FileCpy( szRxPathFilename, szDestPathFileName ) )
                        {
                            ModemLog( ConvertMTMToType( szDestPathFileName, RxMsg.rxMsg.MTMessage.wMTType, EMAXPATH ), MODEMLOG_COPY_SUCCESS );
                        }
                        else
                        {
                            ModemLog( ConvertMTMToType( szDestPathFileName, RxMsg.rxMsg.MTMessage.wMTType, EMAXPATH ), MODEMLOG_COPY_FAILURE );
                        }
                    }
                }
                else
                {
                    ModemLog( ConvertMTMToType( szRxPathFilename, RxMsg.rxMsg.MTMessage.wMTType, EMAXPATH ), MODEMLOG_RECEIVE_FAILURE );
                }

                // Now, get the notification stuff out of the way, right when we get it.
                switch( GetRS422Notification() )
                {
                    case RS422_NOTIFICATION_NONE:

                        // Ensure relay is off
                        if( GetRelayStatus( TXT_MSG_RELAY ) != FALSE )
                        {
                            ToggleRelayState( TXT_MSG_RELAY, FALSE );
                        }

                        break;

                    case RS422_NOTIFICATION_PORT_2:
                        
                        if( subDir == OUTBOX_SUBDIR )
                        {
                            if( deviceDir == RS422_PORT_2_DIR )
                            {
                                ClearPTReadStatus( RS422_PORT_2 );

                                // New message, turn the relay on.
                                if( GetRelayStatus( TXT_MSG_RELAY ) != TRUE )
                                {
                                    ToggleRelayState( TXT_MSG_RELAY, TRUE );
                                }
                            }
                        }

                        break;

                    case RS422_NOTIFICATION_PORT_3:
                        
                        if( subDir == OUTBOX_SUBDIR )
                        {
                            if( ( mtmReturn == COPY_PORT3 )
                                ||
                                ( deviceDir == RS422_PORT_3_DIR ) )
                            {
                                ClearPTReadStatus( RS422_PORT_3 );

                                // New message received, turn relay on.
                                if( GetRelayStatus( TXT_MSG_RELAY ) != TRUE )
                                {
                                    ToggleRelayState( TXT_MSG_RELAY, TRUE );
                                }
                            }
                        }

                        break;

                    // This case only applies to turning off the indicator.
                    // turn it on under all circumstances.
                    case RS422_NOTIFICATION_BOTH:
                    case RS422_NOTIFICATION_EITHER:

                        if( subDir == OUTBOX_SUBDIR )
                        {
                            if( ( deviceDir == RS422_PORT_2_DIR )
                                ||
                                ( deviceDir == RS422_PORT_3_DIR )
                                ||
                                ( mtmReturn == COPY_PORT3 ) )
                            {
                                ClearPTReadStatus( RS422_PORT_2 );
                                ClearPTReadStatus( RS422_PORT_3 );

                                // New message received, turn relay on.
                                if( GetRelayStatus( TXT_MSG_RELAY ) != TRUE )
                                {
                                    ToggleRelayState( TXT_MSG_RELAY, TRUE );
                                }
                            }
                        }

                        break;

                    default:
                        break;
                }

                break;

            case BUFFER_ONLY:

                if( modemResponse == MR_SUCCESS )
                {
                    ModemLog( ConvertMTMToType( szRxFilename, RxMsg.rxMsg.MTMessage.wMTType, MAX_FILENAME_LEN ), MODEMLOG_RECEIVE_SUCCESSFUL );
                }
                else
                {
                    ModemLog( ConvertMTMToType( szRxFilename, RxMsg.rxMsg.MTMessage.wMTType, MAX_FILENAME_LEN ), MODEMLOG_RECEIVE_FAILURE );
                }

                break;

            default:
                break;
        } // switch
    }

    // Reset variable to stop the while loop
    modemInfo.wMTLength = 0;

    return modemResponse;
}


//******************************************************************************
//
//  Function: DefineMsgTypeDestPath
//
//  Arguments:
//    IN  pwMsg      - The message itself
//    OUT pDeviceDir - The device directory defined, based on wMsgType
//    OUT pSubDir    - The subdirectory directory defined, based on wMsgType
//
//  Returns: TRUE to save the buffer to a file.
//           FALSE to not record the buffer.
//
//  Description: Defines the received files' destination path, based on the
//               message type (wMsgType). Defines pDeviceDir and pSubDir
//               based on ranges of message types.
//
//******************************************************************************
MTMDIR_RETURN_TYPE DefineMsgTypeDestPath( WORD* pwMsg, DEVICE_DIR* pDeviceDir, SUBDIR_NAME* pSubDir )
{
#define TYPE_RANGE                  0x001F
#define RPT_HDR_MT_TYPE_WORD_OFFSET 1

    WORD wMsgType;
    WORD wDeviceIndex;
    WORD wSubdirIndex;
    WORD wCorrelateMTMType;
    REQ_MSG_WITH_OPTION* requestMsgWithOption = (REQ_MSG_WITH_OPTION*)pwMsg;
    REQ_MSG* requestMsg = (REQ_MSG*)pwMsg;

	// All received messages have the MT type as the second word
    wMsgType    = pwMsg[RPT_HDR_MT_TYPE_WORD_OFFSET];

    // By default, send it to Modem/Inbox
    *pDeviceDir = (DEVICE_DIR)MODEM_DIR;
    *pSubDir    = (SUBDIR_NAME)INBOX_SUBDIR;

    // We made a conscience effor to ignore the CRC calculation on commands
    // to the AFIRS - it's not worth it and the checksum offers enough protecion
    // (plus, it's backwards compatible).
    switch( wMsgType )
    {
        case A_ARF:
            print( " date/time: " );
            output_hex( requestMsg->dwDateTime, 8 );
			SetResetCmdTime( requestMsg->dwDateTime );
            PrepareRemoteSystemReset( TRUE );
            return BUFFER_ONLY;
        case B_ARF:
			SetResetCmdTime( requestMsg->dwDateTime );
            PrepareRemoteSystemReset( FALSE );
            return BUFFER_ONLY;
        case ROIACK_MSG_TYPE:
			CreateROIAckMessage( requestMsg->dwDateTime );
            print( " date/time: " );
            output_hex( requestMsg->dwDateTime, 8 );
            return BUFFER_ONLY;
        case EEPROM_CFG_REQ:
            SetRemoteConfigFileTime( requestMsg->dwDateTime );
            CreateConfigMessage( CFG_OPTION_NOT_PERSISTENT );
            return BUFFER_ONLY;
        case POWER_CYCLE_MODEM:
            if( ResetModem() )
            {
                CreateCmdAckMessage( wMsgType, TRUE, 0, requestMsg->dwDateTime );
            }
            else
            {
                CreateCmdAckMessage( wMsgType, FALSE, SYS_LOG_IN_VOICE_CALL, requestMsg->dwDateTime );
            }
            return BUFFER_ONLY;
        case FORMAT_FLASH_CARD:
            // Sends cmd ack accordingly.
            FormatPCMCIACardRemotely( requestMsg->dwDateTime );
            return BUFFER_ONLY;
        case POWER_CYCLE_CIS:
            if( PowerCycleCIS() )
            {
                CreateCmdAckMessage( wMsgType, TRUE, 0, requestMsg->dwDateTime );
            }
            else
            {
                CreateCmdAckMessage( wMsgType, FALSE, SYS_LOG_HARDWARE_NOT_SUPPORTED, requestMsg->dwDateTime );
            }
            return BUFFER_ONLY;
        case PURGE_ELA_FLASH:
            if( !ClearELAFromMemory() )
            {
                // ROI ack sent on success
                CreateCmdAckMessage( wMsgType, FALSE, SYS_LOG_BAD_HEADER_START, requestMsg->dwDateTime );
            }
            return BUFFER_ONLY;
        case PURGE_ELA_FILE:
            if( deleteFile( GetRulesBinFileName() ) )
            {
                CreateCmdAckMessage( wMsgType, TRUE, 0, requestMsg->dwDateTime );

                StringCpy( szErrString, GetRulesBinFileName() );
                StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_DELETED ), MAX_SYSTEM_LOG_STR );
                SystemLog( szErrString );
            }
            else
            {
                CreateCmdAckMessage( wMsgType, FALSE, SYS_LOG_FILE_DOES_NOT_EXIST, requestMsg->dwDateTime );
            }
            return BUFFER_ONLY;
        case DOWNLOAD_CIS_CONFIG:
            UploadCISConfig();
            CreateCmdAckMessage( wMsgType, TRUE, 0, requestMsg->dwDateTime );
            return BUFFER_ONLY;
        case FWACK3_MSG_TYPE:
            CreateSystemLogMessage( requestMsg->dwDateTime );
            print( " date/time: " );
            output_hex( requestMsg->dwDateTime, 8 );
            return BUFFER_ONLY;
        case MODEMLOG_MSG_TYPE:
            CreateModemLogMessage( requestMsg->dwDateTime );
            print( " date/time: " );
            output_hex( requestMsg->dwDateTime, 8 );
            return BUFFER_ONLY;
        case AFIRS_VER_SN_TYPE:
            CreateVersionMessage( requestMsg->dwDateTime );
            return BUFFER_ONLY;
        case AC_LOCATION_TYPE:
            CreateGPSMessage( requestMsg->dwDateTime );
            return BUFFER_ONLY;
        case RESET_573_BUS:
            if( ResetArinc573_717() )
            {
                CreateCmdAckMessage( wMsgType, TRUE, 0, requestMsg->dwDateTime );
                ReportSystemLogError( SYS_LOG_REMOTE_573_RESET );
            }
            else
            {
                CreateCmdAckMessage( wMsgType, FALSE, SYS_LOG_573_DISABLED, requestMsg->dwDateTime );
            }
            return BUFFER_ONLY;
        case GET_LOGS_IMMEDIATELY:
            PrepareSystemLogTransmission( requestMsgWithOption->dwDateTime, TRUE, requestMsgWithOption->wOption );
            return BUFFER_ONLY;
        case GET_LOGS_AFTER_FDR:
            PrepareSystemLogTransmission( requestMsgWithOption->dwDateTime, FALSE, requestMsgWithOption->wOption );
            return BUFFER_ONLY;
        default:
            break;
    }

    // Match up the MTM type with the root dir
    if( ( wMsgType >= 0x0700 ) && ( wMsgType <= ( 0x0700 + TYPE_RANGE ) ) )
    {
        *pDeviceDir = (DEVICE_DIR)ROOT_DEVICE_DIR;
        *pSubDir    = (SUBDIR_NAME)NO_SUBDIR;

        return SAVE_TO_FILE;
    }

    wDeviceIndex = (WORD)MODEM_DIR;
    wCorrelateMTMType = 0;

    for( wSubdirIndex = (WORD)NO_SUBDIR; wSubdirIndex <= SENT_SUBDIR; wSubdirIndex++, wCorrelateMTMType += (TYPE_RANGE+1) )
    {
        if( wSubdirIndex == ERROR_SUBDIR )
        {
            // Modem/Error and Modem/Working directories are not supported by the MTM types.
            wSubdirIndex++;
        }

        if( ( wMsgType >= wCorrelateMTMType ) && ( wMsgType <= ( wCorrelateMTMType + TYPE_RANGE ) ) )
        {
            *pDeviceDir = (DEVICE_DIR)wDeviceIndex;
            *pSubDir    = (SUBDIR_NAME)wSubdirIndex;

            return SAVE_TO_FILE;
        }
    }

    wDeviceIndex = (WORD)RS422_PORT_2_DIR;

    for( wSubdirIndex = (WORD)NO_SUBDIR; wSubdirIndex <= WORKING_SUBDIR; wSubdirIndex++, wCorrelateMTMType += (TYPE_RANGE+1) )
    {
        if( ( wMsgType >= wCorrelateMTMType ) && ( wMsgType <= ( wCorrelateMTMType + TYPE_RANGE ) ) )
        {
            *pDeviceDir = (DEVICE_DIR)wDeviceIndex;
            *pSubDir    = (SUBDIR_NAME)wSubdirIndex;

            // Make a copy of the file to port 3
            return COPY_PORT3;
        }
    }

    // throw the rest MTM types into the following algorithm
    for( wDeviceIndex = (WORD)ELA_DIR, wCorrelateMTMType = 0x0180; wDeviceIndex < NBR_DEVICE_DIR; wDeviceIndex++ )
    {
        for( wSubdirIndex = (WORD)NO_SUBDIR; wSubdirIndex < NBR_SUBDIR_NAME; wSubdirIndex++ )
        {
            // Ensure that only devices have the correct subdirs
            // populated.
            if( wDeviceIndex != SYSTEM_DIR )
            {
                // First directories have 6 subdirectories
                if( wSubdirIndex > WORKING_SUBDIR )
                {
                    continue;
                }
            }
            else
            {
                // Last directory only has 1 subdirectory (FDRLogs)
                switch( wSubdirIndex )
                {
                    case NO_SUBDIR:
                    case FDR_SUBDIR:
                        // Allow the below if to populate the device and subdir.
                        break;
                    default:
                        continue;
                }
            }

            if( ( wMsgType >= wCorrelateMTMType ) && ( wMsgType <= ( wCorrelateMTMType + TYPE_RANGE ) ) )
            {
                *pDeviceDir = (DEVICE_DIR)wDeviceIndex;
                *pSubDir    = (SUBDIR_NAME)wSubdirIndex;

                return SAVE_TO_FILE;
            }

            // If we get here, then get the range ready for next time.
            wCorrelateMTMType += (TYPE_RANGE+1);
       }
    }

    return SAVE_TO_FILE;
}


//******************************************************************************
//
//  Function: GetDualResponse
//
//  Arguments:
//    EOL     - BYTE value indicating the end of line character.
//
//  Returns: TRUE if this is not the previous response, or if there
//           is data received from the port.
//           FALSE if the previous command is the response and there is
//           no more data in the buffer, if no data was received,
//
//  Description: This function fills up the byRxBuffer and updates the
//               wRxIndex.  This will constantly poll the lower layer
//               for data, until the eol character is received.  If
//               the buffer overflows, the index is reset and the
//               buffer continues to fill up.  When the EOL character is found,
//               the function returns with the response in the byRxbuffer.
//
//******************************************************************************
BOOL GetDualResponse( BYTE FirstEOL, BYTE SecondEOL )
{
    static BOOL bFoundFirstEOL = FALSE;
    BYTE byRxByte;

    // Read any pending bytes adding them to the end of the
    // modem response buffer.
    while( GetModemPortChar( &byRxByte ) )
    {
        // DO NOT Parse out any <CR> or <LF> characters as they are used
        // during the response comparison

        byRxBuffer[wRxIndex++] = byRxByte;

        // This should never happen as the response buffer should be large enough
        // to accomodate a stream of rx text.
        if( wRxIndex >= MAX_CMD_LINE_LEN )
        {
            // Notify error response that we have a buffer overflow.
            errorCodeRsp = MEC_RX_BUFFER_OVERFLOW;
            wRxIndex = 0;
        }

        if( bFoundFirstEOL )
        {
            // Test for eol character
            if( byRxByte == SecondEOL )
            {
                // We have a full response, break out
                byRxBuffer[wRxIndex++] = NULL;
                bFoundFirstEOL = FALSE;
                return TRUE;
            }
        }
        else
        {
            // Test for eol character
            if( byRxByte == FirstEOL )
            {
                // We have the first EOL, now get the second
                wRxIndex = 0;
                bFoundFirstEOL = TRUE;
            }
        }
    }

    // Assume the packet is incomplete or empty
    return FALSE;
}


//******************************************************************************
//
//  Function: GetCISPortRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail or waiting).
//
//  Description: Parses the CIS response by removing the header "CMD:" then
//               verifying the command sent was echoed back.
//
//******************************************************************************
MODEM_RESPONSES GetCISPortRsp( void )
{
    int iHeaderIndex;

    // End of response is the last character in the command
    // Search for "CMD:", then the actual echoed command
    if( !GetDualResponse( ':', CIS_CMDS[modemInfo.cmdEnum][CIS_CMDLEN[modemInfo.cmdEnum]-1] ) )
    {
        return MR_WAITING;
    }

    // Check if the response is what we expect first
    // (i.e.: look for the heading)
    iHeaderIndex = FindSubStr( 0, (char*)byRxBuffer, (char*)CIS_CMDS[modemInfo.cmdEnum], StringLen( (char*)byRxBuffer ) );

    if( iHeaderIndex > -1 )
    {
        print(":)");
        output_int( modemInfo.cmdEnum );
        return MR_SUCCESS;
    }

    print( ":(" );
    output_int( modemInfo.cmdEnum );

    return MR_FAILED;
}


//******************************************************************************
//
//  Function: GetRingerStatusRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail or waiting).
//
//  Description: Ignores the "CMD:" + echo and looks directly for the response.
//
//******************************************************************************
MODEM_RESPONSES GetRingerStatusRsp( void )
{
    // Cannot use the regular API for AT commands
    // We will need to parse the command on the fly.
    BYTE byRxByte;

    // Read any pending bytes adding them to the end of the
    // modem response buffer.
    while( GetModemPortChar( &byRxByte ) )
    {
        // Test for expected character
        if( byRxByte == CIS_RSPS[CIS_RSP_RINGER_OFF][wRxIndex] )
        {
//            print( &byRxByte );

            // Clear the byte value, to ensure double characters
            // aren't recognized by accident
            byRxByte = 0xFF;

            if( ++wRxIndex == CIS_RSPLEN[CIS_RSP_RINGER_OFF] )
            {
                // We have a full response, break out
                errorCodeRsp = MEC_CIS_RINGER_OFF;
                modemInfo.bRingersOn = FALSE;

                return MR_SUCCESS;
            }
        }

        if( byRxByte == CIS_RSPS[CIS_RSP_RINGER_ON][wRxIndex] )
        {
//            print( &byRxByte );

            // Clear the byte value, to ensure double characters
            // aren't recognized by accident
            byRxByte = 0xFF;

            if( ++wRxIndex == CIS_RSPLEN[CIS_RSP_RINGER_ON] )
            {
                // We have a full response, break out
                errorCodeRsp = MEC_CIS_RINGER_ON;
                modemInfo.bRingersOn = TRUE;

                return MR_SUCCESS;
            }
        }

        // This should never happen as the response buffer should be large enough
        // to accomodate a stream of rx text.
        if( wRxIndex >= MAX_CMD_LINE_LEN )
        {
            // Notify error response that we have a buffer overflow.
            errorCodeRsp = MEC_RX_BUFFER_OVERFLOW;
            wRxIndex = 0;
        }
    }

    // Assume the packet is incomplete or empty
    return MR_WAITING;
}


//******************************************************************************
//
//  Function: GetRelayStatusRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail or waiting).
//
//  Description: Ignores the "CMD:" + echo and looks directly for the response.
//
//******************************************************************************
MODEM_RESPONSES GetRelayStatusRsp( void )
{
    // Cannot use the regular API for AT commands
    // We will need to parse the command on the fly.
    BYTE byRxByte;

    // Read any pending bytes adding them to the end of the
    // modem response buffer.
    while( GetModemPortChar( &byRxByte ) )
    {
        // Test for expected character
        switch( modemInfo.byCurrRelayNbr )
        {
            case RELAY_1:

                if( byRxByte == CIS_RSPS[CIS_RSP_RELAY_1_OFF][wRxIndex] )
                {
//                    print( &byRxByte );

                    // Clear the byte value, to ensure double characters
                    // aren't recognized by accident
                    byRxByte = 0xFF;

                    if( ++wRxIndex == CIS_RSPLEN[CIS_RSP_RELAY_1_OFF] )
                    {
                        // We have a full response, break out
                        errorCodeRsp = MEC_CIS_RELAY1_OFF;
                        modemInfo.bRelayOn[RELAY_1] = FALSE;

                        return MR_SUCCESS;
                    }
                }

                if( byRxByte == CIS_RSPS[CIS_RSP_RELAY_1_ON][wRxIndex] )
                {
//                    print( &byRxByte );

                    // Clear the byte value, to ensure double characters
                    // aren't recognized by accident
                    byRxByte = 0xFF;

                    if( ++wRxIndex == CIS_RSPLEN[CIS_RSP_RELAY_1_ON] )
                    {
                        // We have a full response, break out
                        errorCodeRsp = MEC_CIS_RELAY1_ON;
                        modemInfo.bRelayOn[RELAY_1] = TRUE;

                        return MR_SUCCESS;
                    }
                }

                break;

            case RELAY_2:

                // Test for expected character
                if( byRxByte == CIS_RSPS[CIS_RSP_RELAY_2_OFF][wRxIndex] )
                {
//                    print( &byRxByte );

                    // Clear the byte value, to ensure double characters
                    // aren't recognized by accident
                    byRxByte = 0xFF;

                    if( ++wRxIndex == CIS_RSPLEN[CIS_RSP_RELAY_2_OFF] )
                    {
                        // We have a full response, break out
                        errorCodeRsp = MEC_CIS_RELAY2_OFF;
                        modemInfo.bRelayOn[RELAY_2] = FALSE;

                        return MR_SUCCESS;
                    }
                }

                if( byRxByte == CIS_RSPS[CIS_RSP_RELAY_2_ON][wRxIndex] )
                {
//                    print( &byRxByte );

                    // Clear the byte value, to ensure double characters
                    // aren't recognized by accident
                    byRxByte = 0xFF;

                    if( ++wRxIndex == CIS_RSPLEN[CIS_RSP_RELAY_2_ON] )
                    {
                        // We have a full response, break out
                        errorCodeRsp = MEC_CIS_RELAY2_ON;
                        modemInfo.bRelayOn[RELAY_2] = TRUE;

                        return MR_SUCCESS;
                    }
                }

                break;

            default:
                return MR_FAILED;
        }

        // This should never happen as the response buffer should be large enough
        // to accomodate a stream of rx text.
        if( wRxIndex >= MAX_CMD_LINE_LEN )
        {
            // Notify error response that we have a buffer overflow.
            errorCodeRsp = MEC_RX_BUFFER_OVERFLOW;
            wRxIndex = 0;
        }
    }

    // Assume the packet is incomplete or empty
    return MR_WAITING;
}


//******************************************************************************
//
//  Function: CaptureCISOutput
//
//  Arguments: void.
//
//  Returns: TRUE if MAX_CFG_DOWNLOAD_SIZE bytes have been downloaded.
//
//  Description: Gets the current CIS configuration.
//
//******************************************************************************
BOOL CaptureCISOutput( void )
{
    char* pszCISOutput = ReceiveNewCISConfig();

#ifdef __BORLANDC__
    int iBytesRxd = CommRecv( &ciModemCommPort, pszCISOutput, MAX_CFG_DOWNLOAD_SIZE );

    if( iBytesRxd == 0 )
    {
        return FALSE;
    }

    return TRUE;
#else
    BYTE byRxByte;
    static DWORD dwIndex = 0;

    while( dwIndex < MAX_CFG_DOWNLOAD_SIZE )
    {
        KickHWWatchDog();

        if( !GetModemPortChar( &byRxByte ) )
        {
            return FALSE;
        }

        pszCISOutput[dwIndex++] = byRxByte;
    }

    dwIndex = 0;
    return TRUE;
#endif
}


//******************************************************************************
//
//  Function: GetCISVersionStatusRsp
//
//  Arguments: void.
//
//  Returns: MODEM_RESPONSES (success, fail or waiting).
//
//  Description: Gets the current CIS hardware/software versions.
//
//               Expected Response:
//               20400000 1B010000<CR>  
//
//******************************************************************************
MODEM_RESPONSES GetCISVersionStatusRsp( void )
{
    // Double check the buffer matches the expected version.

    // Actual end of response character (<CR>)
    if( !GetResponseBuffer( CARRIAGE_RETURN ) )
    {
        return MR_WAITING;
    }

    // For safety, look for the "substring" (even though the response should be the only thing there).
    if( FindSubStr( 0, (char*)byRxBuffer, (char*)CIS_RSPS[CIS_RSP_VERSION_CHECK], wRxIndex ) > -1 )
    {
        return MR_SUCCESS;
    }

    return MR_FAILED;
}


//******************************************************************************
//
//  Function: ClearBuffers
//
//  Arguments:
//    IN portState - Defines which port the command is about to be
//                   sent through.
//
//  Returns: void.
//
//  Description: Clears the wRxIndex value and fills byRxBuffer with '0xFF'.
//
//******************************************************************************
void ClearBuffers( CIS_PORT portState )
{
    // Flush the comm queue to illiminate any trailing responses
#ifdef __BORLANDC__
    FlushCommQueue( &ciModemCommPort );
#else
    FlushModemSerialTxQueue();
    FlushModemSerialRxQueue();
#endif

    SetCISPort( portState );

    // Get the rx buffer ready for the next response.
    wRxIndex = 0;
    MemSet( byRxBuffer, 0, MAX_CMD_LINE_LEN );
}


//******************************************************************************
//
//  Function: ClearModemInfo
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: In case of a modem power-out, all variables should be reset.
//
//******************************************************************************
void ClearModemInfo( void )
{
    BOOL bRingerState = modemInfo.bRingersOn;
    BOOL bFaultIndState = modemInfo.bRelayOn[RELAY_1];
    BOOL bTxtIndState = modemInfo.bRelayOn[RELAY_2];

    MemSet( &modemInfo, 0, sizeof( MODEM_INFO_STRUCT ) );

    // Enter in error values.
    modemInfo.iSignalStrength   = -1;
    modemInfo.byCallStatus      = AT_RSP_INVALID_CALL_STATUS;
    modemInfo.bRingersOn        = bRingerState; // by default, the ringers are on.
    modemInfo.bRelayOn[RELAY_1] = bFaultIndState; // ensure the right state is loaded.
    modemInfo.bRelayOn[RELAY_2] = bTxtIndState;
}


//******************************************************************************
//
//  Function: ClearRxBinaryDataVars
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Call this function seperately and outside of ClearModemInfo
//               to allow the modem power cycled type of MT msg to be documented
//               in the modem log.
//
//******************************************************************************
void ClearRxBinaryDataVars( void )
{
    MemSet( RxMsg.pbyRxMsg, 0, MAX_RX_SIZE );

    wCalculatedCheckSum = 0;
    szRxFilename[0]     = NULL;
    szRxPathFilename[0] = NULL;
}