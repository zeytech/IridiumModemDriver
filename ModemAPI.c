//******************************************************************************
//
//  ModemAPI.c: Module Title
//
//      Copyright (c) 2001-2010, Aeromechanical Services Ltd.
//      ALL RIGHTS RESERVED
//
//  This module provides interfaces for the high level AFIRS firmware 
//  to communicate with the satellite modem and for the main purpose 
//  of sending events and summary reports.
//
//  The satellite modem (aka Iridium) accepts one message at a
//  time for transmission. When the modem is idle, it will accept a
//  new message for transmission. The message consists of a binary report.
//  Report data must not exceed MAX_FILE_LEN, as defined in FileUtils.h.
//
//  The state machine that manages the modem interface must be updated
//  periodically in order to retrieve the current status of the modem
//
//  Date            Author      Comment
//  ----------------------------------------------------------------
//  2008-Sept-11     ZJ          Code Review
//
//******************************************************************************


//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#if defined( __BORLANDC__ ) || defined( WIN32 )
    #include "artl.h"
    #include "artlx.h"
    #ifdef __BORLANDC__
        #include "Stubfunctions.h"
        #include "DebugOut.h"
        #include "pcmciaAPIStub.h"
    #endif
#else
    #include "afirs.h"
    #include "ATInterface.h"
    #include "CISAPI.h"
    #include "Debug.h"
    #include "DiscreteSensors.h"
    #include "EEPROMApi.h"
    #include "FileTransfer.h"
    #include "FileUtils.h"
    #include "ModemAPI.h"
    #include "ModemSerial.h"
    #include "ModemLog.h"
    #include "MsgHandler.h"
    #include "pcmciaAPI.h"
    #include "PowerManager.h"
    #include "queue.h"
    #include "RulesBin.h"
    #include "SystemLog.h"
    #include "timer.h"
    #include "tpu.h"
    #include "utils.h"
#endif

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


#define DEFAULT_WAIT_FOR_CALLS          45000   // 45 seconds
#define DEFAULT_DIALING_DELAY           15      // in seconds!!!
#define DEFAULT_SBD_STATUS_DELAY        10000   // 10 seconds

#define DEFAULT_TIMEOUT_DELAY           (10*60000)   // 10 minutes

#define DEFAULT_SIGSTRENGTH_POLL_RATE   150000  // 2.5 * 60 * 1000 = 2 minutes 30 seconds
#define DEFAULT_SIGNAL_CHECK_DELAY      25000   // Wait period on a sig strength fail
#define DEFAULT_CSQ_DEBOUNCE_RETRIES    3      

#define DEFAULT_RETRY_COUNT             5
#define DEFAULT_RETRY_DELAY             3000    // 3 seconds

#define MDM_Q_LEN                       10


// These can only be reset by embedded rules!
// Initialized to default values.
typedef struct
{
    DWORD dwWaitForCalls;
    DWORD dwTimeoutDelay;

    DWORD dwCheckSigStrengthRate;
    DWORD dwCSQDebounceDelay;
    BYTE  byCSQMaxDebounceRetries;

    BYTE  byMaxRetries;
    DWORD dwRetryDelay;

    char  szKeepFileList[MAX_PRIORITY_FLAGS];
} MODEM_CONFIGURABLES;

// Flags are reset every initialization.
#if defined( __BORLANDC__ ) || defined( WIN32 )
#pragma pack(1)
typedef struct
#else
typedef struct __attribute__ ((__packed__)) 
#endif
{
    BYTE  byFileSendRetryCount;     // Used as the retry (>0) or new file indicator (0).
    BYTE  byFileReceiveRetryCount;  // Used to keep track of the amount of times
                                    // we retry reading a buffer from the modem.
    BYTE  byCSQDebounceCount;       // Used to ensure nuance fault lights are not encountered.
} MODEM_FLAGS;


typedef struct
{
    BOOL  bPCMCIAError;             // Indicates if a message should be sent
                                    // to the middle layer.
    BOOL  bCISActionComplete;       // Indicates if CIS command was complete.
    BOOL  bInTransparentMode;
    BOOL  bPrevHookState;
    BOOL  bPrevRIState;

    MODEM_COMMANDS  ModemCmd;
    MODEM_RESPONSES ModemRsp[NBR_MODEM_COMMANDS];

    BOOL  bSendingEnabled;
    char  szPathFileBeingSent[EMAXPATH];
    
    MODEM_STATES modemState;
    MODEM_STATES prevModemState;    // Only used if we access the modem state machine while it is in powered down state.
                                    // This only occurs for CIS commands.
} MODEM_OPTIONS;


//------------------------------------------------------------------------------
//  GLOBAL DECLARATIONS
//------------------------------------------------------------------------------


static TIMERHANDLE  thCheckCSQ;
static TIMERHANDLE  thCheckRetryDelay;
static TIMERHANDLE  thWaitForCalls;
static TIMERHANDLE  thCheckGateway;
static TIMERHANDLE  thCheckCallStatus;
static TIMERHANDLE  thTimeout;

static QUEUE_BUFF   modemQBuff[MDM_Q_LEN];

// Seperate the CIS q in case power manager reports something wrong with the CIS
// shortly after PM init.
static COMMQUEUE    QueuedCISCmd = { 0, 0, modemQBuff, MDM_Q_LEN };

static char         szErrString[MAX_SYSTEM_LOG_STR];

static MODEM_FLAGS          modemFlags;
static MODEM_OPTIONS        modemOptions;
static MODEM_CONFIGURABLES  modemConfigurables;


#if (DEBUG)
static char TEXT_MODEM_STATE[ NBR_MODEM_STATES ][TEXT_STR_LEN] = 
{
  //******************************
    " MODEM_POWERED_DOWN",
    " MODEM_INITTING",
    " MODEM_IDLE",
    " MODEM_BUSY"
}; 
#endif


//------------------------------------------------------------------------------
//  PRIVATE FUNCTION PROTOTYPES
//------------------------------------------------------------------------------


static BOOL HandleQueuedCommands( void );
    // This function initiates the CIS programming port commands
    // sent by anywhere in the system or ELA.


static BOOL GetMailboxStatus( void );
    // This function initiates the modem in sending the 
    // SendReadBinaryFileCmd() command if there is a message
    // to be received. Returns TRUE if there is a message ready 
    // to receive from the modem. FALSE otherwise.


static FILE_SEND_OPTIONS SendFileToModem( void );
    // Sends the filename and adds a modem log entry.
    //  Returns: NOT_SENDING if there are no files to send 
    //           WAITING_TO_SEND if retry delay timer has yet to expire
    //           SENDING_FILE if the file was sent


static BOOL SendCallStatusCmd( void );
    // This function checks on the modem status if we have detected
    // an off-hook event or a "ring" (incoming call).
    //  Returns: FALSE if no command was sent 
    //           FALSE if dialing timer has yet to expire
    //           TRUE if the command was sent


static void CleanUpOnUnexpectedRsp( AT_CMD_STATES atCmdState );
    // Reports in the modem.log file that an unexpected result
    // occured.


static void CleanUpOnIdle( AT_CMD_STATES atCmdState );
    // Takes care of risidual tasks to be handled by upper layer
    // (like notifying modem log, moving files to the correct
    //  directory, LEDs, setting/unsetting flags and handling
    //  retries).


static void WaitForIncommingCalls( void );
    // Provides a wait state for the state machine to allow
    // incomming calls.


static void AddNewDataToQueue( MODEM_COMMANDS Modemcommand );
    // Adds new command to queue, but only if it doesn't already 
    // exist in queue.


static void SetModemStateBusy( MODEM_COMMANDS cmd );
    // Sets all the correct variables when we go into busy state.


void HandleTimeouts( AT_CMD_STATES ATCmdState );
    // Ensures back-to-back timeouts are handled consistently.


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: InitModemAPI
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This function must be the first to be called in this  
//               module. The function initializes module static variables, 
//               and configures the satellite modem port so that 
//               it is ready for receiving and sending data. 
//
//               NOTE: PRIOR TO CALLING THIS ROUTINE MAKE SURE THAT 
//               LOWER LEVEL HARDWARE IS INITIALIZED FIRST 
//               OR ELSE THE MODEM PORT WILL HAVE UNEXPECTED
//               BEHAVIOUR.
//
//******************************************************************************
void InitModemAPI( void )
{
    SERIAL_CFG  modemPortCfg;

    // Initialize lower layer
    InitModemSerialPorts();

    modemPortCfg.dwSpeed   = WM_BPS;
    modemPortCfg.wDataBits = WM_DATA_BITS;
    modemPortCfg.wParity   = PARITY_NONE;
    modemPortCfg.wStopBits = ONE_STOP_BIT;
    modemPortCfg.wFlowCntl = RTS_CTS_FLOW_CNTL;

    // Open the Wireless Modem Serial Port
    OpenModemSerialPort( &modemPortCfg );

    // Initialize middle layer
    InitModem();

    thCheckCSQ         = RegisterTimer();
    thCheckRetryDelay  = RegisterTimer();
    thWaitForCalls     = RegisterTimer();
    thCheckGateway     = RegisterTimer();
    thCheckCallStatus  = RegisterTimer();
    thTimeout          = RegisterTimer();

    // Variables that cannot be reset once set:
    modemConfigurables.dwWaitForCalls          = DEFAULT_WAIT_FOR_CALLS;
    modemConfigurables.dwTimeoutDelay          = DEFAULT_TIMEOUT_DELAY;
                                       
    modemConfigurables.dwCheckSigStrengthRate  = DEFAULT_SIGSTRENGTH_POLL_RATE;
    modemConfigurables.dwCSQDebounceDelay      = DEFAULT_SIGNAL_CHECK_DELAY;
    modemConfigurables.byCSQMaxDebounceRetries = DEFAULT_CSQ_DEBOUNCE_RETRIES;

    modemConfigurables.byMaxRetries            = DEFAULT_RETRY_COUNT;
    modemConfigurables.dwRetryDelay            = DEFAULT_RETRY_DELAY;

    MemSet( modemConfigurables.szKeepFileList, DELETE_ALL_FILES, MAX_PRIORITY_FLAGS );
    MemSet( &modemFlags, 0, sizeof( MODEM_FLAGS ) );

//already initialized    InitQueue( &QueuedCISCmd, modemQBuff, MDM_Q_LEN );

    modemOptions.bSendingEnabled         = FALSE; // this is necessary to avoid accessing the PCMCIA from the timer ISR.
    modemOptions.bPCMCIAError            = FALSE;
    modemOptions.bCISActionComplete      = FALSE;

    modemOptions.bInTransparentMode      = FALSE;
    modemOptions.bPrevHookState          = FALSE;
    modemOptions.bPrevRIState            = FALSE;

    // Tracking variables:
    modemOptions.ModemCmd                = NO_CMD;
    
    MemSet( modemOptions.ModemRsp, (BYTE)MR_NO_RESP, NBR_MODEM_COMMANDS * sizeof( MODEM_RESPONSES ) );

    StringCpy( modemOptions.szPathFileBeingSent, NO_RPT );

    // detect timeouts from init as well
    StartTimer( thTimeout, modemConfigurables.dwTimeoutDelay );                    

    // Ensure the modem has powered up correctly
    if( GetModemAtState() == AT_CMD_POWERED_DOWN )
    {
        modemOptions.modemState     = MODEM_POWERED_DOWN;
        modemOptions.prevModemState = MODEM_POWERED_DOWN;
    }
    else
    {
        modemOptions.modemState     = MODEM_INITTING;
        modemOptions.prevModemState = MODEM_INITTING;
    }
}


//******************************************************************************
//
//  Function: DisableReportSending
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Call this function to prevent the modem state 
//               machine from trying to send files when it is 
//               in the ready state.
//
//               Note that any transmission in progress will be completed.
//
//******************************************************************************
void DisableReportSending( void )
{
    if( modemOptions.bSendingEnabled )
    {
        RecordModemLogError( MODEMLOG_SEND_DISABLED );
    }

    modemOptions.bSendingEnabled = FALSE;

    // Make sure we don't get out of this state until we re-enable sending
    StopTimer( thWaitForCalls ); 
}


//******************************************************************************
//
//  Function: EnableReportSending
//
//  Arguments:  void.
//
//  Returns: void.
//
//  Description: Call this function to allow transmission via modem.
//
//               NOTE: should not call this function during initialization!
//               Sending is automatically turned on after init is done
//               to prevent the state machine firing and hanging up the
//               system!!!
//
//******************************************************************************
void EnableReportSending( void )
{
    if( !modemOptions.bSendingEnabled )
    {
        RecordModemLogError( MODEMLOG_SEND_ENABLED );
    }

    modemOptions.bSendingEnabled = TRUE;
}


//******************************************************************************
//
//  Function: IsModemSendingEnabled
//
//  Arguments:  void.
//
//  Returns: TRUE if sending is enabled,
//           FALSE if sending is disabled.
//
//  Description: Call this function to test if we are sending msgs or not.
//
//******************************************************************************
BOOL IsModemSendingEnabled( void )
{
    return modemOptions.bSendingEnabled;
}


//******************************************************************************
//
//  Function: SendTextMsgToModem
//
//  Arguments:
//    IN  szDataBuf - A pointer to a buffer containing a text message
//                    to be sent to the modem.
//    IN  wMsgLen   - 0 to perform a mailbox check. any other value otherwise.
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: This function sends a text message to the modem. The message
//               is accepted by the modem only if it is in idle state and
//               not in a voice call. If wMsgLen is 0, a mailbox check is done.
//
//               Only 1 message can be sent at a time, and no carriage return
//               '\r' or line feed '\n' characters can be included in the 
//               message stream. The message should only contain ASCII chars.
//               Message verification is done at the middle layer.  If the
//               message exceeds maximum message length, the message will
//               be truncated.
//
//******************************************************************************
BOOL SendTextMsgToModem( const char* szDataBuf, WORD wMsgLen )
{
    // First, make sure the modem is in a state to accept the msg.
    if( modemOptions.modemState != MODEM_IDLE )
    {
        return FALSE;
    }

    // Cannot send anything if there is an incoming or outgoing call
    if( InVoiceCall() )
    {
        return FALSE;
    }

    if( wMsgLen == 0 ) 
    {
        if( !CheckMailbox() )
        {
            return FALSE;
        }

        SetModemStateBusy( MAILBOX_CHECK );
        return TRUE;
    }

    // Is the data valid for sending? Middle layer
    // truncates if the message is too big.
    if( ( szDataBuf == NULL ) 
        || 
        ( szDataBuf[0] == NULL ) )
    {
        return FALSE;
    }

    if( !SendWriteTextMsgCmd( szDataBuf ) )
    {
        // Fall through means we cannot send the message yet.
        return FALSE;
    }

    // Modem is idle - go ahead and send the message.
    SetModemStateBusy( TXING_TEXT );

    return TRUE;
}


//******************************************************************************
//
//  Function: GetTextMsgRspFromModem
//
//  Arguments: void
//
//  Returns: MODEM_RESPONSES enum value.
//           MR_SUCCESS if the modem successfully sent the message
//           MR_FAILED if there was a failed response from the modem.
//           MR_WAITING if there we're still waiting for a rsp.
//
//  Description: This function gets the modem response after a text message
//               is sent.
//
//******************************************************************************
MODEM_RESPONSES GetTextMsgRspFromModem( void )
{
    return modemOptions.ModemRsp[TXING_TEXT];
}


//******************************************************************************
//
//  Function: SendBinMsgToModem
//
//  Arguments:
//    IN  byDataBuf - A pointer to a BYTE buffer containing a text message
//                    to be sent to the modem.
//    IN  wMsgLen   - A WORD value containing the specified length in bytes
//                    of the message to be sent.
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: This function sends a binary or text message to the modem.
//               The message is accepted by the modem only if it is in idle
//               state and not in a voice call. If wMsgLen is 0, a mailbox check
//               is done.
//
//               Only 1 message can be sent at a time. The message
//               can contain ASCII chars. Message verification is done at
//               at the middle layer. If the message exceeds maximum message
//               length, the message will be truncated.
//
//******************************************************************************
BOOL SendBinMsgToModem( const BYTE* byDataBuf, WORD wMsgLen )
{
    // First, make sure the modem is in a state to accept the msg.
    if( modemOptions.modemState != MODEM_IDLE )
    {
        return FALSE;
    }

    // Cannot send anything
    if( InVoiceCall() )
    {
        return FALSE;
    }

    if( wMsgLen == 0 ) 
    {
        if( !CheckMailbox() )
        {
            return FALSE;
        }

        SetModemStateBusy( MAILBOX_CHECK );
        return TRUE;
    }

    // Is the data valid for sending? Middle layer
    // truncates if the message is too big.
    if( byDataBuf == NULL )
    {
        return FALSE;
    }

    if( !SendBinaryBuffer( byDataBuf, wMsgLen ) )
    {
        // Fall through means we cannot send the message yet.
        return FALSE;
    }

    // Modem is idle - go ahead and send the message.
    SetModemStateBusy( TXING_BUFFER );

    return TRUE;
}


//******************************************************************************
//
//  Function: GetBinMsgRspFromModem
//
//  Arguments: void
//
//  Returns: MODEM_RESPONSES enum value.
//           MR_SUCCESS if the modem successfully sent the message
//           MR_FAILED if there was a failed response from the modem.
//           MR_WAITING if there we're still waiting for a rsp.
//
//  Description: This function gets the modem response after a text message
//               is sent.
//
//******************************************************************************
MODEM_RESPONSES GetBinMsgRspFromModem( void )
{
    return modemOptions.ModemRsp[TXING_BUFFER];
}


//******************************************************************************
//
//  Function: GetCurrentModemState
//
//  Arguments: void.
//
//  Returns: One of the MODEM_STATES enumeration values.
//
//  Description: This function gets the current upper layer (API layer)
//               modem state.
//
//******************************************************************************
MODEM_STATES GetCurrentModemState( void )
{
    return( modemOptions.modemState );
}


//******************************************************************************
//
//  Function: SetSignalStrengthPollRate
//
//  Arguments:
//    IN dwPollRateInSeconds - DWORD value indicating second-intervals of checking
//                             the signal strength of the modem.
//                             Must be a value greater than zero! Previous value 
//                             maintained on a zero value.
//                             DEFAULT: 120 s (2 * 60 = 2 minutes)
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the polling rate (in secs)
//               to check the modem signal strength. This can be set 
//               any time over the course of a flight, but must be greater
//               than 0.
//
//******************************************************************************
void SetSignalStrengthPollRate( const DWORD dwPollRateInSeconds )
{
    if( dwPollRateInSeconds )
    {
        modemConfigurables.dwCheckSigStrengthRate = dwPollRateInSeconds * 1000;
    }
}


//******************************************************************************
//
//  Function: GetSignalStrengthPollRate
//
//  Arguments: void.
//
//  Returns: DWORD value indicating second-intervals of checking
//           the signal strength of the modem (default 120 s).
//
//  Description: Allows embedded rules to get the polling rate (in secs)
//               to check the modem signal strength.
//
//******************************************************************************
DWORD GetSignalStrengthPollRate( void )
{
    return modemConfigurables.dwCheckSigStrengthRate / 1000;
}


//******************************************************************************
//
//  Function: SetCSQRetryCount
//
//  Arguments:
//    IN byCSQRetryCount - BYTE value indicating number of retries to attempt on a 
//                         failed signal strength check. CANNOT EXCEED 255.
//                         DEFAULT: 3
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the Max retry count when a
//               signal strength poll fails. This can be set at any
//               time over the course of a flight, however, it is recommended
//               to only set this once at initialization.
//
//******************************************************************************
void SetCSQRetryCount( const BYTE byCSQRetryCount )
{
    modemConfigurables.byCSQMaxDebounceRetries = byCSQRetryCount;
}


//******************************************************************************
//
//  Function: GetCSQRetryCount
//
//  Arguments: void.
//
//  Returns: BYTE value indicating number of retries to attempt on a 
//           failed signal strength check. CANNOT EXCEED 255 (default 3).
//
//  Description: Allows embedded rules to get the Max retry count when a
//               signal strength poll fails.
//
//******************************************************************************
BYTE GetCSQRetryCount( void )
{
    return modemConfigurables.byCSQMaxDebounceRetries;
}


//******************************************************************************
//
//  Function: SetCSQRetryDelay
//
//  Arguments:
//    IN dwCSQRetryDelayInSeconds - DWORD value indicating second-intervals before
//                                  signal can be rechecked (after a failure).
//                                  Must be a value greater than zero! Previous value 
//                                  maintained on a zero value.
//                                  DEFAULT: 25 seconds
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the delay (in secs)
//               for signal strength recheck. This can be set any time over 
//               the course of a flight.
//
//******************************************************************************
void SetCSQRetryDelay( const DWORD dwCSQRetryDelayInSeconds )
{
    if( dwCSQRetryDelayInSeconds )
    {
        modemConfigurables.dwCSQDebounceDelay = dwCSQRetryDelayInSeconds * 1000;
    }
}


//******************************************************************************
//
//  Function: GetCSQRetryDelay
//
//  Arguments: void
//
//  Returns: DWORD value indicating second-intervals before signal can be
//           rechecked after a failure (default 25 seconds)
//
//  Description: Allows embedded rules to get the delay (in secs)
//               for signal strength recheck.
//
//******************************************************************************
DWORD GetCSQRetryDelay( void )
{
    return modemConfigurables.dwCSQDebounceDelay / 1000;
}


//******************************************************************************
//
//  Function: SetMsgRetryCount
//
//  Arguments:
//    IN byRetryCount - BYTE value indicating number of retries to attempt on a 
//                      failed transmission. CANNOT EXCEED 255, and must not be
//                      0 (or else previous value is maintained).
//                      DEFAULT: 5
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the Max retry count when a
//               file fails to be sent by the modem. This can be set at any
//               time over the course of a flight, however, it is recommended
//               to only set this once at initialization.
//
//******************************************************************************
void SetMsgRetryCount( const BYTE byRetryCount )
{
    if( byRetryCount )
    {
        modemConfigurables.byMaxRetries = byRetryCount;
    }
}


//******************************************************************************
//
//  Function: GetMsgRetryCount
//
//  Arguments: void.
//
//  Returns: BYTE value indicating number of retries to attempt on a 
//           failed transmission. CANNOT EXCEED 255 (default 5).
//
//  Description: Allows embedded rules to get the Max retry count when a
//               file fails to be sent by the modem.
//
//******************************************************************************
BYTE GetMsgRetryCount( void )
{
    return modemConfigurables.byMaxRetries;
}


//******************************************************************************
//
//  Function: SetMsgRetryDelay
//
//  Arguments:
//    IN dwRetryDelay - DWORD value indicating second-intervals before
//                      messages can be resent.
//                      DEFAULT: 3 seconds
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the delay (in secs)
//               for messages resent. This can be set any time over 
//               the course of a flight.
//
//******************************************************************************
void SetMsgRetryDelay( const DWORD dwRetryDelay )
{
    modemConfigurables.dwRetryDelay = dwRetryDelay * 1000;
}


//******************************************************************************
//
//  Function: GetMsgRetryDelay
//
//  Arguments: void
//
//  Returns: DWORD value indicating second-intervals before
//           messages can be resent (default 3 s).
//
//  Description: Allows embedded rules to get the delay (in secs)
//               for messages resent.
//
//******************************************************************************
DWORD GetMsgRetryDelay( void )
{
    return modemConfigurables.dwRetryDelay / 1000;
}


//******************************************************************************
//
//  Function: SetModemIncomingCallDelay
//
//  Arguments:
//    IN  dwCallDelayInSeconds - DWORD value indicating second-intervals between
//                               AT commands. 
//                               DEFAULT: 45 seconds.
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the delay (in secs)
//               to allow incoming calls. This takes affect after each AT
//               command, unless if the modem is in use already (voice).
//               This can be set any time over the course of a flight.
//
//******************************************************************************
void SetModemIncomingCallDelay( const DWORD dwCallDelayInSeconds )
{
    modemConfigurables.dwWaitForCalls = dwCallDelayInSeconds * 1000;
}


//******************************************************************************
//
//  Function: GetModemIncomingCallDelay
//
//  Arguments: void
//
//  Returns: DWORD value indicating second-intervals between
//           AT commands (default 45 s).
//
//  Description: Allows embedded rules to get the delay (in secs)
//               to allow incoming calls. Use to check value previously
//               set by rules or to check the default value.
//
//******************************************************************************
DWORD GetModemIncomingCallDelay( void )
{
    return modemConfigurables.dwWaitForCalls / 1000;
}


//******************************************************************************
//
//  Function: SetModemDialingDelay
//
//  Arguments:
//    IN  dwDialingDelayInSeconds - DWORD value indicating (in seconds) the amount
//                                  of time to wait before retrying to send a file
//                                  after discovering the modem is busy dialing 
//                                  (possibly hung)
//                                  DEFAULT: 15 seconds
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the the amount
//               of time (in seconds) to wait before retrying to 
//               send a file after discovering the modem is dialing 
//               (and possibly hung). This can be set any time over the
//               course of a flight.
//
//******************************************************************************
void SetModemDialingDelay( const DWORD dwDialingDelayInSeconds )
{
    // do nothing - function is obsolete
}


//******************************************************************************
//
//  Function: GetModemDialingDelay
//
//  Arguments: void
//
//  Returns: DWORD value indicating second-intervals between
//           retries (default 15 s).
//
//  Description: Allows embedded rules to set the the amount
//               of time (in seconds) to wait before retrying to 
//               send a file after discovering the modem is dialing 
//               (and possibly hung). 
//
//******************************************************************************
DWORD GetModemDialingDelay( void )
{
    return DEFAULT_DIALING_DELAY;
}


//******************************************************************************
//
//  Function: SetModemTimeoutWait
//
//  Arguments:
//    IN  dwDTimeoutDelayInSeconds - DWORD value (in seconds) indicating the amount
//                                   of time to wait before resetting the CIS/modem
//                                   after detecting continuous timeouts.
//                                   DEFAULT: 30 minutes
//
//  Returns: void.
//
//  Description: Allows embedded rules to set the the amount of time (in seconds)
//               to wait before resetting the CIS/modem after discovering 
//               the modem communciations is possibly corrupted/nonexistent.
//               This can be set any time over the course of a flight.
//
//******************************************************************************
void SetModemTimeoutWait( const DWORD dwTimeoutDelayInSeconds )
{
    modemConfigurables.dwTimeoutDelay = dwTimeoutDelayInSeconds * 1000;
}


//******************************************************************************
//
//  Function: GetModemTimeoutWait
//
//  Arguments: void
//
//  Returns: DWORD value indicating the amount of time to wait before resetting
//           the CIS/modem after detecting continuous timeouts (default 30 minutes).
//
//  Description: Allows embedded rules to set the the amount
//               of time (in seconds) to wait before resetting the CIS/modem 
//               after discovering the modem communications is down or corrupt. 
//
//******************************************************************************
DWORD GetModemTimeoutWait( void )
{
    return modemConfigurables.dwTimeoutDelay / 1000;
}


//******************************************************************************
//
//  Function: KeepSentFiles
//
//  Arguments:
//    IN  pcPriorityList - Pointer to an array of characters, that list the
//                         priority flags to keep after the file is sent.
//
//                  NOTE - NULL (DELETE_ALL_FILES define) will delete ALL files
//                         after being sent. ***DEFAULT SETTING***
//                       - "*" (KEEP_ALL_FILES define ) will NOT delete any files
//                         after being sent.
//                       - pcPriorityList must be MAX_PRIORITY_FLAGS in size
//                         and be null terminated!
//
//  Returns: void.
//
//  Description: Keeps any file that starts with a character within the
//               pcPriorityList.
//
//******************************************************************************
void KeepSentFiles( char* pcPriorityList )
{
    if( pcPriorityList == DELETE_ALL_FILES )
    {
        modemConfigurables.szKeepFileList[0] = DELETE_ALL_FILES;
    }
    else
    {
        StringCpy( modemConfigurables.szKeepFileList, pcPriorityList );
    }
}


//******************************************************************************
//
//  Function: InVoiceCall
//
//  Arguments: void.
//
//  Returns: TRUE if there is an outgoing call.
//           FALSE otherwise.
//
//  Description: This function detects when there is an outgoing
//               call the using control signal line (DSR).
//               DOES NOT DETECT INCOMING CALLS - after detecting errors with 
//               Iridium and CIS, RI check was removed.
//
//******************************************************************************
BOOL InVoiceCall( void )
{
    // Cannot send anything if there is an outgoing call
    // DO NOT CHECK FOR INCOMING CALLS due to:
    // - modem power cycles and sometimes causes a RI glitch and CIS latches onto the signal FALSELY
    // - Iridium at one point was having network issues, and messages queued at the gateway
    //   were coming across on the RI line (instead of RA flag). It's just safer to avoid the RI line
    // Negatives:
    // - possible race condition with incoming call and sending data at the same time.
    // The chances of that happening seem lower - integrity of the modem is paramount.
    if( ReadModemPortDSRLine() ) // true (high) if phone is off hook
    {
        return TRUE;
    }

    return FALSE;
}


//******************************************************************************
//
//  Function: ReportPCMCIAError
//
//  Arguments:
//    IN  bErrorWithPCMCIACard - BOOL flag, indicating to this layer, if we
//                               need to send a buffer error message.
//
//  Returns: void.
//
//  Description: Indicates if the PCMCIA card is missing, in which case,
//               a text file is sent via modem ONCE per power up.
//
//******************************************************************************
void ReportPCMCIAError( BOOL bErrorWithPCMCIACard )
{
    modemOptions.bPCMCIAError = bErrorWithPCMCIACard;
}


//******************************************************************************
//
//  Function: UploadCISConfig
//
//  Arguments: void.
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: Call this function to get the current configuration from
//               the CIS board. MAY REQUIRE BACKGROUND INTERRUPTS!!!!
//
//******************************************************************************
BOOL UploadCISConfig( void )
{
    // First, make sure the modem is in a state to accept the msg.
    modemOptions.bCISActionComplete = FALSE;

    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
      )
    {
        if( SendDownloadCISCmd() )
        {
            // Modem is idle - now wait for the response.
            SetModemStateBusy( UPLOAD_CIS_CONFIG );
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( UPLOAD_CIS_CONFIG );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: IsCISActionComplete
//
//  Arguments: void.
//
//  Returns: TRUE if the config upload is complete.
//
//  Description: Poll this function after deploying UploadCISConfig or
//               ProgramCIS to ensure the command is complete.
//
//******************************************************************************
BOOL IsCISActionComplete( void )
{
    return modemOptions.bCISActionComplete;
}


//******************************************************************************
//
//  Function: ProgramCIS
//
//  Arguments: void.
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: Call this function to program the CIS board.
//
//******************************************************************************
BOOL ProgramCIS( void )
{
    // First, make sure the modem is in a state to accept the msg.
    modemOptions.bCISActionComplete = FALSE;

    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
      )
    {
        if( SendProgramCISCmd() )
        {
            // Modem is idle - now wait for the response.
            SetModemStateBusy( CONFIGURE_CIS );
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( CONFIGURE_CIS );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: SaveCISCurrentState
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Call this function to save the current state of the CIS board.
//               SHOULD ONLY BE CALLED PRIOR TO A CIS RESET!
//
//******************************************************************************
void SaveCISCurrentState( void )
{
    if( GetRingerStatus() )
    {
        AddNewDataToQueue( RINGER_ON );
    }
    else
    {
        AddNewDataToQueue( RINGER_OFF );
    }

    if( GetRelayStatus( RELAY_1 ) )
    {
        AddNewDataToQueue( RELAY1_ON );
    }
    else
    {
        AddNewDataToQueue( RELAY1_OFF );
    }

    if( GetRelayStatus( RELAY_2 ) )
    {
        AddNewDataToQueue( RELAY2_ON );
    }
    else
    {
        AddNewDataToQueue( RELAY2_OFF );
    }
}


//******************************************************************************
//
//  Function: ResetCIS
//
//  Arguments: void.
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: This function will perform a soft reset on the CIS (by sending
//               it the "reset" command). Saves current CIS state and
//               power cycles the modem.
//
//******************************************************************************
BOOL ResetCIS( void )
{
    // First, make sure the modem is in a state to accept the msg.
    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
      )
    {
        if( SendCISResetCmd() )
        {
            // Modem is idle - now wait for the response.
            SaveCISCurrentState();
            SetModemStateBusy( RESET_CIS );
            PowerCycleModem();
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( RESET_CIS );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: TogglePhoneRinger
//
//  Arguments:
//    IN bRingerOn - TRUE to turn ringer on
//                   FALSE to turn ringer off
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: Call this function to turn the phone ringer on or off.
//
//******************************************************************************
BOOL TogglePhoneRinger( const BOOL bRingerOn )
{
    MODEM_COMMANDS mdmCmd = bRingerOn ? RINGER_ON:RINGER_OFF;

    // First, make sure the modem is in a state to accept the msg.
    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
      )
    {
        if( SendSetRingerCmd( bRingerOn ) )
        {
            // Modem is idle - now wait for the response.
            SetModemStateBusy( mdmCmd );
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( mdmCmd );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: SendPhoneRingerStatusCmd
//
//  Arguments: void.
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: Call this function to poll the CIS board and send the
//               ringer status command. Must be followed by a call to
//               GetPhoneRingerStatus() and on success, GetRingerStatus().
//
//******************************************************************************
BOOL SendPhoneRingerStatusCmd( void )
{
    // First, make sure the modem is in a state to accept the msg.
    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
    {
        if( SendGetRingerStatusCmd() )
        {
            // Modem is idle - now wait for the response.
            SetModemStateBusy( RINGER_STATUS );
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( RINGER_STATUS );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: GetPhoneRingerStatus
//
//  Arguments: void.
//
//  Returns: MR_SUCCESS successfully received response.
//           MR_FAILED  failed to get a correct response.
//           MR_WAITING command has been deployed; waiting for a response.
//           MR_NO_RESP the response has been cleared or the command was never
//                      sent.
//
//  Description: Call this command after a call to
//               SendPhoneRingerStatusCmd() in order to know when the response
//               has arrived and get the updated value directly from the CIS
//               board by calling GetRingerStatus().
//
//******************************************************************************
MODEM_RESPONSES GetPhoneRingerStatus( void )
{
    return modemOptions.ModemRsp[RINGER_STATUS];
}


//******************************************************************************
//
//  Function: ToggleRelayState
//
//  Arguments:
//    IN byRelayNbr   - Relay number to set state (0 or 1).
//    IN bRingerState - TRUE to set the relay (logical 1)
//                      FALSE to clear the relay (logical 0)
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: Set or clear the output relay accordingly.
//
//******************************************************************************
BOOL ToggleRelayState( const BYTE byRelayNbr, const BOOL bRelayState )
{
    MODEM_COMMANDS mdmCmd = NO_CMD;

    if( byRelayNbr == RELAY_1 )
    {
        mdmCmd = bRelayState ? RELAY1_ON:RELAY1_OFF;
    }
    else if( byRelayNbr == RELAY_2 )
    {
        mdmCmd = bRelayState ? RELAY2_ON:RELAY2_OFF;
    }

    // First, make sure the modem is in a state to accept the msg.
    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
    {
        if( SendSetRelayCmd( byRelayNbr, bRelayState ) )
        {
            // Modem is idle - go ahead and send the message. 
            SetModemStateBusy( mdmCmd );
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( mdmCmd );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: SendOutputRelayStatusCmd
//
//  Arguments:
//    IN byRelayNbr - Relay number to set state (0 or 1).
//
//  Returns: TRUE if the modem is in idle state and can send the message.
//
//  Description: Call this function to poll the CIS board and send the
//               relay status command. Must be followed by a call to
//               GetOutputRelayStatus() and on success,
//               GetRelayStatus( byRelayNbr ).
//
//******************************************************************************
BOOL SendOutputRelayStatusCmd( const BYTE byRelayNbr )
{
    MODEM_COMMANDS mdmCmd = ( byRelayNbr == RELAY_1 ) ? RELAY1_STATUS:RELAY2_STATUS;

    // First, make sure the modem is in a state to accept the msg.
    if( ( modemOptions.modemState == MODEM_IDLE )
        ||
        ( modemOptions.modemState == MODEM_POWERED_DOWN ) )
    {
        if( SendGetRelayStatusCmd( byRelayNbr ) )
        {
            // Modem is idle - now wait for the response.
            SetModemStateBusy( mdmCmd );
            return TRUE;
        }
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( mdmCmd );  // Save data

    return FALSE;
}


//******************************************************************************
//
//  Function: GetOutputRelayStatus
//
//  Arguments:
//    IN byRelayNbr - Relay number to set state (0 or 1).
//
//  Returns: MR_SUCCESS successfully received response.
//           MR_FAILED  failed to get a correct response.
//           MR_WAITING command has been deployed; waiting for a response.
//           MR_NO_RESP the response has been cleared or the command was never
//                      sent.
//
//  Description: Call this command after a call to
//               SendOutputRelayStatusCmd( byRelayNbr ) in order to know when the
//               response has arrived and get the updated value directly from the CIS
//               board by calling GetRelayStatus( byRelayNbr ).
//
//******************************************************************************
MODEM_RESPONSES GetOutputRelayStatus( const BYTE byRelayNbr )
{
    MODEM_COMMANDS mdmCmd = ( byRelayNbr == RELAY_1 ) ? RELAY1_STATUS:RELAY2_STATUS;

    return modemOptions.ModemRsp[mdmCmd];
}


//******************************************************************************
//
//  Function: GetModemCmdRsp
//
//  Arguments:
//    IN mdmCmd - Command enum to verify the response value after sending the command.
//
//  Returns: MR_SUCCESS successfully received response.
//           MR_FAILED  failed to get a correct response.
//           MR_WAITING command has been deployed; waiting for a response.
//           MR_NO_RESP the response has been cleared or the command was never
//                      sent.
//
//  Description: Call this command after a call to any command to verify
//               if the response was received successfully or not.
//
//******************************************************************************
MODEM_RESPONSES GetModemCmdRsp( MODEM_COMMANDS mdmCmd )
{
    return modemOptions.ModemRsp[mdmCmd];
}


//******************************************************************************
//
//  Function: HangupCall
//
//  Arguments: void.
//
//  Returns: TRUE if call was successfully hung up.
//           FALSE if either state machine is not in idle
//
//  Description: Modularizes the hangupcall command as it is used more than
//               once.
//
//******************************************************************************
BOOL HangupCall( void )
{
    if( SendCallHangupCmd() )
    {
        // Modem is idle - go ahead and send the message. 
        SetModemStateBusy( CALL_HANGUP );
        print( " going to busy...." );
        return TRUE;
    }

    // Fall through means we cannot send the message yet.
    // Queue it up instead.
    AddNewDataToQueue( CALL_HANGUP );  // Save data
    return FALSE;
}


//******************************************************************************
//
//  Function: EnteredTransparentModemMode
//
//  Arguments:
//    IN  bMode - BOOL value letting the driver know if
//                transparent mode is on or off.
//
//  Returns: void.
//
//  Description: Notifies the middle driver not to empty the FIFO 
//               as transparent mode is doing this!
//
//******************************************************************************
void EnteredTransparentModemMode( BOOL bMode )
{
    modemOptions.bInTransparentMode = bMode;
}


//******************************************************************************
//
//  Function: ProcessModemStateMachine
//
//  Arguments: void.
//
//  Returns:void.
//
//  Description: This function must be called periodically in order to 
//               process report sending. This function manages the 
//               state machine that interfaces with the modem's middle layer
//               AT command set. 
//
//******************************************************************************
void ProcessModemStateMachine( void )
{
    AT_CMD_STATES atCmdState;
    //static MODEM_STATES prevMdmState = MODEM_POWERED_DOWN;
    //static AT_CMD_STATES prevAtCmdState = AT_CMD_POWERED_DOWN;

    if( modemOptions.bInTransparentMode )
    {
        // Do not process anything as we are in transparent mode!!
        return;
    }

    UpdateModemState();

    atCmdState = GetModemAtState();

    // show me the state every second
    //if( ( prevMdmState != modemOptions.modemState )
    //    ||
    //    ( prevAtCmdState != atCmdState ) )
    //{
    //    prevMdmState   = modemOptions.modemState;
    //    prevAtCmdState = atCmdState;

    //    print( "(" );
    //    output_int( modemOptions.modemState );
    //    print( "/" );
    //    output_int( atCmdState );

    //    if( modemOptions.modemState == MODEM_BUSY )
    //    {
    //        print( "*" );
    //        output_int( modemOptions.ModemCmd );
    //    }

    //    print( ")" );
    //}

    // Always power down the upper level, as soon as the lower level
    // has detected a power down event.
    if( ( atCmdState == AT_CMD_POWERED_DOWN ) 
        &&
        ( modemOptions.modemState != MODEM_POWERED_DOWN ) )
    {
        modemOptions.modemState = MODEM_POWERED_DOWN;
        RecordModemLogError( MODEMLOG_MODEM_POWERED_DOWN );
        MemSet( modemOptions.ModemRsp, (BYTE)MR_NO_RESP, NBR_MODEM_COMMANDS * sizeof( MODEM_RESPONSES ) );
    }

    // Now update the state machine.
    switch( modemOptions.modemState )
    {
        case MODEM_INITTING:

            // Wait here for the modem to be ready. If it is anything but
            // success, reset the modem state to init.
            switch( atCmdState )
            {
                case AT_CMD_INITTING:
                    // Still initializing, wait for either a timeout, or a response
                    break;

                case AT_CMD_SUCCESS:

                    SetATCmdStateIdle();

                    // Be sure to reset these flags in case we ended up here
                    // due to an unexpected state change.
                    modemOptions.modemState     = MODEM_IDLE;
                    modemOptions.prevModemState = MODEM_IDLE;
                    modemOptions.ModemCmd       = NO_CMD;

                    MemSet( &modemFlags, 0, sizeof( MODEM_FLAGS ) );

                    // Stop/restart timers in case this is an error recovery.
                    // Ensure a signal strength is done on start-up
                    // and that we synchronize status with the CIS board.
                    StopTimer( thCheckRetryDelay );
                    StartTimer( thCheckCSQ, 0 );
                    StartTimer( thCheckGateway, DEFAULT_SBD_STATUS_DELAY );
                    StartTimer( thCheckCallStatus, DEFAULT_SBD_STATUS_DELAY );
                    ResetTimer( thTimeout, modemConfigurables.dwTimeoutDelay );                    

                    AddNewDataToQueue( RINGER_STATUS );
                    AddNewDataToQueue( RELAY1_STATUS );
                    AddNewDataToQueue( RELAY2_STATUS );

                    MemSet( modemOptions.ModemRsp, (BYTE)MR_NO_RESP, NBR_MODEM_COMMANDS * sizeof( MODEM_RESPONSES ) );

                    ReportSystemLogError( SYS_LOG_MODEM_INITITIALIZED );

                    break;

                case AT_CMD_FAILED:
                case AT_CMD_TIMED_OUT:

                    // only reset the modem if we previously tried to empty the MT buffer
                    // or if there are no queued messages
                    if( GetMailboxStatus() )
                    {
                        break;
                    }

                    HandleTimeouts( atCmdState );
                    // else, fall through...

                default:
                    // All these responses are unexpected in this state - revert
                    // to the init'ing state until they clear.
                    SetATCmdStateInit();
                    modemOptions.modemState = MODEM_INITTING;
                    break;
            }

            break;

        case MODEM_IDLE:

            // In this state, the modem must be idle too. If it is anything but
            // that, revert to the init'ing state.
            switch( atCmdState )
            {
                case AT_CMD_IDLE:

                    // We can switch ports to the programming port if necessary.
                    HandleQueuedCommands();

                    // Timer will only expire if it was started, so further error checking is not necessary.
                    if( TimerExpired( thWaitForCalls ) )
                    {
                        StopTimer( thWaitForCalls );
                        modemOptions.bSendingEnabled = TRUE;
                    }

                    break;

               default:
                    // All these responses are unexpected in this state - revert
                    // to the init'ing state until they clear.
                    SetATCmdStateInit();
                    CleanUpOnUnexpectedRsp( atCmdState );
                    modemOptions.modemState = MODEM_INITTING;
                    break;
            }

            // Make sure we're still idle after above check
            if( modemOptions.modemState != MODEM_IDLE )
            {
                break;
            }

            // This command is local and can be performed any time
            // (even in the absence of a satellite connection)
            if( GetMailboxStatus() )
            {
                // We have a message pending; 
                // download it from the modem buffer,
                break;
            }

            // Cannot use the satellite if there is an incoming or outgoing call.
            // Ensure we do not enter the modem's race condition - check every time
            // we're in a call.
            if( InVoiceCall() ) // true (high) if phone is off hook
            {
                SendCallStatusCmd();

                if( !modemOptions.bPrevHookState )
                {
                    // if previously we were on hook, this is the time
                    // to report an off hook event
                    RecordModemLogError( MODEMLOG_PHONE_OFF_HOOK );
                    modemOptions.bPrevHookState = TRUE;
                }

                break;
            }
            else if( modemOptions.bPrevHookState )
            {
                // If we were previously off hook - report we're back to normal now.
                RecordModemLogError( MODEMLOG_PHONE_BACK_ON_HOOK );
                modemOptions.bPrevHookState = FALSE;
            }

            if( ReadModemPortRILine() )  // true (high) if Ring indicator is on
            {
                // Report we have an incoming call
                if( !modemOptions.bPrevRIState )
                {
                    RecordModemLogError( MODEMLOG_INCOMING_CALL );
                    modemOptions.bPrevRIState = TRUE;
                }

                // do not stop transmission based on this signal - modem will tell us if it's busy
            }
            else if( modemOptions.bPrevRIState )
            {
                // If we were previously in a call - report we're back to normal now.
                RecordModemLogError( MODEMLOG_INCOMING_CALL_COMPLETE );
                modemOptions.bPrevRIState = FALSE;
            }

            if( TimerExpired( thCheckCSQ ) )
            {
                // Check signal quality and reset the timer.
                // Technically, this command doesn't need satellite - the value
                // polled is stored locally on the modem (like the MT buffer).
                if( SendCSQCmd() )
                {
                    SetModemStateBusy( GETTING_CSQ );
                    ResetTimer( thCheckCSQ, modemConfigurables.dwCheckSigStrengthRate );
                    break;
                }
            }

            if( modemOptions.bSendingEnabled ) 
            {
                // Check to see if we have any files ready to send/resend
                if( SendFileToModem() == SENDING_FILE )
                {
                    // a file is being sent modem state change 
                    // is handled by function.
                    break;
                }

                else if( TimerExpired( thCheckGateway ) )
                {
                    // Do a gateway check (for free!)
                    if( CheckGateway() )
                    {
                        SetModemStateBusy( GATEWAY_CHECK );
                        ResetTimer( thCheckGateway, DEFAULT_SBD_STATUS_DELAY );
                        break;
                    }
                }
            }

            break;

        case MODEM_BUSY:

            // Check to see what the last response was from the modem.
            // The switch() statement only handles responses of interest.
            switch( atCmdState )
            {
                case AT_CMD_TIMED_OUT:
                    // Depending on the what we were just doing,
                    // handle the time out.
                case AT_CMD_SUCCESS:
                    // Msg successfully sent! Modem is now outputting the result
                    // status. Will need to wait for it to stop doing that before
                    // trying to send another msg.
                case AT_CMD_FAILED:
                    // We could not send a report for some reason.
                    // Clear the failed state and go into idle state.
                    SetATCmdStateIdle();
                    modemOptions.modemState = MODEM_IDLE;
                    CleanUpOnIdle( atCmdState );

                    break;

                case AT_CMD_SENDING:
                case AT_CMD_RCVING:
                case AT_CMD_PGMING:
                    // Sending or Receiving:
                    // Wait here until the time-out timer (at the lower level) returns
                    // a failure, or a successful/failed response is received.
                    break;

                default:
                    // Should not be in this state! 
                    // Unexpected response in this state - revert
                    // to the init'ing state until it clears.
                    SetATCmdStateInit();
                    CleanUpOnUnexpectedRsp( atCmdState );
                    modemOptions.modemState = MODEM_INITTING;
                    break;
            }

            break;

        case MODEM_POWERED_DOWN:
            // In this state, the modem is not yet powered up.
            // Check if we are on-line and go to initting mode.
            // Otherwise, stay here.

            switch( atCmdState )
            {
                case AT_CMD_INITTING:
                    // The modem is ready to exit power down mode.
                    // go to initing.
                    RecordModemLogError( MODEMLOG_MODEM_IS_POWERED );
                    modemOptions.modemState = MODEM_INITTING;
                    break;

                case AT_CMD_POWERED_DOWN:

                    // Still powered down, wait here until we're back up.
                    HandleQueuedCommands();
                    StopTimer( thCheckCSQ );
                    StopTimer( thCheckRetryDelay );
                    StopTimer( thCheckGateway );
                    StopTimer( thCheckCallStatus );
                    ResetTimer( thTimeout, modemConfigurables.dwTimeoutDelay );                    

                    break;

                case AT_CMD_TIMED_OUT:
                    // This may occur if the CIS command timed out. Handle this gracefully.
                    modemOptions.modemState = modemOptions.prevModemState;

                    if( modemOptions.modemState != MODEM_IDLE )
                    {
                        SetATCmdStateInit();
                    }

                    break;

                default:
                    // All these responses are unexpected in this state - revert
                    // to the init'ing state until they clear.
                    SetATCmdStateInit();
                    CleanUpOnUnexpectedRsp( atCmdState );
                    modemOptions.modemState = MODEM_INITTING;
                    break;
            }

            break;

        default:
            break;

    }  // switch( current modem state )
}


//------------------------------------------------------------------------------
//  PRIVATE FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: HandleQueuedCommands
//
//  Arguments: void.
//
//  Returns: TRUE if there is a queued command waiting.
//           FALSE otherwise.
//
//  Description: This function initiates the CIS programming port commands
//               sent by anywhere in the system or ELA.
//
//******************************************************************************
BOOL HandleQueuedCommands( void )
{
    MODEM_COMMANDS mdmCmds = NO_CMD;
    WORD wBackupReadIndex;

    // First check if we have something q'd up
    if( QueuedCISCmd.wWriteIndex == QueuedCISCmd.wReadIndex )
    {
        return FALSE;
    }

    // Since we verify the queue for duplicate commands,
    // be sure to clear the command just read, to allow for 
    // the command to be posted in the future.
    wBackupReadIndex = QueuedCISCmd.wReadIndex;
    mdmCmds = (MODEM_COMMANDS)GetDataFromQueue( &QueuedCISCmd );
    modemQBuff[wBackupReadIndex] = NO_CMD;

//    print( " q'd cmd: " );
//    output_int( mdmCmds );

    switch( mdmCmds )
    {
        case RINGER_ON:
            TogglePhoneRinger( TRUE );
            break;

        case RINGER_OFF:
            TogglePhoneRinger( FALSE );
            break;

        case RELAY1_ON:
            ToggleRelayState( RELAY_1, TRUE );
            break;

        case RELAY1_OFF:
            ToggleRelayState( RELAY_1, FALSE );
            break;

        case RELAY2_ON:
            ToggleRelayState( RELAY_2, TRUE );
            break;

        case RELAY2_OFF:
            ToggleRelayState( RELAY_2, FALSE );
            break;

        case RINGER_STATUS:
            SendPhoneRingerStatusCmd();
            break;

        case RELAY1_STATUS:
            SendOutputRelayStatusCmd( RELAY_1 );
            break;

        case RELAY2_STATUS:
            SendOutputRelayStatusCmd( RELAY_2 );
            break;

        case RESET_CIS:
            ResetCIS();
            break;

        case CONFIGURE_CIS:
            ProgramCIS();
            break;

        case UPLOAD_CIS_CONFIG:
            UploadCISConfig();
            break;

        case CALL_HANGUP:
            // This command can only be sent if we're powered up.
            if( modemOptions.modemState != MODEM_POWERED_DOWN )
            {
                HangupCall();
                break;
            }

            // else fall through.

        case NO_CMD:
        default:
            return FALSE;

    }

    return TRUE;
}


//******************************************************************************
//
//  Function: GetMailboxStatus
//
//  Arguments: void.
//
//  Returns: TRUE if there is a message ready to receive from the modem.
//           FALSE otherwise.
//
//  Description: This function initiates the SendReadBinaryFileCmd() command,
//               if there is a message to be received.
//
//******************************************************************************
BOOL GetMailboxStatus( void )
{
    if( GetSBDStatus() == SUCCESSFUL_MSG )
    {
        // We have a message pending; 
        // download it from the modem buffer,
        if( SendReadBinaryFileCmd() )
        {
            // it's possible for this function to be called from an interrupt,
            // however, a download must be performed before this function,
            // and sending is only enabled after we're out of interrupt.
            // Besides, this function writes the MT msg to a file, so,
            // we'll be writing to a file regardless.
            ModemLog( NO_RPT, MODEMLOG_RECEIVE );
            SetModemStateBusy( RXING_FILE );
            return TRUE;
        }
    }

    // Fall through here means there are no pending messages to receive.
    return FALSE;
}


//******************************************************************************
//
//  Function: SendFileToModem
//
//  Arguments: void.
//
//  Returns: NOT_SENDING if there are no files to send 
//           WAITING_TO_SEND if retry delay timer has yet to expire
//           SENDING_FILE if the file was sent
//
//  Description: This function copies each byte of a specified summary report 
//               into a buffer and sends that buffer to the satellite 
//               modem port. 
//
//******************************************************************************
FILE_SEND_OPTIONS SendFileToModem( void )
{
    // Nothing to send or there is no card...let's double check
    if( modemOptions.bPCMCIAError )
    {
        // Notify lower level that a buffer needs to be sent,
        // instead of a file
        if( SendBinaryBuffer( (BYTE*)CreateSystemLogBuffer( 0 ),
                              SYSLOG_MSG_SIZE ) )
        {
            SetModemStateBusy( TXING_BUFFER );
            modemOptions.bPCMCIAError = FALSE;
            return SENDING_FILE;
        }
    }

    // Is this a retry or not? Retry count is bumped up on a failure to transmit.
    if( modemFlags.byFileSendRetryCount == 0 )
    {
        // Get the next file in the queue (have the all-filter
        // turned on automatically)
        modemOptions.szPathFileBeingSent[0] = NULL;

        if( SortAscending( modemOptions.szPathFileBeingSent, GetPCMCIAPath( MODEM_DIR, OUTBOX_SUBDIR ) ) == NULL )
        {
            return NOT_SENDING;
        }

        // If the file was successfully sent, log it and change states.
        ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_SEND );
    }
    else
    {
        if( !TimerExpired( thCheckRetryDelay ) )
        {
            // Cannot resend yet.
            return WAITING_TO_SEND;
        }

        StopTimer( thCheckRetryDelay );
        ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_RETRY_SEND );
    }

    if( SendBinaryFile( modemOptions.szPathFileBeingSent ) )
    {
        SetModemStateBusy( TXING_FILE );

        return SENDING_FILE;
    }
    else
    {
        // Something is wrong with the file! Delete it to keep ELA from resending it over, and over and over....
        if( deleteFile( modemOptions.szPathFileBeingSent ) )
        {
            // Ensure the file being deleted is logged
            StringCpy( szErrString, modemOptions.szPathFileBeingSent );
            StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_DELETED ), MAX_SYSTEM_LOG_STR );
            SystemLog( szErrString );
        }
        else
        {
            // Report if file cannot be deleted.
            StringCpy( szErrString, modemOptions.szPathFileBeingSent );
            StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_DELETED ), MAX_SYSTEM_LOG_STR );
            SystemLog( szErrString );
            MarkFileAsSent( MODEM_DIR, modemOptions.szPathFileBeingSent );
        }
    }

    // Fall through means there were problems sending the file!
    return NOT_SENDING;
}


//******************************************************************************
//
//  Function: SendCallStatusCmd
//
//  Arguments: void.
//
//  Returns: TRUE if the command was sent
//           FALSE if the command was not sent 
//                 if dialing timer has yet to expire
//
//  Description: This function checks on the modem status if we have detected
//               an off-hook event or a "ring" (incoming call).
//
//******************************************************************************
BOOL SendCallStatusCmd( void )
{
    // Is this a retry or not? Retry count is bumped up on a failure,
    // and set to 1 if the modem is busy dialing.
    if( !TimerExpired( thCheckCallStatus ) )
    {
        return FALSE;
    }

    if( SendCLCCCmd() )
    {
        print( " sending CLCC cmd" );
        SetModemStateBusy( CALL_STATUS );
        ResetTimer( thCheckCallStatus, DEFAULT_SBD_STATUS_DELAY );
        return TRUE;
    }

    // Fall through means there were problems sending the cmd!
    return FALSE;
}


//******************************************************************************
//
//  Function: CleanUpOnUnexpectedRsp
//
//  Arguments:
//    atCmdState - AT_CMD_STATES enum type to save to the modem log file.
//
//  Returns: void.
//
//  Description: Reports in the modem.log file that an unexpected result
//               occured.
//
//******************************************************************************
void CleanUpOnUnexpectedRsp( AT_CMD_STATES atCmdState )
{
    RecordModemLogError( MODEMLOG_UNEXPECTED_RSP );

    print( "\r\n" );
    print( TEXT_MODEM_STATE[modemOptions.modemState] );
    print( GetATCmdText( atCmdState ) );
    print( " Unexpected response - going to init!" );
}


//******************************************************************************
//
//  Function: CleanUpOnIdle
//
//  Arguments: void
//
//  Returns: void.
//
//  Description: Takes care of risidual tasks to be handled by upper layer
//               (like notifying modem log, moving files to the correct
//                directory, LEDs, setting/unsetting flags and handling
//                retries).
//
//******************************************************************************
void CleanUpOnIdle( AT_CMD_STATES atCmdState )
{
    static char szFileName[MAX_FILENAME_LEN];
    BYTE byIndex;
    char cPriorityFlag;
    BOOL bKeepFile;

    if( atCmdState == AT_CMD_SUCCESS )
    {
        modemOptions.ModemRsp[modemOptions.ModemCmd] = MR_SUCCESS;
    }
    else
    {
        modemOptions.ModemRsp[modemOptions.ModemCmd] = MR_FAILED;
    }

    HandleTimeouts( atCmdState );

    //print( " -->" );
    //output_int( modemOptions.ModemCmd );
    //print( " " );
    //output_int( modemOptions.ModemRsp[modemOptions.ModemCmd] );

    switch( modemOptions.ModemCmd )
    {
        case RXING_FILE:

            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = MODEM_IDLE;

            switch( atCmdState )
            {
                case AT_CMD_TIMED_OUT:
                    // At least notify the user what happened.
                    RecordModemLogError( MODEMLOG_RECEIVE_FAILURE );

                    // Try reading the buffer again...
                    if( ++modemFlags.byFileReceiveRetryCount < modemConfigurables.byMaxRetries )
                    {
                        // We have a message pending; 
                        // download it from the modem buffer,
                        if( SendReadBinaryFileCmd() )
                        {
                            // no longer in idle!
                            ModemLog( NO_RPT, MODEMLOG_RECEIVE );
                            SetModemStateBusy( RXING_FILE );
                        }

                        break;
                    }

                    // else fall through..

                default:
                case AT_CMD_FAILED:
                case AT_CMD_SUCCESS:

                    // Wait for a bit before continuing.
                    WaitForIncommingCalls();
                    modemFlags.byFileReceiveRetryCount = 0;

                    break;
            }

            break;

        case TXING_FILE:

            switch( atCmdState )
            {
                case AT_CMD_SUCCESS:
                    // Reset retry count on success.
                    modemFlags.byFileSendRetryCount = 0;
                    modemOptions.ModemCmd = NO_CMD;

                    // Ensure the file being deleted is logged
                    ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_SEND_SUCCESSFUL );

                    if( modemConfigurables.szKeepFileList[0] == DELETE_ALL_FILES )
                    {
                        if( !deleteFile( modemOptions.szPathFileBeingSent ) )
                        {
                            // Report if file cannot be deleted.
                            ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_DELETE_FAILURE );
                            MarkFileAsSent( MODEM_DIR, modemOptions.szPathFileBeingSent );
                        }
                    }
                    else
                    {
                        szFileName[0] = NULL;
                        bKeepFile = FALSE;
                        cPriorityFlag = ExtractFileNameFromPath( modemOptions.szPathFileBeingSent, szFileName )[0];

                        for( byIndex = 0; byIndex < MAX_PRIORITY_FLAGS; byIndex++ )
                        {
                            // Look at the "keep" list
                            if( modemConfigurables.szKeepFileList[byIndex] == NULL )
                            {
                                break;
                            }
                            else if( modemConfigurables.szKeepFileList[byIndex] == cPriorityFlag )
                            {
                                bKeepFile = TRUE;
                                break;
                            }
                        }

                        if( bKeepFile || ( modemConfigurables.szKeepFileList[0] == KEEP_ALL_FILES ) )
                        {
                            if( !MarkFileAsSent( MODEM_DIR, modemOptions.szPathFileBeingSent ) )
                            {
                                ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_MOVE_FAILURE );

                                if( deleteFile( modemOptions.szPathFileBeingSent ) )
                                {
                                    // Ensure the file being deleted is logged
                                    StringCpy( szErrString, modemOptions.szPathFileBeingSent );
                                    StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_DELETED ), MAX_SYSTEM_LOG_STR );
                                    SystemLog( szErrString );
                                }
                                else
                                {
                                    // Report if file cannot be deleted.
                                    StringCpy( szErrString, modemOptions.szPathFileBeingSent );
                                    StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_DELETED ), MAX_SYSTEM_LOG_STR );
                                    SystemLog( szErrString );
                                }
                            }
                        }
                        else if( !deleteFile( modemOptions.szPathFileBeingSent ) )
                        {
                            // Report if file cannot be deleted.
                            ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_DELETE_FAILURE );
                            MarkFileAsSent( MODEM_DIR, modemOptions.szPathFileBeingSent );
                        }
                    }

                    if( InVoiceCall() ) // true (high) if phone is off hook
                    {
                        // Function call changes modemState and ModemCmd
                        modemOptions.modemState = MODEM_IDLE;
                        HangupCall();
                        break;
                    }

                    // Only wait if there is nothing in the MT buffer
                    if( !GetMailboxStatus() )
                    {
                        // Wait for a bit before continuing.
                        WaitForIncommingCalls();
                        modemOptions.modemState = MODEM_IDLE;
                    }

                    break;

                case AT_CMD_TIMED_OUT:
                case AT_CMD_FAILED:
                default:
                    // Increment the retry count on failure. Disable txingFile flag, as we 
                    // don't know what will happen in between.
                    modemOptions.modemState = MODEM_IDLE;
                    modemOptions.ModemCmd   = NO_CMD;

                    if( ++modemFlags.byFileSendRetryCount < modemConfigurables.byMaxRetries )
                    {
                        // Setup delay timer. 
                        StartTimer( thCheckRetryDelay, modemConfigurables.dwRetryDelay );
                    }
                    else
                    {
                        // Reset retry count for next file to be sent and go into wait.
                        modemFlags.byFileSendRetryCount = 0;
                        WaitForIncommingCalls();
                        
                        if( MarkFileAsError( MODEM_DIR, modemOptions.szPathFileBeingSent ) )
                        {
                            ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_SEND_FAILURE );
                        }
                        else
                        {
                            ModemLog( modemOptions.szPathFileBeingSent, MODEMLOG_MOVE_FAILURE );

                            if( deleteFile( modemOptions.szPathFileBeingSent ) )
                            {
                                // Ensure the file being deleted is logged
                                StringCpy( szErrString, modemOptions.szPathFileBeingSent );
                                StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_DELETED ), MAX_SYSTEM_LOG_STR );
                                SystemLog( szErrString );
                            }
                            else
                            {
                                // Report if file cannot be deleted.
                                StringCpy( szErrString, modemOptions.szPathFileBeingSent );
                                StringNCat( szErrString, GetSysLogMsg( SYS_LOG_FILE_CANNOT_BE_DELETED ), MAX_SYSTEM_LOG_STR );
                                SystemLog( szErrString );
                            }
                        }
                    }

                    break;
            }

            break;

        case TXING_BUFFER:
        case TXING_TEXT:

            modemOptions.ModemCmd = NO_CMD;

            if( atCmdState == AT_CMD_SUCCESS )
            {
                if( InVoiceCall() ) // true (high) if phone is off hook
                {
                    // Function call changes modemState and ModemCmd
                    modemOptions.modemState = MODEM_IDLE;
                    HangupCall();
                    break;
                }
            }

            // Only wait if there is nothing in the MT buffer
            if( !GetMailboxStatus() )
            {
                // Wait for a bit before continuing.
                WaitForIncommingCalls();
                modemOptions.modemState = MODEM_IDLE;
            }
            
            break;

        case CALL_STATUS:

            switch( atCmdState )
            {
                case AT_CMD_SUCCESS:
                case AT_CMD_TIMED_OUT:
                default:

                    // Reset retry count on success.
                    modemOptions.ModemCmd   = NO_CMD;
                    modemOptions.modemState = MODEM_IDLE;

                    // Let the phone call complete
                    WaitForIncommingCalls();

                    break;
            }

            break;

        case MAILBOX_CHECK:

            modemOptions.ModemCmd = NO_CMD;

            if( atCmdState == AT_CMD_SUCCESS )
            {
                // No need to reset timer, its done in the state machine.
                RecordModemLogError( MODEMLOG_MAILBOXCHECK_SUCCESS );
            }
            else
            {
                // No need to reset timer, its done in the state machine.
                RecordModemLogError( MODEMLOG_MAILBOXCHECK_FAILURE );
            }

            // Only wait if there is nothing in the MT buffer
            if( !GetMailboxStatus() )
            {
                // Wait for a bit before continuing.
                WaitForIncommingCalls();
                modemOptions.modemState = MODEM_IDLE;
            }

            break;

        case GATEWAY_CHECK:

            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = MODEM_IDLE;

            if( atCmdState == AT_CMD_SUCCESS )
            {
                // Do a manual mailbox check - msgs are queueued or waiting at the gateway
                // We know the modem is idle, the command will go through
                if( CheckMailbox() )
                {
                    SetModemStateBusy( MAILBOX_CHECK );
                }
            }

            break;

        case GETTING_CSQ:

            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = MODEM_IDLE;

            if( atCmdState == AT_CMD_SUCCESS )
            {
                modemFlags.byCSQDebounceCount = 0;
            }
            else
            {
                if( ++modemFlags.byCSQDebounceCount < modemConfigurables.byCSQMaxDebounceRetries )
                {
                    // On failure, keep trying to get a decent signal strength.
                    ResetTimer( thCheckCSQ, modemConfigurables.dwCSQDebounceDelay );
                }
                else
                {
                    // Ok, it's definately a failure, report it.
                    modemFlags.byCSQDebounceCount = 0;
                    ClearModemSignalStrength();
            
                    RecordModemLogError( MODEMLOG_SIGNALSTRENGTH_FAILURE );
                    ReportSystemLogError( SYS_LOG_IRIDIUM_ERROR );
                }
            }

            break;

        case CALL_HANGUP:

            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = MODEM_IDLE;

            // Wait for a bit before continuing.
            WaitForIncommingCalls();

            if( atCmdState == AT_CMD_SUCCESS )
            {
                RecordModemLogError( MODEMLOG_HUNG_UP_CALL_SUCCESS );
            }
            else
            {
                RecordModemLogError( MODEMLOG_HUNG_UP_CALL_FAILURE );
            }
        
            break;

            // These set of CIS commands are handled in a special way.
            // First we retry commands if they've failed, and 
            // secondly since they can be deployed from outside of idle,
            // we need to know from where to return.
        case RINGER_ON :
        case RINGER_OFF:
        case RELAY1_ON :
        case RELAY1_OFF:
        case RELAY2_ON :
        case RELAY2_OFF:
        case RINGER_STATUS:
        case RELAY1_STATUS:
        case RELAY2_STATUS:
        case RESET_CIS:

            if( atCmdState != AT_CMD_SUCCESS )
            {
                // Fall through means we have to resend the message.
                AddNewDataToQueue( modemOptions.ModemCmd );  // Save data
            }

            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = modemOptions.prevModemState;

            if( modemOptions.modemState != MODEM_IDLE )
            {
                SetATCmdStateInit();
            }

            break;

        case UPLOAD_CIS_CONFIG:

            // Regardless, continue.
            print( " done!" );
            modemOptions.bCISActionComplete = TRUE;
            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = modemOptions.prevModemState;

            if( modemOptions.modemState != MODEM_IDLE )
            {
                SetATCmdStateInit();
            }

            break;

        case CONFIGURE_CIS:

            if( atCmdState != AT_CMD_SUCCESS )
            {
                // On failure, we have to:
                // 1. Write bogus data to the CIS area of the EEPROM (to allow for recovery)
                // 2. Reset the CIS.
                // CANNOT REPORT ERROR AS THIS IS RUNNING FROM INTERRUPT!
                ReportSystemLogError( SYS_LOG_REMOTE_CONFIG_FAILED_CISCFG );
                print( " failed programming CIS!" );
                WriteConfigEepromByte( CIS_PRIMARY_EXT, INVALID_EXTENSION );
                WriteConfigEepromByte( CIS_SECONDARY_EXT, INVALID_EXTENSION );
            }
#if (DEBUG)
            else
            {
                print( " succeeded programming CIS!" );
            }
#endif

            modemOptions.bCISActionComplete = TRUE;
            modemOptions.ModemCmd = NO_CMD;
            modemOptions.modemState = modemOptions.prevModemState;

            if( modemOptions.modemState != MODEM_IDLE )
            {
                SetATCmdStateInit();
            }

            // We have to:
            // 1. Reset the CIS.
            // 2. Restore the states of the ringer and relays
            PowerCycleCIS();

            break;

        default:
            break;
    }
}


//******************************************************************************
//
//  Function: WaitForIncommingCalls
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Provides a wait state for the state machine to allow
//               incomming calls.
//
//******************************************************************************
void WaitForIncommingCalls( void )
{
    // Only reenable sending (after a delay), if it already is enabled.
    if( modemOptions.bSendingEnabled )
    {
        modemOptions.bSendingEnabled = FALSE;
        StartTimer( thWaitForCalls, modemConfigurables.dwWaitForCalls );
    }
}


//******************************************************************************
//
//  Function: AddNewDataToQueue
//
//  Arguments:
//    IN Modemcommand - Command to queue up.
//
//  Returns: void.
//
//  Description: Adds new command to queue, but only if it doesn't already 
//               exist in queue.
//
//******************************************************************************
void AddNewDataToQueue( MODEM_COMMANDS Modemcommand )
{
    BYTE byIndex;

//    print( " q'd cmd: " );
//    output_int( Modemcommand );

    for( byIndex = 0; byIndex < MDM_Q_LEN; byIndex++ )
    {
        if( modemQBuff[byIndex] == Modemcommand )
        {
            return;
        }
    }

    AddDataToQueue( &QueuedCISCmd, Modemcommand );  // Save data and clear response
    modemOptions.ModemRsp[Modemcommand] = MR_WAITING;
}


//******************************************************************************
//
//  Function: SetModemStateBusy
//
//  Arguments:
//    IN  cmd     - Specifies the command that is making the statemachine busy.
//
//  Returns: void.
//
//  Description: Sets all the correct variables when we go into busy state.
//
//******************************************************************************
void SetModemStateBusy( MODEM_COMMANDS cmd )
{
    modemOptions.prevModemState   = modemOptions.modemState;
    modemOptions.modemState       = MODEM_BUSY;
    modemOptions.ModemCmd         = cmd;
    modemOptions.ModemRsp[modemOptions.ModemCmd] = MR_WAITING;
}


//******************************************************************************
//
//  Function: HandleTimeouts
//
//  Arguments:
//    IN  ATCmdState - current middle-level state
//
//  Returns: void.
//
//  Description: Ensures back-to-back timeouts are handled consistently.
//
//******************************************************************************
void HandleTimeouts( AT_CMD_STATES ATCmdState )
{
    if( ATCmdState == AT_CMD_TIMED_OUT )
    {
        if( TimerExpired( thTimeout ) )
        {
            if( !InVoiceCall() )
            {
                SystemLog( "Modem communications error detected - power cycling CIS" );

                if( !PowerCycleCIS() )
                {
                    AddNewDataToQueue( RESET_CIS );  // Save data
                }
            }

            ResetTimer( thTimeout, modemConfigurables.dwTimeoutDelay );
        }
    }
    else
    {
        ResetTimer( thTimeout, modemConfigurables.dwTimeoutDelay );
    }
}


//******************************************************************************
//
//  Function: FunctName
//
//  Arguments:
//    IN  wMisc     - data used for something
//    OUT byName    - pointer to name
//
//  Returns: what FunctName returns
//
//  Description: Function description
//
//******************************************************************************
