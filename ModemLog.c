//******************************************************************************
//
//  ModemLog.c: Module Title
//
//      Copyright (c) 2001-2010, Aeromechanical Services Ltd.
//      ALL RIGHTS RESERVED
//
//  This module provides interfaces for logging the status of a 
//  summary report transfer.
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
        #include "Stubfunctions.h"
    #endif
#else
    #include "ModemLog.h"
    #include "Modem.h"
    #include "ints.h"
    #include "pcmcia.h"
    #include "pcmciaDirs.h"
    #include "pcmciaAPI.h"
    #include "Queue.h"
    #include "utils.h"
    #include "RulesBin.h"
    #include "GpsPort.h"
    #include "MtcePort.h"
    #include "FileUtils.h"
    #include "FileTransfer.h"
    #include "SystemLog.h"
#endif

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


#define MAX_SIG_STRENGTH_LOG_MSG    10
#define MAX_MODEM_LOG_MSG           35
#define BUFFERED_DATA_SIZE          15
#define MAX_EXTENDED_LOG_MSG        55
#define RECORD_Q_LEN                MODEMLOG_NBR_CODES

#define MODEMLOG_STRUCT_SIZE        sizeof( MODEMLOG_STRUCT )
#define MODEMLOG_MSG_SIZE           sizeof( MODEMLOG_ERR_FILE )

// MAX_MODEM_LOG_STR is the longest message that can be output by the
// modem log. It should be: GPS time size + signal strength + EMAXPATH + modem log msg + extended modem error codes.
#define MAX_MODEM_LOG_STR           ( MAX_DATE_TIME_STR + MAX_SIG_STRENGTH_LOG_MSG + EMAXPATH + MAX_MODEM_LOG_MSG + MAX_EXTENDED_LOG_MSG + 1 /*null term*/ )

#define HAVE_RECORD_DATA()          ( RecordQueue.wWriteIndex != RecordQueue.wReadIndex )

#define NEXT_INDEX( aIndex )        ( ( aIndex + 1 >= BUFFERED_DATA_SIZE ) ? 0 : aIndex + 1 )


typedef struct
{
    DWORD dwDateTimeStamp;
    BYTE  byModemLogErrEnum;
    BYTE  byFrequency;

} MODEMLOG_STRUCT;


typedef struct
{
    RPT_HEADER_STRUCT header;
    MODEMLOG_STRUCT bufferedData[BUFFERED_DATA_SIZE];
    DWORD dwTimeAtStart;
} MODEMLOG_ERR_FILE;


typedef union
{
    MODEMLOG_ERR_FILE modemlogErrFile;
    BYTE     pbyData[MODEMLOG_MSG_SIZE];

} U_MODEMLOG_ERROR_MSG;


//------------------------------------------------------------------------------
//  GLOBAL DECLARATIONS
//------------------------------------------------------------------------------


static BOOL bDisplayModemLogErrors;

static MODEMLOG_STRUCT bufferedErrs[BUFFERED_DATA_SIZE];
static WORD           wBufferedIndex;

static COMMQUEUE      RecordQueue;
static PCOMMQUEUE     pRecordQueue = &RecordQueue;
static QUEUE_BUFF     byRecordQBuff[RECORD_Q_LEN];


static char TEXT_MODEMLOG_ERR_CODE[MODEMLOG_NBR_CODES][MAX_MODEM_LOG_MSG] = 
{
// Do not exceed message length:       "***********************************"
 /* MODEMLOG_START_CODES            */ "No error",
 /* MODEMLOG_MODEM_IS_POWERED       */ " modem is powered up",
 /* MODEMLOG_MODEM_POWERED_DOWN     */ " modem is powered down",
 /* MODEMLOG_SEND_SUCCESSFUL        */ " file sent successfully",
 /* MODEMLOG_SEND_ENABLED           */ " transmission enabled",
 /* MODEMLOG_SEND_FAILURE           */ " failed to send file",
 /* MODEMLOG_SEND_DISABLED          */ " transmission disabled!",
 /* MODEMLOG_SEND                   */ " start sending report",
 /* MODEMLOG_RETRY_SEND             */ " resending file",  
 /* MODEMLOG_RECEIVE                */ " start receiving file",
 /* MODEMLOG_RECEIVE_SUCCESSFUL     */ " received successfully",
 /* MODEMLOG_RECEIVE_FAILURE        */ " failed to receive file",
 /* MODEMLOG_UNEXPECTED_RSP         */ " unexpected response from modem",
 /* MODEMLOG_MOVE_FAILURE           */ " failed to move file from folder",
 /* MODEMLOG_DELETE_FAILURE         */ " failed deleting file",
 /* MODEMLOG_COPY_SUCCESS           */ " file was copied successfully",
 /* MODEMLOG_COPY_FAILURE           */ " failed copying file",
 /* MODEMLOG_MAILBOXCHECK_SUCCESS   */ " mailbox check successful",
 /* MODEMLOG_MAILBOXCHECK_FAILURE   */ " failed mailbox check",
 /* MODEMLOG_SIGNALSTRENGTH_FAILURE */ " failed getting Iridium signal",
 /* MODEMLOG_HUNG_UP_CALL_SUCCESS   */ " successfully hung up call",
 /* MODEMLOG_HUNG_UP_CALL_FAILURE   */ " failed to hang up call",
 /* MODEMLOG_PHONE_OFF_HOOK         */ " phone is off-hook",
 /* MODEMLOG_PHONE_BACK_ON_HOOK     */ " phone is back on-hook",
 /* MODEMLOG_INCOMING_CALL          */ " incoming call",
 /* MODEMLOG_INCOMING_CALL_COMPLETE */ " incoming call complete",
 /* MODEMLOG_MUTE_BTN_PRESSED       */ " Mute button pressed.",
 /* MODEMLOG_MUTE_BTN_RELEASED      */ " Mute button released.",
};


static char TEXT_ERROR_CODE_RSP[NBR_ERR_CODES][MAX_EXTENDED_LOG_MSG] =
{
 // Do not exceed message length:    "*******************************************************"
 /* MEC_NONE */                      "",
 /* MEC_ERROR */                     " - AT cmd failed.",
 /* MEC_HW_ERROR */                  " - Hardware error!",
 /* MEC_RX_BUFFER_OVERFLOW */        " - rx buffer overflowed.",
 /* MEC_RSP_TIMED_OUT */             " - timed out.",
 /* MEC_TX_BIN_DATA_TIMEOUT */       " - transmission timed out.",
 /* MEC_TX_BIN_DATA_BAD_CHECKSUM */  " - data transmitted was corrupt.",
 /* MEC_TX_BIN_DATA_BAD_SIZE */      " - incorrect size of data.",
 /* MEC_SBDI_GSS_TIMEOUT, */         " - GSS reported session did not complete in time.",
 /* MEC_SBDI_GSS_Q_FULL, */          " - MO message queue at the GSS is full.",
 /* MEC_SBDI_MO_SEGMENT_ERR, */      " - MO message has too many segments.",
 /* MEC_SBDI_INCOMPLETE_SESSION */   " - GSS reported that the session did not complete.",
 /* MEC_SBDI_SEGMENT_SIZE_ERR, */    " - invalid segment size.",
 /* MEC_SBDI_GSS_ACCESS_DENIED, */   " - access denied.",
 /* MEC_SBDI_SBD_BLOCKED, */         " - ISU is locked by Gateway, cannot make SBD calls!",
 /* MEC_SBDI_ISU_TIMEOUT, */         " - gateway not responding (local session timeout).",
 /* MEC_SBDI_RF_DROP, */             " - connection lost (RF drop).",
 /* MEC_SBDI_PROTOCOL_ERR, */        " - link failure (protocol error terminated call).",
 /* MEC_SBDI_NO_NETWORK_SERVICE */   " - no network service, unable to initiate call.",
 /* MEC_SBDI_ISU_BUSY, */            " - ISU is busy; unable to initiate call.",
 /* MEC_SBDI_FAIL, */                " - SBD session failure.",
 /* MEC_CLEAR_MODEM_BUFFER_ERROR */  " - could not clear modem buffer.",
 /* MEC_FILE_OPEN_ERR */             " - file open error." ,
 /* MEC_FILE_READ_ERR */             " - file read error." ,
 /* MEC_FILE_WRITE_ERR */            " - file write error." ,
 /* MEC_TRUNCATED_FILE */            " - file was truncated.",
 /* MEC_SBDS_SUCESS */               " - successfully checked mailbox status.",
 /* MEC_SBDS_NO_TX_MSG */            " - no messages waiting to be sent.",
 /* MEC_SBDS_TX_MSG_PENDING */       " - messages waiting to be sent.",
 /* MEC_SBDS_NO_RX_MSG */            " - no messages waiting to be received",
 /* MEC_SBDS_RX_MSG_PENDING */       " - messages waiting to be received.",
 /* MEC_CREG_NOT_REGISTERED */       " - not registered but searching (0)",
 /* MEC_CREG_REGISTERED_HOME */      " - registered on home network",
 /* MEC_CREG_SEARCHING */            " - not registered but searching (2)",
 /* MEC_CREG_DENIED */               " - registration denied",
 /* MEC_CREG_UNKNOWN */              " - not registered (bad antenna connection)",
 /* MEC_CREG_REGISTERED_ROAMING */   " - registered and roaming",
 /* MEC_CSQ_ERROR */                 " - error getting signal strength",
 /* MEC_ACTIVE_CALL_STATUS */        " - modem active in a voice/data call",
 /* MEC_HELD_CALL_STATUS */          " - modem on hold for a voice call",
 /* MEC_DIALING_CALL_STATUS */       " - handset is dialing",
 /* MEC_INCOMING_CALL_STATUS */      " - handset is ringing",
 /* MEC_WAITING_CALL_STATUS */       " - voice call waiting",
 /* MEC_IDLE_CALL_STATUS */          " - modem is idle",
 /* MEC_RX_NO_MSG_WAITING */         " - no message waiting to be received from buffer.",
 /* MEC_RX_BAD_CHECKSUM */           " - data received was corrupt.",
 /* MEC_RX_BAD_FILELENGTH */         " - size of received data was incorrect.",
 /* MEC_MODEM_POWERED_DOWN */        " - modem is powered down.",
 /* MEC_CIS_RINGER_OFF */            " - ringers turned off.",
 /* MEC_CIS_RINGER_ON */             " - ringers turned on.",
 /* MEC_CIS_RELAY1_OFF */            " - relay 1 is off.",
 /* MEC_CIS_RELAY1_ON */             " - relay 1 is on.",
 /* MEC_CIS_RELAY2_OFF */            " - relay 2 is off.",
 /* MEC_CIS_RELAY2_ON */             " - relay 2 is on.",

};


//------------------------------------------------------------------------------
//  PRIVATE FUNCTION PROTOTYPES
//------------------------------------------------------------------------------


static void WriteMdmLogFile( char* szStr );


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: ModemLogInit
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This function must be the first to be called from 
//               this module in order to set local variables.  
//
//******************************************************************************
void ModemLogInit( void )
{
    bDisplayModemLogErrors = FALSE;
    wBufferedIndex = BUFFERED_DATA_SIZE - 1;

    InitQueue( pRecordQueue, byRecordQBuff, RECORD_Q_LEN );
    MemSet( bufferedErrs, 0, MODEMLOG_STRUCT_SIZE * BUFFERED_DATA_SIZE );

    WriteMdmLogFile( GetLogFileHeader() );
}


//******************************************************************************
//
//  Function: ModemLog
//
//  Arguments:
//    IN  szFileName - A buffer containing the summary report
//                     filename that is being logged.
//    IN  errCode    - Enum value that describes the status of the modem.
//
//  Returns: void.
//
//  Description: Call this function to log a message regarding the status 
//               of a summary report transfer in the Modem.log file.  
//               The information logged includes a GPS time stamp, a 
//               summary report filename and a brief message describing 
//               the status of the summary report transfer.
//
//******************************************************************************
void ModemLog( const char* szFileName, MODEMLOG_ERR_CODE errCode )
{
    // NOTE: we must open the file, log error and close file.  The reason
    // that we need to close file immediately after logging because there
    // exists a risk that somebody comes along and unplug the pcmcia flash
    // card or the power to the system is lost.
    static char  szModemLogStr[MAX_MODEM_LOG_STR];
    static char  szSigLevel[MAX_SIG_STRENGTH_LOG_MSG];
    MODEM_ERROR_CODE_RSP subErrorCode = GetErrorCodeRsp();
    BYTE byIndex;
    BOOL bErrCodeExists = FALSE;

    if( errCode >= MODEMLOG_NBR_CODES )
    {
        // zj consider logging actual value in log file instead of reallocating.
        errCode = MODEMLOG_START_CODES;
    }

    if( subErrorCode >= NBR_ERR_CODES )
    {
        subErrorCode = MEC_NONE;
    }

    for( byIndex = 0; byIndex < BUFFERED_DATA_SIZE; byIndex++ )
    {
        if( bufferedErrs[byIndex].byModemLogErrEnum == errCode )
        {
            bufferedErrs[byIndex].byFrequency++;
            bufferedErrs[byIndex].dwDateTimeStamp = GetGpsTime();
            bErrCodeExists = TRUE;
            break;
        }
    }

    if( !bErrCodeExists )
    {
        // Pre increment, to keep the index always current
        wBufferedIndex = NEXT_INDEX( wBufferedIndex );
        bufferedErrs[wBufferedIndex].byModemLogErrEnum = errCode;
        bufferedErrs[wBufferedIndex].byFrequency = 1;
        bufferedErrs[wBufferedIndex].dwDateTimeStamp = GetGpsTime();
    }

    // Compose modem log string. NOTE! NOTE! NOTE! If you add more elements to
    // this string, be sure to review the definition of MAX_MODEM_LOG_STR above!
    GetCurrentDateTimeStr( szModemLogStr );
    IntToString( szSigLevel, GetModemSignalStrength(), 1 );

    StringCat( szModemLogStr, " (" );
    StringCat( szModemLogStr, szSigLevel );
    StringCat( szModemLogStr, "): " );

    StringNCat( szModemLogStr, szFileName, MAX_MODEM_LOG_STR );
    StringNCat( szModemLogStr, TEXT_MODEMLOG_ERR_CODE[errCode], MAX_MODEM_LOG_STR );
    StringNCat( szModemLogStr, TEXT_ERROR_CODE_RSP[subErrorCode], MAX_MODEM_LOG_STR );

    if( errCode == MODEMLOG_SEND_SUCCESSFUL )
    {
        // add MOMSN
        StringNCat( szModemLogStr, " MOMSN: ", MAX_MODEM_LOG_STR );
        StringNCat( szModemLogStr, GetMOMSN(), MAX_MODEM_LOG_STR );
    }

    else if( errCode == MODEMLOG_RECEIVE_SUCCESSFUL )
    {
        // add MTMSN
        StringNCat( szModemLogStr, " MTMSN: ", MAX_MODEM_LOG_STR );
        StringNCat( szModemLogStr, GetMTMSN(), MAX_MODEM_LOG_STR );
    }

    StringNCat( szModemLogStr, "\r\n", MAX_MODEM_LOG_STR );


    if( bDisplayModemLogErrors )
    {
        SendStringToMtcePort( "\r\nMODEMLOG> " );
        SendStringToMtcePort( szModemLogStr );
    }

    WriteMdmLogFile( szModemLogStr );
}


//******************************************************************************
//
//  Function: RecordModemLogError
//
//  Arguments:
//    IN  errCode - A MODEMLOG_ERR_CODE enum that indexes into the
//                  TEXT_ERR_MSG buffer which contains a brief description
//                  string of the error.
//
//  Returns: void.
//
//  Description: Interrupt safe mechanism of reporting system log errors.
//               Can only support entries where there is no file name!
//
//******************************************************************************
void RecordModemLogError( MODEMLOG_ERR_CODE errCode )
{
    BYTE byIndex;

    for( byIndex = 0; byIndex < RECORD_Q_LEN; byIndex++ )
    {
        if( byRecordQBuff[byIndex] == errCode )
        {
            return;
        }
    }

    AddDataToQueue( pRecordQueue, errCode );  // Save data
}


//******************************************************************************
//
//  Function: MonitorModemLogErrors
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Monitors if an interrupt raised a modem log error, and
//               reports it on behalf of the interrupt. Supports upto
//               MODEMLOG_NBR_CODES enum values reported per main loop cycle.
//               Can only support entries where there is no file name!
//
//******************************************************************************
void MonitorModemLogErrors( void )
{
    WORD wLogErrorCode;
    WORD wBackupReadIndex;

    if( HAVE_RECORD_DATA() )
    {
        // Disable interrupts while accessing the queue
        DisableInts();

        // Since we verify the queue for duplicate commands,
        // be sure to clear the command just read, to allow for 
        // the command to be posted in the future.
        wBackupReadIndex = RecordQueue.wReadIndex;
        wLogErrorCode = GetDataFromQueue( pRecordQueue );
        RecordQueue.pQueue[wBackupReadIndex] = MODEMLOG_START_CODES;

        // Enable them again.
        EnableInts();

        // Record error and clear flag.
        ModemLog( NO_RPT, (MODEMLOG_ERR_CODE)wLogErrorCode );
    }
}


//******************************************************************************
//
//  Function: DisplayModemLogErrors
//
//  Arguments:
//    IN  bDisplay  - A BOOL value indicating the display status.
//                   ( TRUE to turn on or 
//                     FALSE to turn off displaying messages (default)).
//
//  Returns: void.
//
//  Description: Call this function to turn off or turn on displaying modem
//               log entries written to the modem.log file, to the  mtce port.
//
//******************************************************************************
void DisplayModemLogErrors( BOOL bDisplay )
{
    bDisplayModemLogErrors = bDisplay;
}


//******************************************************************************
//
//  Function: GetModemLogErrDisplayStatus
//
//  Arguments: void.
//
//  Returns: TRUE  if log errors are printed in mtce port mode.
//           FALSE if log error display is off.
//
//  Description: Call this function to determine if modem log entries are being 
//               displayed or not.
//
//******************************************************************************
BOOL GetModemLogErrDisplayStatus( void )
{
    return bDisplayModemLogErrors;
}


//******************************************************************************
//
//  Function: GetModemLogErr
//
//  Arguments:
//    IN/OUT *ErrCodeEnum - IN = -1 OUT = get current index
//                          IN = 0-GetModemErrQSize() OUT = enum value @ list index
//    OUT    *byEnumFreq  - Number of times the enum has been reported.
//    OUT    *dwEnumTime  - 32-bits of date/time stamp.
//
//  Returns: BYTE value of either given or current critical list index.
//
//  Description: This function either gives you the most current error reported
//               or allows you to rotate through the list (by entering the index
//               value through ErrCodeEnum).
//
//******************************************************************************
MODEMLOG_ERR_CODE GetModemLogErr( short* ErrCodeEnum, BYTE* byEnumFreq, 
                                  DWORD* dwEnumTime )
{
    // To get the list, the error code needs to be populated with
    // a value other than 0
    short CurrentCode = *ErrCodeEnum;

    if( CurrentCode < 0 )
    {
        CurrentCode = wBufferedIndex;
    }

    *ErrCodeEnum = (MODEMLOG_ERR_CODE)bufferedErrs[CurrentCode].byModemLogErrEnum;
    *byEnumFreq  = bufferedErrs[CurrentCode].byFrequency;
    *dwEnumTime  = bufferedErrs[CurrentCode].dwDateTimeStamp;

    return (MODEMLOG_ERR_CODE)CurrentCode;
}


//******************************************************************************
//
//  Function: CreateModemLogMessage
//
//  Arguments:
//    IN  dwTimeRequested - 0 if generated
//                          Julian seconds when requested.
//
//  Returns: TRUE if generated and queued successfully
//           FALSE if the file was not generated (no more space) or queued.
//
//  Description: This function saves the current modem log queued enums to
//               a file with the uptime header and queues it to the modem.
//
//******************************************************************************
BYTE* CreateModemLogMessage( DWORD dwTimeRequested )
{
    PCFD                      fd;
    static char               szPathFilename[EMAXPATH];
    static char               szFilename[MAX_FILENAME_LEN];            
    static U_MODEMLOG_ERROR_MSG modemlogData;

    MemSet( &modemlogData, 0, MODEMLOG_MSG_SIZE );

    // Put all the data into one buffer for the CRC calculation.
    GenerateHeader( &modemlogData.modemlogErrFile.header, MODEMLOG_MSG_TYPE, MODEMLOG_MSG_SIZE, dwTimeRequested );
    MemCpy( &modemlogData.modemlogErrFile.bufferedData, bufferedErrs, MODEMLOG_STRUCT_SIZE * BUFFERED_DATA_SIZE );
    modemlogData.modemlogErrFile.dwTimeAtStart = GetTimeAtStart();

    modemlogData.modemlogErrFile.header.wCRC = CalcCRC( &modemlogData.pbyData[CRC_SIZE], MODEMLOG_MSG_SIZE-CRC_SIZE );
    
    CreateNewSystemFileName( szPathFilename,             // pathfilename
                             szFilename,                 // filename
                             GetPCMCIAPath( MODEM_DIR, WORKING_SUBDIR ),// build dir
                             MODEMLOG_MSG_TYPE ); // MT type

    // FOR RULESIM ONLY - PCMCIA driver ignores this tag.
    fd = fileOpen( szPathFilename, PO_CREAT|PO_TRUNC|PO_WRONLY|PO_TEXT, PS_IWRITE );

    if( fd != -1 )
    {
        fileWrite( fd, modemlogData.pbyData, MODEMLOG_MSG_SIZE );

        fileClose( fd );

        QueueFileForSend( MODEM_DIR, szPathFilename );
    }

    return modemlogData.pbyData;
}


//------------------------------------------------------------------------------
//  PRIVATE FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: WriteMdmLogFile
//
//  Arguments:
//    IN  szStr - String to write out to the system log file.
//
//  Returns: void.
//
//  Description: This function writes a string out to the modem log file.
//
//******************************************************************************
void WriteMdmLogFile( char* szStr )
{
    PCFD  fd;

    // Open the file, create file if it does not exist
    fd = fileOpen( GetLogFileName( MODEM_LOG_FILE ), PO_CREAT|PO_APPEND|PO_WRONLY|PO_TEXT, PS_IWRITE | PS_IREAD );

    // Return if file cannot be opened.
    if( fd == -1 )
    {
        return;
    }

    // Append modem status string.
    fileWrite( fd, (BYTE*)szStr, StringLen( szStr ) );

    // Close file
    fileClose( fd );
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


