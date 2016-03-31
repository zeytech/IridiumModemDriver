//******************************************************************************
//
//  Modem.h: Module Title
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
//******************************************************************************

#ifndef _MODEM_H

    #define _MODEM_H

//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#include "typedefs.h"
#include "ATInterface.h"

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


/*artldef+*/
// Max IMEI characters (including NULL character)
#define IMEI_SIZE                   16 // 15 + NULL - DO NOT EVER CHANGE!
#define ERROR_IMEI                  "000000000000000"
#define CHECKSUM_SIZE               ( sizeof( WORD ) )
#define MODEM_SW_VER_SIZE           (8)   // includes NULL
/*artldef-*/


//------------------------------------------------------------------------------
//  TYPEDEF DECLARATIONS
//------------------------------------------------------------------------------


/*artltyp+*/
// Call status responses
typedef BYTE    CALL_STATUS_RSP;
enum call_status_rsp
{
    AT_RSP_ACTIVE_CALL_STATUS = 0,
    AT_RSP_HELD_CALL_STATUS,
    AT_RSP_DIALING_CALL_STATUS,
    AT_RSP_INVALID_CALL_STATUS, // error response
    AT_RSP_INCOMING_CALL_STATUS,
    AT_RSP_WAITING_CALL_STATUS,
    AT_RSP_IDLE_CALL_STATUS,
    CALL_STATUS_WAITING_FOR_RSP
};
/*artltyp-*/


/*artlxtyp+*/
// Mailbox check responses
typedef BYTE    MAILBOXCHECK_RSP;
enum mailboxcheck_rsp
{
    NO_MSG = 0,
    SUCCESSFUL_MSG,
    FAILED_MSG
};


// The following enum identifies the modem response
typedef BYTE    MODEM_RESPONSES;
enum modem_responses
{
    MR_FAILED,                    // Failure response
    MR_SUCCESS,                   // Modem responded successfully.
    MR_WAITING,                   // Still waiting for a response.
    MR_NO_RESP                    // Timed out waiting for a response
} ;


// Error Codes used to describe in the modem log, exactly what happened (in 
// case of an error). Be sure to align enum values to text values, given 
// in modemlog.c:
typedef BYTE    MODEM_ERROR_CODE_RSP;
enum modem_error_code_rsp
{
    MEC_NONE = 0,
    MEC_ERROR,
    MEC_HW_ERROR,
    MEC_RX_BUFFER_OVERFLOW,
    MEC_RSP_TIMED_OUT,
    MEC_TX_BIN_DATA_TIMEOUT,
    MEC_TX_BIN_DATA_BAD_CHECKSUM,
    MEC_TX_BIN_DATA_BAD_SIZE,
    MEC_SBDI_GSS_TIMEOUT,
    MEC_SBDI_GSS_Q_FULL,
    MEC_SBDI_MO_SEGMENT_ERR,
    MEC_SBDI_INCOMPLETE_SESSION,
    MEC_SBDI_SEGMENT_SIZE_ERR,
    MEC_SBDI_GSS_ACCESS_DENIED,
    MEC_SBDI_SBD_BLOCKED,
    MEC_SBDI_ISU_TIMEOUT,
    MEC_SBDI_RF_DROP,
    MEC_SBDI_PROTOCOL_ERR,
    MEC_SBDI_NO_NETWORK_SERVICE,
    MEC_SBDI_ISU_BUSY,
    MEC_SBDI_FAIL,
    MEC_CLEAR_MODEM_BUFFER_ERROR,
    MEC_FILE_OPEN_ERR,
    MEC_FILE_READ_ERR,
    MEC_FILE_WRITE_ERR,
    MEC_TRUNCATED_FILE,
    MEC_SBDS_SUCESS,
    MEC_SBDS_NO_TX_MSG,
    MEC_SBDS_TX_MSG_PENDING,
    MEC_SBDS_NO_RX_MSG,
    MEC_SBDS_RX_MSG_PENDING,
    MEC_CREG_NOT_REGISTERED,
    MEC_CREG_REGISTERED_HOME,
    MEC_CREG_SEARCHING,
    MEC_CREG_DENIED,
    MEC_CREG_UNKNOWN,
    MEC_CREG_REGISTERED_ROAMING,
    MEC_CSQ_ERROR,
    MEC_ACTIVE_CALL_STATUS,
    MEC_HELD_CALL_STATUS,
    MEC_DIALING_CALL_STATUS,
    MEC_INCOMING_CALL_STATUS,
    MEC_WAITING_CALL_STATUS,
    MEC_IDLE_CALL_STATUS,
    MEC_RX_NO_MSG_WAITING,
    MEC_RX_BAD_CHECKSUM,
    MEC_RX_BAD_FILELENGTH,
    MEC_MODEM_POWERED_DOWN,
    MEC_CIS_RINGER_OFF,
    MEC_CIS_RINGER_ON,
    MEC_CIS_RELAY1_OFF,
    MEC_CIS_RELAY1_ON,
    MEC_CIS_RELAY2_OFF,
    MEC_CIS_RELAY2_ON,
    NBR_ERR_CODES
};
/*artlxtyp-*/


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS PROTOTYPES
//------------------------------------------------------------------------------


/*artlx+*/
//******************************************************************************
//
//  Function: InitModem
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Initializes all local states/variables. Hardware 
//               (lower layer) must be initialized before this
//               is called.  Upper API layer can then be called
//               after this.
//
//******************************************************************************
void InitModem( void );


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
BOOL ResetModem( void );


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
BOOL SendWriteTextMsgCmd( const char *szDataBuf );


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
BOOL SendBinaryFile( char *szPathfileName );


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
BOOL SendBinaryBuffer( const BYTE *pbyDataBuf, WORD wMsgSize );


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
BOOL CheckGateway( void );


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
BOOL CheckMailbox( void );


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
BOOL SendCSQCmd( void );


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
BOOL SendReadBinaryFileCmd( void );


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
BOOL SendCLCCCmd( void );


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
BOOL SendCallHangupCmd( void );


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
BOOL SendCREGCmd( void );


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
BOOL SendDownloadCISCmd( void );


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
BOOL SendProgramCISCmd( void );


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
BOOL SendCISResetCmd( void );


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
BOOL SendSetRingerCmd( BOOL bRingerState );


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
BOOL SendGetRingerStatusCmd( void );


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
BOOL SendSetRelayCmd( BYTE byRelayNbr, BOOL bRelayState );


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
BOOL SendGetRelayStatusCmd( BYTE byRelayNbr );


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
void SetATCmdStateInit( void );


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
void SetATCmdStateIdle( void );
/*artlx-*/


/*artl+*/
//******************************************************************************
//
//  Function: SetModemCmdRspTime
//
//  Arguments:
//    IN *dwTimeoutTime_in_seconds - BYTE value provides time out for satellite
//                                   commands to the modem (in seconds).
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
//               CHANGED DEFINITION IN 200v055 - passes pointer to seconds
//               instead of DWORD array (should not exceed 0xFF).
//               Default is 65 seconds.
//
//******************************************************************************
BOOL SetModemCmdRspTime( DWORD* dwTimeoutTime_in_seconds );


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
BOOL SetModemCmdRspTimeInSeconds( BYTE byTimeoutTime_in_seconds );
/*artl-*/


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
void SetFaultLightOn( void );


/*artlx+*/
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
void ClearModemSignalStrength( void );


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
void UpdateModemState( void );


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
AT_CMD_STATES GetModemAtState( void );


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
MODEM_ERROR_CODE_RSP GetErrorCodeRsp( void );
/*artlx-*/


/*artl+*/
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
const char* GetIMEI( void );
/*artl-*/


/*artlx+*/
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
//  Description: Gets the status information returned by either SBDI or SBDS.
//               MO and MT status values are set to 0 after reading.
//
//               Note MTQueueNbr is only retrieved after an SBDI call.
//
//******************************************************************************
MAILBOXCHECK_RSP GetSBDStatus( void );
/*artlx-*/


/*artl+*/
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
short GetModemSignalStrength( void );


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
CALL_STATUS_RSP GetCallStatus( void );


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
DWORD GetModemCmdRspTime( void );


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
BYTE GetModemCmdRspTimeInSeconds( void );


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
BYTE GetTimeoutCount( void );
/*artl-*/


/*artlx+*/
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
const char* GetModemSWVersion( void );


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
char* GetMOMSN( void );


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
char* GetMTMSN( void );
/*artlx-*/


#endif // _MODEM_H