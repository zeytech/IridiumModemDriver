//******************************************************************************
//
//  ModemAPI.h: Module Title
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
//******************************************************************************

#ifndef _MODEMAPI_H

    #define _MODEMAPI_H


//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#include "typedefs.h"
#include "Modem.h"
#include "PcmciaDirs.h"
#include "utils.h"

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


/*artldef+*/
#define MAX_PRIORITY_FLAGS      BASE_36
#define KEEP_ALL_FILES          '*'
#define DELETE_ALL_FILES        NULL
/*artldef-*/


//------------------------------------------------------------------------------
//  TYPEDEF DECLARATIONS
//------------------------------------------------------------------------------


/*artltyp+*/
// Possible states of our modem handler state machine:
typedef BYTE    MODEM_STATES;
enum modem_states
{
    MODEM_POWERED_DOWN, // Waiting for modem to power up.
    MODEM_INITTING,     // Modem is initializing
    MODEM_IDLE,         // Modem is idle, but can start sending if msg is pending.
    MODEM_BUSY,         // Modem is busy sending data (and waiting for response)
    NBR_MODEM_STATES
} ;
/*artltyp-*/


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS PROTOTYPES
//------------------------------------------------------------------------------


/*artlx+*/
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
void InitModemAPI( void );


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
void DisableReportSending( void );


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
void EnableReportSending( void );
/*artlx-*/


/*artl+*/
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
BOOL IsModemSendingEnabled( void );
/*artl-*/


/*artlx+*/
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
//               Message verification is done at the middle layer. If the
//               message exceeds maximum message length, the message will
//               be truncated.
//
//******************************************************************************
BOOL SendTextMsgToModem( const char* szDataBuf, WORD wMsgLen );


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
MODEM_RESPONSES GetTextMsgRspFromModem( void );


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
BOOL SendBinMsgToModem( const BYTE* byDataBuf, WORD wMsgLen );


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
MODEM_RESPONSES GetBinMsgRspFromModem( void );
/*artlx-*/


/*artl+*/
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
MODEM_STATES GetCurrentModemState( void );


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
void SetSignalStrengthPollRate( const DWORD dwPollRateInSeconds );


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
DWORD GetSignalStrengthPollRate( void );


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
void SetCSQRetryCount( const BYTE byCSQRetryCount );


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
BYTE GetCSQRetryCount( void );


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
void SetCSQRetryDelay( const DWORD dwCSQRetryDelayInSeconds );


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
DWORD GetCSQRetryDelay( void );


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
void SetMsgRetryCount( const BYTE byRetryCount );


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
BYTE GetMsgRetryCount( void );


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
void SetMsgRetryDelay( const DWORD dwRetryDelayInSeconds );


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
DWORD GetMsgRetryDelay( void );


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
void SetModemIncomingCallDelay( const DWORD dwCallDelayInSeconds );


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
DWORD GetModemIncomingCallDelay( void );


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
void SetModemDialingDelay( const DWORD dwDialingDelayInSeconds );


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
DWORD GetModemDialingDelay( void );


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
void SetModemTimeoutWait( const DWORD dwTimeoutDelayInSeconds );


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
DWORD GetModemTimeoutWait( void );


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
void KeepSentFiles( char* pcPriorityList );


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
BOOL InVoiceCall( void );
/*artl-*/


/*artlx+*/
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
void ReportPCMCIAError( BOOL bErrorWithPCMCIACard );


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
MODEM_RESPONSES GetModemCmdRsp( MODEM_COMMANDS mdmCmd );


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
BOOL HangupCall( void );


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
void EnteredTransparentModemMode( BOOL bMode );


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
void ProcessModemStateMachine( void );
/*artlx-*/


#endif // _MODEMAPI_H