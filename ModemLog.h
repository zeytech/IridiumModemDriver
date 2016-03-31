//******************************************************************************
//
//  ModemLog.h: Module Title
//
//      Copyright (c) 2001-2010, Aeromechanical Services Ltd.
//      ALL RIGHTS RESERVED
//
//  This module provides interfaces for logging the status of a 
//  summary report transfer.
//
//******************************************************************************

#ifndef _MODEM_LOG_H

    #define _MODEM_LOG_H

//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#include "typedefs.h"

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
//  TYPEDEF DECLARATIONS
//------------------------------------------------------------------------------


/*artlxtyp+*/
typedef BYTE    MODEMLOG_ERR_CODE;
enum modemlog_err_code
{
    MODEMLOG_START_CODES = 0,

    MODEMLOG_MODEM_IS_POWERED,
    MODEMLOG_MODEM_POWERED_DOWN,
    MODEMLOG_SEND_SUCCESSFUL,
    MODEMLOG_SEND_ENABLED,
    MODEMLOG_SEND_FAILURE,
    MODEMLOG_SEND_DISABLED,
    MODEMLOG_SEND,
    MODEMLOG_RETRY_SEND,
    MODEMLOG_RECEIVE,
    MODEMLOG_RECEIVE_SUCCESSFUL,
    MODEMLOG_RECEIVE_FAILURE,
    MODEMLOG_UNEXPECTED_RSP,
    MODEMLOG_MOVE_FAILURE,
    MODEMLOG_DELETE_FAILURE,
    MODEMLOG_COPY_SUCCESS,
    MODEMLOG_COPY_FAILURE,
    MODEMLOG_MAILBOXCHECK_SUCCESS,
    MODEMLOG_MAILBOXCHECK_FAILURE,
    MODEMLOG_SIGNALSTRENGTH_FAILURE,
    MODEMLOG_HUNG_UP_CALL_SUCCESS,
    MODEMLOG_HUNG_UP_CALL_FAILURE,
    MODEMLOG_PHONE_OFF_HOOK,
    MODEMLOG_PHONE_BACK_ON_HOOK,
    MODEMLOG_INCOMING_CALL,
    MODEMLOG_INCOMING_CALL_COMPLETE,
    MODEMLOG_MUTE_BTN_PRESSED,
    MODEMLOG_MUTE_BTN_RELEASED,
    MODEMLOG_NBR_CODES

};
/*artlxtyp-*/


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS PROTOTYPES
//------------------------------------------------------------------------------


/*artlx+*/
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
void ModemLogInit( void );


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
void ModemLog( const char* szFileName, MODEMLOG_ERR_CODE errCode );


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
void RecordModemLogError( MODEMLOG_ERR_CODE errCode );


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
void MonitorModemLogErrors( void );


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
void DisplayModemLogErrors( BOOL bDisplay );


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
BOOL GetModemLogErrDisplayStatus( void );


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
                                  DWORD* dwEnumTime );


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
BYTE* CreateModemLogMessage( DWORD dwTimeRequested );
/*artlx-*/


#endif // _MODEM_LOG_H