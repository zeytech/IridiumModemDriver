//******************************************************************************
//
//  modemserial.h: Module Title
//
//      Copyright (c) 2001-2010, Aeromechanical Services Ltd.
//      ALL RIGHTS RESERVED
//
//  This module implements the processor's lower level modem port 
//  satellite (wireless) communications. 
//
//  Buffered transmission is implemented so that a calling function
//  doesn't stall here during transmission/receival of data.
//
//******************************************************************************


#ifndef _MODEMSERIAL_H

    #define _MODEMSERIAL_H

//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#include "typedefs.h"

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


/*artlxdef+*/
#define CIS_NOT_POWERED     TRUE
#define CIS_POWERED         FALSE
/*artlxdef-*/


//------------------------------------------------------------------------------
//  TYPEDEF DECLARATIONS
//------------------------------------------------------------------------------


/*artlxtyp+*/
// The following typedefs are used in the call to OpenPort():
#ifndef _WINDOWS_
    typedef BYTE    PARITY_TYPES;
    enum parity_types
    {
       PARITY_NONE,
       PARITY_EVEN,
       PARITY_ODD,
       PARITY_MARK,
       PARITY_SPACE
    };
#else
    typedef BYTE    PARITY_TYPES;
    enum parity_types
    {
       PARITY_NONEt,
       PARITY_EVENt,
       PARITY_ODDt,
       PARITY_MARKt,
       PARITY_SPACEt
    };
#endif


typedef BYTE    STOP_BITS_TYPES;
enum stop_bits_types
{
   ONE_STOP_BIT,
   ONE_AND_HALF_BITS,
   TWO_STOP_BITS
};

typedef BYTE    FLOW_CNTL_TYPES;
enum flow_cntl_types
{
   NO_FLOW_CNTL,
   XON_XOFF_FLOW_CNTL,
   RTS_CTS_FLOW_CNTL
};

typedef struct 
{
   DWORD dwSpeed;
   WORD  wDataBits;
   WORD  wParity;
   WORD  wStopBits;
   WORD  wFlowCntl;
 } SERIAL_CFG;

typedef BYTE    SERIAL_RTN_TYPES;
enum serial_rtn_types
{
   SERIAL_OK,              // Function was successful
   SERIAL_BAD_PORT,        // Port number passed is not valid
   SERIAL_PORT_CLOSED,     // Port is not currently open
   SERIAL_BAD_PARAM,       // Bad parameter passed in function call
   SERIAL_BUFFER_FULL,     // Buffer has overflowed, oldest data thrown out
   SERIAL_NBR_ERRORS       // MUST ALWAYS BE THE LAST ENUM!
};
/*artlxtyp-*/


//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS PROTOTYPES
//------------------------------------------------------------------------------


/*artlx+*/
//******************************************************************************
//
//  Function: InitModemSerialPorts
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This function must be the first to be called from 
//               this module to initialize static variables and the wireless
//               modem UART port.
//
//******************************************************************************
void InitModemSerialPorts( void );


//******************************************************************************
//
//  Function: OpenModemSerialPort
//
//  Arguments:
//    cfgInfo   - A SERIAL_CFG data structure containing specific
//                information requiring the configuration
//                of the modem port.
//
//  Returns: TRUE on success.
//           FALSE otherwise.
//
//  Description: This function configures the wireless modem port based on 
//               the passed SERIAL_CFG parameter.
//
//******************************************************************************
BOOL OpenModemSerialPort( SERIAL_CFG* cfgInfo );


//******************************************************************************
//
//  Function: GetModemSerialPortSettings
//
//  Arguments:
//    OUT cfgInfo - Pointer to a SERIAL_CFG data structure containing specific
//                  information requiring the configuration
//                  of the modem port.
//
//  Returns: void.
//
//  Description: This function gets the configuration the modem port and
//               populates the passed cfgInfo structure.
//
//******************************************************************************
void GetModemSerialPortSettings( SERIAL_CFG* cfgInfo );


//******************************************************************************
//
//  Function: FlushModemSerialTxQueue
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Call this function to flush the transmit data queue.
//
//******************************************************************************
void FlushModemSerialTxQueue( void );


//******************************************************************************
//
//  Function: FlushModemSerialRxQueue
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Call this function to flush the receive data queue.
//
//******************************************************************************
void FlushModemSerialRxQueue( void );


//******************************************************************************
//
//  Function: GetModemPortChar
//
//  Arguments:
//    byData    - data received from the modem's serial interface.
//
//  Returns: TRUE if data was pending - byte returned in byData
//           FALSE otherwise.
//
//  Description: Call this function to get a byte from the hardware's 
//               receiving FIFO and add it to a queue.  
//
//******************************************************************************
BOOL GetModemPortChar( BYTE* byData );
/*artlx-*/


//******************************************************************************
//
//  Function: ModemIsr
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: This is the modem ISR function. It gets called when a
//               TX or RX interrupt is pending.      
//
//******************************************************************************
void ModemIsr( void );


/*artlx+*/
//******************************************************************************
//
//  Function: ModemPortSendBuffer
//
//  Arguments:
//    pBuffer   - An array of BYTE data to be shifted 
//                out to the transmitting port.
//    wLength   - Specifies the length of the pBuffer array.
//
//  Returns: void.
//
//  Description: Call this function to write the passed byte onto the 
//               hardware's transmitting FIFO. The function only writes to
//               the FIFO when hardware's transmitting port is ready for 
//               shifting data out.
//
//******************************************************************************
void ModemPortSendBuffer( BYTE* pBuffer, WORD wLength );


//******************************************************************************
//
//  Function: ModemPortSending
//
//  Arguments: void.
//
//  Returns: TRUE if data is still being sent
//           FALSE otherwise.
//
//  Description: Call this function to determine if the Modem port is still 
//               sending data to the modem.
//
//******************************************************************************
BOOL ModemPortSending( void );


//******************************************************************************
//
//  Function: ReadModemPortRILine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if RI is high.
//           FALSE (0) if RI is low.
//
//  Description: Reads the RI line (TPUCH5) for its latest state.
//
//      Note: This is an RS332 line, hence, the logic is complemented 
//            i.e. HIGH is actually -32 Volts (low)  
//
//******************************************************************************
BOOL ReadModemPortRILine( void );


//******************************************************************************
//
//  Function: ReadModemPortDCDLine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if DCD is high.
//           FALSE (0) if DCD is low.
//
//  Description: Reads the DCD line (TPUCH4) for its latest state.
//
//
//      Note: This is an RS332 line, hence, the logic is complemented 
//            i.e. HIGH is actually -32 Volts (low)  
//******************************************************************************
BOOL ReadModemPortDCDLine( void );


//******************************************************************************
//
//  Function: ReadModemPortDSRLine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if DSR is high.
//           FALSE (0) if DSR is low.
//
//  Description: Reads the DSR line (TPUCH2) for its latest state.
//
//      Note: This is an RS332 line, hence, the logic is complemented 
//            i.e. HIGH is actually -32 Volts (low)  
//
//******************************************************************************
BOOL ReadModemPortDSRLine( void );


//******************************************************************************
//
//  Function: ReadModemPortCTSLine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if CTS is high.
//           FALSE (0) if CTS is low.
//
//  Description: Reads the CTS line (TPUCH0) for its latest state.
//
//      Note: This is an RS332 line, hence, the logic is complemented 
//            i.e. HIGH is actually -12 Volts (low)  
//
//******************************************************************************
BOOL ReadModemPortCTSLine( void );


//******************************************************************************
//
//  Function: ReadModemPortRTSLine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if RTS is high.
//           FALSE (0) if RTS is low.
//
//  Description: Reads the RTS line (TPUCH6) for its latest state.
//
//      Note: This is an RS332 line, hence, the logic is complemented 
//            i.e. HIGH is actually -32 Volts (low)  
//
//******************************************************************************
BOOL ReadModemPortRTSLine( void );


//******************************************************************************
//
//  Function: SetModemPortRTSLow
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 6 TX line HIGH (physically is a low signal).
//
//******************************************************************************
void SetModemPortRTSLow( void );


//******************************************************************************
//
//  Function: SetModemPortRTSHigh
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 6 TX line LOW (physically is a high signal).
//
//******************************************************************************
void SetModemPortRTSHigh( void );


//******************************************************************************
//
//  Function: ReadModemPortDTRLine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if DTR is high.
//           FALSE (0) if DTR is low.
//
//  Description: Reads the DTR line (TPUCH9) for its latest state.
//
//      Note: This is an RS232 line, hence, the logic is complemented 
//            i.e. HIGH is actually -32 Volts (low)  
//
//******************************************************************************
BOOL ReadModemPortDTRLine( void );


//******************************************************************************
//
//  Function: SetModemPortDTRLow
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 9 DTR line HIGH (physically is a low signal).
//
//******************************************************************************
void SetModemPortDTRLow( void );


//******************************************************************************
//
//  Function: SetModemPortDTRHigh
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 9 DTR line LOW (physically is a high signal).
//
//******************************************************************************
void SetModemPortDTRHigh( void );


//******************************************************************************
//
//  Function: SetModemPortTXLow
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 8 TX line HIGH (physically is a low signal).
//
//******************************************************************************
void SetModemPortTXLow( void );


//******************************************************************************
//
//  Function: SetModemPortTXHigh
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 8 TX line LOW (physically is a high signal).
//
//******************************************************************************
void SetModemPortTXHigh( void );


//******************************************************************************
//
//  Function: ReadCISPowerLine
//
//  Arguments: void.
//
//  Returns: TRUE (1) if CISPWR is disabled (high).
//           FALSE (0) if CISPWR is enabled (low).
//
//  Description: Reads the CISPWR line (TPUCH10) for its latest state.
//
//      Note: This is an RS232 line, hence, the logic is complemented 
//            i.e. HIGH is actually -32 Volts (low)  
//
//******************************************************************************
BOOL ReadCISPowerLine( void );


//******************************************************************************
//
//  Function: SetCISPowerLow
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 10 CISPWR line LOW (CIS is enabled).
//
//******************************************************************************
void SetCISPowerLow( void );


//******************************************************************************
//
//  Function: SetCISPowerHigh
//
//  Arguments: void.
//
//  Returns: void.
//
//  Description: Toggles channel 10 CISPWR line HIGH (disables CIS).
//
//******************************************************************************
void SetCISPowerHigh( void );
/*artlx-*/


#endif  // _MODEMSERIAL_H