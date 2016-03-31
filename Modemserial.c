//******************************************************************************
//
//  Modemserial.c: Module Title
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


//------------------------------------------------------------------------------
//  INCLUDES
//------------------------------------------------------------------------------

#include "Modemserial.h"
#include "m68332.h"
#include "tpu.h"
#include "ints.h"
#include "queue.h"
#include "afirs.h"
#include "timer.h"
#include "CISAPI.h"

//------------------------------------------------------------------------------
//  CONSTANT & MACRO DEFINITIONS
//------------------------------------------------------------------------------


// Error Defines for the serial Port
#define SCSR_OVERRUN_ERR  0x0008
#define SCSR_NOISE_ERR    0x0004
#define SCSR_FRAMING_ERR  0x0002
#define SCSR_PARITY_ERR   0x0001
#define SCSR_TXREG_EMPTY  0x0100    // ready for a new character
#define SCSR_TX_BUSY      0x0080
#define SCSR_RX_AVAIL     0x0040

#define SCCR1_EN_INTERPT  0x0080    // there is pending data to write - set Tx interrupt

#define HAVE_TX_Q_DATA()  ( ModemTxQueue.wWriteIndex != ModemTxQueue.wReadIndex )
#define HAVE_RX_Q_DATA()  ( ModemRxQueue.wWriteIndex != ModemRxQueue.wReadIndex )

#define Modem_Q_LEN          4096


//------------------------------------------------------------------------------
//  GLOBAL DECLARATIONS
//------------------------------------------------------------------------------


static COMMQUEUE          ModemTxQueue;
static COMMQUEUE          ModemRxQueue;
static PCOMMQUEUE         pModemTxQueue = &ModemTxQueue; // Assign pointers for faster access in usage
static PCOMMQUEUE         pModemRxQueue = &ModemRxQueue;

static QUEUE_BUFF         txQBuff[Modem_Q_LEN];
static QUEUE_BUFF         rxQBuff[Modem_Q_LEN];


//------------------------------------------------------------------------------
//  PRIVATE FUNCTION PROTOTYPES
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
//  PUBLIC FUNCTIONS
//------------------------------------------------------------------------------


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
void InitModemSerialPorts( void )
{
    InitQueue( pModemTxQueue, txQBuff, Modem_Q_LEN );  // Init the queues
    InitQueue( pModemRxQueue, rxQBuff, Modem_Q_LEN );

    // Init the Queued serial module 
    // SUPV : 1    - Supervisor mode restricted
    // IARB : 1110 - Interrupt arbitration second-highest priority level
    QSMCR = 0x008F;

    // Initialize the Interrupt for the wireless modem port
    QILR = 0x05;     // Interrupt level 5
    QIVR = 0x42;     // SCI interrupt vector (66)
}    


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
BOOL OpenModemSerialPort( SERIAL_CFG* cfgInfo )
{
    // Opens the serial port identified by aPort, and sets its
    // parameters to those in the passed config structure. Returns
    // one of the SERIAL_RTN_TYPES enums.

    WORD wSettings = 0x0000;

    // Set port speed 
    SCCR0 = SYS_CLOCK / ( cfgInfo->dwSpeed * 32 );

    // Set port word size
     
    if( cfgInfo->wDataBits == 9 )
    {
        wSettings = 0x0200;     // Set M bit High (SCCR1)
    }
    else if( cfgInfo->wDataBits == 8 )
    {
        wSettings = 0x0000;
    }
    else
    {
        // Invalid port word size
        return( FALSE );  
    }
    
    // Set port parity
    if( cfgInfo->wParity == PARITY_NONE )
    {
        // No change for this setting
    }
    else if( cfgInfo->wParity == PARITY_EVEN )
    {
        wSettings |= 0x0400;     // Set PE bit High (SCCR1)
    }
    else if( cfgInfo->wParity == PARITY_ODD )
    {
        wSettings |= 0x0C00;    // Set PE and PT bits High (SCCR1)
    }
    else
    {   
        // Invalid port parity
        return( FALSE );   
    }

    // Verify flow control
    if( cfgInfo->wFlowCntl == NO_FLOW_CNTL )
    {
        // do nothing.
    }
    else if( cfgInfo->wFlowCntl == XON_XOFF_FLOW_CNTL )
    {
        // This is not support
        return FALSE;
    }
    else if( cfgInfo->wFlowCntl == RTS_CTS_FLOW_CNTL )
    {
        // Enable signals
    }
    else
    {
        // Invalid flow control
        return FALSE;
    }
           

    // Flush both queues prior to enabling the ports
    FlushModemSerialTxQueue();
    FlushModemSerialRxQueue();
           
    // Now enable the port
    SCCR1 = wSettings | 0x002C;      // interrupts: (Tx) disabled, Rx enabled
                                     // comm: (Tx) enabled, Rx enabled

    return( TRUE );
}


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
void GetModemSerialPortSettings( SERIAL_CFG* cfgInfo )
{
    WORD wSettings = SCCR1 & 0xFFD3;   // mask out interrupts and comm enable bits
    WORD wRegisterSpeed = SCCR0;
    DWORD dwBitRate = SYS_CLOCK / ( (DWORD)wRegisterSpeed * 32L );

    if( ( dwBitRate >= BIT_RATE_1200 ) && ( dwBitRate < BIT_RATE_2400 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_1200;
    }

    else if( ( dwBitRate >= BIT_RATE_2400 ) && ( dwBitRate < BIT_RATE_4800 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_2400;
    }

    else if( ( dwBitRate >= BIT_RATE_4800 ) && ( dwBitRate < BIT_RATE_9600 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_4800;
    }

    else if( ( dwBitRate >= BIT_RATE_9600 ) && ( dwBitRate < BIT_RATE_19200 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_9600;
    }

    else if( ( dwBitRate >= BIT_RATE_19200 ) && ( dwBitRate < BIT_RATE_38400 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_19200;
    }

    else if( ( dwBitRate >= BIT_RATE_38400 ) && ( dwBitRate < BIT_RATE_57600 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_38400;
    }

    else if( ( dwBitRate >= BIT_RATE_57600 ) && ( dwBitRate < BIT_RATE_115200 ) )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_57600;
    }

    else if( dwBitRate >= BIT_RATE_115200 )
    {
        // Set port speed 
        cfgInfo->dwSpeed = BIT_RATE_115200;
    }

    // Set port word size
    if( ( wSettings & 0x0200 ) == 0x0200 )
    {
        // Indicate one more stop bit in frame
        cfgInfo->wDataBits = 8;
        cfgInfo->wStopBits = 2;
    }
    else
    {
        // we have a regular 10-bit frame
        cfgInfo->wDataBits = 8;
        cfgInfo->wStopBits = 1;
    }
    
    // Set port parity
    if( ( wSettings & 0x0400 ) == 0x0400 )
    {
        // Parity is enabled, see what type
        if( ( wSettings & 0x0C00 ) == 0x0C00 )
        {
            cfgInfo->wParity = PARITY_ODD;
        }
        else
        {
            cfgInfo->wParity = PARITY_EVEN;
        }
    }
    else
    {
        cfgInfo->wParity = PARITY_NONE;
    }
}


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
void FlushModemSerialTxQueue( void )
{
    // Disable interrupts while accessing the queue
    DisableInts();

    // Clears data from the port queues.
    FlushQueue( pModemTxQueue );  

    // Enable them again.
    EnableInts();
}


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
void FlushModemSerialRxQueue( void )
{
    // Disable interrupts while accessing the queue
    DisableInts();

    // Clears data from the port queues.
    FlushQueue( pModemRxQueue );  

    // Enable them again.
    EnableInts();
}



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
BOOL GetModemPortChar( BYTE* byData )
{
    WORD qData;

    if( !HAVE_RX_Q_DATA() )
    {
        // Fall through means we have no data.
        return FALSE;
    }

    DisableInts();

    qData = GetDataFromQueue( pModemRxQueue );

    EnableInts();

    *byData = (BYTE)qData;

    return TRUE;
}


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
void ModemIsr( void )
{
    WORD wRxData;

    // while is ok here - max we ever receive is 45 bytes = 47 ms.
    while( SCSR & SCSR_RX_AVAIL )
    {
        // Reading the SCDR clears the rx status flags.
        wRxData = SCDR;
        AddDataToQueue( pModemRxQueue, (BYTE)wRxData );
    }

    // Processor cannot receive and transmit at the same time!
    // Check if we have data to transmit.
    if( HAVE_TX_Q_DATA() )
    {
        // ensure there is room on the register to xmit AND nothing is being received.
        if( ( SCSR & SCSR_TXREG_EMPTY )
            &&
            ( ( SCSR & SCSR_RX_AVAIL ) == 0 ) )
        {
            // Write the byte to the transmit FIFO. Clears SCSR status flags.
            SCDR = (BYTE)GetDataFromQueue( pModemTxQueue );
        }
    }
    else
    {
        // Disable TX interrupt
        SCCR1 &= ~SCCR1_EN_INTERPT;
    }
}


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
void ModemPortSendBuffer( BYTE* pBuffer, WORD wLength )
{
    WORD wIndex;

    for( wIndex = 0; wIndex < wLength; wIndex++ )
    {
        // Add data to queue.
        AddDataToQueue( pModemTxQueue, pBuffer[wIndex] );
    }

    if( HAVE_TX_Q_DATA() )
    {
        // enable the interrupt
        SCCR1 |= SCCR1_EN_INTERPT;
    }
}


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
BOOL ModemPortSending( void )
{
    return( HAVE_TX_Q_DATA() );
}


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
BOOL ReadModemPortRILine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 5.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( WMRI_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
BOOL ReadModemPortDCDLine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 4.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( WMDCD_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
BOOL ReadModemPortDSRLine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 2.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( WMDSR_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
BOOL ReadModemPortCTSLine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 0.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( WMCTS_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
BOOL ReadModemPortRTSLine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 6.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( WMRTS_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
void SetModemPortRTSLow( void )
{
    // Set bit 13 to 0 and bit 12 to 1 
    HSRR1 &= 0xDFFF;
    HSRR1 |= 0x1000;

}


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
void SetModemPortRTSHigh( void )
{
    // Set bit 13 to 1 and bit 12 to 0 to toggle channel 6.
    HSRR1 |= 0x2000;
    HSRR1 &= 0xEFFF;

}


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
BOOL ReadModemPortDTRLine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 9.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( WMDTR_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
void SetModemPortDTRLow( void )
{
    // Set bit 3 to 0 and bit 2 to 1 
    HSRR0 &= 0xFFF7;
    HSRR0 |= 0x0004;

}


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
void SetModemPortDTRHigh( void )
{
    // Set bit 3 to 1 and bit 2 to 0 to toggle channel 9.
    HSRR0 |= 0x0008;
    HSRR0 &= 0xFFFB;

}


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
void SetModemPortTXLow( void )
{
    // Set bit 1 to 0 and bit 0 to 1 
    HSRR0 &= 0xFFFD;
    HSRR0 |= 0x0001;

}


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
void SetModemPortTXHigh( void )
{
    // Set bit 1 to 1 and bit 0 to 0 to toggle channel 8.
    HSRR0 |= 0x0002;
    HSRR0 &= 0xFFFE;

}


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
BOOL ReadCISPowerLine( void )
{
    // The Pin Level line returns 0-15 bits of buffered states
    // on channel 10.  We only want the most recent pin level
    // hence, only check bit 15
    if( ( CISPWR_PL & LATEST_PIN_LEVEL ) == LATEST_PIN_LEVEL )
    {
        return FALSE;
    }

    return TRUE;
}


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
void SetCISPowerLow( void )
{
    // Set bit 5-4 to 10 to toggle channel 10.
    HSRR0 |= 0x0020;
    HSRR0 &= 0xFFEF;
}


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
void SetCISPowerHigh( void )
{
    // Set bit 5-4 to 01 
    HSRR0 &= 0xFFDF;
    HSRR0 |= 0x0010;
}


//------------------------------------------------------------------------------
//  PRIVATE FUNCTIONS
//------------------------------------------------------------------------------


//******************************************************************************
//
//  Function: FunctName
//
//  Arguments:
//    wMisc     - data used for something
//    byName    - pointer to name
//
//  Returns: what FunctName returns
//
//  Description: Function description
//
//******************************************************************************


