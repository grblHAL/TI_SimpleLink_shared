/*
  serial.c - serial stream driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "serial.h"

static void uart_interrupt_handler (void);

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};

#ifdef SERIAL2_MOD
static stream_rx_buffer_t rx2buf;
static void uart2_interrupt_handler (void);
#endif

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    char c;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

//    UARTIntDisable(SERIAL1_PORT, UART_INT_RX|UART_INT_RT);
    c = rxbuffer.data[bptr++];                      // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

//    UARTIntEnable(SERIAL1_PORT, UART_INT_RX|UART_INT_RT);
 #ifdef RTS_PORT
    if (rxbuffer.rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)   // Clear RTS if
        GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = 0);                                        // buffer count is below low water mark
 #endif
    return (int16_t)c;
}

static inline uint16_t serialRxCount (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
 #ifdef RTS_PORT
    GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = 0);
 #endif
}

static void serialRxCancel (void)
{
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
 #ifdef RTS_PORT
    GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = 0);
 #endif
}

static bool serialPutC (const char c)
{
    uint32_t next_head;

    if(txbuffer.head != txbuffer.tail || !UARTCharPutNonBlocking(SERIAL1_BASE, c)) {  // Send character without buffering if possible

        next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);           // Get and update head pointer

        while(txbuffer.tail == next_head) {                               // Buffer full, block until space is available...
            if(!hal.stream_blocking_callback())
                return false;
        }

        txbuffer.data[txbuffer.head] = c;                                         // Add data to buffer
        txbuffer.head = next_head;                                        // and update head pointer

        UARTIntEnable(SERIAL1_BASE, UART_INT_TX); // Enable interrupts
    }

    return true;
}

static void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

uint16_t serialTxCount (void)
{
    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static bool serialDisable (bool disable)
{
    if(disable)
        UARTIntDisable(SERIAL1_BASE, UART_INT_RX|UART_INT_RT);
    else
        UARTIntEnable(SERIAL1_BASE, UART_INT_RX|UART_INT_RT);

    return true;
}

const io_stream_t *serialInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .connected = true,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable = serialDisable
    };

    SysCtlPeripheralEnable(SERIAL1_PERIPH);
    SysCtlPeripheralEnable(SERIAL1_SYSCTL);
    SysCtlDelay(3);

    GPIOPinConfigure(SERIAL1_RX); // JP4.1
    GPIOPinConfigure(SERIAL1_TX);
    GPIOPinTypeUART(SERIAL1_PORT, SERIAL1_PINS);

#ifdef __MSP432E401Y__
    UARTClockSourceSet(SERIAL1_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk(SERIAL1_BASE, 120000000UL, 115200UL, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
#else // TM4C1294
    UARTClockSourceSet(SERIAL1_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(SERIAL1_BASE, 16000000UL, 115200UL, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
#endif

    UARTFIFOEnable(SERIAL1_BASE);
    UARTFIFOLevelSet(SERIAL1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    IntPrioritySet(SERIAL1_INT, 0x40);
    UARTIntRegister(SERIAL1_BASE, uart_interrupt_handler);
    UARTIntEnable(SERIAL1_BASE, UART_INT_RX|UART_INT_RT);
    UARTTxIntModeSet(SERIAL1_BASE, UART_TXINT_MODE_EOT);

    UARTEnable(SERIAL1_BASE);

#ifdef RTS_PORT

    SysCtlPeripheralEnable(RTS_PERIPH);
    SysCtlDelay(3);

    GPIOPinTypeGPIOOutput(RTS_PORT, RTS_PIN);
    GPIOPinWrite(RTS_PORT, RTS_PIN, 0);

#endif

    return &stream;
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    uint32_t iflags = UARTIntStatus(SERIAL1_BASE, true);

    if(iflags & UART_INT_TX) {

        bptr = txbuffer.tail;

        if(txbuffer.head != bptr) {

            UARTCharPut(SERIAL1_BASE, txbuffer.data[bptr++]);           // Put character in TXT FIFO
            bptr &= (TX_BUFFER_SIZE - 1);                               // and update tmp tail pointer

            while(txbuffer.head != bptr && UARTSpaceAvail(SERIAL1_BASE)) {  // While data in TX buffer and free space in TX FIFO
                UARTCharPut(SERIAL1_BASE, txbuffer.data[bptr++]);           // put next character
                bptr &= (TX_BUFFER_SIZE - 1);                               // and update tmp tail pointer
            }

            txbuffer.tail = bptr;                               //  Update tail pinter

            if(bptr == txbuffer.head)                           // Disable TX  interrups
                UARTIntDisable(SERIAL1_BASE, UART_INT_TX);      // when TX buffer empty
        }
    }

    if(iflags & (UART_INT_RX|UART_INT_RT)) {

        int32_t c = UARTCharGet(SERIAL1_BASE);

        if(c == CMD_TOOL_ACK && !rxbuffer.backup) {
            stream_rx_backup(&rxbuffer);
            hal.stream.read = serialGetC; // restore normal input
        } else if(!hal.stream.enqueue_realtime_command((char)c)) {

            bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

            if(bptr == rxbuffer.tail)                           // If buffer full
                rxbuffer.overflow = 1;                          // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)c;         // else add data to buffer
                rxbuffer.head = bptr;                           // and update pointer
            }
        }

   #ifdef RTS_PORT
        if (!rxbuffer.rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
            GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = RTS_PIN);
   #endif
    }
}

#ifdef SERIAL2_MOD

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial2RxFree (void)
{
    uint_fast16_t tail = rx2buf.tail, head = rx2buf.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial2RxFlush (void)
{
    rx2buf.head = rx2buf.tail;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial2RxCancel (void)
{
    rx2buf.data[rx2buf.head] = ASCII_CAN;
    rx2buf.tail = rx2buf.head;
    rx2buf.head = (rx2buf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serial2GetC (void)
{
    uint_fast16_t bptr = rx2buf.tail;

    if(bptr == rx2buf.head)
        return -1; // no data available else EOF

    char data = rx2buf.data[bptr++];             // Get next character, increment tmp pointer
    rx2buf.tail = bptr & (RX_BUFFER_SIZE - 1); // and update pointer

    return (int16_t)data;
}

static bool serial2Disable (bool disable)
{
    if(disable)
        UARTIntDisable(SERIAL2_BASE, UART_INT_RX|UART_INT_RT);
    else
        UARTIntEnable(SERIAL2_BASE, UART_INT_RX|UART_INT_RT);

    return true;
}

const io_stream_t *serial2Init (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .connected = true,
        .read = serial2GetC,
//        .write = serial2WriteS,
//        .write_char = serial2PutC,
//        .write_all = serial2WriteS,
        .get_rx_buffer_free = serial2RxFree,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
//        .suspend_read = serial2SuspendInput,
        .disable = serial2Disable
    };

    SysCtlPeripheralEnable(SERIAL2_PERIPH);
    SysCtlPeripheralEnable(SERIAL2_SYSCTL);
    SysCtlDelay(3);

    GPIOPinConfigure(SERIAL2_RX);
    GPIOPinConfigure(SERIAL2_TX);
    GPIOPinTypeUART(SERIAL2_PORT, SERIAL2_PINS);

  #ifdef __MSP432E401Y__
    UARTClockSourceSet(SERIAL2_BASE, UART_CLOCK_SYSTEM);
    UARTConfigSetExpClk(SERIAL2_BASE, 120000000UL, 115200UL, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  #else // TM4C1294
    UARTClockSourceSet(SERIAL2_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(SERIAL2_BASE, 16000000UL, 115200UL, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  #endif
    UARTFIFOEnable(SERIAL2_BASE);
    UARTFIFOLevelSet(SERIAL2_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    IntPrioritySet(SERIAL2_INT, 0x40);
    UARTIntRegister(SERIAL2_BASE, uart2_interrupt_handler);
    UARTEnable(SERIAL2_BASE);

    return &stream;
}

static void uart2_interrupt_handler (void)
{
    uint32_t iflags = UARTIntStatus(SERIAL2_BASE, true);

    if(iflags & (UART_INT_RX|UART_INT_RT)) {

        uint_fast16_t bptr = (rx2buf.head + 1) & (RX_BUFFER_SIZE - 1); // Get next head pointer

        if(bptr == rx2buf.tail) {                                      // If buffer full
            rx2buf.overflow = 1;                                       // flag overlow
            UARTCharGet(SERIAL2_BASE);                              // and do dummy read to clear interrupt;
        } else {
            int32_t c = UARTCharGet(SERIAL2_BASE);
            if(!hal.stream.enqueue_realtime_command((char)c)) {
                rx2buf.data[rx2buf.head] = (char)c; // Add data to buffer
                rx2buf.head = bptr;                 // and update pointer
            }
        }
    }
}

#endif // SERIAL2_MOD
