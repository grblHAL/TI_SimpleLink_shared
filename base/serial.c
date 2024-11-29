/*
  serial.c - serial stream driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "serial.h"
#include "grbl/protocol.h"

static void uart_interrupt_handler (void);

static stream_tx_buffer_t txbuf = {0};
static stream_rx_buffer_t rxbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

#ifdef SERIAL2_MOD
static stream_rx_buffer_t rx2buf;
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;
static void uart2_interrupt_handler (void);
#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = Off,
      .claim = serialInit
    },
#ifdef SERIAL2_MOD
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.can_set_baud = Off,
      .claim = serial2Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;

    if(tail == rxbuf.head)
        return -1; // no data available

//    UARTIntDisable(SERIAL1_PORT, UART_INT_RX|UART_INT_RT);
    char c = rxbuf.data[tail];          // Get next character, increment tmp pointer
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

//    UARTIntEnable(SERIAL1_PORT, UART_INT_RX|UART_INT_RT);
 #ifdef RTS_PORT
    if (rxbuf.rts_state && BUFCOUNT(rxbuf.head, rxbuf.tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)   // Clear RTS if
        GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuf.rts_state = 0);                                        // buffer count is below low water mark
 #endif
    return (int16_t)c;
}

static inline uint16_t serialRxCount (void)
{
    uint16_t head = rxbuf.head, tail = rxbuf.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static uint16_t serialRxFree (void)
{
    uint_fast16_t head = rxbuf.head, tail = rxbuf.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
 #ifdef RTS_PORT
    GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuf.rts_state = 0);
 #endif
}

static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
 #ifdef RTS_PORT
    GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuf.rts_state = 0);
 #endif
}

static bool serialPutC (const char c)
{
    if(txbuf.head != txbuf.tail || !UARTCharPutNonBlocking(SERIAL1_BASE, c)) {  // Send character without buffering if possible...

        uint_fast16_t next_head = BUFNEXT(txbuf.head, txbuf);                   // .. if not, get pointer to next free slot in buffer

        while(txbuf.tail == next_head) {                                        // Buffer full, block until space is available...
            if(!hal.stream_blocking_callback())
                return false;
        }

        txbuf.data[txbuf.head] = c;                                             // Add data to buffer
        txbuf.head = next_head;                                                 // and update head pointer

        UARTIntEnable(SERIAL1_BASE, UART_INT_TX);                               // Enable interrupts
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
    return stream_rx_suspend(&rxbuf, suspend);
}

uint16_t serialTxCount (void)
{
    uint_fast16_t head = txbuf.head, tail = txbuf.tail;

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

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = true,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_all = serialWriteS,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .get_rx_buffer_count = serialRxCount,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed || baud_rate != 115200)
        return NULL;

    serial[0].flags.claimed = On;

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

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .port = (void *)SERIAL1_BASE,
        .pin = SERIAL1_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .port = (void *)SERIAL1_BASE,
        .pin = SERIAL1_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

static void uart_interrupt_handler (void)
{
    uint32_t iflags = UARTIntStatus(SERIAL1_BASE, true);

    if(iflags & UART_INT_TX) {

        uint_fast16_t tail = txbuf.tail;                                // Get buffer pointer

        if(tail != txbuf.head) {

            UARTCharPut(SERIAL1_BASE, txbuf.data[tail]);                // Put character in TX FIFO
            tail = BUFNEXT(tail, txbuf);                                // and update tmp tail pointer

            while(tail != txbuf.head && UARTSpaceAvail(SERIAL1_BASE)) { // While data in TX buffer and free space in TX FIFO
                UARTCharPut(SERIAL1_BASE, txbuf.data[tail]);            // put next character
                tail = BUFNEXT(tail, txbuf);                            // and update tmp tail pointer
            }

            txbuf.tail = tail;                                          // Update pointer

            if(tail == txbuf.head)                                      // Disable TX interrupts
                UARTIntDisable(SERIAL1_BASE, UART_INT_TX);              // when TX buffer empty
        }
    }

    if(iflags & (UART_INT_RX|UART_INT_RT)) {

        int32_t c = UARTCharGet(SERIAL1_BASE);

        if(!enqueue_realtime_command((char)c)) {

            uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get and increment buffer pointer

            if(next_head == rxbuf.tail)                             // If buffer full
                rxbuf.overflow = 1;                                 // flag overflow,
            else {
                rxbuf.data[rxbuf.head] = (char)c;                   // else add data to buffer
                rxbuf.head = next_head;                             // and update pointer
            }
        }

   #ifdef RTS_PORT
        if (!rxbuf.rts_state && BUFCOUNT(rxbuf.head, rxbuf.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
            GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuf.rts_state = RTS_PIN);
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

static bool serial2EnqueueRtCommand (char c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .is_connected = stream_connected,
        .read = serial2GetC,
//        .write = serial2WriteS,
//        .write_char = serial2PutC,
//        .write_all = serial2WriteS,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
//        .suspend_read = serial2SuspendInput,
        .disable_rx = serial2Disable,
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(serial[1].flags.claimed || baud_rate != 115200)
        return NULL;

    serial[1].flags.claimed = On;

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

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .port = (void *)SERIAL2_BASE,
        .pin = SERIAL2_TX_PIN,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .port = (void *)SERIAL2_BASE,
        .pin = SERIAL2_RX_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

static void uart2_interrupt_handler (void)
{
    uint32_t iflags = UARTIntStatus(SERIAL2_BASE, true);

    if(iflags & (UART_INT_RX|UART_INT_RT)) {
        int32_t c = UARTCharGet(SERIAL2_BASE);
        if(!enqueue_realtime_command2((char)c)) {
            uint16_t next_head = BUFNEXT(rx2buf.head, rx2buf);  // Get and increment buffer pointer
            if(next_head == rx2buf.tail)                        // If buffer full
                rx2buf.overflow = 1;                            // flag overlow
            else {
                rx2buf.data[rx2buf.head] = (char)c;             // Add data to buffer
                rx2buf.head = next_head;                        // and update pointer
            }
        }
    }
}

#endif // SERIAL2_MOD
