/*
  driver.c - main driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

  Some parts
   Copyright (c) 2011-2015 Sungeun K. Jeon
   Copyright (c) 2009-2011 Simen Svale Skogsrud

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


#include "driver.h"
#include "eeprom.h"
#include "serial.h"

#include "grbl/limits.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"

#ifdef FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
static void keyclick_int_handler (void);
#endif

#if TRINAMIC_ENABLE
static void trinamic_diag1_isr (void);
#endif

#ifdef I2C_ENABLE
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#if ETHERNET_ENABLE
  #include "shared/ethernet/enet.h"
  #if TELNET_ENABLE
    #include "networking/TCPStream.h"
  #endif
  #if WEBSOCKET_ENABLE
    #include "networking/WsStream.h"
  #endif
#endif

#define USE_32BIT_TIMER 1

#if USE_32BIT_TIMER
#define USE_PIOSC 0 // enabling PIOSC does not work!
#define STEPPER_TIMER TIMER_BOTH
#else
#define STEPPER_TIMER TIMER_A
// prescale step counter to 20Mhz (120 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 5

static uint16_t step_prescaler[3] = {
    STEPPER_DRIVER_PRESCALER,
    STEPPER_DRIVER_PRESCALER + 8,
    STEPPER_DRIVER_PRESCALER + 64
};
#endif

#define STEPPER_PULSE_PRESCALER (12 - 1)

static const io_stream_t *serial_stream;
#if MPG_MODE_ENABLE
static const io_stream_t *mpg_stream;
#endif

#if ETHERNET_ENABLE

static network_services_t services = {0};

static void enetStreamWriteS (const char *data)
{
#if TELNET_ENABLE
    if(services.telnet)
        TCPStreamWriteS(data);
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        WsStreamWriteS(data);
#endif
    serial_stream->write(data);
}

#endif // ETHERNET_ENABLE

#if PWM_RAMPED

#define SPINDLE_RAMP_STEP_INCR 20 // timer compare register change per ramp step
#define SPINDLE_RAMP_STEP_TIME 2  // ms

typedef struct {
    volatile uint32_t ms_cfg;
    volatile uint32_t delay_ms;
    int32_t pwm_current;
    int32_t pwm_target;
    int32_t pwm_step;
} pwm_ramp_t;

static pwm_ramp_t pwm_ramp;
#endif

#if LASER_PPI

laser_ppi_t laser;

static void ppi_timeout_isr (void);

#endif

#if SPINDLE_SYNC_ENABLE

typedef struct {                     // Set when last encoder pulse count did not match at last index
    float block_start;
    float prev_pos;
    float dpp; // distance per pulse in mm
    void (*stepper_pulse_start_normal)(stepper_t *stepper);
    uint32_t timer_value_start;
    uint_fast8_t segment_id;
    uint32_t segments;
} spindle_sync_t;

static void stepperPulseStartSyncronized (stepper_t *stepper);

#endif

#ifdef MODE_SWITCH_PIN
#define MODE_SWITCH_BIT (1<<MODE_SWITCH_PIN)
#endif

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    input_signal_t *signal[DEBOUNCE_QUEUE];
} debounce_queue_t;

typedef struct {
    uint32_t port;
    void (*handler)(void);
    uint32_t count;
    input_signal_t pins[8];
} irq_handler_t;

static input_signal_t inputpin[] = {
    { .id = Input_Reset,          .port = RESET_PORT,         .pin = RESET_PIN,           .group = PinGroup_Control },
    { .id = Input_FeedHold,       .port = FEED_HOLD_PORT,     .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
    { .id = Input_CycleStart,     .port = CYCLE_START_PORT,   .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
#if SAFETY_DOOR_ENABLE
    { .id = Input_SafetyDoor,     .port = SAFETY_DOOR_PORT,   .pin = SAFETY_DOOR_PIN,     .group = PinGroup_Control },
#endif
    { .id = Input_Probe,          .port = PROBE_PORT,         .pin = PROBE_PIN,           .group = PinGroup_Probe },
#ifdef KEYINTR_PIN
    { .id = Input_KeypadStrobe,   .port = KEYINTR_PORT,       .pin = KEYINTR_PIN,         .group = PinGroup_Keypad },
#endif
#ifdef MODE_SWITCH_PIN
    { .id = Input_ModeSelect,     .port = MODE_PORT,          .pin = MODE_SWITCH_PIN,     .group = PinGroup_MPG },
#endif
// Limit input pins must be consecutive in this array
    { .id = Input_LimitX,         .port = X_LIMIT_PORT,       .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
    { .id = Input_LimitY,         .port = Y_LIMIT_PORT,       .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
    { .id = Input_LimitZ,         .port = Z_LIMIT_PORT,       .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit }
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,         .port = A_LIMIT_PORT,       .pin = A_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,         .port = B_LIMIT_PORT,       .pin = B_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,         .port = C_LIMIT_PORT,       .pin = C_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
// Aux input pins must be consecutive in this array
#ifdef AUXINPUT0_PIN
  , { .id = Input_Aux0,           .port = AUXINPUT0_PORT,     .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT1_PIN
  , { .id = Input_Aux1,           .port = AUXINPUT1_PORT,     .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT2_PIN
  , { .id = Input_Aux2,           .port = AUXINPUT2_PORT,     .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT3_PIN
  , { .id = Input_Aux3,           .port = AUXINPUT3_PORT,     .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT4_PIN
  , { .id = Input_Aux4,           .port = AUXINPUT4_PORT,     .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT5_PIN
  , { .id = Input_Aux5,           .port = AUXINPUT5_PORT,     .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT6_PIN
  , { .id = Input_Aux6,           .port = AUXINPUT6_PORT,     .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput }
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = X_STEP_PORT,            .pin = X_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepY,           .port = Y_STEP_PORT,            .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepZ,           .port = Z_STEP_PORT,            .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = A_STEP_PORT,            .pin = A_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = B_STEP_PORT,            .pin = B_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = C_STEP_PORT,            .pin = C_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = X_DIRECTION_PORT,       .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirY,            .port = Y_DIRECTION_PORT,       .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirZ,            .port = Z_DIRECTION_PORT,       .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = A_DIRECTION_PORT,       .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = B_DIRECTION_PORT,       .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = C_DIRECTION_PORT,       .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#if CNC_BOOSTERPACK_A4998
    { .id = Output_StepperPower,    .port = STEPPERS_VDD_PORT,      .pin = STEPPERS_VDD_PIN,        .group = PinGroup_StepperPower },
  #if CNC_BOOSTERPACK2
    { .id = Output_StepperPower,    .port = STEPPERS_VDD_PORT2,     .pin = STEPPERS_VDD_PIN2,       .group = PinGroup_StepperPower },
  #endif
#endif

#if !TRINAMIC_ENABLE
#ifdef STEPPERS_ENABLE_PORT
    { .id = Output_StepperEnable,   .port = STEPPERS_ENABLE_PORT,   .pin = STEPPERS_ENABLE_PIN,     .group = PinGroup_StepperEnable },
#endif
#ifdef XY_ENABLE_PORT
    { .id = Output_StepperEnableXY, .port = XY_ENABLE_PORT,         .pin = XY_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Z_ENABLE_PORT
    { .id = Output_StepperEnableZ,  .port = Z_ENABLE_PORT,          .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef AB_ENABLE
    { .id = Output_StepperEnableAB, .port = AB_ENABLE_PORT,         .pin = AB_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef A_ENABLE_PIN
    { .id = Output_StepperEnableA,  .port = A_ENABLE_PORT,          .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef B_ENABLE_PIN
    { .id = Output_StepperEnableB,  .port = B_ENABLE_PORT,          .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef C_ENABLE_PIN
    { .id = Output_StepperEnableC,  .port = C_ENABLE_PORT,          .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#endif
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput }
#endif
};

static void port_a_isr (void);
static void port_b_isr (void);
static void port_c_isr (void);
static void port_d_isr (void);
static void port_e_isr (void);
static void port_f_isr (void);
static void port_g_isr (void);
static void port_h_isr (void);
static void port_k_isr (void);
static void port_l_isr (void);
static void port_m_isr (void);
static void port_n_isr (void);
static void port_p_isr (void);
static void port_q_isr (void);

static irq_handler_t irq_handler[] = {
    { .port = GPIO_PORTA_BASE, .handler = port_a_isr },
    { .port = GPIO_PORTB_BASE, .handler = port_b_isr },
    { .port = GPIO_PORTC_BASE, .handler = port_c_isr },
    { .port = GPIO_PORTD_BASE, .handler = port_d_isr },
    { .port = GPIO_PORTE_BASE, .handler = port_e_isr },
    { .port = GPIO_PORTF_BASE, .handler = port_f_isr },
    { .port = GPIO_PORTG_BASE, .handler = port_g_isr },
    { .port = GPIO_PORTH_BASE, .handler = port_h_isr },
    { .port = GPIO_PORTK_BASE, .handler = port_k_isr },
    { .port = GPIO_PORTL_BASE, .handler = port_l_isr },
    { .port = GPIO_PORTM_BASE, .handler = port_m_isr },
    { .port = GPIO_PORTN_BASE, .handler = port_n_isr },
    { .port = GPIO_PORTP_BASE, .handler = port_p_isr },
    { .port = GPIO_PORTQ_BASE, .handler = port_q_isr }
};

static pin_group_pins_t limit_inputs = {0};
static debounce_queue_t debounce_queue = {0};
static bool pwmEnabled = false, IOInitDone = false;
static uint32_t pulse_length, pulse_delay;
#ifndef FreeRTOS
static volatile uint32_t elapsed_tics = 0;
#endif
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm = {0};
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static probe_state_t probe = {
    .connected = On
};

#if STEP_OUTMODE == GPIO_MAP

    static const uint8_t c_step_outmap[8] = {
        0,
        X_STEP_BIT,
        Y_STEP_BIT,
        X_STEP_BIT|Y_STEP_BIT,
        Z_STEP_BIT,
        X_STEP_BIT|Z_STEP_BIT,
        Y_STEP_BIT|Z_STEP_BIT,
        X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT
    };

    static uint8_t step_outmap[8];

#endif

#if DIRECTION_OUTMODE == GPIO_MAP

    static const uint8_t c_dir_outmap[8] = {
        0,
        X_DIRECTION_BIT,
        Y_DIRECTION_BIT,
        X_DIRECTION_BIT|Y_DIRECTION_BIT,
        Z_DIRECTION_BIT,
        X_DIRECTION_BIT|Z_DIRECTION_BIT,
        Y_DIRECTION_BIT|Z_DIRECTION_BIT,
        X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT
    };

    static uint8_t dir_outmap[8];

#endif

#if CNC_BOOSTERPACK2 && DIRECTION_OUTMODE == GPIO_MAP

    static const uint8_t c_dir_outmap2[8] = {
        0,
        A_DIRECTION_BIT,
        B_DIRECTION_BIT,
        A_DIRECTION_BIT|B_DIRECTION_BIT,
        C_DIRECTION_BIT,
        A_DIRECTION_BIT|C_DIRECTION_BIT,
        B_DIRECTION_BIT|C_DIRECTION_BIT,
        A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT
    };

    static uint8_t dir_outmap2[8];

#endif

static bool selectStream (const io_stream_t *stream)
{
    static stream_type_t active_stream = StreamType_Serial;

    if(!stream)
        stream = serial_stream;

    memcpy(&hal.stream, stream, sizeof(io_stream_t));

#if ETHERNET_ENABLE
    if(!hal.stream.write_all)
        hal.stream.write_all = enetStreamWriteS;
#else
    if(!hal.stream.write_all)
        hal.stream.write_all = hal.stream.write;
#endif

    if(!hal.stream.enqueue_realtime_command)
        hal.stream.enqueue_realtime_command = protocol_enqueue_realtime_command;

    switch(stream->type) {

#if TELNET_ENABLE
        case StreamType_Telnet:
            hal.stream.write_all("[MSG:TELNET STREAM ACTIVE]" ASCII_EOL);
            services.telnet = On;
            break;
#endif
#if WEBSOCKET_ENABLE
        case StreamType_WebSocket:
            hal.stream.write_all("[MSG:WEBSOCKET STREAM ACTIVE]" ASCII_EOL);
            services.websocket = On;
            break;
#endif
        case StreamType_Serial:
#if ETHERNET_ENABLE
            services.mask = 0;
#endif
            if(active_stream != StreamType_Serial)
                hal.stream.write_all("[MSG:SERIAL STREAM ACTIVE]" ASCII_EOL);
            break;

        default:
            break;
    }

    active_stream = hal.stream.type;

    return stream->type == hal.stream.type;
}

static void spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void software_debounce_isr (void);

#ifdef DEBUGOUT
static void debug_out (bool enable)
{
    GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, enable ? SPINDLE_DIRECTION_BIT : 0);
}
#endif

#ifdef FreeRTOS

static TimerHandle_t xDelayTimer = NULL;

/*
boolean bCalledFromInterrupt (void)
{
    return (SCB->ICSR & SCBICSRVECTACTIVE_Msk) != 0;
}
*/

#if SDCARD_ENABLE && !defined(__MSP432E401Y__)

void fatfsTimerCallback (TimerHandle_t xTimer)
{
    disk_timerproc();
}

#endif

void vTimerCallback (TimerHandle_t xTimer)
{
    if(delay.callback)
        delay.callback();
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(callback) {

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//        if(xPortIsInsideInterrupt())

        xTimerStopFromISR(xDelayTimer, &xHigherPriorityTaskWoken);

        if(delay.callback)
            delay.callback();

        delay.callback = callback;

        xTimerChangePeriodFromISR(xDelayTimer, pdMS_TO_TICKS(ms), &xHigherPriorityTaskWoken);
        xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
    } else {
        TickType_t now = xTaskGetTickCount();
        vTaskDelayUntil(&now, pdMS_TO_TICKS(ms));
    }
}


//*****************************************************************************
//
// This hook is called by FreeRTOS when memory allocation failure is detected.
//
//*****************************************************************************

void vApplicationMallocFailedHook()
{
    while(true);
}

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    while(true);
}


#else

static void systick_isr (void);

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(ms) {
        delay.ms = ms;
        SysTickEnable();
        if(!(delay.callback = callback))
            while(delay.ms);
    } else {
        if(delay.ms) {
            delay.callback = 0;
            delay.ms = 1;
        }
        if(callback)
            callback();
    }
}
#endif

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift
inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.mask;
    DIGITAL_OUT(X_STEP_PORT, X_STEP_PIN, step_outbits.x);
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_PIN, step_outbits.y);
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_PIN, step_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_PIN, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_PIN, step_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PORT, C_STEP_PIN, step_outbits.c);
#endif
#else
#if CNC_BOOSTERPACK2
    step_outbits.value ^= settings.steppers.step_invert.mask;
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, step_outbits.value << STEP_OUTMODE);
    GPIOPinWrite(A_STEP_PORT, HWSTEP_MASK_AB, step_outbits.value >> (3 - STEP_OUTMODE_2));
  #ifdef C_AXIS
    GPIOPinWrite(C_STEP_PORT, C_STEP_BIT, step_outbits.c ? C_STEP_BIT : 0);
  #endif
#else
  #if STEP_OUTMODE == GPIO_MAP
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, step_outmap[step_outbits.value]);
  #else
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, (step_outbits.value ^ settings.steppers.step_invert.mask) << STEP_OUTMODE);
  #endif
#endif
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline static __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_PIN, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_PIN, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_PIN, dir_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_PIN, dir_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_PIN, dir_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PORT, C_DIRECTION_PIN, dir_outbits.c);
#endif
#else
#if CNC_BOOSTERPACK2
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, dir_outmap[dir_outbits.value & 0x07]);
    GPIOPinWrite(DIRECTION_PORT2, HWDIRECTION_MASK2, dir_outmap2[dir_outbits.value >> 3]);
#else
  #if DIRECTION_OUTMODE == GPIO_MAP
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, dir_outmap[dir_outbits.value]);
  #else
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, (dir_outbits.value ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
  #endif
#endif
#endif
}

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
  #if !CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
    if(!tmc_enable.z)
        GPIOPinWrite(Z_ENABLE_PORT, Z_ENABLE_PIN, enable.z ? Z_ENABLE_PIN : 0);
    if(!tmc_enable.x)
        GPIOPinWrite(Z_ENABLE_PORT, Z_ENABLE_PIN, enable.z ? Z_ENABLE_PIN : 0);
  #endif
#elif defined(STEPPERS_ENABLE_PORT)
    GPIOPinWrite(STEPPERS_ENABLE_PORT, STEPPERS_ENABLE_PIN, enable.x ? STEPPERS_ENABLE_PIN : 0);
#else
    DIGITAL_OUT(XY_ENABLE_PORT, XY_ENABLE_PIN, enable.x);
    DIGITAL_OUT(Z_ENABLE_PORT,  Z_ENABLE_PIN, enable.z);
 #ifdef A_ENABLE_PIN
    DIGITAL_OUT(A_ENABLE_PORT, A_ENABLE_PIN, enable.a);
 #endif
 #ifdef B_ENABLE_PIN
    DIGITAL_OUT(B_ENABLE_PORT, B_ENABLE_PIN, enable.b);
 #endif
 #ifdef C_ENABLE_PIN
    DIGITAL_OUT(C_ENABLE_PORT, C_ENABLE_PIN, enable.c);
 #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_length);

#if LASER_PPI
    laser.next_pulse = 0;
#endif

    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, 5000);    // dummy...
    TimerEnable(STEPPER_TIMER_BASE, STEPPER_TIMER);

    hal.stepper.interrupt_callback(); // Start the show...
}

// Disables stepper driver interrupts and reset outputs
static void stepperGoIdle (bool clear_signals)
{
    TimerDisable(STEPPER_TIMER_BASE, STEPPER_TIMER);

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
#if USE_32BIT_TIMER
#if USE_PIOSC
// Limit min steps/s to about 2 (hal.f_step_timer @ 16 MHz)
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 20) ? cycles_per_tick : (1UL << 20) - 1UL);
  #else
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL);
  #endif
#else
// Limit min steps/s to about 2 (hal.f_step_timer @ 120 MHz)
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL);
  #else
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 26) ? cycles_per_tick : (1UL << 26) - 1UL);
  #endif
#endif
#else
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 16) ? cycles_per_tick : 0xFFFF);
#endif
}

#if !USE_32BIT_TIMER
// Sets up stepper driver interrupt timeout, "Normal" version
// TODO: can use stepperCyclesPerTick and 32bit timer?
static void stepperCyclesPerTickPrescaled (uint32_t cycles_per_tick)
{
    uint32_t prescaler;
    // Compute step timing and timer prescalar for normal step generation.
    if (cycles_per_tick < (1UL << 16)) // < 65536  (4.1ms @ 16MHz)
        prescaler = step_prescaler[0]; // prescaler: 0
    else if (cycles_per_tick < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prescaler = step_prescaler[1]; // prescaler: 8
        cycles_per_tick = cycles_per_tick >> 3;
    } else {
        prescaler = step_prescaler[2]; // prescaler: 64
        cycles_per_tick = cycles_per_tick >> 6;
    }
    TimerPrescaleSet(STEPPER_TIMER_BASE, STEPPER_TIMER, prescaler);
    TimerLoadSet(STEPPER_TIMER_BASE, STEPPER_TIMER, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFF /*Just set the slowest speed possible.*/);
}
#endif

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block) {
        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper_pulse_start;
            hal.stepper_pulse_start = stepperPulseStartSyncronized;
            stepperPulseStartSyncronized(stepper);
            return;
        }
        set_dir_outputs(stepper->dir_outbits);
    }
#else
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);
#endif

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block) {
        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper_pulse_start;
            hal.stepper_pulse_start = stepperPulseStartSyncronized;
            stepperPulseStartSyncronized(stepper);
            return;
        }
        set_dir_outputs(stepper->dir_outbits);
    }
#else
    if(stepper->dir_change) {

        set_dir_outputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            next_step_outbits = stepper->step_outbits; // Store out_bits
            IntRegister(PULSE_TIMER_INT, stepper_pulse_isr_delayed);
            TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_delay);
            TimerEnable(PULSE_TIMER_BASE, TIMER_A);
        }

       return;
    }
#endif

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}

#if SPINDLE_SYNC_ENABLE

#error Spindle sync code not ready!

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSyncronized (stepper_t *stepper)
{
    static spindle_sync_t spindle_sync;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper_pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper_pulse_start(stepper);
            return;
        } else {
            spindle_sync.dpp = stepper->exec_block->programmed_rate * 120.0f;
            spindle_sync.prev_pos = 0.0f;
            spindle_sync.timer_value_start = 123;
            spindle_sync.block_start = 2.33f;
            spindle_sync.segments = 0;
            spindle_sync.segment_id = stepper->exec_segment->id + 1; // force recalc
        }
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }

    if(spindle_sync.segment_id != stepper->exec_segment->id) {

        spindle_sync.segment_id = stepper->exec_segment->id;

        float dist = stepper->exec_segment->target_position - spindle_sync.prev_pos;

        float epulses = dist * spindle_sync.dpp;

        sys.pid_log.target[spindle_sync.segments] = stepper->exec_segment->target_position;

        spindle_sync.segments++;

 //       float current_pos = (spindleGetData(true).angular_position - spindle_sync.block_start) * stepper->exec_block->programmed_rate;

        spindle_sync.prev_pos = stepper->exec_segment->target_position;
    }
}
#endif

#if LASER_PPI

static void spindle_on ();

// Sets stepper direction and pulse pins and starts a step pulse with an initial delay
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartPPI (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
        uint_fast16_t steps_per_pulse = stepper->exec_block->steps_per_mm * 25.4f / laser.ppi;
        if(laser.next_pulse && laser.steps_per_pulse)
            laser.next_pulse = laser.next_pulse * steps_per_pulse / laser.steps_per_pulse;
        laser.steps_per_pulse = steps_per_pulse;
    }

    if(stepper->step_outbits.value) {
        if(stepper->spindle_pwm != current_pwm) {
            current_pwm = spindle_set_speed(stepper->spindle_pwm);
            laser.next_pulse = 0;
        }

        if(laser.next_pulse == 0) {
            laser.next_pulse = laser.steps_per_pulse;
            if(current_pwm != spindle_pwm.off_value) {
                spindle_on();
                TimerEnable(LASER_PPI_TIMER_BASE, TIMER_A);
                // TODO: T2CCP0 - use timer timeout to switch off CCP output w/o using interrupt? single shot PWM?
            }
        } else
            laser.next_pulse--;

        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}
#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    uint32_t i = limit_inputs.n_pins;

    on &= settings.limits.flags.hard_enabled;

    do {
        i--;
        GPIOIntClear(limit_inputs.pins.inputs[i].port, limit_inputs.pins.inputs[i].bit);       // Clear interrupt.
        if(on)
            GPIOIntEnable(limit_inputs.pins.inputs[i].port, limit_inputs.pins.inputs[i].bit);  // Enable interrupt.
        else
            GPIOIntDisable(limit_inputs.pins.inputs[i].port, limit_inputs.pins.inputs[i].bit); // Disable interrupt.
    } while(i);

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
    signals.min.value = settings.limits.invert.value;

#ifdef HWLIMIT_MASK
    uint32_t flags = GPIOPinRead(LIMIT_PORT, HWLIMIT_MASK);

    signals.min.x = !!(flags & X_LIMIT_BIT);
    signals.min.y = !!(flags & Y_LIMIT_BIT);
    signals.min.z = !!(flags & Z_LIMIT_BIT);
#else
    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_PIN);
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_PIN);
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_PIN);
#ifdef A_AXIS
    signals.min.a = DIGITAL_IN(A_LIMIT_PORT, A_LIMIT_PIN);
#endif
#ifdef B_AXIS
    signals.min.b = DIGITAL_IN(B_LIMIT_PORT, B_LIMIT_PIN);
#endif
#ifdef C_AXIS
    signals.min.c = DIGITAL_IN(C_LIMIT_PORT, C_LIMIT_PIN);
#endif
#endif

    if (settings.limits.invert.value)
        signals.min.value ^= settings.limits.invert.value;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

    signals.reset = DIGITAL_IN(RESET_PORT, RESET_PIN);
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_PIN);
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_PIN);
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
  #endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;
/*
    GPIOIntDisable(PROBE_PORT, PROBE_BIT);
    GPIOIntTypeSet(PROBE_PORT, PROBE_BIT, probe.inverted ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    if(probing)
        GPIOIntEnable(PROBE_PORT, PROBE_BIT);
        */
}

// Returns the probe connected and pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    //state.triggered = probeState; // TODO: check out using interrupt instead (we want to trap trigger and not risk losing it due to bouncing)
    state.triggered = DIGITAL_IN(PROBE_PORT, PROBE_PIN) ^ probe.inverted;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off ()
{
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle.invert.on);
}

inline static void spindle_on ()
{
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.spindle.invert.on);
}

inline static void spindle_dir (bool ccw)
{
    DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw));
}


// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

// Variable spindle control functions

// Sets spindle speed
#if PWM_RAMPED

static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwm_ramp.pwm_target = 0;
        pwm_ramp.pwm_step = -SPINDLE_RAMP_STEP_INCR;
        pwm_ramp.delay_ms = 0;
        pwm_ramp.ms_cfg = SPINDLE_RAMP_STEP_TIME;
        SysTickEnable();
     } else {

        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
            pwm_ramp.pwm_current = spindle_pwm.min_value;
            pwm_ramp.delay_ms = 0;
            TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_ramp.pwm_current + 15);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
//            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
        }
        pwm_ramp.pwm_target = pwm_value;
        pwm_ramp.pwm_step = pwm_ramp.pwm_target < pwm_ramp.pwm_current ? -SPINDLE_RAMP_STEP_INCR : SPINDLE_RAMP_STEP_INCR;
        pwm_ramp.ms_cfg = SPINDLE_RAMP_STEP_TIME;
        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
        SysTickEnable();
    }
}

#else

static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
        if(spindle_pwm.always_on) {
            TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.off_value >> 16);
            TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.off_value & 0xFFFF);
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.spindle.invert.pwm);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
        } else {
            uint_fast16_t pwm = spindle_pwm.period + 20000;
            TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm >> 16);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm & 0xFFFF);
            if(!pwmEnabled)
                TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B);                                   // Ensure PWM output is enabled to
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.spindle.invert.pwm);   // ensure correct output level.
            TimerDisable(SPINDLE_PWM_TIMER_BASE, TIMER_B);                                      // Disable PWM.
        }
     } else {
        TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value >> 16);
        TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value & 0xFFFF);
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
            TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period >> 16);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period & 0xFFFF);
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.spindle.invert.pwm);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
        }
    }
}

#endif

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else

static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif


// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || rpm == 0.0f) {
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
    } else {
        spindle_dir(state.ccw);
        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
    }
}

#if SPINDLE_SYNC_ENABLE

static spindle_data_t spindleGetData (spindle_data_request_t request)
{
    static spindle_data_t spindle_data;

    spindle_data.rpm = GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) ? 300.0f : 0.0f;
    spindle_data.angular_position = 0.0f;
    spindle_data.index_count++;

    return spindle_data;
}

static void spindleDataReset (void)
{
}

#endif

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.mask = settings.spindle.invert.mask;

    state.on = DIGITAL_IN(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
    if(hal.driver_cap.spindle_dir)
        state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);

    state.mask ^= settings.spindle.invert.mask;

    if(pwmEnabled)
        state.on = On;
#if PWM_RAMPED
    state.at_speed = pwm_ramp.pwm_current == pwm_ramp.pwm_target;
#endif
#if SPINDLE_SYNC_ENABLE
    state.at_speed = spindleGetData(SpindleData_RPM).rpm == (state.on ? 300.0f : 0.0f);
#endif

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.mask ^= settings.coolant_invert.mask;
    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, mode.flood);
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_PIN, mode.mist);
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = DIGITAL_IN(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
    state.mist  = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
    state.mask ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    IntMasterDisable();
    *ptr |= bits;
    IntMasterEnable();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    IntMasterDisable();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    IntMasterEnable();

    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    IntMasterDisable();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    IntMasterEnable();

    return prev;
}

static void enable_irq (void)
{
    IntMasterEnable();
}

static void disable_irq (void)
{
    IntMasterDisable();
}

#if MPG_MODE_ENABLE

static void modeSelect (bool mpg_mode)
{
    stream_enable_mpg(mpg_stream, mpg_mode);
}

static void modechange (void)
{
    modeSelect(GPIOPinRead(MODE_PORT, MODE_SWITCH_BIT) == 0);
    GPIOIntEnable(MODE_PORT, MODE_SWITCH_BIT);
}

#endif

#ifndef FreeRTOS

uint32_t getElapsedTicks (void)
{
    return elapsed_tics;
}

#endif

static irq_handler_t *get_handler (uint32_t port)
{
    uint32_t i = sizeof(irq_handler) / sizeof(irq_handler_t);
    do {
        if(irq_handler[--i].port == port)
            return &irq_handler[i];
    } while(i);

    return NULL;
}

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    spindle_pwm.offset = -1;
    hal.driver_cap.variable_spindle = spindle_precompute_pwm_values(&spindle_pwm, configCPU_CLOCK_HZ);

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(step_outmap); i++)
        step_outmap[i] = c_step_outmap[i ^ settings->step_invert.mask];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap); i++)
        dir_outmap[i] = c_dir_outmap[i ^ settings->steppers.dir_invert.mask & 0x07];
#if CNC_BOOSTERPACK2
    for(i = 0; i < sizeof(dir_outmap2); i++)
        dir_outmap2[i] = c_dir_outmap2[i ^ settings->steppers.dir_invert.mask >> 3];
#endif
#endif

    if(IOInitDone) {

#if ETHERNET_ENABLE

        static bool enet_ok = false;
        if(!enet_ok)
            enet_ok = enet_start();

#endif

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {
            TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period >> 16);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period & 0xFFFF);
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        pulse_length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            int32_t delay = (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 1.2f)) - 1;
            pulse_delay = delay < 2 ? 2 : delay;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        TimerIntRegister(PULSE_TIMER_BASE, TIMER_A, stepper_pulse_isr);
        TimerIntEnable(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);

      #if LASER_PPI
        if(!settings->flags.laser_mode)
            laser_ppi_mode(false);
      #endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *input;
        irq_handler_t *handler;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {

            pullup = true;
            input = &inputpin[--i];
            input->bit = 1U << input->pin;
            input->irq_mode = IRQ_Mode_None;
            pullup = input->group == PinGroup_AuxInput;

            switch(input->id) {

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    input->irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    input->irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    input->irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    input->irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    break;

                case Input_LimitX:
                    pullup = !settings->limits.disable_pullup.x;
                    input->irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                    pullup = !settings->limits.disable_pullup.y;
                    input->irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                    pullup = !settings->limits.disable_pullup.z;
                    input->irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitA:
                    pullup = !settings->limits.disable_pullup.a;
                    input->irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitB:
                    pullup = !settings->limits.disable_pullup.b;
                    input->irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitC:
                    pullup = !settings->limits.disable_pullup.c;
                    input->irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_ModeSelect:
                    input->irq_mode = IRQ_Mode_Change;
                    break;

                case Input_KeypadStrobe:
                    pullup = true;
                    input->irq_mode = IRQ_Mode_Change;
                    break;
            }

            if(input->group == PinGroup_AuxInput) {
                pullup = true;
                input->cap.pull_mode = PullMode_Up|PullMode_Down;
                input->cap.irq_mode = IRQ_Mode_Rising|IRQ_Mode_Falling;
            }

            input->debounce = hal.driver_cap.software_debounce && (input->group == PinGroup_Limit || input->group == PinGroup_Control);

            GPIOIntDisable(input->port, input->bit);    // Disable pin change interrupt
            GPIOPinTypeGPIOInput(input->port, input->bit);
            GPIOPadConfigSet(input->port, input->bit, GPIO_STRENGTH_2MA, pullup ? GPIO_PIN_TYPE_STD_WPU : GPIO_PIN_TYPE_STD_WPD);
            GPIOIntClear(input->port, input->bit);     // Clear any pending interrupt

            if(input->irq_mode != IRQ_Mode_None || input->group == PinGroup_AuxInput) {

                if((handler = get_handler(input->port)))
                    memcpy(&handler->pins[handler->count++], input, sizeof(input_signal_t));

                if(input->group != PinGroup_AuxInput) {
                    GPIOIntTypeSet(input->port, input->bit, input->irq_mode == IRQ_Mode_Falling ? GPIO_FALLING_EDGE : (input->irq_mode == IRQ_Mode_Change ? GPIO_BOTH_EDGES : GPIO_RISING_EDGE));
                    GPIOIntEnable(input->port, input->bit);    // Enable pin change interrupt for control pins
                }
                if(input->irq_mode != IRQ_Mode_Change)
                    input->active = input->active ^ (input->irq_mode == IRQ_Mode_Falling ? 0 : 1);
            }

        } while(i);


        i = sizeof(irq_handler) / sizeof(irq_handler_t);
        do {
            if(irq_handler[--i].count)
                GPIOIntRegister(irq_handler[i].port, irq_handler[i].handler);
        } while(i);

#if MPG_MODE_ENABLE
       if(sys.mpg_mode != !(GPIOPinRead(MODE_PORT, MODE_SWITCH_BIT) == MODE_SWITCH_BIT))
            modeSelect(true);
       GPIOIntEnable(MODE_PORT, MODE_SWITCH_BIT);
#endif

    }
}

static char *port2char (uint32_t port)
{
    switch(port) {

        case(GPIO_PORTA_BASE):
            return "PA";

        case(GPIO_PORTB_BASE):
            return "PB";

        case(GPIO_PORTC_BASE):
            return "PC";

        case(GPIO_PORTD_BASE):
            return "PD";

        case(GPIO_PORTE_BASE):
            return "PE";

        case(GPIO_PORTF_BASE):
            return "PF";

        case(GPIO_PORTG_BASE):
            return "PG";

        case(GPIO_PORTH_BASE):
            return "PH";

        case(GPIO_PORTK_BASE):
            return "PK";

        case(GPIO_PORTL_BASE):
            return "PL";

        case(GPIO_PORTM_BASE):
            return "PM";

        case(GPIO_PORTN_BASE):
            return "PN";

        case(GPIO_PORTP_BASE):
            return "PP";

        case(GPIO_PORTQ_BASE):
            return "PQ";
    }

    return "?";
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info)
{
    static xbar_t pin = {0};
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin);
    };
/*
    for(i = 0; i < sizeof(peripin) / sizeof(output_signal_t); i++) {
        pin.pin = peripin[i].pin;
        pin.function = peripin[i].id;
        pin.mode.output = PIN_ISOUTPUT(pin.function);
        pin.group = peripin[i].group;
        pin.port = low_level ? (void *)peripin[i].port : (void *)port2char(peripin[i].port);

        pin_info(&pin);
    }; */
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        GPIOPinTypeGPIOOutput(output->port, 1<<output->pin);
        if(output->group == PinGroup_StepperPower) {
            GPIOPadConfigSet(output->port, 1<<output->pin, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
            GPIOPinWrite(STEPPERS_VDD_PORT, 1<<output->pin, 1<<output->pin);
        } else
            GPIOPadConfigSet(output->port, 1<<output->pin, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    }

    /******************
     *  Stepper init  *
     ******************/

    // Configure stepper driver timer
#if USE_32BIT_TIMER
  #if USE_PIOSC
    TimerClockSourceSet(STEPPER_TIMER_BASE, TIMER_CLOCK_PIOSC);
  #endif
    TimerConfigure(STEPPER_TIMER_BASE, TIMER_CFG_PERIODIC);
#else
    TimerConfigure(STEPPER_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    TimerPrescaleSet(STEPPER_TIMER_BASE, TIMER_A, STEPPER_DRIVER_PRESCALER); // 20 MHz step timer clock
#endif
    IntPrioritySet(STEPPER_TIMER_INT, 0x20);                    // lower priority than for step timer (which resets the step-dir signal)
    TimerControlStall(STEPPER_TIMER_BASE, STEPPER_TIMER, true); // timer will stall in debug mode
    TimerIntRegister(STEPPER_TIMER_BASE, STEPPER_TIMER, stepper_driver_isr);
    TimerIntClear(STEPPER_TIMER_BASE, 0xFFFF);
    IntPendClear(STEPPER_TIMER_INT);
    TimerIntEnable(STEPPER_TIMER_BASE, TIMER_TIMA_TIMEOUT);

    // Configure step pulse timer
//  TimerClockSourceSet(PULSE_TIMER_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(PULSE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
    IntPrioritySet(PULSE_TIMER_INT, 0x00);              // highest priority - higher than for stepper timer
    TimerControlStall(PULSE_TIMER_BASE, TIMER_A, true); // timer will stall in debug mode
    TimerIntClear(PULSE_TIMER_BASE, 0xFFFF);
    IntPendClear(PULSE_TIMER_INT);
    TimerPrescaleSet(PULSE_TIMER_BASE, TIMER_A, STEPPER_PULSE_PRESCALER); // for 0.1uS per count

#if LASER_PPI

   /********************************
    *  PPI mode pulse width timer  *
    ********************************/

    laser.ppi = 600.0f;
    laser.pulse_length = 1500;

    SysCtlPeripheralEnable(LASER_PPI_TIMER_PERIPH);
    SysCtlDelay(26); // wait a bit for peripherals to wake up
    TimerConfigure(LASER_PPI_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
    IntPrioritySet(LASER_PPI_TIMER_INT, 0x40);              // lower priority than for step timer (which resets the step-dir signal)
    TimerControlStall(LASER_PPI_TIMER_BASE, TIMER_A, true); // timer will stall in debug mode
    TimerIntClear(LASER_PPI_TIMER_BASE, 0xFFFF);
    IntPendClear(LASER_PPI_TIMER_INT);
    TimerPrescaleSet(LASER_PPI_TIMER_BASE, TIMER_A, 79); // for 1uS per count
    TimerIntRegister(LASER_PPI_TIMER_BASE, TIMER_A, ppi_timeout_isr);
    TimerLoadSet(LASER_PPI_TIMER_BASE, TIMER_A, laser.pulse_length);
    TimerIntEnable(LASER_PPI_TIMER_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);

#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce) {
        SysCtlPeripheralEnable(DEBOUNCE_TIMER_PERIPH);
        SysCtlDelay(26); // wait a bit for peripherals to wake up
        IntPrioritySet(DEBOUNCE_TIMER_INT, 0x40); // lower priority than for Timer2 (which resets the step-dir signal)
        TimerConfigure(DEBOUNCE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
        TimerControlStall(DEBOUNCE_TIMER_BASE, TIMER_A, true); //timer2 will stall in debug mode
        TimerIntRegister(DEBOUNCE_TIMER_BASE, TIMER_A, software_debounce_isr);
        TimerIntClear(DEBOUNCE_TIMER_BASE, 0xFFFF);
        IntPendClear(DEBOUNCE_TIMER_INT);
        TimerPrescaleSet(DEBOUNCE_TIMER_BASE, TIMER_A, 119); // configure for 1us per count
        TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // and for a total of 32ms
        TimerIntEnable(DEBOUNCE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    }


#if !USE_32BIT_TIMER
    if(hal.driver_cap.amass_level == 0)
        hal.stepper_cycles_per_tick = &stepperCyclesPerTickPrescaled;
#endif

   /******************
    *  Spindle init  *
    ******************/

    SysCtlPeripheralEnable(SPINDLE_PWM_TIMER_PERIPH);
    SysCtlDelay(26); // wait a bit for peripherals to wake up
    TimerClockSourceSet(SPINDLE_PWM_TIMER_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(SPINDLE_PWM_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
    TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
    GPIOPinConfigure(SPINDLE_PWM_MAP);
    GPIOPinTypeTimer(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT);
    GPIOPadConfigSet(SPINDLE_PWM_PORT, SPINDLE_PWM_BIT, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  #if PWM_RAMPED
    pwm_ramp.ms_cfg = pwm_ramp.pwm_current = pwm_ramp.pwm_target = 0;
  #endif

#if SDCARD_ENABLE
  #ifdef __MSP432E401Y__
    SDFatFS_init();
  #endif
    sdcard_init();
  #if defined(FreeRTOS) && !defined(__MSP432E401Y__)
    TimerHandle_t xFatFsTimer = xTimerCreate("fatfsTimer", pdMS_TO_TICKS(10), pdTRUE, NULL, fatfsTimerCallback);
    xTimerStart(xFatFsTimer, 10);
  #endif
#endif

#if KEYPAD_ENABLE

   /*********************
    *  I2C KeyPad init  *
    *********************/

    GPIOPinTypeGPIOInput(KEYINTR_PORT, KEYINTR_BIT);
    GPIOPadConfigSet(KEYINTR_PORT, KEYINTR_BIT, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // -> WPU

    GPIOIntRegister(KEYINTR_PORT, keyclick_int_handler);
    GPIOIntTypeSet(KEYINTR_PORT, KEYINTR_BIT, GPIO_BOTH_EDGES);
    GPIOIntEnable(KEYINTR_PORT, KEYINTR_BIT);

#endif

  // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    set_dir_outputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
#ifdef FreeRTOS
    hal.f_step_timer = configCPU_CLOCK_HZ;
    xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(10), pdFALSE, NULL, vTimerCallback);
#else
    FPUEnable();
    FPULazyStackingEnable();
    hal.f_step_timer = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480, 120000000);

    // Set up systick timer with a 1ms period
    SysTickPeriodSet((hal.f_step_timer / 1000) - 1);
    SysTickIntRegister(systick_isr);
    IntPrioritySet(FAULT_SYSTICK, 0x40);
    SysTickIntEnable();
    SysTickEnable();
#endif

#if USE_32BIT_TIMER && USE_PIOSC
    // Enable hibernation module, used to calibrate 16 MHZ PIOSC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
    HibernateEnableExpClk(hal.f_step_timer);
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCEnable();
#endif

    // GPIO enable

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(STEPPER_TIMER_PERIPH);
    SysCtlPeripheralEnable(PULSE_TIMER_PERIPH);

    SysCtlDelay(26); // wait a bit for peripherals to wake up

#ifndef __MSP432E401Y__
    // Unlock GPIOF0, used for stepper disable Z control
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
#endif

    // Unlock GPIOD7, used for disable XY
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
    EEPROMInit();

#if USE_32BIT_TIMER && USE_PIOSC
    // Moved after EEPROM enable for giving time for 32768 Hz crystal to stabilize
    SysCtlPIOSCCalibrate(SYSCTL_PIOSC_CAL_AUTO);
//    HibernateRTCDisable();
//    HibernateClockConfig(HIBERNATE_OSC_DISABLE);
//    HibernateDisable();
#endif

#if I2C_ENABLE
    I2CInit();
#endif

#ifdef __MSP432E401Y__
  #ifdef FreeRTOS
    hal.info = "MSP432E401Y FreeRTOS";
  #else
    hal.info = "MSP432E401Y";
  #endif
#else // TM4C1294
  #ifdef FreeRTOS
    hal.info = "TM4C1294NCPDT FreeRTOS";
  #else
    hal.info = "TM4C1294NCPDT";
  #endif
#endif
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_version = "210704";
    hal.driver_setup = driver_setup;
#if !USE_32BIT_TIMER
    hal.f_step_timer = hal.f_step_timer / (STEPPER_DRIVER_PRESCALER + 1);
#elif USE_PIOSC
    hal.f_step_timer = 16000000UL;
#endif
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;

    hal.spindle.set_state = spindleSetStateVariable;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif
#if SPINDLE_SYNC_ENABLE
    hal.spindle.get_data = spindleGetData;
    hal.spindle.reset_data = spindleDataReset;
#endif

    hal.control.get_state = systemGetState;

    serial_stream = serialInit();

    hal.stream_select = selectStream;
    hal.stream_select(serial_stream);

    eeprom_init();

    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.enumerate_pins = enumeratePins;
#ifdef FreeRTOS
    hal.get_elapsed_ticks = xTaskGetTickCountFromISR;
#else
    hal.get_elapsed_ticks = getElapsedTicks;
#endif

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#ifdef _ATC_H_
    hal.tool_select = atc_tool_selected;
    hal.tool_change = atc_tool_change;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_BIT
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
#if PWM_RAMPED
    hal.driver_cap.spindle_at_speed = On;
#endif
#if SPINDLE_SYNC_ENABLE
    hal.driver_cap.spindle_sync = On;
    hal.driver_cap.spindle_at_speed = On;
#endif
    hal.driver_cap.mist_control = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#if LASER_PPI
    hal.driver_cap.laser_ppi_mode = On;
#endif
#if MPG_MODE_ENABLE
    hal.driver_cap.mpg_mode = On;
    mpg_stream = serial2Init();
#endif

    uint32_t i;
    input_signal_t *input;
    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];

        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            aux_inputs.n_pins++;
        }

        if(input->group == PinGroup_Limit) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        }
    }

    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];

        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            aux_outputs.n_pins++;
        }
    }

#ifdef HAS_IOPORTS
    ioports_init(&aux_inputs, &aux_outputs);
#endif

#if ETHERNET_ENABLE
    enet_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

    my_plugin_init();

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void)
{
    TimerIntClear(STEPPER_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    hal.stepper.interrupt_callback();
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/
// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
// NOTE: TivaC has a shared interrupt for match and timeout
static void stepper_pulse_isr (void)
{
    TimerIntClear(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    set_step_outputs((axes_signals_t){0});
}

static void stepper_pulse_isr_delayed (void)
{
    TimerIntClear(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(PULSE_TIMER_INT, stepper_pulse_isr);

    set_step_outputs(next_step_outbits);

    TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_length);
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}

#if LASER_PPI

void laser_ppi_mode (bool on)
{
    if(on)
        hal.stepper_pulse_start = stepperPulseStartPPI;
    else
        hal.stepper_pulse_start = settings.pulse_delay_microseconds > 0.0f ? stepperPulseStartDelayed : stepperPulseStart;
    gc_set_laser_ppimode(on);
}

// Switches off the spindle (laser) after laser.pulse_length time has elapsed
static void ppi_timeout_isr (void)
{
    TimerIntClear(LASER_PPI_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    spindle_off();
}
#endif

inline static bool enqueue_debounce (input_signal_t *signal)
{
    bool ok;
    uint_fast8_t bptr = (debounce_queue.head + 1) & (DEBOUNCE_QUEUE - 1);

    if((ok = bptr != debounce_queue.tail)) {
        debounce_queue.signal[debounce_queue.head] = signal;
        debounce_queue.head = bptr;
    }

    return ok;
}

// Returns NULL if no debounce checks enqueued
inline static input_signal_t *get_debounce (void)
{
    input_signal_t *signal = NULL;
    uint_fast8_t bptr = debounce_queue.tail;

    if(bptr != debounce_queue.head) {
        signal = debounce_queue.signal[bptr++];
        debounce_queue.tail = bptr & (DEBOUNCE_QUEUE - 1);
    }

    return signal;
}

void software_debounce_isr (void)
{
    uint32_t grp = 0;
    input_signal_t *signal;

    TimerIntClear(DEBOUNCE_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

    while((signal = get_debounce())) {

        GPIOIntClear(signal->port, signal->bit);
        GPIOIntEnable(signal->port, signal->bit);

        if(!!GPIOPinRead(signal->port, signal->bit) == (signal->irq_mode == IRQ_Mode_Falling ? 0 : 1))
            grp |= signal->group;
    }

    if(grp & PinGroup_Limit) {
        limit_signals_t state = limitsGetState();

        if(limit_signals_merge(state).mask)
            hal.limits.interrupt_callback(state);
    }

    if(grp & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());
}

static /* inline __attribute__((always_inline))*/ IRQHandler (input_signal_t *input, uint16_t iflags)
{
    bool debounce = false;
    uint32_t groups = 0;

    while(input->port) {
        if(iflags & input->bit) {

            if(input->debounce && (debounce = enqueue_debounce(input)))
                GPIOIntDisable(input->port, input->bit);    // Disable pin change interrupt

            else switch(input->group) {

#if MPG_MODE_ENABLE
                case PinGroup_MPG:
//                    if(delay.ms == 0) // Ignore if delay is active
//                        driver_delay_ms(50, modeChange);
                    break;
#endif

#if KEYPAD_ENABLE
                case PinGroup_Keypad:
                    keypad_keyclick_handler(!GPIOPinRead(signal->port, signal->bit));
                    break;
#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C
                case PinGroup_Motor_Warning:
                    trinamic_warn_handler();
                    break:

                case PinGroup_Motor_Fault:
                    trinamic_fault_handler();
                    break;
#endif

                case PinGroup_AuxInput:
                    ioports_event(input);
                    break;
/*
                case PinGroup_Probe:
                    if(!probe.triggered) {
                        probe.triggered = On;
                        if(!probe.is_probing) {
                            if(probe.connected && elapsed_tics > ms)
                                hal.control_interrupt_callback((control_signals_t){ .probe_triggered = On });
                        } else
                            ms = elapsed_tics + 300;
                    }
                    break;
*/
                default:
                    groups |= input->group;
                    break;
            }
        }
        input++;
    }

    if(debounce) {
        TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
        TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
    }

    if(groups & PinGroup_Limit) {
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value)
            hal.limits.interrupt_callback(state);
    }

    if(groups & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());
}

static void port_a_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTA_BASE, true);
    GPIOIntClear(GPIO_PORTA_BASE, iflags);

    IRQHandler(irq_handler[0].pins, iflags);
}

static void port_b_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTB_BASE, true);
    GPIOIntClear(GPIO_PORTB_BASE, iflags);

    IRQHandler(irq_handler[1].pins, iflags);
}

static void port_c_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTC_BASE, true);
    GPIOIntClear(GPIO_PORTC_BASE, iflags);

    IRQHandler(irq_handler[2].pins, iflags);
}

static void port_d_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTD_BASE, true);
    GPIOIntClear(GPIO_PORTD_BASE, iflags);

    IRQHandler(irq_handler[3].pins, iflags);
}

static void port_e_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTE_BASE, true);
    GPIOIntClear(GPIO_PORTE_BASE, iflags);

    IRQHandler(irq_handler[4].pins, iflags);
}

static void port_f_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTF_BASE, true);
    GPIOIntClear(GPIO_PORTF_BASE, iflags);

    IRQHandler(irq_handler[5].pins, iflags);
}

static void port_g_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTG_BASE, true);
    GPIOIntClear(GPIO_PORTG_BASE, iflags);

    IRQHandler(irq_handler[6].pins, iflags);
}

static void port_h_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTH_BASE, true);
    GPIOIntClear(GPIO_PORTH_BASE, iflags);

    IRQHandler(irq_handler[7].pins, iflags);
}

static void port_k_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTK_BASE, true);
    GPIOIntClear(GPIO_PORTK_BASE, iflags);

    IRQHandler(irq_handler[8].pins, iflags);
}

static void port_l_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTL_BASE, true);
    GPIOIntClear(GPIO_PORTL_BASE, iflags);

    IRQHandler(irq_handler[9].pins, iflags);
}

static void port_m_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTM_BASE, true);
    GPIOIntClear(GPIO_PORTM_BASE, iflags);

    IRQHandler(irq_handler[10].pins, iflags);
}

static void port_n_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTN_BASE, true);
    GPIOIntClear(GPIO_PORTN_BASE, iflags);

    IRQHandler(irq_handler[11].pins, iflags);
}

static void port_p_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTP_BASE, true);
    GPIOIntClear(GPIO_PORTP_BASE, iflags);

    IRQHandler(irq_handler[12].pins, iflags);
}

static void port_q_isr (void)
{
    uint32_t iflags = GPIOIntStatus(GPIO_PORTQ_BASE, true);
    GPIOIntClear(GPIO_PORTQ_BASE, iflags);

    IRQHandler(irq_handler[13].pins, iflags);
}

#ifndef FreeRTOS
// Interrupt handler for 1 ms interval timer
#if PWM_RAMPED
static void systick_isr (void)
{
    elapsed_tics++;
    
    if(pwm_ramp.ms_cfg) {
        if(++pwm_ramp.delay_ms == pwm_ramp.ms_cfg) {

            pwm_ramp.delay_ms = 0;
            pwm_ramp.pwm_current += pwm_ramp.pwm_step;

            if(pwm_ramp.pwm_step < 0) { // decrease speed

                if(pwm_ramp.pwm_current < pwm_ramp.pwm_target)
                    pwm_ramp.pwm_current = pwm_ramp.pwm_target;

                if(pwm_ramp.pwm_current == 0) { // stop?
                    if(settings.flags.spindle_disable_with_zero_speed)
                        spindle_off();
                    TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period + 20000);
                    TimerDisable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Disable PWM. Output voltage is zero.
                    if(pwmEnabled)
                        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, true);
                    pwmEnabled = false;
                } else
                    TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_ramp.pwm_current); // use LUT?
            } else {
                 if(pwm_ramp.pwm_current > pwm_ramp.pwm_target)
                     pwm_ramp.pwm_current = pwm_ramp.pwm_target;
                TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_ramp.pwm_current); // use LUT?
            }
            if(pwm_ramp.pwm_current == pwm_ramp.pwm_target)
                pwm_ramp.ms_cfg = 0;
        }
    }

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
#else
static void systick_isr (void)
{
    elapsed_tics++;

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
#endif
#endif // FreeRTOS
