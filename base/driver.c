/*
  driver.c - main driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2018-2025 Terje Io

  Some parts
   Copyright (c) 2011-2015 Sungeun K. Jeon
   Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <stdlib.h>

#include "driver.h"
#include "eeprom.h"
#include "serial.h"

#define AUX_DEVICES // until all drivers are converted?
#ifndef AUX_CONTROLS
#define AUX_CONTROLS (AUX_CONTROL_SPINDLE|AUX_CONTROL_COOLANT)
#endif

#include "grbl/machine_limits.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/motor_pins.h"
#include "grbl/pin_bits_masks.h"

#ifdef FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#endif

#if TRINAMIC_ENABLE
static void trinamic_diag1_isr (void);
#endif

#ifdef I2C_ENABLE
#include "i2c.h"
#endif

#if SDCARD_ENABLE && defined(__MSP432E401Y__)
#include "sdcard/sdcard.h"
#include <ti/drivers/SDFatFS.h>
#include <ti/boards/MSP_EXP432E401Y/Board.h>
#endif

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#endif

#if ETHERNET_ENABLE
  #include "shared/ethernet/enet.h"
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

static laser_ppi_t laser;
static spindle_ptrs_t *ppi_spindle;

static void ppi_timeout_isr (void);

#endif

#ifdef MPG_MODE_PIN
#define MPG_MODE_BIT (1<<MPG_MODE_PIN)
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

static periph_signal_t *periph_pins = NULL;
#if AUX_CONTROLS_ENABLED
static input_signal_t *door_pin;
static uint8_t probe_port;
static pin_debounce_t debounce;
static void aux_irq_handler (uint8_t port, bool state);
#endif

static input_signal_t inputpin[] = {
    { .id = Input_Reset,          .port = RESET_PORT,         .pin = RESET_PIN,           .group = PinGroup_Control },
    { .id = Input_FeedHold,       .port = FEED_HOLD_PORT,     .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
    { .id = Input_CycleStart,     .port = CYCLE_START_PORT,   .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
#if SAFETY_DOOR_BIT
    { .id = Input_SafetyDoor,     .port = SAFETY_DOOR_PORT,   .pin = SAFETY_DOOR_PIN,     .group = PinGroup_Control },
#endif
    { .id = Input_Probe,          .port = PROBE_PORT,         .pin = PROBE_PIN,           .group = PinGroup_Probe },
#ifdef I2C_STROBE_PIN
    { .id = Input_KeypadStrobe,   .port = I2C_STROBE_PORT,    .pin = I2C_STROBE_PIN,      .group = PinGroup_Keypad },
#endif
#ifdef MPG_MODE_PIN
    { .id = Input_ModeSelect,     .port = MPG_MODE_PORT,      .pin = MPG_MODE_PIN,        .group = PinGroup_MPG },
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
#ifdef AUXINPUT7_PIN
  , { .id = Input_Aux7,           .port = AUXINPUT7_PORT,     .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput }
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
#if !(AUX_CONTROLS & AUX_CONTROL_SPINDLE)
    { .id = Output_SpindleOn,       .port = SPINDLE_ENABLE_PORT,    .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
    { .id = Output_SpindleDir,      .port = SPINDLE_DIRECTION_PORT, .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
#if !(AUX_CONTROLS & AUX_CONTROL_COOLANT)
    { .id = Output_CoolantFlood,    .port = COOLANT_FLOOD_PORT,     .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
    { .id = Output_CoolantMist,     .port = COOLANT_MIST_PORT,      .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef AUXOUTPUT0_PORT
    { .id = Output_Aux0,            .port = AUXOUTPUT0_PORT,        .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PORT
    { .id = Output_Aux1,            .port = AUXOUTPUT1_PORT,        .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PORT
    { .id = Output_Aux2,            .port = AUXOUTPUT2_PORT,        .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PORT
    { .id = Output_Aux3,            .port = AUXOUTPUT3_PORT,        .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PORT
    { .id = Output_Aux4,            .port = AUXOUTPUT4_PORT,        .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PORT
    { .id = Output_Aux5,            .port = AUXOUTPUT5_PORT,        .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PORT
    { .id = Output_Aux6,            .port = AUXOUTPUT6_PORT,        .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PORT
    { .id = Output_Aux7,            .port = AUXOUTPUT7_PORT,        .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput }
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
static bool IOInitDone = false;
static uint32_t pulse_length, pulse_delay;
#ifndef FreeRTOS
static volatile uint32_t elapsed_tics = 0;
#endif
static axes_signals_t next_step_out;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static probe_state_t probe = {
    .connected = On
};
#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
#if DRIVER_SPINDLE_PWM_ENABLE
static spindle_pwm_t spindle_pwm;
#endif // DRIVER_SPINDLE_PWM_ENABLE
#endif // DRIVER_SPINDLE_ENABLE

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

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif

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

#if SDCARD_ENABLE

#if defined(__MSP432E401Y__)

static char *sdcard_mount (FATFS **fs)
{
    SDFatFS_Handle fsh = SDFatFS_open(Board_SDFatFS0, 0);

    if(fsh)
        *fs = &fsh->object->filesystem;

    return "";
}

static bool sdcard_unmount (FATFS **fs)
{
    return false; // for now
}

#else

void fatfsTimerCallback (TimerHandle_t xTimer)
{
    disk_timerproc();
}

#endif

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
static void stepperEnable (axes_signals_t enable, bool hold)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_MOTOR_ENABLE
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
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);

    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, hal.f_step_timer / 500); // ~2ms delay to allow drivers time to wake up.
    TimerEnable(STEPPER_TIMER_BASE, STEPPER_TIMER);
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
    if(stepper->dir_changed.bits) {
        stepper->dir_changed.bits = 0;
        set_dir_outputs(stepper->dir_out);
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_changed.bits) {

        set_dir_outputs(stepper->dir_out);

        if(stepper->step_out.bits) {

            if(stepper->step_out.bits & stepper->dir_changed.bits) {
                next_step_out = stepper->step_out; // Store out_bits
                IntRegister(PULSE_TIMER_INT, stepper_pulse_isr_delayed);
                TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_delay);
                TimerEnable(PULSE_TIMER_BASE, TIMER_A);
            } else {
                set_step_outputs(stepper->step_out);
                TimerEnable(PULSE_TIMER_BASE, TIMER_A);
            }
        }

        stepper->dir_changed.bits = 0;

        return;
    }

    if(stepper->step_out.bits) {
        set_step_outputs(stepper->step_out);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}

#if LASER_PPI

static void spindle_on ();

// Sets stepper direction and pulse pins and starts a step pulse with an initial delay
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartPPI (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_out);
        uint_fast16_t steps_per_pulse = stepper->exec_block->steps_per_mm * 25.4f / laser.ppi;
        if(laser.next_pulse && laser.steps_per_pulse)
            laser.next_pulse = laser.next_pulse * steps_per_pulse / laser.steps_per_pulse;
        laser.steps_per_pulse = steps_per_pulse;
    }

    if(stepper->step_out.bits) {
        if(stepper->spindle_pwm != current_pwm) {
            current_pwm = spindleSetSpeed(ppi_spindle, &spindle_pwm);
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

        set_step_outputs(stepper->step_out);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}
#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    axes_signals_t pin;
    input_signal_t *limit;
    uint_fast8_t idx = limit_inputs.n_pins;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        limit = &limit_inputs.pins.inputs[--idx];
        if(limit->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(on && homing_cycle.mask) {
                pin = xbar_fn_to_axismask(limit->id);
                disable = limit->group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
            }
            if(disable)
                GPIOIntDisable(limit->port, limit->bit);    // Disable pin change interrupt.
            else {
                GPIOIntClear(limit->port, limit->bit);      // Clear and
                GPIOIntEnable(limit->port, limit->bit);     // enable pin change interrupt.
            }
       }
    } while(idx);
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
  #if SAFETY_DOOR_BIT
    signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
  #endif

#if AUX_CONTROLS_ENABLED

  #ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN);
  #endif
  #ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN);
  #endif
  #ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(MOTOR_WARNING_PORT, MOTOR_WARNING_PIN);
  #endif

  #if AUX_CONTROLS_SCAN
    signals = aux_ctrl_scan_status(signals);
  #endif

#endif // AUX_CONTROLS_ENABLED

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe.connected = !probe.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

    if(hal.signals_cap.probe_triggered) {
        probe.is_probing = Off;
        probe.triggered = hal.probe.get_state().triggered;
        pin_irq_mode_t irq_mode = probing && !probe.triggered ? (probe.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising) : IRQ_Mode_None;
        probe.irq_enabled = hal.port.register_interrupt_handler(probe_port, irq_mode, aux_irq_handler) && irq_mode != IRQ_Mode_None;
    }

    if(!probe.irq_enabled)
        probe.triggered = Off;

    probe.is_probing = probing;
}

// Returns the probe connected and pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = probe.is_probing && probe.irq_enabled ? probe.triggered : DIGITAL_IN(PROBE_PORT, PROBE_PIN) ^ probe.inverted;

    return state;
}

#if AUX_CONTROLS_ENABLED

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *pin;
    control_signals_t signals = {0};

    if((pin = aux_ctrl_get_pin(port))) {
        switch(pin->function) {
#ifdef SAFETY_DOOR_PIN
            case Input_SafetyDoor:
                if(debounce.safety_door)
                    return;
                if((debounce.safety_door = enqueue_debounce(door_pin))) {
                    TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
                    TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
                    return;
                }
                break;
#endif
#ifdef PROBE_PIN
            case Input_Probe:
                if(probe.is_probing) {
                    probe.triggered = On;
                    return;
                } else
                    signals.probe_triggered = On;
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(I2C_STROBE_PORT, I2C_STROBE_PIN) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                protocol_enqueue_foreground_task(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= pin->cap.mask;
        if(pin->irq_mode == IRQ_Mode_Change && pin->function != Input_Probe)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, pin->aux_port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Input, &aux_ctrl->aux_port, NULL))) {
        ioport_set_function(pin, aux_ctrl->function, &aux_ctrl->cap);
#ifdef PROBE_PIN
        if(aux_ctrl->function == Input_Probe) {
            probe_port = aux_ctrl->aux_port;
            hal.probe.get_state = probeGetState;
            hal.probe.configure = probeConfigure;
            hal.probe.connected_toggle = probeConnectedToggle;
            hal.driver_cap.probe_pull_up = On;
            hal.signals_cap.probe_triggered = hal.driver_cap.probe_latch = (pin->cap.irq_mode & aux_ctrl->irq_mode) == aux_ctrl->irq_mode;
        }
#endif
#ifdef SAFETY_DOOR_PIN
        if(aux_ctrl->function == Input_SafetyDoor)
            door_pin = (input_signal_t *)aux_ctrl->input;
#endif
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#endif // AUX_CONTROLS_ENABLED

#if AUX_CONTROLS

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->aux_port, NULL)))
        ioport_set_function(pin, aux_ctrl->function, NULL);
    else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#endif // AUX_CONTROLS

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = Off;
#ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, settings.pwm_spindle.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
    }
#elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = On;
#ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, !settings.pwm_spindle.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
    }
#elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_dir (bool ccw)
{
    DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, (ccw ^ settings.pwm_spindle.invert.ccw));
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(rpm);

    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

#if DRIVER_SPINDLE_PWM_ENABLE

// Variable spindle control functions

// Sets spindle speed
#if PWM_RAMPED

static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if (pwm_value == spindle->context.pwm->off_value) {
        pwm_ramp.pwm_target = 0;
        pwm_ramp.pwm_step = -SPINDLE_RAMP_STEP_INCR;
        pwm_ramp.delay_ms = 0;
        pwm_ramp.ms_cfg = SPINDLE_RAMP_STEP_TIME;
        SysTickEnable();
     } else {

        if(!spindle->context.pwm->flags.enable_out) {
            spindle_on(spindle);
            spindle->context.pwm->flags.enable_out = true;
            pwm_ramp.pwm_current = spindle->context.pwm->min_value;
            pwm_ramp.delay_ms = 0;
            TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle->context.pwm->period - pwm_ramp.pwm_current + 15);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle->context.pwm->period);
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

static void pwm_off (spindle_ptrs_t *spindle)
{
    if(spindle->context.pwm->flags.always_on) {
        TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle->context.pwm->off_value >> 16);
        TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle->context.pwm->off_value & 0xFFFF);
        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.pwm_spindle.invert.pwm);
        TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
    } else {
        uint_fast16_t pwm = spindle->context.pwm->period + 20000;
        TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm >> 16);
        TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm & 0xFFFF);
        if(!spindle->context.pwm->flags.enable_out)
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B);                                   // Ensure PWM output is enabled to
        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !spindle->context.pwm->settings->invert.pwm);   // ensure correct output level.
        TimerDisable(SPINDLE_PWM_TIMER_BASE, TIMER_B);                                      // Disable PWM.
    }
}

static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive) {
                TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value >> 16);
                TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value & 0xFFFF);
            }
        } else
            pwm_off(spindle);

    } else {

         TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value >> 16);
         TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value & 0xFFFF);

         if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

         TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle->context.pwm->period >> 16);
         TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle->context.pwm->period & 0xFFFF);
         TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !spindle->context.pwm->settings->invert.pwm);
         TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
    }
}

#endif

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    spindle_pwm.offset = -1;

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, configCPU_CLOCK_HZ)) {
        TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period >> 16);
        TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period & 0xFFFF);
        spindle->set_state = spindleSetStateVariable;
    } else {
        if(spindle->context.pwm->flags.enable_out)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#endif // DRIVER_SPINDLE_PWM_ENABLE

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { settings.pwm_spindle.invert.mask };

    UNUSED(spindle);

    state.on = DIGITAL_IN(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);

    state.mask ^= settings.pwm_spindle.invert.mask;

#ifdef SPINDLE_PWM_PIN
    state.on |= spindle->param->state.on;
#endif
#if PWM_RAMPED
    state.at_speed = pwm_ramp.pwm_current == pwm_ramp.pwm_target;
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.mask ^= settings.coolant.invert.mask;
    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, mode.flood);
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_PIN, mode.mist);
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = DIGITAL_IN(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
    state.mist  = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
    state.mask ^= settings.coolant.invert.mask;

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

#if  MPG_ENABLE == 1

static void mpg_select (void *data)
{
    stream_mpg_enable(GPIOPinRead(MPG_MODE_PORT, MPG_MODE_BIT) == 0);

    GPIOIntEnable(MPG_MODE_PORT, MPG_MODE_BIT);
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode == (GPIOPinRead(MPG_MODE_PORT, MPG_MODE_BIT) == 0))
        mpg_select(data);

#if I2C_STROBE_ENABLE
//    BITBAND_PERI(I2C_STROBE_PORT->IE, I2C_STROBE_PIN) = 1;
#endif
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
void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
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

#if DRIVER_SPINDLE_PWM_ENABLE
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif

        hal.stepper.go_idle(true);

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
            if(input->group != PinGroup_AuxInput)
                input->mode.irq_mode = IRQ_Mode_None;
            pullup = input->group == PinGroup_AuxInput;

            switch(input->id) {

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    input->mode.irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    input->mode.irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    input->mode.irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    input->mode.irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    break;

                case Input_LimitX:
                    pullup = !settings->limits.disable_pullup.x;
                    input->mode.irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                    pullup = !settings->limits.disable_pullup.y;
                    input->mode.irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                    pullup = !settings->limits.disable_pullup.z;
                    input->mode.irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitA:
                    pullup = !settings->limits.disable_pullup.a;
                    input->mode.irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitB:
                    pullup = !settings->limits.disable_pullup.b;
                    input->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitC:
                    pullup = !settings->limits.disable_pullup.c;
                    input->mode.irq_mode = limit_fei.c ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_ModeSelect:
                    pullup = true;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;

                case Input_KeypadStrobe:
                    pullup = true;
                    input->mode.irq_mode = IRQ_Mode_Change;
                    break;
            }

            if(input->group == PinGroup_AuxInput)
                pullup = true;

            input->debounce = hal.driver_cap.software_debounce && (input->group == PinGroup_Limit || input->group == PinGroup_Control);

            GPIOIntDisable(input->port, input->bit);    // Disable pin change interrupt
            GPIOPinTypeGPIOInput(input->port, input->bit);
            GPIOPadConfigSet(input->port, input->bit, GPIO_STRENGTH_2MA, pullup ? GPIO_PIN_TYPE_STD_WPU : GPIO_PIN_TYPE_STD_WPD);
            GPIOIntClear(input->port, input->bit);     // Clear any pending interrupt

            if(input->mode.irq_mode != IRQ_Mode_None || input->group == PinGroup_AuxInput) {

                if((handler = get_handler(input->port)))
                    memcpy(&handler->pins[handler->count++], input, sizeof(input_signal_t));

                if(input->group != PinGroup_AuxInput) {
                    GPIOIntTypeSet(input->port, input->bit, input->mode.irq_mode == IRQ_Mode_Falling ? GPIO_FALLING_EDGE : (input->mode.irq_mode == IRQ_Mode_Change ? GPIO_BOTH_EDGES : GPIO_RISING_EDGE));
                    GPIOIntEnable(input->port, input->bit);    // Enable pin change interrupt for control pins
                }
                if(input->mode.irq_mode != IRQ_Mode_Change)
                    input->active = input->active ^ (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1);
            }

        } while(i);

#if AUX_CONTROLS_ENABLED
        aux_ctrl_irq_enable(settings, aux_irq_handler);
#endif

        i = sizeof(irq_handler) / sizeof(irq_handler_t);
        do {
            if(irq_handler[--i].count)
                GPIOIntRegister(irq_handler[i].port, irq_handler[i].handler);
        } while(i);
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

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.port = low_level ? (void *)inputpin[i].port : (void *)port2char(inputpin[i].port);
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.port = low_level ? (void *)outputpin[i].port : (void *)port2char(outputpin[i].port);
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.port = low_level ? ppin->pin.port : (void *)port2char((uint32_t)ppin->pin.port);
        pin.mode = ppin->pin.mode;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);
    } while(ppin = ppin->next);
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
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

#if DRIVER_SPINDLE_PWM_ENABLE

   /******************
    *  Spindle init  *
    ******************/

    SysCtlPeripheralEnable(SPINDLE_PWM_TIMER_PERIPH);
    SysCtlDelay(26); // wait a bit for peripherals to wake up
    TimerClockSourceSet(SPINDLE_PWM_TIMER_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(SPINDLE_PWM_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
    TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
    GPIOPinConfigure(SPINDLE_PWM_MAP);
    GPIOPinTypeTimer(SPINDLE_PWM_PORT, 1<<SPINDLE_PWM_PIN);
    GPIOPadConfigSet(SPINDLE_PWM_PORT, 1<<SPINDLE_PWM_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  #if PWM_RAMPED
    pwm_ramp.ms_cfg = pwm_ramp.pwm_current = pwm_ramp.pwm_target = 0;
  #endif

  #if !(AUX_CONTROLS & AUX_CONTROL_SPINDLE)

    static const periph_pin_t pwm = {
        .function = Output_SpindlePWM,
        .group = PinGroup_SpindlePWM,
        .port = (void *)SPINDLE_PWM_PORT,
        .pin = SPINDLE_PWM_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    hal.periph_port.register_pin(&pwm);

  #endif

#endif // DRIVER_SPINDLE_ENABLE

#if SDCARD_ENABLE
  #ifdef __MSP432E401Y__
    sdcard_events_t *card = sdcard_init();
    card->on_mount = sdcard_mount;
    card->on_unmount = sdcard_unmount;
  #else
    sdcard_init();
  #endif
  #if defined(FreeRTOS) && !defined(__MSP432E401Y__)
    TimerHandle_t xFatFsTimer = xTimerCreate("fatfsTimer", pdMS_TO_TICKS(10), pdTRUE, NULL, fatfsTimerCallback);
    xTimerStart(xFatFsTimer, 10);
  #endif
#endif

#if LITTLEFS_ENABLE
    fs_littlefs_mount("/littlefs", ti_littlefs_hal());
#endif

#if I2C_STROBE_ENABLE

   /*********************
    *  I2C KeyPad init  *
    *********************/

    GPIOPinTypeGPIOInput(I2C_STROBE_PORT, I2C_STROBE_PIN);
    GPIOPadConfigSet(I2C_STROBE_PORT, I2C_STROBE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // -> WPU

    GPIOIntTypeSet(I2C_STROBE_PORT, I2C_STROBE_PIN, GPIO_BOTH_EDGES);
    GPIOIntEnable(I2C_STROBE_PORT, I2C_STROBE_PIN);

#endif

  // Set defaults

    IOInitDone = settings->version.id == 23;

    hal.settings_changed(settings, (settings_changed_flags_t){0});

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

#ifdef __MSP432E401Y__
  #ifdef FreeRTOS
    hal.info = "MSP432E401Y FreeRTOS";
    hal.driver_url = "https://github.com/grblHAL/MSP432E401Y";
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
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif
    hal.driver_version = "250404";
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

#if PROBE_ENABLE && !defined(AUX_DEVICES)
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
    hal.driver_cap.probe_pull_up = On;
#endif

    hal.control.get_state = systemGetState;

    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;
#ifdef FreeRTOS
    hal.get_elapsed_ticks = xTaskGetTickCountFromISR;
#else
    hal.get_elapsed_ticks = getElapsedTicks;
#endif

    stream_connect(serialInit(115200));

    eeprom_init();

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_PWM_ENABLE

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
#if DRIVER_SPINDLE_DIR_ENABLE
        .ref_id = SPINDLE_PWM0,
#else
        .ref_id = SPINDLE_PWM0_NODIR,
#endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On,
  #endif
  #if PWM_RAMPED
            .at_speed = On
  #endif
        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
#if DRIVER_SPINDLE_DIR_ENABLE
        .ref_id = SPINDLE_ONOFF0_DIR,
#else
        .ref_id = SPINDLE_ONOFF0,
#endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
    hal.coolant_cap.bits = COOLANT_ENABLE;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
#if LASER_PPI
    hal.driver_cap.laser_ppi_mode = On;
#endif

    uint32_t i;
    input_signal_t *input;
    static pin_group_pins_t aux_inputs = {0}, aux_outputs = {0};

    for(i = 0 ; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        if(input->group == PinGroup_AuxInput) {
            if(aux_inputs.pins.inputs == NULL)
                aux_inputs.pins.inputs = input;
            input->id = (pin_function_t)(Input_Aux0 + aux_inputs.n_pins);
            input->mode.pull_mode = input->cap.pull_mode = PullMode_Up;
            input->cap.irq_mode = IRQ_Mode_Rising|IRQ_Mode_Falling;
#if AUX_CONTROLS_ENABLED
            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit((void *)input->port, input->pin, aux_inputs.n_pins, input))) {
                if(aux_remap->function == Input_Probe && input->cap.irq_mode == IRQ_Mode_Edges)
                    aux_remap->irq_mode = IRQ_Mode_Change;
            }
#endif
            aux_inputs.n_pins++;
        } else if(input->group == PinGroup_Limit) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        }
    }

    output_signal_t *output;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_outputs.pins.outputs == NULL)
                aux_outputs.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_outputs.n_pins);
#if AUX_CONTROLS
            aux_out_remap_explicit((void *)output->port, output->pin, aux_outputs.n_pins, output);
#endif
            aux_outputs.n_pins++;
        }
    }

    ioports_init(&aux_inputs, &aux_outputs);

#if AUX_CONTROLS_ENABLED
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
#elif defined(SAFETY_DOOR_PIN)
    hal.signals_cap.safety_door = On;
#endif

#if AUX_CONTROLS
    aux_ctrl_claim_out_ports(aux_out_claim_explicit, NULL);
#endif

    serialRegisterStreams();

#if ETHERNET_ENABLE
    enet_init();
#endif

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        protocol_enqueue_foreground_task(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 10;
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

    set_step_outputs(next_step_out);

    TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_length);
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}

#if LASER_PPI

void laser_ppi_mode (bool on)
{
    if(on)
        hal.stepper.pulse_start = stepperPulseStartPPI;
    else
        hal.stepper.pulse_start = settings.steppers.pulse_delay_microseconds > 0.0f ? stepperPulseStartDelayed : stepperPulseStart;
    gc_set_laser_ppimode(on);
}

// Switches off the spindle (laser) after laser.pulse_length time has elapsed
static void ppi_timeout_isr (void)
{
    TimerIntClear(LASER_PPI_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    spindle_off(ppi_spindle);
}
#endif

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

#if AUX_CONTROLS_ENABLED
        if(signal->id & Input_SafetyDoor)
            debounce.safety_door = false;
#endif

        if(!!GPIOPinRead(signal->port, signal->bit) == (signal->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1))
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

#if  MPG_ENABLE == 1
                case PinGroup_MPG:
                    GPIOIntDisable(MPG_MODE_PORT, MPG_MODE_BIT);
                    protocol_enqueue_foreground_task(mpg_select, NULL);
                    break;
#endif

#if I2C_STROBE_ENABLE
                case PinGroup_Keypad:
                    if(i2c_strobe.callback)
                        i2c_strobe.callback(0, !GPIOPinRead(input->port, input->bit));
                    break;
#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C
                case PinGroup_Motor_Warning:
                    trinamic_warn_handler();
                    break;

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
                    if(spindle->context.pwm->flags.enable_out)
                        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, true);
                    spindle->context.pwm->flags.enable_out = Off;
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
