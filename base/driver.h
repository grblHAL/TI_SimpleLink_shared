/*
  driver.h - main driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2018-2023 Terje Io
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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#define PREF(x) x   //Use for debugging purposes to trace problems in driver.lib
//#define   PREF(x) MAP_ ## x   //Use to reduce code size

#ifdef __MSP432E401Y__
#include "msp.h"
#include <ti/devices/msp432e4/driverlib/driverlib.h>
#include <ti/devices/msp432e4/driverlib/inc/hw_gpio.h>
#else
#include "tiva.h"
#endif

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

#define FreeRTOS

#ifndef CNC_BOOSTERPACK_SHORTS
#define CNC_BOOSTERPACK_SHORTS  0
#endif
#ifndef CNC_BOOSTERPACK_A4998
#define CNC_BOOSTERPACK_A4998   0
#endif

#define CNC_BOOSTERPACK  0
#define CNC_BOOSTERPACK2 0

#if (TELNET_ENABLE || WEBSOCKET_ENABLE || FTP_ENABLE) && !ETHERNET_ENABLE
#error "Networking protocols requires ethernet enabled!"
#endif

#if WEBUI_ENABLE
  #ifdef WEBUI_INFLASH
  #undef WEBUI_INFLASH
  #endif
  #ifdef LITTLEFS_ENABLE
  #undef LITTLEFS_ENABLE
  #endif
#define WEBUI_INFLASH   1
#define LITTLEFS_ENABLE 1
#endif

#ifdef BOARD_CNC_BOOSTERPACK
#include "cnc_boosterpack_map.h"
#elif defined(BOARD_MY_MACHINE)
#include "my_machine_map.h.h"
#else
#error "No board!"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.2f // microseconds
#endif

// End configuration

#if MPG_ENABLE || KEYPAD_ENABLE == 2
#define SERIAL2_MOD
#endif

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#define GPIOBase(t) gpioB(t)
#define gpioB(t) GPIO_PORT ## t ## _BASE
#define GPIOPort(t) gpioP(t)
#define gpioP(t) GPIO ## t
#define GPIOOut(t) gpioO(t)
#define gpioO(t) GPIO ## t ## )->DATA

#define timerBase(t) timerB(t)
#define timerB(t) TIMER ## t ## _BASE
#define timerPeriph(t) timerP(t)
#define timerP(t) SYSCTL_PERIPH_TIMER ## t
#define timerINT(t, i) timerI(t, i)
#ifdef EK_TM4C129_BP2
#define timerI(t, i) INT_TIMER ## t ## i ## _TM4C129
#else
#define timerI(t, i) INT_TIMER ## t ## i
#endif

#ifdef __MSP432E401Y__
#define DIGITAL_IN(port, pin) HWREGBITW(&((GPIO_Type *)port)->DATA, pin)
#define DIGITAL_OUT(port, pin, on) { HWREGBITW(&((GPIO_Type *)port)->DATA, pin) = on; }
#else
#define DIGITAL_IN(port, pin) !!GPIOPinRead(port, 1<<pin)
#define DIGITAL_OUT(port, pin, on) GPIOPinWrite(port, 1<<pin, (on) ? 1<<pin : 0);
#endif

// Define GPIO output mode options
// Use GPIO_SHIFTx when output bits are consecutive and in the same port
// Use GPIO_MAP when output bits are not consecutive but in the same port
// Use GPIO_BITBAND when output bits are in different ports

#define GPIO_SHIFT0  0
#define GPIO_SHIFT1  1
#define GPIO_SHIFT2  2
#define GPIO_SHIFT3  3
#define GPIO_SHIFT4  4
#define GPIO_SHIFT5  5
#define GPIO_MAP     8
#define GPIO_BITBAND 9

// timer definitions

#define STEPPER_TIM 1
#define STEPPER_TIMER_PERIPH    timerPeriph(STEPPER_TIM)
#define STEPPER_TIMER_BASE      timerBase(STEPPER_TIM)
#define STEPPER_TIMER_INT       timerINT(STEPPER_TIM, A)

#define PULSE_TIM 0
#define PULSE_TIMER_PERIPH      timerPeriph(PULSE_TIM)
#define PULSE_TIMER_BASE        timerBase(PULSE_TIM)
#define PULSE_TIMER_INT         timerINT(PULSE_TIM, A)

#define DEBOUNCE_TIM 4
#define DEBOUNCE_TIMER_PERIPH   timerPeriph(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_BASE     timerBase(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT      timerINT(DEBOUNCE_TIM, A)

#define SPINDLE_PWM_TIM 3
#define SPINDLE_PWM_TIMER_PERIPH    timerPeriph(SPINDLE_PWM_TIM)
#define SPINDLE_PWM_TIMER_BASE      timerBase(SPINDLE_PWM_TIM)
//#define SPINDLE_PWM_TIMER_INT   timerINT(SPINDLE_PWM_TIM, A)

#if LASER_PPI

#define LASER_PPI_TIM 2
#define LASER_PPI_TIMER_PERIPH  timerPeriph(LASER_PPI_TIM)
#define LASER_PPI_TIMER_BASE    timerBase(LASER_PPI_TIM)
#define LASER_PPI_TIMER_INT     timerINT(LASER_PPI_TIM, A)

typedef struct {
    float ppi;
    uint_fast16_t steps_per_pulse;
    uint_fast16_t pulse_length; // uS
    uint32_t next_pulse;
} laser_ppi_t;

extern laser_ppi_t laser;

void laser_ppi_mode (bool on);

#endif

#ifndef X_STEP_PORT
#define X_STEP_PORT STEP_PORT
#endif
#ifndef Y_STEP_PORT
#define Y_STEP_PORT STEP_PORT
#endif
#ifndef Z_STEP_PORT
#define Z_STEP_PORT STEP_PORT
#endif
#ifndef X_DIRECTION_PORT
#define X_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Y_DIRECTION_PORT
#define Y_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Z_DIRECTION_PORT
#define Z_DIRECTION_PORT DIRECTION_PORT
#endif

typedef struct {
    pin_function_t id;
    uint32_t port;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    uint32_t port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

#ifdef HAS_BOARD_INIT
void board_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
#endif

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif
