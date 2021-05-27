/*
  cnc_boosterpack_map.h - pin mapping configuration file for CNC BoosterPack

  - on Texas Instruments MSP432P401R LaunchPad

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

#if CNC_BOOSTERPACK2
#define BOARD_NAME "CNC BoosterPack x2";
#else
#define BOARD_NAME "CNC BoosterPack";
#endif

#define HAS_BOARD_INIT

#if TRINAMIC_ENABLE
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_MIXED_DRIVERS 0
#ifdef TRINAMIC_I2C
#undef TRINAMIC_I2C
#endif
#define TRINAMIC_I2C 1
#endif

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif

#ifdef CNC_BOOSTERPACK2
#undef CNC_BOOSTERPACK2
#endif

#define CNC_BOOSTERPACK         1
#define CNC_BOOSTERPACK_SHORTS  0 // do not change!
#define CNC_BOOSTERPACK_A4998   1 // Using Polulu A4998 drivers - for suppying VDD via GPIO (PE5)

#if N_AXIS > 3
  #define CNC_BOOSTERPACK2 1 // do not change!
#else
  #define CNC_BOOSTERPACK2 0 // do not change!
#endif

// Define step pulse output pins.
#ifdef __MSP432E401Y__
#define STEP_OUTMODE    GPIO_BITBAND
#else
#define STEP_OUTMODE    GPIO_SHIFT1
#endif
#define STEP_PORT_ID    E
#define STEP_PORT       GPIOBase(STEP_PORT_ID)
#define X_STEP_PIN      1
#define Y_STEP_PIN      2
#define Z_STEP_PIN      3
#define X_STEP_BIT      (1 << X_STEP_PIN)
#define Y_STEP_BIT      (1 << Y_STEP_PIN)
#define Z_STEP_BIT      (1 << Z_STEP_PIN)
#define HWSTEP_MASK     (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
#if STEP_OUTMODE == GPIO_BITBAND
#define STEP_OUT_X      GPIOPort(STEP_PORT_ID)
#define STEP_OUT_Y      GPIOPort(STEP_PORT_ID)
#define STEP_OUT_Z      GPIOPort(STEP_PORT_ID)
#endif

#if CNC_BOOSTERPACK2
#define STEP_PORT_ID_AB K
#define STEP_PORT_ID_C  B
#define STEP_PORT_AB    GPIOBase(STEP_PORT_ID_AB)
#define STEP_PORT_A     GPIOBase(STEP_PORT_ID_C)
#define STEP_PORT_B     GPIOBase(STEP_PORT_ID_C)
#define STEP_PORT_C     GPIOBase(STEP_PORT_ID_C)
#if STEP_OUTMODE == GPIO_BITBAND
#define STEP_OUT_A      GPIOPort(STEP_PORT_ID_AB)
#define STEP_OUT_B      GPIOPort(STEP_PORT_ID_AB)
#define STEP_OUT_C      GPIOPort(STEP_PORT_ID_C)
#else
#define STEP_OUTMODE_2  GPIO_SHIFT0
#endif
#define A_STEP_PIN      0
#define B_STEP_PIN      1
#define C_STEP_PIN      5
#define A_STEP_BIT      (1 << A_STEP_PIN)
#define B_STEP_BIT      (1 << B_STEP_PIN)
#define C_STEP_BIT      (1 << C_STEP_PIN)
#define HWSTEP_MASK_AB  (A_STEP_BIT|B_STEP_BIT)
#endif

// Define step direction output pins.
#ifdef __MSP432E401Y__
#define DIRECTION_OUTMODE   GPIO_BITBAND
#else
#define DIRECTION_OUTMODE   GPIO_MAP
#endif
#define DIRECTION_PORT_ID   D
#define DIRECTION_PORT      GPIOBase(DIRECTION_PORT_ID)
#define DIRECTION_OUT_X     GPIOPort(DIRECTION_PORT_ID)
#define DIRECTION_OUT_Y     GPIOPort(DIRECTION_PORT_ID)
#define DIRECTION_OUT_Z     GPIOPort(DIRECTION_PORT_ID)
#define X_DIRECTION_PIN     1
#define Y_DIRECTION_PIN     0
#define Z_DIRECTION_PIN     3
#define X_DIRECTION_BIT     (1 << X_DIRECTION_PIN)
#define Y_DIRECTION_BIT     (1 << Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT     (1 << Z_DIRECTION_PIN)
#define HWDIRECTION_MASK    (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits

#if CNC_BOOSTERPACK2
#define DIRECTION_PORT2_ID  Q
#define DIRECTION_PORT2     GPIOBase(DIRECTION_PORT2_ID)
#define DIRECTION_PORT_A    GPIOBase(DIRECTION_PORT2_ID)
#define DIRECTION_PORT_B    GPIOBase(DIRECTION_PORT2_ID)
#define DIRECTION_PORT_C    GPIOBase(DIRECTION_PORT2_ID)
#if STEP_OUTMODE == GPIO_BITBAND
#define DIRECTION_OUT_A     GPIOPort(DIRECTION_PORT2_ID)
#define DIRECTION_OUT_B     GPIOPort(DIRECTION_PORT2_ID)
#define DIRECTION_OUT_C     GPIOPort(DIRECTION_PORT2_ID)
#else
#define DIRECTION_OUTMODE2  GPIO_MAP
#endif
#define A_DIRECTION_PIN     1
#define B_DIRECTION_PIN     0
#define C_DIRECTION_PIN     3
#define A_DIRECTION_BIT     (1 << A_DIRECTION_PIN)
#define B_DIRECTION_BIT     (1 << B_DIRECTION_PIN)
#define C_DIRECTION_BIT     (1 << C_DIRECTION_PIN)
#define HWDIRECTION_MASK2   (A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT) // All direction bits
#endif

// Define stepper driver enable/disable output pin(s).
#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C
#define TRINAMIC_DIAG_IRQ_PORT      GPIO_PORTD_BASE
#define TRINAMIC_DIAG_IRQ_PIN       GPIO_PIN_7
#define TRINAMIC_WARN_IRQ_PORT      GPIO_PORTH_BASE
#define TRINAMIC_WARN_IRQ_PIN       GPIO_PIN_3
// Define stepper driver enable/disable output pin(s).
#elif CNC_BOOSTERPACK
#define STEPPERS_DISABLE_XY_PORT    GPIO_PORTD_BASE
#define STEPPERS_DISABLE_XY_PIN     GPIO_PIN_7
#define STEPPERS_DISABLE_Z_PORT     GPIO_PORTH_BASE
#define STEPPERS_DISABLE_Z_PIN      GPIO_PIN_3
 #if CNC_BOOSTERPACK2
 #define STEPPERS_DISABLE_AC_PORT   GPIO_PORTK_BASE
  #define STEPPERS_DISABLE_AC_PIN    GPIO_PIN_2
 #define STEPPERS_DISABLE_B_PORT    GPIO_PORTA_BASE
 #define STEPPERS_DISABLE_B_PIN     GPIO_PIN_7
 #endif
#else
#define STEPPERS_DISABLE_PORT       GPIO_PORTE_BASE
#define STEPPERS_DISABLE_PIN        GPIO_PIN_1
#endif

#if CNC_BOOSTERPACK_A4998
// Stepper driver VDD supply
#define STEPPERS_VDD_PORT GPIO_PORTE_BASE
#define STEPPERS_VDD_PIN  GPIO_PIN_5
 #if CNC_BOOSTERPACK2
  #define STEPPERS_VDD_PORT2 GPIO_PORTD_BASE
  #define STEPPERS_VDD_PIN2  GPIO_PIN_5
 #endif
#endif

// Define homing/hard limit switch input pins and limit interrupt vectors.
#if CNC_BOOSTERPACK
    #define LIMIT_PORT_X    GPIO_PORTH_BASE
    #define LIMIT_PORT_YZ   GPIO_PORTF_BASE
    #define LIMIT_PORT_Y    LIMIT_PORT_YZ
    #define LIMIT_PORT_Z    LIMIT_PORT_YZ
    #define X_LIMIT_BIT     GPIO_PIN_2
    #define Y_LIMIT_BIT     GPIO_PIN_2
    #define Z_LIMIT_BIT     GPIO_PIN_1
    #define HWLIMIT_MASK_YZ (Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
 #if CNC_BOOSTERPACK2
  #ifdef A_AXIS
    #define LIMIT_PORT_A    GPIO_PORTK_BASE
    #define A_LIMIT_BIT     GPIO_PIN_4
  #endif
  #ifdef B_AXIS
    #define LIMIT_PORT_B    GPIO_PORTG_BASE
    #define B_LIMIT_BIT     GPIO_PIN_1
  #endif
  #ifdef C_AXIS
    #define LIMIT_PORT_C    GPIO_PORTP_BASE
    #define C_LIMIT_BIT     GPIO_PIN_5
  #endif
#endif
#else
    #define LIMIT_PORT      GPIO_PORTA_BASE
    #define X_LIMIT_BIT     GPIO_PIN_1
    #define Y_LIMIT_BIT     GPIO_PIN_3
    #define Z_LIMIT_BIT     GPIO_PIN_2
    #define HWLIMIT_MASK    (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIO_PORTD_BASE
#define SPINDLE_ENABLE_PIN      GPIO_PIN_6

#define SPINDLE_DIRECTION_PORT  GPIO_PORTM_BASE
#define SPINDLE_DIRECTION_BIT   GPIO_PIN_4

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  GPIO_PORTL_BASE
#define COOLANT_FLOOD_PIN   GPIO_PIN_1

#define COOLANT_MIST_PORT   GPIO_PORTL_BASE
#define COOLANT_MIST_PIN    GPIO_PIN_2

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#if CNC_BOOSTERPACK
  #if CNC_BOOSTERPACK_SHORTS
    #define CONTROL_PORT    GPIO_PORTL_BASE
    #define CONTROL_PORT_SD GPIO_PORTG_BASE
    #define RESET_BIT       GPIO_PIN_0
    #define FEED_HOLD_BIT   GPIO_PIN_4
    #define CYCLE_START_BIT GPIO_PIN_5
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    #define SAFETY_DOOR_BIT GPIO_PIN_0
#endif
  #else
    #define CONTROL_PORT_FH_CS GPIO_PORTL_BASE
    #define CONTROL_PORT_RST   GPIO_PORTF_BASE
    #define CONTROL_PORT_SD    GPIO_PORTG_BASE
    #define RESET_BIT          GPIO_PIN_3
    #define FEED_HOLD_BIT      GPIO_PIN_4
    #define CYCLE_START_BIT    GPIO_PIN_5
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    #define SAFETY_DOOR_BIT    GPIO_PIN_0
#endif
  #endif
#else
#define CONTROL_PORT        GPIO_PORTC_BASE
#define RESET_BIT           GPIO_PIN_7
#define FEED_HOLD_BIT       GPIO_PIN_6
#define CYCLE_START_BIT     GPIO_PIN_5
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_BIT     GPIO_PIN_4
#endif
#endif
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define HWCONTROL_MASK      (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
#else
#define HWCONTROL_MASK      (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT)
#endif

// Define probe switch input pin.
#define PROBE_PORT          GPIO_PORTC_BASE
#define PROBE_BIT           GPIO_PIN_7

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_PORT    GPIO_PORTM_BASE
#define SPINDLE_PWM_BIT     GPIO_PIN_3
#define SPINDLE_PWM_MAP     GPIO_PM3_T3CCP1

/*
 * CNC Boosterpack GPIO assignments
 */

#define AUXIO0_PORT     GPIO_PORTL_BASE
#define AUXIO0_BIT      GPIO_PIN_3
#define AUXIO1_PORT     GPIO_PORTN_BASE
#define AUXIO1_BIT      GPIO_PIN_2
#define AUXIO2_PORT     GPIO_PORTN_BASE
#define AUXIO2_BIT      GPIO_PIN_3
#define AUXIO3_PORT     GPIO_PORTP_BASE
#define AUXIO3_BIT      GPIO_PIN_2
#define AUXIO4_PORT     GPIO_PORTC_BASE
#define AUXIO4_BIT      GPIO_PIN_4
#define AUXIO5_PORT     GPIO_PORTC_BASE
#define AUXIO5_BIT      GPIO_PIN_5
#define AUXIO6_PORT     GPIO_PORTC_BASE
#define AUXIO6_BIT      GPIO_PIN_6

// Output definitions (comment out the port definition if used as input)

#define AUXOUTPUT0_PORT AUXIO0_PORT
#define AUXOUTPUT0_PIN  AUXIO0_BIT

#define AUXOUTPUT1_PORT AUXIO1_PORT
#define AUXOUTPUT1_PIN  AUXIO1_BIT

#define AUXOUTPUT2_PORT AUXIO3_PORT
#define AUXOUTPUT2_PIN  AUXIO3_BIT

// Input definitions

#ifndef AUXOUTPUT0_PORT
#define AUXINPUT0_PORT  AUXIO0_PORT
#define AUXINPUT0_BIT   AUXIO0_BIT
#endif

#ifndef AUXOUTPUT1_PORT
#define AUXINPUT1_PORT  AUXIO1_PORT
#define AUXINPUT1_BIT   AUXIO1_BIT
#endif

#if MPG_MODE_ENABLE
#define MODE_PORT       AUXIO2_PORT
#define MODE_GPIO       GPIO2_GPIO
#define MODE_INT        GPIO2_INT
#define MODE_SWITCH_BIT AUXIO2_BIT
#else
#define AUXINPUT2_PORT  AUXIO2_PORT
#define AUXINPUT2_BIT   AUXIO2_BIT
#endif

#ifndef AUXOUTPUT2_PORT
#define AUXINPUT3_PORT  AUXIO3_PORT
#define AUXINPUT3_BIT   AUXIO3_BIT
#endif

#ifndef SERIAL2_MOD
#define AUXINPUT4_PORT  AUXIO4_PORT
#define AUXINPUT4_BIT   AUXIO4_BIT
#define AUXINPUT5_PORT  AUXIO5_PORT
#define AUXINPUT5_BIT   AUXIO5_BIT
#endif

#if KEYPAD_ENABLE
#define KEYINTR_PORT    AUXIO6_PORT
#define KEYINTR_BIT     AUXIO6_BIT
#else
#define AUXINPUT6_PORT  AUXIO6_PORT
#define AUXINPUT6_BIT   AUXIO6_BIT
#endif

/**/

