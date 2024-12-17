/*
  cnc_boosterpack_map.h - pin mapping configuration file for CNC BoosterPack

  - on Texas Instruments MSP432P401R LaunchPad

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

#if CNC_BOOSTERPACK2
#define BOARD_NAME "CNC BoosterPack x2";
#else
#define BOARD_NAME "CNC BoosterPack";
#endif
#define BOARD_URL "https://github.com/terjeio/CNC_Boosterpack"

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

#if N_ABC_MOTORS
  #define CNC_BOOSTERPACK2 1 // do not change!
#else
  #define CNC_BOOSTERPACK2 0 // do not change!
#endif

#if CNC_BOOSTERPACK_A4998
// Stepper driver VDD supply
#define STEPPERS_VDD_PORT       GPIO_PORTE_BASE
#define STEPPERS_VDD_PIN        5
 #if CNC_BOOSTERPACK2
  #define STEPPERS_VDD_PORT2    GPIO_PORTD_BASE
  #define STEPPERS_VDD_PIN2     5
 #endif
#endif

// Define step pulse output pins.
#define X_STEP_PORT             GPIO_PORTE_BASE
#define X_STEP_PIN              1
#define Y_STEP_PORT             GPIO_PORTE_BASE
#define Y_STEP_PIN              2
#define Z_STEP_PORT             GPIO_PORTE_BASE
#define Z_STEP_PIN              3
#ifdef __MSP432E401Y__
#define STEP_OUTMODE            GPIO_BITBAND
#else
#define STEP_PORT               X_STEP_PORT
#define STEP_OUTMODE            GPIO_SHIFT1
#define HWSTEP_MASK             ((1<<X_STEP_PIN)|(1<<Y_STEP_PIN)|(1<<Z_STEP_PIN))
#endif


// Define step direction output pins.
#define X_DIRECTION_PORT        GPIO_PORTD_BASE
#define X_DIRECTION_PIN         1
#define Y_DIRECTION_PORT        GPIO_PORTD_BASE
#define Y_DIRECTION_PIN         0
#define Z_DIRECTION_PORT        GPIO_PORTD_BASE
#define Z_DIRECTION_PIN         3
#ifdef __MSP432E401Y__
#define DIRECTION_OUTMODE       GPIO_BITBAND
#else
#define DIRECTION_PORT          X_DIRECTION_PORT
#define DIRECTION_OUTMODE       GPIO_MAP
#define X_DIRECTION_BIT         (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT         (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT         (1<<Z_DIRECTION_PIN)
#define HWDIRECTION_MASK        (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT)
#endif

// Define stepper driver enable/disable output pin(s).
#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C
#define TRINAMIC_DIAG_IRQ_PORT  GPIO_PORTD_BASE
#define TRINAMIC_DIAG_IRQ_PIN   7
#define TRINAMIC_WARN_IRQ_PORT  GPIO_PORTH_BASE
#define TRINAMIC_WARN_IRQ_PIN   3
// Define stepper driver enable/disable output pin(s).
#else
#define XY_ENABLE_PORT          GPIO_PORTD_BASE
#define XY_ENABLE_PIN           7
#define Z_ENABLE_PORT           GPIO_PORTH_BASE
#define Z_ENABLE_PIN            3
#endif

// Define homing/hard limit switch input pins and limit interrupt vectors.

#define X_LIMIT_PORT            GPIO_PORTH_BASE
#define X_LIMIT_PIN             2
#define Y_LIMIT_PORT            GPIO_PORTF_BASE
#define Y_LIMIT_PIN             2
#define Z_LIMIT_PORT            GPIO_PORTF_BASE
#define Z_LIMIT_PIN             1

#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIO_PORTK_BASE
#define M3_STEP_PIN             0
#define M3_DIRECTION_PORT       GPIO_PORTQ_BASE
#define M3_DIRECTION_PIN        3
#define M3_LIMIT_PORT           GPIO_PORTK_BASE
#define M3_LIMIT_PIN            4
#ifndef STEPPERS_ENABLE_PORT
#define M3_ENABLE_PORT          GPIO_PORTK_BASE
#define M3_ENABLE_PIN           2
#endif
#endif

#if N_ABC_MOTORS > 1
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIO_PORTK_BASE
#define M4_STEP_PIN             1
#define M4_DIRECTION_PORT       GPIO_PORTQ_BASE
#define M4_DIRECTION_PIN        0
#define M4_LIMIT_PORT           GPIO_PORTG_BASE
#define M4_LIMIT_PIN            1
#ifndef STEPPERS_ENABLE_PORT
#define M4_ENABLE_PORT          GPIO_PORTA_BASE
#define M4_ENABLE_PIN           7
#endif
#endif

#if N_ABC_MOTORS == 3
#define M5_AVAILABLE
#define M5_STEP_PORT            GPIO_PORTB_BASE
#define M5_STEP_PIN             5
#define M5_DIRECTION_PORT       GPIO_PORTQ_BASE
#define M5_DIRECTION_PIN        2
#define M5_LIMIT_PORT           GPIO_PORTP_BASE
#define M5_LIMIT_PIN            5
#endif

#if N_ABC_MOTORS && !defined(__MSP432E401Y__)
#define STEP_OUTMODE_2          GPIO_SHIFT0
#define HWSTEP_MASK_AB          (A_STEP_PIN|B_STEP_PIN)
#define DIRECTION_PORT2         M3_DIRECTION_PORT
#define HWDIRECTION_MASK2       (A_DIRECTION_PIN|B_DIRECTION_PIN|C_DIRECTION_PIN)
#endif

/*
 * CNC Boosterpack GPIO assignments
 */

#define AUXIO0_PORT             GPIO_PORTL_BASE
#define AUXIO0_PIN              3
#define AUXIO1_PORT             GPIO_PORTN_BASE
#define AUXIO1_PIN              2
#define AUXIO2_PORT             GPIO_PORTN_BASE
#define AUXIO2_PIN              3
#define AUXIO3_PORT             GPIO_PORTP_BASE
#define AUXIO3_PIN              2
#define AUXIO4_PORT             GPIO_PORTC_BASE
#define AUXIO4_PIN              4
#define AUXIO5_PORT             GPIO_PORTC_BASE
#define AUXIO5_PIN              5
#define AUXIO6_PORT             GPIO_PORTC_BASE
#define AUXIO6_PIN              6

// Output definitions (comment out the port definition if used as input)

#define AUXOUTPUT0_PORT         AUXIO0_PORT
#define AUXOUTPUT0_PIN          AUXIO0_PIN
#define AUXOUTPUT1_PORT         AUXIO1_PORT
#define AUXOUTPUT1_PIN          AUXIO1_PIN
#define AUXOUTPUT2_PORT         AUXIO3_PORT
#define AUXOUTPUT2_PIN          AUXIO3_PIN
#define AUXOUTPUT3_PORT         GPIO_PORTM_BASE // Spindle PWM
#define AUXOUTPUT3_PIN          3
#define AUXOUTPUT4_PORT         GPIO_PORTM_BASE // Spindle direction
#define AUXOUTPUT4_PIN          4
#define AUXOUTPUT5_PORT         GPIO_PORTD_BASE // Spindle enable
#define AUXOUTPUT5_PIN          6
#define AUXOUTPUT6_PORT         GPIO_PORTL_BASE // Coolant flood
#define AUXOUTPUT6_PIN          1
#define AUXOUTPUT7_PORT         GPIO_PORTL_BASE // Coolant mist
#define AUXOUTPUT7_PIN          2

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT3_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT3_PIN
#define SPINDLE_PWM_MAP         GPIO_PM3_T3CCP1
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT5_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT5_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT6_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT6_PIN
#endif

#if CNC_BOOSTERPACK
#if CNC_BOOSTERPACK_SHORTS
#define AUXINPUT7_PORT          GPIO_PORTL_BASE
#define AUXINPUT7_PIN           0
#else
#define AUXINPUT7_PORT          GPIO_PORTG_BASE
#define AUXINPUT7_PIN           0
#endif
#else
#define AUXINPUT7_PORT          GPIO_PORTC_BASE
#define AUXINPUT7_PIN           4
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#if CNC_BOOSTERPACK
  #if CNC_BOOSTERPACK_SHORTS
    #define CONTROL_PORT        GPIO_PORTL_BASE
    #define CONTROL_PORT_SD     GPIO_PORTG_BASE
    #define RESET_PIN           0
    #define FEED_HOLD_PIN       4
    #define CYCLE_START_PIN     5
  #else
    #define RESET_PORT          GPIO_PORTF_BASE
    #define RESET_PIN           3
    #define FEED_HOLD_PORT      GPIO_PORTL_BASE
    #define FEED_HOLD_PIN       4
    #define CYCLE_START_PORT    GPIO_PORTL_BASE
    #define CYCLE_START_PIN     5
  #endif
#else
#define CONTROL_PORT            GPIO_PORTC_BASE
#define RESET_PIN               7
#define FEED_HOLD_PIN           6
#define CYCLE_START_PIN         5
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT7_PORT
#define SAFETY_DOOR_PIN         AUXINPUT7_PIN
#endif

// Define probe switch input pin.
#define PROBE_PORT              GPIO_PORTC_BASE
#define PROBE_PIN               7

// Input definitions

#ifndef AUXOUTPUT0_PORT
#define AUXINPUT0_PORT          AUXIO0_PORT
#define AUXINPUT0_PIN           AUXIO0_PIN
#endif

#ifndef AUXOUTPUT1_PORT
#define AUXINPUT1_PORT          AUXIO1_PORT
#define AUXINPUT1_PIN           AUXIO1_PIN
#endif

#if  MPG_ENABLE == 1
#define MPG_MODE_PORT           AUXIO2_PORT
#define MODE_GPIO               GPIO2_GPIO
#define MODE_INT                GPIO2_INT
#define MPG_MODE_PIN            AUXIO2_PIN
#else
#define AUXINPUT2_PORT          AUXIO2_PORT
#define AUXINPUT2_PIN           AUXIO2_PIN
#endif

#ifndef AUXOUTPUT2_PORT
#define AUXINPUT3_PORT          AUXIO3_PORT
#define AUXINPUT3_PIN           AUXIO3_PIN
#endif

#ifndef SERIAL2_MOD
#define AUXINPUT4_PORT          AUXIO4_PORT
#define AUXINPUT4_PIN           AUXIO4_PIN
#define AUXINPUT5_PORT          AUXIO5_PORT
#define AUXINPUT5_PIN           AUXIO5_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXIO6_PORT
#define I2C_STROBE_PIN          AUXIO6_PIN
#else
#define AUXINPUT6_PORT          AUXIO6_PORT
#define AUXINPUT6_PIN           AUXIO6_PIN
#endif

#if MOTOR_FAULT_ENABLE && defined(AUXINPUT6_PORT)
#define MOTOR_FAULT_PORT        AUXINPUT6_PORT
#define MOTOR_FAULT_PIN         AUXINPUT6_PIN
#endif

/**/

