/*
  my_machine.h - configuration for Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

// NOTE: Only one board may be enabled!
#define BOARD_CNC_BOOSTERPACK
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable.

//#define WEBUI_ENABLE            1 // Enable ESP3D-WEBUI plugin along with networking and SD card plugins.
//#define ETHERNET_ENABLE         1 // Ethernet streaming. Requires networking plugin.
//#define SDCARD_ENABLE           1 // Run gcode programs from SD card, requires sdcard plugin.
//#define LITTLEFS_ENABLE         1 // Enable flash based storage, automatically enabled if WebUI is enabled
//#define MPG_ENABLE              1 // Enable MPG interface. Requires a serial port and means to switch between normal and MPG mode.
                                    // 1: Mode switching is by handshake pin input unless the keypad plugin is enabled in mode 2 which
                                    //    uses mode switching by the CMD_MPG_MODE_TOGGLE (0x8B) command character.
                                    // 2: Mode switching is by the CMD_MPG_MODE_TOGGLE command character. The keypad plugin is not required.
//#define KEYPAD_ENABLE           1 // 1: uses a I2C keypad for input.
                                    // 2: uses a serial port for input. If MPG_ENABLE is set to 1 the serial stream is shared with the MPG.
//#define PWM_RAMPED              1 // Ramped spindle PWM.
//#define LASER_PPI               1 // Laser PPI (Pulses Per Inch) option.
//#define TRINAMIC_ENABLE      2130 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_ENABLE      5160 // Trinamic TMC5160 stepper driver support. NOTE: work in progress.
//#define TRINAMIC_I2C            0 // Trinamic I2C - SPI bridge interface.
// Optional control signals:
// These will be assigned to aux input pins. Use the $pins command to check which pins are assigned.
// NOTE: If not enough pins are available assignment will silently fail.
//#define SAFETY_DOOR_ENABLE      1
//#define MOTOR_FAULT_ENABLE      1
//#define MOTOR_WARNING_ENABLE    1
//#define PROBE_DISCONNECT_ENABLE 1
//#define STOP_DISABLE_ENABLE     1
//#define BLOCK_DELETE_ENABLE     1
//#define SINGLE_BLOCK_ENABLE     1
//#define LIMITS_OVERRIDE_ENABLE  1

#ifdef BOARD_CNC_BOOSTERPACK
//#define CNC_BOOSTERPACK_SHORTS  1 // Shorts added to BoosterPack for some signals (for faster and simpler driver)
#define CNC_BOOSTERPACK_A4998   1 // Using Polulu A4998 drivers - for suppying VDD via GPIO (PE5)
#endif

#if ETHERNET_ENABLE
#if SDCARD_ENABLE
#define FTP_ENABLE              1 // ftp daemon - requires SD card write enabled (2).
#define HTTP_ENABLE             1 // http daemon - requires SD card write enabled (2).
#define WEBDAV_ENABLE           1 // webdav protocol - requires http daemon and SD card enabled.
#endif
#define TELNET_ENABLE           1 // Telnet daemon - requires Ethernet streaming enabled.
#define WEBSOCKET_ENABLE        1 // Websocket daemon - requires Ethernet streaming enabled.
#define SSDP_ENABLE        1 // Websocket daemon - requires Ethernet streaming enabled.
// The following symbols have the default values as shown, uncomment and change as needed.
//#define NETWORK_HOSTNAME        "GRBL"
//#define NETWORK_IPMODE          1 // do not change! Cannot get static mode to work!
//#define NETWORK_IP              "192.168.5.1"
//#define NETWORK_GATEWAY         "192.168.5.1"
//#define NETWORK_MASK            "255.255.255.0"
//#define NETWORK_FTP_PORT        21
//#define NETWORK_TELNET_PORT     23
//#define NETWORK_WEBSOCKET_PORT  81
//#define NETWORK_HTTP_PORT       80
#endif
