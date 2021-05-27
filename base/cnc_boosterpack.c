/*
  cnc_boosterpack.c - pin aux I/O handling for CNC BoosterPack

  - on Texas Instruments MSP432P401R LaunchPad

  Copyright (c) 2021 Terje Io

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

#if defined(CNC_BOOSTERPACK)

//#include "Arduino.h"
#include <math.h>
#include <string.h>

#include "grbl/protocol.h"

static input_signal_t *aux_in;
static output_signal_t *aux_out;
static ioport_bus_t out = {0};

static void aux_settings_load (void);
static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t int_value);
static uint32_t aux_get_invert_out (setting_id_t setting);
static char input_ports[56] = "", output_ports[56] = "";

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, input_ports, NULL, NULL, Setting_NonCore, &settings.ioport.invert_in.mask },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, output_ports, NULL, NULL, Setting_NonCoreFn, aux_set_invert_out, aux_get_invert_out },
};

static setting_details_t details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t),
    .load = aux_settings_load,
    .save = settings_write_global
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

static void aux_settings_load (void)
{
    uint_fast8_t port = hal.port.num_digital_out;

    do {
        port--;
        GPIOPinWrite(aux_out[port].port, aux_out[port].pin, (settings.ioport.invert_out.mask >> port) & 0x01 ? aux_out[port].pin : 0);
    } while(port);
}

static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t value)
{
    ioport_bus_t invert;
    invert.mask = (uint8_t)value & out.mask;

    if(invert.mask != settings.ioport.invert_out.mask) {
        uint_fast8_t port = hal.port.num_digital_out;
        do {
            port--;
            if(((settings.ioport.invert_out.mask >> port) & 0x01) != ((invert.mask >> port) & 0x01))
                GPIOPinWrite(aux_out[port].port, aux_out[port].pin, GPIOPinRead(aux_out[port].port, aux_out[port].pin) ? 0 : aux_out[port].pin);
        } while(port);

        settings.ioport.invert_out.mask = invert.mask;
    }

    return Status_OK;
}

static uint32_t aux_get_invert_out (setting_id_t setting)
{
    return settings.ioport.invert_out.mask;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < hal.port.num_digital_out)
        GPIOPinWrite(aux_out[port].port, aux_out[port].pin, (on ^ (settings.ioport.invert_out.mask >> port) & 0x01) ? aux_out[port].pin : 0);
}

inline static __attribute__((always_inline)) int32_t get_input (input_signal_t *inp, bool invert, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return !!GPIOPinRead(inp->port, inp->pin) ^ invert;

    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    bool wait_for = wait_mode != WaitMode_Low;

    do {
        if((!!GPIOPinRead(inp->port, inp->pin) ^ invert) == wait_for)
            return !!GPIOPinRead(inp->port, inp->pin);

        if(delay) {
            protocol_execute_realtime();
            hal.delay_ms(50, NULL);
        } else
            break;
    } while(--delay && !sys.abort);

    return -1;
}

static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(digital) {
        if(port < hal.port.num_digital_in)
            value = get_input(&aux_in[port], (settings.ioport.invert_in.mask << port) & 0x01, wait_mode, timeout);
    }
//    else if(port == 0)
//        value = analogRead(41);
/*
    hal.stream.write("[MSG:AUX");
    hal.stream.write(uitoa(port));
    hal.stream.write("=");
    hal.stream.write(value == -1 ? "fail" : uitoa(value));
    hal.stream.write("]" ASCII_EOL);
*/
    return value;
}

void board_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in = aux_inputs->pins.inputs;
    aux_out = aux_outputs->pins.outputs;

    hal.port.digital_out = digital_out;
    hal.port.wait_on_input = wait_on_input;
//    hal.port.num_analog_in  = 1;
    hal.port.num_digital_in = aux_inputs->n_pins;
    hal.port.num_digital_out = aux_outputs->n_pins;

    details.on_get_settings = grbl.on_get_settings;
    grbl.on_get_settings = on_get_settings;

    uint_fast8_t i;

    for(i = 0; i < min(hal.port.num_digital_in, 8); i++) {
        strcat(input_ports, i == 0 ? "Port " : ",Port ");
        strcat(input_ports, uitoa(i));
    }

    for(i = 0; i < min(hal.port.num_digital_out, 8); i++) {
        strcat(output_ports, i == 0 ? "Port " : ",Port ");
        strcat(output_ports, uitoa(i));
    }

//    analog_init();
}

#endif
