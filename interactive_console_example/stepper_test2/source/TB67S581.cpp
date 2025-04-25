//
// TB67S581 stepper motor driver code
// rev 1 - Oct 2023 - shabaz
// This code contains the interrupt routines and the class implementation for the
// TB67S581 stepper motor driver.
// The driver makes use of a MCP23017 IO expander to control many of the pins on the TB67S581.
// One MCP23017 is shared between two TB67S581 ICs (MCP23017 port A for one, port B for the other).
//

#include "TB67S581.h"

// *****************************************************
// ************* interrupt capabilities ****************
// *****************************************************




// *****************************************************
// ************* class implementation ******************
// *****************************************************

TB67S581::TB67S581(uint dir_pin, uint ena_pin, uint clk_pin, int io_exp_addr, char io_exp_port)
        : _dir_pin(dir_pin), _ena_pin(ena_pin), _clk_pin(clk_pin), _io_exp_addr(io_exp_addr), _io_exp_port(io_exp_port)
{
    gpio_init(_dir_pin);
    gpio_init(_ena_pin);
    gpio_init(_clk_pin);
    gpio_set_dir(_dir_pin, GPIO_OUT);
    gpio_set_dir(_ena_pin, GPIO_OUT);
    gpio_set_dir(_clk_pin, GPIO_OUT);
    gpio_put(_ena_pin, 1); // high will set MOSFETs to high impedance
    gpio_put(_dir_pin, 0);
    gpio_put(_clk_pin, 0);

}

void TB67S581::init() {
    _sequence_state = 0;
    _num_steps = 0;
    _turns = 0;
    _ramp = 0;
    _direction = 0;
    _period = 5;
    _target_period = 5;
    _ramp_internal_counter = 0;
    _desired_turns = 0;
    //
}
