//
// TB67S581 stepper motor driver header file
// rev 1 - Oct 2023 - shabaz
// The driver uses a TB67S581 stepper motor driver IC along with a MCP23017 IO expander
// The MCP23017 is shared between two TB67S581 ICs, and is used to control the DMODE0-2 pins
// and the DECAY, SLEEP and RESET pins of the TB67S581 ICs. The MO and LO pins are inputs to
// the MCP23017.
// BANK B of the MCP23017 is used for the first TB67S581 IC, and BANK A is used for the second.
// MCP23017 port bit to TB67S581 pin mapping:
// 0: MO
// 1: DMODE2
// 2: DMODE1
// 3: DMODE0
// 4: DECAY
// 5: LO
// 6: SLEEP
// 7: RESET

#ifndef HEADER_TB67S581_H
#define HEADER_TB67S581_H

// includes
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/util/queue.h"

// defines
// MCP23017 definitions (when IOCON.BANK=1)
#define EXP_IODIRA 0x00
#define EXP_IOCON_ORIG 0x0A
#define EXP_IOCON_IN_SEQUENTIAL_MODE 0x05
#define EXP_GPIOA 0x09
#define EXP_OLATA 0x0A
#define EXP_IODIRB 0x10
#define EXP_IOCONB 0x15
#define EXP_GPIOB 0x19
#define EXP_OLATB 0x1A


// STEPS_PER_TURN is (360 degrees divided by the angle per step) * 2 (for half steps)
// for a 1.8 degree stepper motor, this is (360/1.8)*2 = 200*2 = 400
#define STEPS_PER_TURN (200*2)
#define IO_EXP_PORT_A 0
#define IO_EXP_PORT_B 1
#define M1_OFF gpio_put(_i01_pin, 0)

// class definition
class TB67S581
{
public:
    TB67S581(uint dir_pin, uint ena_pin, uint clk_pin, int io_exp_addr, char io_exp_port);
    void init();


private:
    uint _dir_pin;
    uint _ena_pin;
    uint _clk_pin;
    int _io_exp_addr;
    char _io_exp_port;
    int _sequence_state; // 0 to 7 for half-step mode (total 8 steps)
    int _num_steps; // number of steps
    int _turns; // number of turns
    int _period; // period in 100 microseconds increments per step
    int _direction; // direction of rotation (0 or 1)
    int _ramp; // 0=no ramp, -1=slow down to target_period, +1=speed up to target_period
    int _target_period; // target period in 100 microseconds increments per step
    int _ramp_internal_counter; // internal counter for ramping
    int _motion_state; // 0=STOP, 1=GO
    int _desired_turns; // desired number of turns. Set to zero for unlimited turns until stopped

};


#endif //HEADER_TB67S581_H
