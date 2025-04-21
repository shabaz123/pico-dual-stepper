/**************************************************************
 *  Dual Stepper Motor Control with TB67S581 and
 *  Stepper Motor Library from jancumps
 *  rev 1 - April 2025 - shabaz
 **************************************************************/

// ************* header files ******************
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "TB67S581.h"

#include "hardware/pio.h" // some constant definitions used
#include <array>          // for demo section (commands container)
#include <iterator>       // for demo section (commands container)
#include <span>           // for demo section (commands container)
#include <algorithm>      // for_each
#include <numeric>        // for demo section (accumulate)

import stepper;           // PIO stepper lib



// *********** defines ******************
// #define MICROSTEP_8
#undef MICROSTEP_8
//Board LED on the Pico W
#define PICO_LED_ON cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1)
#define PICO_LED_OFF cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0)
// I2C definitions
#define I2C_PORT i2c0
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQ 100000
// MCP23017 definition
#define EXP_ADDR 0x20
// motor names
#define MOTOR1 0
#define MOTOR2 1
// motor enable pins
#define M1_EN_PIN 6
#define M2_EN_PIN 7
// motor directions and enable/disable definitions
#define DIR_CW 1
#define DIR_CCW 0
#define MOTOR_DISABLE 1
#define MOTOR_ENABLE 0
// misc
#define FOREVER 1

// ******** external variables **********

// ********** global constants ***********
#ifdef MICROSTEP_8
const float clock_divider = 3; // works well for 8 microsteps
const uint microstep_x = 8;
#else
const float clock_divider = 16; // works well for no microsteps
const uint microstep_x = 1;
#endif

// TODO: adapt to your  board design
const uint motor1_dir = 19U; // implies that step is gpio 20
const uint motor2_dir = 21U; // implies that step is gpio 22

// ********** global variables ***********
using motor_t = stepper::stepper_callback_controller;
std::array<motor_t, 2> motors {{{pio0, 0}, {pio0, 1}}};

std::array<volatile size_t, motors.size()> commands_per_motor;

// ********** function prototypes *********

// ************** functions ***************
void
print_title(void) {
    printf("\n\n\n\n\n\n\n\n\n\n\n\n");
    printf("Project built on %s %s\n", __DATE__, __TIME__);
}

// general-purpose long delay timer if required
void
sleep_sec(uint32_t s) {
    sleep_ms(s * 1000);
}

// I2C helper function for I/O expander, to write 8-bit content to a register
void
exp_write8(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = data;
    i2c_write_blocking(I2C_PORT, EXP_ADDR, buf, 2, false);
}

// stepper motor command completion callback
void on_complete(const motor_t &stepper) {
    size_t index = 0U;
    for (auto &m: motors) {
        if (&m == &stepper) {
            commands_per_motor[index] =  commands_per_motor[index] - 1;
            printf("motor %d executed command\n", index + 1);
            break;
        }
        index++;
    }
}

void init_pio() {
    // program the pio used for the motors
    // do this only once per used pio
    int i=0;
    motor_t::pio_program(pio0); // not needed if all sms run on pio1
    motor_t::pio_program(pio1); // not needed if all sms run on pio0

    // individual settings
    motors[0].pio_init(motor1_dir, clock_divider);
    motors[1].pio_init(motor2_dir, clock_divider);
    // common settings
    // initialise and enable the motor state machine
    for (auto &m: motors) {
        m.register_pio_interrupt(0, true);
        if (i==1) { PICO_LED_ON; }
        m.enable(true);
        // and the notification, for demo purpose
        m.on_complete_callback(on_complete);
        i++;
    }
}

// board initialisation
void
board_init(void) {
    int i;

    // motor driver IC enable pin configuration
    gpio_init(M1_EN_PIN);
    gpio_set_dir(M1_EN_PIN, GPIO_OUT);
    gpio_init(M2_EN_PIN);
    gpio_set_dir(M2_EN_PIN, GPIO_OUT);
    // disable the motor outputs
    gpio_put(M1_EN_PIN, MOTOR_DISABLE);
    gpio_put(M2_EN_PIN, MOTOR_DISABLE);

    sleep_ms(100); // wait for devices to come out of power-on reset
    if (cyw43_arch_init()) { // need to call this before using the LED
        printf("ERROR - failed to initialise cyw43_arch!\n");
    }
    sleep_ms(500); // don't know if cyw43 needs a delay

    // I2C configuration, needed for both stepper motor instances
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    exp_write8(EXP_IOCONA, 0xf8); // set IOCON register on IO expander
    // set direction. Pins connected to TB7S581 MO and LO as inputs, rest as outputs:
    exp_write8(EXP_IODIRA, 0x21);
    exp_write8(EXP_IODIRB, 0x21);
    // set pins to a known state (sleep mode for the motor controller)
    // reset
    exp_write8(EXP_GPIOA, 0x08);
    exp_write8(EXP_GPIOB, 0x08);
    sleep_ms(1);
    // exit reset, remain in sleep mode
    exp_write8(EXP_GPIOA, 0x88);
    exp_write8(EXP_GPIOB, 0x88);
    sleep_ms(1);
    // exit sleep mode
    exp_write8(EXP_GPIOA, 0xc8);
    exp_write8(EXP_GPIOB, 0xc8);
    sleep_ms(20); // need minimum 10ms delay after sleep mode

    // stepper motor code initialisation
    init_pio(); // initialise the PIO used for the motors

}

// stepper demo: execute a series of commands ================================

// struct to hold command and motor
struct motor_command {
    stepper::command cmd;
    size_t motor;
    motor_command(stepper::command cmd, size_t motor) : cmd(cmd), motor(motor) {}
};

using commands_t = std::span<motor_command>;

void run_commands(const commands_t& cmd, uint delay0, uint delay1) {
    motors[0].set_delay(delay0);
    motors[1].set_delay(delay1);

    for (auto& count: commands_per_motor) { count = 0; }

    for (auto& c : cmd) {
        // increment commands expected for the motor
        commands_per_motor[c.motor] = commands_per_motor[c.motor] + 1;
    }

    for (auto& c : cmd) {
        if (commands_per_motor[c.motor] > 0U) {
            motors[c.motor].take_steps(c.cmd);
        }
    }

    size_t todo = 0U;
    do {
        todo = std::accumulate(commands_per_motor.begin(),commands_per_motor.end(),0);
    } while (todo > 0U);

    sleep_ms(100); // pause for demo purpose
}

void full_demo(const commands_t & cmd) {
    uint s1delay; // step delay for motor 1
    uint s2delay; // step delay for motor 2

    // enable the motor outputs
    gpio_put(M1_EN_PIN, MOTOR_ENABLE);
    gpio_put(M2_EN_PIN, MOTOR_ENABLE);
    sleep_ms(1);

    // demo 1 :run all in sequence
    s1delay = 9000;
    s2delay = 7000;
    for (auto &c: cmd) {
        std::array<motor_command,1> single_cmd {{c}};
        run_commands(single_cmd, s1delay, s2delay);
    }

    // demo 2: run all in parallel
    s1delay = 7000;
    s2delay = 4300;
    run_commands(cmd, s1delay, s2delay);

    // disable the motor outputs
    gpio_put(M1_EN_PIN, MOTOR_DISABLE);
    gpio_put(M2_EN_PIN, MOTOR_DISABLE);
}

// ******************************************
// ************** main function *************
// ******************************************
int
main(void) {
    uint32_t mdelay;

    stdio_init_all(); // can't use printf before this
    sleep_ms(3000); // wait for the console be connected/ready (can comment this out later)
    // print welcome message on the USB UART or Serial UART (selected in CMakelists.txt)
    print_title();

    board_init(); // GPIO and PIO initialisation


    std::array<motor_command, 4> cmd{{
                                             { {200 * microstep_x, DIR_CW}, MOTOR1},
                                             { {200 * microstep_x, DIR_CCW}, MOTOR1},
                                             { {200 * microstep_x, DIR_CW}, MOTOR2},
                                             { {200 * microstep_x, DIR_CCW}, MOTOR2},
                                     }};



    while (true) {
        full_demo(cmd);
        sleep_ms(200); // pause for demo purpose
    }
    return(0); // won't execute; warning on this line is OK
}
