
#ifndef HEADER_CONSOLE_H
#define HEADER_CONSOLE_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#define COMMAND_NONE 0
#define COMMAND_SPIN 1
#define COMMAND_DISABLE_MOTORS 2
#define COMMAND_PRINT_MENU 3

#define CONSOLE_BUF_SIZE 300

#define TOKEN_RESULT_ERROR 0
#define TOKEN_RESULT_OK 1
#define TOKEN_RESULT_LINE_COMPLETE 2

#define TOKEN_PROGRESS_NONE 0
#define TOKEN_PROGRESS_SEND 1
#define TOKEN_PROGRESS_RECV 2

#define COL_RED printf("\033[31m")
#define COL_GREEN printf("\033[32m")
#define COL_YELLOW printf("\033[33m")
#define COL_BLUE printf("\033[34m")
#define COL_MAGENTA printf("\033[35m")
#define COL_CYAN printf("\033[36m")
#define COL_RESET printf("\033[0m")

// this function initializes the console and parameters
void init_console(void);

// print a menu
void console_menu(void);

// print a prompt
void console_prompt(void);

// this function reads the console and fills the buffer until newline is received.
// returns number of bytes if a newline is received, 0 otherwise
// set up a timer and call this function periodically
int scan_console_input(void);

// call this function whenever scan_console_input returns a positive value greater than 0
int process_line(uint8_t *buf, uint16_t len);

// helper function to print a buffer in hex format
void print_buf_hex(uint8_t *buf, uint16_t len);

/*******************************************************
 * functions that can be executed from the console
 *******************************************************/
void spin_motor(uint8_t motornum, uint32_t steps, uint8_t dir, uint step_delay);


#endif //HEADER_CONSOLE_H
