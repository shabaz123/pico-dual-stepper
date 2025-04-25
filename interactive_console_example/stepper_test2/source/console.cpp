/************************************************************
 * console.cpp
 * rev 1 - shabaz - April 2025
 * Simple command parser, used for console input/output
 ************************************************************/

#include "console.h"

uint8_t console_buffer[CONSOLE_BUF_SIZE];
uint16_t console_buffer_index;
uint8_t do_echo = 1;
uint8_t token_progress = TOKEN_PROGRESS_NONE;

// some user commands may require hex bytes to be input/output
// used when expecting a certain desired number of bytes to be input by the user
int expected_num_bytes = 0;
// bytes for user inout/output are contained in this buffer
uint8_t byte_buffer[256];
uint8_t byte_buffer_index = 0;

// variables used for motor related functions
uint step_delay = 4000;

// parameters used by main function to execute commands
uint8_t param_command;
uint8_t param_motornum;
uint32_t param_steps;
uint param_step_delay;
uint8_t param_dir;


// function prototypes
int decode_token(char *token);


// functions
/**************************************************
 *  init_console
 *  initializes the console and parameters
 **************************************************/
void
init_console(void) {
    console_buffer_index = 0;
    expected_num_bytes = 0;
    byte_buffer_index = 0;
    token_progress = TOKEN_PROGRESS_NONE;
    step_delay = 4000; // default step delay
    param_command = COMMAND_NONE;
    param_motornum = 0;
    param_steps = 0;
    param_step_delay = step_delay;
    param_dir = 0;
}

void
console_menu(void) {
    COL_GREEN;
    printf("COMMANDS\n");
    printf("device?\n");
    printf("      print device name\n");
    printf("step_delay:<delay>\n");
    printf("      set per-step delay in usec, e.g.: step_delay:5000\n");
    printf("spin_cw:<motor_num>,<steps>\n");
    printf("      spin motor CW a number of steps, e.g. spin_cw:0,200\n");
    printf("spin_ccw:<motor_num>,<steps>\n");
    printf("      spin motor CCW a number of steps, e.g. spin_ccw:0,200\n");
    COL_RESET;
}

void
console_prompt(void) {
    printf("\n");
    COL_YELLOW;
    printf("console> ");
    COL_RESET;
}

/**************************************************
 *  scan_console_input
 *  reads the console and returns non-zero
 *  number of bytes, greater than 0
 *  if a newline is received
 **************************************************/
int
scan_console_input(void) {
    int c;
    uint16_t num_bytes;
    c = getchar_timeout_us(1000);
    if (c == PICO_ERROR_TIMEOUT) {
        return 0;
    }

    if ((c == 8) || (c==127)) { // backspace pressed
        if (console_buffer_index > 0) {
            console_buffer_index--;
            if (do_echo) {
                putchar(8);
                putchar(' ');
                putchar(8);
            }
        }
        return 0;
    }
    if (c == 13) {
        // add a space to simplify token parsing
        console_buffer[console_buffer_index++] = ' ';
        console_buffer[console_buffer_index] = 0;
        num_bytes = console_buffer_index;
        console_buffer_index = 0;
        if (do_echo) {
            printf("\n");
        }
        return num_bytes;
    }
    if ((c=='?') && (console_buffer_index == 0)) {
        // this is the shortcut to print the menu
        if (do_echo) {
            printf("?\n");
        }
        param_command = COMMAND_PRINT_MENU;
        return 0;
    }
    console_buffer[console_buffer_index] = (uint8_t) c;
    putchar(c);
    console_buffer_index++;
    if (console_buffer_index >= (CONSOLE_BUF_SIZE-5)) {
        console_buffer_index = 0;
    }
    return 0;
}

/**************************************************
 *  process_line
 *  parse each space-separated token
 **************************************************/
int process_line(uint8_t *buf, uint16_t len) {
    int res;
    char token[20];
    uint16_t i = 0;
    uint16_t j = 0;
    if (len == 0) {
        return TOKEN_RESULT_ERROR;
    }
    while (i < len) {
        if (buf[i] == ' ') {
            token[j] = 0;
            //printf("token: %s\n", token);
            res = decode_token(token);
            if (res == TOKEN_RESULT_LINE_COMPLETE) {
                return TOKEN_RESULT_LINE_COMPLETE;
            }
            j = 0;
        } else {
            token[j] = buf[i];
            j++;
        }
        i++;
    }
    res = decode_token((char*)"end_tok");
    return(TOKEN_RESULT_LINE_COMPLETE); // need to check that this is OK
}

/**************************************************
 *  decode_token
 *  this function decodes the console instructions
 **************************************************/
int decode_token(char *token) {
    unsigned int val;
    uint32_t steps;
    unsigned int motornum;
    int ioport, ioval; // used for the iowrite and ioread commands
    int port_valid;
    int retval = 0;
    if ((strncmp(token, "help", 4) == 0) ||
        (strncmp(token, "?", 1) == 0) ||
        (strncmp(token, "menu", 4) == 0)) {
        param_command = COMMAND_PRINT_MENU;
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strcmp(token, "device?") == 0) {
        printf("controller\n\r");
        // reset any state and variables
        token_progress = TOKEN_PROGRESS_NONE;
        expected_num_bytes = 0;
        byte_buffer_index = 0;
        console_prompt();
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strncmp(token, "step_delay:", 11) == 0) {
        sscanf(token, "step_delay:%d", &val);
        if (val < 100) {
            COL_RED;
            printf("step_delay is too low!\n");
            COL_RESET;
        } else {
            COL_BLUE;
            printf("step_delay set to %d\n", val);
            COL_RESET;
            step_delay = val;
        }
        console_prompt();
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strncmp(token, "spin_cw:", 8) == 0) {
        sscanf(token, "spin_cw:%d,%u", &motornum, &steps);
        COL_BLUE;
        printf("spin_cw:%d,%lu command received\n", motornum, (unsigned long) steps);
        COL_RESET;
        param_motornum = motornum;
        param_steps = steps;
        param_step_delay = step_delay;
        param_dir = 1; // CW
        param_command = COMMAND_SPIN;
        console_prompt();
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    if (strncmp(token, "spin_ccw:", 9) == 0) {
        sscanf(token, "spin_ccw:%d,%u", &motornum, &steps);
        COL_BLUE;
        printf("spin_ccw:%d,%lu command received\n", motornum, (unsigned long) steps);
        COL_RESET;
        param_motornum = motornum;
        param_steps = steps;
        param_step_delay = step_delay;
        param_dir = 0; // CCW
        param_command = COMMAND_SPIN;
        console_prompt();
        return TOKEN_RESULT_LINE_COMPLETE;
    }
    // done
    if (strlen(token) > 0) {
        COL_RED;
        printf("Unknown command: '%s' Type '?' for menu\n", token);
        COL_RESET;
    }
    console_prompt();
    return TOKEN_RESULT_LINE_COMPLETE;
}
/**************************************************
 *  print_buf_hex
 *  prints a buffer in hex format, example:
 *  000: 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F : 0123456789ABCDEF
 ******************************************/
void
print_buf_hex(uint8_t *buf, uint16_t len) {
    uint16_t i, j;
    uint8_t c;
    uint8_t index = 0;

    for (i = 0; i < len; i += 16) {
        COL_BLUE;
        printf("%03d: ", index);
        COL_CYAN;
        for (j = 0; j < 16; j++) {
            if (i + j < len) {
                printf("%02X ", buf[i + j]);
            } else {
                printf("   ");
            }
        }
        COL_BLUE;
        printf(": ");
        COL_GREEN;
        for (j = 0; j < 16; j++) {
            if (i + j < len) {
                c = buf[i + j];
                if ((c < 32) || (c > 126)) {
                    printf(".");
                } else {
                    printf("%c", c);
                }
            } else {
                printf(" ");
            }
        }
        printf("\n");
        index += 16;
    }
    COL_RESET;
}
