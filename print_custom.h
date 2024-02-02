#ifndef PRINT_CUSTOM_H
#define PRINT_CUSTOM_H

//Global variable
extern char printf_buffer[128];

//Functions
void debug_printf(char message[]);
void error_printf(char message[]);
#endif