#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "print_custom.h"

char printf_buffer[128] = "";

void debug_printf(char message[])
{
    printf("[DEBUG]  : %s\n",message);
    memset(printf_buffer, 0, strlen(printf_buffer));
}

void error_printf(char message[])
{
    printf("[ERROR]  : %s\n",message);
    memset(printf_buffer, 0, strlen(printf_buffer));
}
