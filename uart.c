#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include "uart.h"
#include "gpio.h"
int uart_main()
{
    const char *uart_device = "/dev/ttymxc3"; // Replace with the appropriate UART device
    int uart_fd;
    struct termios tio;

    // Open the UART port
    uart_fd = open(uart_device, O_RDWR | O_NOCTTY);
    if (uart_fd < 0)
    {
        perror("Failed to open UART");
        return 1;
    }
    else
        printf("Initial setup is successful\n");

    // Configure UART settings
    tcgetattr(uart_fd, &tio);
    tio.c_cflag = B4800 | CS8 | CLOCAL | CREAD; // Baud rate and 8N1 mode
    tio.c_oflag = 0;
    tcsetattr(uart_fd, TCSANOW, &tio);
    char reverseBits(char num)
    {
        char NO_OF_BITS = sizeof(num) * 8;
        char reverse_num = 0;
        int i;
        for (i = 0; i < NO_OF_BITS; i++)
        {
            if ((num & (1 << i)))
                reverse_num |= 1 << ((NO_OF_BITS - 1) - i);
        }
        return reverse_num;
    }
    gpio_configure_out(66); // pps control gpio
    // Data as ASCII codes
    while (1)
    {

        char glob_on[] = {0xE2, 0x32, 0xF2, 0x42, 0x04, 0x8C, 0xB0, 0x50}; //,0x4C,0x4F,0x42,0x20,0x31,0x0D,0x0A};
        // const char glob_on_ref[]={0x47};
        // for(int i=0;i<1;i++)
        // {
        //     glob_on[i]=reverseBits(glob_on_ref[i]);
        // }ss
        // for(int j =0 ;j<1;j++).2

        // {
        //     printf("%x\n",glob_on[j]);
        // }
        const char glob_off[] = {0x47, 0x4C, 0x4F, 0x42, 0x20, 0x30, 0x0D, 0x0A};
        const char ascii_data_voltage[] = {0x53, 0x56, 0x20, 0x30, 0x20, 0x0D, 0x20, 0x0A}; // ASCII codes for "SV 0 CR LF"
        const char ascii_data_current[] = {0x53, 0x49, 0x20, 0x30, 0x20, 0x0D, 0x20, 0x0A};
        // const char ascii_data_voltage[] ={};
        size_t data_len_volt = sizeof(ascii_data_voltage);
        size_t data_len_cur = sizeof(ascii_data_current);
        size_t glob_len_on = sizeof(glob_on);
        size_t glob_len_off = sizeof(glob_off);
        // Send data
        // const char *ascii_data = "SV 11.95\r\n"; // Include the desired string with CR (Carriage Return) and LF (Line Feed)
        // size_t data_len = strlen(ascii_data);

        // // Send data
        // ssize_t bytes_written = write(uart_fd, ascii_data, data_len);
        // if (bytes_written < 0) {
        //     perror("Failed to write to UART");
        //     close(uart_fd);
        //     return 1;
        // }

        // printf("Sent data: %s", ascii_data);

        ssize_t glob_bytes_written_volt = write(uart_fd, glob_on, glob_len_on);
        if (glob_bytes_written_volt < 0)
        {
            perror("Failed to write to UART");
            close(uart_fd);
            return 1;
        }
        else
        {
            printf("Write success for glob on\n");
        }
        // ssize_t bytes_written_volt = write(uart_fd, ascii_data_voltage, data_len_volt);
        // if (bytes_written_volt < 0) {
        //     perror("Failed to write to UART");
        //     close(uart_fd);
        //     return 1;
        // }
        // else
        // {
        //     printf("Write success for voltage\n");
        // }
        // ssize_t bytes_written_cur = write(uart_fd, ascii_data_current, data_len_cur);
        // if (bytes_written_cur < 0) {
        //     perror("Failed to write to UART");
        //     close(uart_fd);
        //     return 1;
        // }
        // else
        // {
        //     printf("Write success for current\n");
        // }
        // printf("\tSent ASCII codes: ");
        // for (size_t i = 0; i < data_len_volt; i++) {
        //     printf("%c", ascii_data_voltage[i]);
        // }
        // printf("\n");
        // printf("\tSent ASCII codes: ");
        // for (size_t i = 0; i < data_len_cur; i++) {
        //     printf("%c", ascii_data_current[i]);
        // }
        // sleep(5);
        // printf("\n");
        // // Close the UART port
        // ssize_t glob__off_bytes_written_volt = write(uart_fd, glob_off, glob_len_off);
        // if (glob__off_bytes_written_volt < 0) {
        //     perror("Failed to write to UART");
        //     close(uart_fd);
        //     return 1;
        // }
        // else
        // {
        //     printf("Write success for glob off\n");
        // }

        // gpio_configure_out(66, 1);
        // close(uart_fd);
    }
    return 0;
}
