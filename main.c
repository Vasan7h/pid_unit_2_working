#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "spi_functions.h"
#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "i2c_custom.h"
#include "analog.h"
int main()
{
    int choice;
    char option[2];
    while (1)
    {
        // printf("Options to choose : \n");
        // printf("\t1) GPIO Configuration\n");
        // printf("\t2) Power supply\n");
        // printf("\t3) Temperature \n");
        // // printf("\t4) Uart communication \n");
        // printf("\t4) Analog Input\n");
        // printf("\t5) Exit \n");
        // printf("Your choice : ");
        // scanf("%s", option);
        // // checking if valid input is entered or not
        // choice = atoi(option);
        // if (!choice)
        // {
        //     printf("Invalid Input choice !!!\n");
        //     continue;
        // }
        // else
        // {
        //     switch (choice)
        //     {
        //     case 1:
        //         GPIO_SET();
        //         break;
        //     case 2:
        //         spi_main();
        //         break;
        //     case 3:
        //        // dac_set(0,255, 255, 0);
        //         temp_main(0);
        //         break;
        //     // case 4:
        //     //     uart_main();
        //     //     break;
        //     case 4:
        //         analog_main();
        //         break;
        //     case 5:
        //         printf("Exiting program ...\n");
        //         exit(0);
        //         break;
        //     default:
        //         printf("Invalid choice ... !\n");
        //         break;
        //     }
        // }
        spi_main();
    }
    return 0;
}
