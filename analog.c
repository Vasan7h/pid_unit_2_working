#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include "gpio.h"
#include "analog.h"
int analog_main()
{
    gpio_configure_in(GPIO_PIN_NUMBER_1);
    int adc_fd = open(ADC_DEVICE_PATH_1, O_RDONLY);
    if (adc_fd == -1)
    {
        perror("Error opening ADC device file");
        return -1;
    }

    char adc_value_str[10];
    read(adc_fd, adc_value_str, sizeof(adc_value_str));
    close(adc_fd);
    double temp=0;
    
    int adc_value = atoi(adc_value_str);
    // printf("Current(raw): %d\n", adc_value);
    for(int i=0;i<10;i++)
    {
    gpio_configure_in(GPIO_PIN_NUMBER_2);
    adc_fd = open(ADC_DEVICE_PATH_2, O_RDONLY);
    if (adc_fd == -1)
    {
        perror("Error opening ADC device file");
        return -1;
    }

    adc_value_str[10];
    read(adc_fd, adc_value_str, sizeof(adc_value_str));
    close(adc_fd);

    adc_value = atoi(adc_value_str);
    // printf("Voltage:(raw) %d\n",adc_value);
        temp= temp+adc_value;
    }
    adc_value =temp/10;
    printf("Voltage out from pps: %f V\n", ((adc_value*0.0184627)+0.073962843));
    // sleep(1);
    
}

