#define _ANALOG_H

#define GPIO_PIN_NUMBER_1 1 // GPIO pin number
#define GPIO_PIN_NUMBER_2 4 // GPIO pin number
#define ADC_DEVICE_PATH_1 "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define ADC_DEVICE_PATH_2 "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"

int analog_main();