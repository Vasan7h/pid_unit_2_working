#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include "gpio.h"
#include "print_custom.h"
uint8_t led_toggle_count = 0;
uint8_t gpio_export(uint8_t pin_num)
{
    char buffer[64];
    uint8_t fd = open(SYSFS_GPIO_PATH "/export", O_WRONLY);
    if (fd == -1)
    {
        perror("gpio_export: Failed to open export for writing");
        return -1;
    }

    uint8_t len = snprintf(buffer, sizeof(buffer), "%d", pin_num);
    if (write(fd, buffer, len) != len)
    {
        perror("gpio_export: Failed to write to export");
        printf("The GPIO pin num %d is in use by System. Can't use that pin\n", pin_num);
        return -1;
    }
    close(fd);
    return 0;
}

uint8_t gpio_unexport(uint8_t pin_num)
{
    uint8_t fd, len;
    char buffer[64];
    fd = open(SYSFS_GPIO_PATH "/unexport", O_WRONLY);
    if (fd == -1)
    {
        perror("gpio_unexport: Failed to open unexport for writing");
        return -1;
    }

    len = snprintf(buffer, sizeof(buffer), "%d", pin_num);
    if (write(fd, buffer, len) != len)
    {
        perror("gpio_unexport: Failed to write to unexport");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

uint8_t gpio_configure_out(uint8_t pin_num)
{
    uint8_t fd;
    char path[64];

    // STEP 0 : CHECK FIRST IF THE PIN_NUM IS EXPORTED OR NOT
    if (gpio_is_exported(pin_num))
    {
        // snprintf(printf_buffer, sizeof(printf_buffer), "Can't export pin %d. Already in Use", pin_num);
        debug_printf(printf_buffer);
        int ret = direction_check(pin_num);
        if (ret)
        {
            return 0;
        }
        else
        {
            // STEP 2 : PIN DIRECTION
            snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/direction", pin_num);
            fd = open(path, O_WRONLY);
            if (fd == -1)
            {
                perror("gpio_set_direction: Failed to open direction for writing");
                return -1;
            }

            if (write(fd, "out", strlen("out")) == -1)
            {
                perror("gpio_set_direction: Failed to write direction");
                close(fd);
                return -1;
            }
        }
        close(fd);
        return -1;
    }
    else
    {
        // STEP 1 : PIN EXPORT
        uint8_t ret = gpio_export(pin_num);
        if (ret == -1)
            return -1;

        // STEP 2 : PIN DIRECTION
        snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/direction", pin_num);
        fd = open(path, O_WRONLY);
        if (fd == -1)
        {
            perror("gpio_set_direction: Failed to open direction for writing");
            return -1;
        }

        if (write(fd, "out", strlen("out")) == -1)
        {
            perror("gpio_set_direction: Failed to write direction");
            close(fd);
            return -1;
        }
    }
    close(fd);
    return 0;
}

/*
@brief - function checks if the pin_num is already exported or not.
@retval - 1 - if already exported
@retval - 0 - if not exported
@param - pin_num
*/
uint8_t gpio_is_exported(uint8_t pin_num)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin_num);

    // Access the directory to check if it exists
    if (access(path, F_OK) != -1)
        return 1;
    else
        return 0;
}

uint8_t fd_gpio;
char buffer_temp[64];
int8_t init_gpio(uint8_t pin_num)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin_num);
    fd_gpio = open(path, O_WRONLY);
    return fd_gpio;
}
int direction_check(int gpio_pin)
{
    // Replace with the GPIO pin number you want to check
    char gpio_direction_file[64];
    char direction[64];

    // Create the file path for the GPIO direction file
    snprintf(gpio_direction_file, sizeof(gpio_direction_file), "/sys/class/gpio/gpio%d/direction", gpio_pin);

    // Open the file for reading
    FILE *file = fopen(gpio_direction_file, "r");
    if (file == NULL)
    {
        perror("Failed to open direction file");
        return 1;
    }

    // Read the direction from the file
    if (fgets(direction, sizeof(direction), file) == NULL)
    {
        perror("Failed to read direction");
        fclose(file);
        return 1;
    }

    // Close the file
    fclose(file);

    // Check and print the direction
    if (strcmp(direction, "in\n") == 0)
    {
        printf("GPIO %d is configured as an input.\n", gpio_pin);
        return 0;
    }
    else if (strcmp(direction, "out\n") == 0)
    {
        printf("GPIO %d is configured as an output.\n", gpio_pin);
        return 1;
    }
    else
    {
        printf("Unknown direction for GPIO %d: %s", gpio_pin, direction);
    }
    // return 0;
}

/*Configures the pin_num to high if its direction is set to out
if not returns -1
*/
uint8_t set_gpio(uint8_t pin_num, uint8_t value)
{
    char path[64];
    // STEP 1: CHECK IF THE PIN IS EXPORTED OR NOT
    if (gpio_is_exported(pin_num))
    {
        // printf("\n");
        // direction_check(pin_num);
        // STEP 2: CHECK IF THE PIN VALUE ENTERED IS VALID OR NOT
        if (value == 1 || value == 0)
        {
            // STEP 3: WRITING TO THE PIN'S VALUE ATTRIBUTE
            snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/value", pin_num);
            fd_gpio = open(path, O_WRONLY);
            if (fd_gpio == -1)
            {
                perror("gpio_write: Failed to open value for writing");
                return -1;
            }

            if (write(fd_gpio, value ? "1" : "0", 1) != 1)
            {
                perror("gpio_write: Failed to write value");
                close(fd_gpio);
                return -1;
            }
            close(fd_gpio);
            return 0;
        }
        else
        {
            printf("Invalid pin value entered !!!");
            close(fd_gpio);
            return -1;
        }
    }
    else
    {
        // if not exported means configure that pin to output
        gpio_configure_out(pin_num);
        // STEP 2: CHECK IF THE PIN VALUE ENTERED IS VALID OR NOT
        // direction_check(pin_num);
        if (value == 1 || value == 0)
        {
            snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/value", pin_num);
            fd_gpio = open(path, O_WRONLY);
            if (fd_gpio == -1)
            {
                perror("gpio_write: Failed to open value for writing");
                return -1;
            }

            if (write(fd_gpio, value ? "1" : "0", 1) != 1)
            {
                perror("gpio_write: Failed to write value");
                close(fd_gpio);
                return -1;
            }
            close(fd_gpio);
            return 0;
        }
        else
        {
            printf("Invalid pin value entered !!!");
            close(fd_gpio);
            return -1;
        }
    }
}

int gpio_get_fd_to_value(int gpio_num)
{
    FILE *fp;
    char buffer[10];
    char *endptr;
    uint8_t curr_output_port = 0; // Initialize to 0
    char command[100];            // Adjust the buffer size as needed

    snprintf(command, sizeof(command), "cat /sys/class/gpio/gpio%d/value", gpio_num);
    fp = popen(command, "r");
    if (fp == NULL)
    {
        perror("Error : ");
        return -1; // Return an error code to indicate failure
    }
    else
    {
        if (fgets(buffer, sizeof(buffer), fp) != NULL)
        {
            curr_output_port = strtol(buffer, &endptr, 10); // Use base 10
            pclose(fp);                                     // Close the pipe when done

            if (curr_output_port == 1)
            {
                return 1;
            }
            else if (curr_output_port == 0)
            {
                return 0;
            }
        }
    }
    fclose(fp);
    return -1; // Return an error code if something went wrong
}
uint8_t gpio_configure_in(uint8_t pin_num)
{
    uint8_t fd;
    char path[64];
    // STEP 0 : UNEXPORT
    if (gpio_is_exported(pin_num))
    {
        gpio_unexport(pin_num); // Unexport the pin if already exported
        uint8_t ret = gpio_export(pin_num);
        if (ret == -1)
            return -1;

        // STEP 2 : PIN DIRECTION
        snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/direction", pin_num);
        fd = open(path, O_WRONLY);
        if (fd == -1)
        {
            perror("gpio_set_direction: Failed to open direction for writing");
            return -1;
        }

        if (write(fd, "in", strlen("in")) == -1)
        {
            perror("gpio_set_direction: Failed to write direction");
            close(fd);
            return -1;
        }
        // direction_check(pin_num);
        close(fd);
        return 0;
    }
    else
    {
        // STEP 1 : PIN EXPORT
        uint8_t ret = gpio_export(pin_num);
        if (ret == -1)
            return -1;

        // STEP 2 : PIN DIRECTION
        snprintf(path, sizeof(path), SYSFS_GPIO_PATH "/gpio%d/direction", pin_num);
        fd = open(path, O_WRONLY);
        if (fd == -1)
        {
            perror("gpio_set_direction: Failed to open direction for writing");
            return -1;
        }

        if (write(fd, "in", strlen("in")) == -1)
        {
            perror("gpio_set_direction: Failed to write direction");
            close(fd);
            return -1;
        }
        direction_check(pin_num);
        close(fd);
        return 0;
    }
}
void GPIO_SET()
{
    for (int i = 0; i < 6; i++)
    {
        printf("Controlling Output pins\n");
        gpio_configure_out(RF_STATUS); // RF ON /OFF status
        set_gpio(RF_STATUS, ON);
        printf("RF HIGH(ON) \n");
        sleep(1);
        gpio_configure_out(RF_STATUS);
        set_gpio(RF_STATUS, OFF);
        printf("RF LOW(OFF) \n");
        sleep(1);
        gpio_configure_out(OVERHEAT); // OVERHEAT
        set_gpio(OVERHEAT, ON);
        printf("OVERHEAT HIGH \n");
        sleep(1);
        gpio_configure_out(OVERHEAT);
        set_gpio(OVERHEAT, OFF);
        printf("OVERHEAT LOW \n");
        sleep(1);
        gpio_configure_out(MAX_POWER); // MAX POWER
        set_gpio(MAX_POWER, ON);
        printf("MAX POWER HIGH \n");
        sleep(1);
        gpio_configure_out(MAX_POWER);
        set_gpio(MAX_POWER, OFF);
        printf("MAX POWER LOW \n");
        sleep(1);
        gpio_configure_out(DL1_BLUE); // blue led
        set_gpio(DL1_BLUE, LOW);
        printf("LED BLUE ON FOR DL1\n");
        sleep(1);
        set_gpio(DL1_BLUE, HIGH);
        printf("LED BLUE OFF FOR DL1\n");
        sleep(1);

        gpio_configure_out(DL1_GREEN); // GREEN led
        set_gpio(DL1_GREEN, LOW);
        printf("LED GREEN ON FOR DL1\n");
        sleep(1);
        set_gpio(DL1_GREEN, HIGH);
        printf("LED  GREEN OFF FOR DL1\n");
        sleep(1);

        gpio_configure_out(DL1_RED); // RED led
        set_gpio(DL1_RED, LOW);
        printf("LED RED ON FOR DL1\n");
        sleep(1);
        set_gpio(DL1_RED, HIGH);
        printf("LED RED OFF FOR DL1\n");
        sleep(1);

        gpio_configure_out(DL2_BLUE_1); // BLUE led
        set_gpio(DL2_BLUE_1, LOW);
        printf("LED BLUE ON FOR DL2\n");
        sleep(1);
        set_gpio(DL2_BLUE_1, HIGH);
        printf("LED BLUE OFF FOR DL2\n");
        // sleep(1);

        gpio_configure_out(DL2_BLUE_2); // BLUE led proto2
        set_gpio(DL2_BLUE_2, LOW);
        printf("LED BLUE ON FOR DL2\n");
        sleep(1);
        set_gpio(DL2_BLUE_2, HIGH);
        printf("LED BLUE OFF FOR DL2\n");
        sleep(1);

        gpio_configure_out(DL2_GREEN); // GREEN led
        set_gpio(DL2_GREEN, LOW);
        printf("LED GREEN ON FOR DL2\n");
        sleep(1);
        set_gpio(DL2_GREEN, HIGH);
        printf("LED GREEN OFF FOR DL2\n");
        sleep(1);

        gpio_configure_out(DL2_RED); // RED led
        set_gpio(DL2_RED, LOW);
        printf("LED RED ON FOR DL2\n");
        sleep(1);
        set_gpio(DL2_RED, HIGH);
        printf("LED RED OFF FOR DL2\n");
        sleep(1);

        printf("Value across the input pins\n");
        int state_dummy = gpio_configure_in(INTERLOCK_GP); // interlock
        printf("In gpio 117(interlock): %d\n", state_dummy);
        int state = gpio_configure_in(RF_CONTROL); // RF on off control
        printf("In gpio 118(RF on/off control): %d\n", state);
        int user_switch3_sw3 = gpio_configure_in(44);
        printf("In gpio 44(SW3): %d\n", user_switch3_sw3);
        int user_switch1_sw4 = gpio_configure_in(30);
        printf("In gpio 30(SW4): %d\n", user_switch1_sw4);
        int user_switch2_sw5 = gpio_configure_in(31);
        printf("In gpio 31(SW5): %d\n", user_switch2_sw5);
        printf("\n");
    }
}

int amp_set_reset()
{
    set_gpio(124, 1);
    usleep(10);
    set_gpio(122, 1);
    usleep(10);
    set_gpio(122, 0);
    usleep(10);
    set_gpio(124, 0);
}

int system_ready()
{
    set_gpio(DL1_GREEN, LOW);
    usleep(10);
    set_gpio(DL2_GREEN, LOW);
    usleep(10);
    set_gpio(DL1_BLUE, HIGH);
    usleep(10);
    set_gpio(DL2_BLUE_2, HIGH);
    usleep(10);
    set_gpio(DL2_BLUE_1, HIGH);
    usleep(10);
    set_gpio(DL2_RED, HIGH);
    usleep(10);
    set_gpio(DL1_RED, HIGH);
    usleep(10);
}
int rf_within_regulation()
{
    led_toggle_count++;
    set_gpio(DL1_BLUE, HIGH);
    set_gpio(DL2_BLUE_2, HIGH);
    set_gpio(DL2_BLUE_1, HIGH);
    set_gpio(DL2_RED, HIGH);
    set_gpio(DL1_RED, HIGH);
    if (led_toggle_count < 5)
    {
        set_gpio(DL1_GREEN, LOW);
        set_gpio(DL2_GREEN, LOW);
    }
    else
    {
        set_gpio(DL1_GREEN, HIGH);
        set_gpio(DL2_GREEN, HIGH);
        if (led_toggle_count > 10)
        {
            led_toggle_count = 0;
        }
    }
}

int rf_on_led()
{
    led_toggle_count++;
    set_gpio(DL1_BLUE, HIGH);
    set_gpio(DL2_BLUE_2, HIGH);
    set_gpio(DL2_BLUE_1, HIGH);
    set_gpio(DL2_RED, HIGH);
    set_gpio(DL1_RED, HIGH);
    set_gpio(DL2_GREEN, HIGH);
    if (led_toggle_count < 5)
    {
        set_gpio(DL1_GREEN, LOW);
    }
    else
    {
        set_gpio(DL1_GREEN, HIGH);
        if (led_toggle_count > 10)
        {
            led_toggle_count = 0;
        }
    }
}

int interlock_fault()
{
    set_gpio(DL1_BLUE, LOW);
    set_gpio(DL1_RED, HIGH);
    set_gpio(DL1_GREEN, HIGH);
    set_gpio(DL2_GREEN, HIGH);
    set_gpio(DL2_BLUE_2, HIGH);
    set_gpio(DL2_BLUE_1, HIGH);
    set_gpio(DL2_RED, HIGH);
}

int warning()
{
    led_toggle_count++;
    set_gpio(DL1_BLUE, HIGH);
    set_gpio(DL1_RED, HIGH);
    set_gpio(DL1_GREEN, HIGH);
    set_gpio(DL2_GREEN, HIGH);
    set_gpio(DL2_RED, HIGH);
    if (led_toggle_count < 5)
    {
        set_gpio(DL2_BLUE_2, LOW);
        set_gpio(DL2_BLUE_1, LOW);
    }
    else{
        set_gpio(DL2_BLUE_2, HIGH);
        set_gpio(DL2_BLUE_1, HIGH);
        if (led_toggle_count > 10)
        {
            led_toggle_count = 0;
        }
    }
    
}
int fault()
{
    led_toggle_count++;
    set_gpio(DL1_BLUE, HIGH);
    set_gpio(DL1_RED, LOW);
    set_gpio(DL1_GREEN, HIGH);
    set_gpio(DL2_GREEN, HIGH);
    set_gpio(DL2_BLUE_2, HIGH);
    set_gpio(DL2_BLUE_1, HIGH);
    if (led_toggle_count < 5)
    {
        set_gpio(DL2_RED, LOW);
    }
    else{
        set_gpio(DL2_RED, HIGH);
        if (led_toggle_count > 10)
        {
            led_toggle_count = 0;
        }
    }
}
int gpio()
{
    gpio_configure_out(66); // pps control gpio
    set_gpio(66, 0);
    gpio_configure_out(116); // rf on/off status update
    gpio_configure_out(113); // max reflected power
    gpio_configure_in(117);  // interlock
    gpio_configure_in(118);  // rf on off control
    gpio_configure_in(123);  // amp trip
    gpio_configure_out(115); // overheat
    gpio_configure_out(122); // amp  reset
    gpio_configure_out(124); // amp set
    gpio_configure_out(40);  // user led blue colour for DL1
    set_gpio(40, 1);
    gpio_configure_out(42); // user led green colour for DL1
    set_gpio(42, 1);
    gpio_configure_out(46); // user led red colour for DL1
    set_gpio(46, 1);
    gpio_configure_out(43); // user led blue colour for DL2  for proto 1
    set_gpio(43, 1);
    gpio_configure_out(41); // user led blue colour for DL2  for proto 2
    set_gpio(41, 1);
    gpio_configure_out(45); // user led green colour for DL2
    set_gpio(45, 1);
    gpio_configure_out(47); // user led red  colour for DL2
    set_gpio(47, 1);
    gpio_configure_in(44);  // user switch3 for switch 3
    gpio_configure_in(30);  // user switch1 for switch 4
    gpio_configure_in(31);  // user switch2 for switch 5
    gpio_configure_in(114); // analog remote enable
    gpio_configure_in(119); // leveling select
}