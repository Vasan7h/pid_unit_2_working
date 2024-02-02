#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>

#define SYSFS_GPIO_PATH "/sys/class/gpio"
#define SYSFS_GPIO_EXPORT_FN "/export"
#define SYSFS_GPIO_UNEXPORT_FN "/unexport"
#define SYSFS_GPIO_VALUE "/value"
#define SYSFS_GPIO_DIRECTION "/direction"
#define SYSFS_GPIO_EDGE "/edge"

#define DIR_IN "in"
#define DIR_OUT "out"

#define VALUE_HIGH "1"
#define VALUE_LOW "0"

#define EDGE_RISING "rising"
#define EDGE_FALLING "falling"

#define POLL_TIMEOUT 10 * 1000
#define ON 1
#define OFF 0
#define HIGH 1
#define LOW 0

#define  RF_STATUS 116
#define OVERHEAT 115
#define MAX_POWER 113
#define DL1_BLUE 40
#define DL1_RED 46
#define DL1_GREEN 42

#define DL2_BLUE_1 43
#define DL2_BLUE_2 41
#define DL2_RED 47
#define DL2_GREEN 45
#define INTERLOCK_GP 117
#define RF_CONTROL 118



uint8_t gpio_export(uint8_t pin_num);
uint8_t gpio_unexport(uint8_t pin_num);
int gpio_set_direction(int gpio_num, const char *dir);
int gpio_get_fd_to_value(int gpio_num);
int gpio_ctrl_in(int num, const char *dir);
uint8_t gpio_configure_out(uint8_t pin_num);
uint8_t set_gpio(uint8_t pin_num, uint8_t value);
int8_t init_gpio(uint8_t pin_num);
uint8_t gpio_is_exported(uint8_t pin_num);
void GPIO_SET();
int direction_check(int gpio_pin);
uint8_t gpio_configure_in(uint8_t pin_num);
int amp_set_reset();
int system_ready();
int rf_within_regulation();
int interlock_fault();
int warning();
int fault();
int gpio();
int rf_on_led();