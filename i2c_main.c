#include <fcntl.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include "i2c_custom.h"
#include "print_custom.h"
#define Temp_i2c_fd_str "/dev/i2c-0"
int8_t Temperature_i2c_fd;

int8_t Temp_i2c_readwrite_init()
{
    Temperature_i2c_fd = open(Temp_i2c_fd_str, O_RDWR);
    if(Temperature_i2c_fd < 0)
    {
        error_printf("Failed to open Temperature_i2c_fd");
        return -1;
    }
    //specifying the slave device (WRITE) with its address for communication
    if (ioctl(Temperature_i2c_fd, I2C_SLAVE, SLAVE_ADDR) < 0)
    {
        error_printf("Error while setting SlaveID(0x44)");
        return -1;
    }
    // close(Temperature_i2c_fd);
    return 0;
}
int temp_main(int flag)
{
    if (flag)
    {
        Temp_write_register(Temperature_i2c_fd,SLAVE_ADDR,CLK_STRECH_EN_MSB,CLK_STRECH_EN_LSB);
        usleep(1000);
        Temp_read_register(Temperature_i2c_fd,SLAVE_ADDR);
    }
    else
    {
        Temp_i2c_readwrite_init();
        while(1)
        {
        Temp_write_register(Temperature_i2c_fd,SLAVE_ADDR,CLK_STRECH_EN_MSB,CLK_STRECH_EN_LSB);
        usleep(1000);
        Temp_read_register(Temperature_i2c_fd,SLAVE_ADDR);
        sleep(1);
        printf("\n");
        }
    }
    
}