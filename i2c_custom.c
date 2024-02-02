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
#include "gpio.h"
#include "print_custom.h"
#include "spi_functions.h"
//Function definition for temperature sensor
uint8_t Temp_write_register(uint8_t file, unsigned char slave_addr, uint8_t high,uint8_t low)
{
    uint8_t outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    outbuf[0] = high;
    outbuf[1] = low;

    messages[0].addr = slave_addr;
    messages[0].flags = 0;
    messages[0].len = sizeof(outbuf);
    messages[0].buf = outbuf;

    packets.msgs  = messages;
    packets.nmsgs = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        return -1;
        printf("I2C write failed!!\n");
    }
    return 0;
}

int16_t Temp_read_register(uint8_t file, unsigned char slave_addr)
{
    int8_t output_value[6] ={0};
    uint8_t inbuf[6];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[6];

    messages[0].addr  = slave_addr;
    messages[0].flags = I2C_M_RD;
    messages[0].len   = sizeof(inbuf);
    messages[0].buf   = inbuf;
    packets.msgs      = messages;
    packets.nmsgs     = 1;

    uint8_t ret = ioctl(file, I2C_RDWR, &packets);
    if(ret < 0) {
        perror("Temp_read failed : ");
        return -1;
    }
    // for (int i =0;i<6;i++)
    // {
    //     printf("Data[%d]: %d\n",i,inbuf[i]);
    // }
    int over_heat = read_temperature(inbuf[0],inbuf[1]);
   // read_relative_humidity(inbuf[3],inbuf[4]);
    if (over_heat==1)
    {
        return 1;
    }
    if(over_heat==0)
    {
        return 0;
    }
    if(over_heat==2)
    {
        return 2;
    }
}
int read_temperature(unsigned char high, unsigned char low) {
    unsigned char i2c_data[2];
    i2c_data[0]=high;
    i2c_data[1]=low;
    uint16_t temperature_raw = (i2c_data[0] << 8) | (i2c_data[1]);
    float temperature = -45 + 175 * (temperature_raw)/(65536 - 1);
    printf("Temperature : %.2f °C\n", temperature);
    if (temperature >55)
    {
        if(temperature >60)
        {
            set_gpio(115, 1);
            printf("Over Heat Fault!!\n");
            return 1;
        }
        else
        {
            set_gpio(115, 0);
            warning();
            printf("Over Heat Warning!!!!\n");
            return 0;
        }
    }
    else
    {
        return 2;
    }
}

int read_relative_humidity(unsigned char high, unsigned char low) {
    unsigned char i2c_data[2];
    i2c_data[0] = high;
    i2c_data[1] = low;
    uint16_t rh_raw = (i2c_data[0] << 8) | i2c_data[1];
    float rh = 100 * rh_raw/(65536 - 1);
    printf("Humidity    : %.2f %%\n",rh);
   
}