#ifndef _I2C_CUSTOM_H
#define _I2C_CUSTOM_H
#include <stdint.h>
#define SLAVE_ADDR 0x44
#define CLK_STRECH_EN_MSB 0x2C
#define CLK_STRECH_EN_LSB 0x06
int8_t Temp_i2c_readwrite_init();
uint8_t Temp_write_register(uint8_t file, unsigned char slave_addr, uint8_t high,uint8_t low);
int16_t Temp_read_register(uint8_t file, unsigned char slave_addr);
int temp_main(int flag);
int read_temperature(unsigned char high, unsigned char low);
int read_relative_humidity(unsigned char high, unsigned char low);

#endif