#ifndef _SPI_FUNCTIONS_H
#define _SPI_FUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_DEVICE1 "/dev/spidev2.2"
#define SPI_DEVICE2 "/dev/spidev2.0"

#define DAC_SPI_DEVICE_SPEED 1000000 // 1MHz

#define DAC_POWER_DOWN_RELS_CMD 0x09
#define DAC_POWER_DOWN_RELS_DATA 0x00
#define DAC_IO_DA_SEL_CMD 0x03
#define DAC_IO_DA_SEL_DATA 0xFF
#define DAC_IO_STATUS_SEL_CMD 0x0F
#define DAC_IO_STATUS_SEL_DATA 0xFF

#define DAC_DA1_SEL_CMD 0x08
#define DAC_DA2_SEL_CMD 0x04
#define DAC_DA3_SEL_CMD 0x0C
#define DAC_DA4_SEL_CMD 0x02
#define DAC_DA5_SEL_CMD 0x0A
#define DAC_DA6_SEL_CMD 0x06
#define DAC_DA7_SEL_CMD 0x0E
#define DAC_DA8_SEL_CMD 0x01

#define FIFTY_PERCENT 0.5
#define TEN_PERCENT 0.1
#define TWENTY_PERCENT 0.2
#define EIGHTY_PERCENT 0.8
#define FOR_POW 1
#define REF_POW 0



struct spi_ioc_transfer spi1_trx;
struct spi_ioc_transfer spi2_trx;

enum SPI_DEVICES
{
    DAC_DEVICE,
    EMB_DEVICE,
    DAC_FORW_REF_POW,
    SPI1SS1_ADC_MCU,
};

enum SPI_DEVICES device_id;

// Common functions
int spi_initial_setup(u_int8_t spi_cs_num);

// DAC Functions
int DAC_write(int fd, u_int8_t *tx_data, struct spi_ioc_transfer spi_trx);
int DAC_powerDownRelease(int DAC_spi_fd, struct spi_ioc_transfer spi_trx);
int DAC_iodaSelect(int DAC_spi_fd, struct spi_ioc_transfer spi_trx);
int DAC_ioStatusSelect(int DAC_spi_fd, struct spi_ioc_transfer spi_trx);
int DAC_begin(int DAC_spi_fd, struct spi_ioc_transfer spi_trx);
int Volt_cur_forw_ref_load_Set(int DAC_spi_fd, u_int8_t buf_1, struct spi_ioc_transfer spi_trx, u_int8_t buf_2);
int spi_setup(int fd);
int spi_main();
uint8_t pps_dac_setting(int data, int DAC_spi_fd);
int dac_set(int DAC_forw_ref_pow_spi_fd, double forward, double reflected, double load);
void power_set_point(void);
void temp_functioning(void);
void rf_on_off_interlock_functioning(void);
void plasma_init(void);
void Read_forward_power(void);
void Read_reflected_power(void);
void ref_power_condition_check(void);
void pid_overflow_fix(void);

#endif