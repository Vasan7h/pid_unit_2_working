#define _ADC_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI1SS1_ADC_MCU_DEVICE_SPEED 1000000 // 1MHz
#define EMB_SPI_DEVICE_SPEED 3000000          // 850Khz

// ADC Functions
int spi_init_setup(uint8_t spi_cs_num);
uint8_t choose_channel();
int spi_configuration(int file, int num);
uint16_t SPI1SS1_ADC_readRegister(int spi_fd, struct spi_ioc_transfer spi_trx);
uint16_t PG10_EMB_read_register(int file, uint8_t address, uint8_t channel, uint8_t print_flag, struct spi_ioc_transfer spi_trx);
int spi_dev3_fd, spi_dev3_fd;
struct spi_ioc_transfer spi3_trx;
struct spi_ioc_transfer spi4_trx;
double avg_value(int EMB_spi_fd,int flag);
double calibrated_result(uint16_t value,int flag);