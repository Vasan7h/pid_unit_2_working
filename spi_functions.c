#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include "spi_functions.h"

int spi_initial_setup(uint8_t spi_cs_num)
{
	int fd;
	char command[64];
	snprintf(command, sizeof(command), "/dev/spidev2.%d", spi_cs_num);
	fd = open(command, O_RDWR);
	if (fd < 0)
	{
		switch (spi_cs_num)
		{
		case 2:
			perror("Error in opening spi for DAC : ");
			return -1;
		case 0:
			perror("Error in opening spi for pps control dac : ");
			return -1;
		case 3:
			perror("Error in opening spi for EMB : ");
			return -1;
		case 1:
			perror("Error in opening SPI1SS1_ADC_MCU: ");
			return -1;
		default:
			perror("Error in opening spi for unknown device : ");
			return -1;
		}
	}
	return fd;
}

int spi_setup(int fd)
{
	int ret;
	uint32_t scratch32;

	/*Read SPI mode */
	ret = ioctl(fd, SPI_IOC_RD_MODE32, &scratch32);
	if (ret != 0)
	{
		printf("Could not read SPI mode...\r\n");
		close(fd);
		return (-1);
	}
	/*Write SPI mode */
	scratch32 |= SPI_MODE_3;
	ret = ioctl(fd, SPI_IOC_WR_MODE32, &scratch32);
	if (ret != 0)
	{
		printf("Could not write SPI mode...\r\n");
		close(fd);
		return (-1);
	}
	/*Read SPI freq*/
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
	if (ret != 0)
	{
		printf("Could not read the SPI max speed...\r\n");
		close(fd);
		return (-1);
	}
	/*Write SPI freq*/
	scratch32 = DAC_SPI_DEVICE_SPEED;
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
	if (ret != 0)
	{
		printf("Could not write the SPI max speed...\r\n");
		close(fd);
		return (-1);
	}
	return 0;
}

int DAC_powerDownRelease(int DAC_spi_fd, struct spi_ioc_transfer spi_trx)
{
	uint8_t tx_data[2];
	tx_data[0] = DAC_POWER_DOWN_RELS_CMD;
	tx_data[1] = DAC_POWER_DOWN_RELS_DATA;
	printf("powerDown Release ..\n");
	return DAC_write(DAC_spi_fd, tx_data, spi_trx);
}

int DAC_iodaSelect(int DAC_spi_fd, struct spi_ioc_transfer spi_trx)
{
	uint8_t tx_data[2];
	tx_data[0] = DAC_IO_DA_SEL_CMD;
	tx_data[1] = DAC_IO_DA_SEL_DATA;
	printf("I/O : D/A select ..\n");
	return DAC_write(DAC_spi_fd, tx_data, spi_trx);
}

int DAC_ioStatusSelect(int DAC_spi_fd, struct spi_ioc_transfer spi_trx)
{
	uint8_t tx_data[2];
	tx_data[0] = DAC_IO_STATUS_SEL_CMD;
	tx_data[1] = DAC_IO_STATUS_SEL_DATA;
	printf("I/O : OP select ..\n");
	return DAC_write(DAC_spi_fd, tx_data, spi_trx);
}

int DAC_begin(int DAC_spi_fd, struct spi_ioc_transfer spi_trx)
{
	// int ret =DAC_powerDownSetting(DAC_spi_fd);
	// sleep(0.5);
	int ret = DAC_powerDownRelease(DAC_spi_fd, spi_trx);
	sleep(0.5);
	ret = DAC_iodaSelect(DAC_spi_fd, spi_trx);
	sleep(0.5);
	ret = DAC_ioStatusSelect(DAC_spi_fd, spi_trx);
	sleep(0.5);
	return ret;
}

int DAC_write(int fd, uint8_t *tx_data, struct spi_ioc_transfer spi_trx)
{
	uint8_t tx_buffer[4];
	uint8_t rx_buffer[4];
	uint8_t command, data;
	int ret;
	tx_buffer[0] = tx_data[0];
	tx_buffer[1] = tx_data[1];
	// printf("\t Cur-Seq Command :  %d\n", tx_buffer[0]);
	// printf("\t Cur-Seq Data :  %d\n", tx_buffer[1]);

	spi_trx.tx_buf = (unsigned long)tx_buffer;
	spi_trx.rx_buf = (unsigned long)rx_buffer;
	spi_trx.bits_per_word = 0;
	spi_trx.speed_hz = DAC_SPI_DEVICE_SPEED;	
	spi_trx.delay_usecs = 0;
	spi_trx.len = 2;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &spi_trx);
	// if (ret != 0)
	// {
	// 	printf("(SPI transfer returned %d..)\n", ret);
	// }

	// command = ((rx_buffer[0] & 0xf0) >> 4);
	// data = ((rx_buffer[1] & 0xf0) >> 4);
	// data = ((rx_buffer[0] & 0x0f) << 4) | data;
	// printf(" Prev-Seq Command :  %d\n",command);
	// printf("\t Prev-Seq Data :  %d\r\n",data);
	return 0;
}

int Volt_cur_forw_ref_load_Set(int DAC_spi_fd, uint8_t buf_1, struct spi_ioc_transfer spi_trx, uint8_t buf_2)
{
	uint8_t tx_data[2] = {0};
	tx_data[0] = buf_1;
	tx_data[1] = buf_2;
	DAC_write(DAC_spi_fd, tx_data, spi_trx);
}
int dac_set(int DAC_forw_ref_pow_spi_fd, double forward, double reflected, double load)
{
	uint8_t reflected_update,load_update,forward_update;
	if (forward>600)
	{
		Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA2_SEL_CMD, spi2_trx, 255);
		reflected_update = ((reflected*1.27388535)+1.42109E-14);
		load_update = ((load*0.424998587)-2.13163E-13);
		if (load>600)
		{
			Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA3_SEL_CMD, spi2_trx, 255);
			usleep(3);
		}
		else
		{
			Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA3_SEL_CMD, spi2_trx, load_update);
    		usleep(3);
		}
		if(reflected>200)
		{
			Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA1_SEL_CMD, spi2_trx, 255);
   			usleep(3);
		}
		else
		{
			Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA1_SEL_CMD, spi2_trx, reflected_update);
			usleep(3);
		}
	}
	else
	{
	// printf("forward: %lf\n",forward);
		forward_update =(uint8_t) ((forward*0.424998587)-2.13163E-13);
		// printf("forward update : %d\n",forward_update);
		reflected_update = ((reflected*1.27388535)+1.42109E-14);
		load_update = ((load*0.424998587)-2.13163E-13);
		Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA2_SEL_CMD, spi2_trx, forward_update);
		usleep(3);
		Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA1_SEL_CMD, spi2_trx, reflected_update);
		usleep(3);
		
		Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA3_SEL_CMD, spi2_trx, load_update);
		usleep(3);
	}
	// Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA1_SEL_CMD, spi2_trx, 0);
    // usleep(3);
    // Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA2_SEL_CMD, spi2_trx, 1);
    // usleep(3);
    // Volt_cur_forw_ref_load_Set(DAC_forw_ref_pow_spi_fd, DAC_DA3_SEL_CMD, spi2_trx, 100);
    // usleep(3);
}

uint8_t pps_dac_setting(int data, int DAC_spi_fd)
{
	Volt_cur_forw_ref_load_Set(DAC_spi_fd, DAC_DA2_SEL_CMD, spi1_trx, 40);
	return 0;
}
