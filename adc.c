#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "adc.h"

int spi_init_setup(uint8_t spi_cs_num)
{
    int fd;
    char command[64];
    snprintf(command, sizeof(command), "/dev/spidev2.%d", spi_cs_num);
    fd = open(command, O_RDWR);
    if (fd < 0)
    {
        switch (spi_cs_num)
        {
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
int spi_configuration(int spi_fd, int num)
{
    u_int32_t scratch32;

    /*Read SPI mode */
    if (ioctl(spi_fd, SPI_IOC_RD_MODE32, &scratch32) != 0)
    {
        perror("Could not read SPI mode...");
        close(spi_fd);
        return (-1);
    }
    /*Write SPI mode */
    scratch32 |= SPI_MODE_3;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE32, &scratch32) != 0)
    {
        perror("Could not write SPI mode...");
        close(spi_fd);
        return (-1);
    }

    switch (num)
    {
    case 1:
        scratch32 = EMB_SPI_DEVICE_SPEED;
        break;
    case 3:
        scratch32 = SPI1SS1_ADC_MCU_DEVICE_SPEED;
        break;
    }

    /*Read SPI freq*/
    if (ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &scratch32) != 0)
    {
        perror("Could not read the SPI max speed...");
        close(spi_fd);
        return (-1);
    }

    /*Write SPI freq*/
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32) != 0)
    {
        perror("Could not write the SPI max speed...");
        close(spi_fd);
        return (-1);
    }
    return 0;
}

uint16_t PG10_EMB_read_register(int spi_fd, uint8_t address, uint8_t channel, uint8_t print_flag, struct spi_ioc_transfer spi_trx)
{
    uint8_t txData[1];
    uint16_t rxData[1], temp, data;
    float calc_output = 0, reduced_calc_output = 0;
    // uint16_t rxData[1];
    txData[0] = address;

    memset(&spi_trx, 0, sizeof(spi_trx));

    spi_trx.tx_buf = (unsigned long)txData;
    spi_trx.rx_buf = (unsigned long)rxData;
    spi_trx.len = 2; // 2 bcalc_outputtes
    spi_trx.speed_hz = EMB_SPI_DEVICE_SPEED;
    spi_trx.bits_per_word = 8;
    spi_trx.delay_usecs = 0;

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_trx);
    if (ret < 0)
    {
        perror("SPI write failed");
        return -1;
    }
    // printf("\ttxData[0] : %x\n",txData[0]);
    if (print_flag)
    {
        temp = rxData[0];                                                                                                             
        rxData[0] &= 0xFF00;
        rxData[0] >>= 8;
        temp &= 0x00FF;
        temp <<= 8;
        temp |= rxData[0];
        rxData[0] = temp;
        if (channel == 1)
        {
            data = temp & 0x0FFF;
            // printf("Reflected power in channel-1: %d\n",data);
        }
        else if (channel == 2)
        {
            data = temp & 0x0FFF;
            // printf("Forward power in channel-2 : %d\n",data);
        }
    }
    return data;
}
//flag =1 ; forward power avg will happen 
//flag =0 ; reflected power avg will happen 
double avg_value(int EMB_spi_fd,int flag)
{
    double temp=0;
    double data;
    if (flag)
    {
        PG10_EMB_read_register(EMB_spi_fd, 0x08, 2, 1, spi4_trx);
        for (int i=0;i<10;i++)
        {
            uint16_t value = PG10_EMB_read_register(EMB_spi_fd, 0x08, 2, 1, spi4_trx); 
            // printf("Forward power in channel-2(RAW) : %d\n", value);      
            temp = temp+value;
        }   
        data = temp/10;
        // printf("Forward power in channel-2(RAW)(Avg) : %d\n", data);
        return data;
    }
    else
    {
        temp=0;
        PG10_EMB_read_register(EMB_spi_fd, 0x00, 1, 1, spi4_trx);
        for (int i=0;i<10;i++)
        {
            uint16_t value_1 = PG10_EMB_read_register(EMB_spi_fd, 0x00, 1, 1, spi4_trx);
            // printf("Reflected power in channel-1(RAW): %d\n", value_1);        
            temp = temp+value_1;
        }
        data = temp/10;
        // printf("Reflected power in channel-1(RAW)(AVG): %d\n", data);
        return data;
    }
}

double calibrated_result(uint16_t value,int flag)
{
   // value = value - 378;
    double data;
    //flag = 1 for forward power 
    //flag = 0 for reflected power
    if (flag)
    {
        if (value >=0 && value <=1162.5)
        {
            data =0;
            return data;
        }
        // 0 Watt to 3 Watt
        else if (value>1162.5&& value<=1517.5)
        {
            data = (value*0.015428511)-22.93564355;
            data = pow(10,data);
            return data;
        }
        //3 Watt to 100 Watt
        else if (value >1517.5 && value <=1904.5)
        {
            data =(value*0.003935087)-5.494373567;
            data = pow(10,data);
            return data;
        }
        //100 Watt to 200 Watt
        else if (value >1904.5 && value <=2039.5)
        {
            data =(value*0.002245897)-2.277310306;
            data = pow(10,data);
            return data;
        }
        //200 Watt to 300 Watt
        else if (value >2039.5 && value <=2105)
        {
            data =(value*0.002655347)-3.112383178;
            data = pow(10,data);
            return data;
        }
        //300 Watt to 400 Watt
        else if (value >2105 && value <=2157.5)
        {
            data =(value*0.002359079)-2.488739778;
            data = pow(10,data);
            return data;
        }
        //400 Watt to 500 Watt
        else if (value >2157.5 && value <=2191.5)
        {
            data =(value*0.002907789)-3.672582137;
            data = pow(10,data);
            return data;
        }
        //500 Watt to 600 Watt
        else if (value >2191.5 && value <=2216.5)
        {
            data =(value*0.003132541)-4.165125833;
            data = pow(10,data);
            return data;
        }
        //600 Watt to 750 Watt
        else if (value >2216.5 && value <=2232)
        {
            data =(value*0.006252259)-11.07998061;
            data = pow(10,data);
            return data;
        }
      
        //750 Watt to 1000 Watt
        else if (value >2232 && value <=2328)
        {
            data =(value*0.001301445)-0.029764363;
            data = pow(10,data);
            return data;
        }
        /*//80 Watt to 90 Watt
        else if (value >1841.5 && value <=1844)
        {
            data =(value*0.020461009)-35.77585805;
            data = pow(10,data);
            return data;
        }
        //90 Watt to 99 Watt
        else if (value >1844 && value <=1877.5)
        {
            data =(value*0.001235603)-0.324208578;
            data = pow(10,data);
            return data;
        }
        //99 Watt to 100 Watt
        else if (value >1877.5 && value <=1882.5)
        {
            data =(value*0.000872961)+0.356650766;
            data = pow(10,data);
            return data;
        }
        //100 Watt to 150 Watt
        else if (value >1882.5 && value <=1954)
        {
            data =(value*0.002462815)-2.636248884;
            data = pow(10,data);
            return data;
        }
        //150 to 201 Watt
        else if (value >1954 && value <=1985.5)
        {
            data =(value*0.004035073)-5.708441313; 
            data = pow(10,data);
            return data;
        }
        //201 to 300 watt
        else if (value >1985.5 && value <=2047)
        {
            data =(value*0.002828052)-3.311901166;
            data = pow(10,data);
            return data;
        }
        //300 to 399 watt
        else if (value >2047 && value <=2093)
        {
            data =(value*0.002692427)-3.034276768;
            data = pow(10,data);
            return data;
        }
        //399 to 501 watt
        else if (value >2093 && value <=2133.5)
        {
            data =(value*0.002441107)-2.508263884;
            data = pow(10,data);
            return data;
        }
        //501 to  550 watt
        else if (value >2133.5 && value <=2148)
        {
            data =(value*0.002794825)-3.262921577;
            data = pow(10,data);
            return data;
        }
        //550 to  600 watt
        else if (value >2148 && value <=2160.5)
        {
            data =(value*0.003023085)-3.753223614;
            data = pow(10,data);
            return data;
        }
        //600 to 620 watt
        else if (value >2160.5 && value <=2167)
        {
            data =(value*0.002190837)-1.955151628;
            data = pow(10,data);
            return data;
        }
        //620 to 630 watt
        else if (value >2160.5 && value <=2167)
        {
            data =(value*0.004632573)-7.246394659;
            data = pow(10,data);
            return data;
        }
        //630 to 640 watt
        else if (value >2160.5 && value <=2167)
        {
            data =(value*0.003419712)-4.616305498;
            data = pow(10,data);
            return data;
        }
        //640 to 670 watt
        else if (value >2160.5 && value <=2167)
        {
            data =(value*0.00663161)-11.5877286;
            data = pow(10,data);
            return data;
        }*/
        
    }
    else 
    {
        //offset for reflected power
        if (value >=0 && value <=918)
        {
            data =0;
            return data;
        }
        // 0 Watt to 1 Watt
        else if (value>918 && value<=1448.5)
        {
            data = (value*0.009425071)-13.65221489;
            data = pow(10,data);
            return data;
        }
        //1 Watt to 10 Watt
        else if (value >1448.5 && value <=1645)
        {
            data =(value*0.005089059)-7.371501272;
            data = pow(10,data);
            return data;
        }
        //10 Watt to 100 Watt
        else if (value >1645 && value <=1970)
        {
            data =(value*0.003076923)-4.061538462;
            data = pow(10,data);
            return data;
        }
        //100 Watt to 200 Watt
        else if (value >1970 && value <=2270)
        {
            data =(value*0.001003433)+0.023236362;
            data = pow(10,data);
            return data;
        }
        else
        {
            data =(value*0.002147454)-2.573691444;
            data = pow(10,data);
            return data;
        }
    }
}

// int PG10_EMB_read_register(int spi_fd, uint8_t address, uint8_t channel, uint8_t print_flag,struct spi_ioc_transfer xfer)
// {
//     uint8_t txData[1], rxData[2];
//     float calc_output = 0, reduced_calc_output = 0;
//     // uint16_t rxData[1];
//     txData[0] = address;
//     memset(&xfer, 0, sizeof(xfer));

//     xfer.tx_buf = (unsigned long)txData;
//     xfer.rx_buf = (unsigned long)rxData;
//     xfer.len = 2; //2 bcalc_outputtes
//     xfer.speed_hz = EMB_SPI_DEVICE_SPEED;
//     xfer.bits_per_word = 8;
//     xfer.delay_usecs = 0;

//     int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer);
//     if (ret < 0)
//     {
//         perror("SPI write failed");
//         return -1;
//     }
//     // printf("\ttxData[0] : %x\n",txData[0]);
//     if(print_flag)
//     {
//         //printf("Channel %d RAW DATA : rxData[0] : %x rxData[1] : %x\n", channel, rxData[0], rxData[1]);
//         rxData[0] <<= 4;
//         rxData[0] &= 0xF0;
//         rxData[1] >>= 4;
//         rxData[0] |= rxData[1];
//         calc_output = (rxData[0]*(5.08/256)*260) - 25.8;
// 	//if(channel==3)
//         	//printf("\tCalc_output : %0.5f\n", calc_output);
// 	//CALIBRATION LOGICS
//         if(channel == 3) {
//             //printf("Raw data : %f\n",calc_output);
//             if(calc_output>=15.475 && calc_output<=20.634375)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-15.475)*5.8139534883720930232558139534884);
//             else if(calc_output>=20.634375 && calc_output<=87.706253)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+46.446)*0.44843049327354260089686098654709);
//             else if(calc_output>=87.706253 && calc_output<=154.778122)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+46.454)*0.44722719141323792486583184257603);
//             else if(calc_output>=154.778122 && calc_output<=201.212494)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-15.458)*0.64599483204134366925064599483204);
//             else if(calc_output>=201.212494 && calc_output<=206.371872)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-180.572)*5.8139534883720930232558139534884);
//             else if(calc_output>=206.371872 && calc_output<=263.125)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+77.428)*0.52854122621564482029598308668076);
//             else if(calc_output>=263.125 && calc_output<=340.515625)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+201.275)*0.38759689922480620155038759689922);
//             else if(calc_output>=340.515625 && calc_output<=381.790619)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-51.556)*0.72674418604651162790697674418605);
//             else if(calc_output>=381.790619 && calc_output<=417.90625)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-92.831)*0.83056478405315614617940199335548);
//             else if(calc_output>=417.90625 && calc_output<=459.181244)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-46.386)*0.72674418604651162790697674418605);
//             else if(calc_output>=454.021881 && calc_output<=515.934387)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-118.614)*0.83056478405315614617940199335548);
//             else if(calc_output>=515.934387&& calc_output<=598.484375)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+5.23)*0.64599483204134366925064599483204);
//             else if(calc_output>=598.484375&& calc_output<=634.599976)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+105.856)*0.55370985603543743078626799557032);
//             else if(calc_output>=634.599976&& calc_output<=655.237488)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-423.04)*1.9379844961240310077519379844961);
//             else if(calc_output>=655.237488&& calc_output<=675.878)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-345.637)*1.4970059880239520958083832335329);
//             else if(calc_output>=675.878&& calc_output<=732.628113)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+232.282)*0.52854122621564482029598308668076);
//             else if(calc_output>=732.628113&& calc_output<=742.976899)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-556.678)*2.8985507246376811594202898550725);
//             else if(calc_output>=742.976899&& calc_output<=768.743774)
//                 printf("\tForward Power : %0.5f W\n", (calc_output-279.117)*1.1641443538998835855646100116414);
//             else if(calc_output>=768.743774&& calc_output<=810.018738)
//                 printf("\tForward Power : %0.5f W\n", (calc_output+745.712)*0.38759689922480620155038759689922);
//         }
//         if(channel == 4) {
//             printf("\tReflected Power : %0.5f W\n", ((rxData[0]*(5.08/256))-25.8) < 0 ? 0 : (rxData[0]*(5.08/256))-25.8);
//         }
//     }
//     return 0;
// }

uint8_t choose_channel()
{
    int channel_num = 0;
    printf("choose a channel (1-4) : ");
    scanf("%d", &channel_num);
    switch (channel_num)
    {
    case 1:
        return 0;
    case 2:
        return 0x08;
    case 3:
        return 0x10;
    case 4:
        return 0xff;
    default:
        printf("Invalid channel chosen !!!\n");
        return -1;
    }
}

uint16_t SPI1SS1_ADC_readRegister(int spi_fd, struct spi_ioc_transfer spi_trx)
{
    uint16_t output;
    uint16_t txData[1] = {0}, temp;
    uint16_t rxData[1];
    // printf("in read function\n");
    // struct spi_ioc_transfer spi_trx;
    memset(&spi_trx, 0, sizeof(spi_trx));

    spi_trx.tx_buf = (unsigned long)txData;
    spi_trx.rx_buf = (unsigned long)rxData;
    spi_trx.len = 2; // 2 bcalc_outputtes
    spi_trx.speed_hz = SPI1SS1_ADC_MCU_DEVICE_SPEED;
    spi_trx.bits_per_word = 0;
    spi_trx.delay_usecs = 0;

    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_trx);
    // if (ret != 0)
    // {
    //     printf("(SPI transfer returned %d..)\n", ret);
    // }
    // printf("rxData[0] : %x\n", rxData[0]);
    // printf("rxData[1] : %x\n", rxData[1]);
    temp = rxData[0];
    rxData[0] &= 0xFF00;
    rxData[0] >>= 8;
    temp &= 0x00FF;
    temp <<= 8;
    temp |= rxData[0];
    rxData[0] = temp;
    output = rxData[0] >> 5;
    output = output & 0x00FF;
    return output;
}