#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include "spi_functions.h"

#include "gpio.h"
#include "adc.h"
#include "uart.h"
#include "i2c_custom.h"
#include "analog.h"
// initial overshoot applied
// osicllation fixed
extern int8_t Temperature_i2c_fd;
int fault_flag = 0;
int prev_pow_set_pt;
int stable_dac;
int sol_flag; //for fixing imm mismatch 3 solutions in hand  
double tol = 0;
int stable_loop_init_flag = 0;
int err_sign;       // calculates err is +ve or -ve
int overshoot_flag; // to acheive shoot initially
int interlock_flag = 0, ref_fault_flag = 0, overheat_warn_flag = 0;
double forward_power = 0;
double prev_forward_power = 0;
float forward_power_temp; // to store previous stabilized adc val from meter in stable loop
long double forw = 0;
// int count = 1;                            // counter for 6 counts in PID loop for dac update
int8_t toler_set; // offset to set tolerance
// int count_stable;                         // counter for 4 counts in stable loop to set adc_nc_flag
int offset_flag = 0;                      // flag to set variable pid constants for first time
int adc_nc_flag;                          // flag to check 4 counts in stable loop
double osci_fix = 0;                      // for stabilizing the pow_set_ptput
int pow_set_pt;                           // set point (RF input)
double final_forward_power_pps_dac_value; // forw power within PID control
double final_forward_power;               // forw power inside PID when stable o/p
double Prev_final_forward_power = 0;      // to store previous forward power
double t_pps_dac_value = 0;               // 10% of max DAC value (actual 255) , limited for 600W
double I_pe = 0;                          // previous error_cal
double D_pe = 0;                          // previous error_cal
uint8_t prev_pps_dac_value = 0;           // previous DAC value
double Integration_cal = 0;               // integrator (accumulation of error_cal)
double Differentiation_cal = 0;           // derivative (change in error_cal)
double kp = 0;                            // proportional gain 0.1 %
double ki = 0.001;                        // integral gain 0.1 %
double kd = 0.001;                        // differential gain 0.1 %
float offset;                             // pps_dac_value conversion offset
double P_control;                         // proportional term
double I_control;                         // integral term
double D_control;                         // differential term
double error_cal;                         // error_cal
int power_pps_dac_value;
float a_dc;
int16_t pps_dac_value_PID;
uint8_t pps_dac_value; // DAC value
int pps_dac_value_f;
char set_flag_rev = 0, set_flag_for = 0;
uint8_t matched_condition = 0;
int16_t over_heat_detect = 0;
int SPI1SS1_ADC_MCU_fd = 0;
unsigned char pow_set_ptput;
uint8_t address = 0, tx_data[2] = {0};
int DAC_spi_fd = 0;
int EMB_spi_fd = 0;
int DAC_forw_ref_pow_spi_fd = 0;
int interlock = 0;
int rf_control = 0;
uint16_t data = 0;
double final_reflected_power = 0;
double reflected_power = 0;
int8_t condition_check = 0;
double load_power = 0;

int spi_main()
{
    plasma_init();
    while (1)
    {
        temp_functioning();
        rf_on_off_interlock_functioning();
        ref_power_condition_check();
        power_set_point();
        if (fault_flag | interlock_flag | (!pow_set_pt))
        {
            printf("Setting 0 Watt\n");
            // analog_main();
            pps_dac_value = 50;
            Volt_cur_forw_ref_load_Set(DAC_spi_fd, DAC_DA2_SEL_CMD, spi1_trx, pps_dac_value); // programmable power supply dac set point
            usleep(1000);                                                                     // 1ms delay
            // reading forward and reflected power
            Read_forward_power();
            Read_reflected_power();
            // calculating load power according to forward and reflected power
            load_power = final_forward_power - final_reflected_power;
            // DB25 power update
            dac_set(DAC_forw_ref_pow_spi_fd, final_forward_power, final_reflected_power, load_power);
        }
        else
        {
            prev_pps_dac_value = 0;
            offset_flag = 1;
            overshoot_flag = 1;
            error_cal = 0;
            I_pe = error_cal;
            D_pe = error_cal;
            printf("I AM ENTERING FOR FIRST TIME\n");
            while ((((!interlock_flag) && (!fault_flag) && (pow_set_pt > 0))))
            {
                if ((!overheat_warn_flag))
                {
                    rf_within_regulation(); // LED
                }
                power_set_point();
                printf("Setting %d Watt\n", pow_set_pt);
                printf("forw pow in check:    %lf W \n", final_forward_power);
                error_cal = (pow_set_pt - final_forward_power);
                printf("Err: %lf\n", error_cal);

                printf("stable loopinit flag %d\n", stable_loop_init_flag);
                // if (stable_loop_init_flag)
                // {
                //     prev_pps_dac_value = 0;
                // }
                // if(pow_set_pt !=prev_pow_set_pt)
                // {
                //     overshoot_flag=1;
                // }
                if (((fabs(error_cal)) > (0.02 * pow_set_pt)) && (!ref_fault_flag))
                {
                    printf("abs_error %lf\n", (fabs(error_cal)));
                    if (overshoot_flag)
                    {
                        pps_dac_value = (0.24 * (pow_set_pt + 100)) + 85; // dac for overshoot initially
                        printf("overshoot\n");
                        overshoot_flag=0;
                    }
                    else if(sol_flag)
                    {
                        pps_dac_value=60;//idea on 3.15pm
                        
                        printf("fixing imp mismatch\n");
                        sol_flag=0;
                        //overshoot_flag=1;// decision on 3.08pm
                    }
                    else
                    {
                        if (offset_flag)
                        {
                            prev_pps_dac_value = 0; // prev dac = 0 while entering pid loop for first time
                        }
                        // to choose max dac val pow_set_pt of 255
                        if ((fabs(error_cal)) > (FIFTY_PERCENT * pow_set_pt)) // err > 50% of set_point
                        {
                            t_pps_dac_value = (TWENTY_PERCENT * 240); // 20% of max dac val
                            offset_flag = 1;
                        }
                        else
                        {
                            t_pps_dac_value = (TEN_PERCENT * 240); // 10% of max dac val
                        }
                        if ((fabs(error_cal)) > (TWENTY_PERCENT * pow_set_pt)) // err > 20% set value
                        {
                            kp = 0.0065; // 0.65% gain change
                        }
                        else
                        {
                            kp = 0.002;
                        }
                        if (offset_flag)
                        {
                            if ((fabs(error_cal)) > (EIGHTY_PERCENT * pow_set_pt))
                            {
                                if ((pow_set_pt > 350) && (pow_set_pt < 490))
                                {
                                    ki = 0.0024789;
                                    kd = 0.0025;
                                }
                                else if ((pow_set_pt >= 220) && (pow_set_pt < 350))
                                {
                                    ki = 0.007;
                                    kd = 0.0013;
                                }
                                else if ((pow_set_pt >= 120) && (pow_set_pt < 220))
                                {
                                    ki = 0.008;
                                    kd = 0.0035;
                                }
                                else if (pow_set_pt < 120)
                                {
                                    kp = 0.009;
                                    ki = 0.0031;
                                    kd = 0.005;
                                }
                                else
                                {
                                    ki = 0.002;
                                    kd = 0.00125;
                                }
                            }
                            else
                            {
                                ki = 0.001;
                                kd = 0.0013;
                            }
                        }
                        else
                        {
                            kd = 0.002;
                            ki = 0.002;
                        }
                        P_control = kp * t_pps_dac_value * error_cal; // 0.1 % gain
                        // printf("kp : %lf\n", P_control);
                        Integration_cal = (error_cal + I_pe) / 2; // integral value
                        // printf("integral : %lf",Integration_cal);
                        I_control = ki * Integration_cal * t_pps_dac_value; // 0.1 % gain
                        // printf("ki : %lf\n", I_control);
                        Differentiation_cal = (error_cal - D_pe);
                        // printf("derivative : %lf",Differentiation_cal);     // derivative
                        D_control = kd * Differentiation_cal * t_pps_dac_value; // 0.1 % gain
                        // printf("kd: %lf\n", D_control);
                        I_pe = error_cal;                                                 // storing previous error
                        D_pe = error_cal;                                                 // storing previous error
                        pps_dac_value_PID = (int16_t)(P_control + I_control + D_control); // casting double to int
                        printf(" PPS PID value: %d\n", pps_dac_value_PID);
                        err_sign = ((((int32_t)(fabs(error_cal))) / ((int32_t)error_cal)));
                        if ((fabs(error_cal) > (0.03 * pow_set_pt)) && (!offset_flag))
                        {
                            toler_set = 2; // changed from 1 to 2
                            condition_check = (toler_set)*err_sign;
                        }
                        else
                        {
                            toler_set = 1; // changed from 1 to 2
                            condition_check = (toler_set)*err_sign;
                        }
                        // adding offset to acheive desired power
                        pps_dac_value_PID = pps_dac_value_PID + condition_check + prev_pps_dac_value;
                        pid_overflow_fix();
                        // in order to create an overshoot
                    }
                    printf("dac val after const %d\n", pps_dac_value);
                    // updating dac for pps
                    Volt_cur_forw_ref_load_Set(DAC_spi_fd, DAC_DA2_SEL_CMD, spi1_trx, pps_dac_value);
                    prev_pps_dac_value = pps_dac_value;
                    //  reading for_pow
                    usleep(20000);                     // 20 ms delay
                    usleep(round(pow_set_pt * 33.33)); // Delay calculation as per setpoint
                    Read_forward_power();
                    Read_reflected_power();
                    offset_flag = 0;
                    
                    power_set_point();
                    error_cal = (pow_set_pt - final_forward_power);

                    // int abserr=(int)(abs(error_cal));
                    // printf("abserr %lf\n",abserr);
                    if (error_cal < 0)
                    {
                        error_cal = -1 * error_cal;
                    }
                    printf("Err: %lf\n", error_cal);
                    if ((pps_dac_value>=245)&&(((int)final_forward_power)<600))
                    {
                        //printf("abs_error %d\n", (fabs(error_cal)));
                        // stable_loop_init_flag = 1;
                        sol_flag=1;
                    }
                    // prev_pow_set_pt=pow_set_pt;
                }
                else
                {
                    // forw dac update

                    // analog_main();                      // reading voltage from pps
                    //  forw_pow_calculation

                    error_cal = (pow_set_pt - final_forward_power);
                    printf("Err in stable PID: %lf\n", error_cal);
                    osci_fix = ((((int32_t)(fabs(error_cal))) / ((int32_t)error_cal))); // err sign
                    tol = (0.00275 * pow_set_pt) + 1.15;                               // 200 w +/-1.8 w to 600w +/- 2.8W
                    if (pow_set_pt <= 100)
                    {
                        if ((fabs(error_cal)> 2) && (!ref_fault_flag))
                        { // err > 2 W
                            printf("offset   %lf\n", osci_fix);
                            pps_dac_value = pps_dac_value + osci_fix; // adding offset to acheive desired o/p
                                                                      // prev_pps_dac_value = pps_dac_value; // storing dac val
                        }
                    }
                    else
                    {
                        if (((fabs(error_cal)) >= tol) && (!ref_fault_flag))
                        {
                            printf("offset   %lf\n", osci_fix);
                            pps_dac_value = pps_dac_value + osci_fix; // adding offset to acheive desired o/p
                            // prev_pps_dac_value = pps_dac_value; // storing dac val
                        }
                    }
                    Volt_cur_forw_ref_load_Set(DAC_spi_fd, DAC_DA2_SEL_CMD, spi1_trx, pps_dac_value);
                    printf("dac val %d \n", pps_dac_value);
                    printf("prev dac %d\n", prev_pps_dac_value);
                    usleep(10000); // 10ms delay
                    // reading forward and reflected power
                    Read_forward_power();
                    Read_reflected_power();
                    // calculating load power

                    if (prev_pps_dac_value == pps_dac_value)
                    {
                        final_forward_power = (Prev_final_forward_power + final_forward_power) / 2;
                        printf("filtering noise\n");
                    }
                    printf("forw pow after noise fixing:    %lf W \n", final_forward_power);
                    load_power = final_forward_power - final_reflected_power;
                    // DB25 updation

                    dac_set(DAC_forw_ref_pow_spi_fd, final_forward_power, final_reflected_power, load_power);
                    Prev_final_forward_power = final_forward_power;
                    printf("forw pow in after dac:    %lf W \n", final_forward_power);
                    prev_pps_dac_value = pps_dac_value; // storing dac val
                    // stabilizing for +/- 3 watts
                    // Read_forward_power();
                    // printf("forw pow in stable:    %lf W \n", final_forward_power);

                    stable_loop_init_flag = 0;
                    // printf("forw pow in stable:    %lf W \n", final_forward_power);
                   
                }
                printf("\n");
                rf_on_off_interlock_functioning();
                temp_functioning();
                ref_power_condition_check();
            }
        }
        printf("\n");
    }
    return 0;
}

void Read_forward_power(void)
{
    forward_power = avg_value(EMB_spi_fd, FOR_POW);
    printf("raw forw %lf\n", forward_power);
    final_forward_power = calibrated_result(forward_power, FOR_POW);
    printf("forw pow :    %lf W \n", final_forward_power);
}

void Read_reflected_power(void)
{
    reflected_power = avg_value(EMB_spi_fd, REF_POW);
    printf("raw ref %lf\n", reflected_power);
    final_reflected_power = calibrated_result(reflected_power, REF_POW);
    printf("Ref pow:     %lf W\n", final_reflected_power);
    printf("\n");
}

void ref_power_condition_check(void)
{
    if (final_reflected_power > 200) // checking for max reflected power
    {
        // in pid overshoot may occur in that condition reflection detects means pid will get exit
        // so in this case we are not supposed to exit
        if (final_forward_power < pow_set_pt)
        {
            printf("Max Power Detected!!!!\n");
            ref_fault_flag = 1;
            set_gpio(MAX_POWER, ON);
        }
        else
        {
            ref_fault_flag = 0;
        }
        // fault();
    }
    else
    {
        set_gpio(MAX_POWER, OFF);
        ref_fault_flag = 0;
    }
}

void pid_overflow_fix(void)
{
    if (pps_dac_value_PID > 255)
    {
        pps_dac_value_PID = 255;
    }
    pps_dac_value = (uint8_t)pps_dac_value_PID;
}

void power_set_point(void)
{
    data = SPI1SS1_ADC_readRegister(SPI1SS1_ADC_MCU_fd, spi3_trx);
    printf("raw_adc %d\n", data);
    a_dc = (float)data * 0.05;
    printf("adc_manipulated %lf\n", a_dc);
    if (a_dc > 7)
    {
        a_dc = a_dc - 0.05;
    }
    if (pow_set_pt > 298)
    {
        pow_set_pt = (a_dc * 60) + 1;
    }
    // printf("set_pt %d\n",pow_set_pt);
    //  pow_set_pt=pow_set_pt;
    if (pow_set_pt >= 600) // limited for 600 W
    {
        pow_set_pt = 600;
    }
    else
    {
        pow_set_pt = (a_dc * 60);
    }
}

void temp_functioning(void)
{
    Temp_write_register(Temperature_i2c_fd, SLAVE_ADDR, CLK_STRECH_EN_MSB, CLK_STRECH_EN_LSB);
    over_heat_detect = Temp_read_register(Temperature_i2c_fd, SLAVE_ADDR);
    // printf("Overheat: %d\n",over_heat_detect);
    if (over_heat_detect == 1)
    {
        fault_flag = 1;
        fault();
    }
    else
    {
        fault_flag = 0;
        if (!over_heat_detect)
        {
            overheat_warn_flag = 1;
        }
        else
        {
            overheat_warn_flag = 0;
        }
    }
}

void rf_on_off_interlock_functioning(void)
{
    rf_control = gpio_get_fd_to_value(RF_CONTROL); // RF on off control
    // printf("In gpio 118(RF on/off control): %d\n", rf_control);
    if (rf_control)
    {
        // printf("RF is ON!!");
        set_gpio(RF_STATUS, ON);

        interlock = gpio_get_fd_to_value(INTERLOCK_GP); // interlock
        // printf("In gpio 117(interlock): %d\n", interlock);
        // interlock signal is low when connected so rf needs to be on
        if (interlock)
        {
            // printf("\tInterlock is in Open!!!!\n");
            interlock_fault();
            interlock_flag = 1;
        }
        else
        {
            //  printf("\tInterlock is Connected!!!!\n");
            interlock_flag = 0;
            rf_on_led();
        }
    }
    else
    {
        // printf("RF is OFF!!\n");
        set_gpio(RF_STATUS, OFF);
        interlock_flag = 1;
        system_ready();
    }
}

void plasma_init(void)
{
    // spi initial setup
    SPI1SS1_ADC_MCU_fd = spi_init_setup(1);
    if (SPI1SS1_ADC_MCU_fd != -1)
        printf("SPI initial setup for SPI1SS1_ADC_MCU_fd successful\n");
    DAC_spi_fd = spi_initial_setup(2);
    if (DAC_spi_fd != -1)
        printf("SPI initial setup for pps control successful\n");
    EMB_spi_fd = spi_init_setup(3);
    if (EMB_spi_fd != -1)
        printf("SPI initial setup for meter board successful\n");
    // i2c pps_dac_valueerature sensor initialization
    Temp_i2c_readwrite_init();
    // Configuring the SPI_FDs
    printf("Configuring SPI ... \n");

    if (spi_setup(DAC_spi_fd) < 0)
    {
        printf("SPI configuration for DAC failed !\nExiting Program ...\n");
        fault();
        fault_flag = 1;
        exit(-1);
    }
    else
        printf("SPI configuration for DAC successful\n");
    if (spi_configuration(SPI1SS1_ADC_MCU_fd, 3) < 0)
    {
        printf("SPI configuration for SPI1SS1_ADC_MCU_fd failed !\nExiting Program ...\n");
        fault();
        fault_flag = 1;
        exit(-1);
    }
    else
        printf("SPI configuration for SPI1SS1_ADC_MCU_fd successful\n");
    if (spi_configuration(EMB_spi_fd, 1) < 0)
    {
        printf("SPI configuration for EMB failed !\nExiting Program ...\n");
        fault();
        fault_flag = 1;
        exit(-1);
    }
    else
        printf("SPI configuration for EMB successful\n");
    if (DAC_begin(DAC_spi_fd, spi1_trx) < 0)
        perror("Error : ");
    DAC_forw_ref_pow_spi_fd = spi_initial_setup(0);
    if (DAC_forw_ref_pow_spi_fd != -1)
        printf("SPI initial setup for reading forward and reflected power in connector successful\n");
    if (spi_setup(DAC_forw_ref_pow_spi_fd) < 0)
    {
        printf("SPI configuration for DAC_FORW_REF_POW failed !\nExiting Program ...\n");
        exit(-1);
    }
    else
        printf("SPI configuration for DAC_FORW_REF_POW successful\n");
    if (DAC_begin(DAC_forw_ref_pow_spi_fd, spi2_trx) < 0)
        perror("Error : ");
    gpio();
    pps_dac_setting(0, DAC_spi_fd);
    system_ready();
}