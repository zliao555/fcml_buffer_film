/*
 * initialize.h
 *
 *  Created on: Sep 13, 2016
 *      Author: pilawa_group
 */

#ifndef INITIALIZE_H_
#define INITIALIZE_H_

#include "global_define.h"

// disable unused clk to save power
;void disable_unused_clk();
// initialize active rectifier GPIO
void init_rec_GPIO(void);

// clear all interrupts
void clear_interrupts(void);

// initialize ADC
void init_ADCs(void);
void InitADCa(void); // init ADCa, measure Vrec on A2 (SOC0)
void InitADCb(void); // init ADCb, measure Vac_neg on B2 (SOC0) and Vac_pos on B3 (SOC1)
void InitADCc(void); // init ADCc, measure Vout on C3 (SOC1)
void InitADCd(void); // init ADCd, measure IL on D1 (SOC0)
void ADC_bias(void);

// intialize DAC for debug
void init_DACs(void);

//initialize ePWM1 as clock master
void InitEPwmMaster(int32 period);
//initialize ePWM2 - ePWM7
void InitEPwm(volatile struct EPWM_REGS * pwmregs, int32 period, int32 phase, int16 dir);
// call previous two functions to initialize FCML PWM with phase shift
void init_pwms(void);

// ADC callibration
void ADC_manual_calibration(void);

// measure bias value from analog amplifier
void bias_measurement(void);

// initialize the necessary interrupts
void init_interrupts(void);

// function to clear a block of memory
void memset(void *mem, int ch, size_t length);


//spi functions
void delay_loop(void);
void spi_xmit(Uint16 a);
void spi_fifo_init(void);
void spi_init(void);
void error(void);



// interrupt functions
interrupt void ac_trigger(void);
interrupt void bd_trigger(void);
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);
void InitSpiaGpio_alt();
//manual debug variabls
extern int16 dummy_read;

// current measurement data
extern int16 IL_bias;

extern int16 Idc_bias_count;




//Vout moving average data
extern int16 Vout_sample[MOV_AVE_SIZE];
extern int16 Vout_pointer;
extern int32 Vout_sum;
extern float32 Vout_ave;

// FCML control variables
extern volatile float main_duty;
extern int16 deadtime;


#endif /* INITIALIZE_H_ */
