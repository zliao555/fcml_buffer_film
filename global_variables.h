/*
 * global_variables.h
 *
 *  Created on: 3.15.2019
 *      Author: zitao liao
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include "global_define.h"

//manual debug variabls
int16 dummy_read = 0;
int16 debug_flag = 1;
float theta_debug = 0;

//control flow variable
int16 SFO_status = 0;
Uint16 sdata;     // Send data buffer
Uint16 rdata;     // Receive data buffer
Uint16 rdata_point;  // Keep track of where we are
Uint16 i;       // in the data stream to check received data


float theta_test = 0;
float i_buf = 0;
int16 i_buf_count = 0;
int15 i_dc_count = 0;
float i_dc = 0;
// FCML control variables
volatile float32 main_duty = DEFAULT_DUTY;
volatile float32 main_duty_debug = DEFAULT_DUTY;
volatile int16 sine_index = 0;
volatile int16 sine_step_vc1 = 0;
int16 deadtime = 2;
int16 deadtime1 = 2;
int16 num_points= 2500; 	// number of points in a complete 60 sine wave, should be fs/60
float step = 0.0004;     // step = 1/num_points

// voltage measurements
int16 Vout = 0;
int16 Vrec = 0 ;
int16 Vac_pos = 0;
int16 Vac_neg = 0;
int16 Vac = 0;

// pll loop variables
// notch filter
float32 notch_in = 0;
float32 notch_in1 = 0; //x[n-1]
float32 notch_in2 = 0;
float32 notch_out = 0;
float32 notch_out1 = 0;
float32 notch_out2 = 0;

//150k
float32 notch_b0 = 0.994998549663076;
float32 notch_b1 = -1.989896540894321;
float32 notch_b2 = 0.994998549663076;
float32 notch_a0 = 0.989997099326151;
float32 notch_a1 = -1.989896540894321;


//75k
//float32 notch_b0 = 0.990046630379859;
//float32 notch_b1 = -1.979693038985706;
//float32 notch_b2 = 0.990046630379859;
//float32 notch_a0 = 0.980093260759718;
//float32 notch_a1 = -1.979693038985706;



// PI controller
float32 Kp_pll = 3;
float32 Ki_pll = 1.437966465082929e-05;
float32 x_sum_pll = 0;
float32 pll_PI_out = 0;
float32 theta = 0;
float32 theta_pre = 0;
int16 phase_locked = 0;
int16 duty_test = 0;

int16 pll_on = 0;

//Vout, Vrec moving average data
int16 Vout_sample[MOV_AVE_SIZE];
int16 Vout_pointer=0;
int32 Vout_sum=0;
float Vout_ave=0;

float ibuf_sample[MOV_AVE_SIZE];
float ibuf_sum=0;
float ibuf_ave=0;
float ibuf_ave_cal=0;

int ripple_count = 0;
float i_ripple=0;
float i_ripple_sum_sample[625];
float i_ripple_sum=0;
float i_ripple_ave=0;


//average the angle, during zero-crossing, the pll value got really big or small
//float pll_out_sample[MOV_AVE_SIZE];
//float pll_out_sum=0;
//float pll_ave=0;


// voltage loop parameter
volatile float32 voltage_loop_factor = 0;
volatile int16 voltage_loop_enable = 1;
volatile float Vref = 10;
float Vout_err = 0;
float Vout_err_pre = 0;

//for cout = 2500uf. comment out when using SSB
//volatile float32 v_a1 = 31.075489999494810;
//volatile float32 v_a0 = -31.072002335146536;

//for cout = 80uf
//Num_v1 = 5.828292201243743
//Num_v2 = -5.824368574836052

//10hz. use this set for SSB
//volatile float32 v_a1 =  0.910414746078949;
//volatile float32 v_a0 = -0.910312568412496;
//30hz. use this set for SSB
//volatile float32 v_a1 = 2.731551090807975;
//volatile float32 v_a0 = -2.730631491594759;




float32 i_shape = 0;
float Vref_limit = 3610; // 3610 correspondes to about 400V
float Vref_debug= 0;


// current measurements
int16 IL_count = 0;
volatile int16 IL = 0;
volatile int16 IL_first_half = 0;
volatile int16 Iout = 0;
int16 IL_bias = 0;
int16 Iout_bias = 0;
int32 IL_bias_sum = 0;
int32 Iout_bias_sum = 0;

// current loop parameter
volatile float32 current_loop_duty = 0;
volatile int16 current_loop_enable = 2;
volatile float32 Iref = 0;
volatile float32 Iref_pre = 0;
volatile float32 Iref_diff_limit = 0;
volatile float32 IL_err = 0;
volatile float32 IL_err_sum = 0;
volatile float32 IL_err_sum_limit = 0.2;
//10khz bandwidth
//for c1 = 75uf
volatile float32 i_Kdebug = 1;
//volatile float32 i_Kp = 1.008030489937723e-04;
//volatile float32 i_Ki = 1.023816611034157e-05;


//50x gain on lt1999, 20kHz crossover f
//volatile float32 i_Kp = 4.032121959750891e-05;
//volatile float32 i_Ki = 4.095266444136625e-06;

//50x gain on lt1999, 10kHz crossover f
//volatile float32 i_Kp = 1.829320082791843e-05;
//volatile float32 i_Ki = 2.790798180977166e-06;

//shibin use this, why is this so high?
//volatile float32 i_Kp = 1.022477054700217e-04;
//volatile float32 i_Ki = 1.046925546920875e-05;


//20x gain, 10khz crossover f
//4.573300206979606e-05
//6.976995452442910e-06
//
//volatile float32 i_Kp = 7.886149231057317e-05;
//volatile float32 i_Ki = 2.060209058791360e-06;

//30khz bandwidth
//volatile float32 i_Kp = 1.476660037523630e-04;
//volatile float32 i_Ki = 1.642861708826097e-05;


// feedforward term
 float32 feedforward_duty = DEFAULT_DUTY;


// unfolder control term
int16 period_count = 0;
int16 pre_period_count = 0;
int16 pos_gating = 0;
int16 neg_gating = 0;
int16 pos_gating_count = 10;
int16 neg_gating_count = 10;
int16 ac_dir = 0;

int16 boost_wait = 100;


// start-up and zero_crossing control
volatile int16 startup_in_progress = 1;
volatile int16 update_ref = 0;
int16 gain_boost_high = 50;
int16 gain_boost_count = 0;
int16 gain_boost=1.5;
float32 IL_err_sum_trans_step = 0;

float complete_ff = 0;

float theta_ff = 0;

float32 duty_limit =0;


//--------buffer variables------------------------------//
//buffer sensing, vab and vc2 in ADC count
int16 Vc2_count = 0;
int16 Vab_count = 0;

// ADC voltage measurements (in volts)
float Vbus_V = 0;
float Vbus_V_ave = 0;
float Vc2_V = 0;
float Vc1_V = 0; // vc1_v = vbus_v - vab_v
int vcb_pos = 0;
int vcb_neg = 0;

int16 Idc_bias_count = 0;
int16 Vc2_bias_count = 0;
int16 Vbus_bias_count = 0;

float Vc2_adc_fullvolt_to_count_ratio = 0;
float Vc2_adc_count_to_fullvolt_ratio = 0;

float Vbus_adc_fullvolt_to_count_ratio = 0;
float Vbus_adc_count_to_fullvolt_ratio = 0;

float Vab_adc_fullvolt_to_count_ratio = 0;
float Vab_adc_count_to_fullvolt_ratio = 0;


//vc1 notch filter parameters
// Notch filter coefficients are float32 types because high precision is required.
//notch q = 5
//@120Hz
float32 b0_notch = 0.999528983028614;
float32 b1_notch = -1.999035769948092;
float32 b2_notch = 0.999528983028614;
float32 a1_notch = -1.999035769948092;
float32 a2_notch = 0.999057966057229;

//q = 20, fs = 150khz
//float32 b0_notch = 0.999874352082578;
//float32 b1_notch = -1.999723441205725;
//float32 b2_notch = 0.999874352082578;
//float32 a1_notch = -1.999723441205725;
//float32 a2_notch = 0.999748704165156;

// Notch filter sample variables
float32 notch_out2_SSB = 0;
float32 notch_out1_SSB = 0;
float32 notch_out_SSB = 0;
float32 notch_in2_SSB = 0;
float32 notch_in1_SSB = 0;
float32 notch_in_SSB = 0;
float Vc1_notch_V = 0;
float Vc1_bpf_V = 0;
float Vc1_bpf_V_old = 0;



// First-order LPF parameters and coefficients
// vc2 LPF
float w_lpf = 0;
float b0_lpf = 0;
float b1_lpf = 0;
float a1_lpf = 0;
// First-order LPF sample variables
float lpf_out1 = 0;
float lpf_out = 0;
float lpf_in1 = 0;
float lpf_in = 0;


// Vc2 PI compensator
//float kp_PI = 0.3;
//float ki_PI = 5E-2;


float kp_PI = 2E-4;
float ki_PI = 1E-4;
float Vc2_integral_limit = 1;
float a2_I = 0;
float b1_I = 0;
float b2_I = 0;
float Vc2_avg_V = 0;

float theta_c1 = 0;
float theta_c1_pre = 0;
float vc1_shape = 0;
int16 vc1_pll_enable =1;
float pll_compensation =0;
// PI controller sample variables
float Vc2_error = 0;
float Vc2_error_old = 0;
float Vc2_integral = 0;
float Vc2_integral_old = 0;
float Vc2_ref_V = 0;


//full bridge control
float M_control = 0;//beta * dvc1/dt
float Beta = 0; //output of the PI compensator
float Vab_ref = 0;

//vc1 differentiator
// Differentiator parameters and coefficients
float kd_diff = 1;
float fd_Hz = 25*MOV_AVG_FREQ;
float a2_diff = 0;
float b1_diff = 0;
float b2_diff = 0;
// Differentiator sample variables
float Vc1_bpf_diff_V = 0;
float Vc1_bpf_diff_V_old = 0;

int16 compensation = 1;
int16 dynamic = 1;
float vc2_ref_startup = 1;
//determine and hold max for VC1 parameters
//int vc1_cycle_count = 0;
int16 Vc1_abs_count = 0;
int16 Vc1_max_cycle = 1; //find max per cycle
int16 Vc1_max_hold = 1; //final value of the max to set to next cycle
float Vc1_max_delta_limit = 0.05; //update max_hold if the change is greater than this value
float alpha = 1.4; //scalar between vc1_max and vc2,ref. should be above sqrt (2c2/(2c2+c1))




float isqrt = 0;
float isqrt_hold = 0;
float vc1_v_ff = 0;
int input_ff = 1;
float i_offset = 0.06; //150mA offset because of the function generator
float32 km = 0;
float s_d = 0;
float32 k_pr = 50;
float32 k_pr1 = 50;
float32 k_m = 0.04;
float32 k_t = 0;
float32 k_shift = 0;
float fb_en = 3e-4;
//pi loop for voltage regulation

float32 kp_vcb = 0.1;
float32 ki_vcb = 0.00001;
float32 fb_lim = 0.2;

float32 vcb_ref = 0;
float32 vcb_err =0;
float32 vcb_fb = 0;
float32 vcb_sum_lim = 0.01;
float32 pr_lim = 0.1;

float32 vcb_sum = 0;


//PR controller for voltage regulation
//float32 pk_b0 = -4.188614754940812e-05;
//float32 pk_b1 = 0;
//float32 pk_b2 = 4.188614754940812e-05;
//float32 pk_a1 = -1.999909911425985;
//float32 pk_a0 = 0.999916227704901;

//pr controller for 180 hz harmonic PR
//float32 pk_b0 = -1.256479174219782e-04;
//float32 pk_b1 = 0;
//float32 pk_b2 = 1.256479174219782e-04;
//float32 pk_a1 = -1.999691862656037;
//float32 pk_a0 = 0.999748704165156;

//for PR as in s/s^2+w^2 @ 180hz
float32 pk_b0_th = -3.333285959905490e-06;
float32 pk_b1_th = 0;
float32 pk_b2_th = 3.333285959905490e-06;
float32 pk_a1_th = -1.999943151886588;
float32 pk_a0_th = 1;

//for pr as in s/s^2+w^2 @ 60 hz
float32 pk_b0 = -3.333328069552632e-06;
float32 pk_b1 = 0;
float32 pk_b2 = 3.333328069552632e-06;
float32 pk_a1 = -1.999993683463158;
float32 pk_a0 = 1;




float32 pk_in = 0;
float32 pk_in1 = 0; //x[n-1]
float32 pk_in2 = 0;
float32 pk_out = 0;
float32 pk_out1 = 0;
float32 pk_out2 = 0;


float32 pk_in_th = 0;
float32 pk_in1_th = 0; //x[n-1]
float32 pk_in2_th = 0;
float32 pk_out_th = 0;
float32 pk_out1_th = 0;
float32 pk_out2_th = 0;


float thrid_comp= 0;
float thrid_ph = -0.03;

//tuning for mlcc cap
int16 Vout_rp =0;


int16 dac_num = 1;













#endif /* GLOBAL_VARIABLES_H_ */
