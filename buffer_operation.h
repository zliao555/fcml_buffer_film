

#ifndef BUFFER_OPERATION_H_
#define BUFFER_OPERATION_H_

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "global_define.h"

//SSB functions

void Init_SSB(void);

void Process_Signal_SSB (void);

void Set_VC2ref (void);

void Compensate_VC2 (void);

void Control_SSB (void);

//initialize parameters for filters and differentiator
void Init_Filter_and_Diff(void);

// do adc to voltage conversion 
void Init_ADCtoVolts_Ratio(void);

//Low pass filter for Vc2
void LowPassFilter_ibuf(void);

//band pass filter for VC1
void BandPassFilter_VC1(void);

//
void Differentiate_VC1(void);

void Find_VC1_MAX(void);



//--------extern global varibles for SSB----//

extern int16 Vc2_count ;
extern int16 Vab_count ;

// ADC voltage measurements (in volts)
extern float Vbus_V ;
extern float Vc2_V ;
extern float Vc1_V ;
extern float i_buf ;
extern float ibuf_ave ;

extern int16 Vab_bias_count ;
extern int16 Vc2_bias_count ;
extern int16 Vbus_bias_count ;

extern float Vc2_adc_fullvolt_to_count_ratio ;
extern float Vc2_adc_count_to_fullvolt_ratio ;

extern float Vbus_adc_fullvolt_to_count_ratio ;
extern float Vbus_adc_count_to_fullvolt_ratio ;

extern float Vab_adc_fullvolt_to_count_ratio ;
extern float Vab_adc_count_to_fullvolt_ratio ;


//vc1 notch filter parameters
// Notch filter coefficients are float32 types because high precision is required.
//notch q = 5
//@120Hz
extern float32 b0_notch ;
extern float32 b1_notch ;
extern float32 b2_notch;
extern float32 a1_notch ;
extern float32 a2_notch;

// Notch filter sample variables
extern float32 notch_out2_SSB ;
extern float32 notch_out1_SSB ;
extern float32 notch_out_SSB ;
extern float32 notch_in2_SSB ;
extern float32 notch_in1_SSB ;
extern float32 notch_in_SSB ;
extern float Vc1_notch_V ;
extern float Vc1_bpf_V ;
extern float Vc1_bpf_V_old ;



// First-order LPF parameters and coefficients
// vc2 LPF
extern float w_lpf ;
extern float b0_lpf ;
extern float b1_lpf ;
extern float a1_lpf ;
// First-order LPF sample variables
extern float lpf_out1 ;
extern float lpf_out ;
extern float lpf_in1 ;
extern float lpf_in ;


// Vc2 PI compensator
extern float kp_PI ;
extern float ki_PI ;
extern float Vc2_integral_limit ;
extern float a2_I ;
extern float b1_I ;
extern float b2_I ;
extern float Vc2_avg_V ;


// PI controller sample variables
extern float Vc2_error ;
extern float Vc2_error_old ;
extern float Vc2_integral ;
extern float Vc2_integral_old ;
extern float Vc2_ref_V;

//full bridge control
extern float M_control ;
extern float Beta ;
extern float Vab_ref;
extern float vc1_shape;
extern float theta_c1 ;
extern int16 vc1_pll_enable;
extern float pll_compensation;

//vc1 differentiator
// Differentiator parameters and coefficients
extern float kd_diff ;
extern float fd_Hz;
extern float a2_diff ;
extern float b1_diff ;
extern float b2_diff ;
// Differentiator sample variables
extern float Vc1_bpf_diff_V ;
extern float Vc1_bpf_diff_V_old;

extern int16 compensation;
extern int16 dynamic;
extern float vc2_ref_startup;
//determine and hold max for VC1 parameters
extern int vc1_cycle_count ;
extern int16 Vc1_abs_count;
extern int16 Vc1_max_cycle; //find max per cycle
extern int16 Vc1_max_hold; //final value of the vc1_max to set to next cycle
extern float Vc1_max_delta_limit ; //update max_hold if the change is greater than this value
extern float alpha;
extern float theta_c1_pre;

extern float vc1_v_ff;
extern int input_ff;


#endif /* BUFFER_OPERATION_H_ */
