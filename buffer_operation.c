
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "buffer_operation.h"

//initialize all parameters related to the buffer
//call init_Filter_and_Diff and init_ADCtoVolts_Ratio
void Init_SSB(void){
	Init_Filter_and_Diff();
	Init_ADCtoVolts_Ratio();
}


//process all necessary signal for VC2 compensation
//low pass vc2 to get vc2,dc
//band pass vc1 to get 120 Hz component
//differentiate vc1 to get ic1.
void Process_Signal_SSB (void){
	
	LowPassFilter_ibuf();
	//BandPassFilter_VC1();
	//Find_VC1_MAX();
	//Differentiate_VC1();
}

void Set_VC2ref (void){
	if (dynamic == 0) Vc2_ref_V = vc2_ref_startup;
	else {
		if (input_ff == 0)
		Vc2_ref_V = Vc1_max_hold*alpha*Vab_adc_count_to_fullvolt_ratio;
		else
		Vc2_ref_V = vc1_v_ff*alpha;
	}

}



//PI compensator to regulate vc2
void Compensate_VC2 (void) {

	Vc2_error = Vc2_ref_V - Vc2_avg_V; // Compute error term of Vc2 (average Vc2). Note: In units of volts
	Vc2_integral = a2_I*Vc2_integral_old + b1_I*Vc2_error + b2_I*Vc2_error_old;
	if ((Vc2_integral > Vc2_integral_limit) || (Vc2_integral < -Vc2_integral_limit)) Vc2_integral = Vc2_integral_old; // Integral wind-up compensation. Limit Vc2_PI to safe values.
	Beta = kp_PI*Vc2_error + ki_PI*Vc2_integral; //same beta in the SSB paper
	if (Beta <-1E-2)
		Beta = -1E-2;
	else if(Beta >1E-2)
		Beta = 1E-2;
}

//set conversion ratio for full-bridge buffer converter
void Control_SSB(void){
	float vc1_amp = 0;
	if (input_ff == 0)
		vc1_amp = Vc1_max_hold*Vab_adc_count_to_fullvolt_ratio;
	else
		vc1_amp = kd_diff*(vc1_v_ff)*(1+0.002*vc1_v_ff);


	if (vc1_pll_enable == 1){
		pll_compensation = __cospuf32(theta_c1)*Beta*vc1_amp*120*2*3.1415926535;
		if (compensation == 1)
			Vab_ref = vc1_amp*vc1_shape - pll_compensation ;
		else
			Vab_ref = vc1_amp*vc1_shape;

	}else{
		if (compensation == 1)
			Vab_ref = Vc1_bpf_diff_V*Beta - Vc1_bpf_V;//set reference for vab
		else
			Vab_ref =  -Vc1_bpf_V;
	}

	if (Vc2_V < 2)
	M_control = Vab_ref*0.5; //feedforward control for vab
	else
	M_control = Vab_ref/Vc2_V;
}


//initialize parameters for all digital filters and differentiators
void Init_Filter_and_Diff(void){
	
	// Difference equation coefficients: integral controller. u(n) = b1*e(n) + b2*e(n-1) + a2*u(n-1)
	float T = (float) 1/SWITCHING_FREQUENCY/1000;//sampling period for digital filter

	a2_I = 1;
	b1_I = T/2;
	b2_I = T/2;
	//low pass filter initialization
	w_lpf = 10; //10 Hz
	// First-order LPF difference equation coefficients
	b0_lpf = w_lpf*T/(w_lpf*T + 2);
	b1_lpf = b0_lpf;
	a1_lpf = (w_lpf*T - 2)/(w_lpf*T + 2);
	
	//differentiator initialization
	float wd = 2*3.14159265*fd_Hz; // Convert fd from Hz to rad/s
	// Difference equation coefficients: differentiator. u(n) = b1*e(n) + b2*e(n-1) + a2*u(n-1)
	a2_diff = -(T*wd - 2)/(T*wd + 2);
	b1_diff = kd_diff*2*wd/(T*wd + 2);
	b2_diff = -kd_diff*2*wd/(T*wd + 2);	
}


//initialize adc count to volts ratio based on the bias measurement for vc2, vbus and vab
void Init_ADCtoVolts_Ratio(void){
	
	float Vc2_adc_range_count = (float) (4096 - Vc2_bias_count); 						// Full adc range in counts (w/ bias)
	float Vc2_adc_range_count_div = 1/Vc2_adc_range_count; 			// Inverse of full adc range in counts (w/ bias)
	float Vc2_adc_range_fullvolt = (float) (VC2_ADC_MAX_VOLT - VC2_ADC_MIN_VOLT); 		// Full adc range in volts (full voltage)
	float Vc2_adc_range_fullvolt_div = 1/Vc2_adc_range_fullvolt; 	// Inverse of full adc range in volts (full voltage)
	// Define global adc conversion ratios for adc conversion from full voltage to counts (and vice versa)
	Vc2_adc_fullvolt_to_count_ratio = Vc2_adc_range_count*Vc2_adc_range_fullvolt_div; // Full volt to count adc conversion. Count = Volt*Ratio.
	Vc2_adc_count_to_fullvolt_ratio = Vc2_adc_range_count_div*Vc2_adc_range_fullvolt; // Full volt to count adc conversion. Volt = Count*Ratio.

	// Declare and define local variables for adc conversion from full voltage to counts (and vice versa)
	float Vbus_adc_range_count = (float) (4096 - 0); 						// Full adc range in counts (w/ bias)
	float Vbus_adc_range_count_div = 1/Vbus_adc_range_count; 			// Inverse of full adc range in counts (w/ bias)
	float Vbus_adc_range_fullvolt = (float) (VBUS_ADC_MAX_VOLT - VBUS_ADC_MIN_VOLT); 		// Full adc range in volts (full voltage)
	float Vbus_adc_range_fullvolt_div = 1/Vbus_adc_range_fullvolt; 	// Inverse of full adc range in volts (full voltage)
	// Define global adc conversion ratios for adc conversion from full voltage to counts (and vice versa)
	Vbus_adc_fullvolt_to_count_ratio = Vbus_adc_range_count*Vbus_adc_range_fullvolt_div; // Full volt to count adc conversion. Count = Volt*Ratio.
	Vbus_adc_count_to_fullvolt_ratio = Vbus_adc_range_count_div*Vbus_adc_range_fullvolt; // Full volt to count adc conversion. Volt = Count*Ratio.

	// Declare and define local variables for adc conversion from full voltage to counts (and vice versa)
	float Vab_adc_range_count = (float) (4096 - Vab_bias_count); 						// Full adc range in counts (w/ bias)
	float Vab_adc_range_count_div = 1/Vab_adc_range_count; 			// Inverse of full adc range in counts (w/ bias)
	float Vab_adc_range_fullvolt = (float) (VAB_ADC_MAX_VOLT - VAB_ADC_MIN_VOLT); 		// Full adc range in volts (full voltage)
	float Vab_adc_range_fullvolt_div = 1/Vab_adc_range_fullvolt; 	// Inverse of full adc range in volts (full voltage)
	// Define global adc conversion ratios for adc conversion from full voltage to counts (and vice versa)
	Vab_adc_fullvolt_to_count_ratio = Vab_adc_range_count*Vab_adc_range_fullvolt_div; // Full volt to count adc conversion. Count = Volt*Ratio.
	Vab_adc_count_to_fullvolt_ratio = Vab_adc_range_count_div*Vab_adc_range_fullvolt; // Full volt to count adc conversion. Volt = Count*Ratio.

}


//extract dc component for vc2
void LowPassFilter_ibuf (void){
	lpf_in = i_buf;
	lpf_out = b0_lpf*lpf_in + b1_lpf*lpf_in1 - (a1_lpf*lpf_out1);
	
	lpf_out1 = lpf_out;
	lpf_in1 = lpf_in;
	
	ibuf_ave = lpf_out;

}
/*
void PLL_VC1 (void){
	notch_out2_vc1 = notch_out1_vc1;
	notch_out1_vc1 = notch_out_vc1;
	notch_in2_vc1 = notch_in1_vc1;
	notch_in1_vc1 = notch_in_vc1;
	notch_in_vc1 = __cospuf32(theta_vc1)*Vc1_V;
	notch_out_vc1 = notch_b2*notch_in_vc1+notch_b1*notch_in1_vc1+notch_b0*notch_in2_vc1-notch_a1_vc1*notch_out1_vc1-notch_a0*notch_out2_vc1;


	x_sum_pll_vc1 = x_sum_pll_vc1+Ki_pll_vc1*notch_out_vc1;
	if (fabs(x_sum_pll_vc1)>10)
		x_sum_pll_vc1 = 10;
	pll_PI_out_vc1 = Kp_pll_vc1*notch_out_vc1+x_sum_pll_vc1;

	// determined unfolder gating signal
	theta_pre_vc1 = theta_vc1;
	theta_vc1 = theta_vc1 + (6.6666666666667e-6)*(120+pll_PI_out_vc1);

}

*/
//extract 120 Hz ac component for vc1
void BandPassFilter_VC1 (void){
	notch_in_SSB = Vc1_V;
	
	notch_out_SSB = b0_notch*notch_in_SSB + b1_notch*notch_in1_SSB + b2_notch*notch_in2_SSB - (a1_notch*notch_out1_SSB + a2_notch*notch_out2_SSB);
	notch_out2_SSB = notch_out1_SSB;
	notch_out1_SSB = notch_out_SSB;
	notch_in2_SSB = notch_in1_SSB;
	notch_in1_SSB = notch_in_SSB;
	
	Vc1_notch_V = (float) notch_out_SSB;
	Vc1_bpf_V = Vc1_V - Vc1_notch_V;
	
}

//differentiate vc1 to get ic1
void Differentiate_VC1 (void){
	//take derivative of the bandpassed VC1
	Vc1_bpf_diff_V = a2_diff*Vc1_bpf_diff_V_old + b1_diff*Vc1_bpf_V + b2_diff*Vc1_bpf_V_old;
	Vc1_bpf_V_old = Vc1_bpf_V;
}


//find the amplitude of the bpf vc1 to set reference for vc2 compensation
//the maximum of vc1 is updated every 120 hz at the zero crossing of vab
void Find_VC1_MAX(void){
	//vc1_cycle_count++;
	Vc1_abs_count = Vab_adc_fullvolt_to_count_ratio*fabs(Vc1_bpf_V); //absolute value of vc1_bpf
	if (Vc1_abs_count  >Vc1_max_cycle)
		Vc1_max_cycle = Vc1_abs_count ;
	//keep track of the vc1 value and update the max value

	//detect zero-crossing of vc1 bpf and update the cycle max
	if (theta_c1>0 && theta_c1<6.6666666666667e-6 ){

//		vc1_cycle_count = 1;
		//do not change the final output if the change in max value is within the limit
		if (fabs(1-(Vc1_max_cycle/(Vc1_max_hold+0.00001))) > Vc1_max_delta_limit)
			Vc1_max_hold = Vc1_max_cycle;

		else
		{//keep the same vc1_max_hold
		 //do nothing
		}

		Vc1_max_cycle = 0; //reset cycle max for next cycle

	}


}









