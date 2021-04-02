//###########################################################################
// fcml full ripple port control, Zitao Liao
//


#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "global_variables.h"
#include "global_define.h"
#include "initialize.h"
#include "operation.h"
#include "buffer_operation.h"
#include "SFO_V8.h"
//#include "fpu_math.h"
#include "math.h"
//#include "examples_setup.h"


// some variables for SFO library
int MEP_ScaleFactor; // Global variable used by the SFO library
                     // Result can be used for all HRPWM channels
                     // This variable is also copied to HRMSTEP

                     // register by SFO(0) function.

//
// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
{&EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs,
  &EPwm6Regs, &EPwm7Regs, &EPwm8Regs, &EPwm9Regs, &EPwm10Regs};


void main(void)
{
	// Step 1. Initialize System Control:
	// PLL, WatchDog, enable Peripheral Clocks
	// This example function is found in the F2837xD_SysCtrl.c file.





	InitSysCtrl();  //!!!! This function might be different from default.
    				//!!!! go this this function and double check the clock is
    				// set to 120MHz.s

    // disable peripheral clks that we do not use to save power
	disable_unused_clk();

    // Step 2. Initialize GPIO.
    InitGpio(); // set the GPIO to it's default state (i.e., high impedance input)

    // initialize the following pin for active rectifier
    // GPIO14: rec_neg_PWM
    // GPIO15: rec_pos_PWM
    // GPIO16: rec_shutdown
    //init_rec_GPIO();



	// Step 3. Clear all interrupts and initialize PIE vector table:
    clear_interrupts();

	// Step 4: peripheral setup
    init_ADCs(); // initialize all ADCs (a,b,c,d)

    init_DACs(); // initialize DACs for debugging

    init_pwms(); // initialize phase-shifted PWM

    SFO_status = SFO_INCOMPLETE;
    while(SFO_status == SFO_INCOMPLETE) // Call until complete
    {
        SFO_status = SFO();
        if (SFO_status == SFO_ERROR)// SFO function returns 2 if an error occurs & # of MEP steps/coarse step exceeds maximum of 255.
        {
        	ESTOP0;     // Stop here and handle error
        }

    }


	EALLOW;
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1; //enable synchronization of all ePWMs to the TBCLK
										// this will start ADC conversion since ADC are triggered by ePWM1
	EDIS;

	//ADC_bias();
	#ifdef ADC_calibration
	ADC_manual_calibration();
	#endif

	// measure amplifier IC bias value here before anything turn on
	// things to measure include bias for IL, Iout
	bias_measurement();  // this function will take a few seconds

	Init_SSB();
	// clearing array
	// this has to be done, otherwise the corresponding XX_sum variable might not have the correct number
	//clear the moving average array for Vout signal
	memset(Vout_sample, 0, MOV_AVE_SIZE);
	memset(ibuf_sample, 0, MOV_AVE_SIZE);
//	memset(pll_out_sample, 0, MOV_AVE_SIZE);



	// Step 5: interrup setup
	init_interrupts();

	//eanble global interrupts.
	//this will enable the flowing:
	//1, boost converter PFC function
	//2, unfolder start function
	EnableInterrupts();

	// Step 6: infinite loop, two ISR handle the rest
	while(1)
	{
	    while(SFO_status == SFO_INCOMPLETE) // Call until complete
	    {
	        SFO_status = SFO();
	    }
	}

}



// purpose of function ac_trigger():
// given PWM in count_up_down mode, this function should update 300kHz
// 1, based on Iout measurement, calculate the moving average value, figure out the AC part and generate reference for IL
interrupt void ac_trigger(void)
{

    //update_sine_buffer();

		//main_duty = main_duty_debug;
    // pll to sense the input current

    // precalculated vc1 magnitude

    // sense voltage bus and drive it to zero

    //
	//calculate the dc value of ibuf for the load info, 120 Hz moving average
	ibuf_sum= ibuf_sum + i_buf - ibuf_sample [sine_index];
	ibuf_sample [sine_index] = i_buf;
	ibuf_ave = ibuf_sum * MOV_AVE_SIZE_DIV;



	if (Vbus_V_ave < 10){
		Vbus_V_ave = 10;
		Vbus_V = 10;
	}

	isqrt = __sqrt(2*Vbus_V_ave*(ibuf_ave-i_offset)/(2*3.14159265359*60*80e-6));



 // update_d(main_duty_debug);
 // update_d_buffer(main_duty_debug);

	Vout = (signed)AdcdResultRegs.ADCRESULT0;
	i_dc_count = (signed)AdccResultRegs.ADCRESULT0;
//	vcb_pos = (signed)AdcbResultRegs.ADCRESULT0;
	//vcb_neg = (signed)AdccResultRegs.ADCRESULT0;
		//update_d(main_duty);

		//((M_control+1)/2);
		//inductor current is sensed with D1; shibin's pfc is D0.

	switch (dac_num)
	{
	case 1:
	    DaccRegs.DACVALS.bit.DACVALS = pk_out_th*10+1000;
	break;
	case 2:
	    DaccRegs.DACVALS.bit.DACVALS = ibuf_ave*500+1000;
	break;
	case 3:
	    DaccRegs.DACVALS.bit.DACVALS = vcb_ref*10+1000;
	break;


	}

	DacaRegs.DACVALS.bit.DACVALS = 0.1*vcb_ref+1000;
	DacbRegs.DACVALS.bit.DACVALS = pk_out*500+1000;
	// clear flag and get ready for next interrupt
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

}

// purpose of function bd_trigger():
interrupt void bd_trigger(void)
{

    //update_d_buffer (main_duty_debug);
	vcb_pos = (signed)AdcbResultRegs.ADCRESULT1;
	vcb_neg = (signed)AdccResultRegs.ADCRESULT1;

	theta_test = 2*step*sine_index;
	i_buf_count = (signed)AdcaResultRegs.ADCRESULT0;


	i_buf = (float) ((i_buf_count- IL_bias)*3.3/(4096*0.1));

//	i_buf = __sinpuf32(theta_test+theta_debug)*km + km; //simulate buffer current

	Vout_sum = Vout_sum + Vout - Vout_sample[sine_index]; // Vout_sum = Vout_sum + newest value - oldest value
	Vout_sample[sine_index] = Vout; // replace the sample value
	Vout_ave = Vout_sum *MOV_AVE_SIZE_DIV;

	Vout_rp = abs(Vout - Vout_ave);


	Vbus_V = Vout*Vbus_adc_count_to_fullvolt_ratio;
	Vbus_V_ave = Vout_ave*Vbus_adc_count_to_fullvolt_ratio;
	// PLL i_buf 120 Hz
	notch_out2 = notch_out1;
	notch_out1 = notch_out;
	notch_in2 = notch_in1;
	notch_in1 = notch_in;
	notch_in = __cospuf32(theta)*(i_buf-ibuf_ave);
	notch_out = notch_b2*notch_in+notch_b1*notch_in1+notch_b0*notch_in2-notch_a1*notch_out1-notch_a0*notch_out2;
	x_sum_pll = x_sum_pll+Ki_pll*notch_out;

	if (fabs(x_sum_pll)>10)
	    x_sum_pll = 0;

	pll_PI_out = Kp_pll*notch_out+x_sum_pll;

	theta = theta + (6.66666666667e-6)*(120+pll_PI_out);

	if (sine_index == 0.5*num_points){
        sine_index = 0;
    }

    sine_index++;

	//calculate c angle, theta = 2*pi*2wt
	//theta_c = 2pi wt
    if (theta>=2)
        theta = theta - 2;
    else if (theta<0)
        theta = theta + 2;
    theta_pre = theta_c1;
    theta_c1 = (theta+0.5) * 0.5;


    if (theta_c1>=1)
          theta_c1 = theta_c1 - 1;
    else if (theta_c1<0)
          theta_c1 = theta_c1 + 1;

	 if (theta_c1< boost_wait*6.6666e-6 && theta_c1>=0)
	{
		pos_gating = 1;
		isqrt_hold = isqrt;
	}
	else
	{
		pos_gating = 0;
	}


	 //measure 120 hz magnitude and pi for 60 hz voltage



	 //measure 240 hz magnitude and pi for 180 hz voltage injection



	 //measure 360 hz magnitude and pi 300 hz voltage injection





   // main_duty = __sinpuf32(theta+0.5)*k_pr;
    vcb_ref = __sinpuf32(theta_c1+(isqrt_hold*k_shift))*isqrt_hold*198/(198-k_m*isqrt_hold);
    main_duty_debug = vcb_ref/Vbus_V;

	vcb_err = vcb_ref - (vcb_pos - vcb_neg)*Vbus_adc_count_to_fullvolt_ratio;
	vcb_sum = vcb_sum + vcb_err*ki_vcb;

	//pr controller

	pk_out2 = pk_out1;
	pk_out1 = pk_out;
	pk_in2 = pk_in1;
	pk_in1 = pk_in;
	pk_in = vcb_err;
	pk_out = (pk_b2*pk_in+pk_b1*pk_in1+pk_b0*pk_in2)*k_pr-pk_a1*pk_out1-pk_a0*pk_out2;


    pk_out2_th = pk_out1_th;
    pk_out1_th = pk_out_th;
    pk_in2_th = pk_in1_th;
    pk_in1_th = pk_in_th;
    pk_in_th = -(vcb_pos - vcb_neg)*Vbus_adc_count_to_fullvolt_ratio;
    pk_out_th = (pk_b2_th*pk_in_th+pk_b1_th*pk_in1_th+pk_b0_th*pk_in2_th)*k_pr1-pk_a1_th*pk_out1_th-pk_a0_th*pk_out2_th;


	//......


	// if (theta<0.0001&theta>0)
	  //  vcb_sum = 0;
	  if (fabs(vcb_sum)>vcb_sum_lim)
		vcb_sum = 0;
	  if (fabs(pk_out*k_pr)>pr_lim)
		  pk_out = 0;

	vcb_fb = vcb_err+pk_out+pk_out_th*kp_vcb;


	if (fabs(fb_en*vcb_fb)>fb_lim)
		vcb_fb = 0;

    		//+isqrt*(isqrt*k_s+1)*__cospuf32(theta_c1+theta_debug);
   //update_sine_buffer(main_duty_debug+fb_en*vcb_fb+thrid_comp);


   //open loop test fcml
 //update_sine_buffer(__sinpuf32(theta_test)*0.3);

   update_d(0.3);
  update_d_buffer(0.3);

   // clear flag and get ready for next interrupt
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag
	//AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

}


