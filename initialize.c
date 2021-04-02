
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "initialize.h"
#include "global_define.h"

void disable_unused_clk()
{
	EALLOW;
	CpuSysRegs.PCLKCR0.bit.CLA1 = 0;
	CpuSysRegs.PCLKCR0.bit.DMA = 0;
	CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 0;
	CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 0;
	CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 0;
	//CpuSysRegs.PCLKCR0.bit.HRPWM = 1;
	//CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

	CpuSysRegs.PCLKCR1.bit.EMIF1 = 0;
	CpuSysRegs.PCLKCR1.bit.EMIF2 = 0;

	//CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM2 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM3 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM7 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM8 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM9 = 1;
	//CpuSysRegs.PCLKCR2.bit.EPWM10 = 0;
	//CpuSysRegs.PCLKCR2.bit.EPWM11 = 0;
	//CpuSysRegs.PCLKCR2.bit.EPWM12 = 0;

	CpuSysRegs.PCLKCR3.bit.ECAP1 = 0;
	CpuSysRegs.PCLKCR3.bit.ECAP2 = 0;
	CpuSysRegs.PCLKCR3.bit.ECAP3 = 0;
	CpuSysRegs.PCLKCR3.bit.ECAP4 = 0;
	CpuSysRegs.PCLKCR3.bit.ECAP5 = 0;
	CpuSysRegs.PCLKCR3.bit.ECAP6 = 0;

	CpuSysRegs.PCLKCR4.bit.EQEP1 = 0;
	CpuSysRegs.PCLKCR4.bit.EQEP2 = 0;
	CpuSysRegs.PCLKCR4.bit.EQEP3 = 0;

	CpuSysRegs.PCLKCR6.bit.SD1 = 0;
	CpuSysRegs.PCLKCR6.bit.SD2 = 0;

	CpuSysRegs.PCLKCR7.bit.SCI_A = 0;
	CpuSysRegs.PCLKCR7.bit.SCI_B = 0;
	CpuSysRegs.PCLKCR7.bit.SCI_C = 0;
	CpuSysRegs.PCLKCR7.bit.SCI_D = 0;

	CpuSysRegs.PCLKCR8.bit.SPI_A = 0;
	CpuSysRegs.PCLKCR8.bit.SPI_B = 0;
	CpuSysRegs.PCLKCR8.bit.SPI_C = 0;

	CpuSysRegs.PCLKCR9.bit.I2C_A = 0;
	CpuSysRegs.PCLKCR9.bit.I2C_B = 0;

	CpuSysRegs.PCLKCR10.bit.CAN_A = 0;
	CpuSysRegs.PCLKCR10.bit.CAN_B = 0;

	CpuSysRegs.PCLKCR11.bit.McBSP_A = 0;
	CpuSysRegs.PCLKCR11.bit.McBSP_B = 0;
	CpuSysRegs.PCLKCR11.bit.USB_A = 0;

	CpuSysRegs.PCLKCR12.bit.uPP_A = 0;

	//CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
	//CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
	//CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
	//CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

	CpuSysRegs.PCLKCR14.bit.CMPSS1 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS2 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS3 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS4 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS5 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS6 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS7 = 0;
	CpuSysRegs.PCLKCR14.bit.CMPSS8 = 0;

	//CpuSysRegs.PCLKCR16.bit.DAC_A = 0;
	//CpuSysRegs.PCLKCR16.bit.DAC_B = 0;
	//CpuSysRegs.PCLKCR16.bit.DAC_C = 0;

	EDIS;
}
void init_rec_GPIO()
{
    // make GPIO14, GPIO15 and GPIO16 as GPIO output pins for unfolder
    // make sure they are off before changing the pin to output
    GpioDataRegs.GPACLEAR.bit.GPIO20 = 1; //rec_pos_pwm
    GpioDataRegs.GPACLEAR.bit.GPIO21 = 1; //rec_neg_pwm
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1; //rec_shutdown
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(20, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(21, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
}

void clear_interrupts()
{
	// Initialize the PIE control registers to their default state.
	// The default state is all PIE interrupts disabled and flags
	// are cleared.
		InitPieCtrl();

	// Disable CPU interrupts and clear all CPU interrupt flags:
		EALLOW;
		IER = 0x0000;
		IFR = 0x0000;
		EDIS;

	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in F2837xD_DefaultIsr.c.
		InitPieVectTable();

	//Map ISR functions
		EALLOW;
		
		PieVectTable.ADCC2_INT = &ac_trigger; //function for ADCc interrupt 2
		
		//the interrupt has to be triggered by b1 because B1 is on SOC1. which is the last SOC in the interrupt.
		PieVectTable.ADCB2_INT = &bd_trigger; //function for ADCb interrupt 2
		EDIS;
}

void init_ADCs()
{
	//stop PWM clock, so no ADC will be triggerred and we can setup ADC
	EALLOW;
	ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0x0; // make PWM clock the same as SYSCLK
	CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0; //disable synchronization of all ePWMs to the TBCLK
	EDIS;

	// Initialize ADC sampling
	InitADCa(); // init ADCa
	InitADCb(); // init ADCb
	InitADCc(); // init ADCc
	InitADCd(); // init ADCd

}

void InitADCa()
{
	EALLOW;
	//write configurations
	AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
	AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	//Set pulse positions to late (at the end of conversion)
	AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//SOC0 measure Vrec on A2 -- Zitao
	AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin A2
	AdcaRegs.ADCSOC0CTL.bit.ACQPS = 60; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
	AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 6; //trigger on ePWM1 SOCB/D
	//AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
	//AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
	//AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;   //No further ADCINT1 pulses are generated until ADCINT1 flag is cleared by user
	//AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
	// note that enabling flag is different from enabling interrupt

	//SOC1 measure Vc2 on pin A3 -- Zitao
	AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin A3
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = 50; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 6; //trigger on ePWM1 SOCB/D
//	AdcaRegs.ADCINTSEL1N2.bit.INT2SEL = 0; //end of SOC1 will set INT2 flag
//	AdcaRegs.ADCINTSEL1N2.bit.INT2E = 1;   //enable INT2 flag
//	AdcaRegs.ADCINTSEL1N2.bit.INT2CONT = 0;   //No further ADCINT2 pulses are generated until ADCINT2 flag is cleared by user
//	AdcaRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT2 flag is cleared
	
	EDIS;
}


void InitADCb(void)
{
	EALLOW;
	//write configurations
	AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	//Set pulse positions to late (at the end of conversion)
	AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//SOC0 measure Vac_neg on pin B2 -- Zitao
//	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert pin B2
//	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 50; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
//	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCB/D
//	AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//	AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//	AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 0;   //No further ADCINT1 pulses are generated until ADCINT1 flag is cleared by user
//	AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	//SOC1 measure Vac_pos on pin B2 -- Zitao
	AdcbRegs.ADCSOC1CTL.bit.CHSEL = 2;  //SOC1 will convert pin B3
	AdcbRegs.ADCSOC1CTL.bit.ACQPS = 50; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
	AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 6; //trigger on ePWM1 SOCB/D
	AdcbRegs.ADCINTSEL1N2.bit.INT2SEL = 1; //end of SOC1 will set INT2 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT2E = 1;   //enable INT2 flag
	AdcbRegs.ADCINTSEL1N2.bit.INT2CONT = 0;   //No further ADCINT2 pulses are generated until ADCINT2 flag is cleared by user
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT2 flag is cleared

	EDIS;
}


void InitADCc(void)
{
	EALLOW;
	//write configurations
	AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	//Set pulse positions to late (at the end of conversion)
	AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
	
	//SOC0 measure idc on pin C4. -- Zitao.
	AdccRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC1 will convert pin C4
	AdccRegs.ADCSOC0CTL.bit.ACQPS = 50; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
	AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 6; //trigger on ePWM1 SOCB/D
//	AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
//	AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//	AdccRegs.ADCINTSEL1N2.bit.INT1CONT = 0;   //No further ADCINT2 pulses are generated until ADCINT2 flag is cleared by user
//	AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
//


	//SOC1 measure vcb_pos on pin C3. -- Zitao.
	AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert pin C3
	AdccRegs.ADCSOC1CTL.bit.ACQPS = 50; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
	AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
	AdccRegs.ADCINTSEL1N2.bit.INT2SEL = 1; //end of SOC1 will set INT2 flag
	AdccRegs.ADCINTSEL1N2.bit.INT2E = 1;   //enable INT2 flag
	AdccRegs.ADCINTSEL1N2.bit.INT2CONT = 0;   //No further ADCINT2 pulses are generated until ADCINT2 flag is cleared by user
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT2 flag is cleared
	
	

	EDIS;
}

void InitADCd(void)
{
	EALLOW;
	//write configurations
	AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
	//Set pulse positions to late (at the end of conversion)
	AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
	//power up the ADC
	AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;

	//SOC0 measure vout signal on pin D1. -- Zitao
	AdcdRegs.ADCSOC0CTL.bit.CHSEL = 1;  //SOC0 will convert pin D1
	AdcdRegs.ADCSOC0CTL.bit.ACQPS = 50; //sample window (# of SYSCLK, needs to corresponds to at least 75ns)
	AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 5; //trigger on ePWM1 SOCA/C
//	AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
//	AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
//	AdcdRegs.ADCINTSEL1N2.bit.INT1CONT = 0;   //No further ADCINT1 pulses are generated until ADCINT1 flag is cleared by user
//	AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

	EDIS;
}


void ADC_bias(void)
{
	// seems that the first ADC reading might not be accurate, do a dummy read
	while (AdcbRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait for first set of measurement (trigger by ac) to finish
	while (AdccRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait for second set of measurement (trigger by bd) to finish
	dummy_read = AdcaResultRegs.ADCRESULT0;
	dummy_read = AdcaResultRegs.ADCRESULT1;
	dummy_read = AdcbResultRegs.ADCRESULT0;
	dummy_read = AdcbResultRegs.ADCRESULT1;
	dummy_read = AdccResultRegs.ADCRESULT0;
	dummy_read = AdccResultRegs.ADCRESULT1;
	dummy_read = AdcdResultRegs.ADCRESULT0;
	dummy_read = AdcdResultRegs.ADCRESULT1;
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag

	//IL_bias_sum = 0;
	//Iout_bias_sum = 0;
	// measure bias voltage of current sensig amplifier
	int16 adc_count = 0;
	for (adc_count=0;adc_count<512;adc_count++)
	{
		while (AdccRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait first set of measurements to finish
		//Iout_bias_sum += AdcbResultRegs.ADCRESULT0;  // read result from ADCa SOC1 for Iout
		IL_bias_sum += AdcaResultRegs.ADCRESULT0;  //read result from ADCc SOC1 for IL_bias

		AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
		AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag
	}
//	Iout_bias = Iout_bias_sum>>9;
	IL_bias = IL_bias_sum>>9;
}

void init_DACs(){

    // Enable DACOUTA
    EALLOW;
    //Use VDAC as the reference for DAC
    DacaRegs.DACCTL.bit.DACREFSEL = 1;
    //Enable DAC output
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;
    //Set DAC to some initial value
    DacaRegs.DACVALS.bit.DACVALS = 2048;

    //Use VDAC as the reference for DAC
    DacbRegs.DACCTL.bit.DACREFSEL = 1;
    //Enable DAC output
    DacbRegs.DACOUTEN.bit.DACOUTEN = 1;
    //Set DAC to some initial value
    DacbRegs.DACVALS.bit.DACVALS = 2048;

    //Use VDAC as the reference for DAC
    DaccRegs.DACCTL.bit.DACREFSEL = 1;
    //Enable DAC output
    DaccRegs.DACOUTEN.bit.DACOUTEN = 1;
    //Set DAC to some initial value
    DaccRegs.DACVALS.bit.DACVALS = 2048;
    EDIS;
}

void init_pwms() {

    // enable ePWM clks
    //ePWM1 as trigger source for ADCs and 
	CpuSysRegs.PCLKCR2.bit.EPWM1=1;
	//buffer full bridge
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    //FCML PSPWM
	CpuSysRegs.PCLKCR2.bit.EPWM4=1;
    CpuSysRegs.PCLKCR2.bit.EPWM5=1;
    CpuSysRegs.PCLKCR2.bit.EPWM6=1;
    CpuSysRegs.PCLKCR2.bit.EPWM7=1;
    CpuSysRegs.PCLKCR2.bit.EPWM8=1;
    CpuSysRegs.PCLKCR2.bit.EPWM9=1;

    // Init GPIO pins
    InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();
	InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    InitEPwm7Gpio();
    InitEPwm8Gpio();
    InitEPwm9Gpio();
    InitEPwm10Gpio();


	InitEPwmMaster(PERIOD);

	//int ps7=PERIOD*1/3.0;
	int ps6=PERIOD*0.4000;
	int ps5=PERIOD*0.8000;
	int ps4=PERIOD*0.8000;
	int ps3=PERIOD*0.4000;
	int ps2=0;
	//int ps_half = PERIOD;
	
	//buffer pwm
	//it seems that we only need one ePWM for buffer. However, Nathan used 2......and I didn't look carefully
	//if we use 2 epwm channels, they have to be 180 out of phase because of up-down mode
	//i.e., pwm2A and pwm3B should be identical
	//InitEPwm(&EPwm2Regs, PERIOD, 0, 0);
	//InitEPwm(&EPwm3Regs, PERIOD, PERIOD, 0);
	
	//FCML pspwm - 7 level
	//InitEPwm(&EPwm4Regs, PERIOD, ps2, 0);
	//InitEPwm(&EPwm5Regs, PERIOD, ps3, 0);
	//InitEPwm(&EPwm6Regs, PERIOD, ps4, 0);
	//InitEPwm(&EPwm7Regs, PERIOD, ps5, 1);
	//InitEPwm(&EPwm8Regs, PERIOD, ps6, 1);
	//InitEPwm(&EPwm9Regs, PERIOD, ps7, 1);


	//FCML pspwm - 6 level
	//InitEPwm(&EPwm4Regs, PERIOD, ps2, 0);
	//InitEPwm(&EPwm1Regs, PERIOD, ps2, 0);
	InitEPwm(&EPwm2Regs, PERIOD, ps3, 0);
	InitEPwm(&EPwm3Regs, PERIOD, ps4, 0);
	InitEPwm(&EPwm4Regs, PERIOD, ps5, 1);
	InitEPwm(&EPwm5Regs, PERIOD, ps6, 1);

	InitEPwm(&EPwm6Regs, PERIOD, ps2, 0);
	InitEPwm(&EPwm7Regs, PERIOD, ps3, 0);
	InitEPwm(&EPwm8Regs, PERIOD, ps4, 0);
	InitEPwm(&EPwm9Regs, PERIOD, ps5, 1);
	InitEPwm(&EPwm10Regs,PERIOD, ps6, 1);





}

void InitEPwmMaster(int32 period) {
	// Initialize as any other EPwm
	InitEPwm(&EPwm1Regs, period, 0, 0);

    // Setup compare
    EPwm1Regs.CMPA.bit.CMPA = period >> 1; 		// make EPwm1 have duty ratio 0.5

    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;     // output EPWMxSYNCO when CTR=0

	// Enable master-PWM interrupt pulse generations
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;            // enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;   // Select SOC on TB = zero
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;             // Generate pulse on 1st event

    EPwm1Regs.ETSEL.bit.SOCBEN  = 1;            // enable SOC on B group
    EPwm1Regs.ETSEL.bit.SOCBSEL = ET_CTR_PRD;  // Select SOC on TB = CMPA
    EPwm1Regs.ETPS.bit.SOCBPRD = 1;             // Generate pulse on 1st event
}

void InitEPwm(volatile struct EPWM_REGS * pwmregs, int32 period, int32 phase, int16 dir) {
	EALLOW;

	pwmregs->TBPHS.all = 0x00000000;
	pwmregs->TBPRD = period;                        // Set timer period. Note PWM counting starts from 0
	pwmregs->TBPRDHR = 0;
	pwmregs->TBPHS.bit.TBPHS = phase;        	    // Set phase
    pwmregs->TBCTL.bit.PHSDIR = dir;             // count up or down after sync
    pwmregs->TBCTR = 0x0000;                        // Clear counter

    // Set up TBCLK
    pwmregs->TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 		// Count up and down
    pwmregs->TBCTL.bit.PHSEN = TB_ENABLE;           // Phase loading
    pwmregs->TBCTL.bit.HSPCLKDIV = TB_DIV1;         // Clock ratio to SYSCLKOUT
    pwmregs->TBCTL.bit.CLKDIV = TB_DIV1;            // Slow to observe on the scope
    pwmregs->TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // output EPWMxSYNCO from EPWMxSYNCI

	// Setup compare register loading
	pwmregs->CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // load from shadow registor from at both CTR=ZERO and CTR=PRD
	pwmregs->CMPCTL.bit.SHDWAMODE = CC_SHADOW;

    // Setup compare
    pwmregs->CMPA.bit.CMPA = period*main_duty;
	pwmregs->CMPA.bit.CMPAHR = (1<<8);

    // Set actions
    pwmregs->AQCTLA.bit.CAU = AQ_SET;
    pwmregs->AQCTLA.bit.CAD = AQ_CLEAR;

    // Active Low complementary PWMs - setup the deadband
    pwmregs->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    pwmregs->DBCTL.bit.POLSEL = DB_ACTV_HIC;
    pwmregs->DBCTL.bit.IN_MODE = DBA_ALL;
    pwmregs->DBRED = deadtime; //EPWM_MIN_DB;
    pwmregs->DBFED = deadtime; //EPWM_MIN_DB;


	// Setup high resolution
	pwmregs->HRCNFG.all = 0x0;
	//pwmregs->HRCNFG.bit.EDGMODE = HR_BEP;  // MEP control on both edge
	//pwmregs->HRCNFG.bit.CTLMODE = HR_CMP;   // CMPAHR controls MEP
	//pwmregs->HRCNFG.bit.HRLOAD  = HR_CTR_ZERO; //load CMPAHR at both CTR=ZERO and CTR=PRD
	//pwmregs->HRCNFG.bit.AUTOCONV = 1;      // Enable auto-conversion logic
	//pwmregs->HRPCTL.bit.HRPE = 1;        // Turn on high-resolution period control.
	//pwmregs->HRPCTL.bit.TBPHSHRLOADE = 1;  // not sure why, the last two lines has to be there to enable high resolution, although period control is not used...
	EDIS;

}



void ADC_manual_calibration()
{
	//this part is useful for ADC calibration
	// turn off inverter
	while(1)
	{
		while (AdccRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait for first set of measurement (trigger by ac) to finish
		while (AdcbRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait for second set of measurement (trigger by bd) to finish
		//dummy_read = AdcaResultRegs.ADCRESULT0;  //Vout
		dummy_read = AdcaResultRegs.ADCRESULT1;  //AC_pos
		//dummy_read = AdcbResultRegs.ADCRESULT0;  //Iout
		//dummy_read = AdcbResultRegs.ADCRESULT1;  //Vrec
		//dummy_read = AdccResultRegs.ADCRESULT0;
		//dummy_read = AdccResultRegs.ADCRESULT1;  //AC_neg
		//dummy_read = AdcdResultRegs.ADCRESULT0;  //IL
		//dummy_read = AdcdResultRegs.ADCRESULT1;
		AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
		AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag
	}
}

void bias_measurement()
{
	// seems that the first ADC reading might not be accurate, do a dummy read
	while (AdcbRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait for first set of measurement (trigger by ac) to finish
	while (AdccRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait for second set of measurement (trigger by bd) to finish
	dummy_read = AdcaResultRegs.ADCRESULT0;
	dummy_read = AdcaResultRegs.ADCRESULT1;
	dummy_read = AdcbResultRegs.ADCRESULT0;
	dummy_read = AdcbResultRegs.ADCRESULT1;
	dummy_read = AdccResultRegs.ADCRESULT0;
	dummy_read = AdccResultRegs.ADCRESULT1;
	dummy_read = AdcdResultRegs.ADCRESULT0;
	dummy_read = AdcdResultRegs.ADCRESULT1;
	AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
	AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag

	// wait........
	// make sure wait for 1s at least for all the external circuit to power on !!!!!
	// 1s is the measured delay from power on to current sensing amp has valid signal
	// otherwise the bias measurement might have unexpected error
	DELAY_US(700000);

	//int32 Iout_bias_sum = 0;
	int32 IL_bias_sum = 0;
	int32 Vab_bias_sum = 0;
	int32 Vc2_bias_sum = 0;
	int32 Idc_bias_sum = 0;
	int32 adc_count = 0;


	for (adc_count=0;adc_count<512;adc_count++)
	{
		while (AdccRegs.ADCINTFLG.bit.ADCINT2 != 1); //wait first set of measurements to finish
		while (AdcbRegs.ADCINTFLG.bit.ADCINT2 != 1);  ////wait second set of measurements to finish
		//Iout_bias_sum += AdcbResultRegs.ADCRESULT0;  // read result from ADCb SOC0 for Iout
		IL_bias_sum += AdcaResultRegs.ADCRESULT0;  //read result from ADCc SOC1 for IL_bias
		//Vab_bias_sum += AdccResultRegs.ADCRESULT0;
		//Vc2_bias_sum += AdcaResultRegs.ADCRESULT1;
		Idc_bias_sum += AdccResultRegs.ADCRESULT0;

		AdcbRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT1 flag
		AdccRegs.ADCINTFLGCLR.bit.ADCINT2 = 1; //clear INT2 flag
	}
	//Iout_bias = Iout_bias_sum>>9;
	IL_bias = IL_bias_sum>>9;
	Idc_bias_count = Idc_bias_sum>>9;
	Vc2_bias_count = Vc2_bias_sum>>9;
}

void init_interrupts()
{
	EALLOW;
	//Enable global Interrupts and higher priority real-time debug events:
	IER |= M_INT1; //Enable group 1 interrupts
	IER |= M_INT10; //Enable group 10 interrupts
	ERTM;  // Enable Global realtime interrupt DBGM
	//enable PIE interrupt  (check above and make sure the group interrupt is enabled)
	PieCtrlRegs.PIEIER10.bit.INTx10 = 1; //for ADCC2_INT
	PieCtrlRegs.PIEIER10.bit.INTx6 = 1; //for ADCB2_INT

	EDIS;
}



// end of code
