
#ifndef GLOBAL_DEFINE_H_
#define GLOBAL_DEFINE_H_


//#define ADC_calibration
//#define TIMING_CHECK
//#define FIXED_DUTY_CHECK
//#define SINE_TEST         //sine duty ratio test


#define NUM_LEVELS 7
#define SYSCLK 					180000
#define SWITCHING_FREQUENCY 	150 //kHz
#define PERIOD				 	600 //(SYSCLK/SWITCHING_FREQUENCY/2, since PWM is count_up_down)
#define DEFAULT_DUTY            0.7

#define MOV_AVE_SIZE 1250   //array size for the moving average ( = fsw / 120)
#define MOV_AVE_SIZE_DIV 0.0008
#define MOV_AVG_FREQ 120


#define VC2_ADC_MAX_VOLT		120
#define VC2_ADC_MIN_VOLT		0

#define VBUS_ADC_MAX_VOLT		450 	// Expected maximum full voltage value (corresponds to full ADC counts, 4096, without bias compensation)
#define VBUS_ADC_MIN_VOLT		0		// Expected minimum full voltage value (corresponds to minimum ADC counts. In this case, measured bias at zero)

#define VAB_ADC_MAX_VOLT		80 		// Expected maximum full voltage value (corresponds to full ADC counts, 4096, without bias compensation)
#define VAB_ADC_MIN_VOLT		0		// Expected minimum full voltage value (corresponds to minimum ADC counts. In this case, measured bias at zero)


#endif /* GLOBAL_DEFINE_H_ */
