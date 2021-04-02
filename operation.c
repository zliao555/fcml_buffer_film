
#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include "operation.h"
#include "global_define.h"

void update_d(float32 duty)
{
	Uint16 d = 0;

	if (duty > (1 - duty_limit))
		d = PERIOD*(1 - duty_limit);
	else if (duty < duty_limit)
		d = duty_limit*PERIOD ;
	else
		d = duty*PERIOD;

	//EPwm9Regs.CMPA.bit.CMPA = d;
	EPwm5Regs.CMPA.bit.CMPA = d;
	EPwm4Regs.CMPA.bit.CMPA = d;
	EPwm3Regs.CMPA.bit.CMPA = d;
	EPwm2Regs.CMPA.bit.CMPA = d;
	EPwm1Regs.CMPA.bit.CMPA = d;
	//DBRED = deadtime; //EPWM_MIN_DB;
	//DBFED = deadtime; //EPWM_MIN_DB;
	EPwm5Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm4Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm3Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm2Regs.DBRED = deadtime1; //EPWM_MIN_DB;
	EPwm1Regs.DBRED = deadtime1; //EPWM_MIN_DB;

	EPwm5Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm4Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm3Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm2Regs.DBFED = deadtime1; //EPWM_MIN_DB;
	EPwm1Regs.DBFED = deadtime1; //EPWM_MIN_DB;

}

void update_d_buffer(float32 duty)
{
	Uint16 d_test = 0;


    if (duty > (1 - duty_limit))
        d_test = PERIOD*(1 - duty_limit);
    else if (duty < duty_limit)
        d_test = duty_limit*PERIOD ;
    else
        d_test = duty*PERIOD;

       EPwm10Regs.CMPA.bit.CMPA = d_test;
	   EPwm9Regs.CMPA.bit.CMPA = d_test;
	    EPwm8Regs.CMPA.bit.CMPA = d_test;
	    EPwm6Regs.CMPA.bit.CMPA = d_test;
	    EPwm7Regs.CMPA.bit.CMPA = d_test;


		EPwm10Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm9Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm8Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm7Regs.DBRED = deadtime; //EPWM_MIN_DB;
		EPwm6Regs.DBRED = deadtime; //EPWM_MIN_DB;

		EPwm10Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm9Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm8Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm7Regs.DBFED = deadtime; //EPWM_MIN_DB;
		EPwm6Regs.DBFED = deadtime; //EPWM_MIN_DB;



}


//sin check for the buffer full-bridge
void update_sine_buffer(float32 m)
{

   // float d1 = (0.9*m+1)/2;
    //float d2 = 1-d1;

   // float32 d2 = 0.5*(m+1);
   // float32 d2_out = d2;

//    if (d2>0.9)
//    d2_out = 1;
//    else if (d2>0.8)
//    d2_out = 0.2+m;
//    else if (d2>0.7)
//    d2_out = 0.8;
//    else if (d2>0.6)
//    d2_out = 0.4+m;
//    else if (d2>0.5)
//    d2_out = 0.6;
//    else if (d2 >0.4)
//    d2_out = 0.6+m;
//    else if (d2>0.3)
//    d2_out = 0.4;
//    else if (d2>0.2)
//    d2_out = 0.8+m;
//    else if (d2>0.1)
//    d2_out = 0.2;
//    else if (d2>0)
//    d2_out = 1+m;
	 float32 d1_out = 0.5*(m+1);
//    if (m>0.6)
//        d2_out =  0.5*(m+1);
//        else if (m>0.2)
//        d2_out = m+0.2;
//        else if(m>-0.2)
//         d2_out = 0.5*(m+0.6);
//        else if(m>-0.6)
//         d2_out = 0.2;
//        else
//         d2_out = 0.5*(m+1);
//////
        if (m>s_d)
            d1_out = m;
        else
          d1_out = 0;

    float32 d2_out = d1_out - m;

    update_d(1-d2_out);
	update_d_buffer(1-d1_out);


}



