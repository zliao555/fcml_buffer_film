/*
 * operation.h
 *
 *  Created on: Sep 29, 2016
 *      Author: pilawa_group
 */

#ifndef OPERATION_H_
#define OPERATION_H_
//#define FIXED_DUTY_CHECK

#include "global_define.h"

void update_d(float d);  // change duty ratio to a given value, pspwm for phase 1


void update_d_buffer(float d); // buffer duty ratio update
void update_sine_buffer(float32 m); // update duty ratio to give a sinewave output

extern volatile int16 sine_index;
extern int16 num_points; 	// number of points in a complete sine wave
extern float32 step;     // step = 1/num_points

extern float32 duty_limit;
extern float32 km;
extern float s_d;
extern int16 deadtime;
extern int16 deadtime1;


#endif /* OPERATION_H_ */
