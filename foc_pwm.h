#ifndef _FOC_PWM_H_
#define _FOC_PWM_H_

#include "foc_motor_od.h"

// Set power motor object dictionary for reading
void pwm_init( Motor_OD_Entry_t *motor_od );

// check loaded data
void pwm_check_data();

// Set PWM directly to each half bridge
// input values from 0 to 1 (0% to 100%)
// values are limited to 99% of power voltage due to ADC requirements
void pwm_set_channels(float _channel1, float _channel2, float _channel3, float _channel4);

// Apply voltage to stationary frame alpha-beta
void pwm_set_from_Ua_Ub( float Ualpha, float Ubeta );

// Apply voltage to moving frame Direct - Quadrature axis frame
void pwm_set_from_Ud_Uq( float Ud, float Uq, float elect_ang );

#endif
