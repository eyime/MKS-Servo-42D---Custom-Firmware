#include <n32l40x.h>
#include <math.h>

#include "foc_pwm.h"

// Counter max value
#define _MAX_VAL_ 800

// External pointers
// OD pointer 
Motor_OD_Entry_t *pwm_ptrOD = 0;
// Power supply voltage
float *pwm_power_voltage = 0;
// Limit motor voltage to a value lower than power supply
float *pwm_maxmotor_voltage = 0;


// Set location of motor OD, for auto-updating motor voltages
void pwm_init( Motor_OD_Entry_t *pMotorOd ) {
	// store external pointer in local variable
	pwm_ptrOD = pMotorOd;
}

// check data after flash load
void pwm_check_data() {
	// Get pointer to motor PCB board voltage (Vin)
	// and pointer to maximun motor voltage
	pwm_power_voltage    = (float*) (pwm_ptrOD[OD_VIN_VOLTAGE].ptrData);
	pwm_maxmotor_voltage = (float*) (pwm_ptrOD[OD_MOTOR_MAXVOLT].ptrData);
	// Securtity check
	if ( *pwm_power_voltage <= 1.0f ) *pwm_power_voltage = 1.0f;
	if ( *pwm_maxmotor_voltage <= 1.0f ) *pwm_maxmotor_voltage = 1.0f;
	if ( *pwm_maxmotor_voltage > *pwm_power_voltage ) *pwm_maxmotor_voltage = *pwm_power_voltage;	
}

// Set channel level output
// PWM is inverted (started as LOW)
void pwm_set_channels(float _f1, float _f2, float _f3, float _f4) {
	// Limits values between 0 % a 90%
	//                   > 800  to  80
  _f1 = fmaxf( 80.0f, 800.0f * (1.0f - _f1) );
  _f2 = fmaxf( 80.0f, 800.0f * (1.0f - _f2) );
  _f3 = fmaxf( 80.0f, 800.0f * (1.0f - _f3) );
  _f4 = fmaxf( 80.0f, 800.0f * (1.0f - _f4) );

	// Greater than 798	is OFF
  TIM_SetCmp1( TIM3, _f1 );
  TIM_SetCmp2( TIM3, _f2 );
  TIM_SetCmp3( TIM3, _f3 );
  TIM_SetCmp4( TIM3, _f4 );
}

// Set PWM voltages from stationary frames Alpha-Beta
void pwm_set_from_Ua_Ub( float Ualpha, float Ubeta ) {
	// For Stepper Motors
	// Phase1 = Ualpha
	// Phase2 = Ubeta
	
	// Compute Phase1 and Phase2 PWM outputs
	float f1a, f1b, f2a, f2b;
	
	// Security check
	if (*pwm_maxmotor_voltage > 0.0f ) {
		// Ualpha:
		Ualpha = Ualpha / *pwm_maxmotor_voltage;
		// Ubeta:
		Ubeta = Ubeta / *pwm_maxmotor_voltage;
	}
	
	// limit to -1.0 to 1.0
	if (Ualpha > 1.0f) Ualpha =  1.0f;
	if (Ualpha <-1.0f) Ualpha = -1.0f;
	if (Ubeta > 1.0f) Ubeta =  1.0f;
	if (Ubeta <-1.0f) Ubeta = -1.0f;
	
	// apply voltage as percent, -1.0f to 1.0f
	if ( Ualpha >= 0.0f ) {
		f1a = Ualpha;
		f1b = 0.0f;
	} else {
		f1a = 0.0f;
		f1b = -Ualpha;
	}
	if ( Ubeta >= 0.0f ) {
		f2a = Ubeta;
		f2b = 0.0f;
	} else {
		f2a = 0.0f;
		f2b = -Ubeta;
	}
	
	// Write PWM to motor
	pwm_set_channels( f1a, f1b, f2a, f2b );
}

// Set PWM channel from rotating frame Uquad Udirect
// Inverse Park-Clarke
void pwm_set_from_Ud_Uq( float Ud, float Uq, float elect_ang ) {
  float SinA, CosA;
	
	// electrical angle
	SinA = sinf( elect_ang );
	CosA = cosf( elect_ang );
	
  // Inverse park transform
  float Ualpha =  Ud*CosA - Uq*SinA;
  float Ubeta  =  Ud*SinA + Uq*CosA;

  // set PWM from stationary frame
	pwm_set_from_Ua_Ub( Ualpha, Ubeta );
}
