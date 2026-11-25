#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <stdio.h>

#include "foc_utils.h"
#include "foc_time.h"
#include "foc_oled.h"


// angle in radians must be in range [0, 2 PI)
float check_rad_range(float angle) {
	angle = fmodf( angle, _2PI );
	
	// no negative numbers
	if ( angle < 0.0f ) angle += _2PI;
	
	return angle;
}

//
// Low Pass Filter
//

// First Order Equations
// Tf -> filter constant time
// Fc -> filter cut-off frequency (bandwidth)
//
// Relantionship:
//    Fc = 1 / (2 pi Tf )
// At 5 Tf is the filter delay in the signal
//
// Tf: filter constant time, example: Tf = tau = RC
// Ts: filter sampling time
void LPF_Init(LPF_t* LPFData, float Tf, float Ts) {
	LPFData->alpha = Ts /( Ts + Tf );
  LPFData->y_prev = 0.0f;
}

void LPF_Reset( LPF_t *LPFData) {
	LPFData->y_prev = 0.0f;
}

float LPF_Compute(LPF_t* LPFData, float x) {
    // calculate the first order filer
    float y = (LPFData->alpha)*x + (1.0f - LPFData->alpha)*(LPFData->y_prev);

    // save the variables for the future steps
    LPFData->y_prev = y;

    return y;
}

//
// Proportional - Integral - Derivative Controller
//

// PID Initializer
void PID_Init(PID_t *pPID, float Kp, float Ki, float Kd, float Ts, float feedFW, float lim_sp, float lim_out) {
	// Set values
	pPID->Kp = Kp;              // Kp constant
  pPID->Ki = Ki;              // Ki constant
  pPID->Kd = Kd;              // Kd constant
  pPID->Ts = Ts;              // sampling time [S]
	pPID->feedFW = feedFW;      // PID feed-forward value
  pPID->limit_ramp = lim_sp;  // output ramp limit [ex. volts/second]
  pPID->limit_y = lim_out;    // output limit      [ex. maximun volts]
  pPID->err_prev = 0.0f;      // previous error
  pPID->y_prev = 0.0f;        // previous ouput
  pPID->int_prev = 0.0f;      // previous inegral output
}

// Compute PID output
float PID_Compute(PID_t *pPID, float error) {
	// PID controller function
	// u(s) = (P + I/s + Ds) e(s)
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	float Kpout = (pPID->Kp) * error;
	// Tustin transform of the integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	float Kiout = (pPID->int_prev) + (pPID->Ki)*(pPID->Ts)*(error + (pPID->err_prev))/2.0f;
	// Integral antiwindup 
	if ( pPID->limit_y > 0.0f ) {
		if ( Kiout >  pPID->limit_y ) Kiout =  pPID->limit_y;
		if ( Kiout < -pPID->limit_y ) Kiout = -pPID->limit_y;
	}	
	// Discrete derivation
	// u_dk = D(ek - ek_1)/Ts
	float Kdout = (pPID->Kd)*(error - pPID->err_prev) / (pPID->Ts);
	// sum all the components
	float output = Kpout + Kiout + Kdout;

	// Add feed-forward value
	// not set if negative value
	if ( pPID->feedFW > 0.0f ) {
		if ( output >= 0.0f ) output += pPID->feedFW; else output -= pPID->feedFW;
	}
	
	// limit output
	// not set if negative value
	if ( pPID->limit_y > 0.0f ) {
		if ( output >  pPID->limit_y ) output =  pPID->limit_y;
		if ( output < -pPID->limit_y ) output = -pPID->limit_y;
	}
	
	// if limit ramp is defined
	// not set if negative value
	if( pPID->limit_ramp > 0.0f ) {
		// limit the output rate by ramping the output at a known speed
		float output_rate = (output - pPID->y_prev)/pPID->Ts;
		if (output_rate > pPID->limit_ramp)  output = pPID->y_prev + pPID->limit_ramp*pPID->Ts;
		if (output_rate < -pPID->limit_ramp) output = pPID->y_prev - pPID->limit_ramp*pPID->Ts;
	}

	// saving for the next pass
	pPID->int_prev = Kiout;
	pPID->y_prev   = output;
	pPID->err_prev = error;

  return output;
}

// Reset PID when changing Controller
void PID_Reset(PID_t *pPID) {
	pPID->y_prev   = 0.0f;
	pPID->int_prev = 0.0f;
	pPID->err_prev = 0.0f;
}


//
// Autotune variables
//

int   autotune_cycles_requested;
int   autotune_cycle_count = 0;
int   autotune_samples = 0;
int   autotune_output_state = 1;
float autotune_kp_sum = 0;
float autotune_ki_sum = 0;
float autotune_kd_sum = 0;
float autotune_peak_max = -1e12;
float autotune_peak_min = 1e12;
float autotune_y_min = 0;
float autotune_y_max = 0;
float autotune_target_u = 0;
float	autotune_hysteresis = 0;
float autotune_t_last_switch = 0;
float autotune_timeout_ms = 0;
uint64_t autotune_t_init = 0;

PID_t   *autotune_pPID = 0;

// PID Autotuner Initializer
void PID_Autotune_Init(PID_t *pPID, float u, float y_min, float y_max, float hyst, int maxcycles) {
	// reset internal counter and variables
	autotune_cycle_count = 0;
	autotune_samples = 0;
	autotune_kp_sum = 0;
	autotune_ki_sum = 0;
	autotune_kd_sum = 0;
	// Start ciclying to HIGH
	autotune_output_state = 1;
	// Timeout for 2.0 s
	autotune_timeout_ms = 2000.0f;
	// Reset peaks
	autotune_peak_max = -1e12;
	autotune_peak_min =  1e12;	
	// PID input target
	autotune_target_u = u;
	// PID output for searching HIGH or LOW
	autotune_y_min = y_min;
	autotune_y_max = y_max;
	// u hysteresis
	autotune_hysteresis = hyst;
	// maximun number of cycles
	autotune_cycles_requested = maxcycles;
	// PID for storing K's
	autotune_pPID = pPID;
	// Store initial time
	autotune_t_init = time_get_us();
	autotune_t_last_switch = 0.0f;
}


// Autotune main function
// return values:
//               -1 error: timeout while waiting system output to change
//                0 still tunning PID
//                1 done, PID constants where changed

int PID_Autotune_Compute( float *y, float u )  {
	// actual time in miliseconds
  float now_ms = ((float)(time_get_us() - autotune_t_init))/1000.0f;
	// elapsed time
	float elapsed = now_ms - autotune_t_last_switch;

	// Safety check: Abort if system does not oscillate within timeout
	if ( elapsed > autotune_timeout_ms ) return -1;

	// Track peaks for amplitude calculation
	if ( u > autotune_peak_max) autotune_peak_max = u;
	if ( u < autotune_peak_min) autotune_peak_min = u;

	// Relay Logic with Hysteresis (Non-blocking)
  if ( autotune_output_state && (u > ( autotune_target_u + autotune_hysteresis ) ) )  {
    // Switch to LOW
    autotune_output_state = 0;
  }  else if (!autotune_output_state && (u < (autotune_target_u - autotune_hysteresis))) {
		// Switch to HIGH
    autotune_output_state = 1;

    // One full cycle complete
    autotune_t_last_switch = now_ms;
    float tu = elapsed / 1000.0f;    // Ultimate Period (Tu) in seconds

    // Calculate amplitudes
    float a = (autotune_peak_max - autotune_peak_min) / 2.0f;       // Input Amplitude
    float d = (autotune_y_max - autotune_y_min) / 2.0f;             // Output Amplitude

		// Ultimate Gain (Ku)
    if ( a > 0 ) {
			float ku = (4.0 * d) / (_PI * a);
			
			// Tuning rules: mode no overshot
      float kp_c = 0.2, ti_c = 0.5, td_c = 0.33;

			// Skip the first cycle as it is often erratic (transient)
			if (autotune_cycle_count > 0) {
					float current_kp = kp_c * ku;
					autotune_kp_sum += current_kp;
					autotune_ki_sum += current_kp / (ti_c * tu);
					autotune_kd_sum += current_kp * (td_c * tu);
					autotune_samples++;
			}
		}

		// Reset peaks for next cycle
		autotune_peak_max = -1e12;
		autotune_peak_min =  1e12;
		autotune_cycle_count++;

		// Check if finished
		if (autotune_cycle_count >= autotune_cycles_requested) {
			// end, tune PID and return
			PID_Init(autotune_pPID, autotune_kp_sum / autotune_samples, 
								autotune_ki_sum / autotune_samples, autotune_kd_sum / autotune_samples, 
								200.0e-6f, 0.0f, 0.0f, autotune_y_max);
			return 1;
		}
	}

	// change output to HIGH or LOW
	*y = autotune_output_state ? autotune_y_max : autotune_y_min;
	
	// continue autotuning
	return 0;
}


//
// Path Planning
//

// Variables
float PP_t0;     // Initial time in seconds
float PP_Tacel;  // Aceleration time in seconds
float PP_ttotal; // Total motion time in seconds
float PP_Vmax;   // Velocity for constant velocity in pulses/s

int64_t PP_S0;   // Initial position in pulses
int64_t PP_S1;   // Position after acceleration in pulses
int64_t PP_S2;   // Position after constant velocity in pulses
int64_t PP_S3;   // End position in pulses

// Initializer
// S0   : Initial position in pulses
// Sf   : Final position in pulses
// T0   : when start motion in micro-senconds
// Tacel: aceleration time in mili-seconds
// Vmax : velocity in pulses / s
void PP_Init( int64_t S0_pul, int64_t Sf_pul, uint64_t T0_us, float Vmax_rpm, float Tacel_ms ) {
	// Store variables
	// time in seconds
	PP_t0 = T0_us / 1.0e6f;
	// Initial Position
	PP_S0 = S0_pul;
	// FInal displacement in pulses
	PP_S3 = Sf_pul;
	// Max vel
	PP_Vmax = Vmax_rpm*16384.0f/60.0f;
	
	// delta in pulses
	float Sdelta_pul = (float)(Sf_pul - S0_pul);
	
	// Compute total time
	// tacel in seconds
	float tacel = Tacel_ms/1000.0f;
	// compute total time in seconds
	float ttotal = tacel + fabsf(Sdelta_pul/PP_Vmax);
	// check minimum total time
	if (ttotal < 2.0f*tacel ) ttotal = 2.0f*tacel;
	// compute real maximun profile velocity in float
	PP_Vmax = ((float)Sdelta_pul)/(ttotal-tacel);

	// compute displacement for aceleration time
	float Sacel = PP_Vmax*tacel/2.0f;
	// Store time in seconds
	PP_ttotal = ttotal;
	PP_Tacel  = tacel;
	// Store data 
	PP_S1 = S0_pul + Sacel;
	PP_S2 = PP_S1 + PP_Vmax*(ttotal - 2.0f*tacel);
}

// Copmute displacement for actual time
int64_t PP_Compute( uint64_t T_us, int *done ) {
	// not done yet
	*done = 0;
	
	// elapsed time in seconds
	float t = T_us/1.0e6f - PP_t0;

	// time greater than end time
	if ( t > PP_ttotal ) {
		*done = 1;
		return PP_S3;
	}
	
	// time in desaceleration stage
	if ( t > PP_ttotal - PP_Tacel ) {
		t += PP_Tacel - PP_ttotal;
		return PP_S2 + PP_Vmax * t - 0.5f*(PP_Vmax/PP_Tacel) * t * t;
	}
	
	// time in constant velocity
	if ( t > PP_Tacel ) {
		t -= PP_Tacel;
		return PP_S1 + PP_Vmax * t;
	}
	
	// time in aceleration stage
	if ( t > 0.0f )
		return PP_S0 + 0.5f*(PP_Vmax/PP_Tacel) * t * t;

  // motion doesnt start yet	
	return PP_S0;
}
