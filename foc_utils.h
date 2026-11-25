#ifndef _FOC_INTERFACE_H
#define _FOC_INTERFACE_H

#include <stdint.h>

//
// Check angle in range 0 to 2PI
//

// PI constant
#define  _PI 3.14159265359f
#define _2PI 6.28318530718f
#define _3PI2 4.71238898038f

// return angle in range 0 to 2 PI
float check_rad_range(float angle);

//
// LPF functions
//

// LowPassFilter Data
// FOC algorithm based on Timer Interrupts, 
typedef struct {
    float alpha;
    float y_prev;
} LPF_t;


// Low Pass Filter Initializer
// Tf -> filter time constant
// Ts -> sampling time
void LPF_Init(LPF_t *LPFData, float Tf, float Ts);

// Reset LPF
void LPF_Reset( LPF_t *LPFData );

// Low Pass Filter Computation
float LPF_Compute(LPF_t* LPFData, float y);

//
// PID functions
//

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float feedFW;
	float limit_ramp;
	float limit_y;
	float err_prev;
	float y_prev;
	float int_prev;
	float Ts;
} PID_t;

// PID Initializer
void PID_Init(PID_t *PID_data, float Kp, float Ki, float Kd, float Ts, float feedFW, float lim_sp, float lim_out);

// Compute PID output
float PID_Compute(PID_t *PID_data, float error);

// Reset PID
void PID_Reset(PID_t *PID_data);

//
// PID Autotuner
//

// Autotune PID: y = PID( u )
// u     -> espected system output                   (Velocity: 0 rad/s, Position: 0 pulses)
// y_min -> motor input value for reaching a minimun (Velocity: -I_max,  Position: -W_max)
// y_max -> motor input value for reaching a maximun (Velocity:  I_max,  Position:  W_max)
// hyst  -> maximun expected noise in input          (Velocity: 10 RPM (1.05 rad/s), Position: 1.84e-4 rad (2 LS bit) )
void PID_Autotune_Init(PID_t *pPID, float u, float y_min, float y_max, float hyst, int cycles);

// Autotune step
// y = PID( u )
// u -> PID input  ( Velocity: error in position,         Position: desired position)
// y -> PID output ( Velocity: setpoint for current loop, Position: setpoint for velocity loop)  
// Return value
// -1 -> Error, waiting too much for a cycle end
//  1 -> still doing autotuning
//  0 -> autotuning finished
// Autotune set output values directly into the PID
int PID_Autotune_Compute( float* y, float u );

//
// Path Planning
//

// Init data: 
// S0   : actual motor position in pulses
// Sf   : final motor position in pulses
// T0   : when start motion in us
// Vmax : velocity in RPM
// Tacel: aceleration time in ms
void PP_Init( int64_t S0_pul, int64_t Sf_pul, uint64_t T0_us, float Vmax_rpm, float Tacel_ms);

// Compute displacement
int64_t PP_Compute( uint64_t us, int *done );

#endif

