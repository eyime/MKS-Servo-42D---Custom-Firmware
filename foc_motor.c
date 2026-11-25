#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <n32l40x.h>

#include "foc_motor_od.h"
#include "foc_time.h"
#include "foc_adc.h"
#include "foc_pwm.h"
#include "foc_encoder.h"
#include "foc_flash.h"
#include "foc_comm.h"
#include "foc_oled.h"
#include "foc_utils.h"

#include "foc_motor.h"


//
// Motor Variables
//

// Read only variables - Motor Status and Error

// NodeID
uint8_t  motor_NodeID;

// STATUS BITS:

// bit15:  Nor Used
// bit14:  Not Used
// bit13:  Not Used
// bit12:  Motor ControlMode Status bit4 -> Home already done
// bit11:  Motor ControlMode Status bit3 -> Profile Position controller target position
// bit10:  Motor ControlMode Status bit2 -> Position controller reaches setpoint
// bit09:  Motor ControlMode Status bit1 -> Velocity controller reaches setpoint
// bit08:  Motor ControlMode Status bit0 -> Current controller reaches Setpoint
// bit07:  Motor ControlMode bit4    ControlMode: (most important)
// bit06:  Motor ControlMode bit3    0b00000 -> Current controller
// bit05:  Motor ControlMode bit2    0b00001 -> Velocity controller
// bit04:  Motor ControlMode bit1    0b00010 -> Position controller
// bit03:  Motor ControlMode bit0    0b00011 -> Profile Position controller
// bit02:  Not Used
// bit01:  Motor enabled, 1-Power On, 0-Disable
// bit00:  Error Flag, 1-Error set, 0-No Error
uint16_t motor_status;
// ControlMode bits:
//
// 0b00000 Current Controller
// 0b00001 Velocity Controller
// 0b00010 Position Controller
// 0b00011 Profile Position Controller
// 0b00100 Start a Profile Position motion
// ob00101 Perform a homing using current and Setpoint
// ob00111 Perform a homing using velocity setpoint and microswitch
// 0b10000 Checking Encoder Direction
// 0b10001 Checking Current Direction
// 0b10010 Identifying Motor Properties (R, Lq, Ld)
// 0b10011 Tune PID Current
// 0b10100 Autotune PID Velocity
// 0b10101 Autotune PID Position
//
// ControlMode Status bits
//
// 0b00000 No Msg or Error
// 0bxxxxx errorcode or warning in status

// Motor STATUS:
uint8_t  motor_error;             // motor is in error state
uint8_t  motor_enabled;           // motor enabled
uint8_t  motor_control_mode;      // motor active control-loop 
uint8_t  motor_control_status;    // motor active control-loop status

// The Main Interface for USER, Write Only
uint16_t motor_command;           // motor command to execute

// Constants only readable
float    motor_voltage;           // actual voltage applied to board
float    motor_loop_Ts;           // motor time sampling for control loop

// Read / Write variables - User defined values

// Motor parameters properties
float motor_param_maxV;           // motor maximun allowed voltage en volts (V)
float motor_param_maxI;           // motor maximum allowed current en miliamperes (mA)
float motor_param_maxW;           // motor maximun allowed angular velocity in RPM (RPM)
uint8_t motor_param_pp;           // motor number of poles pairs
float motor_voltage_align;        // voltage for move the motor during alignment
float motor_encoder_PPR;          // constant for magnetic encoder
float motor_param_R;              // motor phase resistance
float motor_param_Ld;             // motor direct phase inductance
float motor_param_Lq;             // motor quadrature phase inductance
// Motor operational values
// Low pass filter
LPF_t LPFPosition;                // Position Low Pass Filter
LPF_t LPFVelocity;                // Velocity Low Pass Filter
LPF_t LPFCurrentD;								// Low Pass Filter for current D
LPF_t LPFCurrentQ;                // Low Pass Filter for current Q
// PID values
PID_t PIDCurrent_Q;               // PID for current, Iq
PID_t PIDCurrent_D;               // PID for current, Id
PID_t PIDVelocity;                // PID for velocity loop
PID_t PIDPosition;                // PID for position Loop

// Incoming value from user 
int64_t  motor_user_desired;      // user desired value (setpoint)

// Read only variables - modified at runtime by program 

// Human readable motor position
int64_t  motor_position_long;     // absolute position in pulses
// Profile position
uint16_t motor_pp_maxvel;         // User defined maximun velocity (RPM) in Profile Position
uint16_t motor_pp_tacel;          // User defined aceleration time (ms) in Profile Position
// Actual velocity
int16_t  motor_velocity_RPM;      // computed filtered velocity in RPM
// Motor operational variables
float    motor_pos_rad;           // position in radians, just one turn (0 - 2 _PI)
float    motor_pos_elect;         // motor electrical angle, in range (0 - 2PI)
float    motor_zero_electrical_angle;// computed zero electrical angle, in range (0 - 2PI)
float    motor_vel_rps;           // filtered velocity in rad/s
// Motor currents
float    Id, Iq;                  // Current after clark transformation
float    Idf, Iqf;                // Filtered current values
// Control Loop Local Variables
uint8_t motor_control_prescaler;  // Reduce control loop frequency (position and velocity)
int64_t motor_setpoint_pos;       // Position setpoint, from user input or position loop
float motor_setpoint_vel;         // Velocity Setpoint, from user input or position loop
float motor_setpoint_curr;        // Current Iq setpoint for FOC loop
float motor_position_err;         // Filtered position error for position loop in radians
float motor_velocity_err;         // Velocity error for velocity loop
float motor_current_err;          // Current Iq error for FOC loop
int   motor_overcurrent_counter;  // Check and count overcurrent in motor
float motor_Uq;                   // Voltage to apply 
float motor_Ud;                   // Voltage to apply

// Output for functions, for user feedback
float motor_output_1;
float motor_output_2;
float motor_output_3;
float motor_output_4;

// default values for set controller job done
float motor_window_position;
float motor_window_velocity;
float motor_window_current;


// Compute electrical angle: ang_elect = N pole pairs * ang_mechanical
// But also check that the result is in the range 0 to 2 PI
float compute_electrical_angle( float angle ) {
	return check_rad_range( motor_param_pp * angle - motor_zero_electrical_angle );
}


// Create object dictionary
void motor_init(Motor_OD_Entry_t *motor_dictionary) {
	// Fill each entry of to object dictionary
	// NodeID
	motor_dictionary[OD_NODEID].ptrData = &motor_NodeID;
	motor_dictionary[OD_NODEID].MemSize = MEM_SZ_8;
	motor_dictionary[OD_NODEID].RdWrAtt = ATTR_RW;
	// Status Word
	motor_dictionary[OD_STATUS_WORD].ptrData = &motor_status;
	motor_dictionary[OD_STATUS_WORD].MemSize = MEM_SZ_16;
	motor_dictionary[OD_STATUS_WORD].RdWrAtt = ATTR_READ;
	// ControlWord
	motor_dictionary[OD_COMMAND_WORD].ptrData = &motor_command;
	motor_dictionary[OD_COMMAND_WORD].MemSize = MEM_SZ_16;
	motor_dictionary[OD_COMMAND_WORD].RdWrAtt = ATTR_WRITE;

	// Motor Parameters ---
	// Actual Applied Board Voltage
	motor_dictionary[OD_VIN_VOLTAGE].ptrData = &motor_voltage;
	motor_dictionary[OD_VIN_VOLTAGE].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_VIN_VOLTAGE].RdWrAtt = ATTR_READ;
	// Maximun motor voltage
	motor_dictionary[OD_MOTOR_MAXVOLT].ptrData = &motor_param_maxV;
	motor_dictionary[OD_MOTOR_MAXVOLT].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_MAXVOLT].RdWrAtt = ATTR_RW;
	// Voltage for Motor Identification
	motor_dictionary[OD_MOTOR_VOLT_ID].ptrData = &motor_voltage_align;
	motor_dictionary[OD_MOTOR_VOLT_ID].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_VOLT_ID].RdWrAtt = ATTR_RW;
	// Maximun motor current
	motor_dictionary[OD_MOTOR_MAXCURR].ptrData = &motor_param_maxI;
	motor_dictionary[OD_MOTOR_MAXCURR].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_MAXCURR].RdWrAtt = ATTR_RW;
	// Maximun motor velocity RPM
	motor_dictionary[OD_MOTOR_MAXVEL].ptrData = &motor_param_maxW;
	motor_dictionary[OD_MOTOR_MAXVEL].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_MAXVEL].RdWrAtt = ATTR_RW;
	// Motor number of pole pairs
	motor_dictionary[OD_MOTOR_PP].ptrData = &motor_param_pp;
	motor_dictionary[OD_MOTOR_PP].MemSize = MEM_SZ_8;
	motor_dictionary[OD_MOTOR_PP].RdWrAtt = ATTR_RW;
	// Motor zero electrical angle
	motor_dictionary[OD_MOTOR_ZERO_ELECT].ptrData = &motor_zero_electrical_angle;
	motor_dictionary[OD_MOTOR_ZERO_ELECT].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_ZERO_ELECT].RdWrAtt = ATTR_READ;
	// Motor Resistance
	motor_dictionary[OD_MOTOR_R].ptrData = &motor_param_R;
	motor_dictionary[OD_MOTOR_R].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_R].RdWrAtt = ATTR_RW;	
	// Motor Inductance Ld
	motor_dictionary[OD_MOTOR_LD].ptrData = &motor_param_Ld;
	motor_dictionary[OD_MOTOR_LD].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_LD].RdWrAtt = ATTR_RW;
	// Motor Inductance Lq
	motor_dictionary[OD_MOTOR_LQ].ptrData = &motor_param_Lq;
	motor_dictionary[OD_MOTOR_LQ].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_LQ].RdWrAtt = ATTR_RW;
	// Motor Encoder PPR
	motor_dictionary[OD_MOTOR_ENC_PPR].ptrData = &motor_encoder_PPR;
	motor_dictionary[OD_MOTOR_ENC_PPR].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_ENC_PPR].RdWrAtt = ATTR_READ;
	// Motor Encoder dir 
	// Set in "foc_encoder.c"
	// int16_t, Read Only
	// User defined setpoint
	motor_dictionary[OD_MOTOR_SETPOINT].ptrData = &motor_user_desired;
	motor_dictionary[OD_MOTOR_SETPOINT].MemSize = MEM_SZ_64;
	motor_dictionary[OD_MOTOR_SETPOINT].RdWrAtt = ATTR_RW;
	// Value for Maximun Velocity in Profile Velocity
	motor_dictionary[OD_MOTOR_PROFILE_VEL].ptrData = &motor_pp_maxvel;
	motor_dictionary[OD_MOTOR_PROFILE_VEL].MemSize = MEM_SZ_16;
	motor_dictionary[OD_MOTOR_PROFILE_VEL].RdWrAtt = ATTR_RW;
	// Value for Maximun Aceleration Time in Profile Velocity
	motor_dictionary[OD_MOTOR_PROFILE_TACEL].ptrData = &motor_pp_tacel;
	motor_dictionary[OD_MOTOR_PROFILE_TACEL].MemSize = MEM_SZ_16;
	motor_dictionary[OD_MOTOR_PROFILE_TACEL].RdWrAtt = ATTR_RW;
	// Actual motor position in pulses
	motor_dictionary[OD_MOTOR_POS_PULSES].ptrData = &motor_position_long;
	motor_dictionary[OD_MOTOR_POS_PULSES].MemSize = MEM_SZ_64;
	motor_dictionary[OD_MOTOR_POS_PULSES].RdWrAtt = ATTR_READ;
	// Actual velocity in RPM
	motor_dictionary[OD_MOTOR_VEL_RPM].ptrData = &motor_velocity_RPM;
	motor_dictionary[OD_MOTOR_VEL_RPM].MemSize = MEM_SZ_16;
	motor_dictionary[OD_MOTOR_VEL_RPM].RdWrAtt = ATTR_READ;
	// Actual Id current in mA,
	motor_dictionary[OD_MOTOR_CURRENT_ID_MA].ptrData = (void*) &Idf;
	motor_dictionary[OD_MOTOR_CURRENT_ID_MA].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_CURRENT_ID_MA].RdWrAtt = ATTR_READ;
	// Actual Iq current in mA
	motor_dictionary[OD_MOTOR_CURRENT_IQ_MA].ptrData = (void*) &Iqf;
	motor_dictionary[OD_MOTOR_CURRENT_IQ_MA].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_CURRENT_IQ_MA].RdWrAtt = ATTR_READ;
	// Motor PhaseA inverted
	// Motor PhaseB inverted
	// Motor PhaseA and PhaseB swapped
	// All are set in "foc_adc.c"
	// Adjust control loop prescaler
	motor_dictionary[OD_MOTOR_CONTROL_LOOP_PS].ptrData = (void*) &motor_control_prescaler;
	motor_dictionary[OD_MOTOR_CONTROL_LOOP_PS].MemSize = MEM_SZ_8;
	motor_dictionary[OD_MOTOR_CONTROL_LOOP_PS].RdWrAtt = ATTR_RW;

	// PID Values

	// Adjust Position PID Kp
	motor_dictionary[OD_MOTOR_PID_POS_KP].ptrData = (void*) &PIDPosition.Kp;
	motor_dictionary[OD_MOTOR_PID_POS_KP].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_POS_KP].RdWrAtt = ATTR_RW;
	// Adjust Position PID Ki
	motor_dictionary[OD_MOTOR_PID_POS_KI].ptrData = (void*) &PIDPosition.Ki;
	motor_dictionary[OD_MOTOR_PID_POS_KI].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_POS_KI].RdWrAtt = ATTR_RW;
	// Adjust Position PID Kd
	motor_dictionary[OD_MOTOR_PID_POS_KD].ptrData = (void*) &PIDPosition.Kd;
	motor_dictionary[OD_MOTOR_PID_POS_KD].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_POS_KD].RdWrAtt = ATTR_RW;
	// Adjust Position PID FeedFordward
	motor_dictionary[OD_MOTOR_PID_POS_FF].ptrData = (void*) &PIDPosition.feedFW;
	motor_dictionary[OD_MOTOR_PID_POS_FF].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_POS_FF].RdWrAtt = ATTR_RW;
	// Adjust Position PID Ramp Limit
	motor_dictionary[OD_MOTOR_PID_POS_RAMP_LIM].ptrData = (void*) &PIDPosition.limit_ramp;
	motor_dictionary[OD_MOTOR_PID_POS_RAMP_LIM].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_POS_RAMP_LIM].RdWrAtt = ATTR_RW;
	// Adjust Position PID Output Limit
	motor_dictionary[OD_MOTOR_PID_POS_LIMIT].ptrData = (void*) &PIDPosition.limit_y;
	motor_dictionary[OD_MOTOR_PID_POS_LIMIT].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_POS_LIMIT].RdWrAtt = ATTR_RW;
	// Adjust Velocity PID Kp
	motor_dictionary[OD_MOTOR_PID_VEL_KP].ptrData = (void*) &PIDVelocity.Kp;
	motor_dictionary[OD_MOTOR_PID_VEL_KP].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_VEL_KP].RdWrAtt = ATTR_RW;
	// Adjust Velocity PID Ki
	motor_dictionary[OD_MOTOR_PID_VEL_KI].ptrData = (void*) &PIDVelocity.Ki;
	motor_dictionary[OD_MOTOR_PID_VEL_KI].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_VEL_KI].RdWrAtt = ATTR_RW;
	// Adjust Velocity PID Kd
	motor_dictionary[OD_MOTOR_PID_VEL_KD].ptrData = (void*) &PIDVelocity.Kd;
	motor_dictionary[OD_MOTOR_PID_VEL_KD].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_VEL_KD].RdWrAtt = ATTR_RW;
	// Adjust Velocity PID FeedFordward
	motor_dictionary[OD_MOTOR_PID_VEL_FF].ptrData = (void*) &PIDVelocity.feedFW;
	motor_dictionary[OD_MOTOR_PID_VEL_FF].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_VEL_FF].RdWrAtt = ATTR_RW;
	// Adjust Velocity PID Ramp Limit
	motor_dictionary[OD_MOTOR_PID_VEL_RAMP_LIM].ptrData = (void*) &PIDVelocity.limit_ramp;
	motor_dictionary[OD_MOTOR_PID_VEL_RAMP_LIM].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_VEL_RAMP_LIM].RdWrAtt = ATTR_RW;
	// Adjust Velocity PID Output Limit
	motor_dictionary[OD_MOTOR_PID_VEL_LIMIT].ptrData = (void*) &PIDVelocity.limit_y;
	motor_dictionary[OD_MOTOR_PID_VEL_LIMIT].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_VEL_LIMIT].RdWrAtt = ATTR_RW;
	// Adjust Current Id PID Kp
	motor_dictionary[OD_MOTOR_PID_ID_KP].ptrData = (void*) &PIDCurrent_D.Kp;
	motor_dictionary[OD_MOTOR_PID_ID_KP].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_ID_KP].RdWrAtt = ATTR_RW;
	// Adjust Current Id PID Ki
	motor_dictionary[OD_MOTOR_PID_ID_KI].ptrData = (void*) &PIDCurrent_D.Ki;
	motor_dictionary[OD_MOTOR_PID_ID_KI].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_ID_KI].RdWrAtt = ATTR_RW;
	// Adjust Current Id PID Kd
	motor_dictionary[OD_MOTOR_PID_ID_KD].ptrData = (void*) &PIDCurrent_D.Kd;
	motor_dictionary[OD_MOTOR_PID_ID_KD].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_ID_KD].RdWrAtt = ATTR_RW;
	// Adjust Current Id PID FeedFordward
	motor_dictionary[OD_MOTOR_PID_ID_FF].ptrData = (void*) &PIDCurrent_D.feedFW;
	motor_dictionary[OD_MOTOR_PID_ID_FF].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_ID_FF].RdWrAtt = ATTR_RW;
	// Adjust Current Id PID Ramp Limit
	motor_dictionary[OD_MOTOR_PID_ID_RAMP_LIM].ptrData = (void*) &PIDCurrent_D.limit_ramp;
	motor_dictionary[OD_MOTOR_PID_ID_RAMP_LIM].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_ID_RAMP_LIM].RdWrAtt = ATTR_RW;
	// Adjust Current Id PID Output Limit
	motor_dictionary[OD_MOTOR_PID_ID_LIMIT].ptrData = (void*) &PIDCurrent_D.limit_y;
	motor_dictionary[OD_MOTOR_PID_ID_LIMIT].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_ID_LIMIT].RdWrAtt = ATTR_RW;
	// Adjust Current Iq PID Kp
	motor_dictionary[OD_MOTOR_PID_IQ_KP].ptrData = (void*) &PIDCurrent_Q.Kp;
	motor_dictionary[OD_MOTOR_PID_IQ_KP].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_IQ_KP].RdWrAtt = ATTR_RW;
	// Adjust Current Iq PID Ki
	motor_dictionary[OD_MOTOR_PID_IQ_KI].ptrData = (void*) &PIDCurrent_Q.Ki;
	motor_dictionary[OD_MOTOR_PID_IQ_KI].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_IQ_KI].RdWrAtt = ATTR_RW;
	// Adjust Current Iq PID Kd
	motor_dictionary[OD_MOTOR_PID_IQ_KD].ptrData = (void*) &PIDCurrent_Q.Kd;
	motor_dictionary[OD_MOTOR_PID_IQ_KD].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_IQ_KD].RdWrAtt = ATTR_RW;
	// Adjust Current Iq PID FeedFordward
	motor_dictionary[OD_MOTOR_PID_IQ_FF].ptrData = (void*) &PIDCurrent_Q.feedFW;
	motor_dictionary[OD_MOTOR_PID_IQ_FF].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_IQ_FF].RdWrAtt = ATTR_RW;
	// Adjust Current Iq PID Ramp Limit
	motor_dictionary[OD_MOTOR_PID_IQ_RAMP_LIM].ptrData = (void*) &PIDCurrent_Q.limit_ramp;
	motor_dictionary[OD_MOTOR_PID_IQ_RAMP_LIM].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_IQ_RAMP_LIM].RdWrAtt = ATTR_RW;
	// Adjust Current Iq PID Output Limit
	motor_dictionary[OD_MOTOR_PID_IQ_LIMIT].ptrData = (void*) &PIDCurrent_Q.limit_y;
	motor_dictionary[OD_MOTOR_PID_IQ_LIMIT].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_PID_IQ_LIMIT].RdWrAtt = ATTR_RW;

	// Filters

	// Alpha in LPF velocity
	motor_dictionary[OD_MOTOR_LPFVEL_ALPHA].ptrData = (void*) &LPFVelocity.alpha;
	motor_dictionary[OD_MOTOR_LPFVEL_ALPHA].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_LPFVEL_ALPHA].RdWrAtt = ATTR_RW;
	// Alpha in LPF Current D
	motor_dictionary[OD_MOTOR_LPFID_ALPHA].ptrData = (void*) &LPFCurrentD.alpha;
	motor_dictionary[OD_MOTOR_LPFID_ALPHA].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_LPFID_ALPHA].RdWrAtt = ATTR_RW;
	// Alpha in LPF Current Q
	motor_dictionary[OD_MOTOR_LPFIQ_ALPHA].ptrData = (void*) &LPFCurrentQ.alpha;
	motor_dictionary[OD_MOTOR_LPFIQ_ALPHA].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_LPFIQ_ALPHA].RdWrAtt = ATTR_RW;
	
	// Controller Window
	
	// Current Window
	motor_dictionary[OD_MOTOR_CURRENT_WINDOW].ptrData = (void*) &motor_window_current;
	motor_dictionary[OD_MOTOR_CURRENT_WINDOW].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_CURRENT_WINDOW].RdWrAtt = ATTR_READ;
	// Velocity Window
	motor_dictionary[OD_MOTOR_VELOCITY_WINDOW].ptrData = (void*) &motor_window_velocity;
	motor_dictionary[OD_MOTOR_VELOCITY_WINDOW].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_VELOCITY_WINDOW].RdWrAtt = ATTR_READ;
	// Position Window
	motor_dictionary[OD_MOTOR_POSITION_WINDOW].ptrData = (void*) &motor_window_position;
	motor_dictionary[OD_MOTOR_POSITION_WINDOW].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_POSITION_WINDOW].RdWrAtt = ATTR_READ;

	// Function output

	// First output
	motor_dictionary[OD_MOTOR_OUPUT_1].ptrData = (void*) &motor_output_1;
	motor_dictionary[OD_MOTOR_OUPUT_1].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_OUPUT_1].RdWrAtt = ATTR_READ;
	// Second output
	motor_dictionary[OD_MOTOR_OUPUT_2].ptrData = (void*) &motor_output_2;
	motor_dictionary[OD_MOTOR_OUPUT_2].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_OUPUT_2].RdWrAtt = ATTR_READ;
	// Third output
	motor_dictionary[OD_MOTOR_OUPUT_3].ptrData = (void*) &motor_output_3;
	motor_dictionary[OD_MOTOR_OUPUT_3].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_OUPUT_3].RdWrAtt = ATTR_READ;
	// Fourth output
	motor_dictionary[OD_MOTOR_OUPUT_4].ptrData = (void*) &motor_output_4;
	motor_dictionary[OD_MOTOR_OUPUT_4].MemSize = MEM_SZ_FLOAT;
	motor_dictionary[OD_MOTOR_OUPUT_4].RdWrAtt = ATTR_READ;
}


// Check restored data from flash for avoiding invalid data
void motor_check_data() {
	//
	// Default runtime values
	//
	
	// status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = CURRENT_CONTROLLER;
	motor_control_status = 0;
	motor_status = 0;
	// No motor command
	motor_command = 0;
	// user desired is 0
	motor_user_desired = 0;
	// Actual posiition is zero
	motor_position_long = 0;
	motor_position_err = 0;
	motor_setpoint_pos = 0;
	// Actual velocity is zero
	motor_velocity_RPM = 0;
	motor_vel_rps = 0;
	motor_velocity_err = 0;
	motor_setpoint_vel = 0;
	// Actual current is zero
	Idf = 0;
	Iqf = 0;
	motor_current_err = 0;
	motor_setpoint_curr = 0;
	motor_overcurrent_counter = 0;
	// Default prescaler for position and velocity is 2
	motor_control_prescaler = 2;
	// Default propile position velocity is 1000 RPM
	motor_pp_maxvel = 1000;
	// Default profile position acceleration time is 500 ms
	motor_pp_tacel = 500;
	// default control loop time sampling 200 us
	motor_loop_Ts = 200;

	// Get actual voltage from ADC 
	// Actual voltage applied to PCB
	motor_voltage = adc_get_Vin();
	
	// controller windows
	// 20 mA in current controller
	motor_window_current = 0.020;
	// 10 RPM in velocity controller
	motor_window_velocity = 10.0f*_2PI/60.0f;
	// 100 pulese in position controller
	motor_window_position = 100.0f;

	// outputs to zero
	motor_output_1 = 0.0f;
	motor_output_2 = 0.0f;
	motor_output_3 = 0.0f;
	motor_output_4 = 0.0f;
	
	//
	// Check OD values
	//
	
	//
	// NodeID
	if (motor_NodeID == 0 || motor_NodeID > 15) motor_NodeID = 1;
	// 
	// Motor Parameters
	// Motor maximun voltage not set is 1.0 Volts
	if ( motor_param_maxV < 0.0f || motor_param_maxV > 24.0f ) motor_param_maxV = 1.0f;
	// Motor maximun Angular Velocity not set is 10000.0 RPM
	if ( motor_param_maxW < 0.0f || motor_param_maxW > 10000.0f ) motor_param_maxW = 10000.0f;
	// Motor maximun current not set is 1 Amp
	if ( motor_param_maxI < 0.0f || motor_param_maxI > 4000.0f ) motor_param_maxI = 1000.0f;
	// motor pole pairs limited to 100 (arbritary value)
	if ( motor_param_pp > 100 ) motor_param_pp = 100;
	// motor coil resistance limited to 50 ohm (arbritary)
	if ( motor_param_R < 0.0f || motor_param_R > 50.0f ) motor_param_R = 50.0f;
	// motor coil inductace lmited to 100 mH (arbritary)
	if ( motor_param_Ld < 0.0f || motor_param_Ld > 0.100f ) motor_param_Ld = 0.100f;
	if ( motor_param_Lq < 0.0f || motor_param_Lq > 0.100f ) motor_param_Lq = 0.100f;
	// MKS Servo uses an encoder with 16384.0f PPR
	if ( motor_encoder_PPR != 16384.0f) motor_encoder_PPR = 16384.0f;
	// motor electrical zero
	if (motor_zero_electrical_angle < 0.0f || motor_zero_electrical_angle > _2PI ) motor_zero_electrical_angle = 0.0f;
	//
	// Controller Parameters
	// CurrentD Controller Parameters
	if ( PIDCurrent_D.Kp < 0.0f || PIDCurrent_D.Kp > 1.0e6) PIDCurrent_D.Kp = 1.0f;
	if ( PIDCurrent_D.Ki < 0.0f || PIDCurrent_D.Ki > 1.0e6) PIDCurrent_D.Ki = 0.0f;
	if ( PIDCurrent_D.Kd < 0.0f || PIDCurrent_D.Kd > 1.0e6) PIDCurrent_D.Kd = 0.0f;
	if ( PIDCurrent_D.feedFW < 0.0f || PIDCurrent_D.feedFW > 10.0f ) PIDCurrent_D.feedFW = 0.0;
	if ( PIDCurrent_D.limit_ramp < 0.0f || PIDCurrent_D.limit_ramp > 10.0f ) PIDCurrent_D.limit_ramp = 0.0;
	if ( PIDCurrent_D.limit_y < 0.0f || PIDCurrent_D.limit_y > motor_param_maxV ) PIDCurrent_D.limit_y = motor_param_maxV;
	// CurrentQ Controller Parameters
	if ( PIDCurrent_Q.Kp < 0.0f || PIDCurrent_Q.Kp > 1.0e6) PIDCurrent_Q.Kp = 1.0f;
	if ( PIDCurrent_Q.Ki < 0.0f || PIDCurrent_Q.Ki > 1.0e6) PIDCurrent_Q.Ki = 0.0f;
	if ( PIDCurrent_Q.Kd < 0.0f || PIDCurrent_Q.Kd > 1.0e6) PIDCurrent_Q.Kd = 0.0f;
	if ( PIDCurrent_Q.feedFW < 0.0f || PIDCurrent_Q.feedFW > 1.0f ) PIDCurrent_Q.feedFW = 0.0;
	if ( PIDCurrent_Q.limit_ramp < 0.0f || PIDCurrent_Q.limit_ramp > 10.0f ) PIDCurrent_Q.limit_ramp = 0.0;
	if ( PIDCurrent_Q.limit_y < 0.0f || PIDCurrent_Q.limit_y > motor_param_maxV ) PIDCurrent_Q.limit_y = motor_param_maxV;
	// Velocity Controller Parameters
	if ( PIDVelocity.Kp < 0.0f || PIDVelocity.Kp > 1.0e6) PIDVelocity.Kp = 1.0f;
	if ( PIDVelocity.Ki < 0.0f || PIDVelocity.Ki > 1.0e6) PIDVelocity.Ki = 0.0f;
	if ( PIDVelocity.Kd < 0.0f || PIDVelocity.Kd > 1.0e6) PIDVelocity.Kd = 0.0f;
	if ( PIDVelocity.feedFW < 0.0f || PIDVelocity.feedFW > 1.0f ) PIDVelocity.feedFW = 0.0;
	if ( PIDVelocity.limit_ramp < 0.0f || PIDVelocity.limit_ramp > 10.0f ) PIDVelocity.limit_ramp = 0.0;
	if ( PIDVelocity.limit_y < 0.0f || PIDVelocity.limit_y > motor_param_maxI/1000.0f ) PIDVelocity.limit_y = motor_param_maxI/1000.0f;
	// Position Controller Parameters
	if ( PIDPosition.Kp < 0.0f || PIDPosition.Kp > 1.0e6) PIDPosition.Kp = 1.0f;
	if ( PIDPosition.Ki < 0.0f || PIDPosition.Ki > 1.0e6) PIDPosition.Ki = 0.0f;
	if ( PIDPosition.Kd < 0.0f || PIDPosition.Kd > 1.0e6) PIDPosition.Kd = 0.0f;
	if ( PIDPosition.feedFW < 0.0f || PIDPosition.feedFW > 1.0f ) PIDPosition.feedFW = 0.0;
	if ( PIDPosition.limit_ramp < 0.0f || PIDPosition.limit_ramp > 10.0f ) PIDPosition.limit_ramp = 0.0;
	if ( PIDPosition.limit_y < 0.0f || PIDPosition.limit_y > motor_param_maxW*_2PI/60.0f) PIDPosition.limit_y = motor_param_maxW*_2PI/60.0f;
	// Fill PID missing data, sampling time
	PIDCurrent_D.Ts = motor_loop_Ts/1.0e6f;
	PIDCurrent_Q.Ts = motor_loop_Ts/1.0e6f;
	PIDVelocity.Ts  = motor_loop_Ts/1.0e6f;
	PIDPosition.Ts  = motor_loop_Ts/1.0e6f;
	// Filter Parameters
	if ( LPFVelocity.alpha < 0.0f || LPFVelocity.alpha > 1.0f ) LPFVelocity.alpha = 0.0;
	if ( LPFCurrentD.alpha < 0.0f || LPFCurrentD.alpha > 1.0f ) LPFCurrentD.alpha = 0.0;
	if ( LPFCurrentQ.alpha < 0.0f || LPFCurrentQ.alpha > 1.0f ) LPFCurrentQ.alpha = 0.0;
	
	// Reset PID
	PID_Reset( &PIDCurrent_D );
	PID_Reset( &PIDCurrent_Q );
	PID_Reset( &PIDVelocity );
	PID_Reset( &PIDPosition );
	// Reset LPF
	LPF_Reset( &LPFCurrentD );
	LPF_Reset( &LPFCurrentQ );
	LPF_Reset( &LPFVelocity );
	LPF_Reset( &LPFPosition );
}


// Load Data From Flash
void motor_load_flash_data() {
	// copy data stored in flash to local variables
	flash_read_data();
	// check data loaded from flash
	motor_check_data();
	adc_check_data();
	pwm_check_data();
	encoder_check_data();
}

// update motor status
void motor_update_status() {
	// reset status, set error flag, bit0
	motor_status = motor_error & 1;
	// enabled flag, bit1
	motor_status |= ( (motor_enabled & 1) << 1);
	// Control mode, bits3 to bit7 
	motor_status |= ( (motor_control_mode & 0x1F) << 3);
	// Control mode status, bits8 to 12
	motor_status |= ( (motor_control_status & 0x1F) << 8);
}

// Error during Execution
void set_motor_error( uint8_t ErrorFlag ) {
	// Stop motor
	pwm_set_channels(0.0f, 0.0f, 0.0f, 0.0f);
	// Set error flag
	motor_error = 1;
	// No enabled
	motor_enabled = 0;
	// controlmode not modified
	// controlmode status set to flag
	motor_control_status = ErrorFlag;
	// update status
	motor_update_status();
}

// UPDATE required input values before each PID loop
void motor_update_inputs() {
	// 
	// Motor Vin voltage
	//	
	motor_voltage = adc_get_Vin();

	//
	// Update Position from encoder
	//	
	// Update encoder counters 
	encoder_update_counters();
	// Get delta position in pulses
	int delta = encoder_get_pulses_delta();
	// absolute position
	motor_position_long += delta;
	// Get one-turn position in radians (0 to 2PI)
	motor_pos_rad = encoder_get_angle_rad();
	// compute one-turn electrical angle
	motor_pos_elect = compute_electrical_angle( motor_pos_rad );

	//
	// Update motor current from ADC
	//
	// Get I direct and I quadrature currents
	adc_get_DQ_current(&Id, &Iq, motor_pos_elect);
	// Filter Iq and Id, in Amperes
	Idf = LPF_Compute(&LPFCurrentD, Id);
	Iqf = LPF_Compute(&LPFCurrentQ, Iq);
	
	// 
	// Security check
	//
	// Overcurrent detection
	if ( fabsf(Iqf) > 1.5f*motor_param_maxI/1000.0f ) {
		// No more than 100 ms at max current
		motor_overcurrent_counter++;
		if ( motor_overcurrent_counter > 1000 ) {
			motor_error = 1;
			motor_enabled = 0;
			motor_control_status = MOT_ERROR_OVERCURRENT;
		}
	} else {
		motor_overcurrent_counter--;
		if (motor_overcurrent_counter < 0) motor_overcurrent_counter = 0;
	}
	
	//
	// Update status
	//
	
	motor_update_status();
	// turn off/on led
	if (motor_error) {
		GPIO_ResetBits( GPIOD, GPIO_PIN_0 );
	} else {
		GPIO_SetBits( GPIOD, GPIO_PIN_0 );
	}
	
	//
	// Compute velocity from encoder pulses and use a low pass filter
	//

	// filter motor velocity in rad/s
	motor_vel_rps = LPF_Compute( &LPFVelocity, _2PI * ((float)delta) / (motor_encoder_PPR * 100.0e-6f) );
	// filtered motor velocity in RPM a human redable value
	motor_velocity_RPM = (int16_t) (motor_vel_rps * 30.0f / _PI);
}


//
// PID for each motor control mode
//

// Position Control Loop
void motor_position_loop() {
	// compute error in pulses
	motor_position_err = ((float)(motor_setpoint_pos - motor_position_long));
	// update control status
	if ( fabsf(motor_position_err) < motor_window_position )
		motor_control_status |= 0b00100; else motor_control_status &= 0b01011;
	// velocity setpoint from position error
	motor_setpoint_vel = PID_Compute( &PIDPosition, motor_position_err );
}

// Velocity Control Loop
void motor_velocity_loop() {
	// Compute error variable for velocity loop ( rad / s )
	motor_velocity_err = motor_setpoint_vel - motor_vel_rps;
	// update control status
	if ( fabsf(motor_velocity_err) < motor_window_velocity )
		motor_control_status |= 0b00010; else motor_control_status &= 0b11101;
	// Current setpoint using velocity error
	motor_setpoint_curr = PID_Compute( &PIDVelocity, motor_velocity_err );
}

// current control loop
void motor_current_loop() {
	// Compute error variable for current loop
	motor_current_err = motor_setpoint_curr - Iqf;
	// update control status
	if ( fabsf(motor_current_err) < motor_window_current ) 
		motor_control_status |= 0b00001; else motor_control_status &= 0b11110;
	// compute voltage output from error
	// Setpoint in Iq is motor_setpoint_curr
	// Setpoint in Id is zero
	motor_Uq = PID_Compute( &PIDCurrent_Q, motor_current_err );
	motor_Ud = PID_Compute( &PIDCurrent_D, -Idf );
	// Lag compensation ....
	//motor_Uq += Idf * motor_vel_rps * motor_param_pp * motor_param_Ld;
	//motor_Ud -= motor_setpoint_curr * motor_vel_rps * motor_param_pp * motor_param_Lq;
	// If enabled, then apply voltage to coils
	if ( motor_enabled ) {
		// set the voltages in hardware, limiting values applied
		pwm_set_from_Ud_Uq( motor_Ud, motor_Uq, motor_pos_elect );
	} else {
		// no voltage
		pwm_set_from_Ud_Uq( 0.0f, 0.0f, motor_pos_elect );
	}
}


//
// The main controller function
//

// --------------------------------------------------------
// The MAIN Loop ------------------------------------------
// --------------------------------------------------------
void motor_control_loop() {
	// always update inputs
	motor_update_inputs();
		
	// For each controller
	switch (motor_control_mode) {
	int done;
		case PROF_POS_CONTROLLER:
			// compute position setpoint from path planning
			motor_setpoint_pos = PP_Compute( time_get_us(), &done );
			// update control status
			if (done) motor_control_status |= 0b01000; else motor_control_status &= 0b01111;
			// position conroller
			motor_position_loop();
			// velocity controller
			motor_velocity_loop();
			// current controller
			motor_current_loop();
			break;
		case POSITION_CONTROLLER:
			// setpoint from user
			motor_setpoint_pos = motor_user_desired;
			// position controller
			motor_position_loop();
			// velocity controller
			motor_velocity_loop();
			// current controller
			motor_current_loop();
			break;
		case VELOCITY_CONTROLLER:
			// limit user input to maximun allowed velocity
			if ( motor_user_desired >  motor_param_maxW ) motor_user_desired =  motor_param_maxW;
			if ( motor_user_desired < -motor_param_maxW ) motor_user_desired = -motor_param_maxW;
			// Update setpoint to rad/s
			motor_setpoint_vel = ((float)motor_user_desired) * _2PI/60.0f;
			// Velocity controller
			motor_velocity_loop();
			// Current controller
			motor_current_loop();
			break;
		case CURRENT_CONTROLLER:
			// limit user input to maximun allowed current
			if ( motor_user_desired >  motor_param_maxI ) motor_user_desired =  motor_param_maxI;
			if ( motor_user_desired < -motor_param_maxI ) motor_user_desired = -motor_param_maxI;
			// user redable input in mA but setpoint in Amperes
			motor_setpoint_curr = ((float)motor_user_desired) / 1000.0f;
			// current controller
			motor_current_loop();
			break;
		case HOME_CURRENT:
			// using current controller with user setpoint
			// timeout at 120 seconds
			// limit user input to half allowed current
			if ( motor_user_desired >  motor_param_maxI/2.0f ) motor_user_desired =  motor_param_maxI/2.0f;
			if ( motor_user_desired < -motor_param_maxI/2.0f ) motor_user_desired = -motor_param_maxI/2.0f;
			// User redable input in mA but setpoint in Amperes
			motor_setpoint_curr = ((float)motor_user_desired) / 1000.0f;
			// current controller
			motor_current_loop();
			// check when reaching max allowed current
			if ( fabsf(Iqf) >= fabsf(motor_setpoint_curr) ) {
				// job done
				motor_control_status |= 0b10000;
				// reset encoder
				encoder_get_pulses_delta();
				motor_position_long = 0;
				// change to position controller
				motor_control_mode = POSITION_CONTROLLER;
				// No setpoint
				motor_user_desired = 0;
			} else {
				// job not done
				motor_control_status &= 0b01111;
			}
			// check timeout
			motor_output_2 = ((float) time_get_us()) / 1000.0f;
			if ( motor_output_2 - motor_output_1 > 120000.0f ) {
				// error: timeout
				set_motor_error( MOT_ERROR_TIMEOUT );
			}
			break;
		case HOME_VELOCITY:
			// using velocity controller with user setpoint
			// timeout at 120 seconds
			// limit user input to half allowed velocity
			if ( motor_user_desired >  motor_param_maxW/2.0f ) motor_user_desired =  motor_param_maxW/2.0f;
			if ( motor_user_desired < -motor_param_maxW/2.0f ) motor_user_desired = -motor_param_maxW/2.0f;
			// Update motor setpoint to rad/s
			motor_setpoint_vel = ((float)motor_user_desired) * _2PI/60.0f;
			// velocity controller
			motor_velocity_loop();
			// current controller
			motor_current_loop();
			// check if switch is ON (PA12)
			if ( GPIO_ReadInputDataBit(GPIOA, GPIO_PIN_12) == Bit_SET ) {
				// job done
				motor_control_status |= 0b10000;
				// reset encoder
				encoder_get_pulses_delta();
				motor_position_long = 0;
				// change to position controller
				motor_control_mode = POSITION_CONTROLLER;
				// No setpoint
				motor_user_desired = 0;
			} else {
				// job not done
				motor_control_status &= 0b01111;
			};
			// check timeout
			motor_output_2 = ((float) time_get_us()) / 1000.0f;
			if ( motor_output_2 - motor_output_1 > 120000.0f ) {
				// error: timeout
				set_motor_error( MOT_ERROR_TIMEOUT );
			}
			break;
	}
}


//
// These functions are for verifying motor connections before running control loops
//


// Determine current direction when applied a positive voltage
// Stepper motor
// Function output:
// output1: phA when appliyng voltage to coil A 
// output2: phB when appliyng voltage to coil A
// output3: phA when appliyng voltage to coil B
// output4: phB when appliyng voltage to coil B
void motor_current_align() {
	int i;
	float phA, phB;
	
	// Update motor status
	// no errors
	motor_error = 0;
	// enabled: powered and running
	motor_enabled = 1;
	// control mode: current align
	motor_control_mode = CURRENT_ALIGN;
	// motor_control_status
	// bit3 and bit4 not used
	// bit2: 1 if PhaseB is inverted
	// bit1: 1 if PhaseA is inverted
	// bit0: 1 if phases are swapped
	motor_control_status = 0;
	// update status
	motor_update_status();
	
	// DEBUG:
	char mystring[30];
	oled_clearAll();

	// Outputs to zero
	motor_output_1 = motor_output_2 = motor_output_3 = motor_output_4 = 0.0f;

	// Security check
	if ( motor_voltage_align > motor_param_maxV) motor_voltage_align = motor_param_maxV/2.0f;

	// Step 1: set phaseA Up and phaseB down

	// Set phase A to voltage for aligning
	// ramp 300ms
	for(i=0; i < 100; i++){
		pwm_set_from_Ua_Ub(motor_voltage_align/100.0*((float)i), 0);
		time_delay_ms(3);
	}
	time_delay_ms(500);
	// Read last adc conversion, which was triggered by PWM
	adc_get_phase_current( &phA, &phB );
	// set pwm to zero
	pwm_set_from_Ua_Ub( 0.0f, 0.0f );
	// wait to settle
	time_delay_ms(500);
	
	// Function output
	motor_output_1 = phA;
	motor_output_2 = phB;
	
	// DEBUG:
	sprintf(mystring, "PhA: %f", phA);
	oled_write(0, mystring);
	sprintf(mystring, "PhB: %f", phB);
	oled_write(1, mystring);
	
	// Error if less than 1 mA detected
	if ( fabsf(phA) < 0.001f && fabsf(phB) < 0.001f ) {
		set_motor_error( MOT_ERROR_CURRALIGN_NOPHA );
		
		// DEBUG:
		oled_write(2, "ERROR: i < 1mA");
		
		return;
	}
	// If detected current on PhaseB is greater than PhaseA then swap phases
	if ( fabsf(phB) > fabsf(phA) ) {
		// motor control status to swapped lines
		motor_control_status |= 1;
		// swapped at adc
		adc_set_swapped_phases();
		// but the local variables must be swapped too
		float temp = phB;
		phB = phA;
		phA = temp;
		
		// DEBUG:
		oled_write(2, "Swapped!!");
	}
	
	// Check if phA is inverted, positive voltage -> expected positive current
	if ( phA < 0.0f ) {
		// set phase A as inverted
		motor_control_status |= 1 << 1;
		// invert phases A in adc
		adc_set_inverted_phaseA();
		
		// DEBUG:
		oled_write(2, "Swapped, phA inv");
	}
	
	// Step 2: set phase A down and phase B up	

	// set phase B active and phases A down
	// ramp 300ms
	for(i=0; i < 100; i++){
		pwm_set_from_Ua_Ub(0, motor_voltage_align/100.0*((float)i));
		time_delay_ms(3);
	}
	time_delay_ms(500);
	// Read adc
	adc_get_phase_current( &phA, &phB );
	// set PWM to zero
	pwm_set_from_Ua_Ub( 0.0f, 0.0f );
	// wait to settle
	time_delay_ms( 500 );

	// Function output
	motor_output_3 = phA;
	motor_output_4 = phB;
	
	// DEBUG:
	sprintf(mystring, "PhB: %f", phB);
	oled_write(3, mystring);

	// Error if no current detected
	if ( fabsf(phB) < 0.001f ) {
		set_motor_error( MOT_ERROR_CURRALIGN_NOPHB );
		
		// DEBUG:
		oled_write(4, "Error: i < 1mA");
		
		return;
	}
	// check if inverted
	if (phB < 0.0f) {
		// set phase B as inverted
		motor_control_status |= 1 << 2;
		adc_set_inverted_phaseB();

		// DEBUG:
		oled_write(4, "PhB inverted");
	}

	// Update motor status
	// disabled: ok, done
	motor_enabled = 0;
	// update
	motor_update_status();
}


// Determine encoder direction when motor turn in positive electrical motion
// It also calculate: 
// output1: computed motor pole pairs
// output2: zero electrical angle
// output3: 0.0 (OK), -1.0 (repeat again)
void motor_encoder_align_and_zero() {
	int i, end_pulses, pulses; 
	float voltage, angle;
	
	// update status
	motor_enabled = 1;
	motor_error = 0;
	motor_control_mode = ENCODER_ALIGN;
	motor_control_status = 0;
	motor_update_status();
	// Outputs to zero
	motor_output_1 = motor_output_2 = motor_output_3 = motor_output_4 = 0.0f;
	
	// voltage for alignment
	voltage = fabs( motor_voltage_align );
	
	// security check
	// voltage no greather than max alowed value by motor
	// voltage no greather than actual applied voltage to PCB
	if ( (voltage > motor_param_maxV ) || (voltage > motor_voltage ) ) {
		// Error, wrong voltage
		set_motor_error( MOT_ERROR_WRONG_CONFIG );
		return;
	};

	// DEBUG: Clear display
	char mystring[30];
	oled_clearAll();

	//
	// align encoder: move motor in positive direction, check and correct encoder direction
	//
	pwm_set_from_Ud_Uq(-voltage, 0.0f, 0.0f);
	time_delay_ms( 100 );
	// reset counter
	encoder_update_counters();
	encoder_get_pulses_delta();
	pulses = 0;
	for (i=0; i<=500; i++) {
		// update position
		encoder_update_counters();
		pulses += encoder_get_pulses_delta();
		// position
		angle = ((float)i)*_2PI/500.0f;
		// power motor
		pwm_set_from_Ud_Uq(-voltage, 0.0f, angle);
		// delay
		time_delay_ms( 2 );
	}	
	time_delay_ms(100);
	
	// set encoder direction
	// positive rotation expects postive pulses
	if ( pulses < 0) {
		encoder_set_inverted_dir();
		// DEBUG: oled
		oled_write(0, "ENC: Inverted");
	} else {
		// DEBUG: oled
		oled_write(0, "ENC: Ok");
	}

	// 
	// Compute pole-pairs and zero electrical angle
	//
	
	// Move motor one mechanical revolution, stopping at each zero electrical angle
	// the sum each mechanical position, so the zero electrical angle is a mean value 
	// with reduced impact of friction and noise

	// reset zero electrical angle and pole-pairs
	motor_zero_electrical_angle = 0.0f;
	motor_param_pp = 0;
	// update position and clear delta, counts to 0
	encoder_update_counters();
	encoder_get_pulses_delta();
	pulses = 0;
	// final angle, one revolution in pulses minus 2 degrees
	// start angle is the actual position (electrical angle to zero)
	end_pulses = motor_encoder_PPR - motor_encoder_PPR / 180.0f;
	do {
		// update position
		motor_zero_electrical_angle += encoder_get_angle_rad();
		// update counter
		motor_param_pp++;

		// DEBUG: display values for debugging
		sprintf(mystring, "count: %f", (float)motor_param_pp);
		oled_write( 1, mystring );
		sprintf(mystring, "zero: %f", motor_zero_electrical_angle);
		oled_write( 2, mystring );
		
		// move one positive electrical revolution
		for (i=0; i<=500; i++) {
			// position
			angle = ((float)i)*_2PI/500.0f;
			// power motor
			pwm_set_from_Ud_Uq(-voltage, 0.0f, angle);
			// delay
			time_delay_ms( 1 );
		}
		time_delay_ms( 100 );

		// update position
		encoder_update_counters();
		pulses += encoder_get_pulses_delta();
		// checking last position
	} while ( pulses < end_pulses);

	// last position not computed, is near (-2 degrees) starting position
	// zero electrical angle, already multiplied by Pole-Pairs
	motor_zero_electrical_angle = fmodf( motor_zero_electrical_angle, _2PI );
	
	// DEBUG: display values
	sprintf(mystring, "P.P: %f", (float)motor_param_pp);
	oled_write( 1, mystring );
	sprintf(mystring, "zero: %f", motor_zero_electrical_angle);
	oled_write( 3, mystring );

	// power off
	pwm_set_from_Ud_Uq( 0.0f, 0.0f, 0.0f );
	
	// Check Q-Axis
	// A positive voltage in Q-axis should turn motor in positive direction
	encoder_update_counters();
	encoder_get_pulses_delta();
	pulses = 0;
	for (i=0; i<100; i++) {
		encoder_update_counters();
		pulses += encoder_get_pulses_delta();
		angle = compute_electrical_angle( encoder_get_angle_rad() );
		pwm_set_from_Ud_Uq( 0.0f, voltage, angle );
		time_delay_ms( 1 );
	}
	
	// Error if pulses are not positive
	if (pulses < 0 ) {
		// DEBUG:
		oled_write(4, "Q-axis: ERR");

		// repeat again
		motor_output_3 = -1.0f;
	} else {
		// DEBUG:
		oled_write(4, "Q-axis: Ok");
	}

	// update status
	motor_enabled = 0;
	motor_update_status();

	// update outputs
	motor_output_1 = (float) motor_param_pp;
	motor_output_2 = motor_zero_electrical_angle;
}

// Modeling the Motor
// Electrical modeling of a motor as a L-R circuit
// Function outputs:
// output1: motor resistance
// output2: motor zero electrical angle
// output3: motor inductance Lq
// output4: motor inductance Ld
void motor_identify_R_L( float factor ) {
	//
	// Start Resistance measurement
	//
	
	// Local variables
	float voltage;
	float elect_angle;
	float I0_d, I0_q;
	float IR_d, IR_q;
	float sumId, sumIq;
	float IR;
	int i;
	
	// Update motor status
	// no errors
	motor_error = 0;
	// enabled: powered and running
	motor_enabled = 1;
	// control mode: current align
	motor_control_mode = MOTOR_IDENTIFY_RL;
	motor_control_status = 0;
	// update status
	motor_update_status();
	
	// Outputs to zero
	motor_output_1 = motor_output_2 = motor_output_3 = motor_output_4 = 0.0f;

	// voltage 
	voltage = fabsf( motor_voltage_align );

	// DEBUG: oled
	oled_clearAll();
	char mystring[30];

	// get actual electrical angle
	encoder_update_counters();
	elect_angle = compute_electrical_angle( encoder_get_angle_rad() );
	
	// measure an initial reference current
	pwm_set_from_Ud_Uq(0.0, 0.0, elect_angle);
	// wait to settle
	time_delay_ms( 100 );
	// Get initial or zero D-Q currents
	adc_get_DQ_current(&I0_d, &I0_q, elect_angle);
	
	// Ramp and hold the voltage to measure resistance
	// 300 ms of ramping
	for (i=0; i < 100; i++){
		pwm_set_from_Ud_Uq( ((float)i)*voltage/100.0f, 0.0f, elect_angle );
		// delay 3ms
		time_delay_ms( 3 );
	}
	// wait to settle
	time_delay_ms( 100 );
	// get actual electrical angle
	encoder_update_counters();
	elect_angle = compute_electrical_angle( encoder_get_angle_rad() );
	// Get actual D-Q currents for computing R using V = I R
	adc_get_DQ_current(&IR_d, &IR_q, elect_angle);

	// Zero again
	pwm_set_from_Ud_Uq(0.0, 0.0, elect_angle);
	
	// Error if IR is inverse
	// call motor_current_align() first ..
	if ( IR_d < 0.0f ) {
		set_motor_error( MOT_ERROR_IDENTIFY_I_NEG );
		
		// DEBUG: oled
		oled_write(0, "Error, Id < 0");
		sprintf( mystring, "Id: %f", IR_d);
		oled_write(1, mystring);
				
		return;
	}
	// Error if measured I is lower than initial I
	// current too slow, increase voltage
	if ( IR_d < I0_d ) {
		set_motor_error( MOT_ERROR_IDENTIFY_I_LOWER );

		// DEBUG: oled
		oled_write(0, "Error, Id_f < Id_0");
		sprintf(mystring, "Id_0: %f", I0_d);
		oled_write(1, mystring);
		sprintf(mystring, "Id_f: %f", IR_d);
		oled_write(2, mystring);		

		return;
	}
	
	// Detected current
	IR = IR_d - I0_d;
	// Error if IR is too slow
	// 10 mA ??
	if ( IR < 0.010f ) {
		// Error flag
		set_motor_error( MOT_ERROR_IDENTIFY_R );

		// DEBUG: oled
		oled_write(0, "ERR: IR < 10mA");
		sprintf(mystring, "IR: %f", IR);
		oled_write(1, mystring);

		return;
	}

	// Compute Resistance
	motor_output_1 = 2.0f*voltage / ( factor * IR );
	// save motor resistance if not set
	if ( motor_param_R <= 0.0f) motor_param_R = motor_output_1;
	
	// DEBUG: oled
	sprintf(mystring, "R: %f", motor_output_1);
	oled_write(0, mystring);
	
	//
	// Start inductance measurement
	//
	
	// time values in micros
	uint64_t t0 = 0;
	uint64_t t1 = 0;
	
	float Ltemp = 0;
	float Ld = 0;
	float Lq = 0;
	float d_electrical_angle = 0;
	float delta;
	float dt, timeconstant;
	float inductanced;

	int iterations = 40;    // how often the algorithm gets repeated.
	int cycles = 3;         // averaged measurements for each iteration
	int risetime_us = 200;  // initially short for worst case scenario with low inductance
	int settle_us = 100000; // initially long for worst case scenario with high inductance
	
	int axis, j;

	// Pre-rotate the angle to the q-axis (only useful with sensor, else no harm in doing it)
	elect_angle = check_rad_range( elect_angle - 0.5f * _PI);
	
	// axis: 0 -> direct, 1 -> quadrature
	for (axis = 0; axis < 2; axis++)  {
		
		// iterate many times
		for (i = 0; i < iterations; i++) {
			
			// current_electric_angle = i * _2PI / iterations; // <-- Do a sweep of the inductance. Use eg. for graphing
			inductanced = 0.0f;
			sumId = 0.0f; sumIq = 0.0f;
			// Compute and mean inductance for N(cycles)
			for (j = 0; j < cycles; j++) {
				// read zero current
				adc_get_DQ_current_mean(&I0_d, &I0_q, elect_angle, 20);
				// step the voltage
				pwm_set_from_Ud_Uq(voltage, 0.0f, elect_angle);
				t0 = time_get_us();

				// wait a little bit
				time_delay_us( risetime_us );

				// read D-Q currents
				adc_get_DQ_current(&IR_d, &IR_q, elect_angle);
				t1 = time_get_us();

				// power-off
				pwm_set_from_Ud_Uq(0.0f, 0.0f, elect_angle);

				// wait a bit for the currents to go to 0 again
				time_delay_us( settle_us );

				// calculate the inductance
				dt = (t1 - t0)/1000000.0f;

				// error, quadrature current is greater than expected ... or resistance value is too low than expected
				if ( (IR_d - I0_d) > 0.0f && (voltage - motor_output_1 * (IR_d - I0_d)) > 0.0f ) {
					// Compute inductance
					inductanced += fabsf( -(motor_output_1* dt) / logf(( voltage - motor_output_1 * (IR_d - I0_d)) / voltage ) )/ factor;
					
					// sum measured currents for average
					sumIq += IR_q - I0_q;
					sumId += IR_d - I0_d;
				};
			}
			// average measured currents
			sumIq /= cycles;
			sumId /= cycles;
			// average measured inductance
			inductanced /= cycles;
			// Filtered measured inductance, first two value not filtered
			Ltemp = i < 2 ? inductanced : Ltemp * 0.6 + inductanced * 0.4;			
			timeconstant = fabsf(Ltemp / motor_output_1); // Timeconstant of an RL circuit (L/R) 
		
			// Wait as long as possible (due to limited timing accuracy & sample rate), but as short as needed (while the current still changes)
      risetime_us = fmin( fmax( risetime_us * 0.6f + 0.4f * 1000000.0f * 0.6f * timeconstant, 100.0f ), 10000.0f);
      settle_us = 10.0f * risetime_us;

			/**
			 * How this code works:
			 * If we apply a current spike in the d-axis, there will be cross coupling to the q-axis current, if we didnt use the actual d-axis (ie. d' != d).
			 * This has to do with saliency (Ld != Lq). 
			 * The amount of cross coupled current is somewhat proportional to the angle error, which means that if we iteratively change the angle to min/maximise this current, we get the correct d-axis (and q-axis).
			*/
			
			// d or q axis ?
			delta = sumIq / (fabsf(sumId) + fabsf(sumIq));
			if (axis)	sumIq *= -1.0f;
			if (sumIq < 0) {
				elect_angle += fabsf(delta);
			} else {
				elect_angle -= fabsf(delta);
			}
			// range 0 to 2PI
			elect_angle = check_rad_range(elect_angle);

			// Average the d-axis angle further for calculating the electrical zero later
			if ( axis ) {
				d_electrical_angle = i < 2 ? elect_angle : d_electrical_angle * 0.9 + elect_angle * 0.1;
			}
		}
		
		// We know the minimum is 0.5*PI radians away, so less iterations are needed.
		elect_angle = check_rad_range(elect_angle - 0.5f * _PI);
		iterations /= 2;

		if (axis == 0) {
			Lq = Ltemp;
		} else {
			Ld = Ltemp;
		}
	}
	
	/*
	 * The d_electrical_angle should now be aligned to the d axis or the -d axis. We can therefore calculate two possible electrical zero angles. 
	 * We then report the one closest to the actual value. This could be useful if the zero search method is not reliable enough (eg. high pole count).
	*/
	float estimated_zero_electric_angle_A, estimated_zero_electric_angle_B;
	// read actual position
	encoder_update_counters();
	estimated_zero_electric_angle_A = check_rad_range(  (float)(motor_param_pp) * encoder_get_angle_rad() - d_electrical_angle );
	estimated_zero_electric_angle_B = check_rad_range(  (float)(motor_param_pp) * encoder_get_angle_rad() - d_electrical_angle + _PI );	
	if ( fabsf( estimated_zero_electric_angle_A - motor_zero_electrical_angle ) < fabsf(estimated_zero_electric_angle_B - motor_zero_electrical_angle )) {
		motor_output_2 = estimated_zero_electric_angle_A;
	} else {
		motor_output_2 = estimated_zero_electric_angle_B;
	}

	// Save the measured values
	motor_output_3 = Lq;
	motor_output_4 = Ld;
		
	// Error, Ld > Lq
	if (Ld > Lq) {
		set_motor_error( MOT_ERROR_IDDENTIFY_LD_LQ );
		
		// DEBUG: oled
		oled_write(1, "ERR: Ld>Lq");
		oled_write(2, "L (mH) :");
		sprintf(mystring, "Ld: %f", Ld*1000.0f);
		oled_write(3, mystring);
		sprintf(mystring, "Lq: %f", Lq*1000.0f);
		oled_write(4, mystring);

		return;
	}
	// Error, Lq > 2 Ld 
	if (Lq > Ld * 2.0f) {
		set_motor_error( MOT_ERROR_IDDENTIFY_LQ_2LD );

		// DEBUG: oled
		oled_write(1, "ERR: Lq > 2 Ld");
		oled_write(2, "L (mH) :");
		sprintf(mystring, "Ld: %f", Ld*1000.0f);
		oled_write(3, mystring);
		sprintf(mystring, "Lq: %f", Lq*1000.0f);
		oled_write(4, mystring);

		return;
	}
	
	// Save values to params
	if (motor_param_Lq == 0.0f) motor_param_Lq = motor_output_3;
	if (motor_param_Ld == 0.0f) motor_param_Ld = motor_output_4;

	// DEBUG: oled
	oled_write(2, "L (mH) :");
	sprintf(mystring, "Ld: %f", Ld*1000.0f);
	oled_write(3, mystring);
	sprintf(mystring, "Lq: %f", Lq*1000.0f);
	oled_write(4, mystring);
	
	// Update motor status
	// disabled: ok, done
	motor_enabled = 0;
	// update
	motor_update_status();
}

// Tune Current Controller
// WARNING:
// User should call this function after identifying the motor parameters: R, Ld, Lq
void motor_tune_PID_current() {
	float Kp, Ki;
	// Warning: bandwidth (or fc) is in rad/s
	// for 20 KHz PWM the recommended PID Current bandwidth should between 0.5% to 10%
	// but the recommended value for a 1 sampling per PWM is, fc = 1000 rad/s ....
	float bandwidth = 1000.0f;

	// Set PID current parameters
	// PID current Q values
	Kp = motor_param_Lq * (_2PI*bandwidth);
	Ki = motor_param_R  * (_2PI*bandwidth);
	PID_Init(&PIDCurrent_Q, Kp, Ki, 0.0f, motor_loop_Ts/1.0e6f, 0.0f, 0.0f, motor_param_maxV);
	// PID current D values
  Kp = motor_param_Ld * (_2PI*bandwidth);
  Ki = motor_param_R  * (_2PI*bandwidth);
	PID_Init(&PIDCurrent_D, Kp, Ki, 0.0f, motor_loop_Ts/1.0e6f, 0.0f, 0.0f, motor_param_maxV);
	// Low Pass Filter in Current:
	// Increase bandwidth to 5 when computing first order time constant Tf
	// Increasing banddwidh is similar to reducing constatnt time
	// Tf = 1 / ( 2 pi fc )
	// This is because the low pass filter introduce a delay as 5 Tf
	LPF_Init( &LPFCurrentD, 1.0f/(5.0f*_2PI*bandwidth), motor_loop_Ts/1.0e6f );
  LPF_Init( &LPFCurrentQ, 1.0f/(5.0f*_2PI*bandwidth), motor_loop_Ts/1.0e6f );
	
	// Needed for lag compensation ......
	// Low Pass Filter in Velocity:
	// frec_rps = motor_param_maxW * (2PI) / 60
	// frec_hz  = frec_rps / (2PI) = motor_param_maxW / 60
	// Fcutoff  = 5 FrecHz = 5*motor_param_maxW/60
	// Tf = 1 / ( 2PI Fc ) = 6 / ( PI*motor_param_maxW )
	LPF_Init( &LPFVelocity, 6.0f/( _PI*motor_param_maxW ), motor_loop_Ts/1.0e6f );
	
	// update status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = PID_CURRENT_AUTOTUNE;
	motor_control_status = 0;
	motor_update_status();
	// no user input yet
	motor_user_desired = 0.0f;
}

//
// Tune Velocity PID
// WARNING:
// This function should be called after setting correctly motor parameters: R, L, Max Voltage and Max Current
void motor_tune_PID_velocity() {
	// no user setpoint
	motor_user_desired = 0.0;
	// update motor status
	motor_error = 0;
	motor_enabled = 1;
	motor_control_mode = PID_VELOCITY_AUTOTUNE;
	motor_update_status();
	// clear function output
	motor_output_1 = 0.0f;
	motor_output_2 = 0.0f;
	motor_output_3 = 0.0f;
	motor_output_4 = 0.0f;

	// Reset LPF filters
	LPF_Reset( &LPFCurrentD );
	LPF_Reset( &LPFCurrentQ );
	LPF_Reset( &LPFVelocity );

	// Init autotuner
	// expected velocity: 0 (no motion)
	// Minimun input to motor: -90% Imax in Amp
	// Maximun input to motor:  90% Imax in Amp
	// Expected noise (hysteresis) in motor velocity: 1 RPM (0.1 rad/s)
	// Maximun number of cycles: 50
	PID_Autotune_Init(&PIDVelocity, 0.0f, -0.9f*motor_param_maxI/1000.0f, 0.9f*motor_param_maxI/1000.0f, 0.1f, 50 );

	// DEBUG:
	oled_clearAll();
	oled_write(0, "PID VEL TUN");

	// Do autotune
	int res = 0;
	float *ptrOutput[] = {&motor_output_1, &motor_output_2, &motor_output_3, &motor_output_4};
	int idx = 0;
	while ( res == 0 ) {
		// start time for this loop
		uint64_t starttime = time_get_us();
		// update inputs
		motor_update_inputs();
		// security check, overcurrent detected
		if (motor_error) {
			// security
			pwm_set_channels(0,0,0,0);
			// DEBUG:
			oled_write(1, "ERR: Imax");
			return;
		}
		// update res with autotune rutine
		res = PID_Autotune_Compute( &motor_setpoint_curr, motor_vel_rps );
		// current controller
		motor_current_loop();
		// wait for 200 us between loops
		int16_t waittime = motor_loop_Ts - (time_get_us() - starttime);
		if ( waittime > 0 ) time_delay_us( waittime ); else {
			// overrrun detected, save data for user analysis
			*ptrOutput[idx] = (float) waittime;
			if (++idx > 3) idx = 0;
		}
	}

	// power off motor
	pwm_set_from_Ud_Uq(0.0f, 0.0f, 0.0f);

	// check timeout error
	if (res < 0 ) {
		// Set error flag
		set_motor_error( MOT_ERROR_TUNE_VEL );
		// DEBUG:
		oled_write(1, "ERR: Not Tune");
		return;
	}

	// DEBUG:
	char mystring[30];
	sprintf(mystring, "Kp %f", PIDVelocity.Kp);
	oled_write(1, mystring);
	sprintf(mystring, "Ki %f", PIDVelocity.Ki);
	oled_write(2, mystring);
	sprintf(mystring, "Kd %f", PIDVelocity.Kd);
	oled_write(3, mystring);

	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_status = 0;
	motor_update_status();
	// no user input yet
	motor_user_desired = 0.0f;
}

//
// Tune Position PID
// WARNING:
// This function should be called after setting correctly motor parameters: R, L, Max Voltage and Max Current
// and tunning Velocity PID
void motor_tune_PID_position() {
	// no user setpoint
	motor_user_desired = 0.0;
	// update motor status
	motor_error = 0;
	motor_enabled = 1;
	motor_control_mode = PID_POSITION_AUTOTUNE;
	motor_update_status();
	// clear function output
	motor_output_1 = 0.0f;
	motor_output_2 = 0.0f;
	motor_output_3 = 0.0f;
	motor_output_4 = 0.0f;
	// power off
	pwm_set_from_Ua_Ub(0.0, 0.0);

	// Reset LPF filters
	LPF_Reset( &LPFCurrentD );
	LPF_Reset( &LPFCurrentQ );
	LPF_Reset( &LPFVelocity );

	// clear absolute position
	motor_position_long = 0;
	encoder_update_counters();
	encoder_get_pulses_delta();  // <-- delta set to 0

	// DEBUG:
	char mystring[30];
	oled_clearAll();
	oled_write(0, "PID POS TUN");

	// Init autotuner
	// expected position: 0 pulses (no motion)
	// Minimun input to motor: -maxW in rad/s
	// Maximun input to motor:  maxW in rad/s
	// Expected noise (hysteresis) in motor position: 1000 pulses
	// Maximun number of cycles: 100
	float maxVel = ((float) motor_param_maxW)*_2PI/60.0f;
	PID_Autotune_Init( &PIDPosition, 0.0f, -0.9f*maxVel, 0.9f*maxVel, 100, 100 );

	// Do autotune
	int res = 0;
	float *ptrOutput[] = {&motor_output_1, &motor_output_2, &motor_output_3, &motor_output_4};
	int idx = 0;
	while ( res == 0 ) {
		uint64_t starttime = time_get_us();
		// update inputs
		motor_update_inputs();
		// security check for motor overcurrent
		if (motor_error) {
			// security
			pwm_set_channels(0,0,0,0);
			// DEBUG:
			oled_write(1, "ERR: Imax");
			return;
		}
		// Autotune
		res = PID_Autotune_Compute( &motor_setpoint_vel, motor_position_long );
		// velocity controller
		motor_velocity_loop();
		// current controller
		motor_current_loop();
		// wait for 200 us between loops
		int16_t waittime = motor_loop_Ts - (time_get_us() - starttime);
		if ( waittime > 0 ) time_delay_us( waittime ); else {
			// overrun detected, save data for user analysis
			*ptrOutput[idx] = waittime;
			if (++idx > 3) idx = 0;
		}
	}

	// power off motor
	pwm_set_from_Ud_Uq(0.0f, 0.0f, 0.0f);
		
	// timeout error
	if (res < 0 ) {
		set_motor_error( MOT_ERROR_TUNE_POS );
		
		// DEBUG:
		oled_write(1, "ERR: Not Tune");
		
		return;
	}
	
	// DEBUG:
	sprintf(mystring, "Kp %f", PIDPosition.Kp);
	oled_write(1, mystring);
	sprintf(mystring, "Ki %f", PIDPosition.Ki);
	oled_write(2, mystring);
	sprintf(mystring, "Kd %f", PIDPosition.Kd);
	oled_write(3, mystring);
	
	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_status = 0;
	motor_update_status();
	// no user input yet
	motor_user_desired = 0;
}

// measure time 
void motor_measure_loops() {
	// variables
	int i;
	uint64_t start_time, end_time;
	char mystring[30];
	int counts = 10000;
	
	// output values
	motor_output_1 = motor_output_2 = motor_output_3 = motor_output_4 = 0.0f;

	//
	// Check Current controller
	//
	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = CURRENT_CONTROLLER;
	motor_control_status = MOT_COMMAND_CHECK_LOOP;
	motor_update_status();

	// compute N loops
	start_time = time_get_us();
	for (i = 0; i < counts; i++) {
		// call update
		motor_update_inputs();
		// call controller
		motor_control_loop();
	}
	end_time = time_get_us();
	motor_output_1 = ((float) (end_time - start_time)) / ((float)counts);
	
	// DEBUG:
	oled_clearAll();
	sprintf(mystring, "CUR: %f", motor_output_1);
	oled_write( 0, mystring );

	//
	// Check Velocity controller
	//
	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = VELOCITY_CONTROLLER;
	motor_control_status = MOT_COMMAND_CHECK_LOOP;
	motor_update_status();

	// compute N loops
	start_time = time_get_us();
	for (i = 0; i < counts; i++) {
		// call update
		motor_update_inputs();
		// call controller
		motor_control_loop();
	}
	end_time = time_get_us();
	motor_output_2 = ((float) (end_time - start_time)) / ((float)counts);
	
	// DEBUG:
	sprintf(mystring, "VEL: %f", motor_output_2);
	oled_write( 1, mystring );

	//
	// Check Position controller
	//
	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = POSITION_CONTROLLER;
	motor_control_status = MOT_COMMAND_CHECK_LOOP;
	motor_update_status();

	// compute N loops
	start_time = time_get_us();
	for (i = 0; i < counts; i++) {
		// call update
		motor_update_inputs();
		// call controllers
		motor_control_loop();
	}
	end_time = time_get_us();
	motor_output_3 = ((float) (end_time - start_time)) / ((float)counts);
	
	// DEBUG:
	sprintf(mystring, "POS: %f", motor_output_3);
	oled_write( 2, mystring );
	
	//
	// Check Profile Position controller
	//
	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = PROF_POS_CONTROLLER;
	motor_control_status = MOT_COMMAND_CHECK_LOOP;
	motor_update_status();
	
	// Set profile position data
	// 1.000.000 pulses at 1000 RPM with 500 ms of Tacel
	PP_Init( 0.0f, 1000000, time_get_us()+100, 1000.0f*16384.0f/60.0f, 500.0f );
	
	// compute N loops
	start_time = time_get_us();
	int done;
	for (i = 0; i < counts; i++) {
		// profile position
		motor_setpoint_pos = PP_Compute( time_get_us(), &done );
		// call update
		motor_update_inputs();
		// call controllers
		motor_control_loop();
	}
	end_time = time_get_us();
	motor_output_4 = ((float) (end_time - start_time)) / ((float)counts);
	
	// DEBUG:
	sprintf(mystring, "P.P.: %f", motor_output_4);
	oled_write( 3, mystring );
	
	// Save Status
	motor_error = 0;
	motor_enabled = 0;
	motor_control_mode = CURRENT_CONTROLLER;
	motor_control_status = 0;
	motor_update_status();
}

// Process User Command ...
void motor_command_loop() {
	
	// Is there a new command ? or (motor_command != 0)
	if ( motor_command ) {
		
		// if ERROR, only accept MOT_COMMAND_CLEAR
		if ( motor_error && ( motor_command != MOT_COMMAND_CLEAR) ) return;
		
		// New user command, clear motor status
		motor_control_status = 0;

		// Process command
		switch ( motor_command ) {
			case MOT_COMMAND_STOP:
				// Stop Motor while keeping actual control mode
				// Stop current, velocity, home using current and home using velocity
				motor_user_desired = 0;
				// Stop position Controller using actual position
				if ( motor_control_mode == POSITION_CONTROLLER ) motor_user_desired = motor_position_long;
				// Stop profile Position Controller to actual position
				if ( motor_control_mode == PROF_POS_CONTROLLER ) {
					motor_user_desired = motor_position_long;
					PP_Init( motor_position_long, motor_position_long, time_get_us()+100, (float)motor_pp_maxvel, motor_pp_tacel );
				}
				// Other controller to disable
				if ( motor_control_mode > HOME_VELOCITY ) motor_enabled = 0;
				// Clear command
				motor_command = 0;
				break;

			case MOT_COMMAND_CLEAR:
				// Clear error bit
			  motor_error = 0;
			  motor_enabled = 0;
  			motor_control_status = 0;
				// Erase command
			  motor_command = 0;
			  break;

			case MOT_COMMAND_RESET:
				// Reset controller
				__disable_irq();
				NVIC_SystemReset();
			  break;

			case MOT_COMMAND_ENABLE:
				// already enabled ?
				if ( !motor_enabled ) {
					// Reset LPF
					LPF_Reset( &LPFPosition );
					LPF_Reset( &LPFVelocity );
					LPF_Reset( &LPFCurrentD );
					LPF_Reset( &LPFCurrentQ );
					// Reset PID
					PID_Reset( &PIDPosition );
					PID_Reset( &PIDVelocity );
					PID_Reset( &PIDCurrent_D );
					PID_Reset( &PIDCurrent_Q );
					// Security check, avoid motor run-out
					// Reset user setpoint for current, velocity and home controllers
					motor_user_desired = 0;
					// Position Controller  to actual position
					if ( motor_control_mode == POSITION_CONTROLLER ) motor_user_desired = motor_position_long;
					// Reset path planning for profile position controller
					if ( motor_control_mode == PROF_POS_CONTROLLER ) {
						motor_user_desired = motor_position_long;
						PP_Init( motor_position_long, motor_position_long, time_get_us()+100, (float)motor_pp_maxvel, (float)motor_pp_tacel );
					}
					// Other controller are invalid, so set error 
					if ( motor_control_mode > HOME_VELOCITY ) {
						set_motor_error( MOT_ERROR_CANNOT_ENABLE );
					} else {
						// Enable motor
						motor_enabled = 1;
					}
				}
			  // Erase command
			  motor_command = 0;
				break;
				
			case MOT_COMMAND_DISABLE:
				// disable motor
				motor_enabled = 0;
				// clear command
				motor_command = 0;
				break;

			case MOT_COMMAND_HOME_NOW:
				// reset encoder
				encoder_get_pulses_delta();
				motor_position_long = 0;
				// Security check: avoid motor run-out
				// Setpoint to zero for all types of controller
				motor_user_desired = 0;
				// Reset path-planing to zero
				if ( motor_control_mode == PROF_POS_CONTROLLER ) {
					// set path-planning to constant position -> 0
					PP_Init( motor_position_long, motor_position_long, time_get_us()+100, (float)motor_pp_maxvel, (float)motor_pp_tacel );
				}
			  // Erase command
			  motor_command = 0;
				break;
			
  		case MOT_COMMAND_CURRENT_CONTROL:
			case MOT_COMMAND_VELOCITY_CONTROL:
			case MOT_COMMAND_POSITION_CONTROL:
			case MOT_COMMAND_PROFILE_POSITION_CONTROL:
			case MOT_COMMAND_HOME_CURRENT:
			case MOT_COMMAND_HOME_VELOCITY:
			  // New control type ? -> reset PID and LPF if enabled
			  if ( motor_enabled && (motor_control_mode != (motor_command-1)) ) {
					// Reset LPF
					LPF_Reset( &LPFPosition );
					LPF_Reset( &LPFVelocity );
					LPF_Reset( &LPFCurrentD );
					LPF_Reset( &LPFCurrentQ );
					// Reset PID
					PID_Reset( &PIDPosition );
					PID_Reset( &PIDVelocity );
					PID_Reset( &PIDCurrent_D );
					PID_Reset( &PIDCurrent_Q );
					// Security check: avoid motor run-out
					// Clear setpoint for current, velocity, home using current and home using velocity
					motor_user_desired = 0;
					// position controller to actual position
					if (motor_control_mode == POSITION_CONTROLLER ) motor_user_desired = motor_position_long;
					// Reset path planning for profile position controller
					if ( motor_control_mode == PROF_POS_CONTROLLER ) {
						motor_user_desired = motor_position_long;
						PP_Init( motor_position_long, motor_position_long, time_get_us()+100, (float)motor_pp_maxvel, (float)motor_pp_tacel );
					}
				}
				// update motor control
			  motor_control_mode = motor_command-1;
				// Start counting for time-out if homing
				if ( (motor_command == MOT_COMMAND_HOME_CURRENT) || (motor_command == MOT_COMMAND_HOME_VELOCITY ) ) {
					// Read actual time
					motor_output_1 = ((float) time_get_us()) / 1000.0f;
				}
				// Motor control status Initialized
				motor_control_status = 0;
				// Erase command
			  motor_command = 0;
			  break;

			case MOT_COMMAND_START_PP_MOTION:
				// Command only valid in profile position control
			  if ( motor_control_mode != PROF_POS_CONTROLLER ) {
					// Error: trying to do profile position
					set_motor_error( MOT_ERROR_NOT_PROF_POS );
					// reset command
					motor_command = 0;
					return;
				};
				// motor control status reset
				motor_control_status = 0;
				// Set motion values in motor_user_desired, motor_pp_maxvel and motor_pp_tacel before executing this command
			  // motion start in 100 us
				PP_Init( motor_position_long, motor_user_desired, time_get_us()+100, (float)motor_pp_maxvel, (float)motor_pp_tacel );
				// reset command
				motor_command = 0;
			  break;
				
			case MOT_COMMAND_PARAMS_CHANGED:
				// Verify params and check limits to PIDs
			  motor_param_maxV = fabsf( motor_param_maxV );
			  motor_param_maxI = fabsf( motor_param_maxI );
			  motor_param_maxW = fabsf( motor_param_maxW );
				// Check physical limit: cannot apply a voltage greater than actual voltage
			  if ( motor_param_maxV > motor_voltage ) motor_param_maxV = motor_voltage;
			  // Update PIDs limits
			  PIDCurrent_D.limit_y = motor_param_maxV;
			  PIDCurrent_Q.limit_y = motor_param_maxV;
			  PIDVelocity.limit_y  = motor_param_maxI/1000.0f;
			  PIDPosition.limit_y  = motor_param_maxW*_2PI/60.0f;
			  // clear command
			  motor_command = 0;
			  break;

			case MOT_COMMAND_ALIGN_CURRENT:
				// Call function
				motor_current_align();
				// user should check errors and status after command execution ...
				motor_command = 0;
			  break;

			case MOT_COMMAND_ALIGN_ENCODER:
				// Call function
				motor_encoder_align_and_zero();
				// user should check errors and status after command execution ...
				motor_command = 0;
			  break;
	
			case MOT_COMMAND_IDENTIFY:
				// Call function
  			motor_identify_R_L( 1.0f );
				// user should check errors and status after command execution ...
				motor_command = 0;
			  break;
			
			case MOT_COMMAND_TUNE_CURRENT:
				// call autotune velocity PID
			  motor_tune_PID_current();
			  // reset command
			  motor_command = 0;
			  break;
			
			case MOT_COMMAND_TUNE_VEL:
				// call autotune velocity PID
			  motor_tune_PID_velocity();
			  // reset command
			  motor_command = 0;
			  break;
			
			case MOT_COMMAND_TUNE_POS:
  			// call autotune position PID
			  motor_tune_PID_position();
			  // reset command
			  motor_command = 0;
		    break;
			
			case MOT_COMMAND_CHECK_LOOP:
				// call function
				motor_measure_loops();
				// reset command
				motor_command = 0;
				break;
			
			case MOT_COMMAND_SAVE_DATA:
				// disable motor
				motor_enabled = 0;
				// motor control and status 
				motor_control_mode = MOT_COMMAND_SAVE_DATA;
				// status to BUSY
				motor_control_status = 1;
				// update status
				motor_update_status();
				// Save data to Flash
				flash_write_data();
				// motor control status 
				motor_control_status = 0;
				// update status
				motor_update_status();
			  // reset command
			  motor_command = 0;
		    break;
			
			case MOT_COMMAND_RESTORE_DATA:
				// disable motor
				motor_enabled = 0;
				// motor control and status 
				motor_control_mode = MOT_COMMAND_RESTORE_DATA;
				// status to BUSY
				motor_control_status = 1;
				// update status
				motor_update_status();
				// read data from flash
				motor_load_flash_data();
				// motor control status 
				motor_control_status = 0;
				// update status
				motor_update_status();
			  // reset command
			  motor_command = 0;
		    break;
			
			default:
				// Error Unknown command
  			set_motor_error( MOT_ERROR_UNKNOWN_COMMAND );
			  motor_command = 0;
				break;
		}
	}
}
