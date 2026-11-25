#ifndef _FOC_MOTOR_H_
#define _FOC_MOTOR_H_

#include "foc_motor_od.h"

// Initialize OD
void motor_init(Motor_OD_Entry_t *ptrOD);

// load data from flash
void motor_load_flash_data();

// Enumeration for motor control/status types
enum ControlType {
	CURRENT_CONTROLLER,
	VELOCITY_CONTROLLER,
	POSITION_CONTROLLER,
	PROF_POS_CONTROLLER,
	HOME_CURRENT,
	HOME_VELOCITY,
	ENCODER_ALIGN,
	CURRENT_ALIGN,
	MOTOR_IDENTIFY_RL,
	PID_CURRENT_AUTOTUNE,
	PID_VELOCITY_AUTOTUNE,
	PID_POSITION_AUTOTUNE
};

// The only interface for controlling the motor
void motor_control_loop();

// The only interface for processing user inputs
void motor_command_loop();

#endif

