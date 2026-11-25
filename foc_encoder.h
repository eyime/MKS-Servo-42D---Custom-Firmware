#ifndef _FOC_ENCODER_H_
#define _FOC_ENCODER_H_

#include <stdint.h>
#include "foc_motor_od.h"

// Configure N32 hardware for SPI communication
void encoder_init(Motor_OD_Entry_t *prtOD);

// check local data
void encoder_check_data();

// Set inverted direction
void encoder_set_inverted_dir();

// Read encoder, update internal counters
// return error if communication fails
uint16_t encoder_update_counters();

// Get absolute pulses from reference position
// No encoder reading, just return internal counter 
int encoder_get_pulses_delta();

// Get angle in radians: 0 to 2*pi, only one turn
// No encoder reading, just return internal counter
float encoder_get_angle_rad();

#endif
