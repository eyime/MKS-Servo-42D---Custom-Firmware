#ifndef _FOC_ADC_H_
#define _FOC_ADC_H_

#include <n32l40x.h>
#include <stdint.h>

#include "foc_motor_od.h"

// Initialize the hardware
void adc_init(__IO uint16_t *ptrADC, Motor_OD_Entry_t *prtOD);

// check data loaded from flash
void adc_check_data();

// Read values from ADC channels
void adc_read_values(uint16_t* phase1, uint16_t* phase2, uint16_t *ext_volt);

// swap phases
void adc_set_swapped_phases();

// invert direction of each phase
void adc_set_inverted_phaseA();
void adc_set_inverted_phaseB();

// get MKS Vin voltage
float adc_get_Vin();

// get phase current in amperes
// apply swapped and inverted bits
void adc_get_phase_current( float *curr1, float *curr2 );

// Get alpha and beta currents
void adc_get_AlphaBeta_current( float *alpha, float *beta );

// Get Direct and Quadrature currents
void adc_get_DQ_current( float *Id, float *Iq, float elect_angle );

// Compute a mean of direct and quadrature currents
void adc_get_DQ_current_mean( float *Id, float *Iq, float elect_angle, int N );

#endif
