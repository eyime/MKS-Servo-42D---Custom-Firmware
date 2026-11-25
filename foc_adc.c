#include <n32l40x.h>
#include <math.h>

#include "foc_time.h"
#include "foc_adc.h"

// Pointers to external variables
__IO uint16_t *ptr_ADC_Values = 0;

// Local variables
// ADC gain for converting readings to Amperes
float ADC_Gain;
// ADC values when no current is circulating
uint16_t ADC_zero1, ADC_zero2;
// PhaseA and PhaseB are swapped ?
int16_t swapped;
// Is each phase inverted ?
int16_t invertedA, invertedB;

// calibrate zero 
void adc_calibrate() {
	// temporaty int
	int i;
	// Read 128 convertions, each 50 uS 
	uint32_t Zero1 = 0.0;
	uint32_t Zero2 = 0.0;
	
	// Read 128 values (2^7)
	for (i=0; i<128; i++) {
		// start from actual readings
		Zero1 += ptr_ADC_Values[0];
		Zero2 += ptr_ADC_Values[1];
		// wait next PWM pulse (50 us)
		time_delay_us( 50 );
	}
	
	// Divide by 128
	ADC_zero1 = Zero1 >> 7;
	ADC_zero2 = Zero2 >> 7;
}

// ADC channles: PA1, PA2 and PA3
void adc_init(__IO uint16_t *ptrADCVALS, Motor_OD_Entry_t *prtOD) {
	// Store variable locally
	ptr_ADC_Values = ptrADCVALS;
	
	// Save local variables in Object Dictionary
	// InvertedA
	prtOD[OD_MOTOR_PHASEA_INVERTED].ptrData = &invertedA;
	prtOD[OD_MOTOR_PHASEA_INVERTED].MemSize = MEM_SZ_16;
	prtOD[OD_MOTOR_PHASEA_INVERTED].RdWrAtt = ATTR_READ;
	// InvertedB
	prtOD[OD_MOTOR_PHASEB_INVERTED].ptrData = &invertedB;
	prtOD[OD_MOTOR_PHASEB_INVERTED].MemSize = MEM_SZ_16;
	prtOD[OD_MOTOR_PHASEB_INVERTED].RdWrAtt = ATTR_READ;
	// Swapped
	prtOD[OD_MOTOR_PHASES_SWAPPED].ptrData = &swapped;
	prtOD[OD_MOTOR_PHASES_SWAPPED].MemSize = MEM_SZ_16;
	prtOD[OD_MOTOR_PHASES_SWAPPED].RdWrAtt = ATTR_READ;

	// Define ADC Gain based on MKS PCB schematics
	// ADC_Gain, converts 12bits ADC readings in Amperes
	// Ik = 3.3f/(4095.0f*G*R) * adc_val	
	// R = 0.05 Ohm
	// G = 6.65 (Op-Amp Gain)
	// ADC_Gain, converts ADC values to amperes
	ADC_Gain = 3.3f/(4095.0f*6.65f*0.05f);
	
	// Calibrate ADC
	adc_calibrate();
}

// check data loaded from flash
void adc_check_data() {
	// check and validate loaded data 
	if ( (invertedA != 0) && (invertedA != 1) ) invertedA = 0;
	if ( (invertedB != 0) && (invertedB != 1) ) invertedB = 0;
	if ( (swapped != 0) && (swapped != 1) ) swapped = 0;
}

// Reading PA1 as Phase 1
// Reading PA2 as Phase 2
// Reading PA3 as Power Supply Voltage
void adc_read_values(uint16_t* phase1, uint16_t* phase2, uint16_t *ext_volt) {
	*phase1   = ptr_ADC_Values[0];
	*phase2   = ptr_ADC_Values[1];
	*ext_volt = ptr_ADC_Values[2];
}

// set if phases are swapped
void adc_set_swapped_phases() {
	// set flag to true (1) or false (0)
	swapped = 1 - swapped;
}

// set if phase1 direction is inverted
void adc_set_inverted_phaseA() {
	// set flag to true (1) or false (0)
	invertedA = 1 - invertedA;
}

// set if phase2 direction is inverted
void adc_set_inverted_phaseB() {
	// set flag to true (1) or false (0)
	invertedB = 1 - invertedB;
}

// get MKS Vin voltage
float adc_get_Vin() {
	return ( 16.4f * (3.3f * ((float) ptr_ADC_Values[2]) / 4096.0f) );
}

// Read Current flow trough each channel in Amperes
// Apply swap and inverted bool bits to each phase
void adc_get_phase_current( float *curr1, float *curr2 ) {
	float phase1, phase2;
	// read ADC
	if ( swapped ) {
		phase1 = ADC_Gain*(ptr_ADC_Values[1] - ADC_zero2);
		phase2 = ADC_Gain*(ptr_ADC_Values[0] - ADC_zero1);
	} else {
		phase1 = ADC_Gain*(ptr_ADC_Values[0] - ADC_zero1);
		phase2 = ADC_Gain*(ptr_ADC_Values[1] - ADC_zero2);
	}
	// inverted ?
	if ( invertedA ) *curr1 = -phase1; else *curr1 = phase1;
	if ( invertedB ) *curr2 = -phase2; else *curr2 = phase2;
}

// Get alpha and beta currents
void adc_get_AlphaBeta_current( float *alpha, float *beta ) {
	// For Stepper Motor Alfa and Beta currents are the same that Phase1 and Phase2
	adc_get_phase_current( alpha, beta );
}

// Get Direct and Quadrature phase currents
void adc_get_DQ_current( float *Id, float *Iq, float elect_angle ) {
	float Ialpha, Ibeta;
	float cosang, sinang;
	
	// compute sin and cos
	cosang = cosf(elect_angle);
	sinang = sinf(elect_angle);
	
	// get alpha beta current
	adc_get_AlphaBeta_current( &Ialpha, &Ibeta );
	
	// calculate Park transform
	*Id = (Ialpha * cosang + Ibeta * sinang);
	*Iq = (Ibeta * cosang - Ialpha * sinang);
}

// Compute a mean of Direct and Quadrature currents
void adc_get_DQ_current_mean( float *Id, float *Iq, float elect_angle, int N ) {
	int i;
	float SumId = 0.0f;
	float SumIq = 0.0f;
	
	for (i=0; i<N; i++) {
		// Get Currents
		adc_get_DQ_current( Id, Iq, elect_angle );
		SumId += *Id;
		SumIq += *Iq;
		// wait until next PWM
		time_delay_us( 50 );
	}
	// output
	*Id = SumId / ((float)N);
	*Iq = SumIq / ((float)N);
}
