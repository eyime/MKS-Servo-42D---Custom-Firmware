#include <n32l40x.h>
#include <stdint.h>

#include "foc_flash.h"


// Flash address is last flash page (Number 63) 0x0801-F800
#define FLASH_START_ADDR 0x0801F800

// Struct to read or write to flash
typedef struct {
	int     motor_nodeID;
	float   motor_maxvolt;
	float   motor_maxvel;
	float   motor_maxcurr;
	int     motor_pole_pairs;
	float   motor_resistance;
	float   motor_inductance_d;
	float   motor_inductance_q;
	float   motor_voltage_align;
	float   motor_encoder_ppr;
	int     motor_phases_pol;
	int     motor_encoder_dir;
	int     motor_phaseA_inv;
	int     motor_phaseB_inv;
	int     motor_phase_swapped;
	float   motor_zero_elect;

	float   motor_Kp_current_d;
	float   motor_Ki_current_d;
	float   motor_Kd_current_d;
	float   motor_FF_current_d;
	float   motor_ramp_current_d;
	float   motor_limit_current_d;
	
	float   motor_Kp_current_q;
	float   motor_Ki_current_q;
	float   motor_Kd_current_q;
	float   motor_FF_current_q;
	float   motor_ramp_current_q;
	float   motor_limit_current_q;

	float   motor_Kp_velocity;
	float   motor_Ki_velocity;
	float   motor_Kd_velocity;
	float   motor_FF_velocity;
	float   motor_ramp_velocity;
	float   motor_limit_velocity;

	float   motor_Kp_position;
	float   motor_Ki_position;
	float   motor_Kd_position;
	float   motor_FF_position;
	float   motor_ramp_position;
	float   motor_limit_position;

	float   motor_lpf_current_q;
	float   motor_lpf_current_d;
	float   motor_lpf_velocity;
} flash_data_t;


// External motor OD
Motor_OD_Entry_t *flash_ptr_OD = 0;


// local memory to copy data
flash_data_t flash_localdata;

// Register external OD
void flash_init(Motor_OD_Entry_t *ptr_OD) {
	// copy pointer
	flash_ptr_OD = ptr_OD;
}

// Copy data from flash to motor OD
void flash_read_data( ) {	
	// 32 bits (4 bytes) each time
	uint32_t *pdata = (uint32_t*) &flash_localdata;
	for ( int Index = 0; Index < (int) sizeof(flash_data_t); Index +=4 ) {
		// index is in bytes, pdata points to DWORD (4 bytes)
		// so, division by 4
		pdata[Index>>2] = (*(__IO uint32_t*)(FLASH_START_ADDR + Index));
	}
	
	// Copy to motor OD
	*((uint8_t*)flash_ptr_OD[OD_NODEID].ptrData) = flash_localdata.motor_nodeID;

	*((float*)flash_ptr_OD[OD_MOTOR_MAXVOLT].ptrData) = flash_localdata.motor_maxvolt;
	*((float*)flash_ptr_OD[OD_MOTOR_MAXVEL].ptrData) = flash_localdata.motor_maxvel;
	*((float*)flash_ptr_OD[OD_MOTOR_MAXCURR].ptrData) = flash_localdata.motor_maxcurr;
	*((uint8_t*)flash_ptr_OD[OD_MOTOR_PP].ptrData) = flash_localdata.motor_pole_pairs;
	
	*((float*)flash_ptr_OD[OD_MOTOR_R].ptrData) = flash_localdata.motor_resistance;
	*((float*)flash_ptr_OD[OD_MOTOR_LD].ptrData) = flash_localdata.motor_inductance_d;
	*((float*)flash_ptr_OD[OD_MOTOR_LQ].ptrData) = flash_localdata.motor_inductance_q;
	*((float*)flash_ptr_OD[OD_MOTOR_VOLT_ID].ptrData) = flash_localdata.motor_voltage_align;

	*((float*)flash_ptr_OD[OD_MOTOR_ENC_PPR].ptrData) = flash_localdata.motor_encoder_ppr;
	*((int16_t*)flash_ptr_OD[OD_MOTOR_ENC_DIR].ptrData) = flash_localdata.motor_encoder_dir;
	
	*((float*)flash_ptr_OD[OD_MOTOR_PHASES_POL].ptrData) = flash_localdata.motor_phases_pol;
	
	*((int16_t*)flash_ptr_OD[OD_MOTOR_PHASEA_INVERTED].ptrData) = flash_localdata.motor_phaseA_inv;
	*((int16_t*)flash_ptr_OD[OD_MOTOR_PHASEB_INVERTED].ptrData) = flash_localdata.motor_phaseB_inv;
	*((int16_t*)flash_ptr_OD[OD_MOTOR_PHASES_SWAPPED].ptrData)  = flash_localdata.motor_phase_swapped;
	
	*((float*)flash_ptr_OD[OD_MOTOR_ZERO_ELECT].ptrData) = flash_localdata.motor_zero_elect;

	*((float*)flash_ptr_OD[OD_MOTOR_PID_ID_KP].ptrData) = flash_localdata.motor_Kp_current_d;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_ID_KI].ptrData) = flash_localdata.motor_Ki_current_d;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_ID_KD].ptrData) = flash_localdata.motor_Kd_current_d;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_ID_FF].ptrData) = flash_localdata.motor_FF_current_d;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_ID_RAMP_LIM].ptrData) = flash_localdata.motor_ramp_current_d;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_ID_LIMIT].ptrData) = flash_localdata.motor_limit_current_d;

	*((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_KP].ptrData) = flash_localdata.motor_Kp_current_q;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_KI].ptrData) = flash_localdata.motor_Ki_current_q;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_KD].ptrData) = flash_localdata.motor_Kd_current_q;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_FF].ptrData) = flash_localdata.motor_FF_current_q;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_RAMP_LIM].ptrData) = flash_localdata.motor_ramp_current_q;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_LIMIT].ptrData) = flash_localdata.motor_limit_current_q;

	*((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_KP].ptrData) = flash_localdata.motor_Kp_velocity;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_KI].ptrData) = flash_localdata.motor_Ki_velocity;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_KD].ptrData) = flash_localdata.motor_Kd_velocity;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_FF].ptrData) = flash_localdata.motor_FF_velocity;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_RAMP_LIM].ptrData) = flash_localdata.motor_ramp_velocity;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_LIMIT].ptrData) = flash_localdata.motor_limit_velocity;

	*((float*)flash_ptr_OD[OD_MOTOR_PID_POS_KP].ptrData) = flash_localdata.motor_Kp_position;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_POS_KI].ptrData) = flash_localdata.motor_Ki_position;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_POS_KD].ptrData) = flash_localdata.motor_Kd_position;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_POS_FF].ptrData) = flash_localdata.motor_FF_position;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_POS_RAMP_LIM].ptrData) = flash_localdata.motor_ramp_position;
	*((float*)flash_ptr_OD[OD_MOTOR_PID_POS_LIMIT].ptrData) = flash_localdata.motor_limit_position;
	
	*((float*)flash_ptr_OD[OD_MOTOR_LPFID_ALPHA].ptrData)  = flash_localdata.motor_lpf_current_d;
	*((float*)flash_ptr_OD[OD_MOTOR_LPFIQ_ALPHA].ptrData)  = flash_localdata.motor_lpf_current_q;
	*((float*)flash_ptr_OD[OD_MOTOR_LPFVEL_ALPHA].ptrData) = flash_localdata.motor_lpf_velocity;
}


void flash_write_data() {	
	// Copy OD to local data
	flash_localdata.motor_nodeID = *((uint8_t*)flash_ptr_OD[OD_NODEID].ptrData);

	flash_localdata.motor_maxvolt = *((float*)flash_ptr_OD[OD_MOTOR_MAXVOLT].ptrData);
	flash_localdata.motor_maxvel  = *((float*)flash_ptr_OD[OD_MOTOR_MAXVEL].ptrData);
	flash_localdata.motor_maxcurr = *((float*)flash_ptr_OD[OD_MOTOR_MAXCURR].ptrData);
	flash_localdata.motor_pole_pairs = *((uint8_t*)flash_ptr_OD[OD_MOTOR_PP].ptrData);
	
	flash_localdata.motor_resistance = *((float*)flash_ptr_OD[OD_MOTOR_R].ptrData);
	flash_localdata.motor_inductance_d = *((float*)flash_ptr_OD[OD_MOTOR_LD].ptrData);
	flash_localdata.motor_inductance_q = *((float*)flash_ptr_OD[OD_MOTOR_LQ].ptrData);
	
	flash_localdata.motor_voltage_align = *((float*)flash_ptr_OD[OD_MOTOR_VOLT_ID].ptrData);

	flash_localdata.motor_encoder_ppr = *((float*)flash_ptr_OD[OD_MOTOR_ENC_PPR].ptrData);
	flash_localdata.motor_encoder_dir   = *((int16_t*)flash_ptr_OD[OD_MOTOR_ENC_DIR].ptrData);

	flash_localdata.motor_phases_pol = *((float*)flash_ptr_OD[OD_MOTOR_PHASES_POL].ptrData);

	flash_localdata.motor_phaseA_inv    = *((int16_t*)flash_ptr_OD[OD_MOTOR_PHASEA_INVERTED].ptrData);
	flash_localdata.motor_phaseB_inv    = *((int16_t*)flash_ptr_OD[OD_MOTOR_PHASEB_INVERTED].ptrData);
	flash_localdata.motor_phase_swapped = *((int16_t*)flash_ptr_OD[OD_MOTOR_PHASES_SWAPPED].ptrData);
	
	flash_localdata.motor_zero_elect = *((float*)flash_ptr_OD[OD_MOTOR_ZERO_ELECT].ptrData);
	
	flash_localdata.motor_Kp_current_d = *((float*)flash_ptr_OD[OD_MOTOR_PID_ID_KP].ptrData);
	flash_localdata.motor_Ki_current_d = *((float*)flash_ptr_OD[OD_MOTOR_PID_ID_KI].ptrData);
	flash_localdata.motor_Kd_current_d = *((float*)flash_ptr_OD[OD_MOTOR_PID_ID_KD].ptrData);
	flash_localdata.motor_FF_current_d = *((float*)flash_ptr_OD[OD_MOTOR_PID_ID_FF].ptrData);
	flash_localdata.motor_ramp_current_d = *((float*)flash_ptr_OD[OD_MOTOR_PID_ID_RAMP_LIM].ptrData);
	flash_localdata.motor_limit_current_d = *((float*)flash_ptr_OD[OD_MOTOR_PID_ID_LIMIT].ptrData);

	flash_localdata.motor_Kp_current_q = *((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_KP].ptrData);
	flash_localdata.motor_Ki_current_q = *((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_KI].ptrData);
	flash_localdata.motor_Kd_current_q = *((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_KD].ptrData);
	flash_localdata.motor_FF_current_q = *((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_FF].ptrData);
	flash_localdata.motor_ramp_current_q = *((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_RAMP_LIM].ptrData);
	flash_localdata.motor_limit_current_q = *((float*)flash_ptr_OD[OD_MOTOR_PID_IQ_LIMIT].ptrData);

	flash_localdata.motor_Kp_velocity = *((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_KP].ptrData);
	flash_localdata.motor_Ki_velocity = *((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_KI].ptrData);
	flash_localdata.motor_Kd_velocity = *((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_KD].ptrData);
	flash_localdata.motor_FF_velocity = *((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_FF].ptrData);
	flash_localdata.motor_ramp_velocity = *((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_RAMP_LIM].ptrData);
	flash_localdata.motor_limit_velocity = *((float*)flash_ptr_OD[OD_MOTOR_PID_VEL_LIMIT].ptrData);

	flash_localdata.motor_Kp_position = *((float*)flash_ptr_OD[OD_MOTOR_PID_POS_KP].ptrData);
	flash_localdata.motor_Ki_position = *((float*)flash_ptr_OD[OD_MOTOR_PID_POS_KI].ptrData);
	flash_localdata.motor_Kd_position = *((float*)flash_ptr_OD[OD_MOTOR_PID_POS_KD].ptrData);
	flash_localdata.motor_FF_position = *((float*)flash_ptr_OD[OD_MOTOR_PID_POS_FF].ptrData);
	flash_localdata.motor_ramp_position = *((float*)flash_ptr_OD[OD_MOTOR_PID_POS_RAMP_LIM].ptrData);
	flash_localdata.motor_limit_position = *((float*)flash_ptr_OD[OD_MOTOR_PID_POS_LIMIT].ptrData);
	
	flash_localdata.motor_lpf_current_d = *((float*)flash_ptr_OD[OD_MOTOR_LPFID_ALPHA].ptrData);
	flash_localdata.motor_lpf_current_q = *((float*)flash_ptr_OD[OD_MOTOR_LPFIQ_ALPHA].ptrData);
	flash_localdata.motor_lpf_velocity  = *((float*)flash_ptr_OD[OD_MOTOR_LPFVEL_ALPHA].ptrData);
	
	// Unlock Flash for writing
	FLASH_Unlock();
	// Erase last page, number 63
	FLASH_EraseOnePage(FLASH_START_ADDR);
	// Start writing 32 bits each time
	uint32_t *pdata = (uint32_t*) &flash_localdata;
	for ( int index=0; index < (int) sizeof(flash_data_t); index+=4 ) {
		FLASH_ProgramWord( FLASH_START_ADDR + index, pdata[index>>2] );
	}
	// Lock Flash
	FLASH_Lock();
}
