#include <n32l40x.h>

#include "foc_encoder.h"
#include "foc_utils.h"

// MT8616 definitions
#define MT6816_CPR                          16384.0f
#define MT6816_READ_REG_03                  0x8300
#define MT6816_READ_REG_04                  0x8400
#define MT6816_BITORDER                     MSBFIRST

// Local variables

// Direction
int16_t encoder_dir = 1;
// Previous values in pulses for computing a delta
uint16_t encoder_prv_raw = 0;
// actual position in pulses, read by encoder
uint16_t encoder_actual_raw = 0;
// Delta value between two consecutive readings
int encoder_delta_raw = 0;
// actual position in radian
float encoder_pos_rad = 0.0f;


// Enable SPI at 16 MHz
void encoder_init(Motor_OD_Entry_t *prtOD) {
	/// Set PB6 as HIGH, nSS
	GPIO_SetBits( GPIOB, GPIO_PIN_6 );
	
	// Save local variable to Object Dictionary
	prtOD[OD_MOTOR_ENC_DIR].ptrData = &encoder_dir;
	prtOD[OD_MOTOR_ENC_DIR].MemSize = MEM_SZ_16;
	prtOD[OD_MOTOR_ENC_DIR].RdWrAtt = ATTR_READ;
}

// Check data loaded from flash
void encoder_check_data() {
	if ( (encoder_dir != 1) && (encoder_dir != -1) ) encoder_dir = 1;
}

// Internal function for sending / receiving 16 bits
uint16_t encoder_SPI_transfer16(uint16_t outdata) {
	// Start communication with MT8616, PB6 to LOW
	GPIO_ResetBits( GPIOB, GPIO_PIN_6 );

	// Waiting to SPI1 register to be emplty before sending more bits
	while (SPI_I2S_GetStatus(SPI1, SPI_I2S_TE_FLAG) == RESET);

	// Sending a 16 bits to slave
	SPI_I2S_TransmitData(SPI1, outdata);

	// Waiting to receive 16 bits from slave
	while (SPI_I2S_GetStatus(SPI1, SPI_I2S_RNE_FLAG) == RESET);

	// End communication, PB6 to HIGH
	GPIO_SetBits( GPIOB, GPIO_PIN_6 );

	// Return the slave answer
	return SPI_I2S_ReceiveData(SPI1);
}

// Checking CRC for 16 bits
int encoder_parity_check(uint16_t data) {
    data ^= data >> 8;
    data ^= data >> 4;
    data ^= data >> 2;
    data ^= data >> 1;
    return (~data) & 1;
}

// Read the encoder position
uint16_t encoder_read_position_raw(uint16_t *pos)  {
	uint16_t raw_pos;
	
	// read position
	raw_pos  = encoder_SPI_transfer16(MT6816_READ_REG_03) << 8;
	raw_pos |= encoder_SPI_transfer16(MT6816_READ_REG_04);

	// detect error 
	if ( (raw_pos >> 1) & 1 ) return MOT_ERROR_ENC_MAGNET;
	
	// Parity error in data
	if (!encoder_parity_check(raw_pos)) return MOT_ERROR_ENC_CRC;
	
	// Set position raw data
	*pos = (raw_pos >> 2);

  return 0;
}

// Set inverted direction
void encoder_set_inverted_dir() {
	encoder_dir = -encoder_dir;	
}

// update internal counters from actual reading
uint16_t encoder_update_counters() {
	uint16_t pos, err;
	
	// read encoder
	err = encoder_read_position_raw( &pos );
	// return error if set
	if (err) return err;

	// update local variables
	encoder_prv_raw    = encoder_actual_raw;
	encoder_actual_raw = pos;

	// compute delta
	int delta = (encoder_actual_raw - encoder_prv_raw);
	// pulses varies from 0 to 0x4000 (1<<14);
	// overflow ?
	if (delta < -0x2000) delta += 0x4001;
	// underflow ?
	if (delta >  0x2000) delta -= 0x4001;

	// store delta
	encoder_delta_raw += encoder_dir*delta;

	// store actual absolute position in radian, 0 to 2PI
	encoder_pos_rad = ((float)(1-encoder_dir))*_PI + encoder_dir*_2PI*((float)encoder_actual_raw / MT6816_CPR);

	return 0;
}

// get internal counter
int encoder_get_pulses_delta() {
	int val = encoder_delta_raw;
	encoder_delta_raw = 0;
	return val;
}

// get internal counter
float encoder_get_angle_rad() {
	return encoder_pos_rad;
}
