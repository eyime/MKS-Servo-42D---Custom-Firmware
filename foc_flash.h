#ifndef _FOC_FLASH_H_
#define _FOC_FLASH_H_

#include "foc_motor_od.h"

// Register OD
void flash_init(Motor_OD_Entry_t *pOD);

// Read from flash and store result in motor data
void flash_read_data();

// Write to flash data stored in motor data
void flash_write_data();


#endif
