#ifndef _FOC_COMM_H_
#define _FOC_COMM_H_

#include <stdint.h>
#include "foc_motor_od.h"

// Register the external Object Dictionary
void com485_init(Motor_OD_Entry_t *ptrOD);

#endif

