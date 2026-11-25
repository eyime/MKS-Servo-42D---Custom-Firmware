#ifndef _FOC_TIME_H_
#define _FOC_TIME_H_

#include <stdint.h>

// delay us
void time_delay_us(uint32_t ms);

// delay ms
void time_delay_ms(uint16_t ms);

// get absolute micros from power on
uint64_t time_get_us();


#endif
