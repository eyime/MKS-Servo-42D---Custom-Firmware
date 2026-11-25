#include <n32l40x.h>

#include "foc_time.h"

// Local counter in ms (1000 us)
__IO uint64_t time_us_counter = 0;

// TIM2 int at 1Khz each 1 ms
void TIM2_IRQHandler(void) {
	if ( TIM_GetIntStatus(TIM2, TIM_INT_UPDATE) == SET ) {
		// Clear int flag
		TIM_ClrIntPendingBit(TIM2, TIM_INT_UPDATE);
		// increase counter
		time_us_counter += 1000;
	}
}

// return absolute micros since power-on
uint64_t time_get_us() {
	uint16_t cnt;

	// Interrupt generated at 999 counts, so skip this value 
	// for avoiding weird jumps, like 1999 to 1000, or 0999 to 1999
	while ( (cnt = TIM_GetCnt(TIM2)) > 998);

	uint64_t result = time_us_counter + cnt;

	return result;
}

// delay an amount of microseconds
void time_delay_us(uint32_t us) {
	// compute final micros
  uint64_t time_end = time_get_us() + us;

  // wait elapsed time
  while ( time_get_us() < time_end );
}

// delay an amount of miliseconds
void time_delay_ms(uint16_t ms) {
	// compute final time in micros
  uint64_t time_end = time_get_us() + 1000*ms;

  // wait elapsed time
  while ( time_get_us() < time_end );
}
