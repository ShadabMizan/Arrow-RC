#include "OpenLoop.h"
#include "tim.h"

#include <stdint.h>
#include <stm32g431xx.h>

/* Trapezoidal Control Table:

    Step    A   B   C
       1    1   0   Z
       2    1   Z   0
       3    Z   1   0
       4    0   1   Z
       5    0   Z   1
       6    Z   0   1
*/

void trap_commutation(uint8_t step) {
    // Disable All momentarily
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE);

    switch (step) {
        case 0:
            // A HIGH, B LOW
            TIM1->CCER |= TIM_CCER_CC1E;
            TIM1->CCER |= TIM_CCER_CC2NE;
            break;
        case 1:
            break;
        case 2:
        case 3:
        case 4:
        case 5:
    }
}