#include "stm32f4xx.h"

extern uint32_t SystemCoreClock;

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & ~(3U << 10)) | (1U << 10);

    while (1) {
        GPIOA->ODR ^= (1U << 5);
        for (volatile int i = 0; i < 8400000; i++);
    }
}
