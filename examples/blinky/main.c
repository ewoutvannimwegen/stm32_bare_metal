#include "stm32l476xx.h"

int main() {
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // GPIOA clock enabled

	/* LD2 [PA5] */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // Initial state LOW
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
	GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output

	while(1) {
		GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
		for(int i = 0; 100000; i++); // Sleep 1 sec
	}
}
