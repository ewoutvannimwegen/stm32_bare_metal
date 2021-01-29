#include "stm32l476xx.h"

int main(void) {
	/* LD2 [PA5] */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
	GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output
	
	/* User button [PC13] */
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

	GPIOC->IDR |= GPIO_PUPDR_PUPD13_1;
	GPIOC->IDR &= ~GPIO_PUPDR_PUPD13_0; // Pull-down
	GPIOC->MODER &= ~(GPIO_MODER_MODE13_1 | GPIO_MODER_MODE13_0); // Input

	NVIC_EnableIRQ(EXTI15_10_IRQn);

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	EXTI->IMR1 |= EXTI_IMR1_IM13; 
	EXTI->RTSR1 |= EXTI_RTSR1_RT13; // Rising trigger
	while(1);
}

void EXTI15_10_IRQHandler(void) {
	if(EXTI->PR1 & EXTI_PR1_PIF13) {
		EXTI->PR1 |= EXTI_PR1_PIF13; // Clear flag
		GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
	}
}