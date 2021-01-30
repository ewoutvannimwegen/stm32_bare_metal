#include "stm32l476xx.h"

int main(void) {
	RCC->CR = 0; // Clear control register
	RCC->CR |= RCC_CR_HSION; // HSI clock enabled
	while(!(RCC->CR & RCC_CR_HSIRDY)); // Wait for HSI ready

	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
	PWR->CR1 &= ~PWR_CR1_VOS_1; 
	PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1 

	FLASH->ACR |= FLASH_ACR_ICEN; // Instruction cache enabled
	FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
	FLASH->ACR |= FLASH_ACR_DCEN; // Data cache enabled
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS; // Flash latency 3 CPU cycles

	RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI as system clock
	while(!(RCC->CFGR & RCC_CFGR_SWS_HSI)); 
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; // GPIOA clock enabled

	/* LD2 [PA5] */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // Initial state LOW
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
	GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output

	while(1) {
		for(int i = 0; i < SystemCoreClock; i++); // Sleep 1 sec
		GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
	}
}
