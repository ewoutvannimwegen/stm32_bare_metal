#include "stm32l476xx.h"

void clk_init(void);
void io_init(void);
void tim2_init(void);

int main(void)
{
	clk_init();
	io_init();
	tim2_init();

	while (1);
}

/* Enable the HSI clock source
 * Enable the power interface clock & set the voltage scaling
 * Configure flash 
 * Set HSI as SYSCLK: 16MHz
 * Set the AHB prescaler to 16 -> HCLK: 1MHz
 * APB1 & APB2 prescalers are 1 so APB1 timer clock is running at 1MHz
 * Update the clock configuration
 * Enable the GPIOA & TIM2 clocks */
void clk_init(void)
{
	RCC->CR |= RCC_CR_HSION; // HSI clock enabled
	while (!(RCC->CR & RCC_CR_HSIRDY))
		; // Wait for HSI ready

	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
	PWR->CR1 &= ~PWR_CR1_VOS_1;
	PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1

	FLASH->ACR |= FLASH_ACR_ICEN;	// Instruction cache enabled
	FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
	FLASH->ACR |= FLASH_ACR_DCEN;	// Data cache enabled
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS; // Flash latency 3 CPU cycles

	RCC->CFGR |= RCC_CFGR_HPRE_3 | RCC_CFGR_HPRE_1 | RCC_CFGR_HPRE_0; // AHB/16

	RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI as system clock
	while (!(RCC->CFGR & RCC_CFGR_SWS_HSI))
		;

	SystemCoreClockUpdate();

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Timer2 clock enabled
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  // GPIOA clock enabled
}

/* Set LD2 [PA5] alternate function TIM2 CH1 */
void io_init(void)
{
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;									  // Initial state LOW
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
	GPIOA->MODER |= GPIO_MODER_MODE5_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE5_0; // Alternate function
	GPIOA->AFR[0] = 0;					 // Clear alternate function register
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL5_0; // Timer2 channel 1
}

/* Clear the control register
 * PWM: 2 sec
 * Duty cycle: 1 sec 
 * PWM mode 1 upcounting: 
 	* active if CNT (ARR) < CCR1
	* inactive if CNT >= CCR1 */
void tim2_init(void)
{
	TIM2->CR1 = 0; 
	TIM2->ARR = 2000000; // Auto-reload value (PWM)
	TIM2->CCR1 = 1000000; // Capture/compare value (duty cycle) 
	TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode 1	
	TIM2->CCER |= TIM_CCER_CC1E; // Capture/compare 1 output enabled
	TIM2->CR1 |= TIM_CR1_CEN; // Counter enabled
}