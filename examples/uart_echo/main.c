#include "stm32l476xx.h"

#define BAUD 9600

void clk_init(void);
void io_init(void);
void usart2_init(void);

int main(void)
{
	clk_init();
	io_init();
	usart2_init();

	while (1)
		;
}

/* Enable the HSI clock source
 * Enable the power interface clock & set the voltage scaling
 * Configure flash 
 * Set HSI as SYSCLK: 16MHz
 * Set the AHB prescaler to 16 -> HCLK: 1MHz
 * APB1 & APB2 prescalers are 1 so APB1 timer clock is running at 1MHz
 * Update the clock configuration
 * Enable the GPIOA & USART2 clocks */
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

	RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI->SYSCLK
	while (!(RCC->CFGR & RCC_CFGR_SWS_HSI))
		;

	SystemCoreClockUpdate();

	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; // USART2 clock enabled
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;	// GPIOA clock enabled
}

void io_init(void)
{
	/* USART2 TX [PA2] */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;									  // Output push-pull
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2_1 | GPIO_OSPEEDR_OSPEED2_0); // Low speed
	GPIOA->MODER |= GPIO_MODER_MODE2_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE2_0; // Alternate function
	GPIOA->AFR[0] = 0;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_0; // AF7: USART2_TX

	/* USART2 RX [PA3] */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;									  // Output push-pull
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED3_0); // Low speed
	GPIOA->MODER |= GPIO_MODER_MODE3_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE3_0;										   // Alternate function
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_0; // AF7: USART2_RX
}

void usart2_init(void)
{
	uint16_t usartdiv = (SystemCoreClock / BAUD); // Get baud rate divider
	USART2->BRR = usartdiv;						  // Set baud rate divider
	USART2->CR1 |= USART_CR1_RXNEIE;			  // RX interrupt enabled
	NVIC_EnableIRQ(USART2_IRQn);				  // Interrupt enabled
	USART2->ICR |= USART_ICR_TCCF;				  // Clear transfer complete interrupt
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;	  // USART2 TX & RX enabled
	USART2->CR1 |= USART_CR1_UE;				  // USART2 enabled
}

void USART2_IRQHandler(void)
{
	if (USART2->ISR & USART_ISR_RXNE)
	{
		char val = USART2->RDR; // Copy RX buffer to val
		USART2->TDR = val; // Copy val to TX buffer
		while (!(USART2->ISR & USART_ISR_TC))
			; // Wait till transmission complete
	}
}