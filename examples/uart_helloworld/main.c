#include "stm32l476xx.h"

#define BUFFER_SIZE 17
#define BAUD 9600

uint8_t tx_buffer[BUFFER_SIZE] = "Hello, world!\r\n";

void clk_init(void);
void io_init(void);
void dma1_ch7_init(void);
void usart2_init(void);
void error_handler(void);

int main(void)
{
	clk_init();
	io_init();
	dma1_ch7_init();
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

	RCC->CCIPR &= ~RCC_CCIPR_USART2SEL_0;
	RCC->CCIPR |= RCC_CCIPR_USART2SEL_1; // HSI->USART2

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;		// DMA1 clock enabled
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
}

void dma1_ch7_init(void)
{
	DMA1_Channel7->CCR = 0;				// Reset control register
	DMA1_Channel7->CCR |= DMA_CCR_PL_1; // Priority high
	DMA1_Channel7->CCR |= DMA_CCR_MINC; // Memory increment mode enabled
	DMA1_Channel7->CCR |= DMA_CCR_CIRC; // Circular mode enabled
	DMA1_Channel7->CCR |= DMA_CCR_DIR;	// Memory->Peripheral

	DMA1_CSELR->CSELR |= (2 << DMA_CSELR_C7S_Pos); // Select USART2 TX

	DMA1_Channel7->CCR |= DMA_CCR_TCIE; // Transfer complete ISR enabled
	DMA1_Channel7->CCR |= DMA_CCR_TEIE; // Transfer error ISR enabled
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	DMA1_Channel7->CNDTR = BUFFER_SIZE;				  // Data size
	DMA1_Channel7->CPAR = (uint32_t) & (USART2->TDR); // Peripheral address
	DMA1_Channel7->CMAR = (uint32_t)&tx_buffer[0];	  // Memory address
}

void usart2_init(void)
{
	uint16_t usartdiv = (SystemCoreClock / BAUD); // Get baud rate divider
	USART2->BRR = usartdiv;						  // Set baud rate divider
	USART2->ICR |= USART_ICR_TCCF;				  // Clear transmission complete flag
	USART2->CR1 |= USART_CR1_UE;				  // USART2 controller enabled
	USART2->CR3 |= USART_CR3_DMAT;				  // DMA transmit enabled
	USART2->CR1 |= USART_CR1_TE;				  // USART2 TX enabled
	DMA1_Channel7->CCR |= DMA_CCR_EN;			  // DMA1 Channel7 enabled
}

void error_handler(void)
{
	while (1)
		;
}

void DMA1_Channel7_IRQHandler(void)
{
	if (DMA1->ISR & DMA_ISR_TCIF7)
	{
		DMA1->IFCR |= DMA_IFCR_CTCIF7; // Clear flag
	}

	if (DMA1->ISR & DMA_ISR_TEIF7)
	{
		error_handler();
	}
}