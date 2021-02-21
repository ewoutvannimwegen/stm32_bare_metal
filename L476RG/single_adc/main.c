#include "stm32l476xx.h"

uint16_t val; // ADC data variable

void clk_init(void);
void io_init(void);
void adc1_init(void);

int main(void) {
	clk_init();
	io_init();
	adc1_init();

	while(1);
}

void clk_init(void) {
	/* HSI */
	RCC->CR |= RCC_CR_HSION; // HSI clock enabled
	while(!(RCC->CR & RCC_CR_HSIRDY)); // Wait for HSI ready

	/* Power */
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
	PWR->CR1 &= ~PWR_CR1_VOS_1; 
	PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1 

	/* Flash */
	FLASH->ACR |= FLASH_ACR_ICEN; // Instruction cache enabled
	FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
	FLASH->ACR |= FLASH_ACR_DCEN; // Data cache enabled
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2WS; // Flash latency 3 CPU cycles

	/* PLL */
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // HSI -> PLL
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3; // PLLN/8
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLR_1; // PLLR/2
	RCC->CR |= RCC_CR_PLLON; // PLL enabled
	while(!(RCC->CR & RCC_CR_PLLON)); // Wait till PLL is locked
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // PLLCLK enabled
	RCC->CFGR |= RCC_CFGR_SW_PLL; // PLLCLK -> SYSCLK
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); // Wait till PLL is configured as SYSCLK

	/* SAI1 PLL */
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1N_3; // PLLSAI1N/8
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1R_1 | RCC_PLLSAI1CFGR_PLLSAI1R_0; // PLLSAI1R/8
	RCC->CR |= RCC_CR_PLLSAI1ON; // SAI1 PLL enabled
	while(!(RCC->CR & RCC_CR_PLLSAI1RDY)); // Wait till SAI1 PLL is locked
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN; // PLLSAI1R -> PLLADC1CLK
	RCC->CCIPR &= ~RCC_CCIPR_ADCSEL_1;
	RCC->CCIPR |= RCC_CCIPR_ADCSEL_0; // PLLSAI1R selected as ADC clock

	SystemCoreClockUpdate(); 

	/* Peripheral clocks */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
}

void io_init(void) {
	/* LD2 [PA5] */
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
	GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output
	
	/* ADC1 INP5 [PA0] */
	GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0; // Analog mode
	GPIOA->ASCR |= GPIO_ASCR_ASC0; // Connect analog switch to ADC input

	/* User button [PC13] */
	GPIOC->IDR |= GPIO_PUPDR_PUPD13_1;
	GPIOC->IDR &= ~GPIO_PUPDR_PUPD13_0; // Pull-down
	GPIOC->MODER &= ~(GPIO_MODER_MODE13_1 | GPIO_MODER_MODE13_0); // Input

	/* EXTI13 ISR for User button [PC13] */
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
	SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

	EXTI->IMR1 |= EXTI_IMR1_IM13; 
	EXTI->RTSR1 |= EXTI_RTSR1_RT13; // Rising edge trigger
}

void adc1_init(void) {
	/* Voltage regulator */
	ADC1->CR = 0; // Clear ADC1 control register
	ADC1->CR |= ADC_CR_ADVREGEN; // ADC voltage regulator enabled
	for(int i = 0; i < (SystemCoreClock/8); i++); // Start-up time regulator

	/* INP5 */
	ADC1->SMPR1 |= ADC_SMPR1_SMP5_2 | ADC_SMPR1_SMP5_1 | ADC_SMPR1_SMP5_0; // 640.5 cycles
	ADC1->SQR1 = 0; // Clear conversion sequence register
	ADC1->SQR1 |= ADC_SQR1_SQ1_2 | ADC_SQR1_SQ1_0; // 1st conversion is INP5

	/* Interrupts */
	ADC1->IER |= ADC_IER_ADRDYIE; // ADC ready interrupt enabled
	ADC1->IER |= ADC_IER_EOCIE; // End of regular conversion interrupt enabled
	NVIC_EnableIRQ(ADC1_2_IRQn);

	/* Calibration */
	ADC1->CR |= ADC_CR_ADCAL; // Calibrate ADC1
	while(ADC1->CR & ADC_CR_ADCAL); // Wait till calibration is finished

	/* Enable ADC1 */
	ADC1->CFGR |= ADC_CFGR_CONT | ADC_CFGR_OVRMOD; // Continuous conversion; Overrun enabled
	ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADC ready bit
	ADC1->CR |= ADC_CR_ADEN; // ADC1 enabled
}

void EXTI15_10_IRQHandler(void) {
	if(EXTI->PR1 & EXTI_PR1_PIF13) {
		EXTI->PR1 |= EXTI_PR1_PIF13; // Clear flag
		GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
	}
}

void ADC1_2_IRQHandler(void) {
	if(ADC1->ISR & ADC_ISR_ADRDY) {
		ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADC ready bit
		ADC1->CR |= ADC_CR_ADSTART; // Start AD-Conversion
	}

	if(ADC1->ISR & ADC_ISR_EOC) {
		val = ADC1->DR; // Copy ADC data to variable
	}
}
