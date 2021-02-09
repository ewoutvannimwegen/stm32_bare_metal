#include "stm32l476xx.h"

#define BUF_SIZE 20
#define BAUD 9600

uint16_t raw_buf[BUF_SIZE];
float cal_buf[BUF_SIZE];

void clk_init(void);
void io_init(void);
void adc1_init(void);
void dma1_ch1_init(void);
void dma1_ch7_init(void);
void usart2_init(void);
void error_handler(void);

int main(void)
{
	clk_init();
	io_init();
	adc1_init();
    dma1_ch1_init();
	dma1_ch7_init();
	usart2_init();

	while (1)
		;
}

void clk_init(void)
{
    /* HSI */
    RCC->CR |= RCC_CR_HSION; // HSI clock enabled
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ; // Wait for HSI ready

    /* Power */
    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
    PWR->CR1 &= ~PWR_CR1_VOS_1;
    PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1

    /* Flash */
    FLASH->ACR |= FLASH_ACR_ICEN;   // Instruction cache enabled
    FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
    FLASH->ACR |= FLASH_ACR_DCEN;   // Data cache enabled
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS; // Flash latency 3 CPU cycles

    /* PLL */
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // HSI -> PLL
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3;     // PLLN/8
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLR_1;     // PLLR/2
    RCC->CR |= RCC_CR_PLLON;                // PLL enabled
    while (!(RCC->CR & RCC_CR_PLLON))
        ;                               // Wait till PLL is locked
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // PLLCLK enabled
    RCC->CFGR |= RCC_CFGR_SW_PLL;       // PLLCLK -> SYSCLK
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))
        ; // Wait till PLL is configured as SYSCLK

    /* SAI1 PLL */
    RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1N_3; // PLLSAI1N/8
    RCC->PLLSAI1CFGR |=
        RCC_PLLSAI1CFGR_PLLSAI1R_1 | RCC_PLLSAI1CFGR_PLLSAI1R_0; // PLLSAI1R/8
    RCC->CR |= RCC_CR_PLLSAI1ON;                                 // SAI1 PLL enabled
    while (!(RCC->CR & RCC_CR_PLLSAI1RDY))
        ;                                           // Wait till SAI1 PLL is locked
    RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN; // PLLSAI1R -> PLLADC1CLK
    RCC->CCIPR &= ~RCC_CCIPR_ADCSEL_1;
    RCC->CCIPR |= RCC_CCIPR_ADCSEL_0; // PLLSAI1R selected as ADC clock

    SystemCoreClockUpdate();

    /* Peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; 
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
}

void io_init(void)
{
    /* ADC1 INP5 [PA0] */
    GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE0_0; // Analog mode
    GPIOA->ASCR |= GPIO_ASCR_ASC0;                           // Connect analog switch to ADC input

    /* ADC1 INP6 [PA1] */
    GPIOA->MODER |= GPIO_MODER_MODE1_1 | GPIO_MODER_MODE1_0; // Analog mode
    GPIOA->ASCR |= GPIO_ASCR_ASC1;   

	/* USART2 TX [PA2] */
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;									  // Output push-pull
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2_1 | GPIO_OSPEEDR_OSPEED2_0); // Low speed
	GPIOA->MODER |= GPIO_MODER_MODE2_1;
	GPIOA->MODER &= ~GPIO_MODER_MODE2_0; // Alternate function
	GPIOA->AFR[0] = 0;
	GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_0; // AF7: USART2_TX
}

void adc1_init(void)
{
    ADC1->CR = 0;                // Clear ADC1 control register
    ADC1->CR |= ADC_CR_ADVREGEN; // ADC voltage regulator enabled
    for (int i = 0; i < (SystemCoreClock / 8); i++)
        ; // Start-up time regulator

    /* Rank 1: INP5 */
    ADC1->SMPR1 |=
        ADC_SMPR1_SMP5_2 | ADC_SMPR1_SMP5_1 | ADC_SMPR1_SMP5_0; // 640.5 cycles
    ADC1->SQR1 = 0;                                             // Clear conversion sequence register
    ADC1->SQR1 |= ADC_SQR1_SQ1_2 | ADC_SQR1_SQ1_0;              // 1st conversion is INP5

    /* Rank 2: INP6 */
    ADC1->SMPR1 |=
        ADC_SMPR1_SMP5_2 | ADC_SMPR1_SMP5_1 | ADC_SMPR1_SMP5_0; // 640.5 cycles
    ADC1->SQR1 |= ADC_SQR1_SQ2_2 | ADC_SQR1_SQ2_1;              // 2nd conversion is INP6

    ADC1->SQR1 |= ADC_SQR1_L_0; // 2 conversion ranks

    ADC1->IER |= ADC_IER_ADRDYIE; // ADC ready interrupt enabled
    NVIC_EnableIRQ(ADC1_2_IRQn);

    ADC1->CFGR = 0; 
    ADC1->CFGR |= ADC_CFGR_CONT; // Continuous conversion mode
    ADC1->CFGR |= ADC_CFGR_OVRMOD; // Overrun enabled
    ADC1->CFGR |= ADC_CFGR_DMAEN; // DMA enabled 
    ADC1->CFGR |= ADC_CFGR_DMACFG; // Circular mode

    ADC1->CR |= ADC_CR_ADCAL; // Calibrate ADC1
    while (ADC1->CR & ADC_CR_ADCAL)
        ; // Wait till calibration is finished

    ADC1->ISR |= ADC_ISR_ADRDY;    // Clear ADC ready bit
    ADC1->CR |= ADC_CR_ADEN;       // ADC1 enabled
}

void dma1_ch1_init(void)
{
    DMA1_Channel1->CCR = 0;             // Reset control register
    DMA1_Channel1->CCR |= DMA_CCR_PL_1; // Priority high
    DMA1_Channel1->CCR |=
        DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0; // 16-bits peripheral & memory size
    DMA1_Channel1->CCR |= DMA_CCR_MINC;    // Memory increment mode enabled
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;    // Circular mode enabled

    DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S; // Select ADC1 for DMA1 channel 1

    DMA1_Channel1->CCR |= DMA_CCR_HTIE; // Half complete ISR enabled
    DMA1_Channel1->CCR |= DMA_CCR_TCIE; // Transfer complete ISR enabled
    DMA1_Channel1->CCR |= DMA_CCR_TEIE; // Transfer error ISR enabled
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    DMA1_Channel1->CNDTR = BUF_SIZE;               // Data size
    DMA1_Channel1->CPAR = (uint32_t) & (ADC1->DR); // Peripheral address
    DMA1_Channel1->CMAR = (uint32_t)&raw_buf[0];     // Memory address

    DMA1_Channel1->CCR |= DMA_CCR_EN; // Channel1 enabled
}

void dma1_ch7_init(void)
{
	DMA1_Channel7->CCR = 0;				// Reset control register
	DMA1_Channel7->CCR |= DMA_CCR_PL_1; // Priority high
	DMA1_Channel7->CCR |= DMA_CCR_MINC; // Memory increment mode enabled
	DMA1_Channel7->CCR |= DMA_CCR_CIRC; // Circular mode enabled
	DMA1_Channel7->CCR |= DMA_CCR_DIR;	// Memory->Peripheral
	DMA1_Channel7->CCR |= DMA_CCR_MSIZE_0; // 16-bits memory size

	DMA1_CSELR->CSELR |= (2 << DMA_CSELR_C7S_Pos); // Select USART2 TX

	DMA1_Channel7->CCR |= DMA_CCR_TCIE; // Transfer complete ISR enabled
	DMA1_Channel7->CCR |= DMA_CCR_TEIE; // Transfer error ISR enabled
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	DMA1_Channel7->CNDTR = BUF_SIZE;				  // Data size
	DMA1_Channel7->CPAR = (uint32_t) & (USART2->TDR); // Peripheral address
	DMA1_Channel7->CMAR = (uint32_t)&raw_buf[0];	  // Memory address
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

void ADC1_2_IRQHandler(void)
{
    if (ADC1->ISR & ADC_ISR_ADRDY)
    {
        ADC1->ISR |= ADC_ISR_ADRDY; // Clear ADC ready flag
        ADC1->CR |= ADC_CR_ADSTART; // Start AD-Conversion
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_HTIF1)
    {
        for (int i = 0; i < (BUF_SIZE / 2); i++)
        {
            cal_buf[i] = (raw_buf[i]*3.3)/4095; // Calibrate the 1st half
        }
    }

    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        for (int i = (BUF_SIZE / 2); i < BUF_SIZE; i++)
        {
            cal_buf[i] = (raw_buf[i]*3.3)/4095; // Calibrate the 2nd half
        }
    }

    if (DMA1->ISR & DMA_ISR_TEIF1)
    {
        error_handler();
    }
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