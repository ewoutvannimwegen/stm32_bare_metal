#include "init.h"

#include "main.h"

void clk_init(void) {
    RCC->CR |= RCC_CR_HSION;  // HSI clock enabled
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;  // Wait for HSI ready

    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;  // Power interface clock enabled
    PWR->CR1 &= ~PWR_CR1_VOS_1;
    PWR->CR1 |= PWR_CR1_VOS_0;  // Voltage scaling range 1

    FLASH->ACR |= FLASH_ACR_ICEN;    // Instruction cache enabled
    FLASH->ACR |= FLASH_ACR_PRFTEN;  // Prefetch enabled
    FLASH->ACR |= FLASH_ACR_DCEN;    // Data cache enabled
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS;  // Flash latency 3 CPU cycles

    PWR->CR1 |= PWR_CR1_DBP;  // Access the RTC and backup registers enabled
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;  // RTC write protection unlocked

    RCC->CFGR |= RCC_CFGR_SW_HSI;  // HSI->SYSCLK
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSI))
        ;

    RCC->BDCR |= RCC_BDCR_LSEON;  // LSE oscillator enabled
    while (!(RCC->BDCR & RCC_BDCR_LSERDY))
        ;  // Wait till LSE ready

    RCC->BDCR |= RCC_BDCR_RTCSEL_0;  // LSE->RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;

    SystemCoreClockUpdate();

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->AHB2ENR |=
        RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

void rtc_init(void) {
    RTC->ISR |= RTC_ISR_INIT;  // Init mode enabled
    while (!(RTC->ISR & RTC_ISR_INITF))
        ;                                         // Wait till init mode ready
    RTC->CR = 0;                                  // Clear control register
    RTC->PRER = 0;                                // Clear prescalers
    RTC->PRER |= (127 << RTC_PRER_PREDIV_A_Pos);  // Async prescaler: 127+1=128
    RTC->PRER |= (255 << RTC_PRER_PREDIV_S_Pos);  // Sync prescaler: 255+1=256
    RTC->CR |= RTC_CR_COSEL;                      // Calibration output is 1Hz

    // TODO fix years can't set these.
    RTC->TR = 0;              // Clear time register
    RTC->DR |= RTC_DR_YT_1;   // Year tens: 2
    RTC->DR |= RTC_DR_YU_0;   // Years units: 1
    RTC->DR |= RTC_DR_WDU_0;  // Week day: monday
    RTC->DR |= RTC_DR_MU_0;   // Month: 1
    RTC->DR |= RTC_DR_DU_0;   // Day: 1

    wakeup();
    alarm_a();

    RTC->ISR &= ~RTC_ISR_INIT;  // Init mode disabled
    sync_cal();
}

void io_init(void) {
    /* LED: LD2 [PA5] Digital output */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;  // Initial state LOW
    GPIOA->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0);  // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOA->MODER |= GPIO_MODER_MODE5_0;  // Output

    /* DB7: D2 [PA10] Digital output */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;  // Initial state LOW
    GPIOA->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0);  // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE10_1;
    GPIOA->MODER |= GPIO_MODER_MODE10_0;  // Output

    /* DB6: D3 [PB3] Digital output */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT3;  // Initial state LOW
    GPIOB->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED3_0);  // Low speed
    GPIOB->MODER &= ~GPIO_MODER_MODE3_1;
    GPIOB->MODER |= GPIO_MODER_MODE3_0;  // Output

    /* DB5: D4 [PB5] Digital output */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT5;  // Initial state LOW
    GPIOB->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0);  // Low speed
    GPIOB->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOB->MODER |= GPIO_MODER_MODE5_0;  // Output

    /* DB4: D5 [PB4] Digital output */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT4;  // Initial state LOW
    GPIOB->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED4_1 | GPIO_OSPEEDR_OSPEED4_0);  // Low speed
    GPIOB->MODER &= ~GPIO_MODER_MODE4_1;
    GPIOB->MODER |= GPIO_MODER_MODE4_0;  // Output

    /* E: D6 [PB10] Digital output */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;  // Initial state LOW
    GPIOB->OSPEEDR |=
        GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0;  // Very high speed
    GPIOB->MODER &= ~GPIO_MODER_MODE10_1;
    GPIOB->MODER |= GPIO_MODER_MODE10_0;  // Output

    /* RS: D7 [PA8] Digital output */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT8;  // Initial state LOW
    GPIOA->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED8_1 | GPIO_OSPEEDR_OSPEED8_0);  // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE8_1;
    GPIOA->MODER |= GPIO_MODER_MODE8_0;  // Output

    /* USART2 TX [PA2] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT2;  // Output push-pull
    GPIOA->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED2_1 | GPIO_OSPEEDR_OSPEED2_0);  // Low speed
    GPIOA->MODER |= GPIO_MODER_MODE2_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE2_0;  // Alternate function
    GPIOA->AFR[0] = 0;
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL2_2 | GPIO_AFRL_AFSEL2_1 |
                     GPIO_AFRL_AFSEL2_0;  // AF7: USART2_TX

    /* USART2 RX [PA3] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT3;  // Output push-pull
    GPIOA->OSPEEDR &=
        ~(GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED3_0);  // Low speed
    GPIOA->MODER |= GPIO_MODER_MODE3_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE3_0;  // Alternate function
    GPIOA->AFR[0] |= GPIO_AFRL_AFSEL3_2 | GPIO_AFRL_AFSEL3_1 |
                     GPIO_AFRL_AFSEL3_0;  // AF7: USART2_RX

    /* User button [PC13] */
    GPIOC->IDR |= GPIO_PUPDR_PUPD13_1;
    GPIOC->IDR &= ~GPIO_PUPDR_PUPD13_0;                            // Pull-down
    GPIOC->MODER &= ~(GPIO_MODER_MODE13_1 | GPIO_MODER_MODE13_0);  // Input
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR1 |= EXTI_IMR1_IM13;
    EXTI->RTSR1 |= EXTI_RTSR1_RT13;  // Rising trigger
}

void usart2_init(void) {
    uint16_t usartdiv = (SystemCoreClock / BAUD);  // Get baud rate divider
    USART2->BRR = usartdiv;                        // Set baud rate divider
    USART2->CR1 |= USART_CR1_RXNEIE;               // RX interrupt enabled;
    NVIC_EnableIRQ(USART2_IRQn);                   // Interrupt enabled
    USART2->ICR |= USART_ICR_TCCF;  // Clear transfer complete interrupt
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;  // USART2 TX & RX enabled
    USART2->CR1 |= USART_CR1_UE;                 // USART2 enabled
}

void lcd_init(void) {
    delay_us(50000);  // Vcc rises to 4.5V
    lcd_write_8(48, 0);
    delay_us(4100);
    lcd_write_8(48, 0);
    lcd_write_8(48, 0);
    lcd_write_8(32, 0);
    lcd_write_8(40, 0);  // Set 4-bit instructions
    lcd_write_8(8, 0);   // Display off
    lcd_write_8(1, 0);   // Clear display
    lcd_write_8(6, 0);   // Entry mode
}
