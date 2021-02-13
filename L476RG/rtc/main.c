#include "stm32l476xx.h"

void clk_init(void);
void rtc_init(void);
void io_init(void);
void sync_cal(void);

int main(void)
{
    volatile uint32_t p_time = 0;
    volatile uint32_t time = 0;
    volatile uint32_t date = 0;

    clk_init();
    rtc_init();
    io_init();

    while (1)
    {
        time = RTC->TR; // Get time
        sync_cal();
        date = RTC->DR; // Get date
        sync_cal();

        if (time > p_time + 5)
        {
            GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
            p_time = time;
        }

        for (int i = 0; i < (SystemCoreClock / 8); i++)
            ; // Sleep 1 sec
    }
}

void clk_init(void)
{
    RCC->CR |= RCC_CR_HSION; // HSI clock enabled
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ; // Wait for HSI ready

    RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Power interface clock enabled
    PWR->CR1 &= ~PWR_CR1_VOS_1;
    PWR->CR1 |= PWR_CR1_VOS_0; // Voltage scaling range 1

    FLASH->ACR |= FLASH_ACR_ICEN;   // Instruction cache enabled
    FLASH->ACR |= FLASH_ACR_PRFTEN; // Prefetch enabled
    FLASH->ACR |= FLASH_ACR_DCEN;   // Data cache enabled
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_2WS; // Flash latency 3 CPU cycles

    PWR->CR1 |= PWR_CR1_DBP; // Access the RTC and backup registers enabled
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53; // RTC write protection unlocked

    RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI as system clock
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSI))
        ;

    RCC->BDCR |= RCC_BDCR_LSEON; // LSE oscillator enabled
    while (!(RCC->BDCR & RCC_BDCR_LSERDY))
        ; // Wait till LSE ready

    RCC->BDCR |= RCC_BDCR_RTCSEL_0; // LSE->RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;

    SystemCoreClockUpdate();

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
}

void rtc_init(void)
{
    RTC->ISR |= RTC_ISR_INIT; // Init mode enabled
    while (!(RTC->ISR & RTC_ISR_INITF))
        ; // Wait till init mode ready

    RTC->PRER = 0;                               // Clear prescalers
    RTC->PRER |= (127 << RTC_PRER_PREDIV_A_Pos); // Async prescaler: 127+1=128
    RTC->PRER |= (255 << RTC_PRER_PREDIV_S_Pos); // Sync prescaler: 255+1=256

    RTC->TR = 0;               // Clear time register
    RTC->DR |= RTC_DR_YT_1;    // Year tens: 2
    RTC->DR |= RTC_DR_YU_0;    // Years units: 1
    RTC->DR |= RTC_DR_WDU_0;   // Week day: monday
    RTC->DR |= RTC_DR_MU_0;    // Month: 1
    RTC->DR |= RTC_DR_DU_0;    // Day: 1
    RTC->ISR &= ~RTC_ISR_INIT; // Init mode disabled
    sync_cal();
}

void io_init(void)
{
    /* LD2 [PA5] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;                                    // Initial state LOW
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output
}

void sync_cal(void)
{
    RTC->ISR &= ~(RTC_ISR_RSF); // Clear the sync flag
    while (!(RTC->ISR & RTC_ISR_RSF))
        ; // Wait till sync
}