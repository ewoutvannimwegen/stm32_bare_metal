#include "stm32l476xx.h"
#include "main.h"

uint32_t date_reg = 0; // Store copy of date register
uint32_t time_reg = 0; // Store copy of time register
uint8_t date[5]; 
uint8_t time[8]; 

void clk_init(void);
void rtc_init(void);
void wakeup(void);
void alarm_a(void);
void io_init(void);
void lcd_init(void);
void sync_cal(void);
void sync_lcd(void);
void delay_us(int us);
void trigger_lcd();
void lcd_write_8(uint8_t data, int rs);
void lcd_write_str(uint8_t data[], int size);
void move_cursor(int step);
uint8_t m_bits(uint8_t val);
uint8_t l_bits(uint8_t val);

int main(void)
{    
    date[2] = ':';
    time[2] = ':';
    time[5] = ':';

    clk_init();
    rtc_init();
    io_init();
    lcd_init();
    lcd_write_8(12, 0); // Display on, cursor off
    lcd_write_8(6, 0); // Entry mode

    while (1);
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

    RCC->CFGR |= RCC_CFGR_SW_HSI; // HSI->SYSCLK
    while (!(RCC->CFGR & RCC_CFGR_SWS_HSI))
        ;

    RCC->BDCR |= RCC_BDCR_LSEON; // LSE oscillator enabled
    while (!(RCC->BDCR & RCC_BDCR_LSERDY))
        ; // Wait till LSE ready

    RCC->BDCR |= RCC_BDCR_RTCSEL_0; // LSE->RTC
    RCC->BDCR |= RCC_BDCR_RTCEN;

    SystemCoreClockUpdate();

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
}

void rtc_init(void)
{
    RTC->ISR |= RTC_ISR_INIT; // Init mode enabled
    while (!(RTC->ISR & RTC_ISR_INITF))
        ;                                        // Wait till init mode ready
    RTC->CR = 0;                                 // Clear control register
    RTC->PRER = 0;                               // Clear prescalers
    RTC->PRER |= (127 << RTC_PRER_PREDIV_A_Pos); // Async prescaler: 127+1=128
    RTC->PRER |= (255 << RTC_PRER_PREDIV_S_Pos); // Sync prescaler: 255+1=256
    RTC->CR |= RTC_CR_COSEL;                     // Calibration output is 1Hz

    //TODO fix years can't set these.
    RTC->TR = 0;             // Clear time register
    RTC->DR |= RTC_DR_YT_1;  // Year tens: 2
    RTC->DR |= RTC_DR_YU_0;  // Years units: 1
    RTC->DR |= RTC_DR_WDU_0; // Week day: monday
    RTC->DR |= RTC_DR_MU_0;  // Month: 1
    RTC->DR |= RTC_DR_DU_0;  // Day: 1
    
    wakeup();
    alarm_a();

    RTC->ISR &= ~RTC_ISR_INIT; // Init mode disabled
    sync_cal();
}

void wakeup(void) {
    while(!(RTC->ISR & RTC_ISR_WUTWF)); // Wait till update allowed
    RTC->CR |= RTC_CR_WUCKSEL_2; // 1Hz   
    RTC->WUTR = 1; // Auto-reload value
    RTC->ISR &= ~RTC_ISR_WUTWF; // Wakeup update not allowed
    EXTI->IMR1 |= EXTI_IMR1_IM20;   // Event request not masked
    EXTI->RTSR1 |= EXTI_RTSR1_RT20; // Rising edge trigger enabled
    RTC->CR |= RTC_CR_WUTIE; // Wakeup timer interrupt enabled
    NVIC_EnableIRQ(RTC_WKUP_IRQn); 
    RTC->CR |= RTC_CR_WUTE; // Wakeup timer enabled  
}

void alarm_a(void)
{
    while(!(RTC->ISR & RTC_ISR_ALRAWF)); // Wait till update allowed
    RTC->ALRMAR = 0;                // Clear Alarm A register
    RTC->ALRMAR |= RTC_ALRMAR_MSK4; // Day don't care
    RTC->ALRMAR |= RTC_ALRMAR_MSK3; // Hours don't care
    RTC->ALRMAR |= RTC_ALRMAR_MSK2; // Minutes don't care
    RTC->ISR &= ~RTC_ISR_ALRAWF;    // Alarm A update not allowed
    EXTI->IMR1 |= EXTI_IMR1_IM18;   // Event request not masked
    EXTI->RTSR1 |= EXTI_RTSR1_RT18; // Rising edge trigger enabled
    RTC->CR |= RTC_CR_ALRAIE; // Alarm A interrupt enabled
    NVIC_EnableIRQ(RTC_Alarm_IRQn);
    RTC->CR |= RTC_CR_ALRAE; // Alarm A enabled
}

void io_init(void)
{
    /* LED: LD2 [PA5] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;                                    // Initial state LOW
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOA->MODER |= GPIO_MODER_MODE5_0; // Output

    /* DB7: D2 [PA10] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT10;                                     // Initial state LOW
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0); // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE10_1;
    GPIOA->MODER |= GPIO_MODER_MODE10_0; // Output

    /* DB6: D3 [PB3] */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT3;                                    // Initial state LOW
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3_1 | GPIO_OSPEEDR_OSPEED3_0); // Low speed
    GPIOB->MODER &= ~GPIO_MODER_MODE3_1;
    GPIOB->MODER |= GPIO_MODER_MODE3_0; // Output

    /* DB5: D4 [PB5] */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT5;                                    // Initial state LOW
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_1 | GPIO_OSPEEDR_OSPEED5_0); // Low speed
    GPIOB->MODER &= ~GPIO_MODER_MODE5_1;
    GPIOB->MODER |= GPIO_MODER_MODE5_0; // Output

    /* DB4: D5 [PB4] */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT4;                                    // Initial state LOW
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4_1 | GPIO_OSPEEDR_OSPEED4_0); // Low speed
    GPIOB->MODER &= ~GPIO_MODER_MODE4_1;
    GPIOB->MODER |= GPIO_MODER_MODE4_0; // Output

    /* E: D6 [PB10] */
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT10;                                     // Initial state LOW
    GPIOB->OSPEEDR |=  GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED10_0; // Very high speed
    GPIOB->MODER &= ~GPIO_MODER_MODE10_1;
    GPIOB->MODER |= GPIO_MODER_MODE10_0; // Output

    /* RS: D7 [PA8] */
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT8;                                    // Initial state LOW
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8_1 | GPIO_OSPEEDR_OSPEED8_0); // Low speed
    GPIOA->MODER &= ~GPIO_MODER_MODE8_1;
    GPIOA->MODER |= GPIO_MODER_MODE8_0; // Output
}

void lcd_init(void)
{
    delay_us(50000); // Vcc rises to 4.5V
    lcd_write_8(48, 0); 
    delay_us(4100);
    lcd_write_8(48, 0); 
    lcd_write_8(48, 0); 
    lcd_write_8(32, 0); 
    lcd_write_8(40, 0); // Set 4-bit instructions
    lcd_write_8(8, 0); // Display off
    lcd_write_8(1, 0); // Clear display
    lcd_write_8(6, 0); // Entry mode
}

void sync_cal(void)
{
    RTC->ISR &= ~(RTC_ISR_RSF); // Clear the sync flag
    while (!(RTC->ISR & RTC_ISR_RSF))
        ; // Wait till sync
}

void sync_lcd(void) {
    lcd_write_8(1, 0); // Clear display

    date_reg = RTC->DR; // Copy date register
    sync_cal();
    time_reg = RTC->TR; // Copy time register
    sync_cal();

    date[0] = m_bits((uint8_t)(date_reg))+48; // Day tens
    date[1] = l_bits((uint8_t)(date_reg))+48; // Day units
    date[3] = (m_bits(((uint8_t)(date_reg >> 8)) << 3) >> 3)+48; // Month tens
    date[4] = l_bits((uint8_t)(date_reg >> 8))+48; // Month units

    lcd_write_str(date, sizeof(date)); // Send date to LCD
    move_cursor(7); // Move cursor 7 steps to the right

    time[0] = m_bits((uint8_t)(time_reg >> 16))+48; // Hour tens
    time[1] = l_bits((uint8_t)(time_reg >> 16))+48; // Hour units
    time[3] = m_bits((uint8_t)(time_reg >> 8))+48; // Min tens
    time[4] = l_bits((uint8_t)(time_reg >> 8))+48; // Min units
    time[6] = m_bits((uint8_t)(time_reg))+48; // Sec tens
    time[7] = l_bits((uint8_t)(time_reg))+48; // Sec units
    lcd_write_str(time, sizeof(time)); // Send time to LCD
}

void delay_us(int us)
{
    for (uint32_t i = 0; i < (us*(SystemCoreClock/8000000)); i++)
        ;
}

void trigger_lcd() {
    GPIOB->ODR |= GPIO_ODR_OD10; // LCD trigger enabled
    delay_us(1);
    GPIOB->ODR &= ~GPIO_ODR_OD10; // LCD trigger disabled
    delay_us(1);
}

void lcd_write_8(uint8_t data, int rs) {
    PORT_RS &= ~RS;
    if(rs >= 1) PORT_RS |= RS;

    PORT_DB7 &= ~DB7;
    if(data & 1<<7) PORT_DB7 |= DB7;
    PORT_DB6 &= ~DB6;
    if(data & 1<<6) PORT_DB6 |= DB6;
    PORT_DB5 &= ~DB5;
    if(data & 1<<5) PORT_DB5 |= DB5;
    PORT_DB4 &= ~DB4;
    if(data & 1<<4) PORT_DB4 |= DB4;
    
    trigger_lcd();

    PORT_DB7 &= ~DB7;
    if(data & 1<<3) PORT_DB7 |= DB7;
    PORT_DB6 &= ~DB6;
    if(data & 1<<2) PORT_DB6 |= DB6;
    PORT_DB5 &= ~DB5;
    if(data & 1<<1) PORT_DB5 |= DB5;
    PORT_DB4 &= ~DB4;
    if(data & 1<<0) PORT_DB4 |= DB4;

    trigger_lcd();
    PORT_RS &= ~RS;
    delay_us(1000);
}

void lcd_write_str(uint8_t data[], int size) {
    for(int i = 0; i < size; i++) lcd_write_8(data[i], 1);
}

void move_cursor(int step) {
    for(int i = 0; i < step; i++) lcd_write_8(20, 0); 
}

uint8_t m_bits(uint8_t val) {
    val = (val >> 4);
    return val;
}

uint8_t l_bits(uint8_t val) {
    val = (val << 4);
    val = (val >> 4);
    return val;
}

void RTC_WKUP_IRQHandler(void) {
    if(RTC->ISR & RTC_ISR_WUTF) {
        RTC->ISR &= ~RTC_ISR_WUTF; // Clear flag
        sync_lcd();
    }
}

void RTC_Alarm_IRQHandler(void)
{
    if (RTC->ISR & RTC_ISR_ALRAF)
    {
        RTC->ISR &= ~RTC_ISR_ALRAF; // Clear Alarm A interrupt flag
        GPIOA->ODR ^= GPIO_ODR_OD5; // Toggle LD2
    }
}

