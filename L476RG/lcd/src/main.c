#include "main.h"

uint32_t date_reg = 0;     // Store copy of date register
uint32_t time_reg = 0;     // Store copy of time register
uint8_t date[5];           // LCD date buffer
uint8_t time[8];           // LCD time buffer
uint8_t ser_rx[SER_RX_S];  // Serial receive buffer
int ser_i = 0;             // Index of serial buffer

void sync_cal(void);
void sync_lcd(void);
void delay_us(int us);
void wakeup(void);
void alarm_a(void);
void trigger_lcd();
void lcd_write_8(uint8_t data, int rs);
void lcd_write_str(uint8_t data[], int size);
void move_cursor(int step);
uint8_t m_bits(uint8_t val);
uint8_t l_bits(uint8_t val);

int main(void) {
    date[2] = '/';
    time[2] = ':';
    time[5] = ':';

    clk_init();
    rtc_init();
    io_init();
    usart2_init();
    lcd_init();
    lcd_write_8(12, 0);  // Display on, cursor off
    lcd_write_8(6, 0);   // Entry mode

    while (1)
        ;
}

void wakeup(void) {
    while (!(RTC->ISR & RTC_ISR_WUTWF))
        ;                            // Wait till update allowed
    RTC->CR |= RTC_CR_WUCKSEL_2;     // 1Hz
    RTC->WUTR = 1;                   // Auto-reload value
    RTC->ISR &= ~RTC_ISR_WUTWF;      // Wakeup update not allowed
    EXTI->IMR1 |= EXTI_IMR1_IM20;    // Event request not masked
    EXTI->RTSR1 |= EXTI_RTSR1_RT20;  // Rising edge trigger enabled
    RTC->CR |= RTC_CR_WUTIE;         // Wakeup timer interrupt enabled
    NVIC_EnableIRQ(RTC_WKUP_IRQn);
    RTC->CR |= RTC_CR_WUTE;  // Wakeup timer enabled
}

void alarm_a(void) {
    while (!(RTC->ISR & RTC_ISR_ALRAWF))
        ;                            // Wait till update allowed
    RTC->ALRMAR = 0;                 // Clear Alarm A register
    RTC->ALRMAR |= RTC_ALRMAR_MSK4;  // Day don't care
    RTC->ALRMAR |= RTC_ALRMAR_MSK3;  // Hours don't care
    RTC->ALRMAR |= RTC_ALRMAR_MSK2;  // Minutes don't care
    RTC->ISR &= ~RTC_ISR_ALRAWF;     // Alarm A update not allowed
    EXTI->IMR1 |= EXTI_IMR1_IM18;    // Event request not masked
    EXTI->RTSR1 |= EXTI_RTSR1_RT18;  // Rising edge trigger enabled
    RTC->CR |= RTC_CR_ALRAIE;        // Alarm A interrupt enabled
    NVIC_EnableIRQ(RTC_Alarm_IRQn);
    RTC->CR |= RTC_CR_ALRAE;  // Alarm A enabled
}

void sync_cal(void) {
    RTC->ISR &= ~(RTC_ISR_RSF);  // Clear the sync flag
    while (!(RTC->ISR & RTC_ISR_RSF))
        ;  // Wait till sync
}

void sync_lcd(void) {
    lcd_write_8(1, 0);  // Clear display

    date_reg = RTC->DR;  // Copy date register
    sync_cal();
    time_reg = RTC->TR;  // Copy time register
    sync_cal();

    date[0] = m_bits((uint8_t)(date_reg)) + 48;  // Day tens
    date[1] = l_bits((uint8_t)(date_reg)) + 48;  // Day units
    date[3] =
        (m_bits(((uint8_t)(date_reg >> 8)) << 3) >> 3) + 48;  // Month tens
    date[4] = l_bits((uint8_t)(date_reg >> 8)) + 48;          // Month units

    lcd_write_str(date, sizeof(date));  // Send date to LCD
    move_cursor(7);                     // Move cursor 7 steps to the right

    time[0] = m_bits((uint8_t)(time_reg >> 16)) + 48;  // Hour tens
    time[1] = l_bits((uint8_t)(time_reg >> 16)) + 48;  // Hour units
    time[3] = m_bits((uint8_t)(time_reg >> 8)) + 48;   // Min tens
    time[4] = l_bits((uint8_t)(time_reg >> 8)) + 48;   // Min units
    time[6] = m_bits((uint8_t)(time_reg)) + 48;        // Sec tens
    time[7] = l_bits((uint8_t)(time_reg)) + 48;        // Sec units
    lcd_write_str(time, sizeof(time));                 // Send time to LCD
}

void delay_us(int us) {
    for (uint32_t i = 0; i < (us * (SystemCoreClock / 8000000)); i++)
        ;
}

void trigger_lcd() {
    GPIOB->ODR |= GPIO_ODR_OD10;  // LCD trigger enabled
    delay_us(1);
    GPIOB->ODR &= ~GPIO_ODR_OD10;  // LCD trigger disabled
    delay_us(1);
}

void lcd_write_8(uint8_t data, int rs) {
    PORT_RS &= ~RS;
    if (rs >= 1) PORT_RS |= RS;

    PORT_DB7 &= ~DB7;
    if (data & 1 << 7) PORT_DB7 |= DB7;
    PORT_DB6 &= ~DB6;
    if (data & 1 << 6) PORT_DB6 |= DB6;
    PORT_DB5 &= ~DB5;
    if (data & 1 << 5) PORT_DB5 |= DB5;
    PORT_DB4 &= ~DB4;
    if (data & 1 << 4) PORT_DB4 |= DB4;

    trigger_lcd();

    PORT_DB7 &= ~DB7;
    if (data & 1 << 3) PORT_DB7 |= DB7;
    PORT_DB6 &= ~DB6;
    if (data & 1 << 2) PORT_DB6 |= DB6;
    PORT_DB5 &= ~DB5;
    if (data & 1 << 1) PORT_DB5 |= DB5;
    PORT_DB4 &= ~DB4;
    if (data & 1 << 0) PORT_DB4 |= DB4;

    trigger_lcd();
    PORT_RS &= ~RS;
    delay_us(1000);
}

void lcd_write_str(uint8_t data[], int size) {
    for (int i = 0; i < size; i++) lcd_write_8(data[i], 1);
}

void move_cursor(int step) {
    for (int i = 0; i < step; i++) lcd_write_8(20, 0);
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
