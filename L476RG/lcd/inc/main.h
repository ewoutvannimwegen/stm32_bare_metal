#ifndef L476RG_LCD_INC_MAIN_H_
#define L476RG_LCD_INC_MAIN_H_

#include "stm32l476xx.h"

#define BAUD 9600
#define SER_RX_S 10

#define PORT_DB7 GPIOA->ODR
#define PORT_DB6 GPIOB->ODR
#define PORT_DB5 GPIOB->ODR
#define PORT_DB4 GPIOB->ODR
#define PORT_E GPIOB->ODR
#define PORT_RS GPIOA->ODR

#define DB7 GPIO_ODR_OD10
#define DB6 GPIO_ODR_OD3
#define DB5 GPIO_ODR_OD5
#define DB4 GPIO_ODR_OD4
#define E GPIO_ODR_OD10
#define RS GPIO_ODR_OD8


void clk_init(void);
void rtc_init(void);
void wakeup(void);
void alarm_a(void);
void io_init(void);
void usart2_init(void);
void lcd_init(void);
void adc1_init(void);
void sync_cal(void);
void sync_lcd(void);
void delay_us(int us);
void trigger_lcd();
void lcd_write_8(uint8_t data, int rs);
void lcd_write_str(uint8_t data[], int size);
void move_cursor(int step);
uint8_t m_bits(uint8_t val);
uint8_t l_bits(uint8_t val);

#endif // L476RG_LCD_INC_MAIN_H_
