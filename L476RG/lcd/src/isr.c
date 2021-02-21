#include "isr.h"

#include "main.h"

void RTC_WKUP_IRQHandler(void) {
    if (RTC->ISR & RTC_ISR_WUTF) {
        RTC->ISR &= ~RTC_ISR_WUTF;  // Clear flag
        sync_lcd();
    }
}

void RTC_Alarm_IRQHandler(void) {
    if (RTC->ISR & RTC_ISR_ALRAF) {
        RTC->ISR &= ~RTC_ISR_ALRAF;  // Clear Alarm A interrupt flag
        GPIOA->ODR ^= GPIO_ODR_OD5;  // Toggle LD2
    }
}

void USART2_IRQHandler(void) {
    if (USART2->ISR & USART_ISR_RXNE) {
        uint8_t val = USART2->RDR;  // Copy RX buffer to val
        USART2->TDR = val;          // Copy val to TX buffer
        while (!(USART2->ISR & USART_ISR_TC))
            ;  // Wait till transmission complete
    }
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR1 & EXTI_PR1_PIF13) {
        EXTI->PR1 |= EXTI_PR1_PIF13;  // Clear flag
        GPIOA->ODR ^= GPIO_ODR_OD5;   // Toggle LD2
        // TODO if button pressed update time and date registers of LCD
    }
}
