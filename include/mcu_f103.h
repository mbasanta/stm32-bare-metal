// STM32F103 (Blue Pill) Configuration
#pragma once

#include "stm32f103xb.h"

// Hardware configuration
#define MCU_NAME "STM32F103 (Blue Pill)"
#define FREQ_HZ 72000000UL  // 72 MHz
#define HSE_FREQ_HZ 8000000UL  // 8MHz external crystal

// Bus clocks
#define APB1_FREQ_HZ (FREQ_HZ / 2)  // 36 MHz
#define APB2_FREQ_HZ (FREQ_HZ)      // 72 MHz

// LED pin (PA5 on Nucleo, or PC13 on Blue Pill)
#define LED_PIN PIN('C', 13)
#define LED_ACTIVE_LOW 1  // Blue Pill LED is active-low

// UART configuration
#define UART1_TX_PIN PIN('A', 9)
#define UART1_RX_PIN PIN('A', 10)
#define UART2_TX_PIN PIN('A', 2)
#define UART2_RX_PIN PIN('A', 3)
#define UART3_TX_PIN PIN('B', 10)
#define UART3_RX_PIN PIN('B', 11)

// Clock initialization
static inline void clock_init(void) {
    // Enable HSE (external 8MHz crystal)
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) {}

    // Configure Flash latency: 2 wait states required for 72MHz
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    // Configure PLL: HSE × 9 = 8MHz × 9 = 72MHz
    RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}

    // Set APB1 prescaler to /2 (max 36MHz for APB1)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Switch system clock to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}
}
