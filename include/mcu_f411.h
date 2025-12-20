// STM32F411 (Black Pill) Configuration
#pragma once

#include "stm32f411xe.h"

// Hardware configuration
#define MCU_NAME "STM32F411 (Black Pill)"
#define FREQ_HZ 96000000UL      // 96 MHz
#define HSE_FREQ_HZ 25000000UL  // 25MHz external crystal

// Bus clocks
#define APB1_FREQ_HZ (FREQ_HZ / 2)  // 48 MHz
#define APB2_FREQ_HZ (FREQ_HZ)      // 96 MHz

// LED pin (PC13 on Black Pill)
#define LED_PIN PIN('C', 13)
#define LED_ACTIVE_LOW 1  // Black Pill LED is active-low

// I2C configuration (optional)
#define I2C_SCL_PIN PIN('B', 6)
#define I2C_SDA_PIN PIN('B', 7)
#define I2C_AF 4  // AF4 for I2C1

// UART configuration
#define UART1_TX_PIN PIN('A', 9)
#define UART1_RX_PIN PIN('A', 10)
#define UART1_AF 7  // AF7 for USART1
#define UART2_TX_PIN PIN('A', 2)
#define UART2_RX_PIN PIN('A', 3)
#define UART2_AF 7  // AF7 for USART2i
#define UART6_TX_PIN PIN('A', 11)
#define UART6_RX_PIN PIN('A', 12)
#define UART6_AF 8  // AF8 for USART6

// Clock initialization
static inline void clock_init(void) {
    // Enable HSE (external 25MHz crystal on Black Pill)
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) {
    }

    // Configure Flash latency: 3 wait states required for 96MHz at 3.3V
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_3WS;

    // Configure PLL: HSE ÷ M × N ÷ P
    // 25MHz ÷ 25 = 1MHz (VCO input)
    // 1MHz × 192 = 192MHz (VCO output)
    // 192MHz ÷ 2 = 96MHz (system clock)
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |
                   (192 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) |  // PLLP = 2
                   (4 << RCC_PLLCFGR_PLLQ_Pos) | RCC_PLLCFGR_PLLSRC_HSE;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }

    // Set APB1 prescaler to /2 (max 50MHz for APB1)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Switch system clock to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }
}
