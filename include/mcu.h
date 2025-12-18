#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f411xe.h"

#define FREQ_HZ 96000000UL  // 96 MHz

// Clock configuration: HSE (25MHz on Black Pill) → PLL (÷25 ×192 ÷2) → 96MHz SYSCLK
static inline void clock_init(void) {
    // Enable HSE (external 25MHz crystal on Black Pill)
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) {}  // Wait for HSE ready

    // Configure Flash latency: 3 wait states required for 96MHz at 3.3V
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_3WS;

    // Configure PLL: HSE ÷ M × N ÷ P
    // 25MHz ÷ 25 = 1MHz (VCO input)
    // 1MHz × 192 = 192MHz (VCO output)
    // 192MHz ÷ 2 = 96MHz (system clock)
    // PLLM=25, PLLN=192, PLLP=2 (00), PLLQ=4, PLLSRC=HSE
    RCC->PLLCFGR = (25 << RCC_PLLCFGR_PLLM_Pos) |
                   (192 << RCC_PLLCFGR_PLLN_Pos) |
                   (0 << RCC_PLLCFGR_PLLP_Pos) |  // PLLP = 2
                   (4 << RCC_PLLCFGR_PLLQ_Pos) |
                   RCC_PLLCFGR_PLLSRC_HSE;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {}  // Wait for PLL ready

    // Set APB1 prescaler to /2 (max 50MHz for APB1)
    // Set APB2 prescaler to /1 (max 100MHz for APB2)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 = 48MHz

    // Switch system clock to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}  // Wait for switch
}

#define BIT(x) (1UL << (x))

typedef uint16_t gpio_pin_t;

// Encode pin as 0xBBNN → BB = bank (0=A,1=B,etc), NN = pin number (0–15)
#define PIN(bank, num) ((gpio_pin_t)((((bank) - 'A') << 8) | ((num) & 0xFF)))

#define PINNO(pin) ((uint8_t)((pin) & 0xFF))
#define PINBANK(pin) ((uint8_t)(((pin) >> 8) & 0xFF))

static inline void spin(volatile uint32_t count) {
    while (count--) __NOP();
}

static inline GPIO_TypeDef* gpio_from_bank(uint8_t bank) {
    static GPIO_TypeDef* const ports[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH,
    };

    return (bank < (sizeof(ports) / sizeof(ports[0]))) ? ports[bank]
                                                       : (GPIO_TypeDef*)0;
}

typedef enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_AF,
    GPIO_MODE_ANALOG,
} gpio_mode_t;

typedef enum {
    GPIO_OTYPE_PP = 0,  // Push-pull
    GPIO_OTYPE_OD = 1,  // Open-drain
} gpio_otype_t;

typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM = 1,
    GPIO_SPEED_HIGH = 2,
    GPIO_SPEED_VERY_HIGH = 3,
} gpio_speed_t;

typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP = 1,
    GPIO_PULL_DOWN = 2,
} gpio_pull_t;

static inline void gpio_clock_enable(uint8_t bank) {
    // STM32F4: GPIOx on AHB1
    RCC->AHB1ENR |= (1U << bank);
    (void)RCC->AHB1ENR;  // Read-back to ensure write completes
}

static inline void gpio_set_mode(gpio_pin_t gpio_pin, gpio_mode_t mode,
                                 gpio_speed_t speed) {
    const uint8_t bank = PINBANK(gpio_pin);
    const uint8_t pin = PINNO(gpio_pin);

    GPIO_TypeDef* gpio = gpio_from_bank(bank);
    if (gpio == (GPIO_TypeDef*)0 || pin > 15) return;

    gpio_clock_enable(bank);

    // Set mode (2 bits per pin)
    gpio->MODER = (gpio->MODER & ~(3U << (pin * 2))) | (mode << (pin * 2));

    // Set speed (2 bits per pin)
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3U << (pin * 2))) | (speed << (pin * 2));

    // Default to push-pull output
    gpio->OTYPER &= ~(1U << pin);

    // Default to no pull-up/pull-down
    gpio->PUPDR &= ~(3U << (pin * 2));
}

static inline void gpio_write(uint16_t gpio_pin, bool value) {
    GPIO_TypeDef* gpio = gpio_from_bank(PINBANK(gpio_pin));
    gpio->BSRR = (1U << PINNO(gpio_pin)) << (value ? 0U : 16U);
}

#define UART1 USART1
#define UART2 USART2
#define UART6 USART6

#ifndef UART_DEBUG
#define UART_DEBUG UART2
#endif

static inline void gpio_set_af(gpio_pin_t gpio_pin, uint8_t af) {
    const uint8_t bank = PINBANK(gpio_pin);
    const uint8_t pin = PINNO(gpio_pin);
    GPIO_TypeDef* gpio = gpio_from_bank(bank);
    if (gpio == (GPIO_TypeDef*)0 || pin > 15) return;

    volatile uint32_t* afr = (pin < 8) ? &gpio->AFR[0] : &gpio->AFR[1];
    uint8_t shift = (pin & 7) * 4;
    *afr = (*afr & ~(0xFU << shift)) | ((uint32_t)af << shift);
}

static inline void uart_init(USART_TypeDef* uart, uint32_t baudrate) {
    uint16_t rx;
    uint16_t tx;
    uint32_t pclk;
    uint8_t af;

    if (uart == UART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
        tx = PIN('A', 9);                      // PA9
        rx = PIN('A', 10);                     // PA10
        pclk = FREQ_HZ;                        // APB2 = 96MHz
        af = 7;                                // AF7 = USART1/2
    }

    if (uart == UART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 clock
        tx = PIN('A', 2);                      // PA2
        rx = PIN('A', 3);                      // PA3
        pclk = FREQ_HZ / 2;                    // APB1 = 48MHz
        af = 7;                                // AF7 = USART1/2
    }

    if (uart == UART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;  // Enable USART6 clock
        tx = PIN('A', 11);                     // PA11
        rx = PIN('A', 12);                     // PA12
        pclk = FREQ_HZ;                        // APB2 = 96MHz
        af = 8;                                // AF8 = USART6
    }

    // Configure GPIO pins for UART
    gpio_set_mode(tx, GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
    gpio_set_mode(rx, GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
    gpio_set_af(tx, af);
    gpio_set_af(rx, af);

    uart->CR1 = 0;                // Disable UART
    uart->BRR = pclk / baudrate;  // Use correct APB clock
    uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static inline void uart_write_byte(USART_TypeDef* uart, uint8_t byte) {
    uart->DR = byte;  // Write the byte to DR

    while ((uart->SR & USART_SR_TXE) == 0) {
        spin(1);
    }
}

static inline void uart_write_buffer(USART_TypeDef* uart, const void* buffer,
                                     size_t length) {
    const uint8_t* ptr = (const uint8_t*)buffer;
    while (length-- > 0) {
        uart_write_byte(uart, *ptr++);
    }
}

static inline uint8_t uart_read_ready(USART_TypeDef* uart) {
    return (uart->SR & USART_SR_RXNE) != 0;
}

static inline uint8_t uart_read_byte(USART_TypeDef* uart) {
    return (uint8_t)(uart->DR & 255);
}

static inline bool timer_expired(uint32_t* timer, uint32_t period,
                                 uint32_t now) {
    if (now + period < *timer) {
        *timer = 0;  // Timer overflowed, reset
    }

    if (*timer == 0) {
        *timer = now + period;  // first call, set expiration time
    }

    if (*timer > now) {
        return false;  // Not yet expired
    }

    // set next expiration time
    *timer = (now - *timer) > period ? now + period : *timer + period;
    return true;  // Expired
}
