#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f103xb.h"

#define FREQ_HZ 72000000UL  // 72 MHz (max speed for STM32F103)

// Clock configuration: HSE (8MHz) → PLL (×9) → 72MHz SYSCLK
static inline void clock_init(void) {
    // Enable HSE (external 8MHz crystal on Nucleo board)
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0) {
    }  // Wait for HSE ready

    // Configure Flash latency: 2 wait states required for 72MHz
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    // Configure PLL: HSE × 9 = 8MHz × 9 = 72MHz
    // PLLMUL = 0111 (×9), PLLSRC = 1 (HSE)
    RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC;

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0) {
    }  // Wait for PLL ready

    // Set APB1 prescaler to /2 (max 36MHz for APB1)
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Switch system clock to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }  // Wait for switch
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
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE,
    };

    return (bank < (sizeof(ports) / sizeof(ports[0]))) ? ports[bank]
                                                       : (GPIO_TypeDef*)0;
}

typedef enum {
    GPIO_MODE_INPUT_ANALOG,
    GPIO_MODE_INPUT_FLOATING,
    GPIO_MODE_INPUT_PULLDOWN,
    GPIO_MODE_INPUT_PULLUP,

    GPIO_MODE_OUTPUT_PP,
    GPIO_MODE_OUTPUT_OD,
    GPIO_MODE_AF_PP,
    GPIO_MODE_AF_OD,
} gpio_mode_t;

typedef enum {
    GPIO_SPEED_10MHZ = 1,  // MODE=01
    GPIO_SPEED_2MHZ = 2,   // MODE=10
    GPIO_SPEED_50MHZ = 3,  // MODE=11
} gpio_speed_t;

static inline uint32_t gpio_f1_mode_bits(GPIO_TypeDef* gpio, uint8_t pin,
                                         gpio_mode_t mode, gpio_speed_t speed) {
    uint32_t cnf = 0, mod = 0;

    switch (mode) {
        // Inputs: MODE must be 00
        case GPIO_MODE_INPUT_ANALOG:
            cnf = 0b00;
            mod = 0b00;
            break;
        case GPIO_MODE_INPUT_FLOATING:
            cnf = 0b01;
            mod = 0b00;
            break;

        case GPIO_MODE_INPUT_PULLDOWN:
            cnf = 0b10;
            mod = 0b00;
            gpio->ODR &= ~(1u << pin);  // ODR=0 => pulldown
            break;

        case GPIO_MODE_INPUT_PULLUP:
            cnf = 0b10;
            mod = 0b00;
            gpio->ODR |= (1u << pin);  // ODR=1 => pullup
            break;

        // Outputs: MODE is speed
        case GPIO_MODE_OUTPUT_PP:
            cnf = 0b00;
            mod = (uint32_t)speed;
            break;
        case GPIO_MODE_OUTPUT_OD:
            cnf = 0b01;
            mod = (uint32_t)speed;
            break;
        case GPIO_MODE_AF_PP:
            cnf = 0b10;
            mod = (uint32_t)speed;
            break;
        case GPIO_MODE_AF_OD:
            cnf = 0b11;
            mod = (uint32_t)speed;
            break;
    }

    return ((cnf & 0x3u) << 2) | (mod & 0x3u);
}

static inline void gpio_clock_enable(uint8_t bank) {
    // STM32F103: GPIOA..GPIOE on APB2
    switch (bank) {
        case 0:
            RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
            break;
        case 1:
            RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
            break;
        case 2:
            RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
            break;
        case 3:
            RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
            break;
        case 4:
            RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
            break;
        default:
            return;
    }

    // Optional: read-back to ensure the write completes before peripheral use
    (void)RCC->APB2ENR;
}

static inline void gpio_set_mode(gpio_pin_t gpio_pin, gpio_mode_t mode,
                                 gpio_speed_t speed) {
    const uint8_t bank = PINBANK(gpio_pin);
    const uint8_t pin = PINNO(gpio_pin);

    GPIO_TypeDef* gpio = gpio_from_bank(bank);
    if (gpio == (GPIO_TypeDef*)0 || pin > 15) return;

    gpio_clock_enable(bank);

    volatile uint32_t* cr;
    uint32_t shift;

    if (pin < 8) {
        cr = &gpio->CRL;
        shift = (uint32_t)pin * 4u;
    } else {
        cr = &gpio->CRH;
        shift = (uint32_t)(pin - 8u) * 4u;
    }

    const uint32_t nibble = gpio_f1_mode_bits(gpio, pin, mode, speed);

    *cr = (*cr & ~(0xFu << shift)) | (nibble << shift);
}

static inline void gpio_write(uint16_t gpio_pin, bool value) {
    GPIO_TypeDef* gpio = gpio_from_bank(PINBANK(gpio_pin));
    gpio->BSRR = (1U << PINNO(gpio_pin)) << (value ? 0U : 16U);
}

#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

#ifndef UART_DEBUG
#define UART_DEBUG UART2
#endif

static inline void uart_init(USART_TypeDef* uart, uint32_t baudrate) {
    uint16_t rx;
    uint16_t tx;
    uint32_t pclk;

    if (uart == UART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
        tx = PIN('A', 9);                      // PA9
        rx = PIN('A', 10);                     // PA10
        pclk = FREQ_HZ;                        // APB2 = 72MHz
    }

    if (uart == UART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 clock
        tx = PIN('A', 2);                      // PA2
        rx = PIN('A', 3);                      // PA3
        pclk = FREQ_HZ / 2;                    // APB1 = 36MHz
    }

    if (uart == UART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // Enable USART3 clock
        tx = PIN('B', 10);                     // PB10
        rx = PIN('B', 11);                     // PB11
        pclk = FREQ_HZ / 2;                    // APB1 = 36MHz
    }

    gpio_set_mode(tx, GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
    gpio_set_mode(rx, GPIO_MODE_INPUT_FLOATING,
                  GPIO_SPEED_2MHZ);  // speed ignored for inputs

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
