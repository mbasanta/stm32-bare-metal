#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f103xb.h"

#define FREQ_HZ 8000000UL  // 8 MHz

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

enum GPIO_Input_Mode {
    GPIO_Input_Analog = 0,
    GPIO_Input_Floating = 1,
    GPIO_Input_PullUpDown = 2
};

enum GPIO_Output_Mode {
    GPIO_Output_PushPull = 0,
    GPIO_Output_OpenDrain = 1,
    GPIO_Output_AltPushPull = 2,
    GPIO_Output_AltOpenDrain = 3
};

enum GPIO_Speed {
    GPIO_Speed_10MHz = 1,
    GPIO_Speed_2MHz = 2,
    GPIO_Speed_50MHz = 3
};

static inline void gpio_set_mode(uint16_t gpio_pin, uint8_t output_mode,
                                 uint8_t speed) {
    volatile uint32_t* config_reg;
    uint8_t shift;

    GPIO_TypeDef* gpio = gpio_from_bank(PINBANK(gpio_pin));

    uint16_t pin = PINNO(gpio_pin);

    switch (PINBANK(pin)) {
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
            break;  // F1 parts vary; handle as needed
    }

    if (pin < 8) {
        config_reg = &gpio->CRL;
        shift = (uint8_t)(pin * 4U);
    } else {
        config_reg = &gpio->CRH;
        shift = (uint8_t)((pin - 8U) * 4U);
    }

    // Clear the 4 bits corresponding to the pin
    *config_reg &= ~((uint32_t)0xFU << shift);
    // Set the new mode and speed
    *config_reg |= (((uint32_t)output_mode << 2U) | speed) << shift;
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

    if (uart == UART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
        tx = PIN('A', 9);                      // PA9
        rx = PIN('A', 10);                     // PA10
    }

    if (uart == UART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART2 clock
        tx = PIN('A', 2);                      // PA2
        rx = PIN('A', 3);                      // PA3
    }

    if (uart == UART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // Enable USART3 clock
        tx = PIN('B', 10);                     // PB10
        rx = PIN('B', 11);                     // PB11
    }

    gpio_set_mode(tx, GPIO_Output_AltPushPull, GPIO_Speed_50MHz);
    gpio_set_mode(rx, GPIO_Output_AltPushPull, GPIO_Speed_50MHz);

    uart->CR1 = 0;                   // Disable UART
    uart->BRR = FREQ_HZ / baudrate;  // Assuming PCLK2 = FREQ_HZ
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
