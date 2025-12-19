#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Include target-specific configuration
#if defined(TARGET_f103)
#include "mcu_f103.h"
#elif defined(TARGET_f411)
#include "mcu_f411.h"
#else
#error "No target defined! Use TARGET=f103 or TARGET=f411"
#endif

#define BIT(x) (1UL << (x))

typedef uint16_t gpio_pin_t;

// Encode pin as 0xBBNN → BB = bank (0=A,1=B,etc), NN = pin number (0–15)
#define PIN(bank, num) ((gpio_pin_t)((((bank) - 'A') << 8) | ((num) & 0xFF)))

#define PINNO(pin) ((uint8_t)((pin) & 0xFF))
#define PINBANK(pin) ((uint8_t)(((pin) >> 8) & 0xFF))

static inline void spin(volatile uint32_t count) {
    while (count--) __NOP();
}

// Include target-specific GPIO and UART implementations
#if defined(TARGET_f103)
#include "gpio_f1.h"
#elif defined(TARGET_f411)
#include "gpio_f4.h"
#endif

#ifndef UART_DEBUG
#define UART_DEBUG UART2
#endif

static inline void uart_write_byte(USART_TypeDef* uart, uint8_t byte) {
    uart->DR = byte;
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
