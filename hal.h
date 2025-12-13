#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// ========== Macros ===========

#define FREQ_HZ 8000000UL  // 8 MHz

#define BIT(x) (1UL << (x))

// Encode pin as 0xBBNN → BB = bank (0=A,1=B,etc), NN = pin number (0–15)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))

#define PINNO(pin) ((pin) & 0xFF)
#define PINBANK(pin) ((pin) >> 8)

// ========== Structs for GPIO and RCC ===========

struct gpio {
    volatile uint32_t CRL;   // Port configuration register low
    volatile uint32_t CRH;   // Port configuration register high
    volatile uint32_t IDR;   // Port input data register
    volatile uint32_t ODR;   // Port output data register
    volatile uint32_t BSRR;  // Port bit set/reset register
    volatile uint32_t BRR;   // Port bit reset register
    volatile uint32_t LCKR;  // Port configuration lock register
};

// Returns GPIOA, GPIOB, GPIOC...
#define GPIO(bank) ((struct gpio*)(0x40010800 + 0x400 * (bank)))

struct rcc {
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
};

#define RCC ((struct rcc*)0x40021000)

struct uart {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
};

#define UART1 ((struct uart*)0x40013800)
#define UART2 ((struct uart*)0x40004400)
#define UART3 ((struct uart*)0x40004800)

// ========== Enums for mode configuration ===========

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

enum APB2_Peripheral {
    APB2_Peripheral_AFIO = 0,
    APB2_Peripheral_GPIOA = 2,
    APB2_Peripheral_GPIOB = 3,
    APB2_Peripheral_GPIOC = 4,
    APB2_Peripheral_GPIOD = 5,
    APB2_Peripheral_GPIOE = 6,
    APB2_Peripheral_ADC1 = 9,
    APB2_Peripheral_ADC2 = 10,
    APB2_Peripheral_TIM1 = 11,
    APB2_Peripheral_SPI1 = 12,
    APB2_Peripheral_USART1 = 14,
};

enum APB1_Peripheral {
    APB1_Peripheral_TIM2 = 0,
    APB1_Peripheral_TIM3 = 1,
    APB1_Peripheral_TIM4 = 2,
    APB1_Peripheral_TIM5 = 3,
    APB1_Peripheral_TIM6 = 4,
    APB1_Peripheral_TIM7 = 5,
    APB1_Peripheral_WWDG = 11,
    APB1_Peripheral_SPI2 = 14,
    APB1_Peripheral_SPI3 = 15,
    APB1_Peripheral_USART2 = 17,
    APB1_Peripheral_USART3 = 18,
    APB1_Peripheral_UART4E = 19,
    APB1_Peripheral_UART5E = 20,
    APB1_Peripheral_I2C1 = 21,
    APB1_Peripheral_I2C2 = 22,
    APB1_Peripheral_CAN1 = 25,
    APB1_Peripheral_CAN2 = 26,
    APB1_Peripheral_BKP = 27,
    APB1_Peripheral_PWR = 28,
    APB1_Peripheral_DAC = 29,
};

enum CR1_BITS {
    CR1_SBK = 0,
    CR1_RWU = 1,
    CR1_RE = 2,
    CR1_TE = 3,
    CR1_IDLEIE = 4,
    CR1_RXNEIE = 5,
    CR1_TCIE = 6,
    CR1_TXEIE = 7,
    CR1_PEIE = 8,
    CR1_PS = 9,
    CR1_PCE = 10,
    CR1_WAKE = 11,
    CR1_M = 12,
    CR1_UE = 13,
};

enum SR_BITS {
    SR_PE = 0,
    SR_FE = 1,
    SR_NE = 2,
    SR_ORE = 3,
    SR_IDLE = 4,
    SR_RXNE = 5,
    SR_TC = 6,
    SR_TXE = 7,
    SR_LBD = 8,
    SR_CTS = 9,
};

// ========== Structs for clock control ===========

struct systick {
    volatile uint32_t CTRL;   // SysTick control and status register
    volatile uint32_t LOAD;   // SysTick reload value register
    volatile uint32_t VAL;    // SysTick current value register
    volatile uint32_t CALIB;  // SysTick calibration value register
};

#define SYSTICK ((struct systick*)0xE000E010)

static inline void spin(volatile uint32_t count) {
    while (count--) (void)0;
}

static inline void systick_init(uint32_t ticks) {
    if ((ticks - 1U) > 0xFFFFFFU) {
        return;  // Reload value impossible, Systick timer is 24 bit
    }

    SYSTICK->LOAD = (uint32_t)(ticks - 1U);  // Set reload register
    SYSTICK->VAL = 0U;                       // Load the SysTick Counter Value
    SYSTICK->CTRL =
        BIT(0) | BIT(1) | BIT(2);  // Enable SysTick IRQ and SysTick Timer
}

static inline void gpio_set_mode(uint16_t gpio_pin,
                                 enum GPIO_Output_Mode output_mode,
                                 enum GPIO_Speed speed) {
    volatile uint32_t* config_reg;
    uint8_t shift;

    struct gpio* gpio = GPIO(PINBANK(gpio_pin));
    uint16_t pin = PINNO(gpio_pin);
    RCC->APB2ENR |= BIT(APB2_Peripheral_GPIOA + PINBANK(gpio_pin));

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

static inline void gpio_write(uint16_t pin, bool value) {
    struct gpio* gpio = GPIO(PINBANK(pin));
    gpio->BSRR = (1U << PINNO(pin)) << (value ? 0U : 16U);
}

static inline void uart_init(struct uart* uart, uint32_t baudrate) {
    uint16_t rx;
    uint16_t tx;

    if (uart == UART1) {
        RCC->APB2ENR |= BIT(APB2_Peripheral_USART1);  // Enable USART1 clock
        tx = PIN('A', 9);                             // PA9
        rx = PIN('A', 10);                            // PA10
    }

    if (uart == UART2) {
        RCC->APB1ENR |= BIT(APB1_Peripheral_USART2);  // Enable USART2 clock
        tx = PIN('A', 2);                             // PA2
        rx = PIN('A', 3);                             // PA3
    }

    if (uart == UART3) {
        RCC->APB1ENR |= BIT(APB1_Peripheral_USART3);  // Enable USART3 clock
        tx = PIN('B', 10);                            // PB10
        rx = PIN('B', 11);                            // PB11
    }

    gpio_set_mode(tx, GPIO_Output_AltPushPull, GPIO_Speed_50MHz);
    gpio_set_mode(rx, GPIO_Output_AltPushPull, GPIO_Speed_50MHz);

    uart->CR1 = 0;                   // Disable UART
    uart->BRR = FREQ_HZ / baudrate;  // Assuming PCLK2 = FREQ_HZ
    uart->CR1 = BIT(CR1_TE) | BIT(CR1_RE) | BIT(CR1_UE);  // TE, RE, UE
}

static inline uint8_t uart_read_ready(struct uart* uart) {
    return (uart->SR & BIT(SR_RXNE)) != 0;
}

static inline void uart_write_byte(struct uart* uart, uint8_t byte) {
    uart->DR = byte;  // Write the byte to DR

    while ((uart->SR & BIT(SR_TXE)) == 0) {
        spin(1);
    }
}

static inline void uart_write_buffer(struct uart* uart, const void* buffer,
                                     size_t length) {
    const uint8_t* ptr = (const uint8_t*)buffer;
    while (length-- > 0) {
        uart_write_byte(uart, *ptr++);
    }
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
