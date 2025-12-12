#ifndef MAIN_H
#define MAIN_H

#include <stdbool.h>
#include <stdint.h>

// ========== Macros ===========

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

// Returns GPIOA, GPIOB, GPIOC...
#define GPIO(bank) ((struct gpio*)(0x40010800 + 0x400 * (bank)))

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

#endif  // MAIN_H