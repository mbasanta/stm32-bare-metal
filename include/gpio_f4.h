// STM32F4 GPIO implementation (MODER/OTYPER/OSPEEDR registers)
#pragma once

static inline GPIO_TypeDef* gpio_from_bank(uint8_t bank) {
    static GPIO_TypeDef* const ports[] = {
        GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOH,
    };
    return (bank < (sizeof(ports) / sizeof(ports[0]))) ? ports[bank] : (GPIO_TypeDef*)0;
}

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_AF = 2,
    GPIO_MODE_ANALOG = 3,
} gpio_mode_t;

typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM = 1,
    GPIO_SPEED_HIGH = 2,
    GPIO_SPEED_VERY_HIGH = 3,
} gpio_speed_t;

static inline void gpio_clock_enable(uint8_t bank) {
    RCC->AHB1ENR |= (1U << bank);
    (void)RCC->AHB1ENR;
}

static inline void gpio_set_mode(gpio_pin_t gpio_pin, gpio_mode_t mode, gpio_speed_t speed) {
    const uint8_t bank = PINBANK(gpio_pin);
    const uint8_t pin = PINNO(gpio_pin);
    GPIO_TypeDef* gpio = gpio_from_bank(bank);
    if (gpio == (GPIO_TypeDef*)0 || pin > 15) return;

    gpio_clock_enable(bank);

    gpio->MODER = (gpio->MODER & ~(3U << (pin * 2))) | (mode << (pin * 2));
    gpio->OSPEEDR = (gpio->OSPEEDR & ~(3U << (pin * 2))) | (speed << (pin * 2));
    gpio->OTYPER &= ~(1U << pin);  // Default: push-pull
    gpio->PUPDR &= ~(3U << (pin * 2));  // Default: no pull
}

static inline void gpio_write(uint16_t gpio_pin, bool value) {
    GPIO_TypeDef* gpio = gpio_from_bank(PINBANK(gpio_pin));
    gpio->BSRR = (1U << PINNO(gpio_pin)) << (value ? 0U : 16U);
}

static inline void gpio_set_af(gpio_pin_t gpio_pin, uint8_t af) {
    const uint8_t bank = PINBANK(gpio_pin);
    const uint8_t pin = PINNO(gpio_pin);
    GPIO_TypeDef* gpio = gpio_from_bank(bank);
    if (gpio == (GPIO_TypeDef*)0 || pin > 15) return;

    volatile uint32_t* afr = (pin < 8) ? &gpio->AFR[0] : &gpio->AFR[1];
    uint8_t shift = (pin & 7) * 4;
    *afr = (*afr & ~(0xFU << shift)) | ((uint32_t)af << shift);
}

// UART for STM32F4
#define UART1 USART1
#define UART2 USART2
#define UART6 USART6

static inline void uart_init(USART_TypeDef* uart, uint32_t baudrate) {
    uint16_t tx, rx;
    uint32_t pclk;
    uint8_t af;

    if (uart == UART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        tx = UART1_TX_PIN;
        rx = UART1_RX_PIN;
        af = UART1_AF;
        pclk = APB2_FREQ_HZ;
    } else if (uart == UART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        tx = UART2_TX_PIN;
        rx = UART2_RX_PIN;
        af = UART2_AF;
        pclk = APB1_FREQ_HZ;
    } else if (uart == UART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
        tx = UART6_TX_PIN;
        rx = UART6_RX_PIN;
        af = UART6_AF;
        pclk = APB2_FREQ_HZ;
    } else {
        return;
    }

    gpio_set_mode(tx, GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
    gpio_set_mode(rx, GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
    gpio_set_af(tx, af);
    gpio_set_af(rx, af);

    uart->CR1 = 0;
    uart->BRR = pclk / baudrate;
    uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}
