// STM32F1 GPIO implementation (CRL/CRH registers)
#pragma once

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

    // Compatibility aliases for F4-style code
    GPIO_MODE_INPUT = GPIO_MODE_INPUT_FLOATING,
    GPIO_MODE_OUTPUT = GPIO_MODE_OUTPUT_PP,
    GPIO_MODE_AF = GPIO_MODE_AF_PP,
    GPIO_MODE_ANALOG = GPIO_MODE_INPUT_ANALOG,
} gpio_mode_t;

typedef enum {
    GPIO_SPEED_2MHZ = 2,
    GPIO_SPEED_10MHZ = 1,
    GPIO_SPEED_50MHZ = 3,

    // Compatibility aliases for F4-style code
    GPIO_SPEED_LOW = 2,        // 2MHz
    GPIO_SPEED_MEDIUM = 1,     // 10MHz
    GPIO_SPEED_HIGH = 3,       // 50MHz
    GPIO_SPEED_VERY_HIGH = 3,  // 50MHz (F1 max)
} gpio_speed_t;

static inline uint32_t gpio_f1_mode_bits(GPIO_TypeDef* gpio, uint8_t pin,
                                         gpio_mode_t mode, gpio_speed_t speed) {
    uint32_t cnf = 0, mod = 0;
    switch (mode) {
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
            gpio->ODR &= ~(1u << pin);
            break;
        case GPIO_MODE_INPUT_PULLUP:
            cnf = 0b10;
            mod = 0b00;
            gpio->ODR |= (1u << pin);
            break;
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
    (void)RCC->APB2ENR;
}

static inline void gpio_set_mode(gpio_pin_t gpio_pin, gpio_mode_t mode,
                                 gpio_speed_t speed) {
    const uint8_t bank = PINBANK(gpio_pin);
    const uint8_t pin = PINNO(gpio_pin);
    GPIO_TypeDef* gpio = gpio_from_bank(bank);
    if (gpio == (GPIO_TypeDef*)0 || pin > 15) return;

    gpio_clock_enable(bank);

    volatile uint32_t* cr = (pin < 8) ? &gpio->CRL : &gpio->CRH;
    uint32_t shift = (pin < 8) ? (pin * 4) : ((pin - 8) * 4);
    const uint32_t nibble = gpio_f1_mode_bits(gpio, pin, mode, speed);
    *cr = (*cr & ~(0xFu << shift)) | (nibble << shift);
}

static inline void gpio_write(uint16_t gpio_pin, bool value) {
    GPIO_TypeDef* gpio = gpio_from_bank(PINBANK(gpio_pin));
    gpio->BSRR = (1U << PINNO(gpio_pin)) << (value ? 0U : 16U);
}

// UART for STM32F1
#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

static inline void uart_init(USART_TypeDef* uart, uint32_t baudrate) {
    uint16_t tx, rx;
    uint32_t pclk;

    if (uart == UART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        tx = UART1_TX_PIN;
        rx = UART1_RX_PIN;
        pclk = APB2_FREQ_HZ;
    } else if (uart == UART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        tx = UART2_TX_PIN;
        rx = UART2_RX_PIN;
        pclk = APB1_FREQ_HZ;
    } else if (uart == UART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        tx = UART3_TX_PIN;
        rx = UART3_RX_PIN;
        pclk = APB1_FREQ_HZ;
    } else {
        return;
    }

    gpio_set_mode(tx, GPIO_MODE_AF_PP, GPIO_SPEED_50MHZ);
    gpio_set_mode(rx, GPIO_MODE_INPUT_FLOATING, GPIO_SPEED_2MHZ);

    uart->CR1 = 0;
    uart->BRR = pclk / baudrate;
    uart->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}
