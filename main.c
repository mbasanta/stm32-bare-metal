#include <stdint.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio
{
    volatile uint32_t CRL;  // Port configuration register low
    volatile uint32_t CRH;  // Port configuration register high
    volatile uint32_t IDR;  // Port input data register
    volatile uint32_t ODR;  // Port output data register
    volatile uint32_t BSRR; // Port bit set/reset register
    volatile uint32_t BRR;  // Port bit reset register
    volatile uint32_t LCKR; // Port configuration lock register
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

#define RCC ((struct rcc *)0x40021000)

#define GPIO(bank) ((struct gpio *)(0x40010800 + 0x400 * (bank)))

// Enum for GPIO modes
enum GPIO_Input_Mode
{
    GPIO_Input_Analog = 0,
    GPIO_Input_Floating = 1,
    GPIO_Input_PullUpDown = 2
};

enum GPIO_Output_Mode
{
    GPIO_Output_PushPull = 0,
    GPIO_Output_OpenDrain = 1,
    GPIO_Output_AltPushPull = 2,
    GPIO_Output_AltOpenDrain = 3
};

enum GPIO_Speed
{
    GPIO_Speed_10MHz = 1,
    GPIO_Speed_2MHz = 2,
    GPIO_Speed_50MHz = 3
};

enum APB2_Peripheral
{
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

static inline void gpio_set_mode(uint16_t gpio_pin, enum GPIO_Output_Mode output_mode, enum GPIO_Speed speed)
{
    volatile uint32_t *config_reg;
    uint8_t shift;

    struct gpio* gpio = GPIO(PINBANK(gpio_pin));
    uint16_t pin = PINNO(gpio_pin);

    if (pin < 8)
    {
        config_reg = &gpio->CRL;
        shift = (uint8_t)(pin * 4U);
    }
    else
    {
        config_reg = &gpio->CRH;
        shift = (uint8_t)((pin - 8U) * 4U);
    }

    // Clear the 4 bits corresponding to the pin
    *config_reg &= ~((uint32_t)0xFU << shift);
    // Set the new mode and speed
    *config_reg |= (((uint32_t)output_mode << 2U) | speed) << shift;
}

static inline void gpio_write(uint16_t pin, bool value)
{
    struct gpio *gpio = GPIO(PINBANK(pin));
    gpio->BSRR = (1U << PINNO(pin)) << (value ? 0U : 16U);
}

static inline void spin(volatile uint32_t count)
{
    while (count--) (void) 0;
}

int main(void)
{
    uint16_t pin = PIN('A', 5); // PA5

    // Enable clock for gpio port a
    RCC->APB2ENR |= BIT(APB2_Peripheral_GPIOA); // IOPAEN is bit 2

    gpio_set_mode(pin, GPIO_Output_PushPull, GPIO_Speed_10MHz);

    for (;;) 
    {
        gpio_write(pin, true);
        spin(1000000);
        gpio_write(pin, false);
        spin(1000000);
    }

    return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void)
{
    // memset .bss to zero, and copy .data section to RAM region
    extern long _sbss, _ebss, _sdata, _edata, _sidata;

    for (long *dst = &_sbss; dst < &_ebss; dst++)
    {
        *dst = 0;
    }

    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;)
    {
        *dst++ = *src++;
    }

    main(); // Call main()

    for (;;)
    {
        (void)0; // Infinite loop in the case if main() returns
    }
}

extern void _estack(void); // Defined in link.ld

// 16 standard and 60 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 60])(void) = {
    _estack, _reset};
