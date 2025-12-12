#include <stdint.h>
#include <stdbool.h>

#include "main.h"

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
