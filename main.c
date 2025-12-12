#include "main.h"

#include <stdbool.h>
#include <stdint.h>

static inline void gpio_set_mode(uint16_t gpio_pin,
                                 enum GPIO_Output_Mode output_mode,
                                 enum GPIO_Speed speed) {
    volatile uint32_t* config_reg;
    uint8_t shift;

    struct gpio* gpio = GPIO(PINBANK(gpio_pin));
    uint16_t pin = PINNO(gpio_pin);

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

static inline void spin(volatile uint32_t count) {
    while (count--) asm("nop");
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

bool timer_expired(uint32_t* timer, uint32_t period, uint32_t now) {
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

static volatile uint32_t s_ticks;
void systick_handler(void) { s_ticks++; }

int main(void) {
    // Enable clock for gpio port a
    RCC->APB2ENR |= BIT(APB2_Peripheral_GPIOA);  // IOPAEN is bit 2

    // Configure SysTick for 1ms ticks
    systick_init(FREQ_HZ / 1000);

    // Configure PA5 as output push-pull, max speed 10MHz
    uint16_t pin = PIN('A', 5);  // PA5
    gpio_set_mode(pin, GPIO_Output_PushPull, GPIO_Speed_10MHz);

    uint32_t timer;
    uint32_t period = 500;  // Blink period in ms

    for (;;) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on;
            gpio_write(pin, on);
            on = !on;
        }
    }

    return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
    // memset .bss to zero, and copy .data section to RAM region
    extern long _sbss, _ebss, _sdata, _edata, _sidata;

    for (long* dst = &_sbss; dst < &_ebss; dst++) {
        *dst = 0;
    }

    for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) {
        *dst++ = *src++;
    }

    main();  // Call main()

    for (;;) {
        (void)0;  // Infinite loop in the case if main() returns
    }
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 60 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 60])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, systick_handler};
