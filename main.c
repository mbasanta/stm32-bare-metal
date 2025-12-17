#include <stdio.h>

#include "mcu.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

uint32_t SystemCoreClock = FREQ_HZ;

void SystemInit(void) {
    clock_init();  // Configure clocks to 72MHz
}

int main(void) {
    SysTick_Config(SystemCoreClock / 1000);  // 1ms tick

    uart_init(UART2, 115200);

    // Configure PA5 as output push-pull, max speed 10MHz
    uint16_t pin = PIN('A', 5);  // PA5
    gpio_set_mode(pin, GPIO_MODE_OUTPUT_PP, GPIO_SPEED_10MHZ);

    uint32_t timer = 0;
    uint32_t period = 500;  // Blink period in ms
    uint32_t cycles = 0;

    for (;;) {
        cycles++;
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on;
            printf("LED: %d, tick: %lu, cycles: %lu\r\n", on, s_ticks,
                   cycles);  // Write message
            gpio_write(pin, on);
            on = !on;
            cycles = 0;
        }
    }

    return 0;
}