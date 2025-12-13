#include <stdio.h>

#include "hal.h"

static volatile uint32_t s_ticks;
void systick_handler(void) { s_ticks++; }

int main(void) {
    // Configure SysTick for 1ms ticks
    systick_init(FREQ_HZ / 1000);

    uart_init(UART2, 115200);

    // Configure PA5 as output push-pull, max speed 10MHz
    uint16_t pin = PIN('A', 5);  // PA5
    gpio_set_mode(pin, GPIO_Output_PushPull, GPIO_Speed_10MHz);

    uint32_t timer = 0;
    uint32_t period = 500;  // Blink period in ms

    for (;;) {
        if (timer_expired(&timer, period, s_ticks)) {
            static bool on;
            printf("LED: %d, tick: %lu\r\n", on, s_ticks);  // Write message
            gpio_write(pin, on);
            on = !on;
        }
    }

    return 0;
}
