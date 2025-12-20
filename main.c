#include <stdio.h>
#include <string.h>

#include "lcd.h"
#include "mcu.h"

// TODO: figure out how to make this a 64-bit value for ticks > 49 days
static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

uint32_t SystemCoreClock = FREQ_HZ;

void SystemInit(void) { clock_init(); }

int main(void) {
    SysTick_Config(SystemCoreClock / 1000);  // 1ms tick

    uart_init(UART2, 115200);
    printf("Starting %s @ %lu MHz\r\n", MCU_NAME, FREQ_HZ / 1000000);

    // Configure LED pin
    gpio_set_mode(LED_PIN, GPIO_MODE_OUTPUT, GPIO_SPEED_LOW);

    // Configure i2c pins
    gpio_set_mode(I2C_SCL_PIN, GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
    gpio_set_af(I2C_SCL_PIN, I2C_AF);
    gpio_set_mode(I2C_SDA_PIN, GPIO_MODE_AF, GPIO_SPEED_VERY_HIGH);
    gpio_set_af(I2C_SDA_PIN, I2C_AF);

    // Initialize LCD
    printf("Initializing LCD...\r\n");
    lcd_init();
    printf("LCD initialized\r\n");

    lcd_clear();
    // Display hello message
    lcd_set_cursor(0, 0);  // Row 0, column 0
    lcd_print("Hello World!");
    lcd_set_cursor(1, 0);  // Row 1, column 0
    lcd_print(MCU_NAME);

    uint32_t cycles = 0;

    uint32_t period = 500;  // Blink period in ms
    uint32_t next_tick = 0;

    for (;;) {
        cycles++;
        if (s_ticks >= next_tick) {
            next_tick += period;

            static bool on;
            printf("LED: %d, tick: %lu, cycles: %lu\r\n", on, s_ticks, cycles);
            gpio_write(LED_PIN, LED_ACTIVE_LOW ? !on : on);
            on = !on;
            cycles = 0;
        }
    }

    return 0;
}