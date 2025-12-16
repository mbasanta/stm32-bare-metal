#include <stdio.h>

#include "mcu.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

uint32_t SystemCoreClock = FREQ_HZ;

void SystemInit(void) {
    // Configure Flash latency: 2 wait states for 72MHz operation
    FLASH->ACR |= FLASH_ACR_LATENCY_2;

    // Try to use HSE (8MHz external crystal) for true 72MHz
    // NUCLEO-F103RB has 8MHz HSE crystal (X3)
    RCC->CR |= RCC_CR_HSEON;
    uint32_t timeout = 100000;
    while (!(RCC->CR & RCC_CR_HSERDY) && timeout--) {
    }  // Wait for HSE ready

    if (RCC->CR & RCC_CR_HSERDY) {
        // HSE available: 8MHz * 9 = 72MHz
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL) |
                    RCC_CFGR_PLLMULL9;  // PLL multiplier = 9
        RCC->CFGR |= RCC_CFGR_PLLSRC;   // PLL source = HSE
    } else {
        // HSE failed, use HSI/2: 4MHz * 16 = 64MHz (max with HSI)
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PLLMULL) |
                    RCC_CFGR_PLLMULL16;  // PLL multiplier = 16
        RCC->CFGR &= ~RCC_CFGR_PLLSRC;   // PLL source = HSI/2
    }

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
    }  // Wait for PLL ready

    // Set APB1 prescaler to /2 (max 36MHz), APB2 to /1
    RCC->CFGR =
        (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV2;  // APB1 = 36MHz
    RCC->CFGR &= ~RCC_CFGR_PPRE2;                             // APB2 = 72MHz

    // Switch system clock to PLL
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }  // Wait for switch

    // Configure SysTick for 1ms interrupts
    SysTick_Config(SystemCoreClock / 1000);
}

int main(void) {
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
