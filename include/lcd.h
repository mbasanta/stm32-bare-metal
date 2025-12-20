#include "mcu.h"

// ========== I2C LCD Functions ==========
#define LCD_ADDR 0x27
#define LCD_BACKLIGHT 0x08  // P3
#define LCD_EN 0x04         // P2 - Enable
#define LCD_RW 0x02         // P1 - Read/Write (keep 0)
#define LCD_RS 0x01         // P0 - Register Select (0=cmd, 1=data)

/*
printf("Scanning I2C bus...\r\n");
uint8_t found_count = 0;
for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    // Generate start condition
    I2C1->CR1 |= I2C_CR1_START;
    uint32_t timeout = 10000;
    while ((I2C1->SR1 & I2C_SR1_SB) == 0 && --timeout) {
    }
    if (!timeout) continue;

    // Send address (write)
    I2C1->DR = (addr << 1);
    timeout = 10000;
    while ((I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0 && --timeout) {
    }

    if (I2C1->SR1 & I2C_SR1_ADDR) {
        // Device acknowledged
        (void)I2C1->SR2;  // Clear ADDR flag by reading SR2
        printf("  Found device at 0x%02X\r\n", addr);
        found_count++;
    } else {
        // No acknowledgment, clear AF flag
        I2C1->SR1 &= ~I2C_SR1_AF;
    }

    // Generate stop condition
    I2C1->CR1 |= I2C_CR1_STOP;

    // Wait for stop to complete
    for (volatile int i = 0; i < 5000; i++) {
        __NOP();
    }
}
printf("I2C scan complete - found %d device(s)\r\n", found_count);
*/

// I2C write byte to PCF8574
static void i2c_write_byte(uint8_t data) {
    // Start condition
    I2C1->CR1 |= I2C_CR1_START;
    while ((I2C1->SR1 & I2C_SR1_SB) == 0) {
    }

    // Send address
    I2C1->DR = (LCD_ADDR << 1);
    while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {
    }
    (void)I2C1->SR2;  // Clear ADDR

    // Send data
    I2C1->DR = data;
    while ((I2C1->SR1 & I2C_SR1_TXE) == 0) {
    }

    // Stop condition
    I2C1->CR1 |= I2C_CR1_STOP;
    for (volatile int i = 0; i < 1000; i++) {
    }
}

// Send 4 bits to LCD via I2C (with enable pulse)
static inline void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    i2c_write_byte(data | LCD_EN);  // EN=1
    i2c_write_byte(data);           // EN=0 (falling edge latches data)
}

static inline void lcd_backlight(uint8_t on) {
    if (on) {
        i2c_write_byte(LCD_BACKLIGHT);
    } else {
        i2c_write_byte(0x00);
    }
}

// Send byte in 4-bit mode (high nibble first, then low nibble)
static inline void lcd_send_byte(uint8_t value, uint8_t mode) {
    lcd_send_nibble(value & 0xF0, mode);         // High nibble
    lcd_send_nibble((value << 4) & 0xF0, mode);  // Low nibble
}

// Send command to LCD
static inline void lcd_cmd(uint8_t cmd) {
    lcd_send_byte(cmd, 0);  // RS=0 for command
    for (volatile int i = 0; i < 5000; i++) {
    }
}

// Send data (character) to LCD
static inline void lcd_data(uint8_t data) {
    lcd_send_byte(data, LCD_RS);  // RS=1 for data
}

// Initialize LCD in 4-bit mode
static inline void lcd_init(void) {
    // Enable i2c clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // Configure I2C timing for 100kHz
    // FREQ = APB1/1MHz, TRISE = (1000ns/APB1_period)+1, CCR = APB1/(2*100kHz)
    uint32_t apb1_mhz = APB1_FREQ_HZ / 1000000;
    I2C1->CR2 = apb1_mhz;                     // APB1 frequency in MHz
    I2C1->TRISE = apb1_mhz + 1;               // (1000ns / APB1_period) + 1
    I2C1->CCR = APB1_FREQ_HZ / (2 * 100000);  // APB1 / (2 * 100kHz)
    I2C1->CR1 = I2C_CR1_PE;                   // Enable I2C1

    // Wait >40ms after power-on (datasheet requirement)
    for (volatile int i = 0; i < 100000; i++) {
    }

    // Initialization sequence for 4-bit mode (per HD44780 datasheet)
    lcd_send_nibble(0x30, 0);                   // Function set (8-bit mode)
    for (volatile int i = 0; i < 10000; i++) {  // Wait >4.1ms
    }
    lcd_send_nibble(0x30, 0);                  // Function set again
    for (volatile int i = 0; i < 5000; i++) {  // Wait >100us
    }
    lcd_send_nibble(0x30, 0);                  // Function set third time
    for (volatile int i = 0; i < 5000; i++) {  // Wait >100us
    }
    lcd_send_nibble(0x20, 0);  // Function set (4-bit mode)
    for (volatile int i = 0; i < 5000; i++) {
    }

    // Now in 4-bit mode - send full commands
    lcd_cmd(0x28);  // Function set: 4-bit, 2 lines, 5x8 font
    lcd_cmd(0x0C);  // Display ON, cursor OFF
    lcd_cmd(0x06);  // Entry mode: increment, no shift
    lcd_cmd(0x01);  // Clear display
    for (volatile int i = 0; i < 20000; i++) {
    }  // Clear takes ~2ms
}

// Clear display
static inline void lcd_clear(void) {
    lcd_cmd(0x01);
    for (volatile int i = 0; i < 20000; i++) {
    }
}

// Set cursor position (row: 0-1, col: 0-15)
static inline void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? 0x00 : 0x40;  // Row 0: 0x00, Row 1: 0x40
    lcd_cmd(0x80 | (addr + col));             // Set DDRAM address
}

// Print string to LCD
static inline void lcd_print(const char* str) {
    while (*str) {
        lcd_data(*str++);
    }
}