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

    extern void main(void);
    main();  // Call main()

    for (;;) {
        (void)0;  // Infinite loop in the case if main() returns
    }
}

extern void systick_handler(void);  // Defined in main.c
extern void _estack(void);          // Defined in link.ld

// 16 standard and 60 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 60])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, systick_handler};