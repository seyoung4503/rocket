/* Vector table + reset handler for STM32F407, with FPU enabled before main
 * (this build uses hard-float / VFP instructions). */

#define CPACR (*(volatile unsigned *)0xE000ED88)

extern unsigned _estack;
extern int main(void);

void Reset_Handler(void)
{
    /* Enable CP10/CP11 (FPU) full access BEFORE any VFP instruction runs. */
    CPACR |= (0xFu << 20);
    asm volatile("dsb");
    asm volatile("isb");
    main();
    for (;;) {
    }
}

static void Default_Handler(void)
{
    for (;;) {
    }
}

__attribute__((section(".isr_vector"), used))
void (*const g_vectors[])(void) = {
    (void (*)(void)) & _estack,
    Reset_Handler,
    Default_Handler, /* NMI */
    Default_Handler, /* HardFault */
};
