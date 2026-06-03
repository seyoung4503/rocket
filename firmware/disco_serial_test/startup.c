/* Minimal vector table + reset handler for STM32F407 (Cortex-M4). */

extern unsigned _estack;     /* top of stack, defined by linker */
extern int main(void);

void Reset_Handler(void)
{
    main();
    for (;;) {
    }
}

static void Default_Handler(void)
{
    for (;;) {
    }
}

/* Only the first two entries matter here (initial SP + reset); the rest
 * point at a trap loop. That's enough for this bare-metal test. */
__attribute__((section(".isr_vector"), used))
void (*const g_vectors[])(void) = {
    (void (*)(void)) & _estack, /* 0x00: initial stack pointer */
    Reset_Handler,              /* 0x04: reset */
    Default_Handler,            /* NMI */
    Default_Handler,            /* HardFault */
};
