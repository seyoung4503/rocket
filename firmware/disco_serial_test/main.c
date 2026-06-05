/*
 * STM32F407G-DISC1 live-telemetry test.
 *
 * The F4-Discovery's on-board ST-Link VCP is NOT wired to any USART, so a
 * normal serial console can't reach the chip over the existing USB cable.
 * Instead the firmware publishes live values into a fixed RAM block, and the
 * host reads that block over SWD (ST-Link) — a debugger-backed "monitor".
 *
 * Telemetry block @ 0x20000000:
 *   [0] magic  0x52434B54 ('TKCR')   - marker so the host knows it's valid
 *   [1] tick   heartbeat counter      - proves the board is alive & looping
 *   [2] alt    fake altitude (cm)      - stand-in for a real sensor value
 *
 * Bare-metal, no libc, no semihosting/bkpt -> the core runs freely and the
 * host can read RAM while it runs.
 */

#define RCC_AHB1ENR (*(volatile unsigned *)0x40023830)
#define GPIOD_MODER (*(volatile unsigned *)0x40020C00)
#define GPIOD_BSRR  (*(volatile unsigned *)0x40020C18)
#define TELEM       ((volatile unsigned *)0x20000000)

static void delay(volatile unsigned n)
{
    while (n--) {
        asm volatile("nop");
    }
}

int main(void)
{
    /* Green LED (PD12) heartbeat. */
    RCC_AHB1ENR |= (1u << 3);                               /* GPIODEN */
    GPIOD_MODER = (GPIOD_MODER & ~(3u << 24)) | (1u << 24); /* PD12 output */

    TELEM[0] = 0x52434B54; /* 'TKCR' magic */

    unsigned tick = 0;
    for (;;) {
        TELEM[1] = tick;                /* heartbeat */
        TELEM[2] = (tick * 7u) % 1000u; /* fake altitude in cm */
        GPIOD_BSRR = (tick & 1) ? (1u << 28) : (1u << 12); /* toggle PD12 */
        tick++;
        delay(1500000);
    }
    return 0;
}
