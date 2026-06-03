/*
 * STM32F407G-DISC1 clock + control-loop benchmark.
 *
 * Brings the CPU from the 16 MHz boot clock up to the rated 168 MHz (PLL,
 * HSI-sourced), enables the FPU (hard-float build), and times a representative
 * "attitude + PID update" with the DWT cycle counter. Reports the cycles per
 * update and the resulting max loop rate, so we can see what control rate the
 * board can actually sustain.
 *
 * Telemetry @ 0x20000000:
 *   [0] magic 0x52434B54
 *   [1] tick
 *   [2] sws         clock source (RCC_CFGR SWS): 0=HSI, 2=PLL
 *   [3] sysclk_mhz  168 if PLL locked, else 16
 *   [4] cyc/update  DWT cycles for one control update
 *   [5] max_khz     sysclk / cyc  (max achievable loop rate)
 */

#define REG32(a) (*(volatile unsigned *)(a))

#define RCC_CR      REG32(0x40023800)
#define RCC_PLLCFGR REG32(0x40023804)
#define RCC_CFGR    REG32(0x40023808)
#define RCC_APB1ENR REG32(0x40023840)
#define PWR_CR      REG32(0x40007000)
#define FLASH_ACR   REG32(0x40023C00)

#define DEMCR       REG32(0xE000EDFC)
#define DWT_CTRL    REG32(0xE0001000)
#define DWT_CYCCNT  REG32(0xE0001004)

#define TELEM ((volatile unsigned *)0x20000000)

static int clock_168mhz(void)
{
    /* voltage scale 1 for 168 MHz */
    RCC_APB1ENR |= (1u << 28); /* PWREN */
    PWR_CR |= (1u << 14);      /* VOS = scale 1 */

    /* flash: 5 wait states + caches/prefetch */
    FLASH_ACR = (1u << 10) | (1u << 9) | (1u << 8) | 5u;

    /* bus prescalers: AHB /1, APB1 /4 (42MHz), APB2 /2 (84MHz) */
    RCC_CFGR = (RCC_CFGR & ~0xFCF0u) | (5u << 10) | (4u << 13);

    /* PLL: HSI/16 * 336 / 2 = 168 MHz, Q=7 */
    RCC_PLLCFGR = (16u) | (336u << 6) | (0u << 16) | (7u << 24); /* SRC=HSI */

    RCC_CR |= (1u << 24); /* PLLON */
    unsigned to = 2000000;
    while (!(RCC_CR & (1u << 25)) && --to) {} /* PLLRDY */
    if (!to) return 0;

    RCC_CFGR = (RCC_CFGR & ~3u) | 2u; /* SW = PLL */
    to = 2000000;
    while (((RCC_CFGR >> 2) & 3u) != 2u && --to) {} /* SWS == PLL */
    return to != 0;
}

/* Representative control update: 6x6 matrix-vector (covariance-ish), a
 * first-order filter, and a PID. ~50 float FMAs — a stand-in for one
 * attitude-estimation + control step. */
static float g_state[6];
static float g_integ;
static volatile float g_sink;

static float control_update(const float *in)
{
    static const float A[6][6] = {
        {0.98f, 0.01f, 0.00f, 0.02f, 0.00f, 0.01f},
        {0.00f, 0.97f, 0.03f, 0.00f, 0.01f, 0.00f},
        {0.01f, 0.00f, 0.96f, 0.02f, 0.00f, 0.02f},
        {0.00f, 0.02f, 0.00f, 0.95f, 0.03f, 0.00f},
        {0.02f, 0.00f, 0.01f, 0.00f, 0.97f, 0.01f},
        {0.00f, 0.01f, 0.00f, 0.03f, 0.00f, 0.96f},
    };
    float nxt[6];
    for (int i = 0; i < 6; i++) {
        float acc = 0.0f;
        for (int j = 0; j < 6; j++) {
            acc += A[i][j] * g_state[j];
        }
        nxt[i] = acc + 0.02f * in[i];
    }
    float meas = 0.0f;
    for (int i = 0; i < 6; i++) {
        g_state[i] = nxt[i];
        meas += g_state[i] * in[i];
    }
    float err = 0.5f - meas;
    g_integ += 0.01f * err;
    float deriv = err - g_state[0];
    return 2.0f * err + 0.1f * g_integ + 0.05f * deriv; /* PID */
}

int main(void)
{
    int locked = clock_168mhz();
    unsigned sws = (RCC_CFGR >> 2) & 3u;
    unsigned mhz = locked ? 168u : 16u;

    /* enable DWT cycle counter */
    DEMCR |= (1u << 24);     /* TRCENA */
    DWT_CYCCNT = 0;
    DWT_CTRL |= (1u << 0);   /* CYCCNTENA */

    TELEM[0] = 0x52434B54;
    TELEM[2] = sws;
    TELEM[3] = mhz;

    float in[6] = {0.1f, -0.2f, 0.05f, 0.3f, -0.1f, 0.2f};
    unsigned tick = 0;
    for (;;) {
        /* time 100 updates, report average cycles per update */
        unsigned c0 = DWT_CYCCNT;
        for (int k = 0; k < 100; k++) {
            in[0] = 0.1f + 0.001f * (k & 7);
            g_sink = control_update(in);
        }
        unsigned c1 = DWT_CYCCNT;
        unsigned cyc = (c1 - c0) / 100u;
        unsigned max_khz = cyc ? (mhz * 1000u) / cyc : 0u;

        TELEM[1] = tick;
        TELEM[4] = cyc;
        TELEM[5] = max_khz;
        tick++;
    }
    return 0;
}
