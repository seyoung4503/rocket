/*
 * STM32F407G-DISC1 onboard accelerometer test (SPI1).
 *
 * Reads the board's 3-axis MEMS accelerometer (LIS3DSH or LIS302DL,
 * revision-dependent) over SPI1 and publishes WHO_AM_I + X/Y/Z (in mg) into
 * the RAM telemetry block. Host reads it via SWD with monitor.cfg. Tilt the
 * board and the gravity vector (~1000 mg total) moves between axes.
 *
 * Pins (fixed by the Discovery board):
 *   PA5 = SPI1_SCK (AF5)   PA6 = SPI1_MISO (AF5)   PA7 = SPI1_MOSI (AF5)
 *   PE3 = CS (GPIO output, active low)
 * Clock: default HSI 16 MHz, APB2 = 16 MHz, SPI baud /32 = 500 kHz.
 *
 * Telemetry @ 0x20000000:
 *   [0] magic 0x52434B54 ('TKCR')
 *   [1] tick           heartbeat
 *   [2] who_am_i       raw (0x3F=LIS3DSH, 0x3B=LIS302DL)
 *   [3] accel_x  (signed mg)
 *   [4] accel_y  (signed mg)
 *   [5] accel_z  (signed mg)
 */

#define REG32(a) (*(volatile unsigned *)(a))

/* RCC */
#define RCC_AHB1ENR REG32(0x40023830)
#define RCC_APB2ENR REG32(0x40023844)
/* GPIOA / GPIOD / GPIOE */
#define GPIOA_MODER REG32(0x40020000)
#define GPIOA_AFRL  REG32(0x40020020)
#define GPIOD_MODER REG32(0x40020C00)
#define GPIOD_BSRR  REG32(0x40020C18)
#define GPIOE_MODER REG32(0x40021000)
#define GPIOE_BSRR  REG32(0x40021018)
/* SPI1 */
#define SPI1_CR1 REG32(0x40013000)
#define SPI1_SR  REG32(0x40013008)
#define SPI1_DR  REG32(0x4001300C)
/* telemetry block */
#define TELEM ((volatile unsigned *)0x20000000)

static void delay(volatile unsigned n) { while (n--) asm volatile("nop"); }

static unsigned spi_xfer(unsigned b)
{
    while (!(SPI1_SR & (1u << 1))) {} /* TXE */
    SPI1_DR = b & 0xFF;
    while (!(SPI1_SR & (1u << 0))) {} /* RXNE */
    return SPI1_DR & 0xFF;
}

static void cs_low(void)  { GPIOE_BSRR = (1u << (3 + 16)); }
static void cs_high(void) { GPIOE_BSRR = (1u << 3); }

static unsigned rd_reg(unsigned a)
{
    cs_low();
    spi_xfer(0x80 | a); /* read bit */
    unsigned v = spi_xfer(0x00);
    cs_high();
    return v;
}

static void wr_reg(unsigned a, unsigned v)
{
    cs_low();
    spi_xfer(a & 0x7F);
    spi_xfer(v);
    cs_high();
}

/* sign-extend an 8-bit value */
static int sx8(unsigned v) { return (v & 0x80) ? (int)v - 256 : (int)v; }
/* sign-extend a 16-bit value */
static int sx16(unsigned v) { return (v & 0x8000) ? (int)v - 65536 : (int)v; }

int main(void)
{
    /* --- clocks: GPIOA, GPIOD, GPIOE, SPI1 --- */
    RCC_AHB1ENR |= (1u << 0) | (1u << 3) | (1u << 4); /* A, D, E */
    RCC_APB2ENR |= (1u << 12);                        /* SPI1 */

    /* --- PD12 green LED heartbeat --- */
    GPIOD_MODER = (GPIOD_MODER & ~(3u << 24)) | (1u << 24);

    /* --- PA5/6/7 -> AF5 (SPI1) --- */
    GPIOA_MODER = (GPIOA_MODER & ~(0x3Fu << 10)) | (0x2Au << 10); /* 10 each */
    GPIOA_AFRL  = (GPIOA_AFRL & ~(0xFFFu << 20)) | (0x555u << 20); /* AF5 each */

    /* --- PE3 CS as output, idle high --- */
    GPIOE_MODER = (GPIOE_MODER & ~(3u << 6)) | (1u << 6);
    cs_high();

    /* --- SPI1: master, /32, mode 3 (CPOL=1,CPHA=1), 8-bit, soft NSS --- */
    SPI1_CR1 = (1u << 2)    /* MSTR */
             | (4u << 3)    /* BR = /32 */
             | (1u << 1) | (1u << 0)   /* CPOL=1, CPHA=1 */
             | (1u << 9) | (1u << 8)   /* SSM=1, SSI=1 */
             | (1u << 6);   /* SPE */

    delay(20000);

    unsigned who = rd_reg(0x0F); /* WHO_AM_I */
    TELEM[0] = 0x52434B54;
    TELEM[2] = who;

    int is_3dsh = (who == 0x3F);
    int is_302  = (who == 0x3B);

    if (is_3dsh) {
        wr_reg(0x20, 0x67); /* CTRL_REG4: ODR 100Hz, X/Y/Z enable */
    } else if (is_302) {
        wr_reg(0x20, 0x47); /* CTRL_REG1: 100Hz, active, X/Y/Z enable */
    }
    delay(50000);

    unsigned tick = 0;
    for (;;) {
        int x = 0, y = 0, z = 0;
        if (is_3dsh) {
            x = sx16(rd_reg(0x28) | (rd_reg(0x29) << 8));
            y = sx16(rd_reg(0x2A) | (rd_reg(0x2B) << 8));
            z = sx16(rd_reg(0x2C) | (rd_reg(0x2D) << 8));
            /* 0.06 mg/LSB at +-2g */
            x = x * 6 / 100; y = y * 6 / 100; z = z * 6 / 100;
        } else if (is_302) {
            x = sx8(rd_reg(0x29)) * 18; /* 18 mg/LSB */
            y = sx8(rd_reg(0x2B)) * 18;
            z = sx8(rd_reg(0x2D)) * 18;
        }
        TELEM[1] = tick;
        TELEM[3] = (unsigned)x;
        TELEM[4] = (unsigned)y;
        TELEM[5] = (unsigned)z;
        GPIOD_BSRR = (tick & 1) ? (1u << 28) : (1u << 12);
        tick++;
        delay(400000);
    }
    return 0;
}
