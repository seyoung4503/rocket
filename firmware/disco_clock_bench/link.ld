ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
    /* STM32F407: 128K contiguous SRAM at 0x20000000 (the other 64K is CCM
     * at 0x10000000, not contiguous). _estack must stay <= 0x20020000. */
    RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
    .text : { *(.text*) *(.rodata*) } > FLASH
    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
