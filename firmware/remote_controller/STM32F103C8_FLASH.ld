MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

ENTRY(Reset_Handler)

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } >FLASH

    .text :
    {
        *(.text*)
        *(.rodata*)
        *(.ARM.exidx*)
        _etext = .;
    } >FLASH

    _sidata = LOADADDR(.data);

    .data : AT(_etext)
    {
        _sdata = .;
        *(.data*)
        _edata = .;
    } >RAM

    .bss :
    {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } >RAM

    end = .;
    _end = .;
}
