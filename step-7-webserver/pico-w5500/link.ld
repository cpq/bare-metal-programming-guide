ENTRY(_reset);
MEMORY {
  boot  (rx) : ORIGIN = 0x10000000, LENGTH = 256
  flash (rx) : ORIGIN = 0x10000100, LENGTH = 2048K - LENGTH(boot)
  sram (rwx) : ORIGIN = 0x20000000, LENGTH = 264K
}

_sflash = ORIGIN(flash);
_estack = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */

SECTIONS {

  .boot : ALIGN(4) {
    KEEP(*(.boot))
    . = 256 - 4;
    LONG(0xcccccccc) /* RP2040 expects CRC in the last 4 bytes of this block */
  } > boot

  .text : ALIGN(4) {
    _stext = .;
    KEEP(*(.vectors))
    *(.text*)
    *(.rodata)
    *(.rodata.*)
    . = ALIGN(4);
    _etext = .;
  } > sram AT > flash

  .data : ALIGN(4) {
    _sdata = .;   /* .data section start */
    *(.data SORT(.data.*))
    . = ALIGN(4);
    _edata = .;  /* .data section end */
  } > sram AT > flash

  .bss : ALIGN(4) {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    . = ALIGN(4);
    _ebss = .;              /* .bss section end */
  } > sram

  _end = .;     /* for cmsis_gcc.h  */
}
