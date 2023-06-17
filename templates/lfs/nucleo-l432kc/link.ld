ENTRY(Reset_Handler);
MEMORY {
  flash(rx) : ORIGIN = 0x08000000, LENGTH = 256k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 64k
}
_estack = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */
_eflash = ORIGIN(flash) + LENGTH(flash);  /* points to end of flash */

SECTIONS {
  .vectors : { KEEP(*(.isr_vector)) }  > flash
  .text    : { *(.text* .text.*) }     > flash
  .rodata  : { *(.rodata*) }           > flash
  .data    : { _sdata = .; *(.first_data) *(.data SORT(.data.*)) _edata = .; } > sram AT > flash
  _sidata = LOADADDR(.data);
  .bss     : { _sbss = .; *(.bss SORT(.bss.*) COMMON) _ebss = .; } > sram
  . = ALIGN(8);
  _end = .;
}
