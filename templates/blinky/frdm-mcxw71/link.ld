ENTRY(Reset_Handler);
MEMORY {
  flash(rx) : ORIGIN = 0x00000000, LENGTH = 1024k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 112k
}
STACK_SIZE = DEFINED(__stack_size__) ? __stack_size__ : 0x0800;
__StackTop = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */
__StackLimit = __StackTop - STACK_SIZE;

SECTIONS {
  .vectors : { KEEP(*(.isr_vector)) }  > flash
  .text    : { *(.text* .text.*) }     > flash
  .rodata  : { *(.rodata*) }           > flash
  __etext = .;
  .data    : { __data_start__ = .; *(.first_data) *(.data SORT(.data.*)) __data_end__ = .; } > sram AT > flash
  _sidata = LOADADDR(.data);
  .bss     : { __bss_start__ = .; *(.bss SORT(.bss.*) COMMON) __bss_end__ = .; } > sram
  . = ALIGN(8);
  __end__ = .;
}
