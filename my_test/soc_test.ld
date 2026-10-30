/* soc_test.ld -- linker script for the VeeR EL2 AXI SoC
 *
 *   .text  -> 0xEE00_0000   ICCM (the core's local instruction memory)
 *   .data  -> placed directly after .text
 *
 * The boot program is fetched from the ICCM, all data accesses (and with
 * them the whole UART test) go through the AXI interconnect:
 *
 *   0xEE00_0000  ICCM      instruction memory
 *   0x1000_0000  DMEM      data memory (stack, test results)
 *   0x4000_0000  UART      AXI4-Lite UART (through the bridge)
 *
 * The .data section is placed after .text on purpose: the startup code uses
 * "la" (auipc + addi) which only reaches +/-1 MB, so a .data section far
 * away would silently break the stack pointer.  The DMEM is addressed with
 * absolute lui/addi instead.
 */
OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS {
    . = 0xEE000000;

    .text.init : { *(.text.init*) }
    .text      : { *(.text*) }

    . = ALIGN(16);
    .data      : { *(.data) *(.rodata*) *(.sdata*) }
    .bss       : { *(.bss) *(.sbss*) *(COMMON) }

    _end = .;
    . = ALIGN(16);
    PROVIDE(STACK = . + 0x1000);
}
