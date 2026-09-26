/* hello.ld  --  Linker script for VeeR EL2 testbench
 * SPDX-License-Identifier: Apache-2.0
 *
 * Memory map (matches the default VeeR EL2 testbench):
 *   0x80000000  -- start of ICCM / instruction memory (.text)
 *   0xd0580000  -- tohost I/O register (.data.io)
 */

OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS {
    /* ---- Code starts at 0x8000_0000 ---- */
    . = 0x80000000;

    .text : {
        *(.text.init*)   /* crt0 _start must be first */
        *(.text*)
    }

    _end = .;

    /* ---- Data / BSS / Stack ---- */
    .data : {
        *(.*data)
        *(.rodata*)
        *(.sbss)
        /* Reserve 4 KB for the stack */
        STACK = ALIGN(16) + 0x1000;
    }

    .bss : { *(.bss) }

    /* ---- tohost I/O register (testbench monitors this address) ---- */
    . = 0xd0580000;
    .data.io . : { *(.data.io) }
}
