# Makefile -- build the SoC test programs for the VeeR EL2
# SPDX-License-Identifier: Apache-2.0
#
# Usage:
#   make              -- build program.hex (hello world) and uart_test.hex
#   make uart_test    -- build only the UART test
#   make dump         -- disassemble
#   make clean        -- remove the generated files
#
# The toolchain is the RISC-V bare metal GCC.  It is called through the
# riscv64-unknown-elf-* prefix even though the target is RV32: the
# compiler generates RV32 code for -march=rv32imc -mabi=ilp32.

# ---- Toolchain -------------------------------------------------------------
TOOLCHAIN_PATH ?= /opt/riscv/bin
GCC_PREFIX     ?= riscv64-unknown-elf

CC      := $(TOOLCHAIN_PATH)/$(GCC_PREFIX)-gcc
AS      := $(TOOLCHAIN_PATH)/$(GCC_PREFIX)-as
LD      := $(TOOLCHAIN_PATH)/$(GCC_PREFIX)-ld
OBJCOPY := $(TOOLCHAIN_PATH)/$(GCC_PREFIX)-objcopy
OBJDUMP := $(TOOLCHAIN_PATH)/$(GCC_PREFIX)-objdump
PYTHON  ?= python3

# ---- Flags -----------------------------------------------------------------
ABI     := -mabi=ilp32 -march=rv32imc_zicsr_zifencei
CFLAGS  := $(ABI) -g -O2 -Wall -ffreestanding -nostdlib -fno-builtin
LDFLAGS := -nostdlib -nostartfiles -Wl,--no-warn-rwx-segments

# ---- hello world (kept for the standalone core test) -----------------------
SRCS_C  := hello.c
SRCS_S  := crt0.s
OBJS    := crt0.o hello.o
HELLO_LD := hello.ld

# ---- UART test -------------------------------------------------------------
# The boot program is fetched from the ICCM of the core (32 bit wide), the
# IMEM of the AXI SoC is 64 bit wide (two instructions per word).
UART_AS   := uart_test.S
UART_ELF  := uart_test.elf
UART_HEX  := uart_test.hex
UART_LD   := soc_test.ld

.PHONY: all clean dump uart_test

all: program.hex uart_test.hex

# ---- hello world -----------------------------------------------------------
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

crt0.o: crt0.s
	$(CC) $(ABI) -c $< -o $@

program.elf: $(OBJS) $(HELLO_LD)
	$(CC) $(CFLAGS) $(LDFLAGS) -T $(HELLO_LD) $(OBJS) -o $@

program.bin: program.elf
	$(OBJCOPY) -O binary $< $@

program.hex: program.elf
	$(OBJCOPY) -O ihex $< $@
	@$(OBJDUMP) -h $< | grep -E "Idx|\.text|\.data|\.bss"

# ---- UART test -------------------------------------------------------------
# The SoC instruction memory is 64-bit wide, so the image is converted into
# 64-bit words with addresses relative to the IMEM base (0x8000_0000).
uart_test: $(UART_HEX)

$(UART_ELF): $(UART_AS) $(UART_LD)
	$(CC) $(CFLAGS) $(LDFLAGS) -T $(UART_LD) $< -o $@
	@$(OBJDUMP) -d $@ | head -60

$(UART_HEX): $(UART_ELF) make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $(UART_ELF) $(UART_HEX) 0xEE000000 --width=4
	@head -4 $(UART_HEX)

# ---- helpers ---------------------------------------------------------------
dump: program.elf $(UART_ELF)
	$(OBJDUMP) -d -S program.elf > program.dump
	$(OBJDUMP) -d -S $(UART_ELF) > uart_test.dump
	@echo "disassembly written to program.dump and uart_test.dump"

clean:
	rm -f $(OBJS) program.elf program.bin program.hex program.dump
	rm -f $(UART_ELF) $(UART_HEX) uart_test.dump
