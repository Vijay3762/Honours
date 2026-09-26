# Makefile -- Build hello.hex for VeeR EL2
# SPDX-License-Identifier: Apache-2.0
#
# Usage:
#   make          -- build program.hex
#   make clean    -- remove generated files

# ---- Toolchain ----
GCC_PREFIX = riscv32-unknown-elf

CC      = $(GCC_PREFIX)-gcc
AS      = $(GCC_PREFIX)-as
LD      = $(GCC_PREFIX)-ld
OBJCOPY = $(GCC_PREFIX)-objcopy
OBJDUMP = $(GCC_PREFIX)-objdump

# ---- Flags ----
ABI     = -mabi=ilp32 -march=rv32imc_zicsr_zifencei
CFLAGS  = $(ABI) -g -O2 -Wall -ffreestanding -nostdlib -fno-builtin
LDFLAGS = -T hello.ld -nostdlib

# ---- Source files ----
SRCS_C  = hello.c
SRCS_S  = crt0.s
OBJS    = crt0.o hello.o

# ---- Default target ----
all: program.hex

# ---- Compile C ----
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# ---- Assemble startup ----
crt0.o: crt0.s
	$(CC) $(ABI) -c $< -o $@

# ---- Link to ELF ----
program.elf: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@

# ---- Convert ELF → binary ----
program.bin: program.elf
	$(OBJCOPY) -O binary $< $@

# ---- Convert ELF → Intel HEX ----
program.hex: program.elf
	$(OBJCOPY) -O ihex $< $@
	@echo ""
	@echo "=== Build complete: program.hex ==="
	@$(OBJDUMP) -h $< | grep -E "Idx|\.text|\.data|\.bss"

# ---- Disassembly (optional, useful for debugging) ----
dump: program.elf
	$(OBJDUMP) -d -S $< > program.dump
	@echo "Disassembly written to program.dump"

# ---- Clean ----
clean:
	rm -f $(OBJS) program.elf program.bin program.hex program.dump

.PHONY: all clean dump
