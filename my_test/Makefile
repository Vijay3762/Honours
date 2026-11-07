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

# ---- Timer test ------------------------------------------------------------
TIMER_AS   := timer_test.S
TIMER_ELF  := timer_test.elf
TIMER_HEX  := timer_test.hex
TIMER_LD   := soc_test.ld

.PHONY: timer_test
timer_test: $(TIMER_HEX)

$(TIMER_ELF): $(TIMER_AS) $(TIMER_LD)
	$(CC) $(CFLAGS) $(LDFLAGS) -T $(TIMER_LD) $< -o $@
	@$(OBJDUMP) -d $@ | head -60

$(TIMER_HEX): $(TIMER_ELF) make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $(TIMER_ELF) $(TIMER_HEX) 0xEE000000 --width=4
	@head -4 $(TIMER_HEX)

# ---- GPIO test -------------------------------------------------------------
GPIO_AS   := gpio_test.S
GPIO_ELF  := gpio_test.elf
GPIO_HEX  := gpio_test.hex
GPIO_LD   := soc_test.ld

.PHONY: gpio_test
gpio_test: $(GPIO_HEX)

$(GPIO_ELF): $(GPIO_AS) $(GPIO_LD)
	$(CC) $(CFLAGS) $(LDFLAGS) -T $(GPIO_LD) $< -o $@
	@$(OBJDUMP) -d $@ | head -60

$(GPIO_HEX): $(GPIO_ELF) make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $(GPIO_ELF) $(GPIO_HEX) 0xEE000000 --width=4
	@head -4 $(GPIO_HEX)

# ---- MNIST Inference test --------------------------------------------------
MNIST_SRCS := crt0.s mnist_inference.c
MNIST_ELF  := mnist_inference.elf
MNIST_HEX  := mnist_inference.hex
MNIST_LD   := soc_test.ld

.PHONY: mnist_inference
mnist_inference: $(MNIST_HEX)

$(MNIST_ELF): $(MNIST_SRCS) $(MNIST_LD) mnist_data.h
	$(CC) $(CFLAGS) $(LDFLAGS) -T $(MNIST_LD) $(MNIST_SRCS) -o $@
	@$(OBJDUMP) -h $@ | grep -E "Idx|\.text|\.data|\.bss"

$(MNIST_HEX): $(MNIST_ELF) make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $(MNIST_ELF) $(MNIST_HEX) 0xEE000000 --width=4
	@head -4 $(MNIST_HEX)

# ---- C Tests for IP + Core Co-Verification ---------------------------------
C_TESTS := uart_test_c gpio_test_c timer_test_c seven_seg_test cnn_test relu_test pooling_test dma_test

.PHONY: $(C_TESTS) all_c_tests

all_c_tests: $(foreach t,$(C_TESTS),$(t).hex) mnist_inference

# Generic rule for building IP C test hex files (ICCM 32-bit width at 0xEE000000)
%_test_c.elf: crt0.s %_test.c soc_test.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -T soc_test.ld crt0.s $*_test.c -o $@

%_test_c.hex: %_test_c.elf make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $< $@ 0xEE000000 --width=4

uart_test_c: uart_test_c.hex
gpio_test_c: gpio_test_c.hex
timer_test_c: timer_test_c.hex

# Peripheral & Accelerator C tests
seven_seg_test.elf: crt0.s seven_seg_test.c soc_test.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -T soc_test.ld crt0.s seven_seg_test.c -o $@

seven_seg_test.hex: seven_seg_test.elf make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $< $@ 0xEE000000 --width=4

seven_seg_test: seven_seg_test.hex

cnn_test.elf: crt0.s cnn_test.c soc_test.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -T soc_test.ld crt0.s cnn_test.c -o $@

cnn_test.hex: cnn_test.elf make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $< $@ 0xEE000000 --width=4

cnn_test: cnn_test.hex

relu_test.elf: crt0.s relu_test.c soc_test.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -T soc_test.ld crt0.s relu_test.c -o $@

relu_test.hex: relu_test.elf make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $< $@ 0xEE000000 --width=4

relu_test: relu_test.hex

pooling_test.elf: crt0.s pooling_test.c soc_test.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -T soc_test.ld crt0.s pooling_test.c -o $@

pooling_test.hex: pooling_test.elf make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $< $@ 0xEE000000 --width=4

pooling_test: pooling_test.hex

dma_test.elf: crt0.s dma_test.c soc_test.ld
	$(CC) $(CFLAGS) $(LDFLAGS) -T soc_test.ld crt0.s dma_test.c -o $@

dma_test.hex: dma_test.elf make_hex.py
	OBJCOPY=$(OBJCOPY) $(PYTHON) make_hex.py $< $@ 0xEE000000 --width=4

dma_test: dma_test.hex

# ---- helpers ---------------------------------------------------------------
dump: program.elf $(UART_ELF)
	$(OBJDUMP) -d -S program.elf > program.dump
	$(OBJDUMP) -d -S $(UART_ELF) > uart_test.dump
	@echo "disassembly written to program.dump and uart_test.dump"

clean:
	rm -f $(OBJS) program.elf program.bin program.hex program.dump
	rm -f $(UART_ELF) $(UART_HEX) uart_test.dump
	rm -f $(TIMER_ELF) $(TIMER_HEX)
	rm -f $(GPIO_ELF) $(GPIO_HEX)
	rm -f $(MNIST_ELF) $(MNIST_HEX)
	rm -f *.elf *.hex *.o *.tmp
