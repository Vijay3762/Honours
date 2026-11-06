# =============================================================================
# gpio_test.S -- GPIO read/write and pin manipulation test for the VeeR EL2 SoC
#
# Build target : rv32imc_zicsr_zifencei, ilp32
# Boot address : 0x8000_0000  (IMEM, interconnect m00)
#
# The test exercises the full VeeR EL2 CPU -> Interconnect -> GPIO path:
#   1. Read GPIO_ID and GPIO_VERSION, store to DMEM
#   2. Set GPIO_DIR = 0x00FF (lower 8 pins output, upper 8 pins input)
#   3. Write GPIO_OUT = 0x00A5
#   4. Use GPIO_OUT_SET to set bits 4, 6 (0x50) -> OUT = 0x00F5
#   5. Use GPIO_OUT_CLR to clear bits 0, 2 (0x05) -> OUT = 0x00F0
#   6. Use GPIO_OUT_TOGGLE to toggle bits 0,1,4,5 (0x33) -> OUT = 0x00C3
#   7. Read GPIO_IN (external stimulus driven on pads)
#   8. Store final status into DMEM:
#        [0x1000_0000] = GPIO_ID
#        [0x1000_0004] = GPIO_VERSION
#        [0x1000_0008] = GPIO_DIR
#        [0x1000_000C] = GPIO_OUT (final 0x00C3)
#        [0x1000_0010] = GPIO_IN
#        [0x1000_0014] = Test Status (0x1 = PASS)
#   9. Halt in infinite loop
# =============================================================================

    .section .text.init
    .globl _start
_start:
    # ------------------------------------------------------------------
    # Stack pointer: top of the DMEM window (0x1000_0FF0, grows down)
    # ------------------------------------------------------------------
    lui  sp, 0x10001
    addi sp, sp, -16

    # ------------------------------------------------------------------
    # Base addresses:
    #   t0 = 0x6000_0000 (GPIO base)
    #   s0 = 0x1000_0000 (DMEM base for result verification)
    # ------------------------------------------------------------------
    lui  t0, 0x60000
    lui  s0, 0x10000

    # 1. Read GPIO_ID (0x18) and GPIO_VERSION (0x1C)
    lw   t1, 0x18(t0)           # GPIO_ID (expected 0x4749504F)
    sw   t1, 0x00(s0)           # store to DMEM[0]

    lw   t2, 0x1C(t0)           # GPIO_VERSION (expected 0x00010000)
    sw   t2, 0x04(s0)           # store to DMEM[1]

    # 2. Configure GPIO_DIR = 0x00FF (0x08)
    li   t3, 0x00FF
    sw   t3, 0x08(t0)           # GPIO_DIR = 0x00FF
    lw   t4, 0x08(t0)           # Readback GPIO_DIR
    sw   t4, 0x08(s0)           # store to DMEM[2]

    # 3. Write GPIO_OUT = 0x00A5 (0x04)
    li   t5, 0x00A5
    sw   t5, 0x04(t0)           # GPIO_OUT = 0x00A5

    # 4. Atomic SET: GPIO_OUT_SET = 0x0050 (0x0C) -> OUT = 0x00F5
    li   t6, 0x0050
    sw   t6, 0x0C(t0)

    # 5. Atomic CLR: GPIO_OUT_CLR = 0x0005 (0x10) -> OUT = 0x00F0
    li   t6, 0x0005
    sw   t6, 0x10(t0)

    # 6. Atomic TOGGLE: GPIO_OUT_TOGGLE = 0x0033 (0x14) -> OUT = 0x00C3
    li   t6, 0x0033
    sw   t6, 0x14(t0)

    # Readback final GPIO_OUT (should be 0x00C3)
    lw   a0, 0x04(t0)
    sw   a0, 0x0C(s0)           # store to DMEM[3]

    # 7. Read GPIO_IN (0x00)
    lw   a1, 0x00(t0)
    sw   a1, 0x10(s0)           # store to DMEM[4]

    # 8. Check results and set pass flag
    li   a2, 1                  # assume PASS
    li   a3, 0x4749504F         # expected ID
    bne  t1, a3, fail

    li   a3, 0x00010000         # expected version
    bne  t2, a3, fail

    li   a3, 0x00FF             # expected DIR
    bne  t4, a3, fail

    li   a3, 0x00C3             # expected final OUT
    bne  a0, a3, fail

    # Success: store 1 to status
    sw   a2, 0x14(s0)
    j    halt

fail:
    sw   x0, 0x14(s0)           # 0 = FAIL

halt:
    j    halt
