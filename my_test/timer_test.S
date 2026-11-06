# =============================================================================
# timer_test.S -- AXI Timer test program for the RISC-V SoC (VeeR EL2)
#
# Build target : rv32imc_zicsr_zifencei, ilp32
# Boot address : 0xEE00_0000 (ICCM) or 0x8000_0000 (IMEM)
#
# Register map (Timer at 0x5000_0000):
#   0x00: TIMER_CTRL    (bit0: en, bit1: auto-reload, bit2: irq_en)
#   0x04: TIMER_LOAD    (initial count)
#   0x08: TIMER_VALUE   (current count, RO)
#   0x0C: TIMER_COMPARE (compare match threshold)
#   0x10: TIMER_STATUS  (bit0: match W1C, bit1: overflow W1C)
#   0x14: TIMER_ID      (0x54494D52 = "TIMR", RO)
#   0x18: TIMER_VERSION (0x00010000 = v1.0, RO)
#
# Test steps:
#   1. Read TIMER_ID and TIMER_VERSION -> store to DMEM
#   2. Write TIMER_LOAD = 30 -> read back to DMEM
#   3. Write TIMER_COMPARE = 15 -> read back to DMEM
#   4. Write TIMER_CTRL = 5 (enable=1, irq_en=1)
#   5. Wait for down-counter to count past 15
#   6. Read TIMER_VALUE -> store to DMEM
#   7. Read TIMER_STATUS (match flag bit 0 should be 1) -> store to DMEM
#   8. Write TIMER_STATUS = 1 (clear match flag via W1C)
#   9. Read TIMER_STATUS (match flag should now be 0) -> store to DMEM
#  10. Verify all assertions and write 0x1 (PASS) to DMEM status word
# =============================================================================

    .section .text.init
    .globl _start
_start:
    # ------------------------------------------------------------------
    # Set stack pointer at top of DMEM (0x1000_0FF0)
    # ------------------------------------------------------------------
    lui  sp, 0x10001
    addi sp, sp, -16

    # ------------------------------------------------------------------
    # Base addresses:
    #   t0 = 0x5000_0000 (AXI Timer base)
    #   s0 = 0x1000_0000 (DMEM base for result verification)
    # ------------------------------------------------------------------
    lui  t0, 0x50000
    lui  s0, 0x10000

    # 1. Read TIMER_ID (0x14) and TIMER_VERSION (0x18)
    lw   t1, 0x14(t0)           # expected 0x54494D52 ("TIMR")
    sw   t1, 0x00(s0)           # DMEM[0]

    lw   t2, 0x18(t0)           # expected 0x00010000 (v1.0)
    sw   t2, 0x04(s0)           # DMEM[1]

    # 2. Configure TIMER_LOAD = 30 (0x04)
    li   t3, 30
    sw   t3, 0x04(t0)
    lw   t4, 0x04(t0)           # readback LOAD
    sw   t4, 0x08(s0)           # DMEM[2]

    # 3. Configure TIMER_COMPARE = 15 (0x0C)
    li   t5, 15
    sw   t5, 0x0C(t0)
    lw   t6, 0x0C(t0)           # readback COMPARE
    sw   t6, 0x0C(s0)           # DMEM[3]

    # 4. Start Timer: TIMER_CTRL = 0x05 (enable=1, irq_en=1, one-shot)
    li   a0, 5
    sw   a0, 0x00(t0)

    # 5. Delay loop: wait for down-counter to pass compare threshold 15
    li   a1, 100
wait_loop:
    addi a1, a1, -1
    bnez a1, wait_loop

    # 6. Read TIMER_VALUE (0x08)
    lw   a2, 0x08(t0)
    sw   a2, 0x10(s0)           # DMEM[4]

    # 7. Read TIMER_STATUS (0x10) - expect bit 0 (match) == 1
    lw   a3, 0x10(t0)
    sw   a3, 0x14(s0)           # DMEM[5]

    # 8. W1C: Clear match flag by writing 1 to bit 0 of TIMER_STATUS
    li   a4, 1
    sw   a4, 0x10(t0)

    # 9. Read back TIMER_STATUS - expect bit 0 == 0
    lw   a5, 0x10(t0)
    sw   a5, 0x18(s0)           # DMEM[6]

    # 10. Check test results
    li   a6, 0x54494D52         # expected ID
    bne  t1, a6, fail

    li   a6, 0x00010000         # expected version
    bne  t2, a6, fail

    li   a6, 30                 # expected LOAD
    bne  t4, a6, fail

    li   a6, 15                 # expected COMPARE
    bne  t6, a6, fail

    andi s1, a3, 1              # match bit before clear must be 1
    beqz s1, fail

    andi s2, a5, 1              # match bit after clear must be 0
    bnez s2, fail

    # All checks passed! Store 1 into DMEM[7] (byte offset 0x1C)
    li   s3, 1
    sw   s3, 0x1C(s0)
    j    halt

fail:
    sw   x0, 0x1C(s0)           # 0 = FAIL

halt:
    j    halt
