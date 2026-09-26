# crt0.s  --  Minimal C-runtime startup for VeeR EL2 testbench
# SPDX-License-Identifier: Apache-2.0
#
# Boot sequence:
#   1. Set up the stack pointer
#   2. Call main()
#   3. Map return value: 0 → 0xFF (pass), non-zero → 1 (fail)
#   4. Write result to 'tohost' to signal the testbench

.section .text.init
.global _start
_start:
    # ---- Set up stack ----
    la   sp, STACK

    # ---- Call main() ----
    call main

    # ---- Map exit code ----
    #   a0 holds main()'s return value
    mv   a1, a0          # save return value
    li   a0, 0xFF        # assume PASS
    beq  a1, x0, _finish # if return==0, keep 0xFF
    li   a0, 1           # else FAIL

.global _finish
_finish:
    la   t0, tohost
    sb   a0, 0(t0)       # write result byte to tohost
    beq  x0, x0, _finish # spin forever (testbench monitors tohost)
    .rept 10
    nop
    .endr

# ---- tohost register (monitored by testbench) ----
.section .data.io
.global tohost
tohost: .word 0
