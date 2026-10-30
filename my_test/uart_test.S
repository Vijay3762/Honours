# uart_test.S -- UART read/write test for the VeeR EL2 SoC
#
# Build target : rv32imc_zicsr_zifencei, ilp32
# Boot address : 0x8000_0000  (IMEM, interconnect m00)
#
# The test exercises the full CPU -> interconnect -> bridge -> UART path:
#
#   1. program the UART (LCR, baud divisor)
#   2. write 'A' into the transmit holding register (THR)
#   3. poll the line status register (LSR) until the looped back byte arrives
#   4. read the receive buffer register (RBR)  -> must be 'A'
#   5. write 'B' into the THR
#   6. store the received byte and the LSR in the DMEM so that the
#      testbench can check the result
#   7. halt in a tight loop
#
# UART register map (CPU64 window, base 0x4000_0000):
#
#   0x00  THR / RBR   transmit holding / receive buffer
#   0x08  IER         interrupt enable
#   0x10  BAUD_DIV    baud rate divisor (DLAB = 1)
#   0x18  LCR         line control (DLAB = 1 -> access BAUD_DIV)
#   0x20  LSR         line status (bit 0 = data ready, bit 5 = THRE, bit 6 = TEMT)
#
# The 8-byte strided window is required because the LSU of the core issues
# 64-bit AXI beats with 8-byte aligned addresses - see rtl/axi_uart_bridge.v.
#
# Note: GNU as does not accept ';' as a comment character, comments are '#'.

    .section .text.init
    .globl _start
_start:
    # ------------------------------------------------------------------
    # Stack pointer: top of the DMEM window (0x1000_0FF0, grows down)
    # ------------------------------------------------------------------
    lui  sp, 0x10001
    addi sp, sp, -16

    # ------------------------------------------------------------------
    # LCR = 0x83  (DLAB = 1, 8N1)  ->  the BAUD_DIV register becomes visible
    # ------------------------------------------------------------------
    lui  t0, 0x40000            # t0 = 0x4000_0000 (UART base)
    li   t1, 0x83
    sw   t1, 0x18(t0)           # 0x4000_0018 = LCR

    # ------------------------------------------------------------------
    # Baud divisor = 50  ->  51 clocks per bit at 100 MHz  (510 ns)
    # ------------------------------------------------------------------
    li   t1, 50
    sw   t1, 0x10(t0)           # 0x4000_0010 = BAUD_DIV

    # ------------------------------------------------------------------
    # LCR = 0x03  (8N1, DLAB = 0)
    # ------------------------------------------------------------------
    li   t1, 3
    sw   t1, 0x18(t0)           # 0x4000_0018 = LCR

    # ------------------------------------------------------------------
    # IER = 0x01
    #   The UART IP gates the "data ready" bit of the LSR with the
    #   interrupt enable, so the IER has to be set before the receive
    #   buffer can be polled.  The interrupt itself stays masked because
    #   mstatus.MIE is 0 (the firmware never enables interrupts).
    # ------------------------------------------------------------------
    li   t1, 1
    sw   t1, 0x08(t0)           # 0x4000_0008 = IER

    # ------------------------------------------------------------------
    # Transmit 'A' (0x41)
    # ------------------------------------------------------------------
    li   t1, 0x41
    sw   t1, 0x00(t0)           # 0x4000_0000 = THR

    # ------------------------------------------------------------------
    # Wait until the byte has been received back (loopback)
    # ------------------------------------------------------------------
wait_rx:
    lw   t2, 0x20(t0)           # 0x4000_0020 = LSR
    andi t2, t2, 1              # LSR[0] = data ready
    beq  t2, x0, wait_rx

    # ------------------------------------------------------------------
    # Read the received byte, it has to be 'A'
    # ------------------------------------------------------------------
    lw   t3, 0x00(t0)           # 0x4000_0000 = RBR
    mv   t4, t3                 # keep it

    # ------------------------------------------------------------------
    # Read the line status register once more and store both values in the
    # DMEM so the testbench can check them
    # ------------------------------------------------------------------
    lw   t2, 0x20(t0)           # LSR

    lui  t0, 0x10000            # t0 = 0x1000_0000 (DMEM base)
    sw   t4, 0(t0)              # [0x1000_0000] = received byte
    sw   t2, 4(t0)              # [0x1000_0004] = LSR

    # ------------------------------------------------------------------
    # Transmit 'B' (0x42)
    # ------------------------------------------------------------------
    lui  t0, 0x40000
    li   t1, 0x42
    sw   t1, 0x00(t0)           # THR

    # ------------------------------------------------------------------
    # Done - park the core
    # ------------------------------------------------------------------
done:
    j    done
