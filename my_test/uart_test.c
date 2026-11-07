// ============================================================================
// File    : uart_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI UART IP co-verification.
//           Exercises baud programming, character TX, serial loopback RX,
//           and verifies received data integrity and line status.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define UART_BASE       0x40000000

// UART registers in 64-bit LSU window (8-byte stride)
#define UART_THR        (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_RBR        (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_IER        (*(volatile uint32_t *)(UART_BASE + 0x08))
#define UART_BAUD_DIV   (*(volatile uint32_t *)(UART_BASE + 0x10))
#define UART_LCR        (*(volatile uint32_t *)(UART_BASE + 0x18))
#define UART_LSR        (*(volatile uint32_t *)(UART_BASE + 0x20))

// Results logged to DMEM for testbench inspection
#define DMEM_RX_DATA    (*(volatile uint32_t *)(DMEM_BASE + 0x00))
#define DMEM_LSR_VAL    (*(volatile uint32_t *)(DMEM_BASE + 0x04))
#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

int main(void) {
    // 1. Configure UART baud rate via DLAB sequence (8N1, divisor 50)
    UART_LCR = 0x83;        // DLAB = 1, 8N1
    UART_BAUD_DIV = 50;     // Divisor 50 -> 51 clocks/bit at 100MHz
    UART_LCR = 0x03;        // DLAB = 0, 8N1

    // 2. Enable RX interrupt (gates DATA_READY in LSR)
    UART_IER = 0x01;

    // 3. Transmit test character 'A' (0x41)
    while ((UART_LSR & 0x20) == 0); // Wait for THRE (bit 5)
    UART_THR = 0x41;

    // 4. Poll for loopback reception (DATA_READY, bit 0)
    uint32_t timeout = 100000;
    while ((UART_LSR & 0x01) == 0) {
        if (--timeout == 0) {
            DMEM_PASS_FLAG = 0xDEAD0001; // Timeout waiting for RX
            return 1;
        }
    }

    // 5. Read back received byte from RBR
    uint32_t rx_byte = UART_RBR & 0xFF;
    uint32_t lsr_val = UART_LSR;

    // Store results in DMEM
    DMEM_RX_DATA = rx_byte;
    DMEM_LSR_VAL = lsr_val;

    if (rx_byte != 0x41) {
        DMEM_PASS_FLAG = 0xDEAD0002; // Data mismatch
        return 2;
    }

    // 6. Transmit second test character 'B' (0x42)
    while ((UART_LSR & 0x20) == 0);
    UART_THR = 0x42;

    // 7. Signal PASS to testbench
    DMEM_PASS_FLAG = 0x00000001;
    return 0; // PASS -> crt0 writes 0xFF to tohost
}
