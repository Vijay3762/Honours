// ============================================================================
// File    : dma_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI DMA Controller co-verification.
//           Initializes source memory buffer in DMEM, programs DMA channels,
//           triggers memory-to-memory burst transfer, and verifies destination.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define DMA_BASE        0x20000000

#define DMA_CTRL        (*(volatile uint32_t *)(DMA_BASE + 0x00))
#define DMA_SRC_ADDR    (*(volatile uint32_t *)(DMA_BASE + 0x04))
#define DMA_DST_ADDR    (*(volatile uint32_t *)(DMA_BASE + 0x08))
#define DMA_LEN         (*(volatile uint32_t *)(DMA_BASE + 0x0C))
#define DMA_STATUS      (*(volatile uint32_t *)(DMA_BASE + 0x10))
#define DMA_IRQ_STATUS  (*(volatile uint32_t *)(DMA_BASE + 0x14))
#define DMA_ID          (*(volatile uint32_t *)(DMA_BASE + 0x18))
#define DMA_VERSION     (*(volatile uint32_t *)(DMA_BASE + 0x1C))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

#define TEST_WORDS      8
#define SRC_BUFFER_ADDR (DMEM_BASE + 0x1000)
#define DST_BUFFER_ADDR (DMEM_BASE + 0x2000)

int main(void) {
    volatile uint32_t *src = (volatile uint32_t *)SRC_BUFFER_ADDR;
    volatile uint32_t *dst = (volatile uint32_t *)DST_BUFFER_ADDR;

    // 1. Initialize source test pattern and clear destination buffer
    for (int i = 0; i < TEST_WORDS; i++) {
        src[i] = 0xA0B0C0D0 + i;
        dst[i] = 0x00000000;
    }

    // 2. Program DMA transfer parameters
    DMA_SRC_ADDR = SRC_BUFFER_ADDR;
    DMA_DST_ADDR = DST_BUFFER_ADDR;
    DMA_LEN      = TEST_WORDS / 2; // In 64-bit beats (4 beats = 8 32-bit words)

    // 3. Start DMA transfer: enable=1, start=1, irq_en=1 -> CTRL = 0x07
    DMA_CTRL = 0x07;

    // 4. Poll DMA_STATUS until busy bit (bit 0) clears and done bit (bit 1) sets
    uint32_t timeout = 100000;
    while ((DMA_STATUS & 0x02) == 0) {
        if (--timeout == 0) {
            DMEM_PASS_FLAG = 0xDEAD0001;
            return 1;
        }
    }

    // 5. Clear DMA done flag / interrupt via W1C
    DMA_IRQ_STATUS = 0x01;

    // 6. Verify destination buffer contents
    for (int i = 0; i < TEST_WORDS; i++) {
        if (dst[i] != src[i]) {
            DMEM_PASS_FLAG = 0xDEAD0000 | (i + 2);
            return (i + 2);
        }
    }

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
