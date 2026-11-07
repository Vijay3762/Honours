// ============================================================================
// File    : timer_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI Timer IP co-verification.
//           Exercises peripheral ID/version, load/compare configuration,
//           down-counting, compare match interrupt flag, and W1C clear.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define TIMER_BASE      0x50000000

#define TIMER_CTRL      (*(volatile uint32_t *)(TIMER_BASE + 0x00))
#define TIMER_LOAD      (*(volatile uint32_t *)(TIMER_BASE + 0x04))
#define TIMER_VALUE     (*(volatile uint32_t *)(TIMER_BASE + 0x08))
#define TIMER_COMPARE   (*(volatile uint32_t *)(TIMER_BASE + 0x0C))
#define TIMER_STATUS    (*(volatile uint32_t *)(TIMER_BASE + 0x10))
#define TIMER_ID        (*(volatile uint32_t *)(TIMER_BASE + 0x14))
#define TIMER_VERSION   (*(volatile uint32_t *)(TIMER_BASE + 0x18))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

int main(void) {
    // 1. Check ID ("TIMR" = 0x54494D52) and VERSION (v1.0 = 0x00010000)
    uint32_t id = TIMER_ID;
    uint32_t ver = TIMER_VERSION;
    *(volatile uint32_t *)(DMEM_BASE + 0x00) = id;
    *(volatile uint32_t *)(DMEM_BASE + 0x04) = ver;

    if (id != 0x54494D52 || ver != 0x00010000) {
        DMEM_PASS_FLAG = 0xDEAD0001;
        return 1;
    }

    // 2. Program LOAD = 30 and COMPARE = 15
    TIMER_LOAD = 30;
    TIMER_COMPARE = 15;
    *(volatile uint32_t *)(DMEM_BASE + 0x08) = TIMER_LOAD;

    // 3. Start timer (enable = 1, irq_en = 1) -> CTRL = 5
    TIMER_CTRL = 0x05;

    // 4. Poll down-counter until it passes compare threshold (15)
    uint32_t timeout = 10000;
    while (TIMER_VALUE > 15) {
        if (--timeout == 0) {
            DMEM_PASS_FLAG = 0xDEAD0002;
            return 2;
        }
    }

    // Small delay to ensure match flag is captured
    volatile int d = 5;
    while (d--);

    // 5. Verify match status flag (bit 0 == 1)
    uint32_t stat = TIMER_STATUS;
    *(volatile uint32_t *)(DMEM_BASE + 0x0C) = stat;
    if ((stat & 0x01) == 0) {
        DMEM_PASS_FLAG = 0xDEAD0003;
        return 3;
    }

    // 6. Write-1-to-Clear (W1C) match flag
    TIMER_STATUS = 0x01;
    stat = TIMER_STATUS;
    *(volatile uint32_t *)(DMEM_BASE + 0x10) = stat;
    if ((stat & 0x01) != 0) {
        DMEM_PASS_FLAG = 0xDEAD0004;
        return 4;
    }

    // 7. Stop timer
    TIMER_CTRL = 0x00;

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
