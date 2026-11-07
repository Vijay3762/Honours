// ============================================================================
// File    : gpio_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI GPIO IP co-verification.
//           Exercises peripheral ID/version, direction config, data out,
//           atomic SET/CLR/TOGGLE operations, and input sampling.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define GPIO_BASE       0x60000000

#define GPIO_DATA       (*(volatile uint32_t *)(GPIO_BASE + 0x00))
#define GPIO_OUT        (*(volatile uint32_t *)(GPIO_BASE + 0x04))
#define GPIO_DIR        (*(volatile uint32_t *)(GPIO_BASE + 0x08))
#define GPIO_OUT_SET    (*(volatile uint32_t *)(GPIO_BASE + 0x0C))
#define GPIO_OUT_CLR    (*(volatile uint32_t *)(GPIO_BASE + 0x10))
#define GPIO_OUT_TOGGLE (*(volatile uint32_t *)(GPIO_BASE + 0x14))
#define GPIO_ID         (*(volatile uint32_t *)(GPIO_BASE + 0x18))
#define GPIO_VERSION    (*(volatile uint32_t *)(GPIO_BASE + 0x1C))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

int main(void) {
    // 1. Verify Peripheral ID ("GPIO" = 0x4749504F) and VERSION (v1.0 = 0x00010000)
    uint32_t id = GPIO_ID;
    uint32_t ver = GPIO_VERSION;
    *(volatile uint32_t *)(DMEM_BASE + 0x00) = id;
    *(volatile uint32_t *)(DMEM_BASE + 0x04) = ver;

    if (id != 0x4749504F || ver != 0x00010000) {
        DMEM_PASS_FLAG = 0xDEAD0001;
        return 1;
    }

    // 2. Configure Direction: lower 8 bits output, upper 8 bits input (0x00FF)
    GPIO_DIR = 0x00FF;
    uint32_t dir_rb = GPIO_DIR;
    *(volatile uint32_t *)(DMEM_BASE + 0x08) = dir_rb;
    if ((dir_rb & 0xFFFF) != 0x00FF) {
        DMEM_PASS_FLAG = 0xDEAD0002;
        return 2;
    }

    // 3. Write initial pattern to GPIO_OUT (0x00A5)
    GPIO_OUT = 0x00A5;

    // 4. Atomic SET: set bits 4, 6 (mask 0x0050) -> expect 0x00F5
    GPIO_OUT_SET = 0x0050;
    if ((GPIO_OUT & 0xFFFF) != 0x00F5) {
        DMEM_PASS_FLAG = 0xDEAD0003;
        return 3;
    }

    // 5. Atomic CLR: clear bits 0, 2 (mask 0x0005) -> expect 0x00F0
    GPIO_OUT_CLR = 0x0005;
    if ((GPIO_OUT & 0xFFFF) != 0x00F0) {
        DMEM_PASS_FLAG = 0xDEAD0004;
        return 4;
    }

    // 6. Atomic TOGGLE: toggle bits 0, 1, 4, 5 (mask 0x0033) -> expect 0x00C3
    GPIO_OUT_TOGGLE = 0x0033;
    uint32_t final_out = GPIO_OUT & 0xFFFF;
    *(volatile uint32_t *)(DMEM_BASE + 0x0C) = final_out;
    if (final_out != 0x00C3) {
        DMEM_PASS_FLAG = 0xDEAD0005;
        return 5;
    }

    // 7. Read GPIO_IN
    uint32_t in_val = GPIO_DATA;
    *(volatile uint32_t *)(DMEM_BASE + 0x10) = in_val;

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
