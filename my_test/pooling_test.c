// ============================================================================
// File    : pooling_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI 2x2 Spatial Pooling Accelerator.
//           Exercises mode configuration, quad-pixel loading, and max
//           reduction across 2x2 spatial window.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define POOLING_BASE    0xA0000000

#define POOL_CTRL_REG   (*(volatile uint32_t *)(POOLING_BASE + 0x00))
#define POOL_P01_REG    (*(volatile uint32_t *)(POOLING_BASE + 0x0C))
#define POOL_P23_REG    (*(volatile uint32_t *)(POOLING_BASE + 0x10))
#define POOL_DOUT_REG   (*(volatile uint32_t *)(POOLING_BASE + 0x14))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

int main(void) {
    // 1. Enable Max Pooling mode (mode = 0, enable = 1) -> CTRL = 0x0001
    POOL_CTRL_REG = 0x0001;

    // 2. Test positive quad: q0=12, q1=45, q2=8, q3=30 -> max should be 45
    int16_t q0 = 12, q1 = 45, q2 = 8, q3 = 30;
    POOL_P01_REG = ((uint32_t)(uint16_t)q1 << 16) | (uint16_t)q0;
    POOL_P23_REG = ((uint32_t)(uint16_t)q3 << 16) | (uint16_t)q2;

    int16_t pool_res1 = (int16_t)(POOL_DOUT_REG & 0xFFFF);
    if (pool_res1 != 45) {
        DMEM_PASS_FLAG = 0xDEAD0001;
        return 1;
    }

    // 3. Test quad where maximum is in quadrant 3 (q2): q0=10, q1=20, q2=95, q3=40
    q0 = 10; q1 = 20; q2 = 95; q3 = 40;
    POOL_P01_REG = ((uint32_t)(uint16_t)q1 << 16) | (uint16_t)q0;
    POOL_P23_REG = ((uint32_t)(uint16_t)q3 << 16) | (uint16_t)q2;

    int16_t pool_res2 = (int16_t)(POOL_DOUT_REG & 0xFFFF);
    if (pool_res2 != 95) {
        DMEM_PASS_FLAG = 0xDEAD0002;
        return 2;
    }

    // 4. Test negative quad: q0=-50, q1=-10, q2=-25, q3=-5 -> max should be -5
    q0 = -50; q1 = -10; q2 = -25; q3 = -5;
    POOL_P01_REG = ((uint32_t)(uint16_t)q1 << 16) | (uint16_t)q0;
    POOL_P23_REG = ((uint32_t)(uint16_t)q3 << 16) | (uint16_t)q2;

    int16_t pool_res3 = (int16_t)(POOL_DOUT_REG & 0xFFFF);
    if (pool_res3 != -5) {
        DMEM_PASS_FLAG = 0xDEAD0003;
        return 3;
    }

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
