// ============================================================================
// File    : relu_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI ReLU Accelerator co-verification.
//           Exercises positive passthrough, negative clamping (max(0, x)),
//           zero handling, and boundary saturation.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define RELU_BASE       0x90000000

#define RELU_CTRL_REG   (*(volatile uint32_t *)(RELU_BASE + 0x00))
#define RELU_DIN_REG    (*(volatile uint32_t *)(RELU_BASE + 0x08))
#define RELU_DOUT_REG   (*(volatile uint32_t *)(RELU_BASE + 0x0C))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

int main(void) {
    // 1. Enable ReLU unit (bit 0 = 1)
    RELU_CTRL_REG = 0x0001;

    // 2. Test positive input -> expect linear passthrough
    int32_t test_pos = 1250;
    RELU_DIN_REG = (uint32_t)test_pos;
    int32_t out_pos = (int32_t)RELU_DOUT_REG;
    if (out_pos != test_pos) {
        DMEM_PASS_FLAG = 0xDEAD0001;
        return 1;
    }

    // 3. Test negative input -> expect clamp to 0
    int32_t test_neg = -840;
    RELU_DIN_REG = (uint32_t)test_neg;
    int32_t out_neg = (int32_t)RELU_DOUT_REG;
    if (out_neg != 0) {
        DMEM_PASS_FLAG = 0xDEAD0002;
        return 2;
    }

    // 4. Test zero input -> expect 0
    RELU_DIN_REG = 0;
    int32_t out_zero = (int32_t)RELU_DOUT_REG;
    if (out_zero != 0) {
        DMEM_PASS_FLAG = 0xDEAD0003;
        return 3;
    }

    // 5. Test boundary negative value
    RELU_DIN_REG = 0x80000000; // -2147483648
    int32_t out_bnd = (int32_t)RELU_DOUT_REG;
    if (out_bnd != 0) {
        DMEM_PASS_FLAG = 0xDEAD0004;
        return 4;
    }

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
