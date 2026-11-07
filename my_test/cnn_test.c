// ============================================================================
// File    : cnn_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI CNN 3x3 Convolution Accelerator.
//           Programs filter weights and bias, feeds 3x3 input receptive patch,
//           reads hardware MAC output, and verifies against software golden model.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define CNN_BASE        0x30000000

#define CNN_CTRL        (*(volatile uint32_t *)(CNN_BASE + 0x00))
#define CNN_STATUS      (*(volatile uint32_t *)(CNN_BASE + 0x04))
#define CNN_BIAS_REG    (*(volatile uint32_t *)(CNN_BASE + 0x3C))
#define CNN_RESULT_REG  (*(volatile uint32_t *)(CNN_BASE + 0x6C))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

// 3x3 Sobel/Laplacian-style test kernel
static const int32_t test_kernel[9] = {
    -1, -1, -1,
    -1,  8, -1,
    -1, -1, -1
};
static const int32_t test_bias = 10;

// 3x3 sample pixel patch
static const uint32_t test_patch[9] = {
    50, 50, 50,
    50, 100, 50,
    50, 50, 50
};

int main(void) {
    // 1. Program 3x3 kernel weights into CNN accelerator registers (0x18 .. 0x38)
    for (int i = 0; i < 9; i++) {
        *(volatile uint32_t *)(CNN_BASE + 0x18 + (i * 4)) = (uint32_t)test_kernel[i];
    }

    // 2. Program bias
    CNN_BIAS_REG = (uint32_t)test_bias;

    // 3. Load 3x3 pixel receptive field into accelerator input registers (0x48 .. 0x68)
    for (int i = 0; i < 9; i++) {
        *(volatile uint32_t *)(CNN_BASE + 0x48 + (i * 4)) = test_patch[i];
    }

    // 4. Compute expected convolution in software:
    //    sum = bias + sum(kernel[i] * patch[i])
    //    Here: 10 + (-1*50*8 + 8*100) = 10 + (-400 + 800) = 410
    int32_t expected_sum = test_bias;
    for (int i = 0; i < 9; i++) {
        expected_sum += test_kernel[i] * (int32_t)test_patch[i];
    }

    // 5. Read hardware convolution MAC result
    int32_t hw_result = (int32_t)CNN_RESULT_REG;

    *(volatile uint32_t *)(DMEM_BASE + 0x00) = (uint32_t)hw_result;
    *(volatile uint32_t *)(DMEM_BASE + 0x04) = (uint32_t)expected_sum;

    if (hw_result != expected_sum) {
        DMEM_PASS_FLAG = 0xDEAD0001;
        return 1;
    }

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
