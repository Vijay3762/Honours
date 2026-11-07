// ============================================================================
// File    : mnist_inference.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : End-to-end bare-metal firmware executing full MNIST handwritten
//           digit recognition pipeline on the VeeR EL2 processor using the
//           hardware accelerators (CNN Conv, ReLU, Pooling, Timer, UART, 7-Seg).
// ============================================================================

#include <stdint.h>
#include "mnist_data.h"

// ----------------------------------------------------------------------------
// Base Addresses (matching soc_top_veer memory map)
// ----------------------------------------------------------------------------
#define DMEM_BASE       0x10000000
#define DMA_BASE        0x20000000
#define CNN_BASE        0x30000000
#define UART_BASE       0x40000000
#define TIMER_BASE      0x50000000
#define GPIO_BASE       0x60000000
#define SEVEN_SEG_BASE  0x70000000
#define RELU_BASE       0x90000000
#define POOLING_BASE    0xA0000000

// ----------------------------------------------------------------------------
// Peripheral Register Offsets
// ----------------------------------------------------------------------------
// UART (CPU64 window: stride 8)
#define UART_THR        (*(volatile uint32_t *)(UART_BASE + 0x00))
#define UART_IER        (*(volatile uint32_t *)(UART_BASE + 0x08))
#define UART_BAUD_DIV   (*(volatile uint32_t *)(UART_BASE + 0x10))
#define UART_LCR        (*(volatile uint32_t *)(UART_BASE + 0x18))
#define UART_LSR        (*(volatile uint32_t *)(UART_BASE + 0x20))

// Timer
#define TIMER_LOAD      (*(volatile uint32_t *)(TIMER_BASE + 0x00))
#define TIMER_CMP       (*(volatile uint32_t *)(TIMER_BASE + 0x04))
#define TIMER_CTRL      (*(volatile uint32_t *)(TIMER_BASE + 0x08))
#define TIMER_STATUS    (*(volatile uint32_t *)(TIMER_BASE + 0x0C))
#define TIMER_VALUE     (*(volatile uint32_t *)(TIMER_BASE + 0x10))

// GPIO
#define GPIO_OUT        (*(volatile uint32_t *)(GPIO_BASE + 0x00))
#define GPIO_DIR        (*(volatile uint32_t *)(GPIO_BASE + 0x04))

// 7-Segment Display
#define SEVEN_SEG_DIG   (*(volatile uint32_t *)(SEVEN_SEG_BASE + 0x00))
#define SEVEN_SEG_EN    (*(volatile uint32_t *)(SEVEN_SEG_BASE + 0x04))
#define SEVEN_SEG_CTRL  (*(volatile uint32_t *)(SEVEN_SEG_BASE + 0x14))

// CNN Convolution
#define CNN_CTRL        (*(volatile uint32_t *)(CNN_BASE + 0x00))
#define CNN_STATUS      (*(volatile uint32_t *)(CNN_BASE + 0x04))
#define CNN_BIAS_REG    (*(volatile uint32_t *)(CNN_BASE + 0x3C))
#define CNN_RESULT_REG  (*(volatile uint32_t *)(CNN_BASE + 0x6C))

// ReLU
#define RELU_CTRL_REG   (*(volatile uint32_t *)(RELU_BASE + 0x00))
#define RELU_DIN_REG    (*(volatile uint32_t *)(RELU_BASE + 0x08))
#define RELU_DOUT_REG   (*(volatile uint32_t *)(RELU_BASE + 0x0C))

// Pooling
#define POOL_CTRL_REG   (*(volatile uint32_t *)(POOLING_BASE + 0x00))
#define POOL_P01_REG    (*(volatile uint32_t *)(POOLING_BASE + 0x0C))
#define POOL_P23_REG    (*(volatile uint32_t *)(POOLING_BASE + 0x10))
#define POOL_DOUT_REG   (*(volatile uint32_t *)(POOLING_BASE + 0x14))

// Status flag in DMEM monitored by simulation testbench
#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

// ----------------------------------------------------------------------------
// Helper Functions
// ----------------------------------------------------------------------------
static void uart_init(void) {
    UART_LCR = 0x83;        // DLAB = 1, 8N1
    UART_BAUD_DIV = 50;     // Baud divisor
    UART_LCR = 0x03;        // DLAB = 0, 8N1
    UART_IER = 0x01;        // Enable RX interrupt/polling
}

static void uart_putc(char c) {
    while ((UART_LSR & 0x20) == 0); // Wait for THRE
    UART_THR = (uint32_t)c;
}

static void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}

static void uart_put_dec(uint32_t val) {
    char buf[12];
    int i = 0;
    if (val == 0) {
        uart_putc('0');
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    while (i > 0) {
        uart_putc(buf[--i]);
    }
}

// ----------------------------------------------------------------------------
// Main MNIST Pipeline
// ----------------------------------------------------------------------------
int main(void) {
    // 1. Initialize peripherals
    uart_init();
    GPIO_DIR = 0x00FF;      // Lower 8 bits output (LEDs)
    GPIO_OUT = 0x0001;      // LED 0 on (booting)

    // 7-Seg: Enable digit 0, enable display with fast simulation mode
    SEVEN_SEG_DIG  = 0x0000;
    SEVEN_SEG_EN   = 0x0001; // Enable D0
    SEVEN_SEG_CTRL = 0x0009; // disp_en = 1, fast_sim = 1

    // ReLU: Enable unit
    RELU_CTRL_REG = 0x0001;

    // Pooling: Enable max pooling (mode 0)
    POOL_CTRL_REG = 0x0001;

    uart_puts("\r\n==================================================");
    uart_puts("\r\n  VeeR EL2 SoC -- MNIST Hardware CNN Inference    ");
    uart_puts("\r\n==================================================\n");

    // 2. Program CNN weights and bias
    for (int i = 0; i < 9; i++) {
        *(volatile uint32_t *)(CNN_BASE + 0x18 + (i * 4)) = (uint32_t)cnn_kernel[i];
    }
    CNN_BIAS_REG = (uint32_t)cnn_bias;

    // 3. Start performance timer
    TIMER_LOAD = 0x0FFFFFFF;
    TIMER_CTRL = 0x00000001; // Enable countdown

    // 4. Feature map buffer: 13 x 13 = 169 pooled features
    static int16_t features[POOL_SIZE];

    uart_puts("[CNN] Executing Conv 3x3 + ReLU + 2x2 MaxPool...\n");

    // Process 13x13 pooling regions across the 28x28 image
    for (int pr = 0; pr < 13; pr++) {
        for (int pc = 0; pc < 13; pc++) {
            int16_t q[4];

            // 2x2 window of convolution outputs
            for (int dr = 0; dr < 2; dr++) {
                for (int dc = 0; dc < 2; dc++) {
                    int r = pr * 2 + dr;
                    int c = pc * 2 + dc;

                    // Load 3x3 image patch into CNN accelerator window (0x48 .. 0x68)
                    for (int kr = 0; kr < 3; kr++) {
                        for (int kc = 0; kc < 3; kc++) {
                            uint32_t pix = (uint32_t)mnist_sample_image[(r + kr) * IMG_WIDTH + (c + kc)];
                            int reg_idx = kr * 3 + kc;
                            *(volatile uint32_t *)(CNN_BASE + 0x48 + (reg_idx * 4)) = pix;
                        }
                    }

                    // Read hardware convolution product
                    int32_t conv_res = (int32_t)CNN_RESULT_REG;

                    // Pass through hardware ReLU accelerator
                    RELU_DIN_REG = (uint32_t)conv_res;
                    int32_t relu_res = (int32_t)RELU_DOUT_REG;

                    q[dr * 2 + dc] = (int16_t)(relu_res & 0xFFFF);
                }
            }

            // Pass 4 activated values into hardware Pooling accelerator
            // P01: [15:0] = q[0], [31:16] = q[1]
            POOL_P01_REG = ((uint32_t)(uint16_t)q[1] << 16) | (uint16_t)q[0];
            // P23: [15:0] = q[2], [31:16] = q[3] (triggers calculation)
            POOL_P23_REG = ((uint32_t)(uint16_t)q[3] << 16) | (uint16_t)q[2];

            int16_t pooled_val = (int16_t)(POOL_DOUT_REG & 0xFFFF);
            features[pr * 13 + pc] = pooled_val;
        }
    }

    // Read elapsed timer cycles
    uint32_t timer_rem = TIMER_VALUE;
    uint32_t elapsed_cycles = 0x0FFFFFFF - timer_rem;

    // 5. Software Fully-Connected (FC) Layer Classification on VeeR EL2
    uart_puts("[CPU] Executing Fully-Connected Classification Layer...\n");
    int32_t best_score = -2147483647;
    int predicted_digit = -1;

    for (int d = 0; d < NUM_CLASSES; d++) {
        int32_t score = fc_biases[d];
        for (int i = 0; i < POOL_SIZE; i++) {
            score += (int32_t)fc_weights[d][i] * (int32_t)features[i];
        }
        if (score > best_score) {
            best_score = score;
            predicted_digit = d;
        }
    }

    // 6. Display predicted digit on 7-Segment display controller
    SEVEN_SEG_DIG = (uint32_t)predicted_digit;

    // 7. Report recognition results to UART
    uart_puts("--------------------------------------------------\n");
    uart_puts("  Predicted Digit   : ");
    uart_put_dec(predicted_digit);
    uart_puts("\n  Expected Digit    : ");
    uart_put_dec(EXPECTED_DIGIT);
    uart_puts("\n  Accelerator Cycles: ");
    uart_put_dec(elapsed_cycles);
    uart_puts("\n--------------------------------------------------\n");

    // 8. Verification Assertion
    if (predicted_digit == EXPECTED_DIGIT) {
        GPIO_OUT = 0x00AA; // Pass pattern on LEDs
        uart_puts(">>> INFERENCE PASSED: Digit Correctly Classified! <<<\n");
        DMEM_PASS_FLAG = 1; // Signal testbench PASS
    } else {
        GPIO_OUT = 0x00FF; // Fail pattern on LEDs
        uart_puts(">>> INFERENCE FAILED: Misclassification! <<<\n");
        DMEM_PASS_FLAG = 2; // Signal testbench FAIL
    }

    // Halt
    while (1);
    return 0;
}
