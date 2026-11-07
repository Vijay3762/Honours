// ============================================================================
// File    : seven_seg_test.c
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : C test for VeeR EL2 core + AXI 7-Segment Display Controller.
//           Exercises digit programming (0-9), anode enable configuration,
//           and display control in simulation mode.
// ============================================================================

#include <stdint.h>

#define DMEM_BASE       0x10000000
#define SEVEN_SEG_BASE  0x70000000

#define SEVEN_SEG_DIG   (*(volatile uint32_t *)(SEVEN_SEG_BASE + 0x00))
#define SEVEN_SEG_EN    (*(volatile uint32_t *)(SEVEN_SEG_BASE + 0x04))
#define SEVEN_SEG_CTRL  (*(volatile uint32_t *)(SEVEN_SEG_BASE + 0x14))

#define DMEM_PASS_FLAG  (*(volatile uint32_t *)(DMEM_BASE + 0x14))

int main(void) {
    // 1. Enable 4 digits (anodes D0-D3)
    SEVEN_SEG_EN = 0x000F;

    // 2. Enable display with fast simulation mode (disp_en=1, fast_sim=1)
    SEVEN_SEG_CTRL = 0x0009;

    // 3. Sequentially display digits 0 through 9
    for (uint32_t digit = 0; digit <= 9; digit++) {
        SEVEN_SEG_DIG = digit;
        // Verify readback
        if (SEVEN_SEG_DIG != digit) {
            DMEM_PASS_FLAG = 0xDEAD0000 | digit;
            return 1;
        }
        volatile int delay = 100;
        while (delay--);
    }

    // 4. Set final display digit to 7 (MNIST recognized digit example)
    SEVEN_SEG_DIG = 7;

    // Signal PASS
    DMEM_PASS_FLAG = 0x00000001;
    return 0;
}
