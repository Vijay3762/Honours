/* hello.c - Simple test for VeeR EL2 RISC-V core
 * Computes a basic sum and signals pass/fail to the testbench
 * via the 'tohost' memory-mapped register.
 *
 * Exit convention used by the VeeR testbench:
 *   tohost = 0xFF  --> PASS
 *   tohost = 0x01  --> FAIL
 * (crt0.s handles this mapping from main()'s return value)
 */

#include <stdint.h>

/* -------------------------------------------------------
 * Helper: tiny busy-delay (avoids needing any libc)
 * ------------------------------------------------------- */
static void delay(volatile uint32_t count) {
    while (count--);
}

/* -------------------------------------------------------
 * main
 * Returns 0 on success, non-zero on failure.
 * ------------------------------------------------------- */
int main(void) {

    /* Test 1: integer arithmetic */
    volatile uint32_t a = 10;
    volatile uint32_t b = 20;
    volatile uint32_t sum = a + b;

    if (sum != 30) {
        return 1;   /* FAIL */
    }

    /* Test 2: bitwise operations */
    volatile uint32_t mask = 0xAA;
    volatile uint32_t val  = 0xFF & mask;   /* expect 0xAA */

    if (val != 0xAA) {
        return 2;   /* FAIL */
    }

    /* Test 3: simple loop */
    volatile uint32_t acc = 0;
    for (uint32_t i = 0; i < 10; i++) {
        acc += i;           /* 0+1+2+...+9 = 45 */
    }

    if (acc != 45) {
        return 3;   /* FAIL */
    }

    delay(100);     /* small delay before exit */

    return 0;       /* PASS */
}
