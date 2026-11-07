#!/usr/bin/env python3
"""
gen_mnist_data.py -- Generate synthetic/trained MNIST weights and a test digit '7'
for the RISC-V VeeR EL2 CNN SoC inference verification.
"""

import math

def generate_digit_7():
    """Generates a clean 28x28 grayscale image of digit 7 (values 0-255)."""
    img = [[0 for _ in range(28)] for _ in range(28)]
    # Horizontal top stroke: rows 5 to 7, cols 6 to 21
    for r in range(5, 8):
        for c in range(6, 22):
            img[r][c] = 240
    # Diagonal stroke down to the bottom
    for i in range(16):
        r = 7 + i
        c = 21 - int(i * 0.8)
        if 0 <= r < 28 and 0 <= c < 28:
            img[r][c] = 250
            if c - 1 >= 0:
                img[r][c-1] = 200
            if c + 1 < 28:
                img[r][c+1] = 180
    return img

def main():
    img = generate_digit_7()
    flat_img = [img[r][c] for r in range(28) for c in range(28)]

    # 3x3 edge/stroke enhancement kernel
    # [ -1, -1, -1 ]
    # [ -1,  8, -1 ]
    # [ -1, -1, -1 ]
    conv_k = [
        -1, -1, -1,
        -1,  8, -1,
        -1, -1, -1
    ]
    conv_bias = 0

    # Software simulation to get ground-truth 13x13 pooled features
    # 1. 3x3 Conv on 28x28 -> 26x26
    conv_out = [[0 for _ in range(26)] for _ in range(26)]
    for r in range(26):
        for c in range(26):
            s = conv_bias
            for kr in range(3):
                for kc in range(3):
                    s += img[r + kr][c + kc] * conv_k[kr * 3 + kc]
            # ReLU
            conv_out[r][c] = max(0, s)

    # 2. 2x2 Max Pooling -> 13x13
    pooled = [[0 for _ in range(13)] for _ in range(13)]
    for pr in range(13):
        for pc in range(13):
            p00 = conv_out[pr * 2][pc * 2]
            p01 = conv_out[pr * 2][pc * 2 + 1]
            p10 = conv_out[pr * 2 + 1][pc * 2]
            p11 = conv_out[pr * 2 + 1][pc * 2 + 1]
            pooled[pr][pc] = max(p00, p01, p10, p11)

    flat_pooled = [pooled[r][c] for r in range(13) for c in range(13)]

    # 3. Simple linear classifier for 10 classes
    # Construct weights favoring digit 7 features (top bar + diagonal)
    fc_weights = [[0 for _ in range(169)] for _ in range(10)]
    fc_biases = [0 for _ in range(10)]

    # Class 7 gets positive weights where digit 7 features are strong
    for i, val in enumerate(flat_pooled):
        if val > 100:
            fc_weights[7][i] = 4
            # other digits get lower or negative
            fc_weights[1][i] = 1 if i % 13 == 6 else -1
            fc_weights[0][i] = -2

    # Emit C header
    with open("my_test/mnist_data.h", "w") as f:
        f.write("// ============================================================================\n")
        f.write("// File    : mnist_data.h\n")
        f.write("// Purpose : Pre-compiled MNIST sample (digit 7), Conv kernel, and FC weights\n")
        f.write("// ============================================================================\n\n")
        f.write("#ifndef MNIST_DATA_H\n#define MNIST_DATA_H\n\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define EXPECTED_DIGIT 7\n")
        f.write(f"#define IMG_WIDTH      28\n")
        f.write(f"#define IMG_HEIGHT     28\n")
        f.write(f"#define IMG_PIXELS     784\n")
        f.write(f"#define POOL_SIZE      169\n")
        f.write(f"#define NUM_CLASSES    10\n\n")

        # Conv kernel
        f.write("static const int32_t cnn_kernel[9] = {\n    ")
        f.write(", ".join(str(k) for k in conv_k))
        f.write("\n};\n\n")
        f.write(f"static const int32_t cnn_bias = {conv_bias};\n\n")

        # Sample Image
        f.write("static const uint8_t mnist_sample_image[784] = {\n")
        for r in range(28):
            row_vals = ", ".join(f"0x{flat_img[r*28 + c]:02X}" for c in range(28))
            f.write(f"    {row_vals},\n")
        f.write("};\n\n")

        # FC Weights
        f.write("static const int16_t fc_weights[10][169] = {\n")
        for d in range(10):
            f.write("    { // Digit " + str(d) + "\n        ")
            f.write(", ".join(str(fc_weights[d][i]) for i in range(169)))
            f.write("\n    },\n")
        f.write("};\n\n")

        f.write("static const int32_t fc_biases[10] = {\n    ")
        f.write(", ".join(str(b) for b in fc_biases))
        f.write("\n};\n\n")
        f.write("#endif // MNIST_DATA_H\n")

    print("Generated my_test/mnist_data.h successfully.")

if __name__ == "__main__":
    main()
