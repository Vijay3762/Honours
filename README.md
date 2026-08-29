# RISC-V CNN Digit Recognition SoC

**RISC-V Based SoC with Hardware CNN Accelerator for Handwritten Digit Recognition**

---

## Overview

This project designs and verifies a RISC-V based System-on-Chip (SoC) for handwritten digit recognition. The SoC integrates a RISC-V processor with instruction memory, data memory, standard peripherals, and application-specific hardware accelerators connected over an AHB-Lite bus interconnect.

The system processes a 28 × 28 grayscale handwritten digit image (MNIST-style) and classifies it into one of ten digit classes (0–9). The computationally intensive 3 × 3 convolution is offloaded to a dedicated hardware CNN accelerator, while the RISC-V processor handles system control and the remaining lightweight classification.

The recognized digit is displayed on the onboard 7-segment display of the **Basys3 FPGA**, with UART providing system status and result communication to a connected PC.

---

## Problem Statement

Modern embedded systems increasingly require image-processing and machine-learning capabilities under limited computational and memory resources. CNN convolution requires repeated multiply-accumulate (MAC) operations that place a heavy load on a general-purpose processor when executed entirely in software.

This project addresses this by combining a RISC-V processor with dedicated hardware acceleration — delegating convolution to a CNN accelerator, data movement to a DMA controller, and activation to a ReLU unit — to achieve efficient digit recognition on an FPGA.

---

## SoC Architecture

The RISC-V processor is the central control unit. All components communicate over an **AHB-Lite interconnect** through memory-mapped registers.

```
PC → UART → DMEM → DMA → CNN Accelerator → ReLU → Feature Data → RISC-V Classifier → 7-Segment Display / UART
```

### Major Blocks

| Block                      | Role                                                                                 |
|----------------------------|--------------------------------------------------------------------------------------|
| RISC-V Processor (RV32I)   | Central control, program execution, IP configuration, final classification           |
| Instruction Memory (IMEM)  | Stores the RISC-V application program                                                |
| Data Memory (DMEM)         | Stores image pixels, CNN weights, biases, feature maps, and results                 |
| AHB-Lite Interconnect      | Memory-mapped system bus for CPU/DMA transactions and peripheral/accelerator access  |
| DMA Controller             | Transfers image and weight data between DMEM and CNN accelerator                    |
| CNN Convolution Accelerator| Performs 3 × 3 convolution using dedicated multipliers, adders, and accumulators     |
| ReLU Unit                  | Applies ReLU(x) = max(0, x) to convolution output                                   |
| Feature Map Buffer         | Holds processed feature data for the classification path                             |
| UART                       | PC–FPGA communication for image input, status, and result output                    |
| Timer                      | Measures processing and execution time                                               |
| GPIO                       | General-purpose control and status signals                                           |
| 7-Segment Display Controller | Maps recognized digit to Basys3 seven-segment pattern and drives multiplexing      |
| Clock & Reset Generator    | Provides common clock and reset infrastructure                                       |

---

## Additional Hardware IPs

### DMA Controller
Transfers blocks of image and CNN weight data between memory and the accelerator with minimal CPU involvement. Configured through memory-mapped source address, destination address, transfer length, control, and status registers.

### CNN Convolution Accelerator
Performs the 3 × 3 convolution using dedicated hardware multipliers, adders, and accumulators:

```
Y = Σ(Input[i,j] × Weight[i,j]) + Bias,   i,j = 0..2
```

A fully parallel baseline uses nine concurrent multipliers — one per 3 × 3 kernel element — followed by an adder tree and bias addition.

### ReLU Unit
Applies the ReLU activation function to the convolution output:

```
ReLU(x) = max(0, x)
```

Negative values are mapped to zero; positive values are retained unchanged.

### 7-Segment Display Controller
Converts the final predicted digit (0–9) into the appropriate segment pattern and drives/multiplexes the Basys3 onboard four-digit seven-segment display.

---

## Memory Architecture

| Memory              | Contents                                      | Primary Consumer          |
|---------------------|-----------------------------------------------|---------------------------|
| IMEM                | RISC-V application instructions               | RISC-V                    |
| DMEM                | 28 × 28 image (784 pixel values)              | RISC-V / DMA              |
| DMEM                | CNN weights and biases                        | DMA / CNN Accelerator     |
| DMEM / Feature Store| Processed feature maps and results            | RISC-V classifier / system|

---

## End-to-End Functional Flow

1. User draws a handwritten digit on a PC.
2. PC converts the drawing into a 28 × 28 grayscale image.
3. PC transmits the pixel stream to the FPGA over UART.
4. SoC receives the image and stores it in DMEM.
5. RISC-V initializes the system and configures all peripherals and IPs.
6. RISC-V programs the DMA with source address, destination address, and transfer length.
7. DMA transfers image and weight data to the CNN accelerator.
8. CNN accelerator performs the hardware 3 × 3 convolution.
9. Convolution output is passed to the ReLU unit.
10. Processed feature-map data is stored for the classification path.
11. RISC-V software performs remaining classification and determines digit 0–9.
12. RISC-V writes the digit to the 7-Segment Display Controller.
13. 7-Segment Controller drives the Basys3 display with the recognized digit.
14. UART reports status and recognition result; Timer measures execution time.

---

## Data Paths

| ID  | From              | To                  | Data / Purpose                   |
|-----|-------------------|---------------------|----------------------------------|
| D1  | PC                | UART                | Serialized image data            |
| D2  | UART              | DMEM                | 784 input pixel values           |
| D3  | DMEM              | DMA                 | Image / weight blocks            |
| D4  | DMA               | CNN Accelerator     | Image / weight data              |
| D5  | CNN Accelerator   | ReLU Unit           | Convolution feature data         |
| D6  | ReLU Unit         | Feature Storage     | Processed feature map            |
| D7  | Feature Storage   | RISC-V Software     | Classification input             |
| D8  | RISC-V            | 7-Segment Controller| Predicted digit                  |
| D9  | 7-Segment Controller | Basys3           | Segment and digit-select signals |
| D10 | UART              | PC                  | Status / result output           |

---

## Control Paths

| ID  | From                          | To                  | Control / Status                  |
|-----|-------------------------------|---------------------|-----------------------------------|
| C1  | RISC-V                        | AHB Interconnect    | Bus address / control transaction |
| C2  | RISC-V                        | DMA                 | DMA configuration and start       |
| C3  | RISC-V                        | CNN Accelerator     | CNN configuration and start       |
| C4  | CNN Accelerator               | RISC-V / status     | Busy / done / status              |
| C5  | RISC-V                        | ReLU Unit           | Enable / configuration            |
| C6  | RISC-V                        | 7-Segment Controller| Display control                   |
| C7  | RISC-V                        | UART                | UART configuration / control      |
| C8  | RISC-V                        | Timer               | Timer configuration / start / stop|
| C9  | RISC-V                        | GPIO                | GPIO configuration                |
| C10 | DMA / CNN / Timer / UART / GPIO | RISC-V            | Interrupt / event signals         |
| C11 | Clock / Reset Generator       | SoC                 | Clock and reset                   |

---

## Project Structure

```
Project/
├── rtl/                  # RTL source files (Verilog)
│   └── interconnect/     # AXI/AHB interconnect RTL
├── tb/                   # Testbenches (SystemVerilog)
├── run/                  # Simulation run directory (.f filelists, .v wrappers)
├── script/               # Helper scripts (Python)
├── doc/                  # Architecture and abstract documents
├── lib/                  # Library files
├── reg/                  # Register definition files
└── README.md
```

---

## Target Platform

- **FPGA Board:** Digilent Basys3 (Xilinx Artix-7)
- **Processor ISA:** RISC-V RV32I
- **Bus Protocol:** AHB-Lite
- **Simulation Tool:** Synopsys VCS
- **Waveform Viewer:** Verdi

---

## Performance Metrics

The accelerator is evaluated across:

- Execution cycles and total processing time
- MAC throughput and utilization
- FPGA resource usage (LUT, FF, DSP, BRAM)
- Memory traffic and effective bandwidth
- DMA overhead and CPU involvement
- Clock frequency
- Power estimation (if available)
- Recognition accuracy after weight quantization

---

## CNN Weight Flow

Weights are trained offline and loaded into DMEM for FPGA inference:

```
Train → Validate → Freeze Weights → Quantize → Export Memory Data → Load into DMEM → CNN Inference
```

> Final pixel width, weight width, signedness, fixed-point scaling, bias width, and memory layout are implementation parameters finalized before RTL freeze.

---

## Expected Demonstration

1. A 28 × 28 handwritten digit image is stored in Data Memory.
2. RISC-V configures the DMA and CNN accelerator through memory-mapped registers.
3. DMA transfers the required image and weight data.
4. CNN accelerator performs hardware-accelerated 3 × 3 convolution followed by ReLU.
5. RISC-V performs remaining classification and determines the digit (0–9).
6. Predicted digit is displayed on the Basys3 onboard seven-segment display.
7. UART reports processing status and recognition results; Timer provides execution-time measurements.
