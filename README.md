# RISC-V CNN Digit Recognition SoC

**RISC-V Based SoC with Hardware CNN Accelerator for Handwritten Digit Recognition**

> Honours Project — Silicon design and verification of a complete RISC-V SoC targeting the Digilent Basys3 FPGA (Xilinx Artix-7). The system classifies handwritten digits (MNIST, 28 × 28) using a hardware CNN accelerator offloaded from the processor over an AXI4 interconnect.

---

## Project Status

| Block | RTL | Standalone TB | Integration TB | Notes |
|---|---|---|---|---|
| AXI4 Interconnect (3 masters × 8 slaves) | ✅ Complete | ✅ Verified | ✅ Verified | Full address-decode, arbitration, all 8 slave ports active |
| AXI4-Lite UART IP (axi_uart_top) | ✅ Complete | ✅ Verified | ✅ Verified | TX/RX, FIFO, interrupts, parity, baud programming |
| AXI UART Bridge | ✅ Complete | — | ✅ Verified | 64-bit → 32-bit width adapter, AR-channel FSM |
| UART–Interconnect Integration | ✅ Complete | — | ✅ 19/19 pass | Full end-to-end path from master through bus to UART |
| RISC-V Processor (SweRV EL2) | 🔄 Integration planned | — | — | Western Digital VeeR EL2 RV32IMC core; AXI4 master ports to be wired to SoC bus |
| DMA Controller | 📋 Planned | — | — | |
| CNN Convolution Accelerator | 📋 Planned | — | — | 3 × 3 kernel, 9-multiplier parallel baseline |
| ReLU Unit | 📋 Planned | — | — | |
| Timer | 📋 Planned | — | — | |
| GPIO | 📋 Planned | — | — | |
| 7-Segment Display Controller | 📋 Planned | — | — | Basys3 4-digit multiplexed display |

---

## Overview

This project designs and verifies a RISC-V based System-on-Chip (SoC) for handwritten digit recognition. The SoC integrates the **Western Digital VeeR EL2 (SweRV EL2)** — a production-grade RV32IMC core from CHIPS Alliance — with instruction memory, data memory, standard peripherals, and application-specific hardware accelerators, all connected over an **AXI4 interconnect**.

The system accepts a 28 × 28 grayscale MNIST-style image over UART, offloads the 3 × 3 convolution to a dedicated hardware CNN accelerator via DMA, and classifies the digit in software running on the VeeR EL2 core. The result is displayed on the Basys3 seven-segment display and reported back over UART.

The key engineering goal is to demonstrate the performance benefit of hardware acceleration: the CNN convolution — which requires 9 multiply-accumulate operations per output pixel — is moved entirely off the general-purpose processor onto dedicated silicon, freeing the CPU for lightweight control and classification.

---

## What Has Been Built and Verified

### AXI4 Interconnect  (`rtl/interconnect/`)

A fully parameterised **3-master × 8-slave AXI4 crossbar** (`axi_interconnect.v`) with:

- Round-robin arbitration (`arbiter.v`) and a priority encoder (`priority_encoder.v`)
- Per-slave configurable base address and address width
- 64-bit data bus, 32-bit address bus, 8-bit transaction IDs
- All 8 slave ports mapped to the SoC memory map (see address map below)

A Python script (`script/axi_interconnect_wrap.py`) generates the flat port-map wrapper (`run/axi_interconnect_wrap_3x8.v`) that is used for simulation and synthesis.

**Verified by** `tb/tb_axi_interconnect_wrap_3x8.sv`:
- Read/write to all 8 slave ports from all 3 masters
- Simultaneous multi-master access and contention resolution
- VCD waveform captured for analysis

---

### AXI4-Lite UART IP  (`rtl/uart/`)

A production-quality AXI4-Lite UART core ported from open-source (BSC / CIC-IPN) and integrated into the SoC:

| File | Purpose |
|---|---|
| `axi_uart_top.v` | AXI4-Lite slave, read/write FSMs, register map |
| `uart_controller.v` | Baud generation, TX/RX coordination |
| `uart_transmitter.v` | Serial TX shift register with start/data/parity/stop framing |
| `uart_receiver.v` | Serial RX with metastability filter and bit-centre sampling |
| `uart_parity_bit_compute.v` | Configurable odd/even parity |
| `axi_internal_fifo.v` | Parameterised sync FIFO used for both TX and RX paths |
| `include/axi_uart_defines.vh` | Bus width and FIFO depth parameters |
| `include/axi_uart.vh` | UART register map and configuration bit definitions |

Features verified:
- 8N1, 8N2, 8O1, 8E1 frame formats
- Programmable baud rate divisor (DLAB access)
- RX interrupt (IER/LSR)
- TX FIFO full / empty (THRE, TEMT)
- RX FIFO ordering across multiple back-to-back bytes
- Reset during active transmission

**Verified by** `tb/tb_axi_uart_top.v` (9 standalone test cases, all passing).

---

### AXI UART Bridge  (`rtl/axi_uart_bridge.v`)

A protocol adapter that connects the 64-bit AXI4 interconnect master port `m04` to the 32-bit AXI4-Lite UART IP:

| Adaptation | Detail |
|---|---|
| Data width | 64-bit → 32-bit (lower word forwarded) |
| Address width | 32-bit → 5-bit (lower 5 bits forwarded as UART register offset) |
| ID width | 8-bit → 12-bit (zero-extended) |
| Strobe | 8-byte → 4-byte |
| `rlast` | Absent in UART; driven constant `1` (single-beat only) |
| AR-channel FSM | Latches AR request and holds `arvalid` to UART until `rvalid` fires — required because the UART IP gates `rvalid` on `arvalid` |

---

### UART–Interconnect Integration  (`tb/tb_uart_on_interconnect.sv`)

The main integration testbench connects the complete path:

```
TB AXI master  →  axi_interconnect_wrap_3x8  →  m04  →  axi_uart_bridge  →  axi_uart_top
                                                              uart_tx ──loopback──► uart_rx
                         m00..m03, m05..m07  →  dummy_slave_intg × 7
```

**7 integration test cases — 19 assertions — 0 failures:**

| Test | Coverage |
|---|---|
| T1 Reset | `uart_tx` idle during reset, LSR TEMT+THRE=1 after release |
| T2 Bridge reach | LCR write and LSR read through the full interconnect→bridge chain |
| T3 Baud config | DLAB sequence to program baud divisor, LSR stable after |
| T4 TX loopback | Write 0x41 and 0xA5 to THR, read back via RBR over serial loopback |
| T5 RX FIFO | 4-byte burst in, drain in correct order (0x11, 0x22, 0x33, 0x44) |
| T6 Interrupt | IER=0 no interrupt; IER=1 interrupt fires when RX data pending |
| T7 Address routing | IMEM (m00) write+read proves non-UART addresses still route correctly |

Run with:
```bash
cd Honours/
./run_uart_tb.sh vcs          # compile + simulate, generates run/dump.fsdb
verdi -sv -ssf run/dump.fsdb &  # open waveform in Verdi
```

---

## SoC Architecture

```
┌─────────────────────────────────────────────────────────────────────│
│                        AXI4 Interconnect                            │
│         3 Masters (VeeR EL2 IFU, VeeR EL2 LSU, DMA)                 │
│               8 Slaves  (IMEM, DMEM, DMA-reg, CNN,                  │
│                           UART, Timer, GPIO, 7-Seg)                 │
└──┬──────┬──────┬──────┬──────┬──────┬──────┬────────────────────────│
   │      │      │      │      │      │      │
 IMEM   DMEM   DMA    CNN   UART  Timer  GPIO  7-Seg
 m00    m01    m02    m03    m04   m05    m06   m07
                              │
                       axi_uart_bridge
                              │
                        axi_uart_top
                         TX ──► RX (FPGA↔PC)
```

**End-to-end flow:**
```
PC → UART → DMEM → DMA → CNN Accelerator → ReLU → Feature Map → RISC-V Classifier → 7-Segment / UART
```

---

## Memory Map

| Slave | Base Address | Size | Block |
|---|---|---|---|
| m00 | `0x0000_0000` | 256 MB | IMEM |
| m01 | `0x1000_0000` | 256 MB | DMEM |
| m02 | `0x2000_0000` | 256 MB | DMA Controller registers |
| m03 | `0x3000_0000` | 256 MB | CNN Accelerator registers |
| m04 | `0x4000_0000` | 256 MB | UART (via bridge) |
| m05 | `0x5000_0000` | 256 MB | Timer |
| m06 | `0x6000_0000` | 256 MB | GPIO |
| m07 | `0x7000_0000` | 256 MB | 7-Segment Display Controller |

### UART Register Map (offset from `0x4000_0000`)

| Offset | Register | Access | Description |
|---|---|---|---|
| `0x00` | THR / RBR | W / R | TX holding / RX buffer (DLAB=0) |
| `0x04` | IER | R/W | Interrupt enable |
| `0x08` | BAUD\_DIV | R/W | Baud rate divisor (DLAB=1) |
| `0x0C` | LCR | R/W | Line control (data bits, stop, parity, DLAB) |
| `0x14` | LSR | R | Line status (THRE, TEMT, DATA\_READY) |

---

## Repository Structure

```
Honours/
├── rtl/
│   ├── interconnect/
│   │   ├── axi_interconnect.v          # 3×8 AXI4 crossbar core
│   │   ├── arbiter.v                   # Round-robin arbiter
│   │   └── priority_encoder.v
│   ├── uart/
│   │   ├── axi_uart_top.v              # AXI4-Lite UART slave (top)
│   │   ├── uart_controller.v           # Baud + TX/RX control
│   │   ├── uart_transmitter.v          # Serial TX
│   │   ├── uart_receiver.v             # Serial RX
│   │   ├── uart_parity_bit_compute.v   # Parity logic
│   │   ├── axi_internal_fifo.v         # Sync FIFO (TX + RX)
│   │   └── include/
│   │       ├── axi_uart_defines.vh     # Bus/FIFO parameters
│   │       └── axi_uart.vh             # Register map defines
│   └── axi_uart_bridge.v               # 64b↔32b AXI protocol bridge
├── tb/
│   ├── tb_uart_on_interconnect.sv      # ★ Integration TB (UART on bus)
│   ├── tb_axi_uart_top.v               # Standalone UART TB
│   ├── tb_axi_interconnect_wrap_3x8.sv # Standalone interconnect TB
│   └── tb_aes_core.sv
├── run/
│   ├── axi_interconnect_wrap_3x8.v     # Generated wrapper (Python script)
│   ├── dump.fsdb                       # Verdi waveform (post-simulation)
│   └── simv_uart_tb                    # VCS simulation binary
├── script/
│   └── axi_interconnect_wrap.py        # Wrapper generator
├── doc/
│   ├── RISC_V_CNN_SoC_Architecture_Document.docx
│   └── Project_Abstract.pdf
├── run_uart_tb.sh                      # ★ One-shot compile + simulate script
└── README.md
```

---

## Toolchain

| Tool | Version | Purpose |
|---|---|---|
| Synopsys VCS | U-2023.03 | RTL simulation and compilation |
| Synopsys Verdi | U-2023.03-SP1 | Waveform debug (FSDB) |
| Python 3 | — | Interconnect wrapper generation |
| Target FPGA | Digilent Basys3 (Xilinx Artix-7) | Hardware implementation |

---

## Planned Work

The SoC backbone (interconnect + UART) is complete and verified. Remaining blocks to be implemented and integrated:

1. **RISC-V Processor (SweRV EL2)** — Western Digital VeeR EL2 (RV32IMC), a production-grade 9-stage dual-issue in-order core from CHIPS Alliance. The processor exposes AXI4 master ports (IFU and LSU) that will be wired directly into the SoC interconnect slave ports `s00` and `s01`. No custom CPU design is required — integration effort involves port mapping, reset sequencing, and boot-address configuration.
2. **DMA Controller** — block-transfer engine, memory-mapped CSRs
3. **CNN Convolution Accelerator** — 9-multiplier parallel 3 × 3 kernel, adder tree, bias add
4. **ReLU Unit** — single-cycle activation function, feeds feature map buffer
5. **Timer / GPIO** — standard peripheral register interfaces
6. **7-Segment Display Controller** — BCD-to-segment decoder, Basys3 digit multiplexer
7. **Full SoC Integration TB** — all blocks on bus, SweRV EL2 running firmware to drive CNN inference end-to-end
8. **FPGA Implementation** — synthesis, place-and-route, timing closure on Artix-7

---

## CNN Design Details

### Convolution Operation

```
Y[r,c] = Σ  W[i,j] × I[r+i, c+j]  +  B
         i,j ∈ {0,1,2}
```

The fully parallel baseline instantiates 9 multipliers operating in a single clock cycle, followed by a 4-level adder tree and bias accumulation. This delivers one output pixel per clock at the operating frequency.

### Weight Flow

```
Offline training → Quantise (fixed-point) → Export to .mem file
                                          → Load into DMEM at boot
                                          → DMA transfer to CNN accelerator
                                          → Hardware inference
```

### ReLU Activation

```
ReLU(x) = max(0, x)
```

Applied element-wise to every convolution output before the feature map is stored.

---

## Data Paths

| ID | From | To | Content |
|---|---|---|---|
| D1 | PC | UART RX | Serialised 28×28 pixel stream |
| D2 | UART | DMEM | 784 pixel bytes |
| D3 | DMEM | DMA | Image and weight blocks |
| D4 | DMA | CNN Accelerator | Image patches + kernel weights |
| D5 | CNN Accelerator | ReLU Unit | Raw convolution output |
| D6 | ReLU Unit | Feature Buffer | Activated feature map |
| D7 | Feature Buffer | RISC-V | Classification input |
| D8 | RISC-V | 7-Seg Controller | Predicted digit (0–9) |
| D9 | 7-Seg Controller | Basys3 display | Segment + digit-select signals |
| D10 | UART TX | PC | Status and recognition result |

---

## Performance Targets

| Metric | Target |
|---|---|
| Clock frequency | ≥ 50 MHz on Artix-7 |
| Convolution throughput | 1 output pixel / clock (fully parallel) |
| UART baud rate | 115200 bps (configurable) |
| FPGA resources | Within Basys3 limits (LUT, FF, DSP, BRAM) |
| Recognition accuracy | ≥ 90 % on MNIST test set after quantisation |
