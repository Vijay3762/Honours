# RISC-V CNN Digit Recognition SoC

**RISC-V Based SoC with Hardware CNN Accelerator for Handwritten Digit Recognition**

> Honours Project — Silicon design and verification of a complete RISC-V SoC targeting the Digilent Basys3 FPGA (Xilinx Artix-7). The system classifies handwritten digits (MNIST, 28 × 28) using a hardware CNN accelerator offloaded from the processor over an AXI4 interconnect.

---

## Project Status

| Block | RTL | Standalone TB | Integration TB | Notes |
|---|---|---|---|---|
| AXI4 Interconnect (3 masters × 10 slaves) | ✅ Complete | ✅ Verified | ✅ Verified | Full address-decode, arbitration, all 10 slave ports active |
| AXI4-Lite UART IP (axi_uart_top) | ✅ Complete | ✅ Verified | ✅ Verified | TX/RX, FIFO, interrupts, parity, baud programming |
| AXI UART Bridge | ✅ Complete | — | ✅ Verified | 64-bit → 32-bit width adapter, sub-word (byte lane) aware |
| UART–Interconnect Integration | ✅ Complete | — | ✅ 19/19 pass | Full end-to-end path from master through bus to UART |
| RISC-V Processor (SweRV EL2) | ✅ Integrated | — | ✅ 4/4 pass | VeeR EL2 RV32IMC, IFU+LSU as AXI4 masters, boots from the ICCM |
| CPU → AXI → UART read/write test | ✅ Complete | — | ✅ All pass | Core configures the UART, transmits and receives a byte |
| AXI GPIO (axi_gpio) | ✅ Complete | ✅ 86/86 pass | ✅ 29/29 pass (Interconnect) + 20/20 pass (CPU+Interconnect+GPIO) | 16-bit GPIO, per-bit direction, SET/CLR/TOGGLE, two-flop sync, integrated with CPU & Interconnect |
| CPU → AXI → GPIO read/write test | ✅ Complete | — | ✅ 20/20 pass | CPU core configures GPIO DIR, OUT, SET/CLR/TOGGLE, reads pad inputs via AXI bus |
| AXI Timer (axi_timer) | ✅ Complete | ✅ 38/38 pass | ✅ 31/31 pass (Interconnect) + 14/14 pass (CPU+Interconnect+Timer) | 32-bit down-counter, auto-reload, compare match, W1C status, IRQ, verified with CPU Master & VeeR |
| CPU → AXI → Timer read/write test | ✅ Complete | — | ✅ 14/14 pass | CPU core configures Timer LOAD, CMP, CTRL, observes countdown & compare match IRQ, clears status via W1C |
| CNN Convolution Accelerator (axi_cnn_conv) | ✅ Complete | ✅ 41/41 pass | ✅ 29/29 pass | 3×3 kernel, 9-multiplier parallel, 4-level adder tree, bias, integrated at m03 |
| DMA Controller (`axi_dma_ctrl`) | ✅ Complete | ✅ Verified (38/38 pass) | ✅ 24/24 pass (Interconnect Port 2) | AXI4 slave (CPU programming) + AXI4 master (data movement), interrupt, W1C status, auto-restart |
| ReLU Unit (`relu_unit` / `axi_relu`) | ✅ Complete | ✅ 76/76 pass (Core) + 35/35 pass (AXI) | ✅ 44/44 pass (Interconnect Port 8) | Element-wise max(0, x), streaming + AXI4 slave, clipping, sparsity profiling, integrated at m08 |
| 7-Segment Display Controller (`axi_seven_seg`) | ✅ Complete | ✅ 46/46 pass | ✅ 11/11 pass (Interconnect Port 7) | Basys3 4-digit multiplexed cathode/anode scanner, hex decoder, raw segment mode, DP control |
| 2x2 Spatial Pooling Unit (`pooling_unit` / `axi_pooling`) | ✅ Complete | ✅ 127/127 pass (Core) + 31/31 pass (AXI) | ✅ 14/14 pass (Interconnect Port 9) | 2x2 spatial downsampler, Max/Avg/Min/Bypass modes, argmax index, symmetric rounding, integrated at m09 |
| Full SoC MNIST End-to-End Inference | ✅ Integrated | — | ✅ Ready | VeeR EL2 running full MNIST pipeline: Conv -> ReLU -> Pool -> Software FC Classifier -> 7-Seg / UART |
| UVM Verification Environment (Core + AXI + UART) | ✅ Complete | ✅ 100% Pass (VCS & Verdi) | ✅ 100% Pass | Full UVM 1.2 testbench with AXI4 & UART VIPs, Scoreboard, Functional Coverage |

---

## VeeR EL2 Core → AXI → UART (new)

The processor is now integrated and drives the UART itself.  The test proves
the complete path

```
VeeR EL2 (LSU)  →  AXI4 interconnect  →  axi_uart_bridge  →  axi_uart_top  →  UART
```

| File | Purpose |
|---|---|
| `rtl/veer_core_wrapper.sv` | Wraps `el2_veer_wrapper`: exposes the IFU/LSU AXI4 masters, ties off the SB/DMA/AHB/JTAG ports, connects the SRAM interface. **Generated** by `script/gen_veer_wrapper.py` |
| `rtl/veer_iccm_ram.sv` | Behavioural model of the core's ICCM (instruction memory) behind the `el2_mem_if` interface |
| `rtl/soc_top_veer.sv` | SoC top: core + 2×8 interconnect + IMEM/DMEM + UART bridge |
| `tb/tb_cpu_uart_veer.sv` | Testbench: loopback decode of the serial line + checks of the values the core read back |
| `script/run_cpu_uart_veer.sh` | Build firmware → `xvlog` → `xelab` → `xsim` (one shot) |
| `script/patch_veer_config.py` | Patches the generated VeeR configuration (see below) |

---

## CPU Core → AXI → GPIO Integration (Icarus Verilog & VeeR EL2)

The GPIO controller (`axi_gpio`) is integrated at interconnect port 6 (`m06`, base `0x6000_0000`) and connected to external GPIO pads (`gpio_pads_i`, `gpio_pads_o`, `gpio_pads_oen`).

The end-to-end CPU → Interconnect → GPIO path is verified using Icarus Verilog (`iverilog` / `vvp`):

```
CPU Master / Core  →  AXI4 Interconnect (m06)  →  axi_gpio  →  GPIO Pads (i/o/oen)
```

### Verified Features:
1. **Reset defaults**: `gpio_pads_o = 0x0000`, `gpio_pads_oen = 0x0000`.
2. **Peripheral identification**: CPU reads `GPIO_ID` (`0x4749504F` = "GPIO") and `GPIO_VERSION` (`0x00010000` = v1.0).
3. **Direction register (`GPIO_DIR`)**: CPU writes `0x00FF` (lower 8 output, upper 8 input); external `gpio_pads_oen` immediately asserts `0x00FF`.
4. **Data output (`GPIO_OUT`)**: CPU writes `0x00A5`; external `gpio_pads_o` outputs `0x00A5`.
5. **Atomic set (`GPIO_OUT_SET`)**: CPU writes mask `0x0050` (bits 4, 6); `GPIO_OUT` becomes `0x00F5`.
6. **Atomic clear (`GPIO_OUT_CLR`)**: CPU writes mask `0x0005` (bits 0, 2); `GPIO_OUT` becomes `0x00F0`.
7. **Atomic toggle (`GPIO_OUT_TOGGLE`)**: CPU writes mask `0x0033` (bits 0, 1, 4, 5); `GPIO_OUT` becomes `0x00C3`.
8. **Physical input sampling (`GPIO_IN`)**: External stimulus (`0x5A3C`) applied to `gpio_pads_i` is synchronized through two-stage flip-flops and correctly read by the CPU.
9. **64-bit sub-word addressing & read data duplication**: Enhanced `axi_gpio` handles both 32-bit and 64-bit AXI masters (including VeeR EL2 LSU).

### Running the CPU-to-GPIO Simulation:
```bash
./script/run_cpu_gpio_tb.sh
```
All 20/20 test assertions pass cleanly.

---

## CPU Core → AXI → Timer Integration (Icarus Verilog & VeeR EL2)

The 32-bit Timer unit (`axi_timer`) is integrated at interconnect port 5 (`m05`, base `0x5000_0000`) and its interrupt line (`timer_irq_o`) is routed both to the external pin and into the processor core (`core.timer_int`).

The end-to-end CPU → Interconnect → Timer path is verified using Icarus Verilog (`iverilog` / `vvp`):

```
CPU Master / Core  →  AXI4 Interconnect (m05)  →  axi_timer  →  timer_irq_o / core.timer_int
```

### Verified Features:
1. **Reset defaults**: `timer_irq = 0`, `TIMER_CTRL = 0`, `TIMER_STATUS = 0`.
2. **Peripheral identification**: CPU reads `TIMER_ID` (`0x54494D52` = `"TIMR"`) and `TIMER_VERSION` (`0x00010000` = v1.0).
3. **Load register (`TIMER_LOAD`)**: CPU writes initial count `30` and reads back `30`.
4. **Compare register (`TIMER_COMPARE`)**: CPU writes threshold `15` and reads back `15`.
5. **Timer control & start (`TIMER_CTRL`)**: CPU writes `0x05` (`enable=1`, `irq_en=1`, one-shot mode).
6. **Hardware countdown**: Timer counter decrements each cycle while running; readback confirms `TIMER_VALUE < 30`.
7. **Compare match detection & interrupt**: When `TIMER_VALUE == TIMER_COMPARE` (15), the match flag (`TIMER_STATUS[0]`) asserts and `timer_irq_o` goes HIGH.
8. **Write-1-to-Clear (W1C) status**: CPU writes `1` to `TIMER_STATUS[0]`; the match flag clears to `0` and `timer_irq_o` de-asserts LOW.
9. **Firmware testbench**: RISC-V Bare-Metal assembly test (`my_test/timer_test.S` -> `my_test/timer_test.hex`) compiled with `riscv64-unknown-elf-gcc` for execution on the VeeR EL2 core.

### Running the CPU-to-Timer Simulation:
```bash
./script/run_cpu_timer_tb.sh
```
All 14/14 test assertions pass cleanly.

---

## DMA Controller → AXI Interconnect Integration

The DMA controller (`axi_dma_ctrl`) is integrated at interconnect port 2 (`m02`, base `0x2000_0000`) for CPU-initiated register programming, while its AXI4 master interface performs high-throughput 64-bit data movement between memory and accelerator peripherals.

```
CPU Master / Core  →  AXI4 Interconnect (m02)  →  axi_dma_ctrl (Slave)
DMA Master (axi_dma_ctrl)  →  AXI4 Interconnect / Memory (Read & Write Channels)
```

### Verified Features:
1. **Reset defaults**: `CTRL = 0x0`, `STATUS = 0x0`, `IRQ = 0`.
2. **Register programming & readback**: `SRC_ADDR`, `DST_ADDR`, `LEN`, `CTRL`.
3. **Single-beat DMA transfer**: 64-bit beat moved between source and destination with proper AW/W handshakes.
4. **Multi-beat burst transfer**: 4 sequential beats moved with automatic address incrementing (`+8` bytes/beat).
5. **Interrupt & W1C handling**: `dma_irq_o` assertion on completion; deasserted via W1C write to `DMA_IRQ_STAT` (`0x14`).
6. **RO register protection**: Writes to `DMA_ID` and `DMA_VERSION` return `SLVERR` (`2'b10`).
7. **Address error handling**: Out-of-aperture accesses return `DECERR` (`2'b11`), misaligned addresses return `SLVERR` (`2'b10`).
8. **Byte strobe (WSTRB) partial writes**: Byte 0 and byte 1 write masking verified.

### Running the DMA Verification Suite:
```bash
./script/run_dma_on_interconnect.sh
```
- Standalone: 38/38 checks pass cleanly.
- Interconnect integration: 24/24 checks pass cleanly.

---

## ReLU Unit → AXI Interconnect Integration

The parameterized ReLU activation engine (`axi_relu`) is integrated at interconnect port 3 (`m03`, base `0x3000_0000`, 4 KiB aperture) to perform hardware-accelerated non-linear activations and activation sparsity profiling for CNN layers.

```
CPU Master / DMA  →  AXI4 Interconnect (m03)  →  axi_relu (Slave)  →  relu_unit (Core)
```

### Verified Features:
1. **Reset & peripheral identification**: `RELU_CTRL = 0x01` (enabled by default), `RELU_ID = 'RELU'` (`0x52454C55`), `RELU_VERSION = v1.0` (`0x00010000`).
2. **Configuration & thresholds**: Dynamic programming of `RELU_CTRL` (enable, bypass, clip_en) and `RELU_CLIP_MAX`.
3. **Standard positive ReLU activation**: Linear passthrough $\max(0, x)$ with status flag assertions (`last_active=1`, `last_zero=0`).
4. **Negative clamping & zero handling**: Signed negative numbers clamped to zero with `last_zero=1`.
5. **Boundary values**: Verification across extreme 32-bit signed limits (`0x7FFF_FFFF`, `0x8000_0000`, `-1`).
6. **Bypass mode**: Unclamped signed passthrough for linear layer inspection.
7. **Saturated clipping activation (ReLU6)**: Saturation at programmable `CLIP_MAX` threshold with `last_clipped=1`.
8. **Sparsity & performance profiling**: Real-time tracking of `ZERO_COUNT` and `TOTAL_COUNT` metrics, with single-pulse `reset_stats`.
9. **Read-only protection**: Writes to `STATUS`, `DOUT`, `ZERO_COUNT`, `TOTAL_COUNT`, `ID`, `VERSION` return `SLVERR` (`2'b10`).
10. **Address decoding & apertures**: Out-of-aperture accesses return `DECERR` (`2'b11`), misaligned accesses return `SLVERR` (`2'b10`).
11. **WSTRB partial writes**: Sub-word byte strobe write masking verified on `CLIP_MAX`.
12. **Interconnect multi-slave routing**: Independent access to `m02`, `m03` (ReLU), and `m04` without bus locking or cross-talk.

### Running the ReLU Verification Suite:
```bash
./script/run_relu_tb.sh
```
- Standalone core (`tb_relu_unit`): 76/76 checks pass cleanly.
- Memory-mapped slave (`tb_axi_relu`): 35/35 checks pass cleanly.
- Interconnect integration (`tb_relu_on_interconnect`): 44/44 checks pass cleanly.
| `script/gen_veer_wrapper.py` | Regenerates `rtl/veer_core_wrapper.sv` from the core's port list |
| `my_test/uart_test.S` | The firmware (assembly) |
| `my_test/soc_test.ld` | Linker script (code at the ICCM base `0xEE00_0000`) |
| `my_test/make_hex.py` | ELF → `$readmemh` word list (32/64 bit wide) |

### Running it

```bash
cd Honours
source /tools/Xilinx/Vivado/2024.2/settings64.sh
./script/run_cpu_uart_veer.sh
```

Expected output:

```
  [PASS] first transmitted byte  = 0x41 ('A')
  [PASS] second transmitted byte = 0x42 ('B')
  [PASS] RBR read by the core   = 0x41 ('A')
  [PASS] LSR read by the core   = 0x60 (TX idle, RX drained)
  *** ALL TESTS PASSED ***
```

The waveform is written to `work_cpu_uart_veer/cpu_uart_veer.wdb` and can be
opened in Vivado (*Tools → Open Simulation Waveform*).

### What the test does

1. The core boots from the reset vector `0xEE00_0000` (the ICCM) and
   executes the program from `my_test/uart_test.S`.
2. It programs the UART through the AXI bus: `LCR = 0x83` (DLAB=1, 8N1),
   `BAUD_DIV = 50`, `LCR = 0x03` (8N1), `IER = 1`.
3. It writes `'A'` into the THR → the byte appears on `uart_tx`.
4. The testbench loops `uart_tx` back to `uart_rx`; the core polls the LSR
   until `DATA_READY` is set and then reads the RBR → it gets `0x41`.
5. It stores the received byte and the LSR into the DMEM (`0x1000_0000`) and
   writes `'B'` into the THR.

### UART register map seen by the CPU

The LSU of the VeeR EL2 issues **64-bit** AXI beats with **8-byte aligned**
addresses and carries the byte select in `WSTRB`.  A 16550 style register map
(offsets `0x00, 0x04, 0x08, 0x0C, 0x14`) therefore cannot be used directly:
a 32-bit access to the LSR (`0x14`) would be an access to the beat at
`0x10`, and the slave cannot know which half the core will use.

The bridge therefore offers a **CPU64 window with 8-byte strides**
(`MAP_MODE = 1`):

| CPU offset (from `0x4000_0000`) | Register | IP offset (16550) |
|---|---|---|
| `0x00` | THR / RBR | `0x00` |
| `0x08` | IER | `0x04` |
| `0x10` | BAUD_DIV | `0x08` |
| `0x18` | LCR | `0x0C` |
| `0x20` | LSR | `0x14` |

Every register is 8-byte aligned, so a 64-bit master always uses the low
half of the beat.  The legacy 16550 map is still available
(`MAP_MODE = 0`, the default) and is used by `rtl/soc_top.sv`.

> Note: the UART IP gates the `DATA_READY` bit of the LSR with the interrupt
> enable, so `IER` has to be written before the receive buffer can be polled.
> The interrupt itself stays masked because `mstatus.MIE` is 0.

### VeeR EL2 configuration

The core is configured as a **pure AXI master with the ICCM as its local
instruction memory** (`script/patch_veer_config.py`):

| Parameter | Value | Reason |
|---|---|---|
| `ICCM_ENABLE` / `ICCM_ONLY` | 1 | the boot program lives in the ICCM |
| `ICACHE_ENABLE` | 0 | no instruction cache → no external I$ SRAM model needed |
| `DCCM_ENABLE` | 0 | no data cache memory |
| `FAST_INTERRUPT_REDIRECT` | 0 | needs the DCCM |

Without this the core stalls: with the instruction cache disabled the fetched
line is streamed into the miss buffer and the critical word is overwritten
before it can be executed (the bypass path of the core only works with the
cache).  The ICCM is the supported way of running the core from local memory.

The patch script decodes the 2294-bit packed parameter literal in
`rtl/cpu/veer_el2/config/el2_param.vh` using the struct definition in
`el2_pdef.vh`, so the configuration stays consistent with the upstream
`veer_config_gen` output.

### Bugs found and fixed while integrating the core

| Where | Problem | Fix |
|---|---|---|
| `rtl/axi_uart_bridge.v` | the bridge only kept 5 address bits → the LSR at `0x20` was truncated to `0x00` (RBR) | internal offset width is now 6 bits (`OFF_W`) |
| `rtl/axi_uart_bridge.v` | AW and W were passed through together, but a standard AXI master may present them in different cycles (the VeeR LSU does) | the bridge latches AW and W separately and presents them to the UART IP together |
| `rtl/axi_uart_bridge.v` | the byte lane of a 64-bit write was ignored | the data half and the register offset are now taken from `WSTRB` |
| `rtl/axi_mem.sv` | a write whose address and data arrive in the same cycle never produced a B response → the interconnect hung | the write is accepted when the data arrives and an address was seen before or in the same cycle |
| `rtl/interconnect/axi_interconnect.v` | the response was routed with the *live* arbiter grant; a master that drops its request early (the VeeR LSU does) moved the grant and the response went to the wrong master | the master of the running transaction is captured in `s_sel_reg` |
| `rtl/interconnect/axi_interconnect.v` | the "acknowledge" of a transaction was lost when the grant moved before the response was accepted | a sticky `txn_rsp_done` flag releases `STATE_WAIT_IDLE` |
| `rtl/interconnect/axi_interconnect.v` | the arbiter releases the grant as soon as the master drops its request, so the acknowledge of the running transaction was lost and `STATE_WAIT_IDLE` never ended | the master of the transaction is captured (`s_sel_reg`) and a sticky `txn_rsp_done` flag ends `STATE_WAIT_IDLE` |

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

### DMA Controller (`rtl/dma/axi_dma_ctrl.sv`)

A dual-interface DMA Controller acting as both an AXI4 slave (for CPU programming) and an
AXI4 master (for data movement). Designed and verified with Vivado XSim 2024.2.

| Register | Offset | Access | Description |
|---|---|---|---|
| `DMA_CTRL` | `0x00` | R/W | bit0: enable, bit1: start (W1C trigger), bit2: irq_en, bit3: dir, bit4: auto_restart |
| `DMA_SRC_ADDR` | `0x04` | R/W | Source address (AXI master read) |
| `DMA_DST_ADDR` | `0x08` | R/W | Destination address (AXI master write) |
| `DMA_LEN` | `0x0C` | R/W | Transfer length in 64-bit beats (1–255) |
| `DMA_STATUS` | `0x10` | R/W | bit0: busy, bit1: done, bit2: error (done/error W1C) |
| `DMA_IRQ_STATUS` | `0x14` | R/W | bit0: done_irq (W1C) |
| `DMA_ID` | `0x18` | R/O | Fixed peripheral ID `"DMA "` |
| `DMA_VERSION` | `0x1C` | R/O | v1.0 |

**Features:**
- AXI4 slave interface (64-bit data, 32-bit addr, 8-bit ID) matching the interconnect
- AXI4 master interface for data movement (read from SRC, write to DST)
- Independent AW/W handshake support (handles masters that present AW and W in different cycles)
- Write-1-to-clear status flags (done, error, irq)
- Auto-restart mode for continuous transfers
- Interrupt output (`dma_irq_o`)
- Burst rejection with SLVERR response

**Verified by** `tb/tb_axi_dma.sv` (38 checks, all passing):
- Reset defaults, register write/readback
- Single-beat DMA transfer (read src → write dst)
- Multi-beat DMA transfer (4 beats, loopback memory)
- DMA done interrupt generation and W1C clear
- RO register writes → SLVERR
- Invalid / reserved / misaligned / out-of-aperture addresses
- WSTRB partial writes

**Integration TB:** `tb/tb_dma_on_interconnect.sv` (DMA on m02)

Run with:
```bash
cd Honours/
source /tools/Xilinx/Vivado/2024.2/settings64.sh
xvlog -sv -i rtl/uart/include \
    rtl/dma/axi_dma_ctrl.sv \
    rtl/interconnect/axi_interconnect.v \
    rtl/interconnect/arbiter.v \
    rtl/interconnect/priority_encoder.v \
    rtl/interconnect/axi_interconnect_wrap_3x8.v \
    rtl/axi_mem.sv \
    rtl/axi_dummy_slave.sv \
    tb/tb_dma_on_interconnect.sv
xelab -debug typical -timescale 1ns/1ps -snapshot tb_dma_on_interconnect tb_dma_on_interconnect
xsim tb_dma_on_interconnect -runall
```

---

### CPU → AXI → UART Read/Write Test (direct, no interconnect)

A standalone testbench (`tb/tb_cpu_uart_direct.sv`) connects a simple CPU-like AXI4
master directly to the UART bridge, bypassing the interconnect. Validates the
complete CPU → bridge → UART IP path.

**Verified by** `tb/tb_cpu_uart_direct.sv` (all tests passing):
- CPU writes LCR, BAUD_DIV, THR registers through the AXI bridge
- UART transmits 'A' (0x41) and 'B' (0x42) on uart_tx
- UART RX loopback captures both bytes correctly
- CPU reads LSR and RBR (receive buffer) successfully

Run with:
```bash
cd Honours/
source /tools/Xilinx/Vivado/2024.2/settings64.sh
./script/run_cpu_uart_direct.sh
open work_cpu_uart_direct/cpu_uart_direct.wdb   # waveform in Vivado
```

## SoC Architecture

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                   AXI4 Interconnect                                    │
│                     3 Masters (VeeR EL2 IFU, VeeR EL2 LSU, DMA Master)                 │
│         10 Slaves (IMEM, DMEM, DMA-reg, CNN, UART, Timer, GPIO, 7-Seg, ReLU, Pooling)   │
└──┬──────┬──────┬──────┬──────┬───────────────┬──────┬──────┬────────┬────────┬─────────┘
   │      │      │      │      │               │      │      │        │        │
 IMEM   DMEM   DMA    CNN   UART             Timer  GPIO  7-Seg    ReLU    Pooling
 m00    m01    m02    m03   m04               m05    m06   m07      m08      m09
                             │
                      axi_uart_bridge
                             │
                        axi_uart_top
                         TX ──► RX (FPGA↔PC)
```

**End-to-end flow:**
```
PC → UART → DMEM → DMA → CNN Accelerator → ReLU → Pooling → RISC-V Classifier → 7-Segment / UART
```

---

## Memory Map

| Slave | Base Address | Size | Block |
|---|---|---|---|
| m00 | `0x8000_0000` | 256 MB | IMEM (AXI instruction memory / firmware) |
| m01 | `0x1000_0000` | 256 MB | DMEM (Data memory, images, weights, features) |
| m02 | `0x2000_0000` | 256 MB | DMA Controller registers (`axi_dma_ctrl`) |
| m03 | `0x3000_0000` | 256 MB | CNN Conv Accelerator registers (`axi_cnn_conv`) |
| m04 | `0x4000_0000` | 256 MB | UART (via `axi_uart_bridge`) |
| m05 | `0x5000_0000` | 256 MB | Timer (`axi_timer`) |
| m06 | `0x6000_0000` | 256 MB | GPIO (`axi_gpio`) |
| m07 | `0x7000_0000` | 256 MB | 7-Segment Display Controller (`axi_seven_seg`) |
| m08 | `0x9000_0000` | 256 MB | ReLU Unit (`axi_relu`) |
| m09 | `0xA000_0000` | 256 MB | 2x2 Spatial Pooling Engine (`axi_pooling`) |

The processor boots from `0xEE00_0000` (the ICCM of the core, outside the
interconnect) and reaches the peripherals over the bus.

### UART Register Map (offset from `0x4000_0000`)

Classic 16550 spacing (used by the 32-bit style masters, `MAP_MODE = 0`):

| Offset | Register | Access | Description |
|---|---|---|---|
| `0x00` | THR / RBR | W / R | TX holding / RX buffer (DLAB=0) |
| `0x04` | IER | R/W | Interrupt enable |
| `0x08` | BAUD\_DIV | R/W | Baud rate divisor (DLAB=1) |
| `0x0C` | LCR | R/W | Line control (data bits, stop, parity, DLAB) |
| `0x14` | LSR | R | Line status (THRE, TEMT, DATA\_READY) |

CPU64 window (used by the VeeR EL2, `MAP_MODE = 1`, 8-byte strides):

| Offset | Register | IP offset |
|---|---|---|
| `0x00` | THR / RBR | `0x00` |
| `0x08` | IER | `0x04` |
| `0x10` | BAUD\_DIV | `0x08` |
| `0x18` | LCR | `0x0C` |
| `0x20` | LSR | `0x14` |

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
│   ├── cpu/veer_el2/                     # Western Digital VeeR EL2 (RV32IMC)
│   ├── veer_core_wrapper.sv             # ★ core integration wrapper (generated)
│   ├── veer_iccm_ram.sv                 # ★ ICCM model behind el2_mem_if
│   ├── soc_top_veer.sv                  # ★ SoC with the real processor
│   ├── axi_mem.sv                       # AXI slave memory model
│   └── axi_dummy_slave.sv               # AXI slave dummy (unused ports)
│   ├── simple_cpu_master.sv             # AXI4 test master (CPU replacement for sim)
│   ├── soc_top.sv                       # ★ SoC top (interconnect + peripherals)
│   ├── soc_top_direct.sv                # ★ Direct CPU→UART test (no interconnect)
│   ├── dma/
│   │   └── axi_dma_ctrl.sv              # ★ DMA Controller (slave+master)
│   └── relu/
│       ├── relu_unit.sv                 # ★ Streaming ReLU activation engine
│       └── axi_relu.sv                  # ★ AXI4 memory-mapped ReLU slave
├── tb/
│   ├── tb_relu_unit.sv                  # Standalone ReLU streaming TB
│   ├── tb_axi_relu.sv                   # Standalone AXI ReLU TB
│   ├── tb_uart_on_interconnect.sv      # ★ Integration TB (UART on bus)
│   ├── tb_cpu_uart_veer.sv              # ★ Core → AXI → UART test
│   ├── tb_axi_uart_top.v               # Standalone UART TB
│   ├── tb_axi_interconnect_wrap_3x8.sv # Standalone interconnect TB
│   ├── tb_axi_timer.sv                 # Standalone Timer TB
│   ├── tb_timer_on_interconnect.sv     # Integration TB (Timer on bus)
│   ├── tb_axi_gpio.sv                  # Standalone GPIO TB
│   ├── tb_gpio_on_interconnect.sv      # Integration TB (GPIO on bus)
│   ├── tb_axi_dma.sv                   # Standalone DMA TB
│   ├── tb_dma_on_interconnect.sv       # Integration TB (DMA on bus)
│   ├── tb_cpu_uart.sv                  # SoC-level CPU→UART test
│   ├── tb_cpu_uart_direct.sv          # Direct CPU→UART test
│   └── tb_aes_core.sv
├── run/
│   ├── axi_interconnect_wrap_3x8.v     # Generated wrapper (Python script)
│   ├── dump.fsdb                       # Verdi waveform (post-simulation)
│   └── simv_uart_tb                    # VCS simulation binary
├── my_test/
│   ├── uart_test.S                     # ★ firmware for the core test
│   ├── soc_test.ld                     # ★ linker script (ICCM base)
│   ├── make_hex.py                     # ★ ELF → $readmemh image
│   └── Makefile                        # toolchain rules
├── script/
│   ├── axi_interconnect_wrap.py        # Wrapper generator
│   ├── gen_veer_wrapper.py             # ★ generates rtl/veer_core_wrapper.sv
│   ├── patch_veer_config.py            # ★ patches the VeeR configuration
│   ├── run_cpu_uart_veer.sh            # ★ core → UART test (Vivado XSim)
│   ├── run_cpu_uart_tb.sh              # ★ SoC-level CPU→UART test (Vivado XSim)
│   ├── run_cpu_uart_direct.sh          # ★ Direct CPU→UART test (Vivado XSim)
│   ├── run_dma_on_interconnect.sh      # ★ DMA on interconnect test (Vivado XSim)
│   └── run_relu_tb.sh                  # ★ ReLU test suite (Verilator / Vivado)
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
| Xilinx Vivado (xvlog / xelab / xsim) | 2024.2 | RTL simulation, waveform (WDB) and FPGA synthesis |
| RISC-V GCC | `riscv64-unknown-elf-gcc` 16.1.0 (`/opt/riscv/bin`) | firmware build (RV32IMC, ilp32) |
| Python 3 | — | wrapper / configuration / image generators |
| Synopsys VCS / Verdi | U-2023.03 | optional alternative for simulation (not required) |
| Target FPGA | Digilent Basys3 (Xilinx Artix-7) | Hardware implementation |

Vivado alone is enough to compile and simulate the whole design: `xvlog`
compiles the VeeR EL2 core and the SoC, `xelab` elaborates, `xsim` runs and
writes a `.wdb` waveform.  The RISC-V compiler is only needed for the
firmware.  Two notes:

* `el2_veer_wrapper` has two SystemVerilog **interface ports**, so it can
  never be the top level of an `xelab` run – it always has to be instantiated
  inside a wrapper module (`rtl/veer_core_wrapper.sv`).
* `xvlog` cannot read a `@file` list – the sources have to be passed on the
  command line (see `script/run_cpu_uart_veer.sh`).

---

## Completed & Verified IP Blocks

The full suite of application-specific and peripheral IP blocks has been successfully designed, verified standalone, and integrated:
1. **AXI4-Lite UART** (`axi_uart_top` + `axi_uart_bridge`) — verified standalone and end-to-end with VeeR EL2 core
2. **AXI GPIO** (`axi_gpio`) — 16-bit configurable I/O, verified standalone, on interconnect, and with CPU core
3. **AXI Timer** (`axi_timer`) — 32-bit auto-reload down counter with compare match and interrupt
4. **CNN Convolution Accelerator** (`axi_cnn_conv`) — 3×3 kernel with 9 parallel DSP multipliers and adder tree
5. **DMA Controller** (`axi_dma_ctrl`) — AXI4 master + slave engine for high-throughput image and weight transfers
6. **ReLU Unit** (`relu_unit` / `axi_relu`) — streaming activation core + AXI slave with clipping and sparsity profiling
7. **7-Segment Display Controller** (`axi_seven_seg`) — 4-digit dynamic multiplexer mapped to interconnect Port 7 (`0x7000_0000`)
8. **2×2 Spatial Pooling Engine** (`pooling_unit` / `axi_pooling`) — Max, Average, Min, and Bypass pooling for feature downsampling

## Remaining Planned Work

1. **Full SoC End-to-End Firmware Execution** — SweRV EL2 running full MNIST recognition pipeline: DMA transfer -> Conv -> ReLU -> Pool -> Classification -> 7-Segment Display
2. **FPGA Implementation** — synthesis, place-and-route, timing closure on Xilinx Artix-7 (Digilent Basys3)
   (the ICCM model in `rtl/veer_iccm_ram.sv` to be mapped to FPGA block RAM)

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

---

## UVM Verification Environment (VeeR Core + AXI Interconnect + UART)

An enterprise-grade **Universal Verification Methodology (UVM 1.2)** verification environment is established under `verif/` to verify the co-operative operation of the **VeeR EL2 RISC-V Core**, **AXI4 Multi-Master Interconnect**, and the **AXI UART peripheral subsystem** under **Synopsys VCS** and **Verdi**.

### Architecture Overview

```
                               +---------------------------------------------------------+
                               |                    UVM TESTBENCH TOP                    |
                               |                                                         |
  +----------------------+     |    +-------------------+       +-------------------+    |
  |    AXI Master VIP    |     |    |   Scoreboard      |       | Functional        |    |
  |  - Sequencer         |     |    |  - AXI Writes     |       | Coverage          |    |
  |  - Driver (s02 port) |---->|    |  - AXI Reads      |       |  - AXI Bus Types  |    |
  |  - Monitor           |     |    |  - UART TX Match  |       |  - UART Regs 100% |    |
  +----------------------+     |    |  - UART RX Match  |       +-------------------+    |
                               |    +-------------------+                 ^              |
  +----------------------+     |              ^                           |              |
  |    AXI Snoop VIP     |     |              |                           |              |
  |  - Passive Monitor   |-----+--------------+---------------------------+              |
  |    (m04 slave port)  |     |                                                         |
  +----------------------+     |                                                         |
                               |    +-----------------------------------------------+    |
  +----------------------+     |    |                      DUT                      |    |
  |    UART VIP          |     |    |  +------------+   +--------------+   +--------+   |    |
  |  - Driver (rx_i)     |====>|===>|  | VeeR EL2   |-->| AXI Crossbar |-->| AXI    |==>|===> (uart_tx)
  |  - Monitor (tx_o)    |<====|<===|  | RISC-V Core|   | (3M x 10S)   |   | Bridge |   |    |
  +----------------------+     |    |  +------------+   +--------------+   +--------+   |    |
                               |    +-----------------------------------------------+    |
                               +---------------------------------------------------------+
```

### Verification IPs & Testbench Components (VIPs)

1. **AXI4 Master & Passive Monitor VIP (`verif/vip/axi/`):**
   - [`axi_agent_config.sv`](verif/vip/axi/axi_agent_config.sv): Agent configuration object managing active/passive mode, widths, and virtual interface handles.
   - [`axi_driver.sv`](verif/vip/axi/axi_driver.sv): Active master driver retrieving `vif` via `uvm_config_db::get` and executing AW, W, B, AR, R handshakes.
   - [`axi_monitor.sv`](verif/vip/axi/axi_monitor.sv): Bus monitor retrieving `vif` via `uvm_config_db::get` and capturing bus transactions.
   - [`axi_base_seq.sv`](verif/vip/axi/axi_base_seq.sv), [`axi_write_seq.sv`](verif/vip/axi/axi_write_seq.sv), [`axi_read_seq.sv`](verif/vip/axi/axi_read_seq.sv): Individual reusable sequence files.
   - [`axi_agent.sv`](verif/vip/axi/axi_agent.sv): Active and passive AXI agent.

2. **UART Serial VIP (`verif/vip/uart/`):**
   - [`uart_agent_config.sv`](verif/vip/uart/uart_agent_config.sv): Configuration object managing baud divisors, parity, and stop bits.
   - [`uart_driver.sv`](verif/vip/uart/uart_driver.sv): Active serial driver retrieving `vif` via `uvm_config_db::get` and driving timed bitstreams into `uart_rx_i`.
   - [`uart_monitor.sv`](verif/vip/uart/uart_monitor.sv): Mid-bit sampler monitoring `uart_tx_o` and publishing frames.
   - [`uart_base_seq.sv`](verif/vip/uart/uart_base_seq.sv), [`uart_rx_seq.sv`](verif/vip/uart/uart_rx_seq.sv): Individual reusable UART sequences.
   - [`uart_agent.sv`](verif/vip/uart/uart_agent.sv): Active UART agent.

3. **Virtual Sequencer & Virtual Sequences (`verif/tb/`, `verif/seq/`):**
   - [`soc_virtual_sequencer.sv`](verif/tb/soc_virtual_sequencer.sv): Top-level sequencer holding sub-sequencer handles (`master_axi_seqr`, `uart_seqr`).
   - [`soc_base_virtual_seq.sv`](verif/seq/soc_base_virtual_seq.sv): Base virtual sequence declaring `p_sequencer`.
   - [`soc_virtual_seq.sv`](verif/seq/soc_virtual_seq.sv): Coordinated sequence orchestrating AXI write/read and UART serial transmission/reception.

4. **Environment & Configuration:**
   - [`soc_env_config.sv`](verif/tb/soc_env_config.sv): Top-level environment configuration object.
   - [`soc_uvm_env.sv`](verif/tb/soc_uvm_env.sv): Connects virtual sequencer, agents, scoreboard, and coverage.
   - [`soc_uvm_scoreboard.sv`](verif/tb/soc_uvm_scoreboard.sv): Compares AXI write transactions to `THR` against UART serial output, verifies `RBR` reads against RX injected data.
   - [`soc_uvm_coverage.sv`](verif/tb/soc_uvm_coverage.sv): Covergroups for AXI transactions and UART register access (100% coverage).

### Running UVM Simulations (Synopsys VCS & Verdi)

A unified execution script is provided at `verif/sim/run_uvm_vcs.sh`:

```bash
# 1. Run Coordinated SOC Virtual Sequence Test (Virtual Sequencer Orchestration)
./verif/sim/run_uvm_vcs.sh test_virtual_seq

# 2. Run direct AXI Master Read/Write & Interconnect Test
./verif/sim/run_uvm_vcs.sh test_axi_uart_rw

# 3. Run VeeR EL2 Core + AXI Interconnect + UART Co-Simulation Test
./verif/sim/run_uvm_vcs.sh test_core_axi_uart
```

### Viewing Waveforms in Verdi

Every simulation run automatically records an FSDB waveform:

```bash
verdi -sv -ssf verif/sim/work/dump.fsdb &
```

