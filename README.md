# RISC-V CNN Digit Recognition SoC

**RISC-V Based SoC with Hardware CNN Accelerator for Handwritten Digit Recognition**

> Honours Project — Silicon design and verification of a complete RISC-V SoC targeting the Digilent Basys3 FPGA (Xilinx Artix-7). The system classifies handwritten digits (MNIST, 28 × 28) using a hardware CNN accelerator offloaded from the processor over an AXI4 interconnect.

---

## Project Status

| Block | RTL | Standalone TB | Integration TB | Notes |
|---|---|---|---|---|
| AXI4 Interconnect (3 masters × 8 slaves) | ✅ Complete | ✅ Verified | ✅ Verified | Full address-decode, arbitration, all 8 slave ports active |
| AXI4-Lite UART IP (axi_uart_top) | ✅ Complete | ✅ Verified | ✅ Verified | TX/RX, FIFO, interrupts, parity, baud programming |
| AXI UART Bridge | ✅ Complete | — | ✅ Verified | 64-bit → 32-bit width adapter, sub-word (byte lane) aware |
| UART–Interconnect Integration | ✅ Complete | — | ✅ 19/19 pass | Full end-to-end path from master through bus to UART |
| RISC-V Processor (SweRV EL2) | ✅ Integrated | — | ✅ 4/4 pass | VeeR EL2 RV32IMC, IFU+LSU as AXI4 masters, boots from the ICCM |
| CPU → AXI → UART read/write test | ✅ Complete | — | ✅ All pass | Core configures the UART, transmits and receives a byte |
| AXI GPIO (axi_gpio) | ✅ Complete | ✅ 86/86 pass | ✅ 29/29 pass | 16-bit GPIO, per-bit direction, SET/CLR/TOGGLE, two-flop sync |
| AXI Timer (axi_timer) | ✅ Complete | ✅ 38/38 pass | ✅ 31/31 pass | 32-bit down-counter, auto-reload, compare match, W1C status, IRQ |
| CNN Convolution Accelerator (axi_cnn_conv) | ✅ Complete | ✅ 41/41 pass | ✅ 29/29 pass | 3×3 kernel, 9-multiplier parallel, 4-level adder tree, bias |
| DMA Controller (`axi_dma_ctrl`) | ✅ Complete | ✅ Verified (38/38 pass) | ✅ Verified | AXI4 slave (CPU programming) + AXI4 master (data movement), interrupt, W1C status, auto-restart |
| ReLU Unit | 📋 Planned | — | — | |
| 7-Segment Display Controller | 📋 Planned | — | — | Basys3 4-digit multiplexed display |

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
| m00 | `0x8000_0000` | 256 MB | IMEM (AXI instruction memory) |
| m01 | `0x1000_0000` | 256 MB | DMEM |
| m02 | `0x2000_0000` | 256 MB | DMA Controller registers |
| m03 | `0x3000_0000` | 256 MB | CNN Accelerator registers |
| m04 | `0x4000_0000` | 256 MB | UART (via bridge) |
| m05 | `0x5000_0000` | 256 MB | Timer |
| m06 | `0x6000_0000` | 256 MB | GPIO |
| m07 | `0x7000_0000` | 256 MB | 7-Segment Display Controller |

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
│   └── dma/
│       └── axi_dma_ctrl.sv              # ★ DMA Controller (slave+master)
├── tb/
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
│   └── run_dma_on_interconnect.sh      # ★ DMA on interconnect test (Vivado XSim)
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

## Planned Work

The SoC backbone (interconnect + UART + processor) is complete and verified.
GPIO, Timer, and CNN Convolution Accelerator are also complete and verified (standalone + integration TB).
Remaining blocks to be implemented and integrated:
1. **ReLU Unit** — single-cycle activation function, feeds feature map buffer
2. **7-Segment Display Controller** — BCD-to-segment decoder, Basys3 digit multiplexer (m07)
3. **Full SoC Integration TB** — all blocks on bus, SweRV EL2 running firmware to drive CNN inference end-to-end
4. **FPGA Implementation** — synthesis, place-and-route, timing closure on Artix-7
   (the ICCM model in `rtl/veer_iccm_ram.sv` has to be replaced by real block RAM)

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
