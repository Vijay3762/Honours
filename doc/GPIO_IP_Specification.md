# GPIO IP Specification

**Project:** RISC-V CNN SoC — Honours Project  
**Block:** AXI GPIO Controller  
**Target:** Digilent Basys3 (Xilinx Artix-7)  
**Toolchain:** Synopsys VCS U-2023.03, Vivado 2024.2  
**Language:** SystemVerilog (synthesizable)  
**Version:** 1.0  

---

## 1. Overview

The AXI GPIO IP is a parameterized General-Purpose Input/Output controller that connects as an AXI4 slave on the SoC interconnect. It provides per-bit direction control, set/clear/toggle action registers for atomic bit manipulation, and two-flop synchronizers for asynchronous GPIO inputs.

The IP is designed to connect to master port **m06** of the `axi_interconnect_wrap_3x8` interconnect at base address `0x6000_0000`.

---

## 2. AXI Interface

### 2.1 Interface Widths

The GPIO IP's AXI4 slave interface matches the existing interconnect exactly:

| Parameter | Value | Source |
|---|---|---|
| `AXI_DATA_WIDTH` | 64 | `axi_interconnect_wrap_3x8` DATA_WIDTH used in `tb_uart_on_interconnect.sv` |
| `AXI_ADDR_WIDTH` | 32 | `axi_interconnect_wrap_3x8` ADDR_WIDTH |
| `AXI_ID_WIDTH` | 8 | `axi_interconnect_wrap_3x8` ID_WIDTH |
| `AXI_STRB_WIDTH` | 8 | DATA_WIDTH/8 |
| User signals | Disabled | AWUSER_ENABLE=0, WUSER_ENABLE=0, etc. (all user signals absent) |
| FORWARD_ID | 0 | Interconnect does not forward IDs |

### 2.2 Signal List

The slave port exposes the full AXI4 signal set (same as the `m06_axi_*` port on the interconnect):

**Write Address Channel:**
| Signal | Direction | Width | Description |
|---|---|---|---|
| `s_axi_awid` | in | 8 | Write address ID |
| `s_axi_awaddr` | in | 32 | Write address |
| `s_axi_awlen` | in | 8 | Burst length (beats-1) |
| `s_axi_awsize` | in | 3 | Burst size |
| `s_axi_awburst` | in | 2 | Burst type |
| `s_axi_awlock` | in | 1 | Lock |
| `s_axi_awcache` | in | 4 | Cache |
| `s_axi_awprot` | in | 3 | Protection |
| `s_axi_awqos` | in | 4 | Quality of service |
| `s_axi_awregion` | in | 4 | Region |
| `s_axi_awvalid` | in | 1 | Valid |
| `s_axi_awready` | out | 1 | Ready |

**Write Data Channel:**
| Signal | Direction | Width | Description |
|---|---|---|---|
| `s_axi_wdata` | in | 64 | Write data (lower 32 bits used for registers) |
| `s_axi_wstrb` | in | 8 | Write strobe (lower 4 bits used for byte enables) |
| `s_axi_wlast` | in | 1 | Last beat |
| `s_axi_wvalid` | in | 1 | Valid |
| `s_axi_wready` | out | 1 | Ready |

**Write Response Channel:**
| Signal | Direction | Width | Description |
|---|---|---|---|
| `s_axi_bid` | out | 8 | Response ID (echoes `s_axi_awid`) |
| `s_axi_bresp` | out | 2 | Response code |
| `s_axi_bvalid` | out | 1 | Valid |
| `s_axi_bready` | in | 1 | Ready |

**Read Address Channel:**
| Signal | Direction | Width | Description |
|---|---|---|---|
| `s_axi_arid` | in | 8 | Read address ID |
| `s_axi_araddr` | in | 32 | Read address |
| `s_axi_arlen` | in | 8 | Burst length (beats-1) |
| `s_axi_arsize` | in | 3 | Burst size |
| `s_axi_arburst` | in | 2 | Burst type |
| `s_axi_arlock` | in | 1 | Lock |
| `s_axi_arcache` | in | 4 | Cache |
| `s_axi_arprot` | in | 3 | Protection |
| `s_axi_arqos` | in | 4 | Quality of service |
| `s_axi_arregion` | in | 4 | Region |
| `s_axi_arvalid` | in | 1 | Valid |
| `s_axi_arready` | out | 1 | Ready |

**Read Data Channel:**
| Signal | Direction | Width | Description |
|---|---|---|---|
| `s_axi_rid` | out | 8 | Response ID (echoes `s_axi_arid`) |
| `s_axi_rdata` | out | 64 | Read data (lower 32 bits contain register data, upper 32 bits read as zero) |
| `s_axi_rresp` | out | 2 | Response code |
| `s_axi_rlast` | out | 1 | Last beat |
| `s_axi_rvalid` | out | 1 | Valid |
| `s_axi_rready` | in | 1 | Ready |

### 2.3 Data Bus Usage

GPIO registers are 32-bit. The 64-bit AXI data bus is used as follows:

- **Writes:** Only `s_axi_wdata[31:0]` is used. `s_axi_wdata[63:32]` is ignored.
- **Write strobe:** Only `s_axi_wstrb[3:0]` is used. `s_axi_wstrb[7:4]` is ignored.
- **Reads:** Register data is returned in `s_axi_rdata[31:0]`. `s_axi_rdata[63:32]` is always zero.

### 2.4 Reset Convention

Reset is **active-high** (`rst`), matching the interconnect's `rst` signal. This is consistent with the existing `axi_interconnect_wrap_3x8` and `axi_uart_bridge` modules.

---

## 3. Parameters

| Parameter | Default | Description |
|---|---|---|
| `GPIO_WIDTH` | 16 | Number of GPIO pins |
| `AXI_DATA_WIDTH` | 64 | AXI data bus width (matches interconnect) |
| `AXI_ADDR_WIDTH` | 32 | AXI address bus width |
| `AXI_ID_WIDTH` | 8 | AXI ID width |
| `AXI_STRB_WIDTH` | 8 | AXI write strobe width (AXI_DATA_WIDTH/8) |
| `REG_ADDR_WIDTH` | 12 | Register aperture address bits (4 KiB) |
| `GPIO_ID_VALUE` | `0x4749504F` | Fixed peripheral ID ("GPIO" in ASCII: G=0x47, I=0x49, P=0x50, O=0x4F) |
| `GPIO_VERSION_VALUE` | `0x00010000` | Version v1.0 (major=1 in upper 16 bits, minor=0 in lower 16) |

---

## 4. Register Map

All registers are 32-bit and word-aligned. The register aperture is 4 KiB (offsets 0x000–0xFFF from the slave base address). Only offsets 0x00–0x1C are valid registers; all other offsets return SLVERR.

GPIO data occupies the least-significant `GPIO_WIDTH` bits of each register. Reserved bits read as zero.

| Offset | Register | Access | Reset | Description |
|---|---|---|---|---|
| `0x00` | `GPIO_IN` | RO | 0x0 | Synchronized input values (two-flop synchronizer) |
| `0x04` | `GPIO_OUT` | RW | 0x0 | Output data register |
| `0x08` | `GPIO_DIR` | RW | 0x0 | Direction: 1=output, 0=input |
| `0x0C` | `GPIO_OUT_SET` | WO | — | Set selected output bits (mask, action) |
| `0x10` | `GPIO_OUT_CLR` | WO | — | Clear selected output bits (mask, action) |
| `0x14` | `GPIO_OUT_TOGGLE` | WO | — | Toggle selected output bits (mask, action) |
| `0x18` | `GPIO_ID` | RO | `0x4749504F` | Fixed peripheral ID |
| `0x1C` | `GPIO_VERSION` | RO | `0x00010000` | Fixed version |

### 4.1 Register Details

**GPIO_IN (0x00, RO):** Returns the synchronized GPIO input values. A two-flop synchronizer is used: after changing `gpio_i`, the new value appears in `GPIO_IN` after 2 clock cycles. Writing to this register returns SLVERR.

**GPIO_OUT (0x04, RW):** Controls the output data. When a GPIO bit is configured as output (via GPIO_DIR), the corresponding `gpio_o` pin is driven with the value from this register. Bits above `GPIO_WIDTH` are masked and read as zero. Byte strobes are supported: only bytes with `wstrb[n]=1` are written.

**GPIO_DIR (0x08, RW):** Controls per-bit direction. `1` = output, `0` = input. When a bit is configured as input, `gpio_oen[n]` is 0 and `gpio_o[n]` is 0. Bits above `GPIO_WIDTH` are masked.

**GPIO_OUT_SET (0x0C, WO):** Action register. Writing a 1 to bit n sets `GPIO_OUT[n]` to 1. The write data is a mask: only bits with a 1 are affected. Bits above `GPIO_WIDTH` have no effect. Reads return 0. Byte strobes apply: only bytes with `wstrb[n]=1` contribute to the mask.

**GPIO_OUT_CLR (0x10, WO):** Action register. Writing a 1 to bit n clears `GPIO_OUT[n]` to 0. Same masking and strobe rules as SET.

**GPIO_OUT_TOGGLE (0x14, WO):** Action register. Writing a 1 to bit n toggles `GPIO_OUT[n]`. Same masking and strobe rules as SET.

**GPIO_ID (0x18, RO):** Returns the fixed peripheral ID `0x4749504F`. Writing returns SLVERR.

**GPIO_VERSION (0x1C, RO):** Returns the fixed version `0x00010000` (v1.0). Writing returns SLVERR.

---

## 5. Physical GPIO Interface

| Signal | Direction | Width | Description |
|---|---|---|---|
| `gpio_i` | in | GPIO_WIDTH | Asynchronous GPIO inputs (synchronized internally) |
| `gpio_o` | out | GPIO_WIDTH | Output data (driven when direction=output) |
| `gpio_oen` | out | GPIO_WIDTH | Output enable (1=output, 0=input/high-Z) |

The output mux is: `gpio_o = gpio_dir_reg & gpio_out_reg`, `gpio_oen = gpio_dir_reg`.

---

## 6. Reset Behavior

On reset (`rst=1`):

| Register/State | Reset Value |
|---|---|
| `gpio_out_reg` | 0 (all outputs 0) |
| `gpio_dir_reg` | 0 (all inputs) |
| `gpio_sync1`, `gpio_sync2` | 0 (synchronizers cleared) |
| Write FSM state | WR_IDLE |
| Read FSM state | RD_IDLE |
| `b_valid_r` | 0 (no stale B response) |
| `r_valid_r` | 0 (no stale R response) |
| All latched AXI transaction info | 0 |

No stale response remains asserted after reset.

---

## 7. AXI Protocol Implementation

### 7.1 Independent AW/W Handshakes

The write channel implements independent AW and W acceptance:
- **AW arrives first:** AW is latched, FSM enters `WR_WAIT_W`, waits for W.
- **W arrives first:** W is latched, FSM enters `WR_WAIT_AW`, waits for AW.
- **AW and W simultaneous:** Both accepted in the same cycle.

In all cases, the B response is generated only after both AW and W have been accepted.

### 7.2 BVALID/RVALID Stability

- `s_axi_bvalid` and its payload (`bid`, `bresp`) are held stable until `s_axi_bready` is asserted.
- `s_axi_rvalid` and its payload (`rid`, `rdata`, `rresp`, `rlast`) are held stable until `s_axi_rready` is asserted.

### 7.3 Response Codes

| Code | Value | Meaning |
|---|---|---|
| OKAY | `2'b00` | Valid register access |
| SLVERR | `2'b10` | Reserved offset, write to RO register, misaligned access, or burst |
| DECERR | `2'b11` | Address outside the 4 KiB register aperture (bits [31:12] ≠ 0) |

### 7.4 Address Validation

The full 32-bit address is validated:
- **In-aperture check:** Address bits [31:12] must be zero (address within 4 KiB of base). If not, DECERR is returned.
- **Alignment check:** Address bits [1:0] must be zero (word-aligned). If not, SLVERR is returned.
- **Register validity:** Word offset [5:2] must be 0–7 (valid registers 0x00–0x1C). Offsets 0x20–0xFFC within the aperture are reserved and return SLVERR.

This prevents aliasing: the register bank cannot appear at multiple addresses within the 256 MB m06 region.

### 7.5 Error Policies

| Condition | Write Response | Read Response |
|---|---|---|
| Valid register, aligned, single-beat | OKAY | OKAY |
| Reserved offset (0x20–0xFFC) | SLVERR | SLVERR (data=0) |
| Write to RO register (0x00, 0x18, 0x1C) | SLVERR | — |
| Misaligned address | SLVERR | SLVERR (data=0) |
| Out-of-aperture (bits [31:12] ≠ 0) | DECERR | DECERR (data=0) |
| Burst (len > 0) | SLVERR (after draining) | SLVERR (all beats) |

### 7.6 Burst Handling

**Single-beat transfers (len=0):** Fully supported for both reads and writes.

**Burst writes (len>0):** The IP accepts all W data beats (WREADY stays high) until WLAST is received, then returns a single SLVERR on the B channel. No register modification occurs.

**Burst reads (len>0):** The IP returns `arlen+1` R beats, each with `rresp=SLVERR` and `rdata=0`. `RLAST` is asserted on the final beat.

All accepted transactions eventually complete — there is no deadlock path.

### 7.7 Byte Strobe Semantics

Byte strobes (`s_axi_wstrb[3:0]`) control which bytes of a 32-bit register word are written:
- `wstrb[0]=1`: byte 0 (bits [7:0]) written
- `wstrb[1]=1`: byte 1 (bits [15:8]) written
- `wstrb[2]=1`: byte 2 (bits [23:16]) written
- `wstrb[3]=1`: byte 3 (bits [31:24]) written

For SET/CLR/TOGGLE action registers, the strobe-masked write data is used as the bit mask. For example, `SET` with `wdata=0xFF, wstrb=0x1` only sets bits [7:0] if the corresponding GPIO_MASK bits are 1.

Bytes with `wstrb[n]=0` are ignored — the existing register content is preserved.

---

## 8. Synchronizer Behavior

GPIO inputs (`gpio_i`) are asynchronous to the AXI clock. A two-flop synchronizer is used per bit:

```
always @(posedge clk) begin
    gpio_sync1 <= gpio_i;
    gpio_sync2 <= gpio_sync1;
end
```

- After changing `gpio_i`, the new value appears in `GPIO_IN` after 2 clock edges.
- Both synchronizer stages are cleared to 0 on reset.
- This is a standard metastability mitigator, not a debouncer.

---

## 9. Limitations

1. **No interrupts:** This version does not generate interrupts on input changes.
2. **No debounce:** Input synchronizers only handle metastability, not switch bounce.
3. **No output drive strength control:** All outputs use the same drive capability.
4. **Register width:** Registers are 32-bit. GPIO widths up to 32 are supported. For GPIO_WIDTH=32, the full 32-bit register is used.
5. **Burst support:** Only single-beat transfers are fully supported. Bursts are safely rejected with SLVERR but no data is transferred.
6. **No AXI user signals:** All user signals are disabled (matching the interconnect configuration).

---

## 10. Files

| File | Description |
|---|---|
| `rtl/gpio/axi_gpio.sv` | GPIO IP RTL (SystemVerilog) |
| `tb/tb_axi_gpio.sv` | Standalone self-checking testbench |
| `doc/GPIO_IP_Specification.md` | This document |

---

## 11. Testbench Description

The testbench (`tb/tb_axi_gpio.sv`) is a standalone self-checking testbench with 15 test cases:

| Test | Description |
|---|---|
| T1 | Reset defaults: OUT=0, DIR=0, IN=0, gpio_o=0, gpio_oen=0 |
| T2 | Input synchronisation: two-flop delay verification |
| T3 | Output register write/readback and GPIO_WIDTH masking |
| T4 | Direction register write/readback, physical oen check |
| T5 | SET, CLR, TOGGLE action registers and write-only read-as-0 |
| T6 | Partial writes with WSTRB (byte-level, half-word) |
| T7 | GPIO_ID and GPIO_VERSION values |
| T8 | Read-only register writes → SLVERR |
| T9 | Invalid, reserved, misaligned addresses → SLVERR/DECERR |
| T10 | Independent AW/W timing (W-first and AW-first, with gaps) |
| T11 | BREADY backpressure (BVALID held stable) |
| T12 | RREADY backpressure (RVALID held stable) |
| T13 | Burst handling: burst write drain + SLVERR, burst read RLAST |
| T14 | Reset recovery: state cleared, bus functional after reset |
| T15 | GPIO-width masking comprehensive (all-ones, SET/CLR/TOGGLE) |

The testbench uses bounded AXI read/write tasks with cycle-count timeouts, a global watchdog timer, and prints a PASS/FAIL summary. On failure, it terminates with `$fatal`.

---

## 12. Verification — Vivado 2024.2 Simulation Steps

### 12.1 Using Vivado Simulator (xsim)

1. **Create a project** (if not already created):
   ```tcl
   create_project gpio_sim ./gpio_sim -part xc7a35tcpg236-1 -force
   ```

2. **Add the RTL and testbench:**
   ```tcl
   add_files -norecurse {rtl/gpio/axi_gpio.sv}
   add_files -norecurse -fileset sim_1 {tb/tb_axi_gpio.sv}
   ```

3. **Set the testbench as the top simulation module:**
   ```tcl
   set_property top tb_axi_gpio [get_filesets sim_1]
   set_property top_lib xil_defaultlib [get_filesets sim_1]
   ```

4. **Launch simulation:**
   ```tcl
   launch_simulation
   ```

5. **Run the simulation:**
   ```tcl
   run all
   ```

6. **Check the console for the test summary:**
   ```
   ==============================================================
     Test Summary
   ==============================================================
     Total checks : <N>
     Passed       : <N>
     Failed       : 0
   ==============================================================
   *** ALL TESTS PASSED ***
   ```

### 12.2 Using VCS (command line)

```bash
cd Honours/
vcs -full64 -sverilog -timescale=1ns/1ps \
    -top tb_axi_gpio \
    rtl/gpio/axi_gpio.sv \
    tb/tb_axi_gpio.sv \
    -o simv_gpio_tb
./simv_gpio_tb
```

### 12.3 Using Icarus Verilog (command line)

```bash
cd Honours/
iverilog -g2012 \
    -s tb_axi_gpio \
    -o tb_axi_gpio.vvp \
    rtl/gpio/axi_gpio.sv \
    tb/tb_axi_gpio.sv
vvp tb_axi_gpio.vvp
```

---

## 13. SoC Integration (Future)

The GPIO IP is not yet connected to the SoC. When integrating:

1. Instantiate `axi_gpio` in the SoC top-level.
2. Connect the AXI slave port to interconnect master port `m06`:
   ```verilog
   axi_gpio #(
       .GPIO_WIDTH(16)
   ) gpio_inst (
       .clk(clk),
       .rst(rst),
       // Connect to m06_axi_* signals from interconnect
       .s_axi_awid    (m06_axi_awid),
       .s_axi_awaddr  (m06_axi_awaddr),
       .s_axi_awlen   (m06_axi_awlen),
       // ... all other AXI signals ...
       // Physical GPIO pins
       .gpio_i    (gpio_pads_i),
       .gpio_o    (gpio_pads_o),
       .gpio_oen  (gpio_pads_oen)
   );
   ```
3. The interconnect already routes addresses `0x6000_0000`–`0x6FFF_FFFF` to m06 with ADDR_WIDTH=28 (256 MB aperture).
4. The GPIO IP validates the full address, so only offsets 0x000–0xFFF within the aperture are accepted. All other addresses return DECERR.

---

## 14. Interface Widths Discovered (from source code inspection)

The following widths were confirmed by reading the actual source files:

- **`rtl/interconnect/axi_interconnect.v`:** Parameters `DATA_WIDTH=32` (default), `ADDR_WIDTH=32`, `ID_WIDTH=8`, all user signals disabled, `FORWARD_ID=0`.
- **`rtl/interconnect/axi_interconnect_wrap_3x8.v`:** Wrapper passes through all parameters. Master ports `m00`–`m07` with full AXI4 signal set including `awregion`, `awlen`, `awburst`, `awsize`, `awlock`, `awcache`, `awprot`, `awqos`.
- **`tb/tb_uart_on_interconnect.sv`:** Actual instantiation uses `DATA_WIDTH=64`, `ADDR_WIDTH=32`, `ID_WIDTH=8`, `STRB_WIDTH=8`. M06_BASE_ADDR=`0x6000_0000`, M06_ADDR_WIDTH=28.
- **`rtl/axi_uart_bridge.v`:** Slave port pattern with 64-bit data (lower 32 used), 32-bit address, 8-bit ID, 8-bit strobe, active-high reset input.

---

## 15. Constants Selected

| Constant | Value | Rationale |
|---|---|---|
| `GPIO_ID_VALUE` | `0x4749504F` | ASCII "GPIO" (G=0x47, I=0x49, P=0x50, O=0x4F) — readable in hex dump |
| `GPIO_VERSION_VALUE` | `0x00010000` | v1.0 — major version in upper 16 bits, minor in lower 16 |
