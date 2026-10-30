#!/usr/bin/env bash
# =============================================================================
# run_cpu_uart_tb.sh  –  Compile and simulate tb_cpu_uart with Vivado XSim
#
# Usage (run from Honours/ directory):
#   source /tools/Xilinx/Vivado/2024.2/settings64.sh
#   ./run_cpu_uart_tb.sh
#
# This test:
#   1. simple_cpu_master configures UART (LCR, BAUD) via AXI interconnect + bridge
#   2. Writes 'A' (0x41) then 'B' (0x42) to UART_THR
#   3. UART tx is looped back to rx internally
#   4. Testbench captures transmitted bytes and verifies 'A' then 'B'
// =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/script
PROJ="$(cd "${ROOT}/.." && pwd)"                      # Honours/
WORK="${PROJ}/work_cpu_uart"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  CPU-to-UART Test — Vivado XSim"
echo "============================================================"
echo "Project root: ${PROJ}"
echo "Work dir:     ${WORK}"
echo ""

# ---------------------------------------------------------------------------
# Source file lists
# ---------------------------------------------------------------------------

# SoC + simple CPU master + interconnect + UART
SOC_FILES=(
    "${PROJ}/rtl/soc_top.sv"
    "${PROJ}/rtl/simple_cpu_master.sv"
    "${PROJ}/rtl/axi_mem.sv"
    "${PROJ}/rtl/axi_dummy_slave.sv"
    "${PROJ}/rtl/interconnect/axi_interconnect.v"
    "${PROJ}/rtl/interconnect/arbiter.v"
    "${PROJ}/rtl/interconnect/priority_encoder.v"
    "${PROJ}/rtl/axi_uart_bridge.v"
    "${PROJ}/rtl/uart/axi_uart_top.v"
    "${PROJ}/rtl/uart/uart_controller.v"
    "${PROJ}/rtl/uart/uart_transmitter.v"
    "${PROJ}/rtl/uart/uart_receiver.v"
    "${PROJ}/rtl/uart/uart_parity_bit_compute.v"
    "${PROJ}/rtl/uart/axi_internal_fifo.v"
)

# Testbench
TB_FILES=(
    "${PROJ}/tb/tb_cpu_uart.sv"
)

ALL_FILES=("${SOC_FILES[@]}" "${TB_FILES[@]}")

# ---------------------------------------------------------------------------
# Validate source files
# ---------------------------------------------------------------------------
echo "=== Checking source files ==="
MISSING=0
for f in "${ALL_FILES[@]}"; do
    if [[ ! -f "$f" ]]; then
        echo "  MISSING: $f"
        MISSING=1
    fi
done

# Check include files
INC_FILES=(
    "${PROJ}/rtl/uart/include/axi_uart_defines.vh"
    "${PROJ}/rtl/uart/include/axi_uart.vh"
)
for f in "${INC_FILES[@]}"; do
    if [[ ! -f "$f" ]]; then
        echo "  MISSING include: $f"
        MISSING=1
    fi
done

[[ $MISSING -eq 0 ]] || { echo "Aborting."; exit 1; }
echo "All source files found."
echo ""

# ---------------------------------------------------------------------------
# Check hex file
# ---------------------------------------------------------------------------
HEX="${PROJ}/my_test/uart_test.hex"
if [[ ! -f "${HEX}" ]]; then
    echo "ERROR: Firmware hex not found: ${HEX}"
    exit 1
fi
echo "Firmware: ${HEX}"
echo ""

# ---------------------------------------------------------------------------
# Compile with xvlog
# ---------------------------------------------------------------------------
echo "=== Compiling with xvlog ==="
xvlog -sv -i "${PROJ}/rtl/uart/include" \
    -log compile.log \
    "${ALL_FILES[@]}"

if [[ $? -ne 0 ]]; then
    echo "ERROR: xvlog compilation failed. See compile.log"
    exit 1
fi
echo "Compilation OK."
echo ""

# ---------------------------------------------------------------------------
# Elaborate with xelab
# ---------------------------------------------------------------------------
echo "=== Elaborating with xelab ==="
xelab -debug typical \
    -timescale 1ns/1ps \
    -snapshot tb_cpu_uart \
    -log elaborate.log \
    tb_cpu_uart

if [[ $? -ne 0 ]]; then
    echo "ERROR: xelab elaboration failed. See elaborate.log"
    exit 1
fi
echo "Elaboration OK."
echo ""

# ---------------------------------------------------------------------------
# Simulate with xsim
# ---------------------------------------------------------------------------
echo "=== Running simulation with xsim ==="
xsim tb_cpu_uart \
    -runall \
    -log sim.log \
    -wdb cpu_uart_test.wdb

if [[ $? -ne 0 ]]; then
    echo "ERROR: xsim simulation failed. See sim.log"
    exit 1
fi
echo ""

echo "============================================================"
echo "  Simulation complete. View waveforms:"
echo "    open cpu_uart_test.wdb in Vivado"
echo "============================================================"
