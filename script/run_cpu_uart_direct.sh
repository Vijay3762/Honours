#!/usr/bin/env bash
# Direct CPU-to-UART test (bypass interconnect)
set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "${ROOT}/.." && pwd)"
WORK="${PROJ}/work_cpu_uart_direct"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  Direct CPU-to-UART Test — Vivado XSim"
echo "============================================================"
echo "Work dir: ${WORK}"
echo ""

SOC_FILES=(
    "${PROJ}/rtl/soc_top_direct.sv"
    "${PROJ}/rtl/simple_cpu_master.sv"
    "${PROJ}/rtl/axi_uart_bridge.v"
    "${PROJ}/rtl/uart/axi_uart_top.v"
    "${PROJ}/rtl/uart/uart_controller.v"
    "${PROJ}/rtl/uart/uart_transmitter.v"
    "${PROJ}/rtl/uart/uart_receiver.v"
    "${PROJ}/rtl/uart/uart_parity_bit_compute.v"
    "${PROJ}/rtl/uart/axi_internal_fifo.v"
)

TB_FILES=(
    "${PROJ}/tb/tb_cpu_uart_direct.sv"
)

ALL_FILES=("${SOC_FILES[@]}" "${TB_FILES[@]}")

echo "=== Checking source files ==="
MISSING=0
for f in "${ALL_FILES[@]}"; do
    if [[ ! -f "$f" ]]; then
        echo "  MISSING: $f"
        MISSING=1
    fi
done
[[ $MISSING -eq 0 ]] || { echo "Aborting."; exit 1; }
echo "All source files found."
echo ""

echo "=== Compiling with xvlog ==="
xvlog -sv -i "${PROJ}/rtl/uart/include" \
    -log compile.log \
    "${ALL_FILES[@]}"

if [[ $? -ne 0 ]]; then
    echo "ERROR: xvlog compilation failed."
    exit 1
fi
echo "Compilation OK."
echo ""

echo "=== Elaborating with xelab ==="
xelab -debug typical \
    -timescale 1ns/1ps \
    -snapshot tb_cpu_uart_direct \
    -log elaborate.log \
    tb_cpu_uart_direct

if [[ $? -ne 0 ]]; then
    echo "ERROR: xelab elaboration failed."
    exit 1
fi
echo "Elaboration OK."
echo ""

echo "=== Running simulation with xsim ==="
xsim tb_cpu_uart_direct \
    -runall \
    -log sim.log \
    -wdb cpu_uart_direct.wdb

if [[ $? -ne 0 ]]; then
    echo "ERROR: xsim simulation failed."
    exit 1
fi
echo ""
echo "============================================================"
echo "  Simulation complete."
echo "============================================================"
