#!/usr/bin/env bash
# =============================================================================
# run_dma_on_interconnect.sh -- Compile and run DMA controller testbenches
#
# Supports:
#   1. iverilog (default if available)
#   2. Vivado XSim (xvlog / xelab / xsim)
#
# Usage:
#   ./script/run_dma_on_interconnect.sh
#   ./script/run_dma_on_interconnect.sh iverilog
#   ./script/run_dma_on_interconnect.sh xsim
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="${1:-auto}"

echo "============================================================"
echo "  DMA Controller Verification Suite"
echo "  Project: ${ROOT}"
echo "============================================================"

if [[ "$SIM" == "auto" ]]; then
    if command -v iverilog &>/dev/null; then
        SIM="iverilog"
    elif command -v xvlog &>/dev/null; then
        SIM="xsim"
    elif [ -f "/tools/Xilinx/Vivado/2024.2/settings64.sh" ]; then
        source /tools/Xilinx/Vivado/2024.2/settings64.sh
        SIM="xsim"
    else
        echo "ERROR: No supported simulator (iverilog or vivado) found in PATH."
        exit 1
    fi
fi

echo "Simulator: ${SIM}"

if [[ "$SIM" == "iverilog" ]]; then
    WORK="${ROOT}/work_dma"
    mkdir -p "${WORK}"
    cd "${WORK}"

    echo ""
    echo ">>> Running 1/2: Standalone DMA Controller (tb_axi_dma)..."
    iverilog -g2012 \
        -I "${ROOT}/rtl/dma" \
        -o sim_axi_dma \
        "${ROOT}/rtl/dma/axi_dma_ctrl.sv" \
        "${ROOT}/tb/tb_axi_dma.sv"
    vvp sim_axi_dma

    echo ""
    echo ">>> Running 2/2: DMA on AXI Interconnect (tb_dma_on_interconnect)..."
    iverilog -g2012 \
        -I "${ROOT}/rtl/interconnect" \
        -I "${ROOT}/rtl/dma" \
        -o sim_dma_ic \
        "${ROOT}/rtl/dma/axi_dma_ctrl.sv" \
        "${ROOT}/rtl/interconnect/axi_interconnect.v" \
        "${ROOT}/rtl/interconnect/arbiter.v" \
        "${ROOT}/rtl/interconnect/priority_encoder.v" \
        "${ROOT}/rtl/interconnect/axi_interconnect_wrap_3x8.v" \
        "${ROOT}/rtl/axi_mem.sv" \
        "${ROOT}/rtl/axi_dummy_slave.sv" \
        "${ROOT}/tb/tb_dma_on_interconnect.sv"
    vvp sim_dma_ic

    echo ""
    echo ">>> All DMA simulations completed successfully!"

elif [[ "$SIM" == "xsim" ]]; then
    WORK="${ROOT}/work_dma_on_interconnect"
    mkdir -p "${WORK}"
    cd "${WORK}"

    echo ""
    echo ">>> Compiling and running DMA on AXI Interconnect with XSim..."
    xvlog -sv -i "${ROOT}/rtl/uart/include" -log compile.log \
        "${ROOT}/rtl/dma/axi_dma_ctrl.sv" \
        "${ROOT}/rtl/interconnect/axi_interconnect.v" \
        "${ROOT}/rtl/interconnect/arbiter.v" \
        "${ROOT}/rtl/interconnect/priority_encoder.v" \
        "${ROOT}/rtl/interconnect/axi_interconnect_wrap_3x8.v" \
        "${ROOT}/rtl/axi_mem.sv" \
        "${ROOT}/rtl/axi_dummy_slave.sv" \
        "${ROOT}/tb/tb_dma_on_interconnect.sv"

    xelab -debug typical -timescale 1ns/1ps -snapshot tb_dma_on_interconnect -log elaborate.log tb_dma_on_interconnect
    xsim tb_dma_on_interconnect -runall -log sim.log -wdb dma_on_interconnect.wdb 2>&1 | grep -E "\[FAIL\]|\[PASS\]|T[0-9] done|passed|failed|ALL TESTS|Watchdog"
fi
