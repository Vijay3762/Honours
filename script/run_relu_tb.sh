#!/usr/bin/env bash
# =============================================================================
# run_relu_tb.sh -- Compile and run ReLU unit testbenches
#
# Supports:
#   1. Verilator (default if available)
#   2. Vivado XSim (xvlog / xelab / xsim)
#
# Usage:
#   ./script/run_relu_tb.sh              # auto-run with Verilator or Vivado
#   ./script/run_relu_tb.sh verilator
#   ./script/run_relu_tb.sh xsim
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM="${1:-auto}"

echo "============================================================"
echo "  ReLU Unit Verification Suite"
echo "  Project: ${ROOT}"
echo "============================================================"

if [[ "$SIM" == "auto" ]]; then
    if command -v iverilog &>/dev/null; then
        SIM="iverilog"
    elif command -v verilator &>/dev/null; then
        SIM="verilator"
    elif command -v xvlog &>/dev/null; then
        SIM="xsim"
    elif [ -f "/tools/Xilinx/Vivado/2024.2/settings64.sh" ]; then
        source /tools/Xilinx/Vivado/2024.2/settings64.sh
        SIM="xsim"
    else
        echo "ERROR: No supported simulator (iverilog, verilator, or vivado) found in PATH."
        exit 1
    fi
fi

echo "Simulator: ${SIM}"

if [[ "$SIM" == "iverilog" ]]; then
    WORK="${ROOT}/work_relu"
    mkdir -p "${WORK}"
    cd "${WORK}"

    echo ""
    echo ">>> Running 1/3: Standalone Streaming ReLU Engine (tb_relu_unit)..."
    iverilog -g2012 \
        -I "${ROOT}/rtl/relu" \
        -o sim_relu_unit \
        "${ROOT}/rtl/relu/relu_unit.sv" \
        "${ROOT}/tb/tb_relu_unit.sv"
    vvp sim_relu_unit

    echo ""
    echo ">>> Running 2/3: Memory-Mapped AXI ReLU Slave (tb_axi_relu)..."
    iverilog -g2012 \
        -I "${ROOT}/rtl/relu" \
        -o sim_axi_relu \
        "${ROOT}/rtl/relu/relu_unit.sv" \
        "${ROOT}/rtl/relu/axi_relu.sv" \
        "${ROOT}/tb/tb_axi_relu.sv"
    vvp sim_axi_relu

    echo ""
    echo ">>> Running 3/3: ReLU on AXI Interconnect (tb_relu_on_interconnect)..."
    iverilog -g2012 \
        -I "${ROOT}/rtl/interconnect" \
        -I "${ROOT}/rtl/relu" \
        -o sim_relu_ic \
        "${ROOT}/rtl/relu/relu_unit.sv" \
        "${ROOT}/rtl/relu/axi_relu.sv" \
        "${ROOT}/rtl/interconnect/axi_interconnect.v" \
        "${ROOT}/rtl/interconnect/arbiter.v" \
        "${ROOT}/rtl/interconnect/priority_encoder.v" \
        "${ROOT}/rtl/interconnect/axi_interconnect_wrap_3x8.v" \
        "${ROOT}/tb/tb_relu_on_interconnect.sv"
    vvp sim_relu_ic

    echo ""
    echo ">>> All ReLU simulations completed successfully!"

elif [[ "$SIM" == "verilator" ]]; then
    TMP_DIR=$(mktemp -d)
    echo "Temporary build dir: ${TMP_DIR}"

    echo ""
    echo ">>> Running 1/3: Standalone Streaming ReLU Engine (tb_relu_unit)..."
    verilator --binary --timing -Wno-fatal -Wno-UNSIGNED \
        -sv "${ROOT}/rtl/relu/relu_unit.sv" \
        "${ROOT}/tb/tb_relu_unit.sv" \
        -top tb_relu_unit \
        --Mdir "${TMP_DIR}/obj_relu_unit" \
        -o "${TMP_DIR}/sim_relu_unit"
    "${TMP_DIR}/sim_relu_unit"

    echo ""
    echo ">>> Running 2/2: Memory-Mapped AXI ReLU Slave (tb_axi_relu)..."
    verilator --binary --timing -Wno-fatal -Wno-UNSIGNED \
        -sv "${ROOT}/rtl/relu/relu_unit.sv" \
        "${ROOT}/rtl/relu/axi_relu.sv" \
        "${ROOT}/tb/tb_axi_relu.sv" \
        -top tb_axi_relu \
        --Mdir "${TMP_DIR}/obj_axi_relu" \
        -o "${TMP_DIR}/sim_axi_relu"
    "${TMP_DIR}/sim_axi_relu"

    rm -rf "${TMP_DIR}"
    echo ""
    echo ">>> All ReLU simulations completed successfully!"

elif [[ "$SIM" == "xsim" ]]; then
    WORK="${ROOT}/work_relu"
    mkdir -p "${WORK}"
    cd "${WORK}"

    echo ""
    echo ">>> Compiling with Vivado XSim..."
    xvlog -sv -log compile.log \
        "${ROOT}/rtl/relu/relu_unit.sv" \
        "${ROOT}/rtl/relu/axi_relu.sv" \
        "${ROOT}/tb/tb_relu_unit.sv" \
        "${ROOT}/tb/tb_axi_relu.sv"

    echo ">>> Elaborating and running tb_relu_unit..."
    xelab -debug typical -timescale 1ns/1ps -snapshot tb_relu_unit tb_relu_unit
    xsim tb_relu_unit -runall

    echo ">>> Elaborating and running tb_axi_relu..."
    xelab -debug typical -timescale 1ns/1ps -snapshot tb_axi_relu tb_axi_relu
    xsim tb_axi_relu -runall
fi
