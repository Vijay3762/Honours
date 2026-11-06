#!/usr/bin/env bash
# =============================================================================
# run_cpu_gpio_tb.sh  –  Compile and simulate tb_cpu_gpio with Icarus Verilog
#
# Usage (run from Honours/ directory):
#   ./script/run_cpu_gpio_tb.sh
#
# This test:
#   1. simple_cpu_master executes AXI4 transactions to the GPIO IP (0x6000_0000)
#   2. Configures GPIO_DIR (0x00FF -> pins 7:0 output, 15:8 input)
#   3. Writes and reads back GPIO_OUT (0x00A5)
#   4. Performs atomic set (0x0050 -> 0x00F5)
#   5. Performs atomic clear (0x0005 -> 0x00F0)
#   6. Performs atomic toggle (0x0033 -> 0x00C3)
#   7. Reads back GPIO_IN with external stimulus applied to gpio_pads_i
#   8. Reads and verifies GPIO_ID (0x4749504F) and GPIO_VERSION (0x00010000)
#   9. Verifies gpio_pads_o and gpio_pads_oen track the internal register changes
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/script
PROJ="$(cd "${ROOT}/.." && pwd)"                      # Honours/
WORK="${PROJ}/work_cpu_gpio"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  CPU-to-GPIO Integration Test — Icarus Verilog"
echo "============================================================"
echo "Project root: ${PROJ}"
echo "Work dir:     ${WORK}"
echo ""

# ---------------------------------------------------------------------------
# Source file lists
# ---------------------------------------------------------------------------
SRC_FILES=(
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
    "${PROJ}/rtl/gpio/axi_gpio.sv"
    "${PROJ}/rtl/timer/axi_timer.sv"
    "${PROJ}/rtl/cnn/axi_cnn_conv.sv"
    "${PROJ}/rtl/seven_seg/axi_seven_seg.sv"
    "${PROJ}/tb/tb_cpu_gpio.sv"
)

# ---------------------------------------------------------------------------
# Check files
# ---------------------------------------------------------------------------
echo "=== Checking source files ==="
MISSING=0
for f in "${SRC_FILES[@]}"; do
    if [[ ! -f "$f" ]]; then
        echo "  MISSING: $f"
        MISSING=1
    fi
done

[[ $MISSING -eq 0 ]] || { echo "Aborting due to missing files."; exit 1; }
echo "All source files found."
echo ""

# ---------------------------------------------------------------------------
# Compile and run with iverilog
# ---------------------------------------------------------------------------
echo "=== Compiling with iverilog ==="
iverilog -g2012 \
    -I "${PROJ}/rtl" \
    -I "${PROJ}/rtl/interconnect" \
    -I "${PROJ}/rtl/uart/include" \
    -o cpu_gpio_sim \
    "${SRC_FILES[@]}"

echo "Compilation successful."
echo ""

echo "=== Running simulation with vvp ==="
vvp cpu_gpio_sim
echo ""
echo "=== Simulation finished successfully ==="
