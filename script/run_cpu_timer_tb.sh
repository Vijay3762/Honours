#!/usr/bin/env bash
# =============================================================================
# run_cpu_timer_tb.sh  –  Compile and simulate tb_cpu_timer with Icarus Verilog
#
# Usage (run from Honours/ directory):
#   ./script/run_cpu_timer_tb.sh
#
# This test:
#   1. simple_cpu_master executes AXI4 transactions to the Timer IP (0x5000_0000)
#   2. Reads and verifies TIMER_ID (0x54494D52) and TIMER_VERSION (0x00010000)
#   3. Writes TIMER_LOAD = 30 and reads back
#   4. Writes TIMER_COMPARE = 15 and reads back
#   5. Starts Timer with TIMER_CTRL = 5 (enable=1, irq_en=1)
#   6. Waits for countdown past 15, checks TIMER_VALUE < 30
#   7. Checks TIMER_STATUS match bit set and timer_irq_o asserted
#   8. Clears match bit via W1C and checks TIMER_STATUS match bit cleared
#   9. Checks timer_irq_o deasserted
#  10. Asserts cpu_done and cpu_pass
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/script
PROJ="$(cd "${ROOT}/.." && pwd)"                      # Honours/
WORK="${PROJ}/work_cpu_timer"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  CPU-to-Timer Integration Test — Icarus Verilog"
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
    "${PROJ}/tb/tb_cpu_timer.sv"
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
    -o cpu_timer_sim \
    "${SRC_FILES[@]}"

echo "Compilation successful."
echo ""

echo "=== Running simulation with vvp ==="
vvp cpu_timer_sim
echo ""
echo "=== Simulation finished successfully ==="
