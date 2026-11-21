#!/usr/bin/env bash
# =============================================================================
# run_cpu_uart_c_veer_vcs.sh -- Build C firmware (uart_test_c.hex), compile
#                               the full VeeR EL2 SoC with Synopsys VCS, and
#                               run the UART co-verification simulation.
#
# External Data Paths Tested:
#   - Signal coming OUT of SoC: uart_tx_o (captured by SV testbench serial decoder)
#   - Signal coming IN to SoC  : uart_rx_i (serial data received by SoC UART IP)
# =============================================================================

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${DIR}/.." && pwd)"
WORK="${ROOT}/run_cpu_uart_c"

mkdir -p "${WORK}"

# 1. Environment and Tool Setup
export VCS_HOME="${VCS_HOME:-/home/student/snps_tools_target/vcs/U-2023.03}"
export VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
export SNPSLMD_LICENSE_FILE="${SNPSLMD_LICENSE_FILE:-27021@14.139.1.126}"
export TOOLCHAIN_PATH="${TOOLCHAIN_PATH:-/opt/riscv/bin}"
export PATH="${VCS_HOME}/bin:${VERDI_HOME}/bin:${TOOLCHAIN_PATH}:${PATH}"

echo "====================================================================="
echo "  1. Compiling C Firmware (uart_test.c -> uart_test_c.hex)"
echo "====================================================================="
make -C "${ROOT}/my_test" uart_test_c TOOLCHAIN_PATH="${TOOLCHAIN_PATH}"
cp "${ROOT}/my_test/uart_test_c.hex" "${WORK}/uart_test.hex"
echo "Firmware hex ready at: ${WORK}/uart_test.hex"
echo ""

echo "====================================================================="
echo "  2. Compiling VeeR EL2 SoC + Testbench with Synopsys VCS"
echo "====================================================================="
cd "${ROOT}"

NOVAS_TAB="${VERDI_HOME}/share/PLI/VCS/linux64/novas.tab"
NOVAS_LIB="${VERDI_HOME}/share/PLI/VCS/linux64/pli.a"
NOVAS_FLAGS=()
if [[ -f "${NOVAS_TAB}" && -f "${NOVAS_LIB}" ]]; then
    NOVAS_FLAGS=(-P "${NOVAS_TAB}" "${NOVAS_LIB}" +define+FSDB)
fi

vcs -full64 -sverilog -timescale=1ns/1ps \
    -debug_access+all \
    +define+SIMULATION \
    "${NOVAS_FLAGS[@]}" \
    +incdir+rtl/uart/include \
    +incdir+rtl/cpu/veer_el2/config \
    +incdir+rtl/cpu/veer_el2/include \
    +incdir+rtl/cpu/veer_el2/lib \
    +incdir+rtl/cpu/veer_el2/dmi \
    -f rtl/cpu/veer_el2/flist.vcs \
    rtl/soc_top_veer.sv \
    rtl/veer_core_wrapper.sv \
    rtl/veer_iccm_ram.sv \
    rtl/axi_mem.sv \
    rtl/axi_dummy_slave.sv \
    rtl/interconnect/axi_interconnect.v \
    rtl/interconnect/arbiter.v \
    rtl/interconnect/priority_encoder.v \
    rtl/axi_uart_bridge.v \
    rtl/uart/axi_uart_top.v \
    rtl/uart/uart_controller.v \
    rtl/uart/uart_transmitter.v \
    rtl/uart/uart_receiver.v \
    rtl/uart/uart_parity_bit_compute.v \
    rtl/uart/axi_internal_fifo.v \
    rtl/gpio/axi_gpio.sv \
    rtl/timer/axi_timer.sv \
    rtl/seven_seg/axi_seven_seg.sv \
    rtl/dma/axi_dma_ctrl.sv \
    rtl/cnn/axi_cnn_conv.sv \
    rtl/relu/relu_unit.sv \
    rtl/relu/axi_relu.sv \
    rtl/pooling/pooling_unit.sv \
    rtl/pooling/axi_pooling.sv \
    tb/tb_cpu_uart_veer.sv \
    -top tb_cpu_uart_veer \
    -o "${WORK}/simv" \
    -l "${WORK}/compile.log"

echo "VCS Compilation Successful!"
echo ""

echo "====================================================================="
echo "  3. Running VeeR EL2 SoC Simulation (C Firmware Co-Verification)"
echo "====================================================================="
cd "${WORK}"
./simv -no_save | tee sim.log

echo ""
echo "====================================================================="
echo "  Simulation Completed Successfully!"
echo "  Log output: ${WORK}/sim.log"
if [[ -f "${WORK}/dump.fsdb" ]]; then
    echo "  FSDB Waveform: ${WORK}/dump.fsdb"
fi
echo "====================================================================="
