#!/usr/bin/env bash
# =============================================================================
# run_c_test_waveform.sh -- Build C test, simulate SoC with VCS, and
#                           open the waveform in Synopsys Verdi with signals.
#
# Usage (run from Honours/ directory):
#   ./run_c_test_waveform.sh
# =============================================================================

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${DIR}/run_cpu_uart_c"

mkdir -p "${WORK}"

# 1. Environment and Tool Setup
export VCS_HOME="${VCS_HOME:-/home/student/snps_tools_target/vcs/U-2023.03}"
export VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
export SNPSLMD_LICENSE_FILE="${SNPSLMD_LICENSE_FILE:-27021@14.139.1.126}"
export TOOLCHAIN_PATH="${TOOLCHAIN_PATH:-/opt/riscv/bin}"
export PATH="${VCS_HOME}/bin:${VERDI_HOME}/bin:${TOOLCHAIN_PATH}:${PATH}"

echo "====================================================================="
echo "  1. Compiling C Firmware (my_test/uart_test.c -> uart_test_c.hex)   "
echo "====================================================================="
make -C "${DIR}/my_test" uart_test_c TOOLCHAIN_PATH="${TOOLCHAIN_PATH}"
cp "${DIR}/my_test/uart_test_c.hex" "${WORK}/uart_test.hex"
echo "Firmware hex ready: ${WORK}/uart_test.hex"
echo ""

echo "====================================================================="
echo "  2. Compiling VeeR EL2 SoC with Synopsys VCS (-kdb & Verdi PLI)     "
echo "====================================================================="
cd "${DIR}"

NOVAS_TAB="${VERDI_HOME}/share/PLI/VCS/linux64/novas.tab"
NOVAS_LIB="${VERDI_HOME}/share/PLI/VCS/linux64/pli.a"

vcs -full64 -sverilog -timescale=1ns/1ps \
    -debug_access+all \
    -kdb \
    +define+SIMULATION +define+FSDB \
    -P "${NOVAS_TAB}" "${NOVAS_LIB}" \
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
echo "  3. Running Simulation to generate dump.fsdb                        "
echo "====================================================================="
cd "${WORK}"
./simv -no_save | tee sim.log
ln -sf "${WORK}/dump.fsdb" "${DIR}/dump.fsdb"

echo ""
echo "====================================================================="
echo "  4. Opening Verdi with Waveform and Signal List                     "
echo "====================================================================="
if [[ -f "signals.rc" ]]; then
    verdi -simflow -dbdir simv.daidir -ssf dump.fsdb -sswr signals.rc &
else
    verdi -ssf dump.fsdb &
fi

echo "Verdi launched in the background!"
