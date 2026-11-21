#!/usr/bin/env bash
# =============================================================================
# run_soc_uvm.sh - Compile & Simulate VeeR EL2 SoC + C Test in UVM 1.2
# =============================================================================

set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${DIR}/.." && pwd)"

cd "${DIR}"

unset VCS_ARCH_OVERRIDE

# Environment Setup
export VCS_HOME="${VCS_HOME:-/home/student/snps_tools_target/vcs/U-2023.03}"
export VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
export SNPSLMD_LICENSE_FILE="${SNPSLMD_LICENSE_FILE:-27021@14.139.1.126}"
export TOOLCHAIN_PATH="${TOOLCHAIN_PATH:-/opt/riscv/bin}"
export PATH="${VCS_HOME}/bin:${VERDI_HOME}/bin:${TOOLCHAIN_PATH}:${PATH}"

NOVAS_TAB="${VERDI_HOME}/share/PLI/VCS/linux64/novas.tab"
NOVAS_LIB="${VERDI_HOME}/share/PLI/VCS/linux64/pli.a"

VCS_BIN="${VCS_HOME}/bin/vcs"
[[ -x "$VCS_BIN" ]] || VCS_BIN="vcs"

echo "====================================================================="
echo "  1. Ensuring C Test .hex file is generated                          "
echo "====================================================================="
make -C "${ROOT}/my_test" uart_test_c TOOLCHAIN_PATH="${TOOLCHAIN_PATH}"
cp "${ROOT}/my_test/uart_test_c.hex" "${DIR}/uart_test_c.hex"
echo "Firmware hex ready: ${DIR}/uart_test_c.hex"
echo ""

echo "====================================================================="
echo "  2. Compiling VeeR EL2 SoC + UVM Testbench with Synopsys VCS       "
echo "====================================================================="
cd "${ROOT}"

"${VCS_BIN}" \
    -full64 \
    -sverilog \
    -ntb_opts uvm-1.2 \
    -timescale=1ns/1ps \
    -debug_access+all \
    -kdb \
    +define+SIMULATION \
    +define+FSDB \
    -P "${NOVAS_TAB}" "${NOVAS_LIB}" \
    +incdir+"${ROOT}/rtl/uart" \
    +incdir+"${ROOT}/rtl/uart/include" \
    +incdir+"${ROOT}/rtl/cpu/veer_el2/config" \
    +incdir+"${ROOT}/rtl/cpu/veer_el2/include" \
    +incdir+"${ROOT}/rtl/cpu/veer_el2/lib" \
    +incdir+"${ROOT}/rtl/cpu/veer_el2/dmi" \
    +incdir+"${DIR}" \
    -f "${ROOT}/rtl/cpu/veer_el2/flist.vcs" \
    "${ROOT}/rtl/soc_top_veer.sv" \
    "${ROOT}/rtl/veer_core_wrapper.sv" \
    "${ROOT}/rtl/veer_iccm_ram.sv" \
    "${ROOT}/rtl/axi_mem.sv" \
    "${ROOT}/rtl/axi_dummy_slave.sv" \
    "${ROOT}/rtl/interconnect/axi_interconnect.v" \
    "${ROOT}/rtl/interconnect/arbiter.v" \
    "${ROOT}/rtl/interconnect/priority_encoder.v" \
    "${ROOT}/rtl/axi_uart_bridge.v" \
    "${ROOT}/rtl/uart/axi_uart_top.v" \
    "${ROOT}/rtl/uart/uart_controller.v" \
    "${ROOT}/rtl/uart/uart_transmitter.v" \
    "${ROOT}/rtl/uart/uart_receiver.v" \
    "${ROOT}/rtl/uart/uart_parity_bit_compute.v" \
    "${ROOT}/rtl/uart/axi_internal_fifo.v" \
    "${ROOT}/rtl/gpio/axi_gpio.sv" \
    "${ROOT}/rtl/timer/axi_timer.sv" \
    "${ROOT}/rtl/seven_seg/axi_seven_seg.sv" \
    "${ROOT}/rtl/dma/axi_dma_ctrl.sv" \
    "${ROOT}/rtl/cnn/axi_cnn_conv.sv" \
    "${ROOT}/rtl/relu/relu_unit.sv" \
    "${ROOT}/rtl/relu/axi_relu.sv" \
    "${ROOT}/rtl/pooling/pooling_unit.sv" \
    "${ROOT}/rtl/pooling/axi_pooling.sv" \
    "${DIR}/uart_if.sv" \
    "${DIR}/uart_pkg.sv" \
    "${DIR}/soc_tb_top.sv" \
    -top soc_tb_top \
    -o "${DIR}/simv_soc_uvm" \
    -l "${DIR}/compile_soc_uvm.log"

echo "VCS Compilation Successful!"
echo ""

echo "====================================================================="
echo "  3. Running SoC UVM Simulation (Executing C test from ICCM)        "
echo "====================================================================="
cd "${DIR}"
cp "${ROOT}/my_test/uart_test_c.hex" "${DIR}/uart_test_c.hex"
cp "${ROOT}/my_test/uart_test_c.hex" "${ROOT}/uart_test_c.hex"

./simv_soc_uvm \
    -no_save \
    +UVM_TESTNAME=soc_uart_test \
    +UVM_VERBOSITY=UVM_LOW \
    -l "${DIR}/sim_soc_uvm.log"

echo ""
echo "====================================================================="
echo "  4. UVM Verification Summary                                       "
echo "====================================================================="
grep -E "(SET VERIFIED|GET VERIFIED|MATCH|PASSED|FAILED|Scoreboard Summary)" "${DIR}/sim_soc_uvm.log" || true

echo ""
echo "====================================================================="
echo "  Waveform dump.fsdb generated at: ${DIR}/dump.fsdb"
echo "  To view waveform in Verdi, run:"
echo "    cd ${DIR} && verdi -ssf dump.fsdb &"
echo "====================================================================="
