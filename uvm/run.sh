#!/usr/bin/env bash
# =============================================================================
# run.sh - Compile, Simulate UART UVM TB & Dump Waveforms for Verdi
# =============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${DIR}/.." && pwd)"

cd "${DIR}"

unset VCS_ARCH_OVERRIDE

# Synopsys paths
export VCS_HOME="${VCS_HOME:-/home/student/snps_tools_target/vcs/U-2023.03}"
export VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
export PATH="${VCS_HOME}/bin:${VERDI_HOME}/bin:${PATH}"

NOVAS_TAB="${VERDI_HOME}/share/PLI/VCS/linux64/novas.tab"
NOVAS_LIB="${VERDI_HOME}/share/PLI/VCS/linux64/pli.a"

VCS_BIN="${VCS_HOME}/bin/vcs"
[[ -x "$VCS_BIN" ]] || VCS_BIN="vcs"

echo "====================================================================="
echo "  1. Compiling UART UVM TB with VCS (-kdb enabled for Verdi)        "
echo "====================================================================="

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
    +incdir+"${DIR}" \
    "${ROOT}/rtl/uart/uart_parity_bit_compute.v" \
    "${ROOT}/rtl/uart/uart_transmitter.v" \
    "${ROOT}/rtl/uart/uart_receiver.v" \
    "${ROOT}/rtl/uart/uart_controller.v" \
    "${DIR}/uart_dut.sv" \
    "${DIR}/uart_if.sv" \
    "${DIR}/uart_pkg.sv" \
    "${DIR}/tb_top.sv" \
    -top tb_top \
    -o "${DIR}/simv" \
    -l "${DIR}/compile.log"

echo "Compilation successful!"
echo ""
echo "====================================================================="
echo "  2. Running Simulation (Executing SET in tb_top, GET in Driver/Mon) "
echo "====================================================================="

"${DIR}/simv" \
    +UVM_TESTNAME=uart_test \
    +UVM_VERBOSITY=UVM_LOW \
    -l "${DIR}/sim.log"

echo ""
echo "====================================================================="
echo "  3. Verification Summary from sim.log                              "
echo "====================================================================="
grep -E "(SET VERIFIED|GET VERIFIED|MATCH|PASSED|FAILED)" "${DIR}/sim.log" || true

echo ""
echo "====================================================================="
echo "  Waveform dump.fsdb generated at: ${DIR}/dump.fsdb"
echo "  To view waveforms in Verdi, run:"
echo "    ${DIR}/view_verdi.sh"
echo "  Or simply:"
echo "    cd ${DIR} && verdi -ssf dump.fsdb &"
echo "====================================================================="
