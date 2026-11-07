#!/usr/bin/env bash
# =============================================================================
# run_uart_unit_tb.sh  –  Compile and simulate tb_uart_unit
#                         Testing UART Write and Read individually
#                         WITHOUT AXI and WITHOUT Core (just TB).
#
# Generates dump.fsdb (Verdi FSDB) via Novas PLI.
#
# Usage (run from Honours/ directory):
#   ./run_uart_unit_tb.sh
#
# View waveform in Verdi:
#   verdi -sv -ssf run/dump.fsdb &
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/
RUN="${ROOT}/run"

mkdir -p "${RUN}"
cd "${RUN}"

unset VCS_ARCH_OVERRIDE

# ---------------------------------------------------------------------------
# Synopsys Tool Paths
# ---------------------------------------------------------------------------
VCS_HOME="${VCS_HOME:-/home/student/snps_tools_target/vcs/U-2023.03}"
VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
NOVAS_TAB="${VERDI_HOME}/share/PLI/VCS/linux64/novas.tab"
NOVAS_LIB="${VERDI_HOME}/share/PLI/VCS/linux64/pli.a"

VCS_BIN="${VCS_HOME}/bin/vcs"
[[ -x "$VCS_BIN" ]] || VCS_BIN="vcs"

SOURCES=(
    "${ROOT}/rtl/uart/uart_parity_bit_compute.v"
    "${ROOT}/rtl/uart/uart_transmitter.v"
    "${ROOT}/rtl/uart/uart_receiver.v"
    "${ROOT}/rtl/uart/uart_controller.v"
    "${ROOT}/tb/tb_uart_unit.sv"
)

echo "============================================================"
echo "  UART Unit Testbench (NO AXI, NO Core, Just TB)"
echo "============================================================"
echo "Project Root : ${ROOT}"
echo "Run Dir      : ${RUN}"
echo ""

# ---------------------------------------------------------------------------
# Check Source Files
# ---------------------------------------------------------------------------
for f in "${SOURCES[@]}"; do
    if [[ ! -f "$f" ]]; then
        echo "ERROR: Missing file: $f"
        exit 1
    fi
done

# ---------------------------------------------------------------------------
# Check Verdi PLI
# ---------------------------------------------------------------------------
if [[ -f "${NOVAS_TAB}" && -f "${NOVAS_LIB}" ]]; then
    NOVAS_FLAGS=(-P "${NOVAS_TAB}" "${NOVAS_LIB}")
    echo "Verdi PLI    : Found"
else
    NOVAS_FLAGS=()
    echo "Verdi PLI    : Not found (compiling without FSDB)"
fi

# ---------------------------------------------------------------------------
# VCS Compile
# ---------------------------------------------------------------------------
echo "=== Step 1: Compiling with VCS ==="
"${VCS_BIN}" \
    -full64 \
    -sverilog \
    -timescale=1ns/1ps \
    -debug_access+all \
    +define+SIMULATION \
    +define+FSDB \
    -top tb_uart_unit \
    -o "${RUN}/simv_uart_unit" \
    -l "${RUN}/vcs_compile_uart_unit.log" \
    "${NOVAS_FLAGS[@]}" \
    "${SOURCES[@]}"

# ---------------------------------------------------------------------------
# Simulation Execution
# ---------------------------------------------------------------------------
echo ""
echo "=== Step 2: Running Simulation ==="
"${RUN}/simv_uart_unit" 2>&1 | tee "${RUN}/vcs_sim_uart_unit.log"

if [[ -f "${RUN}/dump.fsdb" ]]; then
    echo ""
    echo "============================================================"
    echo "  FSDB Waveform written: ${RUN}/dump.fsdb"
    echo "  Open with:"
    echo "  verdi -sv -ssf ${RUN}/dump.fsdb &"
    echo "============================================================"
fi

echo "=== Done ==="
