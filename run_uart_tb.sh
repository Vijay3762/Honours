#!/usr/bin/env bash
# =============================================================================
# run_uart_tb.sh  –  Compile and simulate tb_uart_on_interconnect
#
# Generates dump.fsdb (Verdi FSDB) via the Novas PLI library.
#
# Usage (run from Honours/ directory):
#   ./run_uart_tb.sh              # auto-detect VCS or Icarus
#   ./run_uart_tb.sh vcs          # force VCS  (produces dump.fsdb)
#   ./run_uart_tb.sh iverilog     # force Icarus (VCD fallback, no FSDB)
#
# After simulation open waveform with:
#   verdi -sv -ssf run/dump.fsdb &
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/
RUN="${ROOT}/run"

# ---------------------------------------------------------------------------
# Verdi / Novas PLI paths  (adjust if your install differs)
# ---------------------------------------------------------------------------
VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
NOVAS_PLI_DIR="${VERDI_HOME}/share/PLI/VCS/linux64"
NOVAS_TAB="${NOVAS_PLI_DIR}/novas.tab"
NOVAS_LIB="${NOVAS_PLI_DIR}/pli.a"

ALL_SOURCES=(
    "${RUN}/axi_interconnect_wrap_3x8.v"
    "${ROOT}/rtl/interconnect/axi_interconnect.v"
    "${ROOT}/rtl/interconnect/arbiter.v"
    "${ROOT}/rtl/interconnect/priority_encoder.v"
    "${ROOT}/rtl/uart/axi_uart_top.v"
    "${ROOT}/rtl/uart/uart_controller.v"
    "${ROOT}/rtl/uart/uart_transmitter.v"
    "${ROOT}/rtl/uart/uart_receiver.v"
    "${ROOT}/rtl/uart/uart_parity_bit_compute.v"
    "${ROOT}/rtl/uart/axi_internal_fifo.v"
    "${ROOT}/rtl/axi_uart_bridge.v"
    "${ROOT}/tb/tb_uart_on_interconnect.sv"
)

# ---------------------------------------------------------------------------
# Validate source files
# ---------------------------------------------------------------------------
echo "=== Checking source files ==="
MISSING=0
for f in "${ALL_SOURCES[@]}"; do
    if [[ ! -f "$f" ]]; then echo "  MISSING: $f"; MISSING=1
    else echo "  OK: $(basename $f)"; fi
done
[[ $MISSING -eq 0 ]] || { echo "Aborting."; exit 1; }
echo ""

# ---------------------------------------------------------------------------
# Simulator selection
# ---------------------------------------------------------------------------
SIM="${1:-auto}"
if [[ "$SIM" == "auto" ]]; then
    command -v vcs      &>/dev/null && SIM="vcs"      ||
    command -v iverilog &>/dev/null && SIM="iverilog" || {
        echo "ERROR: neither vcs nor iverilog found in PATH"; exit 1; }
fi
echo "=== Simulator: ${SIM} ==="
mkdir -p "${RUN}"

# ---------------------------------------------------------------------------
# VCS  (with Verdi FSDB PLI)
# ---------------------------------------------------------------------------
if [[ "$SIM" == "vcs" ]]; then
    VCS_BIN="${VCS_HOME:-/home/student/snps_tools_target/vcs/U-2023.03}/bin/vcs"
    [[ -x "$VCS_BIN" ]] || VCS_BIN="vcs"

    # Verify PLI files exist
    if [[ ! -f "${NOVAS_TAB}" || ! -f "${NOVAS_LIB}" ]]; then
        echo "WARNING: Verdi PLI not found at ${NOVAS_PLI_DIR}"
        echo "         Compiling without FSDB support (no dump.fsdb will be written)"
        NOVAS_FLAGS=()
    else
        echo "=== Verdi PLI: ${NOVAS_PLI_DIR} ==="
        NOVAS_FLAGS=(
            -P "${NOVAS_TAB}"
            "${NOVAS_LIB}"
        )
    fi

    echo "=== Compiling ==="
    "${VCS_BIN}" \
        -full64 -sverilog -timescale=1ns/1ps \
        +incdir+"${ROOT}/rtl/uart/include" \
        +define+SIMULATION \
        -top tb_uart_on_interconnect \
        -o  "${RUN}/simv_uart_tb" \
        -l  "${RUN}/vcs_compile_uart_tb.log" \
        "${NOVAS_FLAGS[@]}" \
        "${ALL_SOURCES[@]}"

    echo ""
    echo "=== Running simulation ==="
    cd "${RUN}"
    ./simv_uart_tb 2>&1 | tee vcs_sim_uart_tb.log
    cd - >/dev/null

    FSDB="${RUN}/dump.fsdb"
    if [[ -f "${FSDB}" ]]; then
        echo ""
        echo "=== FSDB written: ${FSDB} ==="
        echo "    Open with:"
        echo "    verdi -sv -ssf ${FSDB} &"
    fi

# ---------------------------------------------------------------------------
# Icarus  (no FSDB support — VCD fallback)
# ---------------------------------------------------------------------------
elif [[ "$SIM" == "iverilog" ]]; then
    echo "NOTE: Icarus does not support the Verdi PLI."
    echo "      \$fsdbDump* calls will be ignored; no FSDB generated."
    echo ""
    echo "=== Compiling ==="
    iverilog \
        -g2012 \
        -I "${ROOT}/rtl/uart/include" \
        -D SIMULATION \
        -s tb_uart_on_interconnect \
        -o "${RUN}/sim_uart_tb.vvp" \
        "${ALL_SOURCES[@]}"

    echo ""
    echo "=== Running simulation ==="
    cd "${RUN}"
    vvp sim_uart_tb.vvp 2>&1 | tee iverilog_sim_uart_tb.log
    cd - >/dev/null

else
    echo "ERROR: Unknown simulator '$SIM'"; exit 1
fi

echo ""
echo "=== Done ==="
