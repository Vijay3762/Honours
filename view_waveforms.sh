#!/usr/bin/env bash
# =============================================================================
# view_waveforms.sh -- Unified script to open verification waveforms in Verdi.
#
# Usage (run from Honours/ directory):
#   ./view_waveforms.sh          # Opens the SoC + C test waveform (default)
#   ./view_waveforms.sh c        # Opens the VeeR EL2 SoC + C test waveform
#   ./view_waveforms.sh uvm      # Opens the UVM UART testbench waveform
#   ./view_waveforms.sh intg     # Opens the AXI Interconnect + UART test waveform
#   ./view_waveforms.sh all      # Opens all waveforms
# =============================================================================

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export VERDI_HOME="${VERDI_HOME:-/home/student/snps_tools_target/verdi/U-2023.03-SP1}"
export SNPSLMD_LICENSE_FILE="${SNPSLMD_LICENSE_FILE:-27021@14.139.1.126}"
export PATH="${VERDI_HOME}/bin:${PATH}"

TARGET="${1:-c}"

open_c_waveform() {
    FSDB="${DIR}/run_cpu_uart_c/dump.fsdb"
    DAIDIR="${DIR}/run_cpu_uart_c/simv.daidir"
    RC="${DIR}/run_cpu_uart_c/signals.rc"
    if [[ ! -f "${FSDB}" ]]; then
        echo "Error: ${FSDB} not found! Run ./run_c_test_waveform.sh first."
        return 1
    fi
    echo "Opening VeeR EL2 SoC + C Test waveform in Verdi..."
    if [[ -d "${DAIDIR}" && -f "${RC}" ]]; then
        verdi -simflow -dbdir "${DAIDIR}" -ssf "${FSDB}" -sswr "${RC}" &
    elif [[ -f "${RC}" ]]; then
        verdi -ssf "${FSDB}" -sswr "${RC}" &
    else
        verdi -ssf "${FSDB}" &
    fi
}

open_uvm_waveform() {
    FSDB="${DIR}/uvm/dump.fsdb"
    DAIDIR="${DIR}/uvm/simv_soc_uvm.daidir"
    [[ -d "${DAIDIR}" ]] || DAIDIR="${DIR}/uvm/simv.daidir"
    RC="${DIR}/uvm/signals.rc"
    if [[ ! -f "${FSDB}" ]]; then
        echo "Error: ${FSDB} not found! Run cd uvm && ./run_soc_uvm.sh first."
        return 1
    fi
    echo "Opening UVM SoC UART waveform in Verdi..."
    if [[ -d "${DAIDIR}" && -f "${RC}" ]]; then
        verdi -simflow -dbdir "${DAIDIR}" -ssf "${FSDB}" -sswr "${RC}" &
    elif [[ -f "${RC}" ]]; then
        verdi -ssf "${FSDB}" -sswr "${RC}" &
    else
        verdi -ssf "${FSDB}" &
    fi
}

open_intg_waveform() {
    FSDB="${DIR}/run/dump.fsdb"
    if [[ ! -f "${FSDB}" ]]; then
        echo "Error: ${FSDB} not found! Run ./run_uart_tb.sh vcs first."
        return 1
    fi
    echo "Opening AXI Interconnect + UART waveform in Verdi..."
    verdi -ssf "${FSDB}" &
}

case "$TARGET" in
    c|soc|core)
        open_c_waveform
        ;;
    uvm)
        open_uvm_waveform
        ;;
    intg|ic|axi)
        open_intg_waveform
        ;;
    all)
        open_c_waveform
        sleep 1
        open_uvm_waveform
        sleep 1
        open_intg_waveform
        ;;
    *)
        echo "Usage: ./view_waveforms.sh [c|uvm|intg|all]"
        echo "  c    : VeeR EL2 Core + AXI Interconnect + UART C Test (default)"
        echo "  uvm  : UVM 1.2 UART VIP Testbench"
        echo "  intg : 3x8 AXI Interconnect + UART IP Test"
        echo "  all  : Open all waveforms"
        exit 1
        ;;
esac
