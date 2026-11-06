#!/usr/bin/env bash
# =============================================================================
# run_seven_seg_tb.sh – Run Standalone & Interconnect TBs for 7-Segment Controller
#
# Usage (run from Honours/ directory):
#   ./script/run_seven_seg_tb.sh
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/script
PROJ="$(cd "${ROOT}/.." && pwd)"                      # Honours/
WORK="${PROJ}/work_seven_seg"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  7-Segment Display Controller Verification Suite"
echo "  Project: ${PROJ}"
echo "============================================================"
echo ""

# 1. Standalone Verification
echo ">>> Running 1/2: Standalone AXI 7-Segment Controller (tb_axi_seven_seg)..."
iverilog -g2012 \
    -o sim_seven_seg_standalone \
    "${PROJ}/rtl/seven_seg/axi_seven_seg.sv" \
    "${PROJ}/tb/tb_axi_seven_seg.sv"

vvp sim_seven_seg_standalone
echo ""

# 2. Interconnect Integration Verification
echo ">>> Running 2/2: 7-Segment on AXI Interconnect Port 7 (tb_seven_seg_on_interconnect)..."
iverilog -g2012 \
    -I "${PROJ}/rtl/interconnect" \
    -o sim_seven_seg_ic \
    "${PROJ}/rtl/seven_seg/axi_seven_seg.sv" \
    "${PROJ}/rtl/interconnect/axi_interconnect.v" \
    "${PROJ}/rtl/interconnect/arbiter.v" \
    "${PROJ}/rtl/interconnect/priority_encoder.v" \
    "${PROJ}/tb/tb_seven_seg_on_interconnect.sv"

vvp sim_seven_seg_ic
echo ""

echo ">>> All 7-Segment Controller simulations completed successfully!"
