#!/usr/bin/env bash
# =============================================================================
# run_pooling_tb.sh – Complete Verification Suite for 2x2 Spatial Pooling IP
#
# Usage (run from Honours/ directory):
#   ./script/run_pooling_tb.sh
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/script
PROJ="$(cd "${ROOT}/.." && pwd)"                      # Honours/
WORK="${PROJ}/work_pooling"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  2x2 Spatial Pooling Engine Verification Suite"
echo "  Project: ${PROJ}"
echo "============================================================"
echo ""

# 1. Standalone Core Engine Verification
echo ">>> Running 1/3: Standalone Pooling Engine (tb_pooling_unit)..."
iverilog -g2012 \
    -I "${PROJ}/rtl/pooling" \
    -o sim_pool_unit \
    "${PROJ}/rtl/pooling/pooling_unit.sv" \
    "${PROJ}/tb/tb_pooling_unit.sv"

vvp sim_pool_unit
echo ""

# 2. Standalone AXI Slave Peripheral Verification
echo ">>> Running 2/3: Standalone AXI Pooling Peripheral (tb_axi_pooling)..."
iverilog -g2012 \
    -I "${PROJ}/rtl/pooling" \
    -o sim_axi_pooling \
    "${PROJ}/rtl/pooling/pooling_unit.sv" \
    "${PROJ}/rtl/pooling/axi_pooling.sv" \
    "${PROJ}/tb/tb_axi_pooling.sv"

vvp sim_axi_pooling
echo ""

# 3. Interconnect Integration Verification
echo ">>> Running 3/3: Pooling Unit on AXI Interconnect (tb_pooling_on_interconnect)..."
iverilog -g2012 \
    -I "${PROJ}/rtl/interconnect" \
    -I "${PROJ}/rtl/pooling" \
    -o sim_pool_ic \
    "${PROJ}/rtl/pooling/pooling_unit.sv" \
    "${PROJ}/rtl/pooling/axi_pooling.sv" \
    "${PROJ}/rtl/interconnect/axi_interconnect.v" \
    "${PROJ}/rtl/interconnect/arbiter.v" \
    "${PROJ}/rtl/interconnect/priority_encoder.v" \
    "${PROJ}/tb/tb_pooling_on_interconnect.sv"

vvp sim_pool_ic
echo ""

echo ">>> All Pooling Unit simulations (Core, AXI Peripheral, Interconnect) completed successfully!"
