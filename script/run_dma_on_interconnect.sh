#!/usr/bin/env bash
source /tools/Xilinx/Vivado/2024.2/settings64.sh

PROJ="/home/shinchan/Desktop/Honours Project/Honours"
WORK="${PROJ}/work_dma_on_interconnect"
mkdir -p "${WORK}"
cd "${WORK}"

echo "============================================================"
echo "  DMA on Interconnect Test — Vivado XSim"
echo "============================================================"

xvlog -sv -i "${PROJ}/rtl/uart/include" -log compile.log \
    "${PROJ}/rtl/dma/axi_dma_ctrl.sv" \
    "${PROJ}/rtl/interconnect/axi_interconnect.v" \
    "${PROJ}/rtl/interconnect/arbiter.v" \
    "${PROJ}/rtl/interconnect/priority_encoder.v" \
    "${PROJ}/rtl/interconnect/axi_interconnect_wrap_3x8.v" \
    "${PROJ}/rtl/axi_mem.sv" \
    "${PROJ}/rtl/axi_dummy_slave.sv" \
    "${PROJ}/tb/tb_dma_on_interconnect.sv"

xelab -debug typical -timescale 1ns/1ps -snapshot tb_dma_on_interconnect -log elaborate.log tb_dma_on_interconnect
xsim tb_dma_on_interconnect -runall -log sim.log -wdb dma_on_interconnect.wdb 2>&1 | grep -E "\[FAIL\]|\[PASS\]|T[0-9] done|passed|failed|ALL TESTS|Watchdog"
