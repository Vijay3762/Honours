#!/usr/bin/env bash
set -e
source /tools/Xilinx/Vivado/2024.2/settings64.sh

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ="$(cd "${ROOT}/.." && pwd)"
WORK="${PROJ}/work_cpu_uart_debug"
mkdir -p "${WORK}"
cd "${WORK}"

echo "=== Debug trace: IC->bridge bvalid path ==="

xvlog -sv -i "${PROJ}/rtl/uart/include" -log compile.log \
    "${PROJ}/rtl/soc_top.sv" \
    "${PROJ}/rtl/simple_cpu_master.sv" \
    "${PROJ}/rtl/axi_mem.sv" \
    "${PROJ}/rtl/axi_dummy_slave.sv" \
    "${PROJ}/rtl/interconnect/axi_interconnect.v" \
    "${PROJ}/rtl/interconnect/arbiter.v" \
    "${PROJ}/rtl/interconnect/priority_encoder.v" \
    "${PROJ}/rtl/axi_uart_bridge.v" \
    "${PROJ}/rtl/uart/axi_uart_top.v" \
    "${PROJ}/rtl/uart/uart_controller.v" \
    "${PROJ}/rtl/uart/uart_transmitter.v" \
    "${PROJ}/rtl/uart/uart_receiver.v" \
    "${PROJ}/rtl/uart/uart_parity_bit_compute.v" \
    "${PROJ}/rtl/uart/axi_internal_fifo.v" \
    "${PROJ}/tb/tb_cpu_uart_debug.sv"

xelab -debug typical -timescale 1ns/1ps -snapshot tb_cpu_uart_debug \
    -log elaborate.log tb_cpu_uart_debug

xsim tb_cpu_uart_debug -runall -log sim.log
