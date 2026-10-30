#!/usr/bin/env bash
# =============================================================================
# run_cpu_uart_veer.sh -- compile and simulate the VeeR EL2 UART test with
#                          Vivado XSim (xvlog / xelab / xsim).
#
# Usage:
#   source /tools/Xilinx/Vivado/2024.2/settings64.sh
#   ./script/run_cpu_uart_veer.sh
#
# What it does:
#   1. builds the UART test firmware with the RISC-V toolchain
#      (my_test/uart_test.S -> uart_test.elf -> uart_test.hex, 64-bit words)
#   2. compiles the VeeR EL2 core, the SoC and the testbench with xvlog
#   3. elaborates and runs the simulation
#   4. leaves a waveform database (cpu_uart_veer.wdb) for Vivado
#
# NOTE: xvlog cannot read a @file list, so the sources are passed explicitly.
#       The project path must not contain spaces.
# =============================================================================

set -e

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # Honours/script
PROJ="$(cd "${ROOT}/.." && pwd)"                      # Honours/
WORK="${PROJ}/work_cpu_uart_veer"
TOOLCHAIN_PATH="${TOOLCHAIN_PATH:-/opt/riscv/bin}"
VIVADO_SETTINGS="${VIVADO_SETTINGS:-/tools/Xilinx/Vivado/2024.2/settings64.sh}"

# ---------------------------------------------------------------------
# Toolchain
# ---------------------------------------------------------------------
if [ ! -x "${TOOLCHAIN_PATH}/riscv64-unknown-elf-gcc" ]; then
    echo "ERROR: RISC-V compiler not found in ${TOOLCHAIN_PATH}"
    echo "       set TOOLCHAIN_PATH=<path to the riscv bin directory>"
    exit 1
fi

# ---------------------------------------------------------------------
# Vivado
# ---------------------------------------------------------------------
if ! command -v xvlog > /dev/null 2>&1; then
    echo "ERROR: xvlog not found - source ${VIVADO_SETTINGS} first"
    exit 1
fi

# ---------------------------------------------------------------------
# Work directory
# ---------------------------------------------------------------------
mkdir -p "${WORK}"
cd "${WORK}"

# xvlog does not like spaces in the path -> use a link without spaces
LINK_ROOT="/tmp/opencode/$(basename "${PROJ}")"
mkdir -p "$(dirname "${LINK_ROOT}")"
ln -sfn "${PROJ}" "${LINK_ROOT}"
SRC="${LINK_ROOT}"

echo "============================================================"
echo "  VeeR EL2 -> UART read/write test (Vivado XSim)"
echo "============================================================"
echo "Project : ${PROJ}"
echo "Work    : ${WORK}"
echo ""

# ---------------------------------------------------------------------
# 1. Firmware
# ---------------------------------------------------------------------
echo "=== Building the firmware ==="
make -C "${SRC}/my_test" uart_test TOOLCHAIN_PATH="${TOOLCHAIN_PATH}"
cp "${SRC}/my_test/uart_test.hex" "${WORK}/uart_test.hex"
echo ""

# ---------------------------------------------------------------------
# 2. Source list
# ---------------------------------------------------------------------
CORE_FILES=(
    "${SRC}/rtl/cpu/veer_el2/include/el2_def.sv"
    "${SRC}/rtl/cpu/veer_el2/lib/el2_assert.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_mubi_pkg.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_veer_wrapper.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_lockstep_pkg.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_mem.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_pic_ctrl.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_veer.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_dma_ctrl.sv"
    "${SRC}/rtl/cpu/veer_el2/el2_pmp.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_aln_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_compress_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_ifc_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_bp_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_ic_mem.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_mem_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu_iccm_mem.sv"
    "${SRC}/rtl/cpu/veer_el2/ifu/el2_ifu.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec_decode_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec_gpr_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec_ib_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec_pmp_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec_tlu_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec_trigger.sv"
    "${SRC}/rtl/cpu/veer_el2/dec/el2_dec.sv"
    "${SRC}/rtl/cpu/veer_el2/exu/el2_exu_alu_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/exu/el2_exu_mul_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/exu/el2_exu_div_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/exu/el2_exu.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_clkdomain.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_addrcheck.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_lsc_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_stbuf.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_bus_buffer.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_bus_intf.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_ecc.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_dccm_mem.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_dccm_ctl.sv"
    "${SRC}/rtl/cpu/veer_el2/lsu/el2_lsu_trigger.sv"
    "${SRC}/rtl/cpu/veer_el2/dbg/el2_dbg.sv"
    "${SRC}/rtl/cpu/veer_el2/dmi/dmi_mux.v"
    "${SRC}/rtl/cpu/veer_el2/dmi/dmi_wrapper.v"
    "${SRC}/rtl/cpu/veer_el2/dmi/dmi_jtag_to_core_sync.v"
    "${SRC}/rtl/cpu/veer_el2/dmi/rvjtag_tap.v"
    "${SRC}/rtl/cpu/veer_el2/lib/el2_lib.sv"
    "${SRC}/rtl/cpu/veer_el2/lib/el2_mem_if.sv"
    "${SRC}/rtl/cpu/veer_el2/lib/el2_prim_generic_buf.sv"
    "${SRC}/rtl/cpu/veer_el2/lib/el2_prim_buf.sv"
    "${SRC}/rtl/cpu/veer_el2/lib/beh_lib.sv"
    "${SRC}/rtl/cpu/veer_el2/lib/mem_lib.sv"
)

SOC_FILES=(
    "${SRC}/rtl/soc_top_veer.sv"
    "${SRC}/rtl/veer_core_wrapper.sv"
    "${SRC}/rtl/veer_iccm_ram.sv"
    "${SRC}/rtl/axi_mem.sv"
    "${SRC}/rtl/axi_dummy_slave.sv"
    "${SRC}/rtl/interconnect/axi_interconnect.v"
    "${SRC}/rtl/interconnect/arbiter.v"
    "${SRC}/rtl/interconnect/priority_encoder.v"
    "${SRC}/rtl/axi_uart_bridge.v"
    "${SRC}/rtl/uart/axi_uart_top.v"
    "${SRC}/rtl/uart/uart_controller.v"
    "${SRC}/rtl/uart/uart_transmitter.v"
    "${SRC}/rtl/uart/uart_receiver.v"
    "${SRC}/rtl/uart/uart_parity_bit_compute.v"
    "${SRC}/rtl/uart/axi_internal_fifo.v"
)

TB_FILES=(
    "${SRC}/tb/tb_cpu_uart_veer.sv"
)

echo "=== Checking the source files ==="
MISSING=0
for f in "${CORE_FILES[@]}" "${SOC_FILES[@]}" "${TB_FILES[@]}"; do
    if [ ! -f "${f}" ]; then
        echo "  MISSING: ${f}"
        MISSING=1
    fi
done
for f in "${SRC}/rtl/uart/include/axi_uart_defines.vh" \
         "${SRC}/rtl/uart/include/axi_uart.vh" \
         "${SRC}/rtl/cpu/veer_el2/config/common_defines.vh"; do
    if [ ! -f "${f}" ]; then
        echo "  MISSING include: ${f}"
        MISSING=1
    fi
done
[ ${MISSING} -eq 0 ] || { echo "Aborting."; exit 1; }
echo "All source files found."
echo ""

# ---------------------------------------------------------------------
# 3. Compile
# ---------------------------------------------------------------------
echo "=== Compiling with xvlog ==="
xvlog -sv \
    -i "${SRC}/rtl/uart/include" \
    -i "${SRC}/rtl/cpu/veer_el2/config" \
    -i "${SRC}/rtl/cpu/veer_el2/include" \
    -i "${SRC}/rtl/cpu/veer_el2/lib" \
    -i "${SRC}/rtl/cpu/veer_el2/dmi" \
    -log compile.log \
    "${CORE_FILES[@]}" "${SOC_FILES[@]}" "${TB_FILES[@]}"

echo "Compilation OK."
echo ""

# ---------------------------------------------------------------------
# 4. Elaborate
# ---------------------------------------------------------------------
echo "=== Elaborating with xelab ==="
xelab -debug typical \
    -timescale 1ns/1ps \
    -snapshot tb_cpu_uart_veer \
    -log elaborate.log \
    tb_cpu_uart_veer

echo "Elaboration OK."
echo ""

# ---------------------------------------------------------------------
# 5. Simulate
# ---------------------------------------------------------------------
echo "=== Running the simulation with xsim ==="
xsim tb_cpu_uart_veer \
    -runall \
    -log sim.log \
    -wdb cpu_uart_veer.wdb

echo ""
echo "============================================================"
echo "  Simulation complete."
echo "  Waveform : ${WORK}/cpu_uart_veer.wdb"
echo "              open it in Vivado (Tools -> Open Simulation Waveform)"
echo "  Log      : ${WORK}/sim.log"
echo "============================================================"
