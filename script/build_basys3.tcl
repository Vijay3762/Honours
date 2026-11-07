# ==============================================================================
# build_basys3.tcl -- Automated Vivado batch build script for Basys3 FPGA
#                      (Synthesis, Implementation, Timing, and Bitstream)
#
# Usage (Vivado batch mode):
#   vivado -mode batch -source script/build_basys3.tcl
# ==============================================================================

set proj_dir [file normalize [file join [file dirname [info script]] ".."]]
set output_dir [file join $proj_dir "build_basys3"]
file mkdir $output_dir

set part "xc7a35tcpg236-1"
set top  "soc_top_veer"

puts "============================================================"
puts "  Building RISC-V MNIST SoC for Digilent Basys3 FPGA"
puts "  Part   : $part"
puts "  Top    : $top"
puts "  Output : $output_dir"
puts "============================================================"

# Create in-memory project
create_project -in_memory -part $part

# Include directories
set_property include_dirs [list \
    [file join $proj_dir "rtl/uart/include"] \
    [file join $proj_dir "rtl/cpu/veer_el2/config"] \
    [file join $proj_dir "rtl/cpu/veer_el2/include"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dmi"] \
] [current_fileset]

# Add VeeR EL2 Core Sources
add_files [list \
    [file join $proj_dir "rtl/cpu/veer_el2/include/el2_def.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/el2_assert.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_mubi_pkg.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_veer_wrapper.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_lockstep_pkg.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_mem.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_pic_ctrl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_veer.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_dma_ctrl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/el2_pmp.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_aln_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_compress_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_ifc_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_bp_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_ic_mem.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_mem_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu_iccm_mem.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/ifu/el2_ifu.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec_decode_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec_gpr_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec_ib_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec_pmp_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec_tlu_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec_trigger.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dec/el2_dec.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/exu/el2_exu_alu_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/exu/el2_exu_mul_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/exu/el2_exu_div_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/exu/el2_exu.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_clkdomain.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_addrcheck.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_lsc_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_stbuf.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_bus_buffer.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_bus_intf.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_ecc.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_dccm_mem.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_dccm_ctl.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lsu/el2_lsu_trigger.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dbg/el2_dbg.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dmi/dmi_mux.v"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dmi/dmi_wrapper.v"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dmi/dmi_jtag_to_core_sync.v"] \
    [file join $proj_dir "rtl/cpu/veer_el2/dmi/rvjtag_tap.v"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/el2_lib.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/el2_mem_if.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/el2_prim_generic_buf.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/el2_prim_buf.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/beh_lib.sv"] \
    [file join $proj_dir "rtl/cpu/veer_el2/lib/mem_lib.sv"] \
]

# Set common defines as global include
set_property is_global_include true [get_files [file join $proj_dir "rtl/cpu/veer_el2/config/common_defines.vh"]]

# Add SoC, Interconnect & Peripheral Sources
add_files [list \
    [file join $proj_dir "rtl/soc_top_veer.sv"] \
    [file join $proj_dir "rtl/veer_core_wrapper.sv"] \
    [file join $proj_dir "rtl/veer_iccm_ram.sv"] \
    [file join $proj_dir "rtl/axi_mem.sv"] \
    [file join $proj_dir "rtl/axi_dummy_slave.sv"] \
    [file join $proj_dir "rtl/interconnect/axi_interconnect.v"] \
    [file join $proj_dir "rtl/interconnect/arbiter.v"] \
    [file join $proj_dir "rtl/interconnect/priority_encoder.v"] \
    [file join $proj_dir "rtl/axi_uart_bridge.v"] \
    [file join $proj_dir "rtl/uart/axi_uart_top.v"] \
    [file join $proj_dir "rtl/uart/uart_controller.v"] \
    [file join $proj_dir "rtl/uart/uart_transmitter.v"] \
    [file join $proj_dir "rtl/uart/uart_receiver.v"] \
    [file join $proj_dir "rtl/uart/uart_parity_bit_compute.v"] \
    [file join $proj_dir "rtl/uart/axi_internal_fifo.v"] \
    [file join $proj_dir "rtl/gpio/axi_gpio.sv"] \
    [file join $proj_dir "rtl/timer/axi_timer.sv"] \
    [file join $proj_dir "rtl/seven_seg/axi_seven_seg.sv"] \
    [file join $proj_dir "rtl/dma/axi_dma_ctrl.sv"] \
    [file join $proj_dir "rtl/cnn/axi_cnn_conv.sv"] \
    [file join $proj_dir "rtl/relu/relu_unit.sv"] \
    [file join $proj_dir "rtl/relu/axi_relu.sv"] \
    [file join $proj_dir "rtl/pooling/pooling_unit.sv"] \
    [file join $proj_dir "rtl/pooling/axi_pooling.sv"] \
]

# Add XDC Constraints
read_xdc [file join $proj_dir "constraint/basys3_master.xdc"]

# 1. Synthesis
puts "=== Running Synthesis ==="
synth_design -top $top -part $part -flatten_hierarchy rebuilt
write_checkpoint -force [file join $output_dir "post_synth.dcp"]
report_utilization -file [file join $output_dir "utilization_synth.rpt"]

# 2. Optimization & Placement
puts "=== Running Optimization and Placement ==="
opt_design
place_design
write_checkpoint -force [file join $output_dir "post_place.dcp"]

# 3. Routing
puts "=== Running Routing ==="
route_design
write_checkpoint -force [file join $output_dir "post_route.dcp"]
report_timing_summary -file [file join $output_dir "timing_summary.rpt"]
report_utilization -file [file join $output_dir "utilization_route.rpt"]

# 4. Generate Bitstream
puts "=== Writing Bitstream ==="
write_bitstream -force [file join $output_dir "basys3_mnist_soc.bit"]

puts "============================================================"
puts "  Build Complete! Bitstream written to:"
puts "  [file join $output_dir basys3_mnist_soc.bit]"
puts "============================================================"
