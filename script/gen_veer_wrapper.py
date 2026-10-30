#!/usr/bin/env python3
"""
gen_veer_wrapper.py -- generate rtl/veer_core_wrapper.sv from the VeeR EL2
core's own port list.

The VeeR EL2 wrapper (rtl/cpu/veer_el2/el2_veer_wrapper.sv) has ~200 ports:
the IFU and the LSU AXI4 master interfaces are needed by the SoC, everything
else (system bus slave, DMA, AHB, JTAG, debug, performance counters) is tied
off.  Writing that by hand is error prone, so this script parses the core's
port list and emits the wrapper.

Usage:
    python3 script/gen_veer_wrapper.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.normpath(os.path.join(HERE, ".."))
CORE = os.path.join(PROJ, "rtl", "cpu", "veer_el2", "el2_veer_wrapper.sv")
OUT = os.path.join(PROJ, "rtl", "veer_core_wrapper.sv")

PORT_RE = re.compile(
    r"^\s*(input|output|inout)\s+(?:wire|logic)?\s*"
    r"(?:\[([^\]]*)\]\s*)?([A-Za-z_][A-Za-z0-9_]*)\s*,?\s*(?://.*)?$"
)

# ---------------------------------------------------------------------------
# Ports that are simply tied to a constant
# ---------------------------------------------------------------------------
TIE_ONE = {
    "mpc_reset_run_req",
    "lsu_bus_clk_en", "ifu_bus_clk_en", "dbg_bus_clk_en", "dma_bus_clk_en",
}

# Ports that are left open (unused outputs of the core)
LEAVE_OPEN = {
    "dec_tlu_perfcnt0", "dec_tlu_perfcnt1", "dec_tlu_perfcnt2",
    "dec_tlu_perfcnt3", "iccm_ecc_single_error", "iccm_ecc_double_error",
    "dccm_ecc_single_error", "dccm_ecc_double_error",
    "dccm_write_readback_error", "jtag_tdo", "jtag_tdoEn",
    "shadow_core_trace_rv_i_insn_ip", "shadow_core_trace_rv_i_address_ip",
    "shadow_core_trace_rv_i_valid_ip", "shadow_core_trace_rv_i_exception_ip",
    "shadow_core_trace_rv_i_ecause_ip", "shadow_core_trace_rv_i_interrupt_ip",
    "shadow_core_trace_rv_i_tval_ip",
    "mpc_debug_halt_ack", "mpc_debug_run_ack", "debug_brkpt_status",
    "o_cpu_halt_ack", "o_cpu_halt_status", "o_debug_mode_status",
    "o_cpu_run_ack", "dmi_uncore_rdata", "dmi_active",
    "trace_rv_i_insn_ip", "trace_rv_i_address_ip", "trace_rv_i_valid_ip",
    "trace_rv_i_exception_ip", "trace_rv_i_ecause_ip",
    "trace_rv_i_interrupt_ip", "trace_rv_i_tval_ip",
}

# Ports that the SoC actually uses
EXPOSE_PREFIX = ("lsu_axi_", "ifu_axi_")

# Ports of the core that become inputs of the wrapper
WRAPPER_INPUTS = {
    "clk", "rst_l", "dbg_rst_l", "rst_vec", "nmi_int", "nmi_vec", "jtag_id",
    "timer_int", "soft_int", "extintsrc_req", "core_id",
}

# Ports of the core that become outputs of the wrapper
WRAPPER_OUTPUTS = set()


def parse_defined_macros(path):
    """Macros defined in common_defines.vh (`define NAME [value])."""
    macros = set()
    with open(path) as f:
        for line in f:
            m = re.match(r"\s*`define\s+([A-Za-z_][A-Za-z0-9_]*)", line)
            if m:
                macros.add(m.group(1))
    return macros


def parse_ports(path, macros):
    """Return the ports of the core, honouring the `ifdef blocks."""
    ports = []
    inside = False
    # stack of (active_now, any_branch_active)
    ifdef_stack = []
    skip_depth = 0

    def skipping():
        return skip_depth > 0

    with open(path) as f:
        for line in f:
            if not inside:
                if re.match(r"^\s*module\s+el2_veer_wrapper", line):
                    inside = True
                continue
            if re.match(r"^\s*\);\s*$", line):
                break

            m_if = re.match(r"^\s*`ifdef\s+([A-Za-z_][A-Za-z0-9_]*)", line)
            m_elsif = re.match(r"^\s*`elsif\s+([A-Za-z_][A-Za-z0-9_]*)", line)
            m_else = re.match(r"^\s*`else\b", line)
            m_endif = re.match(r"^\s*`endif\b", line)

            if m_if:
                cond = m_if.group(1) in macros
                ifdef_stack.append(cond)
                if not cond:
                    skip_depth += 1
                continue
            if m_elsif:
                cond = m_elsif.group(1) in macros
                prev = ifdef_stack.pop()
                if prev:
                    ifdef_stack.append(prev)
                else:
                    ifdef_stack.append(cond)
                    if cond:
                        skip_depth -= 1
                continue
            if m_else:
                prev = ifdef_stack.pop()
                ifdef_stack.append(not prev)
                if prev:
                    skip_depth += 1
                else:
                    skip_depth -= 1
                continue
            if m_endif:
                prev = ifdef_stack.pop()
                if not prev:
                    skip_depth -= 1
                continue

            if skipping():
                continue

            m = PORT_RE.match(line)
            if m:
                direction, width, name = m.groups()
                ports.append((direction, width, name))
    return ports


def main():
    macros = parse_defined_macros(
        os.path.join(PROJ, "rtl", "cpu", "veer_el2", "config",
                     "common_defines.vh"))
    ports = parse_ports(CORE, macros)
    if not ports:
        print("ERROR: no ports parsed from %s" % CORE)
        return 1

    lsu = [p for p in ports if p[2].startswith("lsu_axi_")]
    ifu = [p for p in ports if p[2].startswith("ifu_axi_")]
    other = [p for p in ports
             if not p[2].startswith(EXPOSE_PREFIX)
             and p[2] not in WRAPPER_INPUTS
             and p[2] not in WRAPPER_OUTPUTS]

    # Everything that is not used by the SoC is tied off:
    #   - inputs  -> constant
    #   - outputs -> an internal wire that is simply not forwarded
    # The ports that matter are classified explicitly above (TIE_ONE /
    # LEAVE_OPEN), the rest are the unused SB / DMA / AHB / DMI interfaces.

    o = []
    a = o.append
    a("// ============================================================================")
    a("// File    : veer_core_wrapper.sv")
    a("// Project : RISC-V CNN Digit Recognition SoC")
    a("// Purpose : Integration wrapper for the Western Digital VeeR EL2 core.")
    a("//")
    a("//           The core exposes two AXI4 master interfaces that the SoC uses:")
    a("//             IFU - instruction fetch   -> interconnect master port 0")
    a("//             LSU - data accesses       -> interconnect master port 1")
    a("//           Everything else (system bus slave, DMA, AHB, JTAG, debug) is")
    a("//           tied off.")
    a("//")
    a("//           The two SystemVerilog interface ports of the core")
    a("//           (el2_mem_export / el2_icache_export) are the external SRAM")
    a("//           ports of the ICCM / DCCM / instruction cache.  They are")
    a("//           disabled in this configuration (ICCM_ENABLE = DCCM_ENABLE =")
    a("//           ICACHE_ENABLE = 0 in rtl/cpu/veer_el2/config/el2_param_simple.vh)")
    a("//           so nothing has to be attached to them.")
    a("//")
    a("//           NOTE: el2_veer_wrapper has interface ports, therefore it can")
    a("//                 not be the top level of an xelab run - it always has to")
    a("//                 be instantiated inside a module like this one.")
    a("//")
    a("//           Generated by script/gen_veer_wrapper.py - do not edit by hand.")
    a("// ============================================================================")
    a("")
    a("`timescale 1ns/1ps")
    a("`default_nettype none")
    a("")
    a("module veer_core_wrapper")
    a("import el2_pkg::*;")
    a("#(")
    a("`include \"el2_param.vh\"")
    a(",")
    a("parameter ICCM_HEX = \"\"")
    a(")")
    a("(")
    a("    // ------------------------------------------------------------------")
    a("    // Clock / reset")
    a("    // ------------------------------------------------------------------")
    a("    input  wire         clk,")
    a("    input  wire         rst_l,           // active low core reset")
    a("    input  wire         dbg_rst_l,       // active low debug reset")
    a("    input  wire  [31:1] rst_vec,         // reset vector (byte address >> 1)")
    a("    input  wire         nmi_int,")
    a("    input  wire  [31:1] nmi_vec,")
    a("    input  wire  [31:1] jtag_id,")
    a("")
    a("    // ------------------------------------------------------------------")
    a("    // LSU AXI4 master (data accesses)")
    a("    // ------------------------------------------------------------------")
    for direction, width, name in lsu:
        a("    %-6s %-28s %s," % (
            direction + " wire",
            ("[%s]" % width) if width else "",
            name))
    a("")
    a("    // ------------------------------------------------------------------")
    a("    // IFU AXI4 master (instruction fetch)")
    a("    // ------------------------------------------------------------------")
    for direction, width, name in ifu:
        a("    %-6s %-28s %s," % (
            direction + " wire",
            ("[%s]" % width) if width else "",
            name))
    a("")
    a("    // ------------------------------------------------------------------")
    a("    // Interrupt sources / core id")
    a("    // ------------------------------------------------------------------")
    a("    input  wire         timer_int,")
    a("    input  wire         soft_int,")
    a("    input  wire  [pt.PIC_TOTAL_INT:1] extintsrc_req,")
    a("    input  wire  [31:4] core_id")
    a(");")
    a("")
    a("    // ------------------------------------------------------------------")
    a("    // Interface ports of the core (external ICCM / DCCM / I$ SRAM).")
    a("    // The local memories are disabled in this configuration, so the")
    a("    // signals are constant and nothing has to be attached to them.")
    a("    // ------------------------------------------------------------------")
    a("    el2_mem_if mem_export_if ();")
    a("")

    # ---- internal declarations for the tied off ports -----------------------
    a("    // ------------------------------------------------------------------")
    a("    // Tied off core ports")
    a("    // ------------------------------------------------------------------")
    for direction, width, name in other:
        if name in LEAVE_OPEN:
            continue
        if name in WRAPPER_INPUTS or name in WRAPPER_OUTPUTS:
            continue
        typ = ("logic [%s]" % width) if width else "logic"
        a("    %-18s %s;" % (typ, name))
    a("")

    # ---- instantiation ------------------------------------------------------
    a("    // ------------------------------------------------------------------")
    a("    // Core instance")
    a("    // ------------------------------------------------------------------")
    a("    el2_veer_wrapper rvtop (")
    conns = []
    for direction, width, name in ports:
        if name.startswith(EXPOSE_PREFIX) or name in WRAPPER_OUTPUTS \
                or name in WRAPPER_INPUTS:
            conns.append("        .%-34s (%s)" % (name, name))
            continue
        if name in LEAVE_OPEN:
            conns.append("        .%-34s ()" % name)
            continue
        if name in TIE_ONE:
            conns.append("        .%-34s (1'b1)" % name)
            continue
        if direction == "input":
            # '0 is context sized, so it fits every port width
            conns.append("        .%-34s ('0)" % name)
        else:
            conns.append("        .%-34s (%s)" % (name, name))
    # The two interface ports are not parsed (they are not input/output
    # declarations) and have to be connected explicitly.
    conns.append("        .%-34s (mem_export_if.veer_sram_src)" % "el2_mem_export")
    conns.append("        .%-34s (mem_export_if.veer_icache_src)" % "el2_icache_export")
    a(",\n".join(conns))
    a("    );")
    a("")
    a("    // The SRAM interfaces of the core are connected to a local interface")
    a("    // instance.  The instruction cache memory (ICCM) is modelled by")
    a("    // veer_iccm_ram, the data cache memory (DCCM) is not used in this")
    a("    // configuration (DCCM_ENABLE = 0).")
    a("    assign mem_export_if.clk = clk;")
    a("")
    a("    veer_iccm_ram #(")
    a("        .ICCM_NUM_BANKS     (4),")
    a("        .ICCM_BITS          (16),")
    a("        .ICCM_BANK_INDEX_LO (4),")
    a("        .ICCM_ECC_WIDTH     (7),")
    a("        .MEM_HEX            (ICCM_HEX)")
    a("    ) iccm_ram (")
    a("        .clk       (clk),")
    a("        .clken     (mem_export_if.iccm_clken),")
    a("        .wren_bank (mem_export_if.iccm_wren_bank),")
    a("        .addr_bank (mem_export_if.iccm_addr_bank),")
    a("        .wr_data   (mem_export_if.iccm_bank_wr_data),")
    a("        .wr_ecc    (mem_export_if.iccm_bank_wr_ecc),")
    a("        .rd_data   (mem_export_if.iccm_bank_dout),")
    a("        .rd_ecc    (mem_export_if.iccm_bank_ecc)")
    a("    );")
    a("")
    a("endmodule")
    a("")
    a("`default_nettype wire")
    a("")

    with open(OUT, "w") as f:
        f.write("\n".join(o))

    print("wrote %s" % OUT)
    print("  LSU ports : %d" % len(lsu))
    print("  IFU ports : %d" % len(ifu))
    print("  tied off  : %d" % len(other))
    return 0


if __name__ == "__main__":
    sys.exit(main())
