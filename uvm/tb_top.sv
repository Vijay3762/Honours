// =============================================================================
// File    : tb_top.sv
// Purpose : Top-level testbench for UART UVM Verification
//           Demonstrates uvm_config_db::set and FSDB dumping for Verdi
// =============================================================================

`timescale 1ns / 1ps

module tb_top;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import uart_pkg::*;

  // Clock and Reset signals
  logic clk;
  logic rstn;

  // Clock generation: 100MHz (10ns period)
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // Reset generation
  initial begin
    rstn = 1'b0;
    #100;
    rstn = 1'b1;
  end

  // Instantiate Interface (ONLY uart_rx_i and uart_tx_o)
  uart_if vif (
    .clk  (clk),
    .rstn (rstn)
  );

  // Instantiate Standalone DUT (ONLY uart_rx_i and uart_tx_o)
  uart_dut u_dut (
    .clk       (clk),
    .rstn      (rstn),
    .uart_rx_i (vif.uart_rx_i),
    .uart_tx_o (vif.uart_tx_o)
  );

  // Waveform Dumper for Verdi
  initial begin
    $fsdbDumpfile("dump.fsdb");
    $fsdbDumpvars(0, tb_top);
  end

  // UVM Config DB SET and Test Kickoff
  initial begin
    `uvm_info("TB_TOP", "=================================================", UVM_LOW)
    `uvm_info("TB_TOP", "Setting virtual interface vif into uvm_config_db...", UVM_LOW)
    
    // Explicit uvm_config_db::set
    uvm_config_db#(virtual uart_if)::set(null, "*", "vif", vif);
    
    `uvm_info("UVM_SET_CHECK", ">>> [SET VERIFIED] tb_top successfully set vif into uvm_config_db! <<<", UVM_LOW)
    `uvm_info("TB_TOP", "=================================================", UVM_LOW)

    // Run test
    run_test("uart_test");
  end

endmodule
