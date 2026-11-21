// =============================================================================
// File    : soc_tb_top.sv
// Purpose : Top-level UVM Testbench for VeeR EL2 SoC + C-Firmware Verification
//           Loads .hex file into ICCM via $readmemh, boots core, and verifies
//           UART external data paths with UVM Monitor and Scoreboard.
// =============================================================================

`timescale 1ns / 1ps

module soc_tb_top;

  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import uart_pkg::*;

  // Clock & Reset
  logic clk;
  logic rstn;
  wire  rst = ~rstn; // active-high reset for SoC

  // 100 MHz Clock (10 ns period)
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk;
  end

  // Reset sequence
  initial begin
    rstn = 1'b0;
    #200;
    rstn = 1'b1;
  end

  // Instantiate UART Virtual Interface
  uart_if vif (
    .clk  (clk),
    .rstn (rstn)
  );

  // External Data Path Loopback: uart_tx_o -> uart_rx_wire
  wire uart_rx_wire = vif.uart_tx_o;
  assign vif.uart_rx_i = uart_rx_wire;

  // Probe CPU core TX byte to UART IP into interface for Verdi comparison
  assign vif.core_tx_byte = dut.uart_bridge.uart_top.tx_fifo_data_in_int;

  // Instantiate the real VeeR EL2 SoC
  // Loads uart_test_c.hex into ICCM RAM via $readmemh
  soc_top_veer #(
      .DATA_WIDTH (64),
      .ADDR_WIDTH (32),
      .STRB_WIDTH (8),
      .ID_WIDTH   (8),
      .IMEM_HEX   (""),
      .ICCM_HEX   ("uart_test_c.hex")
  ) dut (
      .clk             (clk),
      .rst             (rst),
      .uart_tx_o       (vif.uart_tx_o),
      .uart_rx_i       (uart_rx_wire),
      .uart_irq_o      (),
      .gpio_pads_i     (16'h0000),
      .gpio_pads_o     (),
      .gpio_pads_oen   (),
      .timer_irq_o     (),
      .dma_irq_o       (),
      .cnn_irq_o       (),
      .seven_seg_seg_o (),
      .seven_seg_dp_o  (),
      .seven_seg_an_o  (),
      .ext_master_en   (1'b0)
  );

  // FSDB Waveform Dumper for Verdi
  initial begin
    $fsdbDumpfile("dump.fsdb");
    $fsdbDumpvars(0, soc_tb_top, "+all");
  end

  // UVM Config DB Setup and Test Launch
  initial begin
    `uvm_info("SOC_TB_TOP", "=================================================", UVM_LOW)
    `uvm_info("SOC_TB_TOP", "Configuring UVM virtual interface for SoC UART...", UVM_LOW)

    uvm_config_db#(virtual uart_if)::set(null, "*", "vif", vif);

    `uvm_info("UVM_SET_CHECK", ">>> [SET VERIFIED] soc_tb_top set vif into uvm_config_db! <<<", UVM_LOW)
    `uvm_info("SOC_TB_TOP", "=================================================", UVM_LOW)

    run_test("soc_uart_test");
  end

endmodule
