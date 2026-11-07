// =============================================================================
// File    : uart_scoreboard.sv
// Purpose : Simple UART Scoreboard comparing driven vs echoed characters
// =============================================================================

`ifndef UART_SCOREBOARD_SV
`define UART_SCOREBOARD_SV

`uvm_analysis_imp_decl(_drv)
`uvm_analysis_imp_decl(_mon)

class uart_scoreboard extends uvm_scoreboard;

  `uvm_component_utils(uart_scoreboard)

  uvm_analysis_imp_drv #(uart_seq_item, uart_scoreboard) drv_export;
  uvm_analysis_imp_mon #(uart_seq_item, uart_scoreboard) mon_export;

  bit [7:0] exp_queue[$];
  int match_count = 0;
  int mismatch_count = 0;

  function new(string name = "uart_scoreboard", uvm_component parent = null);
    super.new(name, parent);
    drv_export = new("drv_export", this);
    mon_export = new("mon_export", this);
  endfunction : new

  virtual function void write_drv(uart_seq_item item);
    exp_queue.push_back(item.data);
    `uvm_info("SCB_DRV", $sformatf("Queued expected byte from driver: 0x%02X ('%c')",
              item.data, (item.data >= 32 && item.data <= 126) ? item.data : "."), UVM_HIGH)
  endfunction : write_drv

  virtual function void write_mon(uart_seq_item item);
    bit [7:0] exp_data;
    if (exp_queue.size() == 0) begin
      `uvm_error("SCB_UNEXPECTED", $sformatf("Received unexpected byte from DUT: 0x%02X", item.data))
      mismatch_count++;
      return;
    end

    exp_data = exp_queue.pop_front();
    if (exp_data === item.data) begin
      match_count++;
      `uvm_info("SCB_MATCH", $sformatf("MATCH #%0d! Expected: 0x%02X ('%c') == Received: 0x%02X ('%c')",
                match_count, exp_data, (exp_data >= 32 && exp_data <= 126) ? exp_data : ".",
                item.data, (item.data >= 32 && item.data <= 126) ? item.data : "."), UVM_LOW)
    end else begin
      mismatch_count++;
      `uvm_error("SCB_MISMATCH", $sformatf("MISMATCH #%0d! Expected: 0x%02X != Received: 0x%02X",
                 mismatch_count, exp_data, item.data))
    end
  endfunction : write_mon

  virtual function void check_phase(uvm_phase phase);
    super.check_phase(phase);
    if (exp_queue.size() != 0) begin
      `uvm_error("SCB_LEFTOVER", $sformatf("%0d bytes were sent but never received back!", exp_queue.size()))
    end
    `uvm_info("SCB_SUMMARY", $sformatf("Scoreboard Summary: %0d Matches, %0d Mismatches",
              match_count, mismatch_count), UVM_LOW)
  endfunction : check_phase

endclass : uart_scoreboard

`endif // UART_SCOREBOARD_SV
