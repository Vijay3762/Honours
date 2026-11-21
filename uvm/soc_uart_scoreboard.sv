// =============================================================================
// File    : soc_uart_scoreboard.sv
// Purpose : UVM Scoreboard verifying C firmware UART output from VeeR EL2 SoC
// =============================================================================

`ifndef SOC_UART_SCOREBOARD_SV
`define SOC_UART_SCOREBOARD_SV

class soc_uart_scoreboard extends uvm_scoreboard;

  `uvm_component_utils(soc_uart_scoreboard)

  virtual uart_if vif;
  uvm_analysis_imp #(uart_seq_item, soc_uart_scoreboard) mon_export;

  bit [7:0] exp_queue[$];
  int match_count = 0;
  int mismatch_count = 0;

  function new(string name = "soc_uart_scoreboard", uvm_component parent = null);
    super.new(name, parent);
    mon_export = new("mon_export", this);
    // Expected bytes from C firmware (uart_test.c): 'A' then 'B'
    exp_queue.push_back(8'h41); // 'A'
    exp_queue.push_back(8'h42); // 'B'
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    void'(uvm_config_db#(virtual uart_if)::get(this, "", "vif", vif));
  endfunction : build_phase

  virtual function void write(uart_seq_item item);
    bit [7:0] exp_data;
    if (exp_queue.size() == 0) begin
      `uvm_error("SCB_UNEXPECTED", $sformatf("Received unexpected byte from SoC: 0x%02X", item.data))
      mismatch_count++;
      if (vif != null) begin
        vif.vip_mismatch_count = mismatch_count;
        vif.vip_match          = 1'b0;
      end
      return;
    end

    exp_data = exp_queue.pop_front();
    if (vif != null) begin
      vif.vip_expected_byte = exp_data;
    end

    if (exp_data === item.data) begin
      match_count++;
      if (vif != null) begin
        vif.vip_match          = 1'b1;
        vif.vip_match_count    = match_count;
      end
      `uvm_info("SCB_MATCH", $sformatf("MATCH #%0d! Expected: 0x%02X ('%c') == Received from SoC: 0x%02X ('%c')",
                match_count, exp_data, (exp_data >= 32 && exp_data <= 126) ? exp_data : ".",
                item.data, (item.data >= 32 && item.data <= 126) ? item.data : "."), UVM_LOW)
    end else begin
      mismatch_count++;
      if (vif != null) begin
        vif.vip_match          = 1'b0;
        vif.vip_mismatch_count = mismatch_count;
      end
      `uvm_error("SCB_MISMATCH", $sformatf("MISMATCH #%0d! Expected: 0x%02X != Received from SoC: 0x%02X",
                 mismatch_count, exp_data, item.data))
    end
  endfunction : write

  virtual function void check_phase(uvm_phase phase);
    super.check_phase(phase);
    if (exp_queue.size() != 0) begin
      `uvm_error("SCB_LEFTOVER", $sformatf("%0d expected bytes were not emitted by the SoC!", exp_queue.size()))
    end
    `uvm_info("SCB_SUMMARY", $sformatf("SoC UART Scoreboard Summary: %0d Matches, %0d Mismatches",
              match_count, mismatch_count), UVM_LOW)
  endfunction : check_phase

endclass : soc_uart_scoreboard

`endif // SOC_UART_SCOREBOARD_SV
