// =============================================================================
// File    : uart_monitor.sv
// Purpose : Simple UART Monitor verifying uvm_config_db::get and sampling uart_tx_o
// =============================================================================

`ifndef UART_MONITOR_SV
`define UART_MONITOR_SV

class uart_monitor extends uvm_monitor;

  `uvm_component_utils(uart_monitor)

  virtual uart_if vif;
  uvm_analysis_port #(uart_seq_item) ap;

  // Timing: In uart_transmitter.v, each bit duration is (baud_div + 1) clocks.
  // With baud_div = 10 and 10ns clock period, bit duration is 110.0 ns.
  localparam real TX_BIT_TIME_NS = 110.0;

  function new(string name = "uart_monitor", uvm_component parent = null);
    super.new(name, parent);
    ap = new("ap", this);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    // Explicit verification of uvm_config_db::get
    if (!uvm_config_db#(virtual uart_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("GET_FAIL", "Failed to get virtual interface from uvm_config_db in monitor!");
    end else begin
      `uvm_info("UVM_GET_CHECK", ">>> [GET VERIFIED] Monitor successfully fetched vif from uvm_config_db! <<<", UVM_LOW)
    end
  endfunction : build_phase

  virtual task run_phase(uvm_phase phase);
    uart_seq_item item;
    bit [7:0] captured_byte;

    @(posedge vif.rstn);
    #200;

    forever begin
      // Wait for start bit (falling edge of uart_tx_o)
      @(negedge vif.uart_tx_o);

      // Wait 1.5 bit periods to align to the middle of data bit 0
      #(TX_BIT_TIME_NS * 1.5);

      // Sample 8 data bits (LSB to MSB)
      for (int i = 0; i < 8; i++) begin
        captured_byte[i] = vif.uart_tx_o;
        if (i < 7) begin
          #(TX_BIT_TIME_NS);
        end
      end

      // Wait for stop bit
      #(TX_BIT_TIME_NS);

      item = uart_seq_item::type_id::create("item");
      item.data = captured_byte;

      `uvm_info("MON_RX", $sformatf("Captured serial frame from DUT uart_tx_o: 0x%02X ('%c')",
                captured_byte, (captured_byte >= 32 && captured_byte <= 126) ? captured_byte : "."), UVM_LOW)

      ap.write(item);
    end
  endtask : run_phase

endclass : uart_monitor

`endif // UART_MONITOR_SV
