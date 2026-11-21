// =============================================================================
// File    : soc_uart_monitor.sv
// Purpose : UVM Monitor capturing serial frames from SoC uart_tx_o
//           Baud Divisor = 50, Clock = 100 MHz -> Bit Period = 510.0 ns
// =============================================================================

`ifndef SOC_UART_MONITOR_SV
`define SOC_UART_MONITOR_SV

class soc_uart_monitor extends uvm_monitor;

  `uvm_component_utils(soc_uart_monitor)

  virtual uart_if vif;
  uvm_analysis_port #(uart_seq_item) ap;

  // Baud Divisor = 50: (50 + 1) * 10 ns = 510.0 ns
  localparam real BIT_TIME_NS = 510.0;

  function new(string name = "soc_uart_monitor", uvm_component parent = null);
    super.new(name, parent);
    ap = new("ap", this);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    if (!uvm_config_db#(virtual uart_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("GET_FAIL", "Failed to get virtual interface from uvm_config_db in soc_uart_monitor!");
    end else begin
      `uvm_info("UVM_GET_CHECK", ">>> [GET VERIFIED] soc_uart_monitor successfully fetched vif from uvm_config_db! <<<", UVM_LOW)
    end
  endfunction : build_phase

  virtual task run_phase(uvm_phase phase);
    uart_seq_item item;
    bit [7:0] captured_byte;

    @(posedge vif.rstn);
    #200;

    forever begin
      // Wait for Start bit (falling edge of uart_tx_o)
      @(negedge vif.uart_tx_o);

      #(BIT_TIME_NS / 2.0); // Centre of the start bit

      // Sample 8 data bits (LSB to MSB)
      for (int i = 0; i < 8; i++) begin
        #(BIT_TIME_NS);     // Centre of each data bit
        captured_byte[i] = vif.uart_tx_o;
      end

      // Wait for Stop bit
      #(BIT_TIME_NS);

      item = uart_seq_item::type_id::create("item");
      item.data = captured_byte;

      // Update interface probe signals for Verdi display
      vif.vip_captured_byte = captured_byte;
      vif.vip_byte_valid    = 1'b1;

      `uvm_info("SOC_MON_RX", $sformatf("Captured serial frame from SoC uart_tx_o: 0x%02X ('%c')",
                captured_byte, (captured_byte >= 32 && captured_byte <= 126) ? captured_byte : "."), UVM_LOW)

      ap.write(item);

      #100;
      vif.vip_byte_valid    = 1'b0;
    end
  endtask : run_phase

endclass : soc_uart_monitor

`endif // SOC_UART_MONITOR_SV
