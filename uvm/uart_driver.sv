// =============================================================================
// File    : uart_driver.sv
// Purpose : Simple UART Driver verifying uvm_config_db::get and driving uart_rx_i
// =============================================================================

`ifndef UART_DRIVER_SV
`define UART_DRIVER_SV

class uart_driver extends uvm_driver #(uart_seq_item);

  `uvm_component_utils(uart_driver)

  virtual uart_if vif;
  uvm_analysis_port #(uart_seq_item) ap;

  // Timing: 10 clocks per bit @ 100MHz = 100ns per bit
  localparam real BIT_TIME_NS = 100.0;

  function new(string name = "uart_driver", uvm_component parent = null);
    super.new(name, parent);
    ap = new("ap", this);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    // Explicit verification of uvm_config_db::get
    if (!uvm_config_db#(virtual uart_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("GET_FAIL", "Failed to get virtual interface from uvm_config_db in driver!");
    end else begin
      `uvm_info("UVM_GET_CHECK", ">>> [GET VERIFIED] Driver successfully fetched vif from uvm_config_db! <<<", UVM_LOW)
    end
  endfunction : build_phase

  virtual task run_phase(uvm_phase phase);
    vif.uart_rx_i <= 1'b1; // Idle line
    @(posedge vif.rstn);
    #200;

    forever begin
      seq_item_port.get_next_item(req);
      drive_byte(req.data);
      ap.write(req);
      seq_item_port.item_done();
    end
  endtask : run_phase

  virtual task drive_byte(bit [7:0] data);
    `uvm_info("DRV_TX", $sformatf("Driving serial frame on uart_rx_i: 0x%02X ('%c')",
              data, (data >= 32 && data <= 126) ? data : "."), UVM_LOW)

    // Start bit (0)
    vif.uart_rx_i <= 1'b0;
    #(BIT_TIME_NS);

    // 8 Data bits (LSB to MSB)
    for (int i = 0; i < 8; i++) begin
      vif.uart_rx_i <= data[i];
      #(BIT_TIME_NS);
    end

    // Stop bit (1)
    vif.uart_rx_i <= 1'b1;
    #(BIT_TIME_NS);

    // Guard time between frames
    #(BIT_TIME_NS * 2);
  endtask : drive_byte

endclass : uart_driver

`endif // UART_DRIVER_SV
