// =============================================================================
// File    : uart_agent.sv
// Purpose : Simple UART Agent packaging sequencer, driver, and monitor
// =============================================================================

`ifndef UART_AGENT_SV
`define UART_AGENT_SV

class uart_agent extends uvm_agent;

  `uvm_component_utils(uart_agent)

  uart_sequencer  seqr;
  uart_driver     drv;
  uart_monitor    mon;

  uvm_analysis_port #(uart_seq_item) drv_ap;
  uvm_analysis_port #(uart_seq_item) mon_ap;

  function new(string name = "uart_agent", uvm_component parent = null);
    super.new(name, parent);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    drv_ap = new("drv_ap", this);
    mon_ap = new("mon_ap", this);

    seqr = uart_sequencer::type_id::create("seqr", this);
    drv  = uart_driver::type_id::create("drv", this);
    mon  = uart_monitor::type_id::create("mon", this);
  endfunction : build_phase

  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    drv.seq_item_port.connect(seqr.seq_item_export);
    drv.ap.connect(drv_ap);
    mon.ap.connect(mon_ap);
  endfunction : connect_phase

endclass : uart_agent

`endif // UART_AGENT_SV
