// =============================================================================
// File    : soc_uart_agent.sv
// Purpose : UVM VIP Agent encapsulating Sequencer, Driver, and Monitor
//           for SoC UART verification
// =============================================================================

`ifndef SOC_UART_AGENT_SV
`define SOC_UART_AGENT_SV

class soc_uart_agent extends uvm_agent;

  `uvm_component_utils(soc_uart_agent)

  soc_uart_monitor mon;
  uart_driver      drv;
  uart_sequencer   seqr;

  uvm_analysis_port #(uart_seq_item) ap;

  function new(string name = "soc_uart_agent", uvm_component parent = null);
    super.new(name, parent);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    ap = new("ap", this);

    mon = soc_uart_monitor::type_id::create("mon", this);
    if (get_is_active() == UVM_ACTIVE) begin
      drv  = uart_driver::type_id::create("drv", this);
      seqr = uart_sequencer::type_id::create("seqr", this);
    end
  endfunction : build_phase

  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    mon.ap.connect(ap);
    if (get_is_active() == UVM_ACTIVE) begin
      drv.seq_item_port.connect(seqr.seq_item_export);
    end
  endfunction : connect_phase

endclass : soc_uart_agent

`endif // SOC_UART_AGENT_SV
