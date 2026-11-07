// =============================================================================
// File    : uart_env.sv
// Purpose : Simple UART Environment connecting Agent and Scoreboard
// =============================================================================

`ifndef UART_ENV_SV
`define UART_ENV_SV

class uart_env extends uvm_env;

  `uvm_component_utils(uart_env)

  uart_agent      agent;
  uart_scoreboard scb;

  function new(string name = "uart_env", uvm_component parent = null);
    super.new(name, parent);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent = uart_agent::type_id::create("agent", this);
    scb   = uart_scoreboard::type_id::create("scb", this);
  endfunction : build_phase

  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    agent.drv_ap.connect(scb.drv_export);
    agent.mon_ap.connect(scb.mon_export);
  endfunction : connect_phase

endclass : uart_env

`endif // UART_ENV_SV
