// =============================================================================
// File    : soc_uart_env.sv
// Purpose : UVM Environment for SoC UART C-Firmware Co-Verification
// =============================================================================

`ifndef SOC_UART_ENV_SV
`define SOC_UART_ENV_SV

class soc_uart_env extends uvm_env;

  `uvm_component_utils(soc_uart_env)

  soc_uart_agent      agent;
  soc_uart_scoreboard sb;

  function new(string name = "soc_uart_env", uvm_component parent = null);
    super.new(name, parent);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    agent = soc_uart_agent::type_id::create("agent", this);
    sb    = soc_uart_scoreboard::type_id::create("sb", this);
  endfunction : build_phase

  virtual function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    agent.ap.connect(sb.mon_export);
  endfunction : connect_phase

endclass : soc_uart_env

`endif // SOC_UART_ENV_SV
