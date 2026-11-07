// =============================================================================
// File    : uart_pkg.sv
// Purpose : Package compiling all UART UVM Verification Components
// =============================================================================

`ifndef UART_PKG_SV
`define UART_PKG_SV

package uart_pkg;

  import uvm_pkg::*;
  `include "uvm_macros.svh"

  `include "uart_seq_item.sv"
  `include "uart_sequencer.sv"
  `include "uart_driver.sv"
  `include "uart_monitor.sv"
  `include "uart_agent.sv"
  `include "uart_scoreboard.sv"
  `include "uart_env.sv"
  `include "uart_sequence.sv"
  `include "uart_test.sv"

endpackage : uart_pkg

`endif // UART_PKG_SV
