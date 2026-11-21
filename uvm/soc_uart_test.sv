// =============================================================================
// File    : soc_uart_test.sv
// Purpose : UVM Test for VeeR EL2 SoC C-Firmware UART Verification
// =============================================================================

`ifndef SOC_UART_TEST_SV
`define SOC_UART_TEST_SV

class soc_uart_test extends uvm_test;

  `uvm_component_utils(soc_uart_test)

  soc_uart_env env;

  function new(string name = "soc_uart_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = soc_uart_env::type_id::create("env", this);
  endfunction : build_phase

  virtual task run_phase(uvm_phase phase);
    phase.raise_objection(this);

    `uvm_info("SOC_TEST", "=================================================", UVM_LOW)
    `uvm_info("SOC_TEST", "Starting VeeR EL2 SoC UART C-Firmware UVM Test...", UVM_LOW)
    `uvm_info("SOC_TEST", "Booting core from ICCM (loaded with .hex)...", UVM_LOW)
    `uvm_info("SOC_TEST", "=================================================", UVM_LOW)

    // Wait until both expected UART frames ('A' and 'B') are processed by scoreboard
    fork
      begin
        wait(env.sb.match_count == 2);
        #10000;
        `uvm_info("SOC_TEST", "All expected UART bytes successfully matched in UVM Scoreboard!", UVM_LOW)
      end
      begin
        #30000000; // 30 ms timeout
        `uvm_error("SOC_TIMEOUT", "Simulation timed out waiting for UART transactions from SoC!")
      end
    join_any

    #5000;
    phase.drop_objection(this);
  endtask : run_phase

  virtual function void report_phase(uvm_phase phase);
    uvm_report_server svr;
    super.report_phase(phase);
    svr = uvm_report_server::get_server();

    if (svr.get_severity_count(UVM_FATAL) == 0 && svr.get_severity_count(UVM_ERROR) == 0 && env.sb.match_count == 2) begin
      `uvm_info("TEST_STATUS", "*****************************************************************", UVM_NONE)
      `uvm_info("TEST_STATUS", "**   >>> [PASSED] ALL UART SOC C-TEST UVM CHECKS COMPLETED <<<  **", UVM_NONE)
      `uvm_info("TEST_STATUS", "**   CPU loaded .hex via $readmemh, verified on AXI & UART VIP **", UVM_NONE)
      `uvm_info("TEST_STATUS", "*****************************************************************", UVM_NONE)
    end else begin
      `uvm_info("TEST_STATUS", "*****************************************************************", UVM_NONE)
      `uvm_info("TEST_STATUS", "**   >>> [FAILED] ERRORS DETECTED IN SOC UART UVM TEST <<<     **", UVM_NONE)
      `uvm_info("TEST_STATUS", "*****************************************************************", UVM_NONE)
    end
  endfunction : report_phase

endclass : soc_uart_test

`endif // SOC_UART_TEST_SV
