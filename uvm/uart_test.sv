// =============================================================================
// File    : uart_test.sv
// Purpose : Simple UART Test running sequence and verifying status
// =============================================================================

`ifndef UART_TEST_SV
`define UART_TEST_SV

class uart_test extends uvm_test;

  `uvm_component_utils(uart_test)

  uart_env env;

  function new(string name = "uart_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction : new

  virtual function void build_phase(uvm_phase phase);
    super.build_phase(phase);
    env = uart_env::type_id::create("env", this);
  endfunction : build_phase

  virtual task run_phase(uvm_phase phase);
    uart_sequence seq;
    phase.raise_objection(this);

    `uvm_info("TEST", "Starting UART test...", UVM_LOW)
    seq = uart_sequence::type_id::create("seq");
    seq.start(env.agent.seqr);

    #1000;
    `uvm_info("TEST", "UART test sequence finished.", UVM_LOW)
    phase.drop_objection(this);
  endtask : run_phase

  virtual function void report_phase(uvm_phase phase);
    uvm_report_server svr;
    super.report_phase(phase);
    svr = uvm_report_server::get_server();

    if (svr.get_severity_count(UVM_FATAL) == 0 && svr.get_severity_count(UVM_ERROR) == 0) begin
      `uvm_info("TEST_STATUS", "*************************************************", UVM_NONE)
      `uvm_info("TEST_STATUS", "**   >>> [PASSED] ALL UART TESTS COMPLETED <<<  **", UVM_NONE)
      `uvm_info("TEST_STATUS", "**   SET in tb_top and GET in Driver & Monitor **", UVM_NONE)
      `uvm_info("TEST_STATUS", "*************************************************", UVM_NONE)
    end else begin
      `uvm_info("TEST_STATUS", "*************************************************", UVM_NONE)
      `uvm_info("TEST_STATUS", "**   >>> [FAILED] ERRORS DETECTED IN UART TEST **", UVM_NONE)
      `uvm_info("TEST_STATUS", "*************************************************", UVM_NONE)
    end
  endfunction : report_phase

endclass : uart_test

`endif // UART_TEST_SV
