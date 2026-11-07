// =============================================================================
// File    : uart_sequence.sv
// Purpose : Simple UART Sequence driving sample characters
// =============================================================================

`ifndef UART_SEQUENCE_SV
`define UART_SEQUENCE_SV

class uart_sequence extends uvm_sequence #(uart_seq_item);

  `uvm_object_utils(uart_sequence)

  function new(string name = "uart_sequence");
    super.new(name);
  endfunction : new

  virtual task body();
    byte test_data[5] = '{8'h41, 8'h42, 8'h43, 8'h44, 8'h45}; // 'A', 'B', 'C', 'D', 'E'
    uart_seq_item item;

    `uvm_info("SEQ", "Starting UART sequence (5 characters: 'A', 'B', 'C', 'D', 'E')", UVM_LOW)

    for (int i = 0; i < 5; i++) begin
      item = uart_seq_item::type_id::create($sformatf("item_%0d", i));
      start_item(item);
      item.data = test_data[i];
      finish_item(item);
      // Wait for echo to complete before sending next byte
      #2000;
    end

    // Additional settling time
    #5000;
    `uvm_info("SEQ", "Completed UART sequence!", UVM_LOW)
  endtask : body

endclass : uart_sequence

`endif // UART_SEQUENCE_SV
