// =============================================================================
// File    : uart_seq_item.sv
// Purpose : Simple UART Sequence Item
// =============================================================================

`ifndef UART_SEQ_ITEM_SV
`define UART_SEQ_ITEM_SV

class uart_seq_item extends uvm_sequence_item;

  `uvm_object_utils(uart_seq_item)

  rand bit [7:0] data;

  function new(string name = "uart_seq_item");
    super.new(name);
    data = 8'h00;
  endfunction : new

  virtual function string convert2string();
    return $sformatf("data=0x%02X ('%c')", data,
                     (data >= 32 && data <= 126) ? data : ".");
  endfunction : convert2string

endclass : uart_seq_item

`endif // UART_SEQ_ITEM_SV
