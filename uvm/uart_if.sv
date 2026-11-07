// =============================================================================
// File    : uart_if.sv
// Purpose : UART Interface with ONLY uart_rx_i and uart_tx_o
// =============================================================================

`ifndef UART_IF_SV
`define UART_IF_SV

interface uart_if (
    input wire clk,
    input wire rstn
);

  logic uart_rx_i; // Serial input into DUT
  logic uart_tx_o; // Serial output from DUT

  initial begin
    uart_rx_i = 1'b1; // Idle line is HIGH
  end

endinterface : uart_if

`endif // UART_IF_SV
