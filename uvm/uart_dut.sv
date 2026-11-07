// =============================================================================
// File    : uart_dut.sv
// Purpose : Standalone UART DUT with ONLY uart_rx_i and uart_tx_o
//           (All internal parallel lines are fully enclosed inside the module)
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module uart_dut (
    input  wire clk,
    input  wire rstn,
    input  wire uart_rx_i,
    output wire uart_tx_o
);

  // Internal parallel wires (hidden from user)
  wire [7:0] rx_data_int;
  wire       rx_push_int;
  wire       tx_pull_int;

  reg        tx_load_reg;
  reg  [7:0] tx_data_reg;

  // Echo received bytes back to transmitter
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      tx_load_reg <= 1'b0;
      tx_data_reg <= 8'h00;
    end else begin
      if (rx_push_int) begin
        tx_data_reg <= rx_data_int;
        tx_load_reg <= 1'b1;
      end else if (tx_pull_int) begin
        tx_load_reg <= 1'b0;
      end
    end
  end

  // Hardware UART Core (Configured for 8N1, div=10)
  uart_controller #(
      .DATA_UART(8),
      .DATA_SIZE(32),
      .DIV_SIZE (16)
  ) u_uart_core (
      .clk_i                 (clk),
      .rstn_i                (rstn),
      .uart_en_i             (1'b1),
      .uart_stop_bits_i      (1'b0),        // 1 stop bit
      .uart_parity_bit_i     (1'b0),        // No parity
      .uart_parity_bit_mode_i(1'b0),
      .uart_baudrate_div_i   (16'd10),      // Baud divisor = 10
      .uart_rx_i             (uart_rx_i),   // Serial RX
      .uart_tx_o             (uart_tx_o),   // Serial TX
      .rx_data_o             (rx_data_int),
      .rx_push_o             (rx_push_int),
      .tx_load_i             (tx_load_reg),
      .tx_data_i             (tx_data_reg),
      .tx_pull_o             (tx_pull_int)
  );

endmodule

`default_nettype wire
