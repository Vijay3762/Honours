// =============================================================================
// File    : uart_if.sv
// Purpose : UART Interface connecting SoC Pins to UVM VIP with Probe Signals
// =============================================================================

`ifndef UART_IF_SV
`define UART_IF_SV

interface uart_if (
    input wire clk,
    input wire rstn
);

  // Physical serial pins
  logic uart_rx_i; // Serial input into DUT
  logic uart_tx_o; // Serial output from DUT

  // ---------------------------------------------------------------------------
  // VIP Verification & Comparison Signals (Visible in Verdi)
  // ---------------------------------------------------------------------------
  logic [7:0]  core_tx_byte;        // Parallel byte transmitted by CPU core into UART IP
  logic [7:0]  vip_captured_byte;   // Actual byte deserialized from uart_tx_o by VIP monitor
  logic [7:0]  vip_expected_byte;   // Expected reference byte in VIP scoreboard
  logic        vip_byte_valid;      // Strobe pulse when a complete frame is received & checked
  logic        vip_match;           // High (1) when vip_captured_byte === vip_expected_byte
  logic [31:0] vip_match_count;     // Tally of verified matching bytes
  logic [31:0] vip_mismatch_count;  // Tally of mismatches (0 on pass)

  initial begin
    vip_captured_byte  = 8'h00;
    vip_expected_byte  = 8'h00;
    vip_byte_valid     = 1'b0;
    vip_match          = 1'b0;
    vip_match_count    = 32'd0;
    vip_mismatch_count = 32'd0;
  end

endinterface : uart_if

`endif // UART_IF_SV
