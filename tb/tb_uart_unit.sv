// =============================================================================
// File    : tb_uart_unit.sv
// Project : RISC-V SoC Honours
// Purpose : Standalone self-checking testbench for UART Controller
//           (Testing UART Write and Read individually WITHOUT AXI and WITHOUT Core)
//
// Verification Scope:
//   PART 1: INDIVIDUAL UART WRITE (TRANSMITTER) TESTS
//           (No RX activity; verifying parallel write -> serial TX framing)
//           - Test W1: 8N1 write with alternating byte 0x55 (01010101)
//           - Test W2: 8N1 write with alternating byte 0xAA (10101010)
//           - Test W3: 8N1 write with boundary byte 0x00 (all zeros)
//           - Test W4: 8N1 write with boundary byte 0xFF (all ones)
//           - Test W5: 8N1 write with ASCII character 0x41 ('A')
//           - Test W6: 8N1 back-to-back writes with ASCII string "HELLO"
//           - Test W7: Odd Parity mode (8O1) write with byte 0x3C
//           - Test W8: Even Parity mode (8E1) write with byte 0x3C
//           - Test W9: 2 Stop bits mode (8N2) write with byte 0x96
//           - Test W10: Custom baud divisor (div=20) write with byte 0x7E
//
//   PART 2: INDIVIDUAL UART READ (RECEIVER) TESTS
//           (No TX activity; verifying serial RX framing -> parallel read)
//           - Test R1: 8N1 read with alternating byte 0x55
//           - Test R2: 8N1 read with alternating byte 0xAA
//           - Test R3: 8N1 read with boundary byte 0x00
//           - Test R4: 8N1 read with boundary byte 0xFF
//           - Test R5: 8N1 read with ASCII character 0x4B ('K')
//           - Test R6: 8N1 consecutive reads with string "WORLD"
//           - Test R7: Odd Parity mode (8O1) read with byte 0xC3
//           - Test R8: Even Parity mode (8E1) read with byte 0xC3
//           - Test R9: 2 Stop bits mode (8N2) read with byte 0x69
//           - Test R10: Custom baud divisor (div=20) read with byte 0xE7
//
//   PART 3: UART FULL-DUPLEX / LOOPBACK CONCURRENT TESTS
//           - Test L1: Single byte loopback 0xA5 (tx_o tied to rx_i)
//           - Test L2: Single byte loopback 0x5A
//           - Test L3: Single byte loopback 0x55
//           - Test L4: Single byte loopback 0xAA
//           - Test L5: Single byte loopback '!' (0x21)
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_uart_unit;

  // ---------------------------------------------------------------------------
  // Parameters & Constants
  // ---------------------------------------------------------------------------
  localparam integer DATA_UART  = 8;
  localparam integer DATA_SIZE  = 32;
  localparam integer DIV_SIZE   = 16;
  localparam integer CLK_PERIOD = 10;   // 100 MHz clock (10 ns period)

  // ---------------------------------------------------------------------------
  // Clock & Reset Signals
  // ---------------------------------------------------------------------------
  logic clk;
  logic rstn;

  initial clk = 1'b0;
  always #(CLK_PERIOD / 2) clk = ~clk;

  // ---------------------------------------------------------------------------
  // DUT Signals (Direct UART Controller Interface - NO AXI, NO Core)
  // ---------------------------------------------------------------------------
  logic                  uart_en;
  logic                  uart_stop_bits;
  logic                  uart_parity_bit;
  logic                  uart_parity_bit_mode;
  logic [DIV_SIZE-1:0]   uart_baudrate_div;

  logic                  uart_rx;
  wire                   uart_tx;
  wire  [DATA_UART-1:0]  rx_data;
  wire                   rx_push;

  logic                  tx_load;
  logic [DATA_UART-1:0]  tx_data;
  wire                   tx_pull;

  // ---------------------------------------------------------------------------
  // Instantiate DUT: uart_controller (Native non-AXI UART Core)
  // ---------------------------------------------------------------------------
  uart_controller #(
      .DATA_UART(DATA_UART),
      .DATA_SIZE(DATA_SIZE),
      .DIV_SIZE (DIV_SIZE)
  ) dut (
      .clk_i                 (clk),
      .rstn_i                (rstn),
      .uart_en_i             (uart_en),
      .uart_stop_bits_i      (uart_stop_bits),
      .uart_parity_bit_i     (uart_parity_bit),
      .uart_parity_bit_mode_i(uart_parity_bit_mode),
      .uart_baudrate_div_i   (uart_baudrate_div),
      .uart_rx_i             (uart_rx),
      .uart_tx_o             (uart_tx),
      .rx_data_o             (rx_data),
      .rx_push_o             (rx_push),
      .tx_load_i             (tx_load),
      .tx_data_i             (tx_data),
      .tx_pull_o             (tx_pull)
  );

  // ---------------------------------------------------------------------------
  // Testbench Tracking Variables
  // ---------------------------------------------------------------------------
  integer tx_pass_count = 0;
  integer tx_fail_count = 0;
  integer rx_pass_count = 0;
  integer rx_fail_count = 0;
  integer loop_pass_count = 0;
  integer loop_fail_count = 0;

  // ---------------------------------------------------------------------------
  // Waveform Dumping (Verdi FSDB & VCD Fallback)
  // ---------------------------------------------------------------------------
  initial begin
`ifdef FSDB
    $fsdbDumpfile("dump.fsdb");
    $fsdbDumpvars(0, tb_uart_unit);
`else
    $dumpfile("uart_unit.vcd");
    $dumpvars(0, tb_uart_unit);
`endif
  end

  // ---------------------------------------------------------------------------
  // Helper: Hardware Reset Task
  // ---------------------------------------------------------------------------
  task reset_dut();
    begin
      rstn                 <= 1'b0;
      uart_en              <= 1'b0;
      uart_stop_bits       <= 1'b0;
      uart_parity_bit      <= 1'b0;
      uart_parity_bit_mode <= 1'b0;
      uart_baudrate_div    <= 16'd10;
      uart_rx              <= 1'b1;  // UART serial line idles high
      tx_load              <= 1'b0;
      tx_data              <= 8'h00;
      repeat (10) @(posedge clk);
      rstn                 <= 1'b1;
      repeat (5) @(posedge clk);
      uart_en              <= 1'b1;
      repeat (5) @(posedge clk);
    end
  endtask

  // ---------------------------------------------------------------------------
  // Helper: Configure UART
  // ---------------------------------------------------------------------------
  task set_config(
      input [DIV_SIZE-1:0] div,
      input bit            parity_en,
      input bit            parity_mode,
      input bit            stop_bits
  );
    begin
      @(posedge clk);
      uart_baudrate_div    <= div;
      uart_parity_bit      <= parity_en;
      uart_parity_bit_mode <= parity_mode;
      uart_stop_bits       <= stop_bits;
      uart_en              <= 1'b1;
      repeat (2) @(posedge clk);
    end
  endtask

  // ===========================================================================
  // TASK: test_uart_write
  // Drives a byte to DUT's parallel write interface (tx_data, tx_load),
  // monitors uart_tx serially, and verifies start bit, data bits, parity, stop bit.
  // ===========================================================================
  task test_uart_write(
      input [7:0] byte_val,
      input string test_id,
      input string desc
  );
    integer i;
    logic [7:0] captured_byte;
    logic captured_parity;
    logic expected_parity;
    time bit_time_ns;
    integer div_val;
    bit par_en;
    bit par_mode;
    bit stop_mode;
    bit test_ok;

    begin
      div_val     = uart_baudrate_div;
      par_en      = uart_parity_bit;
      par_mode    = uart_parity_bit_mode;
      stop_mode   = uart_stop_bits;
      bit_time_ns = (div_val + 1) * CLK_PERIOD; // RTL transmitter bit duration
      test_ok     = 1'b1;

      // Fork parallel write driver and serial tx monitor
      fork
        // Thread 1: Parallel Write Stimulus
        begin
          @(posedge clk);
          tx_data <= byte_val;
          tx_load <= 1'b1;
          // Wait for DUT to assert tx_pull handshake
          @(posedge clk);
          while (!tx_pull) @(posedge clk);
          tx_load <= 1'b0;
        end

        // Thread 2: Serial TX Output Monitor & Verifier
        begin
          // Wait for Start bit falling edge
          @(negedge uart_tx);
          // Sample in the middle of the start bit
          #(bit_time_ns / 2);
          if (uart_tx !== 1'b0) begin
            $display("  [FAIL] %s - Start bit error! Expected 0, got %b", test_id, uart_tx);
            test_ok = 1'b0;
          end

          // Sample 8 Data Bits (LSB first) at mid-bit
          for (i = 0; i < 8; i = i + 1) begin
            #(bit_time_ns);
            captured_byte[i] = uart_tx;
          end

          // Verify Parity Bit if enabled
          if (par_en) begin
            #(bit_time_ns);
            captured_parity = uart_tx;
            expected_parity = (par_mode == 1'b0) ? ~(^byte_val) : (^byte_val);
            if (captured_parity !== expected_parity) begin
              $display("  [FAIL] %s - Parity bit mismatch! Expected %b, got %b (mode=%0d)",
                       test_id, expected_parity, captured_parity, par_mode);
              test_ok = 1'b0;
            end
          end

          // Verify Stop Bit 1
          #(bit_time_ns);
          if (uart_tx !== 1'b1) begin
            $display("  [FAIL] %s - Stop bit 1 error! Expected 1, got %b", test_id, uart_tx);
            test_ok = 1'b0;
          end

          // Verify Stop Bit 2 if configured
          if (stop_mode) begin
            #(bit_time_ns);
            if (uart_tx !== 1'b1) begin
              $display("  [FAIL] %s - Stop bit 2 error! Expected 1, got %b", test_id, uart_tx);
              test_ok = 1'b0;
            end
          end

          // Final byte check
          if (captured_byte !== byte_val) begin
            $display("  [FAIL] %s - Data mismatch! Sent=0x%02X, Captured=0x%02X (%s)",
                     test_id, byte_val, captured_byte, desc);
            test_ok = 1'b0;
          end
        end
      join

      // Wait for transmitter to return to idle
      #(bit_time_ns * 2);

      if (test_ok) begin
        $display("  [PASS] %s | WRITE TX: 0x%02X ('%c') | Div=%0d Parity=%0d Stop=%0d | %s",
                 test_id, byte_val, (byte_val >= 32 && byte_val <= 126) ? byte_val : ".",
                 div_val, par_en, stop_mode + 1, desc);
        tx_pass_count = tx_pass_count + 1;
      end else begin
        tx_fail_count = tx_fail_count + 1;
      end
    end
  endtask

  // ===========================================================================
  // TASK: test_uart_read
  // Injects serial bits into uart_rx (Start bit, 8 data bits, parity, stop bit),
  // monitors DUT's parallel rx_push and rx_data, and validates received data.
  // ===========================================================================
  task test_uart_read(
      input [7:0] byte_val,
      input string test_id,
      input string desc
  );
    integer i;
    time bit_time_ns;
    integer div_val;
    bit par_en;
    bit par_mode;
    bit stop_mode;
    bit par_bit_val;
    bit test_ok;
    logic [7:0] received_byte;
    bit push_seen;

    begin
      div_val     = uart_baudrate_div;
      par_en      = uart_parity_bit;
      par_mode    = uart_parity_bit_mode;
      stop_mode   = uart_stop_bits;
      bit_time_ns = div_val * CLK_PERIOD; // RTL receiver sampling interval
      par_bit_val = (par_mode == 1'b0) ? ~(^byte_val) : (^byte_val);
      test_ok     = 1'b1;
      push_seen   = 1'b0;

      fork
        // Thread 1: Serial RX Bit Stream Injector
        begin
          // Start bit (active low)
          uart_rx <= 1'b0;
          #(bit_time_ns);

          // 8 Data Bits (LSB first)
          for (i = 0; i < 8; i = i + 1) begin
            uart_rx <= byte_val[i];
            #(bit_time_ns);
          end

          // Parity Bit if enabled
          if (par_en) begin
            uart_rx <= par_bit_val;
            #(bit_time_ns);
          end

          // Stop Bit 1 (high)
          uart_rx <= 1'b1;
          #(bit_time_ns);

          // Stop Bit 2 if enabled
          if (stop_mode) begin
            #(bit_time_ns);
          end

          // Return to idle line
          uart_rx <= 1'b1;
        end

        // Thread 2: Monitor rx_push and rx_data from DUT
        begin
          fork
            begin
              @(posedge rx_push);
              received_byte = rx_data;
              push_seen = 1'b1;
            end
            begin
              #(bit_time_ns * 15);
              if (!push_seen) begin
                $display("  [FAIL] %s - TIMEOUT waiting for rx_push!", test_id);
                test_ok = 1'b0;
              end
            end
          join_any
          disable fork; // Cancel remaining timeout thread
        end
      join

      // Verify received byte
      if (push_seen) begin
        if (received_byte !== byte_val) begin
          $display("  [FAIL] %s - Data mismatch! Injected=0x%02X, Received=0x%02X (%s)",
                   test_id, byte_val, received_byte, desc);
          test_ok = 1'b0;
        end
      end else begin
        test_ok = 1'b0;
      end

      #(bit_time_ns * 2);

      if (test_ok) begin
        $display("  [PASS] %s | READ  RX: 0x%02X ('%c') | Div=%0d Parity=%0d Stop=%0d | %s",
                 test_id, byte_val, (byte_val >= 32 && byte_val <= 126) ? byte_val : ".",
                 div_val, par_en, stop_mode + 1, desc);
        rx_pass_count = rx_pass_count + 1;
      end else begin
        rx_fail_count = rx_fail_count + 1;
      end
    end
  endtask

  // ===========================================================================
  // TASK: test_uart_loopback
  // Connects uart_tx directly to uart_rx and performs write-read loopback.
  // ===========================================================================
  task test_uart_loopback(
      input [7:0] byte_val,
      input string test_id,
      input string desc
  );
    time bit_time_ns;
    integer div_val;
    bit test_ok;
    logic [7:0] received_byte;
    bit push_seen;

    begin
      div_val     = uart_baudrate_div;
      bit_time_ns = (div_val + 1) * CLK_PERIOD;
      test_ok     = 1'b1;
      push_seen   = 1'b0;

      fork
        // Thread 1: Wire TX output directly to RX input while test is running
        begin
          while (!push_seen && test_ok) begin
            uart_rx <= uart_tx;
            #1;
          end
        end

        // Thread 2: Parallel Write to TX
        begin
          @(posedge clk);
          tx_data <= byte_val;
          tx_load <= 1'b1;
          @(posedge clk);
          while (!tx_pull) @(posedge clk);
          tx_load <= 1'b0;
        end

        // Thread 3: Parallel Read from RX
        begin
          fork
            begin
              @(posedge rx_push);
              received_byte = rx_data;
              push_seen = 1'b1;
            end
            begin
              #(bit_time_ns * 30);
              if (!push_seen) begin
                $display("  [FAIL] %s - Loopback TIMEOUT waiting for rx_push!", test_id);
                test_ok = 1'b0;
              end
            end
          join_any
          disable fork;
        end
      join

      uart_rx <= 1'b1;

      if (push_seen) begin
        if (received_byte !== byte_val) begin
          $display("  [FAIL] %s - Loopback mismatch! Written=0x%02X, Read=0x%02X (%s)",
                   test_id, byte_val, received_byte, desc);
          test_ok = 1'b0;
        end
      end else begin
        test_ok = 1'b0;
      end

      #(bit_time_ns * 3);

      if (test_ok) begin
        $display("  [PASS] %s | LOOPBACK: 0x%02X ('%c') | %s",
                 test_id, byte_val, (byte_val >= 32 && byte_val <= 126) ? byte_val : ".", desc);
        loop_pass_count = loop_pass_count + 1;
      end else begin
        loop_fail_count = loop_fail_count + 1;
      end
    end
  endtask

  // ===========================================================================
  // MAIN TESTBENCH EXECUTION
  // ===========================================================================
  initial begin
    $display("");
    $display("================================================================================");
    $display("       UART UNIT TESTBENCH: INDIVIDUAL WRITE & READ VERIFICATION                ");
    $display("       (Pure Hardware Verification: NO AXI Bus, NO CPU Core, JUST TB)           ");
    $display("================================================================================");
    $display("");

    // Initial reset
    reset_dut();

    // =========================================================================
    // PART 1: INDIVIDUAL UART WRITE (TRANSMITTER) TESTS
    // (Serial RX line is idle high; testing purely the TX path)
    // =========================================================================
    $display("--- PART 1: INDIVIDUAL UART WRITE (TX) TESTS ---");
    uart_rx <= 1'b1; // Idle RX line

    // Configuration 1: 8N1 (Baud Div = 10, Parity = Disabled, Stop bits = 1)
    set_config(16'd10, 1'b0, 1'b0, 1'b0);

    test_uart_write(8'h55, "TEST_W1 ", "Alternating bit pattern 01010101");
    test_uart_write(8'hAA, "TEST_W2 ", "Alternating bit pattern 10101010");
    test_uart_write(8'h00, "TEST_W3 ", "All zeros byte 0x00");
    test_uart_write(8'hFF, "TEST_W4 ", "All ones byte 0xFF");
    test_uart_write(8'h41, "TEST_W5 ", "ASCII character 'A'");

    // Back-to-back writes ("HELLO")
    test_uart_write(8'h48, "TEST_W6a", "Burst write 'H'");
    test_uart_write(8'h45, "TEST_W6b", "Burst write 'E'");
    test_uart_write(8'h4C, "TEST_W6c", "Burst write 'L'");
    test_uart_write(8'h4C, "TEST_W6d", "Burst write 'L'");
    test_uart_write(8'h4F, "TEST_W6e", "Burst write 'O'");

    // Configuration 2: 8O1 (Odd Parity enabled, mode=0, Stop bits = 1)
    set_config(16'd10, 1'b1, 1'b0, 1'b0);
    test_uart_write(8'h3C, "TEST_W7 ", "Odd Parity (8O1) verification");

    // Configuration 3: 8E1 (Even Parity enabled, mode=1, Stop bits = 1)
    set_config(16'd10, 1'b1, 1'b1, 1'b0);
    test_uart_write(8'h3C, "TEST_W8 ", "Even Parity (8E1) verification");

    // Configuration 4: 8N2 (2 Stop bits, No parity)
    set_config(16'd10, 1'b0, 1'b0, 1'b1);
    test_uart_write(8'h96, "TEST_W9 ", "2 Stop bits (8N2) verification");

    // Configuration 5: Custom Baud Divisor = 20
    set_config(16'd20, 1'b0, 1'b0, 1'b0);
    test_uart_write(8'h7E, "TEST_W10", "Custom baud divisor (div=20) verification");

    $display("");

    // =========================================================================
    // PART 2: INDIVIDUAL UART READ (RECEIVER) TESTS
    // (TX interface is quiescent; testing purely the RX path)
    // =========================================================================
    $display("--- PART 2: INDIVIDUAL UART READ (RX) TESTS ---");
    tx_load <= 1'b0; // Quiescent TX interface

    // Configuration 1: 8N1 (Baud Div = 10, Parity = Disabled, Stop bits = 1)
    set_config(16'd10, 1'b0, 1'b0, 1'b0);

    test_uart_read(8'h55, "TEST_R1 ", "Alternating bit pattern 01010101");
    test_uart_read(8'hAA, "TEST_R2 ", "Alternating bit pattern 10101010");
    test_uart_read(8'h00, "TEST_R3 ", "All zeros byte 0x00");
    test_uart_read(8'hFF, "TEST_R4 ", "All ones byte 0xFF");
    test_uart_read(8'h4B, "TEST_R5 ", "ASCII character 'K'");

    // Consecutive reads ("WORLD") with varying inter-byte delays
    test_uart_read(8'h57, "TEST_R6a", "Consecutive read 'W'");
    #200;
    test_uart_read(8'h4F, "TEST_R6b", "Consecutive read 'O'");
    #500;
    test_uart_read(8'h52, "TEST_R6c", "Consecutive read 'R'");
    #100;
    test_uart_read(8'h4C, "TEST_R6d", "Consecutive read 'L'");
    #300;
    test_uart_read(8'h44, "TEST_R6e", "Consecutive read 'D'");

    // Configuration 2: 8O1 (Odd Parity enabled, mode=0, Stop bits = 1)
    set_config(16'd10, 1'b1, 1'b0, 1'b0);
    test_uart_read(8'hC3, "TEST_R7 ", "Odd Parity (8O1) reception");

    // Configuration 3: 8E1 (Even Parity enabled, mode=1, Stop bits = 1)
    set_config(16'd10, 1'b1, 1'b1, 1'b0);
    test_uart_read(8'hC3, "TEST_R8 ", "Even Parity (8E1) reception");

    // Configuration 4: 8N2 (2 Stop bits, No parity)
    set_config(16'd10, 1'b0, 1'b0, 1'b1);
    test_uart_read(8'h69, "TEST_R9 ", "2 Stop bits (8N2) reception");

    // Configuration 5: Custom Baud Divisor = 20
    set_config(16'd20, 1'b0, 1'b0, 1'b0);
    test_uart_read(8'hE7, "TEST_R10", "Custom baud divisor (div=20) reception");

    $display("");

    // =========================================================================
    // PART 3: UART FULL-DUPLEX / LOOPBACK CONCURRENT TESTS
    // (Validates write and read operate simultaneously without interference)
    // =========================================================================
    $display("--- PART 3: UART LOOPBACK CONCURRENT TESTS ---");
    // Use div=50 for loopback to keep baud clock discrepancy under 2%
    set_config(16'd50, 1'b0, 1'b0, 1'b0);

    test_uart_loopback(8'hA5, "TEST_L1 ", "Loopback single byte 0xA5");
    test_uart_loopback(8'h5A, "TEST_L2 ", "Loopback single byte 0x5A");
    test_uart_loopback(8'h55, "TEST_L3 ", "Loopback byte 0x55");
    test_uart_loopback(8'hAA, "TEST_L4 ", "Loopback byte 0xAA");
    test_uart_loopback(8'h21, "TEST_L5 ", "Loopback ASCII character '!'");

    $display("");
    $display("================================================================================");
    $display("                       FINAL UART VERIFICATION SUMMARY                          ");
    $display("================================================================================");
    $display("  INDIVIDUAL WRITE (TX) TESTS : %0d Passed, %0d Failed", tx_pass_count, tx_fail_count);
    $display("  INDIVIDUAL READ  (RX) TESTS : %0d Passed, %0d Failed", rx_pass_count, rx_fail_count);
    $display("  LOOPBACK DUPLEX       TESTS : %0d Passed, %0d Failed", loop_pass_count, loop_fail_count);
    $display("--------------------------------------------------------------------------------");
    $display("  TOTAL TESTS EXECUTED        : %0d", tx_pass_count + tx_fail_count +
                                                    rx_pass_count + rx_fail_count +
                                                    loop_pass_count + loop_fail_count);
    $display("  TOTAL FAILURES              : %0d", tx_fail_count + rx_fail_count + loop_fail_count);
    $display("================================================================================");

    if (tx_fail_count == 0 && rx_fail_count == 0 && loop_fail_count == 0) begin
      $display("  >>> ALL INDIVIDUAL UART WRITE & READ TESTS PASSED SUCCESSFULLY! <<<");
      $display("================================================================================");
    end else begin
      $display("  >>> SOME TESTS FAILED! PLEASE CHECK THE LOG ABOVE. <<<");
      $display("================================================================================");
      $finish(1);
    end

    $finish(0);
  end

endmodule

`default_nettype wire
