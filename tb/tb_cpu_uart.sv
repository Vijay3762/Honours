// ============================================================================
// File    : tb_cpu_uart.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Testbench — VeeR EL2 CPU executing UART read/write test
//           UART tx is looped back to rx internally; testbench captures
//           transmitted characters and verifies 'A' then 'B' appear.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_uart;

    // --------------------------------------------------------------
    // Clock / reset
    // --------------------------------------------------------------
    logic clk;
    logic rst;           // active-high

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    // --------------------------------------------------------------
    // DUT: SoC top (VeeR + interconnect + UART bridge + UART + mem)
    // --------------------------------------------------------------
    logic uart_tx;
    logic uart_rx;
    logic uart_irq;

    // Internal loopback: tx -> rx
    assign uart_rx = uart_tx;

    soc_top #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8)
    ) dut (
        .clk         (clk),
        .rst         (rst),
        .uart_tx_o   (uart_tx),
        .uart_rx_i   (uart_rx),
        .uart_irq_o  (uart_irq)
    );

    // --------------------------------------------------------------
    // UART RX capture: sample uart_tx at baud rate to reconstruct bytes
    // --------------------------------------------------------------
    // BAUD_DIV = 50, CLK_PERIOD = 10ns => bit time ~ (50+1)*10ns = 510ns
    localparam integer BAUD_DIV = 50;
    localparam real      CLK_PERIOD_NS = 10.0;
    localparam real      BIT_TIME_NS = (BAUD_DIV + 1) * CLK_PERIOD_NS;  // ~510 ns
    localparam real      FRAME_TIME_NS = 10 * BIT_TIME_NS;               // start+8data+stop

    logic [7:0] captured_byte;
    logic       capture_valid;
    integer     captured_count;
    logic [7:0] captured_bytes [0:15];
    integer     errors;

    initial begin
        captured_count = 0;
        captured_byte  = 8'h00;
        capture_valid  = 1'b0;
        errors         = 0;
    end

    // Simple UART RX: detect start bit, sample data bits at center
    integer bit_idx;
    logic [7:0] rx_shift;
    logic       rx_active;
    real        sample_time;

    always @(negedge uart_tx) begin
        // Start bit detected (falling edge)
        if (!rst) begin
            rx_active = 1'b1;
            bit_idx   = 0;
            rx_shift  = 8'h00;
            // Wait half bit time, then sample at center of each bit
            #(BIT_TIME_NS / 2);
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                #(BIT_TIME_NS);
                rx_shift[bit_idx] = uart_tx;
            end
            #(BIT_TIME_NS);  // wait for stop bit
            // Capture received byte
            captured_bytes[captured_count] = rx_shift;
            captured_count = captured_count + 1;
            captured_byte  = rx_shift;
            capture_valid  = 1'b1;
            #1 capture_valid = 1'b0;
            rx_active = 1'b0;
            $display("[UART RX] Captured byte: 0x%02h (%c)", rx_shift,
                     (rx_shift >= 32 && rx_shift <= 126) ? rx_shift : "?");
        end
    end

    // --------------------------------------------------------------
    // Test control
    // --------------------------------------------------------------
    initial begin
        $dumpfile("cpu_uart_test.vcd");
        $dumpvars(0, tb_cpu_uart);

        rst = 1'b1;
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (10) @(posedge clk);

        $display("\n==============================================");
        $display("  CPU-to-UART Read/Write Test");
        $display("  VeeR EL2 executing UART test firmware");
        $display("==============================================\n");

        // Wait for CPU to transmit 'A' and 'B' via UART
        // Each UART frame at BAUD_DIV=50, 100MHz clk: ~510ns per bit
        // Full frame ~5.1us. Two frames + config ~ 50us. Give 10ms.
        repeat (20000) @(posedge clk);

        // ---- Check results ----
        $display("\n----------------------------------------------");
        $display("  Captured %0d byte(s) from UART", captured_count);

        if (captured_count >= 1) begin
            if (captured_bytes[0] == 8'h41)
                $display("  [PASS] Byte 0 = 0x41 ('A')");
            else begin
                $display("  [FAIL] Byte 0 = 0x%02h, expected 0x41 ('A')", captured_bytes[0]);
                errors = errors + 1;
            end
        end else begin
            $display("  [FAIL] No bytes captured from UART");
            errors = errors + 1;
        end

        if (captured_count >= 2) begin
            if (captured_bytes[1] == 8'h42)
                $display("  [PASS] Byte 1 = 0x42 ('B')");
            else begin
                $display("  [FAIL] Byte 1 = 0x%02h, expected 0x42 ('B')", captured_bytes[1]);
                errors = errors + 1;
            end
        end else begin
            $display("  [FAIL] Only %0d byte(s) captured, expected 2", captured_count);
            errors = errors + 1;
        end

        $display("----------------------------------------------");
        if (errors == 0) begin
            $display("  *** ALL TESTS PASSED ***");
        end else begin
            $display("  *** %0d TEST(S) FAILED ***", errors);
        end
        $display("==============================================\n");

        #1000;
        $finish;
    end

    // Watchdog timeout
    initial begin
        #50_000_000;   // 50 ms
        $display("\n[WATCHDOG] Simulation timed out!");
        $finish;
    end

endmodule

`default_nettype wire