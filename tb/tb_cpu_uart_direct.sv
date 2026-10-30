// ============================================================================
// File    : tb_cpu_uart_direct.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Direct CPU-to-UART testbench (bypass interconnect)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_uart_direct;

    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    logic uart_tx;
    logic uart_rx;
    logic uart_irq;

    assign uart_rx = uart_tx;

    soc_top_direct #(
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

    // UART RX capture
    localparam integer BAUD_DIV = 50;
    localparam real      CLK_PERIOD_NS = 10.0;
    localparam real      BIT_TIME_NS = (BAUD_DIV + 1) * CLK_PERIOD_NS;

    logic [7:0] captured_bytes [0:15];
    integer     captured_count;
    logic [7:0] captured_byte;
    logic       capture_valid;
    integer     errors;

    initial begin
        captured_count = 0;
        captured_byte  = 8'h00;
        capture_valid  = 1'b0;
        errors         = 0;
    end

    logic rx_active;
    integer bit_idx;
    logic [7:0] rx_shift;

    always @(negedge uart_tx) begin
        if (!rst) begin
            rx_active = 1'b1;
            bit_idx   = 0;
            rx_shift  = 8'h00;
            #(BIT_TIME_NS / 2);
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                #(BIT_TIME_NS);
                rx_shift[bit_idx] = uart_tx;
            end
            #(BIT_TIME_NS);
            captured_bytes[captured_count] = rx_shift;
            captured_count = captured_count + 1;
            captured_byte  = rx_shift;
            capture_valid  = 1'b1;
            #1 capture_valid = 1'b0;
            rx_active = 1'b0;
        end
    end

    initial begin
        $dumpfile("cpu_uart_direct.vcd");
        $dumpvars(0, tb_cpu_uart_direct);

        rst = 1'b1;
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (10) @(posedge clk);

        $display("\n==============================================");
        $display("  Direct CPU-to-UART Test");
        $display("==============================================\n");

        repeat (50000) @(posedge clk);

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

    initial begin
        #100_000_000;
        $display("\n[WATCHDOG] Simulation timed out!");
        $finish;
    end

endmodule