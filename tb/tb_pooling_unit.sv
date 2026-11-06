// =============================================================================
// File    : tb_pooling_unit.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Standalone self-checking testbench for pooling_unit.sv.
//
// Verification scope:
//   1. Reset behavior
//   2. Mode 0: Max-Pooling (Positive, Negative, Mixed, Equal, Argmax index)
//   3. Mode 1: Average-Pooling (Symmetric rounding, signed arithmetic, negative inputs)
//   4. Mode 2: Min-Pooling (Positive, Negative, Mixed, Extremes)
//   5. Mode 3: Bypass Mode (p0 passed directly)
//   6. Module Enable / Disable behavior
//   7. Handshaking & Backpressure (ready_i / ready_o)
//   8. Combinational mode instance (PIPELINED = 0)
//   9. Randomized vector verification against golden model
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_pooling_unit;

    localparam integer DATA_WIDTH = 16;
    localparam integer CLK_PERIOD = 10;

    logic clk;
    logic rst;

    // Clock generator (100 MHz)
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // DUT signals (Pipelined instance)
    logic [1:0]                      mode_i;
    logic                            enable_i;
    logic                            valid_i;
    wire                             ready_o;
    logic signed [DATA_WIDTH-1:0]    p0_i, p1_i, p2_i, p3_i;
    wire                             valid_o;
    logic                            ready_i;
    wire  signed [DATA_WIDTH-1:0]    data_o;
    wire  [1:0]                      max_idx_o;

    // Pipelined DUT instantiation
    pooling_unit #(
        .DATA_WIDTH (DATA_WIDTH),
        .PIPELINED  (1)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .mode_i     (mode_i),
        .enable_i   (enable_i),
        .valid_i    (valid_i),
        .ready_o    (ready_o),
        .p0_i       (p0_i),
        .p1_i       (p1_i),
        .p2_i       (p2_i),
        .p3_i       (p3_i),
        .valid_o    (valid_o),
        .ready_i    (ready_i),
        .data_o     (data_o),
        .max_idx_o  (max_idx_o)
    );

    // Combinational DUT signals & instantiation (PIPELINED = 0)
    wire                             comb_valid_o;
    wire signed [DATA_WIDTH-1:0]     comb_data_o;
    wire [1:0]                       comb_max_idx_o;

    pooling_unit #(
        .DATA_WIDTH (DATA_WIDTH),
        .PIPELINED  (0)
    ) dut_comb (
        .clk        (clk),
        .rst        (rst),
        .mode_i     (mode_i),
        .enable_i   (enable_i),
        .valid_i    (valid_i),
        .ready_o    (),
        .p0_i       (p0_i),
        .p1_i       (p1_i),
        .p2_i       (p2_i),
        .p3_i       (p3_i),
        .valid_o    (comb_valid_o),
        .ready_i    (1'b1),
        .data_o     (comb_data_o),
        .max_idx_o  (comb_max_idx_o)
    );

    // Test tracking counters
    integer pass_count = 0;
    integer fail_count = 0;

    // -------------------------------------------------------------------------
    // Verification Tasks
    // -------------------------------------------------------------------------
    task automatic check(
        input string desc,
        input signed [DATA_WIDTH-1:0] actual_data,
        input signed [DATA_WIDTH-1:0] exp_data,
        input [1:0]                   actual_idx,
        input [1:0]                   exp_idx,
        input check_idx
    );
        logic data_match;
        logic idx_match;
        begin
            data_match = (actual_data === exp_data);
            idx_match  = !check_idx || (actual_idx === exp_idx);

            if (data_match && idx_match) begin
                pass_count = pass_count + 1;
                $display("  [PASS] %-48s | data: %0d (0x%04h)", desc, actual_data, actual_data);
            end else begin
                fail_count = fail_count + 1;
                $display("  [FAIL] %-48s | Got data: %0d (0x%04h), exp: %0d (0x%04h) | Got idx: %0d, exp: %0d",
                         desc, actual_data, actual_data, exp_data, exp_data, actual_idx, exp_idx);
            end
        end
    endtask

    // Golden model calculation function
    function automatic logic signed [DATA_WIDTH-1:0] golden_pool(
        input [1:0] mode,
        input signed [DATA_WIDTH-1:0] p0,
        input signed [DATA_WIDTH-1:0] p1,
        input signed [DATA_WIDTH-1:0] p2,
        input signed [DATA_WIDTH-1:0] p3
    );
        logic signed [DATA_WIDTH-1:0] m;
        logic signed [DATA_WIDTH+2:0] s;
        begin
            case (mode)
                2'b00: begin // MAX
                    m = p0;
                    if (p1 > m) m = p1;
                    if (p2 > m) m = p2;
                    if (p3 > m) m = p3;
                    golden_pool = m;
                end
                2'b01: begin // AVG with symmetric rounding (+2 >> 2)
                    s = (DATA_WIDTH+3)'(p0) + (DATA_WIDTH+3)'(p1) +
                        (DATA_WIDTH+3)'(p2) + (DATA_WIDTH+3)'(p3) + (DATA_WIDTH+3)'(2);
                    golden_pool = s[DATA_WIDTH+1:2];
                end
                2'b10: begin // MIN
                    m = p0;
                    if (p1 < m) m = p1;
                    if (p2 < m) m = p2;
                    if (p3 < m) m = p3;
                    golden_pool = m;
                end
                2'b11: begin // BYPASS
                    golden_pool = p0;
                end
            endcase
        end
    endfunction

    function automatic [1:0] golden_argmax(
        input signed [DATA_WIDTH-1:0] p0,
        input signed [DATA_WIDTH-1:0] p1,
        input signed [DATA_WIDTH-1:0] p2,
        input signed [DATA_WIDTH-1:0] p3
    );
        logic signed [DATA_WIDTH-1:0] max_01, max_23;
        logic [1:0] idx_01, idx_23;
        begin
            max_01 = (p0 >= p1) ? p0 : p1;
            idx_01 = (p0 >= p1) ? 2'd0 : 2'd1;

            max_23 = (p2 >= p3) ? p2 : p3;
            idx_23 = (p2 >= p3) ? 2'd2 : 2'd3;

            golden_argmax = (max_01 >= max_23) ? idx_01 : idx_23;
        end
    endfunction

    // Drive 1 quad through pipelined DUT and check result
    task automatic test_quad(
        input string desc,
        input [1:0] mode,
        input signed [DATA_WIDTH-1:0] p0,
        input signed [DATA_WIDTH-1:0] p1,
        input signed [DATA_WIDTH-1:0] p2,
        input signed [DATA_WIDTH-1:0] p3,
        input check_idx
    );
        logic signed [DATA_WIDTH-1:0] exp_data;
        logic [1:0]                   exp_idx;
        begin
            exp_data = golden_pool(mode, p0, p1, p2, p3);
            exp_idx  = golden_argmax(p0, p1, p2, p3);

            @(posedge clk);
            mode_i   <= mode;
            enable_i <= 1'b1;
            valid_i  <= 1'b1;
            ready_i  <= 1'b1;
            p0_i     <= p0;
            p1_i     <= p1;
            p2_i     <= p2;
            p3_i     <= p3;

            @(posedge clk);
            valid_i  <= 1'b0;

            // Wait for valid_o (pipeline delay = 1 cycle)
            while (!valid_o) @(posedge clk);

            check(desc, data_o, exp_data, max_idx_o, exp_idx, check_idx);

            // Also check combinational output immediately
            if (comb_data_o !== exp_data) begin
                $display("  [FAIL-COMB] %s | Got comb: %0d, exp: %0d", desc, comb_data_o, exp_data);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main Test Stimulus
    // -------------------------------------------------------------------------
    integer iter;
    logic signed [DATA_WIDTH-1:0] rand_p0, rand_p1, rand_p2, rand_p3;

    initial begin
        $display("=================================================================");
        $display(" tb_pooling_unit : Verification of 2x2 Spatial Pooling Engine    ");
        $display("=================================================================");

        // Initialize signals
        mode_i   = 2'b00;
        enable_i = 1'b0;
        valid_i  = 1'b0;
        ready_i  = 1'b1;
        p0_i     = 16'sh0;
        p1_i     = 16'sh0;
        p2_i     = 16'sh0;
        p3_i     = 16'sh0;
        rst      = 1'b1;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        @(posedge clk);

        // --- TEST 1: Reset Behavior ---
        $display("\n--- Test Group 1: Reset State & Defaults ---");
        if (!valid_o && data_o === 16'sh0 && max_idx_o === 2'b00) begin
            pass_count = pass_count + 1;
            $display("  [PASS] Reset defaults verified (valid_o=0, data_o=0, max_idx=0)");
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] Reset failed: valid_o=%0b, data_o=%0h, max_idx=%0b", valid_o, data_o, max_idx_o);
        end

        // --- TEST 2: Mode 0 (Max-Pooling) ---
        $display("\n--- Test Group 2: Mode 0 (Max-Pooling) ---");
        test_quad("MAX: positive values {10, 50, 30, 20}",
                  2'b00, 16'sd10, 16'sd50, 16'sd30, 16'sd20, 1'b1);

        test_quad("MAX: negative values {-100, -20, -50, -5}",
                  2'b00, -16'sd100, -16'sd20, -16'sd50, -16'sd5, 1'b1);

        test_quad("MAX: mixed signs {-300, 250, -10, 0}",
                  2'b00, -16'sd300, 16'sd250, -16'sd10, 16'sd0, 1'b1);

        test_quad("MAX: all equal values {42, 42, 42, 42}",
                  2'b00, 16'sd42, 16'sd42, 16'sd42, 16'sd42, 1'b1);

        test_quad("MAX: extreme positive boundary {+32767, 100, 0, -50}",
                  2'b00, 16'sh7FFF, 16'sd100, 16'sd0, -16'sd50, 1'b1);

        test_quad("MAX: extreme negative boundary {-32768, -32768, -1, -32768}",
                  2'b00, -16'sd32768, -16'sd32768, -16'sd1, -16'sd32768, 1'b1);

        test_quad("MAX: maximum at pixel 0 {99, 10, 20, 30}",
                  2'b00, 16'sd99, 16'sd10, 16'sd20, 16'sd30, 1'b1);

        test_quad("MAX: maximum at pixel 2 {10, 20, 99, 30}",
                  2'b00, 16'sd10, 16'sd20, 16'sd99, 16'sd30, 1'b1);

        test_quad("MAX: maximum at pixel 3 {10, 20, 30, 99}",
                  2'b00, 16'sd10, 16'sd20, 16'sd30, 16'sd99, 1'b1);

        // --- TEST 3: Mode 1 (Average-Pooling) ---
        $display("\n--- Test Group 3: Mode 1 (Average-Pooling with Rounding) ---");
        test_quad("AVG: exact division {4, 4, 4, 4} -> 4",
                  2'b01, 16'sd4, 16'sd4, 16'sd4, 16'sd4, 1'b0);

        test_quad("AVG: round half up {1, 2, 3, 4} sum=10 -> 3",
                  2'b01, 16'sd1, 16'sd2, 16'sd3, 16'sd4, 1'b0);

        test_quad("AVG: negative exact {-4, -4, -4, -4} -> -4",
                  2'b01, -16'sd4, -16'sd4, -16'sd4, -16'sd4, 1'b0);

        test_quad("AVG: negative with rounding {-1, -2, -3, -4} sum=-10 -> -2",
                  2'b01, -16'sd1, -16'sd2, -16'sd3, -16'sd4, 1'b0);

        test_quad("AVG: balanced zero {100, -100, 200, -200} -> 0",
                  2'b01, 16'sd100, -16'sd100, 16'sd200, -16'sd200, 1'b0);

        test_quad("AVG: large positive {30000, 30000, 30000, 30000} -> 30000",
                  2'b01, 16'sd30000, 16'sd30000, 16'sd30000, 16'sd30000, 1'b0);

        test_quad("AVG: large negative {-30000, -30000, -30000, -30000} -> -30000",
                  2'b01, -16'sd30000, -16'sd30000, -16'sd30000, -16'sd30000, 1'b0);

        // --- TEST 4: Mode 2 (Min-Pooling) ---
        $display("\n--- Test Group 4: Mode 2 (Min-Pooling) ---");
        test_quad("MIN: positive values {10, 50, 30, 20} -> 10",
                  2'b10, 16'sd10, 16'sd50, 16'sd30, 16'sd20, 1'b0);

        test_quad("MIN: negative values {-100, -20, -50, -5} -> -100",
                  2'b10, -16'sd100, -16'sd20, -16'sd50, -16'sd5, 1'b0);

        test_quad("MIN: mixed signs {-300, 250, -10, 0} -> -300",
                  2'b10, -16'sd300, 16'sd250, -16'sd10, 16'sd0, 1'b0);

        test_quad("MIN: extreme boundaries {-32768, +32767, 0, 100} -> -32768",
                  2'b10, -16'sd32768, 16'sh7FFF, 16'sd0, 16'sd100, 1'b0);

        // --- TEST 5: Mode 3 (Bypass Mode) ---
        $display("\n--- Test Group 5: Mode 3 (Bypass Mode) ---");
        test_quad("BYPASS: should pass p0 {777, 10, 20, 30} -> 777",
                  2'b11, 16'sd777, 16'sd10, 16'sd20, 16'sd30, 1'b0);

        test_quad("BYPASS: negative p0 {-999, 100, 200, 300} -> -999",
                  2'b11, -16'sd999, 16'sd100, 16'sd200, 16'sd300, 1'b0);

        // --- TEST 6: Enable / Disable ---
        $display("\n--- Test Group 6: Enable / Disable Behavior ---");
        @(posedge clk);
        enable_i <= 1'b0;
        valid_i  <= 1'b1;
        p0_i     <= 16'sd100;
        p1_i     <= 16'sd200;
        p2_i     <= 16'sd300;
        p3_i     <= 16'sd400;
        @(posedge clk);
        valid_i  <= 1'b0;
        @(posedge clk);
        #1;
        if (!valid_o && data_o === 16'sh0) begin
            pass_count = pass_count + 1;
            $display("  [PASS] Module disable suppresses output correctly");
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] Module disable failed: valid_o=%0b, data_o=%0h", valid_o, data_o);
        end
        enable_i <= 1'b1;

        // --- TEST 7: Backpressure (ready_i) ---
        $display("\n--- Test Group 7: Handshake & Backpressure Throttling ---");
        @(posedge clk);
        ready_i  <= 1'b0; // Downstream not ready
        valid_i  <= 1'b1;
        mode_i   <= 2'b00; // MAX
        p0_i     <= 16'sd15;
        p1_i     <= 16'sd45;
        p2_i     <= 16'sd25;
        p3_i     <= 16'sd35;
        @(posedge clk);
        valid_i  <= 1'b0;
        @(posedge clk);
        #1;

        // valid_o should assert with data=45
        if (valid_o && data_o === 16'sd45) begin
            pass_count = pass_count + 1;
            $display("  [PASS] valid_o asserted while ready_i is low");
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] valid_o not asserted properly under backpressure");
        end

        // Wait another cycle while ready_i remains 0; data_o and valid_o must remain stable
        @(posedge clk);
        #1;
        if (valid_o && data_o === 16'sd45) begin
            pass_count = pass_count + 1;
            $display("  [PASS] Output remains held stable during backpressure stall");
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] Output corrupted during stall");
        end

        // Release backpressure
        ready_i <= 1'b1;
        @(posedge clk);
        #1;
        if (!valid_o) begin
            pass_count = pass_count + 1;
            $display("  [PASS] valid_o cleared once ready_i accepted the item");
        end else begin
            fail_count = fail_count + 1;
            $display("  [FAIL] valid_o did not clear after acceptance");
        end

        // --- TEST 8: Randomized Stress Testing ---
        $display("\n--- Test Group 8: Randomized Stress Vectors (100 quads across modes) ---");
        for (iter = 0; iter < 100; iter = iter + 1) begin
            rand_p0 = $random % 32000;
            rand_p1 = $random % 32000;
            rand_p2 = $random % 32000;
            rand_p3 = $random % 32000;

            test_quad($sformatf("Random quad #%0d [mode %0d]", iter, iter % 4),
                      iter[1:0], rand_p0, rand_p1, rand_p2, rand_p3, (iter[1:0] == 2'b00));
        end

        // --- Summary ---
        $display("\n=================================================================");
        $display(" tb_pooling_unit SUMMARY : PASSED = %0d, FAILED = %0d", pass_count, fail_count);
        $display("=================================================================");

        if (fail_count == 0) begin
            $display(">>> ALL tb_pooling_unit CHECKS PASSED SUCCESSFULLY! <<<\n");
        end else begin
            $display(">>> VERIFICATION FAILED with %0d errors! <<<\n", fail_count);
        end

        $finish;
    end

endmodule

`default_nettype wire
