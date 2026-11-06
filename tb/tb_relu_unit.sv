// =============================================================================
// File    : tb_relu_unit.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Standalone self-checking testbench for relu_unit.sv.
//
// Verification scope:
//   1. Reset defaults
//   2. Positive numbers: pass-through verification
//   3. Negative numbers: zero clamping verification
//   4. Zero input verification
//   5. Extreme boundary values (0x7FFFFFFF, 0x80000000, 0xFFFFFFFF, +1)
//   6. Sparsity flags: zero_flag_o, active_flag_o, clipped_flag_o
//   7. Bypass mode verification
//   8. Upper-bound clipping saturation mode (ReLU_clip)
//   9. Streaming backpressure (ready_i handshaking)
//  10. Random signed stimulus against golden reference model
//  11. Combinational mode (PIPELINE_STAGES = 0)
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_relu_unit;

    localparam integer DATA_WIDTH = 32;
    localparam integer CLK_PERIOD = 10;

    logic clk;
    logic rst;

    // Clock generator (100 MHz)
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // DUT signals (Pipelined instance)
    logic                      enable_i;
    logic                      bypass_i;
    logic                      clip_en_i;
    logic [DATA_WIDTH-1:0]     clip_max_i;
    logic [DATA_WIDTH-1:0]     data_i;
    logic                      valid_i;
    wire                       ready_o;
    wire  [DATA_WIDTH-1:0]     data_o;
    wire                       valid_o;
    logic                      ready_i;
    wire                       zero_flag_o;
    wire                       active_flag_o;
    wire                       clipped_flag_o;

    // Instantiate pipelined DUT (PIPELINE_STAGES = 1)
    relu_unit #(
        .DATA_WIDTH     (DATA_WIDTH),
        .PIPELINE_STAGES(1),
        .CLIPPING_ENABLE(1)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .enable_i       (enable_i),
        .bypass_i       (bypass_i),
        .clip_en_i      (clip_en_i),
        .clip_max_i     (clip_max_i),
        .data_i         (data_i),
        .valid_i        (valid_i),
        .ready_o        (ready_o),
        .data_o         (data_o),
        .valid_o        (valid_o),
        .ready_i        (ready_i),
        .zero_flag_o    (zero_flag_o),
        .active_flag_o  (active_flag_o),
        .clipped_flag_o (clipped_flag_o)
    );

    // Instantiate combinational DUT (PIPELINE_STAGES = 0)
    wire [DATA_WIDTH-1:0]     comb_data_o;
    wire                      comb_valid_o;
    wire                      comb_ready_o;
    wire                      comb_zero_o;
    wire                      comb_active_o;
    wire                      comb_clipped_o;

    relu_unit #(
        .DATA_WIDTH     (DATA_WIDTH),
        .PIPELINE_STAGES(0),
        .CLIPPING_ENABLE(1)
    ) dut_comb (
        .clk            (clk),
        .rst            (rst),
        .enable_i       (enable_i),
        .bypass_i       (bypass_i),
        .clip_en_i      (clip_en_i),
        .clip_max_i     (clip_max_i),
        .data_i         (data_i),
        .valid_i        (valid_i),
        .ready_o        (comb_ready_o),
        .data_o         (comb_data_o),
        .valid_o        (comb_valid_o),
        .ready_i        (ready_i),
        .zero_flag_o    (comb_zero_o),
        .active_flag_o  (comb_active_o),
        .clipped_flag_o (comb_clipped_o)
    );

    // =========================================================================
    // Test tracking & check tasks
    // =========================================================================
    integer pass_count = 0;
    integer fail_count = 0;

    task automatic check(
        input string test_name,
        input [DATA_WIDTH-1:0] actual,
        input [DATA_WIDTH-1:0] expected,
        input logic exp_zero,
        input logic act_zero,
        input logic exp_active,
        input logic act_active,
        input logic exp_clipped,
        input logic act_clipped
    );
        if ((actual === expected) &&
            (exp_zero === act_zero) &&
            (exp_active === act_active) &&
            (exp_clipped === act_clipped)) begin
            $display("[PASS] %s: Data=0x%08X (Exp=0x%08X), Zero=%0d, Active=%0d, Clip=%0d",
                     test_name, actual, expected, act_zero, act_active, act_clipped);
            pass_count++;
        end else begin
            $display("[FAIL] %s: Data=0x%08X (Exp=0x%08X), Zero=%0d (Exp=%0d), Active=%0d (Exp=%0d), Clip=%0d (Exp=%0d)",
                     test_name, actual, expected, act_zero, exp_zero, act_active, exp_active, act_clipped, exp_clipped);
            fail_count++;
        end
    endtask

    // Transaction drive task
    task automatic send_and_check(
        input string desc,
        input signed [DATA_WIDTH-1:0] in_val,
        input signed [DATA_WIDTH-1:0] exp_val,
        input logic exp_z,
        input logic exp_a,
        input logic exp_c
    );
        @(posedge clk);
        while (!ready_o) @(posedge clk);

        data_i  <= in_val;
        valid_i <= 1'b1;

        @(posedge clk);
        data_i  <= '0;
        valid_i <= 1'b0;

        // Sample pipelined output (stage 1)
        while (!valid_o) @(posedge clk);
        check({desc, " [Pipe]"}, data_o, exp_val, exp_z, zero_flag_o, exp_a, active_flag_o, exp_c, clipped_flag_o);
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================
    initial begin
        $display("============================================================");
        $display("  Starting Standalone ReLU Unit Testbench (tb_relu_unit)    ");
        $display("============================================================");

        // Initialization
        rst        = 1'b1;
        enable_i   = 1'b1;
        bypass_i   = 1'b0;
        clip_en_i  = 1'b0;
        clip_max_i = 32'd0;
        data_i     = '0;
        valid_i    = 1'b0;
        ready_i    = 1'b1;

        #(CLK_PERIOD * 3);
        @(negedge clk);
        rst = 1'b0;
        #(CLK_PERIOD * 2);

        // ---------------------------------------------------------------------
        // Test 1: Reset check
        // ---------------------------------------------------------------------
        $display("\n--- Test 1: Reset Defaults ---");
        if (valid_o === 1'b0 && data_o === 32'd0) begin
            $display("[PASS] Reset state clean: valid_o=0, data_o=0");
            pass_count++;
        end else begin
            $display("[FAIL] Reset state dirty: valid_o=%0b, data_o=0x%08X", valid_o, data_o);
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // Test 2: Standard positive numbers (ReLU(x) = x)
        // ---------------------------------------------------------------------
        $display("\n--- Test 2: Positive Numbers Pass-Through ---");
        send_and_check("Positive small (1)",       32'sd1,        32'sd1,        1'b0, 1'b1, 1'b0);
        send_and_check("Positive moderate (42)",   32'sd42,       32'sd42,       1'b0, 1'b1, 1'b0);
        send_and_check("Positive 8-bit max (255)", 32'sd255,      32'sd255,      1'b0, 1'b1, 1'b0);
        send_and_check("Positive 16-bit (32767)",  32'sd32767,    32'sd32767,    1'b0, 1'b1, 1'b0);
        send_and_check("Positive large (1000000)", 32'sd1000000,  32'sd1000000,  1'b0, 1'b1, 1'b0);

        // ---------------------------------------------------------------------
        // Test 3: Standard negative numbers (ReLU(x) = 0)
        // ---------------------------------------------------------------------
        $display("\n--- Test 3: Negative Numbers Zero-Clamping ---");
        send_and_check("Negative small (-1)",      -32'sd1,       32'sd0,        1'b1, 1'b0, 1'b0);
        send_and_check("Negative moderate (-42)",  -32'sd42,      32'sd0,        1'b1, 1'b0, 1'b0);
        send_and_check("Negative 8-bit (-255)",    -32'sd255,     32'sd0,        1'b1, 1'b0, 1'b0);
        send_and_check("Negative 16-bit (-32768)", -32'sd32768,   32'sd0,        1'b1, 1'b0, 1'b0);
        send_and_check("Negative large (-1000000)",-32'sd1000000, 32'sd0,        1'b1, 1'b0, 1'b0);

        // ---------------------------------------------------------------------
        // Test 4: Zero input (ReLU(0) = 0)
        // ---------------------------------------------------------------------
        $display("\n--- Test 4: Zero Input ---");
        send_and_check("Zero input (0)",           32'sd0,        32'sd0,        1'b1, 1'b0, 1'b0);

        // ---------------------------------------------------------------------
        // Test 5: Boundary & Extreme Values
        // ---------------------------------------------------------------------
        $display("\n--- Test 5: Boundary Values ---");
        // Max signed 32-bit: 0x7FFFFFFF (2147483647)
        send_and_check("Max positive (0x7FFFFFFF)", 32'h7FFF_FFFF, 32'h7FFF_FFFF, 1'b0, 1'b1, 1'b0);
        // Min signed 32-bit: 0x80000000 (-2147483648)
        send_and_check("Min negative (0x80000000)", 32'h8000_0000, 32'sd0,        1'b1, 1'b0, 1'b0);
        // Negative one: 0xFFFFFFFF (-1)
        send_and_check("Negative one (0xFFFFFFFF)", 32'hFFFF_FFFF, 32'sd0,        1'b1, 1'b0, 1'b0);

        // ---------------------------------------------------------------------
        // Test 6: Bypass Mode
        // ---------------------------------------------------------------------
        $display("\n--- Test 6: Bypass Mode ---");
        bypass_i = 1'b1;
        send_and_check("Bypass negative (-42)",     -32'sd42,      -32'sd42,      1'b0, 1'b0, 1'b0);
        send_and_check("Bypass positive (+42)",     32'sd42,       32'sd42,      1'b0, 1'b1, 1'b0);
        send_and_check("Bypass min neg (0x80000000)", 32'h8000_0000, 32'h8000_0000, 1'b0, 1'b0, 1'b0);
        bypass_i = 1'b0;

        // ---------------------------------------------------------------------
        // Test 7: Upper-Bound Saturation / Clipping Mode (ReLU6 / Cap)
        // ---------------------------------------------------------------------
        $display("\n--- Test 7: Clipping Mode ---");
        clip_en_i  = 1'b1;
        clip_max_i = 32'sd100; // Cap at 100

        send_and_check("Clip below threshold (50)",  32'sd50,       32'sd50,       1'b0, 1'b1, 1'b0);
        send_and_check("Clip at threshold (100)",     32'sd100,      32'sd100,      1'b0, 1'b1, 1'b0);
        send_and_check("Clip above threshold (150)",  32'sd150,      32'sd100,      1'b0, 1'b1, 1'b1);
        send_and_check("Clip large (10000)",          32'sd10000,    32'sd100,      1'b0, 1'b1, 1'b1);
        send_and_check("Clip negative (-25)",         -32'sd25,      32'sd0,        1'b1, 1'b0, 1'b0);
        clip_en_i  = 1'b0;

        // ---------------------------------------------------------------------
        // Test 8: Backpressure / Stalling
        // ---------------------------------------------------------------------
        $display("\n--- Test 8: Backpressure & Handshake ---");
        @(posedge clk);
        data_i  <= 32'sd88;
        valid_i <= 1'b1;
        ready_i <= 1'b0; // Deassert downstream ready

        @(posedge clk);
        valid_i <= 1'b0;

        // The data should be held in output register because ready_i is low
        #(CLK_PERIOD * 3);
        if (valid_o === 1'b1 && data_o === 32'sd88) begin
            $display("[PASS] Data held during downstream stall (valid_o=1, data_o=88)");
            pass_count++;
        end else begin
            $display("[FAIL] Data lost during downstream stall: valid_o=%0b, data_o=%0d", valid_o, data_o);
            fail_count++;
        end

        // Release ready
        @(negedge clk);
        ready_i <= 1'b1;
        @(posedge clk);
        @(posedge clk);

        // ---------------------------------------------------------------------
        // Test 9: Combinational Instance Check
        // ---------------------------------------------------------------------
        $display("\n--- Test 9: Combinational Instance Check ---");
        data_i  = -32'sd77;
        valid_i = 1'b1;
        #1;
        if (comb_data_o === 32'sd0 && comb_zero_o === 1'b1) begin
            $display("[PASS] Comb instance: negative input -77 -> 0 immediately");
            pass_count++;
        end else begin
            $display("[FAIL] Comb instance failed: data=%0d, zero=%0b", comb_data_o, comb_zero_o);
            fail_count++;
        end

        data_i  = 32'sd77;
        valid_i = 1'b0;
        #1;
        if (comb_data_o === 32'sd77 && comb_active_o === 1'b1) begin
            $display("[PASS] Comb instance: positive input 77 -> 77 immediately");
            pass_count++;
        end else begin
            $display("[FAIL] Comb instance failed: data=%0d, active=%0b", comb_data_o, comb_active_o);
            fail_count++;
        end

        // Drain pipeline cleanly
        @(posedge clk);
        data_i  <= '0;
        valid_i <= 1'b0;
        ready_i <= 1'b1;
        @(posedge clk);

        // ---------------------------------------------------------------------
        // Test 10: Randomized Stimulus (50 random signed vectors)
        // ---------------------------------------------------------------------
        $display("\n--- Test 10: Randomized Signed Stimulus ---");
        begin : rand_test
            integer i;
            logic signed [31:0] rand_in, rand_exp;
            logic r_zero, r_act;

            for (i = 0; i < 50; i++) begin
                rand_in = $random;
                rand_exp = (rand_in < 0) ? 32'sd0 : rand_in;
                r_zero   = (rand_exp == 32'sd0);
                r_act    = (rand_exp > 32'sd0);

                send_and_check($sformatf("Rand #%0d (%0d)", i, rand_in),
                               rand_in, rand_exp, r_zero, r_act, 1'b0);
            end
        end

        // ---------------------------------------------------------------------
        // Final Summary
        // ---------------------------------------------------------------------
        $display("\n============================================================");
        $display("  ReLU Unit Test Summary");
        $display("  PASSED: %0d", pass_count);
        $display("  FAILED: %0d", fail_count);
        $display("============================================================");

        if (fail_count == 0) begin
            $display("  *** ALL RELU UNIT TESTS PASSED ***\n");
        end else begin
            $display("  *** SOME TESTS FAILED ***\n");
        end

        $finish;
    end

endmodule

`default_nettype wire
