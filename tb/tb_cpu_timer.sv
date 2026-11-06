// ============================================================================
// File    : tb_cpu_timer.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : End-to-end SoC testbench: CPU Master -> AXI Interconnect -> Timer
//           Verifies CPU-driven Timer configuration, down-counter operation,
//           compare match detection, IRQ assertion, and W1C status clear.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_timer;

    // ------------------------------------------------------------------
    // Clock & reset generation (100 MHz clock)
    // ------------------------------------------------------------------
    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // DUT signals
    // ------------------------------------------------------------------
    logic        timer_irq;
    logic        cpu_done;
    logic        cpu_pass;

    // ------------------------------------------------------------------
    // Instantiate DUT: SoC Top in TIMER test mode
    // ------------------------------------------------------------------
    soc_top #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8),
        .TEST_MODE  ("TIMER")
    ) dut (
        .clk             (clk),
        .rst             (rst),

        // UART external (idle)
        .uart_tx_o       (),
        .uart_rx_i       (1'b1),
        .uart_irq_o      (),

        // GPIO external (idle)
        .gpio_pads_i     (16'h0000),
        .gpio_pads_o     (),
        .gpio_pads_oen   (),

        // Timer external IRQ
        .timer_irq_o     (timer_irq),

        // CNN external
        .cnn_irq_o       (),

        // 7-Seg external
        .seven_seg_seg_o (),
        .seven_seg_dp_o  (),
        .seven_seg_an_o  (),

        // CPU status
        .cpu_done_o      (cpu_done),
        .cpu_pass_o      (cpu_pass)
    );

    // ------------------------------------------------------------------
    // Test verification tracking
    // ------------------------------------------------------------------
    integer pass_count;
    integer fail_count;
    logic   saw_irq_asserted;

    task check(input string desc, input logic condition);
        if (condition) begin
            $display("  [PASS] %s", desc);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %s", desc);
            fail_count = fail_count + 1;
        end
    endtask

    // Monitor Timer IRQ assertion during execution
    always @(posedge clk) begin
        if (!rst && timer_irq) begin
            saw_irq_asserted <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        pass_count       = 0;
        fail_count       = 0;
        saw_irq_asserted = 1'b0;

        // VCD Dump
        $dumpfile("cpu_timer_test.vcd");
        $dumpvars(0, tb_cpu_timer);

        $display("============================================================");
        $display("  tb_cpu_timer: CPU Master -> AXI Interconnect -> Timer Test");
        $display("============================================================");

        // Apply Reset
        rst = 1'b1;
        repeat (10) @(posedge clk);
        #1;

        // Verify Reset State
        check("Reset: timer_irq = 0",  timer_irq == 1'b0);
        check("Reset: cpu_done = 0",   cpu_done == 1'b0);
        check("Reset: cpu_pass = 0",   cpu_pass == 1'b0);

        // Release Reset
        rst = 1'b0;
        $display("\n[SoC] System reset released. CPU master executing Timer transactions...");

        // Wait for CPU test sequence completion with timeout
        fork
            begin
                wait (cpu_done);
                @(posedge clk);
            end
            begin
                #100000; // 100 us timeout watchdog
                $display("\n[TIMEOUT] Simulation timed out waiting for cpu_done!");
            end
        join_any

        // Verify Final CPU Master Checks
        $display("\n[SoC] CPU Master transaction sequence completed.");
        check("CPU Status: cpu_done asserted", cpu_done == 1'b1);
        check("CPU Status: cpu_pass asserted (all Timer register/bus checks passed)", cpu_pass == 1'b1);

        // Verification of internal captured values from simple_cpu_master
        check("CPU Read: TIMER_ID = 0x54494D52 ('TIMR')", dut.cpu_inst.rx_timer_id == 32'h54494D52);
        check("CPU Read: TIMER_VERSION = 0x00010000 (v1.0)", dut.cpu_inst.rx_timer_ver == 32'h00010000);
        check("CPU Read: TIMER_LOAD readback = 30", dut.cpu_inst.rx_timer_load == 32'd30);
        check("CPU Read: TIMER_COMPARE readback = 15", dut.cpu_inst.rx_timer_cmp == 32'd15);
        check("CPU Read: TIMER_VALUE decreased below load (< 30)", dut.cpu_inst.rx_timer_val < 32'd30);
        check("CPU Read: TIMER_STATUS match flag observed set (bit 0 == 1)", dut.cpu_inst.rx_timer_stat_match[0] == 1'b1);
        check("CPU Read: TIMER_STATUS match flag cleared by W1C (bit 0 == 0)", dut.cpu_inst.rx_timer_stat_cleared[0] == 1'b0);

        // External IRQ line behavior checks
        check("Timer IRQ: saw timer_irq_o assert high during countdown match", saw_irq_asserted == 1'b1);
        check("Timer IRQ: timer_irq_o deasserted low after W1C clear", timer_irq == 1'b0);

        // Final Summary
        $display("\n============================================================");
        $display("  tb_cpu_timer Test Summary");
        $display("============================================================");
        $display("  Total Passed: %0d", pass_count);
        $display("  Total Failed: %0d", fail_count);
        $display("============================================================");

        if (fail_count == 0 && pass_count >= 12) begin
            $display("\n*** ALL CPU -> INTERCONNECT -> TIMER TESTS PASSED ***\n");
        end else begin
            $display("\n*** TEST FAILED ***\n");
        end

        $finish;
    end

endmodule

`default_nettype wire
