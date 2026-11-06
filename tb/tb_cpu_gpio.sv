// ============================================================================
// File    : tb_cpu_gpio.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : End-to-end SoC testbench: CPU Master -> AXI Interconnect -> GPIO
//           Verifies CPU-driven GPIO configuration, output writes, atomic
//           set/clr/toggle operations, and input sampling via pad pins.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_gpio;

    // ------------------------------------------------------------------
    // Clock & reset generation (100 MHz clock)
    // ------------------------------------------------------------------
    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ------------------------------------------------------------------
    // GPIO pad connections
    // ------------------------------------------------------------------
    logic [15:0] gpio_pads_i;
    logic [15:0] gpio_pads_o;
    logic [15:0] gpio_pads_oen;

    // CPU status signals
    logic        cpu_done;
    logic        cpu_pass;

    // ------------------------------------------------------------------
    // Instantiate DUT: SoC Top in GPIO test mode
    // ------------------------------------------------------------------
    soc_top #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8),
        .TEST_MODE  ("GPIO")
    ) dut (
        .clk           (clk),
        .rst           (rst),

        // UART external (idle)
        .uart_tx_o     (),
        .uart_rx_i     (1'b1),
        .uart_irq_o    (),

        // GPIO external pads
        .gpio_pads_i   (gpio_pads_i),
        .gpio_pads_o   (gpio_pads_o),
        .gpio_pads_oen (gpio_pads_oen),

        // Timer external
        .timer_irq_o   (),

        // CNN external
        .cnn_irq_o     (),

        // CPU status
        .cpu_done_o    (cpu_done),
        .cpu_pass_o    (cpu_pass)
    );

    // ------------------------------------------------------------------
    // Test verification tracking
    // ------------------------------------------------------------------
    integer pass_count;
    integer fail_count;

    task check(input string desc, input logic condition);
        if (condition) begin
            $display("  [PASS] %s", desc);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %s", desc);
            fail_count = fail_count + 1;
        end
    endtask

    // ------------------------------------------------------------------
    // Main test sequence
    // ------------------------------------------------------------------
    initial begin
        pass_count = 0;
        fail_count = 0;

        // Drive initial external GPIO inputs (upper byte = 0x5A, lower byte = 0x3C)
        gpio_pads_i = 16'h5A3C;

        // VCD Dump
        $dumpfile("cpu_gpio_test.vcd");
        $dumpvars(0, tb_cpu_gpio);

        $display("============================================================");
        $display("  tb_cpu_gpio: CPU Core -> AXI Interconnect -> GPIO Test");
        $display("============================================================");

        // Apply Reset
        rst = 1'b1;
        repeat (10) @(posedge clk);
        #1;

        // Verify Reset State
        check("Reset: gpio_pads_o = 0x0000",   gpio_pads_o == 16'h0000);
        check("Reset: gpio_pads_oen = 0x0000", gpio_pads_oen == 16'h0000);
        check("Reset: cpu_done = 0",          cpu_done == 1'b0);

        // Release Reset
        rst = 1'b0;
        $display("\n[SoC] System reset released. CPU master executing GPIO transactions...");

        // Monitor external pad responses to CPU register accesses
        // Phase 1: Wait for DIR to be written to 0x00FF
        wait (gpio_pads_oen == 16'h00FF || cpu_done);
        check("Pad Check: gpio_pads_oen updated to 0x00FF (DIR write)", gpio_pads_oen == 16'h00FF);

        // Phase 2: Wait for OUT to be written to 0x00A5
        wait (gpio_pads_o == 16'h00A5 || cpu_done);
        check("Pad Check: gpio_pads_o updated to 0x00A5 (OUT write)", gpio_pads_o == 16'h00A5);

        // Phase 3: Wait for SET action (0xA5 | 0x50 = 0x00F5)
        wait (gpio_pads_o == 16'h00F5 || cpu_done);
        check("Pad Check: gpio_pads_o updated to 0x00F5 (SET 0x0050)", gpio_pads_o == 16'h00F5);

        // Phase 4: Wait for CLR action (0xF5 & ~0x05 = 0x00F0)
        wait (gpio_pads_o == 16'h00F0 || cpu_done);
        check("Pad Check: gpio_pads_o updated to 0x00F0 (CLR 0x0005)", gpio_pads_o == 16'h00F0);

        // Phase 5: Wait for TOGGLE action (0xF0 ^ 0x33 = 0x00C3)
        wait (gpio_pads_o == 16'h00C3 || cpu_done);
        check("Pad Check: gpio_pads_o updated to 0x00C3 (TOGGLE 0x0033)", gpio_pads_o == 16'h00C3);

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
        check("CPU Status: cpu_pass asserted (all 10 register/bus checks passed)", cpu_pass == 1'b1);

        // Verification of internal captured values
        check("CPU Read: GPIO_ID = 0x4749504F ('GPIO')", dut.cpu_inst.rx_gpio_id == 32'h4749504F);
        check("CPU Read: GPIO_VERSION = 0x00010000 (v1.0)", dut.cpu_inst.rx_gpio_ver == 32'h00010000);
        check("CPU Read: Initial GPIO_OUT = 0x00000000", dut.cpu_inst.rx_gpio_out_init == 32'h0);
        check("CPU Read: Initial GPIO_DIR = 0x00000000", dut.cpu_inst.rx_gpio_dir_init == 32'h0);
        check("CPU Read: GPIO_DIR readback = 0x000000FF", dut.cpu_inst.rx_gpio_dir == 32'h000000FF);
        check("CPU Read: GPIO_OUT readback = 0x000000A5", dut.cpu_inst.rx_gpio_out_wr == 32'h000000A5);
        check("CPU Read: GPIO_OUT after SET = 0x000000F5", dut.cpu_inst.rx_gpio_out_set == 32'h000000F5);
        check("CPU Read: GPIO_OUT after CLR = 0x000000F0", dut.cpu_inst.rx_gpio_out_clr == 32'h000000F0);
        check("CPU Read: GPIO_OUT after TOGGLE = 0x000000C3", dut.cpu_inst.rx_gpio_out_tog == 32'h000000C3);
        check("CPU Read: GPIO_IN upper byte = 0x5A (from external pad)", dut.cpu_inst.rx_gpio_in[15:8] == 8'h5A);

        // Final Summary
        $display("\n============================================================");
        $display("  tb_cpu_gpio Test Summary");
        $display("============================================================");
        $display("  Total Passed: %0d", pass_count);
        $display("  Total Failed: %0d", fail_count);
        $display("============================================================");

        if (fail_count == 0 && pass_count >= 18) begin
            $display("\n*** ALL CPU -> INTERCONNECT -> GPIO TESTS PASSED ***\n");
        end else begin
            $display("\n*** TEST FAILED ***\n");
        end

        $finish;
    end

endmodule

`default_nettype wire
