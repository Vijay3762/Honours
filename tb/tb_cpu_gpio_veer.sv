// ============================================================================
// File    : tb_cpu_gpio_veer.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Testbench — VeeR EL2 Core executing GPIO firmware test
//           through the AXI interconnect and the axi_gpio slave.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_gpio_veer;

    // --------------------------------------------------------------
    // Clock / reset
    // --------------------------------------------------------------
    logic clk;
    logic rst;           // active-high

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    // --------------------------------------------------------------
    // DUT signals
    // --------------------------------------------------------------
    logic        uart_tx;
    logic        uart_rx;
    logic        uart_irq;

    logic [15:0] gpio_pads_i;
    logic [15:0] gpio_pads_o;
    logic [15:0] gpio_pads_oen;

    soc_top_veer #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8),
        .IMEM_HEX   (""),
        .ICCM_HEX   ("gpio_test.hex")
    ) dut (
        .clk           (clk),
        .rst           (rst),
        .uart_tx_o     (uart_tx),
        .uart_rx_i     (1'b1),
        .uart_irq_o    (uart_irq),
        .gpio_pads_i   (gpio_pads_i),
        .gpio_pads_o   (gpio_pads_o),
        .gpio_pads_oen (gpio_pads_oen)
    );

    // --------------------------------------------------------------
    // Bus activity counters
    // --------------------------------------------------------------
    integer ifu_reads   = 0;
    integer lsu_reads   = 0;
    integer lsu_writes  = 0;
    integer gpio_reads  = 0;
    integer gpio_writes = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (dut.ifu_axi_arvalid && dut.ifu_axi_arready) ifu_reads   = ifu_reads + 1;
            if (dut.lsu_axi_arvalid && dut.lsu_axi_arready) lsu_reads   = lsu_reads + 1;
            if (dut.lsu_axi_awvalid && dut.lsu_axi_awready) lsu_writes  = lsu_writes + 1;
            if (dut.gpio_inst.s_axi_arvalid && dut.gpio_inst.s_axi_arready)
                gpio_reads = gpio_reads + 1;
            if (dut.gpio_inst.s_axi_awvalid && dut.gpio_inst.s_axi_awready)
                gpio_writes = gpio_writes + 1;
        end
    end

    // --------------------------------------------------------------
    // Test control
    // --------------------------------------------------------------
    integer errors = 0;

    initial begin
`ifdef FSDB
        $fsdbDumpfile("dump.fsdb");
        $fsdbDumpvars(0, tb_cpu_gpio_veer, "+all");
        $fsdbDumpSVA();
        $fsdbDumpMDA();
`else
        $dumpfile("cpu_gpio_veer.vcd");
        $dumpvars(0, tb_cpu_gpio_veer);
`endif

        gpio_pads_i = 16'h5A00;

        $display("============================================================");
        $display("  VeeR EL2 Core -> AXI Interconnect -> GPIO Test");
        $display("============================================================");

        // Reset
        rst = 1'b1;
        repeat (10) @(posedge clk);
        #1;
        rst = 1'b0;

        $display("[SoC] System reset released. Waiting for VeeR EL2 core...");

        // Wait for execution to store pass flag into DMEM
        // DMEM[5] (byte offset 0x14) == 1
        fork
            begin
                wait (dut.dmem.mem[5] == 64'd1 || dut.dmem.mem[5] == 32'd1);
                @(posedge clk);
            end
            begin
                #200000;
                $display("\n[TIMEOUT] Simulation timed out waiting for firmware completion!");
            end
        join_any

        $display("\n[BUS COUNTERS]");
        $display("  IFU reads   : %0d", ifu_reads);
        $display("  LSU reads   : %0d", lsu_reads);
        $display("  LSU writes  : %0d", lsu_writes);
        $display("  GPIO reads  : %0d", gpio_reads);
        $display("  GPIO writes : %0d", gpio_writes);

        $display("\n[DMEM VERIFICATION]");
        $display("  GPIO_ID      : 0x%08h (expected 0x4749504F)", dut.dmem.mem[0][31:0]);
        $display("  GPIO_VERSION : 0x%08h (expected 0x00010000)", dut.dmem.mem[1][31:0]);
        $display("  GPIO_DIR     : 0x%08h (expected 0x000000FF)", dut.dmem.mem[2][31:0]);
        $display("  GPIO_OUT     : 0x%08h (expected 0x000000C3)", dut.dmem.mem[3][31:0]);
        $display("  GPIO_IN      : 0x%08h", dut.dmem.mem[4][31:0]);
        $display("  Status Flag  : %0d (expected 1)", dut.dmem.mem[5][31:0]);

        if (dut.dmem.mem[5][31:0] == 32'd1) begin
            $display("\n*** ALL VEER EL2 -> GPIO TESTS PASSED ***\n");
        end else begin
            $display("\n*** TEST FAILED ***\n");
            errors = errors + 1;
        end

        $finish;
    end

endmodule

`default_nettype wire
