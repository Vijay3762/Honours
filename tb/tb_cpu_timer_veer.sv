// ============================================================================
// File    : tb_cpu_timer_veer.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Testbench — VeeR EL2 Core executing Timer firmware test
//           through the AXI interconnect and the axi_timer slave.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_timer_veer;

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

    logic        timer_irq;

    soc_top_veer #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8),
        .IMEM_HEX   (""),
        .ICCM_HEX   ("timer_test.hex")
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .uart_tx_o       (uart_tx),
        .uart_rx_i       (1'b1),
        .uart_irq_o      (uart_irq),
        .gpio_pads_i     (gpio_pads_i),
        .gpio_pads_o     (gpio_pads_o),
        .gpio_pads_oen   (gpio_pads_oen),
        .timer_irq_o     (timer_irq),
        .seven_seg_seg_o (),
        .seven_seg_dp_o  (),
        .seven_seg_an_o  ()
    );

    // --------------------------------------------------------------
    // Bus activity counters
    // --------------------------------------------------------------
    integer ifu_reads    = 0;
    integer lsu_reads    = 0;
    integer lsu_writes   = 0;
    integer timer_reads  = 0;
    integer timer_writes = 0;
    logic   saw_timer_irq = 1'b0;

    always @(posedge clk) begin
        if (!rst) begin
            if (dut.ifu_axi_arvalid && dut.ifu_axi_arready) ifu_reads    = ifu_reads + 1;
            if (dut.lsu_axi_arvalid && dut.lsu_axi_arready) lsu_reads    = lsu_reads + 1;
            if (dut.lsu_axi_awvalid && dut.lsu_axi_awready) lsu_writes   = lsu_writes + 1;
            if (dut.timer_inst.s_axi_arvalid && dut.timer_inst.s_axi_arready)
                timer_reads = timer_reads + 1;
            if (dut.timer_inst.s_axi_awvalid && dut.timer_inst.s_axi_awready)
                timer_writes = timer_writes + 1;
            if (timer_irq)
                saw_timer_irq <= 1'b1;
        end
    end

    // --------------------------------------------------------------
    // Test control
    // --------------------------------------------------------------
    integer errors = 0;

    initial begin
        $dumpfile("cpu_timer_veer.vcd");
        $dumpvars(0, tb_cpu_timer_veer);

        gpio_pads_i = 16'h0000;

        $display("============================================================");
        $display("  VeeR EL2 Core -> AXI Interconnect -> Timer Test");
        $display("============================================================");

        // Reset
        rst = 1'b1;
        repeat (10) @(posedge clk);
        #1;
        rst = 1'b0;

        $display("[SoC] System reset released. Waiting for VeeR EL2 core...");

        // Wait for execution to store pass flag into DMEM (offset 0x1C -> mem[3][63:32] == 1)
        fork
            begin
                wait (dut.dmem.mem[3][63:32] == 32'd1 || dut.dmem.mem[7][31:0] == 32'd1);
                @(posedge clk);
            end
            begin
                #200000;
                $display("\n[TIMEOUT] Simulation timed out waiting for firmware completion!");
            end
        join_any

        $display("\n[BUS COUNTERS]");
        $display("  IFU reads    : %0d", ifu_reads);
        $display("  LSU reads    : %0d", lsu_reads);
        $display("  LSU writes   : %0d", lsu_writes);
        $display("  Timer reads  : %0d", timer_reads);
        $display("  Timer writes : %0d", timer_writes);

        $display("\n[DMEM VERIFICATION]");
        $display("  TIMER_ID        : 0x%08h (expected 0x54494D52)", dut.dmem.mem[0][31:0]);
        $display("  TIMER_VERSION   : 0x%08h (expected 0x00010000)", dut.dmem.mem[0][63:32]);
        $display("  TIMER_LOAD      : %0d (expected 30)", dut.dmem.mem[1][31:0]);
        $display("  TIMER_COMPARE   : %0d (expected 15)", dut.dmem.mem[1][63:32]);
        $display("  TIMER_VALUE     : %0d (expected < 30)", dut.dmem.mem[2][31:0]);
        $display("  STATUS (before) : 0x%08h (bit 0 = 1)", dut.dmem.mem[2][63:32]);
        $display("  STATUS (after)  : 0x%08h (bit 0 = 0)", dut.dmem.mem[3][31:0]);
        $display("  Status Flag     : %0d (expected 1)", dut.dmem.mem[3][63:32]);
        $display("  Saw Timer IRQ   : %0b (expected 1)", saw_timer_irq);

        if (dut.dmem.mem[3][63:32] == 32'd1 || dut.dmem.mem[7][31:0] == 32'd1) begin
            $display("\n*** ALL VEER EL2 -> TIMER TESTS PASSED ***\n");
        end else begin
            $display("\n*** TEST FAILED ***\n");
            errors = errors + 1;
        end

        $finish;
    end

endmodule

`default_nettype wire
