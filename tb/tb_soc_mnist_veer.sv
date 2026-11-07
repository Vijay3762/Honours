// ============================================================================
// File    : tb_soc_mnist_veer.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : System-level testbench for end-to-end MNIST digit recognition.
//           Boots the VeeR EL2 core, runs hardware-accelerated CNN convolution,
//           ReLU activation, 2x2 spatial pooling, and RISC-V software classifier.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_soc_mnist_veer;

    // --------------------------------------------------------------
    // Clock / reset
    // --------------------------------------------------------------
    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz (10 ns period)

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
    logic        dma_irq;
    logic        cnn_irq;

    logic [6:0]  seven_seg_seg;
    logic        seven_seg_dp;
    logic [3:0]  seven_seg_an;

    assign uart_rx = 1'b1;

    soc_top_veer #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8),
        .IMEM_HEX   (""),
        .ICCM_HEX   ("mnist_inference.hex")
    ) dut (
        .clk             (clk),
        .rst             (rst),
        .uart_tx_o       (uart_tx),
        .uart_rx_i       (uart_rx),
        .uart_irq_o      (uart_irq),
        .gpio_pads_i     (gpio_pads_i),
        .gpio_pads_o     (gpio_pads_o),
        .gpio_pads_oen   (gpio_pads_oen),
        .timer_irq_o     (timer_irq),
        .dma_irq_o       (dma_irq),
        .cnn_irq_o       (cnn_irq),
        .seven_seg_seg_o (seven_seg_seg),
        .seven_seg_dp_o  (seven_seg_dp),
        .seven_seg_an_o  (seven_seg_an)
    );

    // --------------------------------------------------------------
    // UART Real-Time Terminal Output Monitor
    //   BAUD_DIV = 50, 100 MHz clock -> 51 clocks per bit (510 ns)
    // --------------------------------------------------------------
    localparam integer BAUD_DIV      = 50;
    localparam real    CLK_PERIOD_NS = 10.0;
    localparam real    BIT_TIME_NS   = (BAUD_DIV + 1) * CLK_PERIOD_NS;

    integer bit_idx;
    logic [7:0] rx_shift;

    always @(negedge uart_tx) begin
        if (!rst) begin
            #(BIT_TIME_NS / 2);              // Center of start bit
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                #(BIT_TIME_NS);              // Center of each data bit
                rx_shift[bit_idx] = uart_tx;
            end
            #(BIT_TIME_NS);                  // Stop bit
            $write("%c", rx_shift);
            $fflush();
        end
    end

    // --------------------------------------------------------------
    // Bus activity counters
    // --------------------------------------------------------------
    integer ifu_reads   = 0;
    integer lsu_reads   = 0;
    integer lsu_writes  = 0;
    integer cnn_access  = 0;
    integer relu_access = 0;
    integer pool_access = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (dut.ifu_axi_arvalid && dut.ifu_axi_arready) ifu_reads = ifu_reads + 1;
            if (dut.lsu_axi_arvalid && dut.lsu_axi_arready) lsu_reads = lsu_reads + 1;
            if (dut.lsu_axi_awvalid && dut.lsu_axi_awready) lsu_writes = lsu_writes + 1;
            if ((dut.cnn_inst.s_axi_awvalid && dut.cnn_inst.s_axi_awready) ||
                (dut.cnn_inst.s_axi_arvalid && dut.cnn_inst.s_axi_arready))
                cnn_access = cnn_access + 1;
            if ((dut.relu_inst.s_axi_awvalid && dut.relu_inst.s_axi_awready) ||
                (dut.relu_inst.s_axi_arvalid && dut.relu_inst.s_axi_arready))
                relu_access = relu_access + 1;
            if ((dut.pooling_inst.s_axi_awvalid && dut.pooling_inst.s_axi_awready) ||
                (dut.pooling_inst.s_axi_arvalid && dut.pooling_inst.s_axi_arready))
                pool_access = pool_access + 1;
        end
    end

    // --------------------------------------------------------------
    // Test control
    // --------------------------------------------------------------
    integer errors = 0;
    logic test_done = 0;

`ifdef FSDB
    initial begin
        $fsdbDumpfile("dump.fsdb");
        $fsdbDumpvars(0, tb_soc_mnist_veer, "+all");
        $fsdbDumpSVA();
        $fsdbDumpMDA();
    end
`endif

    initial begin
        gpio_pads_i = 16'h0000;

        $display("\n============================================================");
        $display("  VeeR EL2 SoC -- Full End-to-End MNIST Recognition TB");
        $display("============================================================\n");

        // Apply reset
        rst = 1'b1;
        repeat (20) @(posedge clk);
        #1;
        rst = 1'b0;

        $display("[TB] Reset deasserted. VeeR core executing from ICCM (0xEE000000)...");

        // Wait for firmware to write PASS flag into DMEM[2] (offset 0x14)
        // or timeout
        fork : wait_block
            begin
                wait (dut.dmem.mem[2][39:32] == 8'h01 ||
                      dut.dmem.mem[2][7:0]   == 8'h01 ||
                      gpio_pads_o            == 16'h00AA);
                @(posedge clk);
                test_done = 1'b1;
                disable wait_block;
            end
            begin
                repeat (1_500_000) @(posedge clk);
                $display("\n[TB ERROR] Simulation timeout reached!");
                errors = errors + 1;
                disable wait_block;
            end
        join

        // Allow final UART characters to drain
        repeat (5000) @(posedge clk);

        $display("\n============================================================");
        $display("  Simulation Verification Summary");
        $display("============================================================");
        $display("  IFU Instruction Reads : %0d", ifu_reads);
        $display("  LSU Data Reads        : %0d", lsu_reads);
        $display("  LSU Data Writes       : %0d", lsu_writes);
        $display("  CNN Conv Transactions : %0d", cnn_access);
        $display("  ReLU Transactions     : %0d", relu_access);
        $display("  Pooling Transactions  : %0d", pool_access);
        $display("  Final GPIO Output     : 0x%04X", gpio_pads_o);
        $display("  7-Seg Digit Register  : 0x%04X", dut.seven_seg_inst.reg_digits);

        if (dut.seven_seg_inst.reg_digits[3:0] == 4'h7) begin
            $display("  [PASS] 7-Segment display shows recognized digit '7'!");
        end else begin
            $display("  [FAIL] 7-Segment display shows 0x%X (expected 0x7)",
                     dut.seven_seg_inst.reg_digits[3:0]);
            errors = errors + 1;
        end

        if (gpio_pads_o == 16'h00AA) begin
            $display("  [PASS] GPIO LEDs output matches PASS pattern (0x00AA)!");
        end else begin
            $display("  [FAIL] GPIO LEDs show 0x%04X (expected 0x00AA)", gpio_pads_o);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("\n*** ALL END-TO-END MNIST INFERENCE TESTS PASSED ***\n");
        end else begin
            $display("\n*** %0d ERROR(S) DETECTED IN INFERENCE RUN ***\n", errors);
        end

        $finish;
    end

endmodule

`default_nettype wire
