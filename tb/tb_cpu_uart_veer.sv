// ============================================================================
// File    : tb_cpu_uart_veer.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Testbench - the VeeR EL2 core executing the UART read/write test
//           through the AXI interconnect and the UART bridge.
//
//           The UART tx line is looped back to rx, so the byte the core
//           transmits is received again.  The testbench
//             * decodes the serial line and checks that 'A' and 'B' appear
//             * checks the value the core read back from the RBR (stored by
//               the firmware in the DMEM at 0x1000_0000)
//             * checks the line status register the core read
//               (stored at 0x1000_0004)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_uart_veer;

    // --------------------------------------------------------------
    // Clock / reset
    // --------------------------------------------------------------
    logic clk;
    logic rst;           // active-high

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    // --------------------------------------------------------------
    // DUT
    // --------------------------------------------------------------
    logic uart_tx;
    logic uart_rx;
    logic uart_irq;

    // Internal loopback: tx -> rx
    assign uart_rx = uart_tx;

    soc_top_veer #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8),
        .IMEM_HEX   (""),
        .ICCM_HEX   ("uart_test.hex")
    ) dut (
        .clk         (clk),
        .rst         (rst),
        .uart_tx_o   (uart_tx),
        .uart_rx_i   (uart_rx),
        .uart_irq_o  (uart_irq)
    );

    // --------------------------------------------------------------
    // Bus activity counters (proof that the core drives the bus)
    // --------------------------------------------------------------
    integer ifu_reads = 0;
    integer lsu_reads = 0;
    integer lsu_writes = 0;
    integer uart_reads = 0;
    integer uart_writes = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (dut.ifu_axi_arvalid && dut.ifu_axi_arready) ifu_reads = ifu_reads + 1;
            if (dut.lsu_axi_arvalid && dut.lsu_axi_arready) lsu_reads = lsu_reads + 1;
            if (dut.lsu_axi_awvalid && dut.lsu_axi_awready) lsu_writes = lsu_writes + 1;
            if (dut.uart_bridge.s_axi_arvalid && dut.uart_bridge.s_axi_arready)
                uart_reads = uart_reads + 1;
            if (dut.uart_bridge.s_axi_awvalid && dut.uart_bridge.s_axi_awready)
                uart_writes = uart_writes + 1;
        end
    end

    // --------------------------------------------------------------
    // UART serial decoder
    //   BAUD_DIV = 50, 100 MHz clock -> 51 clocks per bit (510 ns)
    // --------------------------------------------------------------
    localparam integer BAUD_DIV     = 50;
    localparam real    CLK_PERIOD_NS = 10.0;
    localparam real    BIT_TIME_NS   = (BAUD_DIV + 1) * CLK_PERIOD_NS;

    logic [7:0] captured_bytes [0:15];
    integer     captured_count = 0;

    integer bit_idx;
    logic [7:0] rx_shift;

    always @(negedge uart_tx) begin
        if (!rst) begin
            bit_idx  = 0;
            rx_shift = 8'h00;
            #(BIT_TIME_NS / 2);              // centre of the start bit
            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                #(BIT_TIME_NS);              // centre of each data bit
                rx_shift[bit_idx] = uart_tx;
            end
            #(BIT_TIME_NS);                  // stop bit
            if (captured_count < 16) begin
                captured_bytes[captured_count] = rx_shift;
                captured_count = captured_count + 1;
                $display("[%0t] [UART RX] byte 0x%02h (%c)",
                         $time, rx_shift,
                         (rx_shift >= 32 && rx_shift <= 126) ? rx_shift : "?");
            end
        end
    end

    // --------------------------------------------------------------
    // Test control
    // --------------------------------------------------------------
    integer errors = 0;
    integer i;
    reg [63:0] dmem_word0;
    reg [31:0] rx_byte;
    reg [31:0] lsr_value;

    initial begin
        rst = 1'b1;
        repeat (20) @(posedge clk);
        rst = 1'b0;

        $display("\n======================================================");
        $display("  VeeR EL2 -> AXI interconnect -> UART read/write test");
        $display("======================================================\n");

        // Let the core boot, configure the UART and run the test.
        repeat (200000) @(posedge clk);

        // ---- results ----
        $display("\n------------------------------------------------------");
        $display("  Bus activity");
        $display("    IFU read transactions : %0d", ifu_reads);
        $display("    LSU read transactions : %0d", lsu_reads);
        $display("    LSU write transactions: %0d", lsu_writes);
        $display("    UART read transactions: %0d", uart_reads);
        $display("    UART write transactions: %0d", uart_writes);
        $display("------------------------------------------------------");

        // ---- 1) serial output ----
        $display("  Serial line");
        if (captured_count >= 1) begin
            if (captured_bytes[0] == 8'h41)
                $display("    [PASS] first transmitted byte  = 0x41 ('A')");
            else begin
                $display("    [FAIL] first transmitted byte  = 0x%02h, expected 0x41",
                         captured_bytes[0]);
                errors = errors + 1;
            end
        end else begin
            $display("    [FAIL] no byte received on the serial line");
            errors = errors + 1;
        end

        if (captured_count >= 2) begin
            if (captured_bytes[1] == 8'h42)
                $display("    [PASS] second transmitted byte = 0x42 ('B')");
            else begin
                $display("    [FAIL] second transmitted byte = 0x%02h, expected 0x42",
                         captured_bytes[1]);
                errors = errors + 1;
            end
        end else begin
            $display("    [FAIL] only %0d byte(s) on the serial line, expected 2",
                     captured_count);
            errors = errors + 1;
        end

        // ---- 2) value the core read back from the RBR ----
        dmem_word0 = dut.dmem.mem[0];
        rx_byte    = dmem_word0[31:0];
        lsr_value  = dmem_word0[63:32];

        $display("  CPU read back");
        if (rx_byte == 8'h41)
            $display("    [PASS] RBR read by the core   = 0x%02h ('A')", rx_byte);
        else begin
            $display("    [FAIL] RBR read by the core   = 0x%02h, expected 0x41",
                     rx_byte);
            errors = errors + 1;
        end

        // The firmware reads the LSR once more after the byte has been taken
        // out of the receive FIFO, so the data ready bit is cleared again.
        // The transmit side has to be idle (TEMT and THRE set).
        if (lsr_value[6] === 1'b1 && lsr_value[5] === 1'b1)
            $display("    [PASS] LSR read by the core   = 0x%02h (TX idle, RX drained)",
                     lsr_value);
        else begin
            $display("    [FAIL] LSR read by the core   = 0x%02h (TEMT/THRE not set)",
                     lsr_value);
            errors = errors + 1;
        end

        $display("------------------------------------------------------");
        if (errors == 0)
            $display("  *** ALL TESTS PASSED ***");
        else
            $display("  *** %0d TEST(S) FAILED ***", errors);
        $display("======================================================\n");

        #1000;
        $finish;
    end

    // Watchdog
    initial begin
        #50_000_000;
        $display("\n[WATCHDOG] Simulation timed out!");
        $finish;
    end

endmodule

`default_nettype wire
