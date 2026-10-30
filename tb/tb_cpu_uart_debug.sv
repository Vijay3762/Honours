// ============================================================================
// File    : tb_cpu_uart_debug.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Debug testbench - traces bvalid path: CPU->IC->bridge->UART
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module tb_cpu_uart_debug;

    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    logic uart_tx, uart_rx, uart_irq;
    assign uart_rx = uart_tx;

    soc_top #(
        .DATA_WIDTH (64),
        .ADDR_WIDTH (32),
        .STRB_WIDTH (8),
        .ID_WIDTH   (8)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .uart_tx_o  (uart_tx),
        .uart_rx_i  (uart_rx),
        .uart_irq_o (uart_irq)
    );

    // Interconnect internal state
    wire [2:0] ic_state = dut.ic_inst.state_reg;
    // CPU->interconnect s00
    wire cpu_awv   = dut.cpu_awvalid;
    wire cpu_awr   = dut.cpu_awready;
    wire cpu_wv    = dut.cpu_wvalid;
    wire cpu_wr    = dut.cpu_wready;
    wire cpu_bv    = dut.cpu_bvalid;
    wire cpu_br    = dut.cpu_bready;
    // Interconnect->bridge m04
    wire m04_awv   = dut.m_axi_awvalid[4];
    wire m04_awr   = dut.m_axi_awready[4];
    wire m04_wv    = dut.m_axi_wvalid[4];
    wire m04_wr    = dut.m_axi_wready[4];
    wire m04_bv    = dut.m_axi_bvalid[4];
    wire m04_br    = dut.m_axi_bready[4];
    // Bridge->UART
    wire br_awv    = dut.uart_bridge.uart_awvalid;
    wire br_awr    = dut.uart_bridge.uart_awready;
    wire br_wv     = dut.uart_bridge.uart_wvalid;
    wire br_wr     = dut.uart_bridge.uart_wready;
    wire br_bv     = dut.uart_bridge.uart_bvalid;
    wire br_arv    = dut.uart_bridge.uart_arvalid;  // FSM-driven
    wire br_arr    = dut.uart_bridge.uart_arready;
    wire br_rv     = dut.uart_bridge.uart_rvalid;
    wire br_rr     = dut.uart_bridge.s_axi_rready;
    // UART internals
    wire u_wren    = dut.uart_bridge.uart_top.axi_wren;
    wire u_sync    = dut.uart_bridge.uart_top.axi_sync_wren;
    wire [2:0] u_ws = dut.uart_bridge.uart_top.write_state;
    wire [3:0] u_rs = dut.uart_bridge.uart_top.read_state;
    wire u_rvalid  = dut.uart_bridge.uart_top.axi_rvalid;
    wire u_arv     = dut.uart_bridge.uart_top.axi_arvalid_i;

    integer cycle;
    initial cycle = 0;

    always @(posedge clk) begin
        if (!rst) begin
            cycle = cycle + 1;
            if (cycle <= 60) begin
                $display("[%3d] IC=%0d | CPU aw=%b/%b w=%b/%b b=%b/%b ar=%b/%b r=%b/%b | M04 aw=%b/%b w=%b/%b b=%b/%b ar=%b/%b rv=%b | BR bv=%b arv=%b/%b rv=%b/%b | UART rs=%0d rv=%b",
                    cycle, ic_state,
                    cpu_awv, cpu_awr, cpu_wv, cpu_wr, cpu_bv, cpu_br,
                    dut.cpu_arvalid, dut.cpu_arready, dut.cpu_rvalid, dut.cpu_rready,
                    m04_awv, m04_awr, m04_wv, m04_wr, m04_bv, m04_br,
                    dut.m_axi_arvalid[4], dut.m_axi_arready[4], dut.m_axi_rvalid[4],
                    br_bv, br_arv, br_arr, br_rv, br_rr,
                    u_rs, u_rvalid);
            end
        end
    end

    initial begin
        rst = 1'b1;
        repeat (10) @(posedge clk);
        rst = 1'b0;

        $display("\n=== CPU-to-UART Debug: IC state 0=IDLE 1=DECODE 2=WRITE 3=WRITE_RESP 4=WRITE_DROP 5=READ 6=READ_DROP 7=WAIT_IDLE ===");
        $display("[CYC] IC | CPU aw=v/r w=v/r b=v/r | M04 aw=v/r w=v/r b=v/r | BR aw=v/r w=v/r b=v | UART wren sync ws");

        repeat (60) @(posedge clk);

        $display("\n=== Trace complete ===");
        #100;
        $finish;
    end

endmodule

`default_nettype wire
