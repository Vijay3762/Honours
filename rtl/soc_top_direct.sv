// ============================================================================
// File    : soc_top_direct.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Direct CPU-to-UART test (bypass interconnect) to isolate issues
//           CPU -> axi_uart_bridge -> axi_uart_top (UART tx looped back to rx)
// ============================================================================

`timescale 1ns/1ps

module soc_top_direct #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter ID_WIDTH   = 8
)(
    input  wire                    clk,
    input  wire                    rst,          // active-high

    output wire                    uart_tx_o,
    input  wire                    uart_rx_i,
    output wire                    uart_irq_o
);

    // ------------------------------------------------------------------
    // Simple CPU master AXI4 interface (connects directly to UART bridge)
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     cpu_awid,  cpu_arid,  cpu_rid,   cpu_bid;
    wire [ADDR_WIDTH-1:0]   cpu_awaddr, cpu_araddr;
    wire [7:0]              cpu_awlen, cpu_arlen;
    wire [2:0]              cpu_awsize, cpu_arsize;
    wire [1:0]              cpu_awburst, cpu_arburst;
    wire                    cpu_awlock, cpu_arlock;
    wire [3:0]              cpu_awcache, cpu_arcache;
    wire [2:0]              cpu_awprot, cpu_arprot;
    wire [3:0]              cpu_awqos, cpu_arqos;
    wire                    cpu_awvalid, cpu_awready;
    wire                    cpu_wvalid, cpu_wready;
    wire [DATA_WIDTH-1:0]   cpu_wdata;
    wire [STRB_WIDTH-1:0]   cpu_wstrb;
    wire                    cpu_wlast;
    wire                    cpu_bvalid, cpu_bready;
    wire [1:0]              cpu_bresp;
    wire                    cpu_arvalid, cpu_arready;
    wire                    cpu_rvalid, cpu_rready;
    wire [DATA_WIDTH-1:0]   cpu_rdata;
    wire [1:0]              cpu_rresp;
    wire                    cpu_rlast;

    wire                    cpu_done, cpu_pass;

    // ------------------------------------------------------------------
    // Bridge signals (32-bit AXI4-Lite, 12-bit ID)
    // These connect the CPU (64-bit) to the bridge (32-bit)
    // ------------------------------------------------------------------
    wire [11:0]     bridge_awid;
    wire [4:0]      bridge_awaddr;
    wire [7:0]      bridge_awlen;
    wire [2:0]      bridge_awsize;
    wire [1:0]      bridge_awburst;
    wire            bridge_awlock;
    wire [3:0]      bridge_awcache;
    wire [2:0]      bridge_awprot;
    wire [3:0]      bridge_awqos;
    wire            bridge_awvalid;
    wire            bridge_awready;

    wire [31:0]     bridge_wdata;
    wire [3:0]      bridge_wstrb;
    wire            bridge_wlast;
    wire            bridge_wvalid;
    wire            bridge_wready;

    wire [11:0]     bridge_bid;
    wire [1:0]      bridge_bresp;
    wire            bridge_bvalid;
    wire            bridge_bready;

    wire [11:0]     bridge_arid;
    wire [4:0]      bridge_araddr;
    wire [7:0]      bridge_arlen;
    wire [2:0]      bridge_arsize;
    wire [1:0]      bridge_arburst;
    wire            bridge_arlock;
    wire [3:0]      bridge_arcache;
    wire [2:0]      bridge_arprot;
    wire [3:0]      bridge_arqos;
    wire            bridge_arvalid;
    wire            bridge_arready;

    wire [11:0]     bridge_rid;
    wire [31:0]     bridge_rdata;
    wire [1:0]      bridge_rresp;
    wire            bridge_rlast;
    wire            bridge_rvalid;
    wire            bridge_rready;

    // ------------------------------------------------------------------
    // Width adaptation: CPU 64-bit -> bridge 32-bit
    // Address: CPU[4:0] -> bridge[4:0] (lower 5 bits)
    // Data: CPU[31:0] -> bridge[31:0] (lower 32 bits)
    // Strobe: CPU[3:0] -> bridge[3:0] (lower 4 bits)
    // ID: CPU[7:0] -> bridge[11:0] (zero-extend)
    // ------------------------------------------------------------------
    assign bridge_awid     = {{4{1'b0}}, cpu_awid};
    assign bridge_awaddr   = cpu_awaddr[4:0];
    assign bridge_awlen    = cpu_awlen;
    assign bridge_awsize   = cpu_awsize;
    assign bridge_awburst  = cpu_awburst;
    assign bridge_awlock   = cpu_awlock;
    assign bridge_awcache  = cpu_awcache;
    assign bridge_awprot   = cpu_awprot;
    assign bridge_awqos    = cpu_awqos;
    assign bridge_awvalid  = cpu_awvalid;
    assign cpu_awready     = bridge_awready;

    assign bridge_wdata    = cpu_wdata[31:0];
    assign bridge_wstrb    = cpu_wstrb[3:0];
    assign bridge_wlast    = cpu_wlast;
    assign bridge_wvalid   = cpu_wvalid;
    assign cpu_wready      = bridge_wready;

    assign cpu_bvalid      = bridge_bvalid;
    assign cpu_bresp       = bridge_bresp[1:0];
    assign cpu_bid         = bridge_bid[7:0];
    assign bridge_bready   = cpu_bready;

    assign bridge_arid     = {{4{1'b0}}, cpu_arid};
    assign bridge_araddr   = cpu_araddr[4:0];
    assign bridge_arlen    = cpu_arlen;
    assign bridge_arsize   = cpu_arsize;
    assign bridge_arburst  = cpu_arburst;
    assign bridge_arlock   = cpu_arlock;
    assign bridge_arcache  = cpu_arcache;
    assign bridge_arprot   = cpu_arprot;
    assign bridge_arqos    = cpu_arqos;
    assign bridge_arvalid  = cpu_arvalid;
    assign cpu_arready     = bridge_arready;

    assign cpu_rid         = bridge_rid[7:0];
    assign cpu_rdata       = {{32{1'b0}}, bridge_rdata};
    assign cpu_rresp       = bridge_rresp;
    assign cpu_rlast       = bridge_rlast;
    assign cpu_rvalid      = bridge_rvalid;
    assign bridge_rready   = cpu_rready;

    // ------------------------------------------------------------------
    // Instantiate Simple CPU Master
    // ------------------------------------------------------------------
    simple_cpu_master #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .STRB_WIDTH(STRB_WIDTH),
        .ID_WIDTH  (ID_WIDTH)
    ) cpu_inst (
        .clk            (clk),
        .rst            (rst),
        .m_axi_awid     (cpu_awid),     .m_axi_awready (cpu_awready),
        .m_axi_awaddr   (cpu_awaddr),   .m_axi_awvalid (cpu_awvalid),
        .m_axi_awlen    (cpu_awlen),    .m_axi_awsize  (cpu_awsize),
        .m_axi_awburst  (cpu_awburst),  .m_axi_awlock  (cpu_awlock),
        .m_axi_awcache  (cpu_awcache),  .m_axi_awprot  (cpu_awprot),
        .m_axi_awqos    (cpu_awqos),
        .m_axi_wdata    (cpu_wdata),    .m_axi_wready  (cpu_wready),
        .m_axi_wstrb    (cpu_wstrb),    .m_axi_wvalid  (cpu_wvalid),
        .m_axi_wlast    (cpu_wlast),
        .m_axi_bid      (cpu_bid),      .m_axi_bresp   (cpu_bresp),
        .m_axi_bvalid   (cpu_bvalid),   .m_axi_bready  (cpu_bready),
        .m_axi_arid     (cpu_arid),     .m_axi_arready (cpu_arready),
        .m_axi_araddr   (cpu_araddr),   .m_axi_arvalid (cpu_arvalid),
        .m_axi_arlen    (cpu_arlen),    .m_axi_arsize  (cpu_arsize),
        .m_axi_arburst  (cpu_arburst),  .m_axi_arlock  (cpu_arlock),
        .m_axi_arcache  (cpu_arcache),  .m_axi_arprot  (cpu_arprot),
        .m_axi_arqos    (cpu_arqos),
        .m_axi_rid      (cpu_rid),      .m_axi_rdata   (cpu_rdata),
        .m_axi_rresp    (cpu_rresp),    .m_axi_rlast   (cpu_rlast),
        .m_axi_rvalid   (cpu_rvalid),   .m_axi_rready  (cpu_rready),
        .cpu_done       (cpu_done),
        .cpu_pass       (cpu_pass)
    );

    // ------------------------------------------------------------------
    // Instantiate AXI UART Bridge (32-bit AXI4-Lite slave)
    // The bridge internally connects to axi_uart_top via its own wires
    // ------------------------------------------------------------------
    axi_uart_bridge #(
        .IC_DATA_WIDTH (DATA_WIDTH),
        .IC_ADDR_WIDTH (ADDR_WIDTH),
        .IC_ID_WIDTH   (ID_WIDTH)
    ) uart_bridge (
        .clk    (clk),
        .rst    (rst),
        .s_axi_awid   (bridge_awid),
        .s_axi_awaddr (bridge_awaddr),
        .s_axi_awlen  (bridge_awlen),
        .s_axi_awsize (bridge_awsize),
        .s_axi_awburst(bridge_awburst),
        .s_axi_awlock (bridge_awlock),
        .s_axi_awcache(bridge_awcache),
        .s_axi_awprot (bridge_awprot),
        .s_axi_awqos  (bridge_awqos),
        .s_axi_awvalid(bridge_awvalid),
        .s_axi_awready(bridge_awready),
        .s_axi_wdata  (bridge_wdata),
        .s_axi_wstrb  (bridge_wstrb),
        .s_axi_wlast  (bridge_wlast),
        .s_axi_wvalid (bridge_wvalid),
        .s_axi_wready (bridge_wready),
        .s_axi_bid    (bridge_bid),
        .s_axi_bresp  (bridge_bresp),
        .s_axi_bvalid (bridge_bvalid),
        .s_axi_bready (bridge_bready),
        .s_axi_arid   (bridge_arid),
        .s_axi_araddr (bridge_araddr),
        .s_axi_arlen  (bridge_arlen),
        .s_axi_arsize (bridge_arsize),
        .s_axi_arburst(bridge_arburst),
        .s_axi_arlock (bridge_arlock),
        .s_axi_arcache(bridge_arcache),
        .s_axi_arprot (bridge_arprot),
        .s_axi_arqos  (bridge_arqos),
        .s_axi_arvalid(bridge_arvalid),
        .s_axi_arready(bridge_arready),
        .s_axi_rid    (bridge_rid),
        .s_axi_rdata  (bridge_rdata),
        .s_axi_rresp  (bridge_rresp),
        .s_axi_rlast  (bridge_rlast),
        .s_axi_rvalid (bridge_rvalid),
        .s_axi_rready (bridge_rready),
        .uart_tx_o    (uart_tx_o),
        .uart_rx_i    (uart_rx_i),
        .uart_irq_o   (uart_irq_o)
    );

endmodule