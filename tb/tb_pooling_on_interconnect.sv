// =============================================================================
// File    : tb_pooling_on_interconnect.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Integration testbench — verifies axi_pooling connected to port
//           m03 (0x3000_0000) of the AXI interconnect with dummy slaves on other ports.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module dummy_slave_pool #(
    parameter integer DW  = 64,
    parameter integer AW  = 32,
    parameter integer IDW = 8
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire [IDW-1:0]       awid,
    input  wire [AW-1:0]        awaddr,
    input  wire                 awvalid,
    output reg                  awready,
    input  wire [DW-1:0]        wdata,
    input  wire                 wvalid,
    output reg                  wready,
    output reg  [IDW-1:0]       bid,
    output reg  [1:0]           bresp,
    output reg                  bvalid,
    input  wire                 bready,
    input  wire [IDW-1:0]       arid,
    input  wire [AW-1:0]        araddr,
    input  wire                 arvalid,
    output reg                  arready,
    output reg  [IDW-1:0]       rid,
    output reg  [DW-1:0]        rdata,
    output reg  [1:0]           rresp,
    output reg                  rlast,
    output reg                  rvalid,
    input  wire                 rready
);
    always @(posedge clk) begin
        if (rst) begin
            awready <= 1'b0; wready <= 1'b0; bvalid <= 1'b0;
            arready <= 1'b0; rvalid <= 1'b0; rlast <= 1'b0;
        end else begin
            awready <= awvalid && !awready;
            wready  <= wvalid  && !wready;
            if (awvalid && awready && wvalid && wready) begin
                bid <= awid; bresp <= 2'b00; bvalid <= 1'b1;
            end else if (bvalid && bready) begin
                bvalid <= 1'b0;
            end
            arready <= arvalid && !arready;
            if (arvalid && arready) begin
                rid <= arid; rdata <= 64'hD00D_D00D_D00D_D00D;
                rresp <= 2'b00; rlast <= 1'b1; rvalid <= 1'b1;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0; rlast <= 1'b0;
            end
        end
    end
endmodule

module tb_pooling_on_interconnect;

    localparam DW   = 64;
    localparam AW   = 32;
    localparam IDW  = 8;
    localparam STRBW= DW / 8;
    localparam POOLING_BASE = 32'h3000_0000;

    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk;

    // Master signals (s00)
    logic [IDW-1:0]   m_awid;
    logic [AW-1:0]    m_awaddr;
    logic [7:0]       m_awlen;
    logic [2:0]       m_awsize;
    logic [1:0]       m_awburst;
    logic             m_awlock;
    logic [3:0]       m_awcache;
    logic [2:0]       m_awprot;
    logic [3:0]       m_awqos;
    logic             m_awvalid;
    wire              m_awready;

    logic [DW-1:0]    m_wdata;
    logic [STRBW-1:0] m_wstrb;
    logic             m_wlast;
    logic             m_wvalid;
    wire              m_wready;

    wire [IDW-1:0]    m_bid;
    wire [1:0]        m_bresp;
    wire              m_bvalid;
    logic             m_bready;

    logic [IDW-1:0]   m_arid;
    logic [AW-1:0]    m_araddr;
    logic [7:0]       m_arlen;
    logic [2:0]       m_arsize;
    logic [1:0]       m_arburst;
    logic             m_arlock;
    logic [3:0]       m_arcache;
    logic [2:0]       m_arprot;
    logic [3:0]       m_arqos;
    logic             m_arvalid;
    wire              m_arready;

    wire [IDW-1:0]    m_rid;
    wire [DW-1:0]     m_rdata;
    wire [1:0]        m_rresp;
    wire              m_rlast;
    wire              m_rvalid;
    logic             m_rready;

    // Interconnect master wires (m00..m07)
    wire [IDW-1:0]   ic_m_awid    [0:7];
    wire [AW-1:0]    ic_m_awaddr  [0:7];
    wire [7:0]       ic_m_awlen   [0:7];
    wire [2:0]       ic_m_awsize  [0:7];
    wire [1:0]       ic_m_awburst [0:7];
    wire             ic_m_awlock  [0:7];
    wire [3:0]       ic_m_awcache [0:7];
    wire [2:0]       ic_m_awprot  [0:7];
    wire [3:0]       ic_m_awqos   [0:7];
    wire             ic_m_awvalid [0:7];
    wire             ic_m_awready [0:7];

    wire [DW-1:0]    ic_m_wdata   [0:7];
    wire [STRBW-1:0] ic_m_wstrb   [0:7];
    wire             ic_m_wlast   [0:7];
    wire             ic_m_wvalid  [0:7];
    wire             ic_m_wready  [0:7];

    wire [IDW-1:0]   ic_m_bid     [0:7];
    wire [1:0]       ic_m_bresp   [0:7];
    wire             ic_m_bvalid  [0:7];
    wire             ic_m_bready  [0:7];

    wire [IDW-1:0]   ic_m_arid    [0:7];
    wire [AW-1:0]    ic_m_araddr  [0:7];
    wire [7:0]       ic_m_arlen   [0:7];
    wire [2:0]       ic_m_arsize  [0:7];
    wire [1:0]       ic_m_arburst [0:7];
    wire             ic_m_arlock  [0:7];
    wire [3:0]       ic_m_arcache [0:7];
    wire [2:0]       ic_m_arprot  [0:7];
    wire [3:0]       ic_m_arqos   [0:7];
    wire             ic_m_arvalid [0:7];
    wire             ic_m_arready [0:7];

    wire [IDW-1:0]   ic_m_rid     [0:7];
    wire [DW-1:0]    ic_m_rdata   [0:7];
    wire [1:0]       ic_m_rresp   [0:7];
    wire             ic_m_rlast   [0:7];
    wire             ic_m_rvalid  [0:7];
    wire             ic_m_rready  [0:7];

    // Instantiate 1x8 AXI Interconnect
    axi_interconnect #(
        .S_COUNT         (1),
        .M_COUNT         (8),
        .DATA_WIDTH      (DW),
        .ADDR_WIDTH      (AW),
        .STRB_WIDTH      (STRBW),
        .ID_WIDTH        (IDW),
        .FORWARD_ID      (0),
        .M_REGIONS       (1),
        .M_BASE_ADDR     ({
            32'h7000_0000,  // m07
            32'h6000_0000,  // m06
            32'h5000_0000,  // m05
            32'h4000_0000,  // m04
            POOLING_BASE,   // m03: Pooling Unit (0x3000_0000)
            32'h2000_0000,  // m02
            32'h1000_0000,  // m01
            32'h8000_0000   // m00
        }),
        .M_ADDR_WIDTH    ({
            {32{32'd28}}, {32{32'd28}}, {32{32'd28}}, {32{32'd28}},
            {32{32'd28}}, {32{32'd28}}, {32{32'd28}}, {32{32'd28}}
        }),
        .M_CONNECT_READ  ({8{1'b1}}),
        .M_CONNECT_WRITE ({8{1'b1}})
    ) ic_inst (
        .clk (clk),
        .rst (rst),

        .s_axi_awid   (m_awid),
        .s_axi_awaddr (m_awaddr),
        .s_axi_awlen  (m_awlen),
        .s_axi_awsize (m_awsize),
        .s_axi_awburst(m_awburst),
        .s_axi_awlock (m_awlock),
        .s_axi_awcache(m_awcache),
        .s_axi_awprot (m_awprot),
        .s_axi_awqos  (m_awqos),
        .s_axi_awvalid(m_awvalid),
        .s_axi_awready(m_awready),

        .s_axi_wdata  (m_wdata),
        .s_axi_wstrb  (m_wstrb),
        .s_axi_wlast  (m_wlast),
        .s_axi_wvalid (m_wvalid),
        .s_axi_wready (m_wready),

        .s_axi_bid    (m_bid),
        .s_axi_bresp  (m_bresp),
        .s_axi_bvalid (m_bvalid),
        .s_axi_bready (m_bready),

        .s_axi_arid   (m_arid),
        .s_axi_araddr (m_araddr),
        .s_axi_arlen  (m_arlen),
        .s_axi_arsize (m_arsize),
        .s_axi_arburst(m_arburst),
        .s_axi_arlock (m_arlock),
        .s_axi_arcache(m_arcache),
        .s_axi_arprot (m_arprot),
        .s_axi_arqos  (m_arqos),
        .s_axi_arvalid(m_arvalid),
        .s_axi_arready(m_arready),

        .s_axi_rid    (m_rid),
        .s_axi_rdata  (m_rdata),
        .s_axi_rresp  (m_rresp),
        .s_axi_rlast  (m_rlast),
        .s_axi_rvalid (m_rvalid),
        .s_axi_rready (m_rready),

        .m_axi_awid   ({ic_m_awid[7],   ic_m_awid[6],   ic_m_awid[5],   ic_m_awid[4],
                        ic_m_awid[3],   ic_m_awid[2],   ic_m_awid[1],   ic_m_awid[0]}),
        .m_axi_awaddr ({ic_m_awaddr[7], ic_m_awaddr[6], ic_m_awaddr[5], ic_m_awaddr[4],
                        ic_m_awaddr[3], ic_m_awaddr[2], ic_m_awaddr[1], ic_m_awaddr[0]}),
        .m_axi_awlen  ({ic_m_awlen[7],  ic_m_awlen[6],  ic_m_awlen[5],  ic_m_awlen[4],
                        ic_m_awlen[3],  ic_m_awlen[2],  ic_m_awlen[1],  ic_m_awlen[0]}),
        .m_axi_awsize ({ic_m_awsize[7], ic_m_awsize[6], ic_m_awsize[5], ic_m_awsize[4],
                        ic_m_awsize[3], ic_m_awsize[2], ic_m_awsize[1], ic_m_awsize[0]}),
        .m_axi_awburst({ic_m_awburst[7],ic_m_awburst[6],ic_m_awburst[5],ic_m_awburst[4],
                        ic_m_awburst[3],ic_m_awburst[2],ic_m_awburst[1],ic_m_awburst[0]}),
        .m_axi_awlock ({ic_m_awlock[7], ic_m_awlock[6], ic_m_awlock[5], ic_m_awlock[4],
                        ic_m_awlock[3], ic_m_awlock[2], ic_m_awlock[1], ic_m_awlock[0]}),
        .m_axi_awcache({ic_m_awcache[7],ic_m_awcache[6],ic_m_awcache[5],ic_m_awcache[4],
                        ic_m_awcache[3],ic_m_awcache[2],ic_m_awcache[1],ic_m_awcache[0]}),
        .m_axi_awprot ({ic_m_awprot[7], ic_m_awprot[6], ic_m_awprot[5], ic_m_awprot[4],
                        ic_m_awprot[3], ic_m_awprot[2], ic_m_awprot[1], ic_m_awprot[0]}),
        .m_axi_awqos  ({ic_m_awqos[7],  ic_m_awqos[6],  ic_m_awqos[5],  ic_m_awqos[4],
                        ic_m_awqos[3],  ic_m_awqos[2],  ic_m_awqos[1],  ic_m_awqos[0]}),
        .m_axi_awvalid({ic_m_awvalid[7],ic_m_awvalid[6],ic_m_awvalid[5],ic_m_awvalid[4],
                        ic_m_awvalid[3],ic_m_awvalid[2],ic_m_awvalid[1],ic_m_awvalid[0]}),
        .m_axi_awready({ic_m_awready[7],ic_m_awready[6],ic_m_awready[5],ic_m_awready[4],
                        ic_m_awready[3],ic_m_awready[2],ic_m_awready[1],ic_m_awready[0]}),

        .m_axi_wdata  ({ic_m_wdata[7],  ic_m_wdata[6],  ic_m_wdata[5],  ic_m_wdata[4],
                        ic_m_wdata[3],  ic_m_wdata[2],  ic_m_wdata[1],  ic_m_wdata[0]}),
        .m_axi_wstrb  ({ic_m_wstrb[7],  ic_m_wstrb[6],  ic_m_wstrb[5],  ic_m_wstrb[4],
                        ic_m_wstrb[3],  ic_m_wstrb[2],  ic_m_wstrb[1],  ic_m_wstrb[0]}),
        .m_axi_wlast  ({ic_m_wlast[7],  ic_m_wlast[6],  ic_m_wlast[5],  ic_m_wlast[4],
                        ic_m_wlast[3],  ic_m_wlast[2],  ic_m_wlast[1],  ic_m_wlast[0]}),
        .m_axi_wvalid ({ic_m_wvalid[7], ic_m_wvalid[6], ic_m_wvalid[5], ic_m_wvalid[4],
                        ic_m_wvalid[3], ic_m_wvalid[2], ic_m_wvalid[1], ic_m_wvalid[0]}),
        .m_axi_wready ({ic_m_wready[7], ic_m_wready[6], ic_m_wready[5], ic_m_wready[4],
                        ic_m_wready[3], ic_m_wready[2], ic_m_wready[1], ic_m_wready[0]}),

        .m_axi_bid    ({ic_m_bid[7],    ic_m_bid[6],    ic_m_bid[5],    ic_m_bid[4],
                        ic_m_bid[3],    ic_m_bid[2],    ic_m_bid[1],    ic_m_bid[0]}),
        .m_axi_bresp  ({ic_m_bresp[7],  ic_m_bresp[6],  ic_m_bresp[5],  ic_m_bresp[4],
                        ic_m_bresp[3],  ic_m_bresp[2],  ic_m_bresp[1],  ic_m_bresp[0]}),
        .m_axi_bvalid ({ic_m_bvalid[7], ic_m_bvalid[6], ic_m_bvalid[5], ic_m_bvalid[4],
                        ic_m_bvalid[3], ic_m_bvalid[2], ic_m_bvalid[1], ic_m_bvalid[0]}),
        .m_axi_bready ({ic_m_bready[7], ic_m_bready[6], ic_m_bready[5], ic_m_bready[4],
                        ic_m_bready[3], ic_m_bready[2], ic_m_bready[1], ic_m_bready[0]}),

        .m_axi_arid   ({ic_m_arid[7],   ic_m_arid[6],   ic_m_arid[5],   ic_m_arid[4],
                        ic_m_arid[3],   ic_m_arid[2],   ic_m_arid[1],   ic_m_arid[0]}),
        .m_axi_araddr ({ic_m_araddr[7], ic_m_araddr[6], ic_m_araddr[5], ic_m_araddr[4],
                        ic_m_araddr[3], ic_m_araddr[2], ic_m_araddr[1], ic_m_araddr[0]}),
        .m_axi_arlen  ({ic_m_arlen[7],  ic_m_arlen[6],  ic_m_arlen[5],  ic_m_arlen[4],
                        ic_m_arlen[3],  ic_m_arlen[2],  ic_m_arlen[1],  ic_m_arlen[0]}),
        .m_axi_arsize ({ic_m_arsize[7], ic_m_arsize[6], ic_m_arsize[5], ic_m_arsize[4],
                        ic_m_arsize[3], ic_m_arsize[2], ic_m_arsize[1], ic_m_arsize[0]}),
        .m_axi_arburst({ic_m_arburst[7],ic_m_arburst[6],ic_m_arburst[5],ic_m_arburst[4],
                        ic_m_arburst[3],ic_m_arburst[2],ic_m_arburst[1],ic_m_arburst[0]}),
        .m_axi_arlock ({ic_m_arlock[7], ic_m_arlock[6], ic_m_arlock[5], ic_m_arlock[4],
                        ic_m_arlock[3], ic_m_arlock[2], ic_m_arlock[1], ic_m_arlock[0]}),
        .m_axi_arcache({ic_m_arcache[7],ic_m_arcache[6],ic_m_arcache[5],ic_m_arcache[4],
                        ic_m_arcache[3],ic_m_arcache[2],ic_m_arcache[1],ic_m_arcache[0]}),
        .m_axi_arprot ({ic_m_arprot[7], ic_m_arprot[6], ic_m_arprot[5], ic_m_arprot[4],
                        ic_m_arprot[3], ic_m_arprot[2], ic_m_arprot[1], ic_m_arprot[0]}),
        .m_axi_arqos  ({ic_m_arqos[7],  ic_m_arqos[6],  ic_m_arqos[5],  ic_m_arqos[4],
                        ic_m_arqos[3],  ic_m_arqos[2],  ic_m_arqos[1],  ic_m_arqos[0]}),
        .m_axi_arvalid({ic_m_arvalid[7],ic_m_arvalid[6],ic_m_arvalid[5],ic_m_arvalid[4],
                        ic_m_arvalid[3],ic_m_arvalid[2],ic_m_arvalid[1],ic_m_arvalid[0]}),
        .m_axi_arready({ic_m_arready[7],ic_m_arready[6],ic_m_arready[5],ic_m_arready[4],
                        ic_m_arready[3],ic_m_arready[2],ic_m_arready[1],ic_m_arready[0]}),

        .m_axi_rid    ({ic_m_rid[7],    ic_m_rid[6],    ic_m_rid[5],    ic_m_rid[4],
                        ic_m_rid[3],    ic_m_rid[2],    ic_m_rid[1],    ic_m_rid[0]}),
        .m_axi_rdata  ({ic_m_rdata[7],  ic_m_rdata[6],  ic_m_rdata[5],  ic_m_rdata[4],
                        ic_m_rdata[3],  ic_m_rdata[2],  ic_m_rdata[1],  ic_m_rdata[0]}),
        .m_axi_rresp  ({ic_m_rresp[7],  ic_m_rresp[6],  ic_m_rresp[5],  ic_m_rresp[4],
                        ic_m_rresp[3],  ic_m_rresp[2],  ic_m_rresp[1],  ic_m_rresp[0]}),
        .m_axi_rlast  ({ic_m_rlast[7],  ic_m_rlast[6],  ic_m_rlast[5],  ic_m_rlast[4],
                        ic_m_rlast[3],  ic_m_rlast[2],  ic_m_rlast[1],  ic_m_rlast[0]}),
        .m_axi_rvalid ({ic_m_rvalid[7], ic_m_rvalid[6], ic_m_rvalid[5], ic_m_rvalid[4],
                        ic_m_rvalid[3], ic_m_rvalid[2], ic_m_rvalid[1], ic_m_rvalid[0]}),
        .m_axi_rready ({ic_m_rready[7], ic_m_rready[6], ic_m_rready[5], ic_m_rready[4],
                        ic_m_rready[3], ic_m_rready[2], ic_m_rready[1], ic_m_rready[0]})
    );

    // Dummy slaves on all ports except m03
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : gen_slaves
            if (gi != 3) begin : gen_dummy
                dummy_slave_pool #(.DW(DW), .AW(AW), .IDW(IDW)) u_dummy (
                    .clk    (clk),
                    .rst    (rst),
                    .awid   (ic_m_awid[gi]),
                    .awaddr (ic_m_awaddr[gi]),
                    .awvalid(ic_m_awvalid[gi]),
                    .awready(ic_m_awready[gi]),
                    .wdata  (ic_m_wdata[gi]),
                    .wvalid (ic_m_wvalid[gi]),
                    .wready (ic_m_wready[gi]),
                    .bid    (ic_m_bid[gi]),
                    .bresp  (ic_m_bresp[gi]),
                    .bvalid (ic_m_bvalid[gi]),
                    .bready (ic_m_bready[gi]),
                    .arid   (ic_m_arid[gi]),
                    .araddr (ic_m_araddr[gi]),
                    .arvalid(ic_m_arvalid[gi]),
                    .arready(ic_m_arready[gi]),
                    .rid    (ic_m_rid[gi]),
                    .rdata  (ic_m_rdata[gi]),
                    .rresp  (ic_m_rresp[gi]),
                    .rlast  (ic_m_rlast[gi]),
                    .rvalid (ic_m_rvalid[gi]),
                    .rready (ic_m_rready[gi])
                );
            end
        end
    endgenerate

    // m03 = axi_pooling
    axi_pooling #(
        .AXI_DATA_WIDTH      (DW),
        .AXI_ADDR_WIDTH      (AW),
        .AXI_ID_WIDTH        (IDW),
        .BASE_ADDR           (POOLING_BASE)
    ) u_pool (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (ic_m_awid[3]),
        .s_axi_awaddr        (ic_m_awaddr[3]),
        .s_axi_awlen         (ic_m_awlen[3]),
        .s_axi_awsize        (ic_m_awsize[3]),
        .s_axi_awburst       (ic_m_awburst[3]),
        .s_axi_awlock        (ic_m_awlock[3]),
        .s_axi_awcache       (ic_m_awcache[3]),
        .s_axi_awprot        (ic_m_awprot[3]),
        .s_axi_awqos         (ic_m_awqos[3]),
        .s_axi_awregion      (4'b0),
        .s_axi_awvalid       (ic_m_awvalid[3]),
        .s_axi_awready       (ic_m_awready[3]),
        .s_axi_wdata         (ic_m_wdata[3]),
        .s_axi_wstrb         (ic_m_wstrb[3]),
        .s_axi_wlast         (ic_m_wlast[3]),
        .s_axi_wvalid        (ic_m_wvalid[3]),
        .s_axi_wready        (ic_m_wready[3]),
        .s_axi_bid           (ic_m_bid[3]),
        .s_axi_bresp         (ic_m_bresp[3]),
        .s_axi_bvalid        (ic_m_bvalid[3]),
        .s_axi_bready        (ic_m_bready[3]),
        .s_axi_arid          (ic_m_arid[3]),
        .s_axi_araddr        (ic_m_araddr[3]),
        .s_axi_arlen         (ic_m_arlen[3]),
        .s_axi_arsize        (ic_m_arsize[3]),
        .s_axi_arburst       (ic_m_arburst[3]),
        .s_axi_arlock        (ic_m_arlock[3]),
        .s_axi_arcache       (ic_m_arcache[3]),
        .s_axi_arprot        (ic_m_arprot[3]),
        .s_axi_arqos         (ic_m_arqos[3]),
        .s_axi_arregion      (4'b0),
        .s_axi_arvalid       (ic_m_arvalid[3]),
        .s_axi_arready       (ic_m_arready[3]),
        .s_axi_rid           (ic_m_rid[3]),
        .s_axi_rdata         (ic_m_rdata[3]),
        .s_axi_rresp         (ic_m_rresp[3]),
        .s_axi_rlast         (ic_m_rlast[3]),
        .s_axi_rvalid        (ic_m_rvalid[3]),
        .s_axi_rready        (ic_m_rready[3])
    );

    integer pass_count = 0;
    integer fail_count = 0;

    task check(input string desc, input logic cond);
        if (cond) begin
            $display("  [PASS] %s", desc);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %s", desc);
            fail_count = fail_count + 1;
        end
    endtask

    task axi_write(input [AW-1:0] addr, input [31:0] data, input [7:0] strb, output [1:0] resp);
        begin
            @(posedge clk); #1;
            m_awaddr  <= addr;
            m_awid    <= 8'h11;
            m_awlen   <= 8'h00;
            m_awsize  <= 3'b011; // 8 bytes
            m_awburst <= 2'b01;  // INCR
            m_awlock  <= 1'b0;
            m_awcache <= 4'b0;
            m_awprot  <= 3'b0;
            m_awqos   <= 4'b0;
            m_awvalid <= 1'b1;

            m_wdata   <= {data, data};
            m_wstrb   <= strb;
            m_wlast   <= 1'b1;
            m_wvalid  <= 1'b1;
            m_bready  <= 1'b1;

            fork
                begin : aw_wait
                    wait (m_awready);
                    @(posedge clk); #1;
                    m_awvalid <= 1'b0;
                end
                begin : w_wait
                    wait (m_wready);
                    @(posedge clk); #1;
                    m_wvalid  <= 1'b0;
                    m_wlast   <= 1'b0;
                end
            join

            wait (m_bvalid);
            resp = m_bresp;
            @(posedge clk); #1;
            m_bready <= 1'b0;

            repeat (2) @(posedge clk);
        end
    endtask

    task axi_read(input [AW-1:0] addr, output [31:0] data, output [1:0] resp);
        begin
            @(posedge clk); #1;
            m_araddr  <= addr;
            m_arid    <= 8'h22;
            m_arlen   <= 8'h00;
            m_arsize  <= 3'b011;
            m_arburst <= 2'b01;
            m_arlock  <= 1'b0;
            m_arcache <= 4'b0;
            m_arprot  <= 3'b0;
            m_arqos   <= 4'b0;
            m_arvalid <= 1'b1;
            m_rready  <= 1'b1;

            wait (m_arready);
            @(posedge clk); #1;
            m_arvalid <= 1'b0;

            wait (m_rvalid);
            data = m_rdata[31:0];
            resp = m_rresp;
            @(posedge clk); #1;
            m_rready <= 1'b0;

            repeat (2) @(posedge clk);
        end
    endtask

    logic [31:0] rdata;
    logic [1:0]  resp;

    initial begin
        m_awvalid = 0; m_wvalid = 0; m_bready = 0;
        m_arvalid = 0; m_rready = 0;
        m_awaddr = 0; m_wdata = 0; m_wstrb = 0;
        m_araddr = 0;
        rst = 1'b1;

        $display("============================================================");
        $display("  tb_pooling_on_interconnect: Pooling Integration TB");
        $display("============================================================");

        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // --- T1: Dummy Slave Access ---
        $display("\n--- T1: Dummy Slave Access (Port 0: 0x8000_0000) ---");
        axi_read(32'h8000_0000, rdata, resp);
        check("T1: Dummy slave port 0 returns D00DD00D", rdata == 32'hD00D_D00D && resp == 2'b00);

        // --- T2: Read ID & Version through interconnect ---
        $display("\n--- T2: Read ID & Version of Pooling Unit (Port 3) ---");
        axi_read(POOLING_BASE + 32'h18, rdata, resp);
        check("T2: Read ID = 0x504F4F4C ('POOL')", rdata == 32'h504F4F4C && resp == 2'b00);

        axi_read(POOLING_BASE + 32'h1C, rdata, resp);
        check("T2: Read VERSION = 0x00010000", rdata == 32'h00010000 && resp == 2'b00);

        // --- T3: Read Default CTRL & STATUS ---
        $display("\n--- T3: Read Default Status ---");
        axi_read(POOLING_BASE + 32'h00, rdata, resp);
        check("T3: Default CTRL = 0x01 (mode=00, enable=1)", rdata == 32'h01 && resp == 2'b00);

        axi_read(POOLING_BASE + 32'h04, rdata, resp);
        check("T3: Default STATUS = 0x00 (idle)", rdata == 32'h00 && resp == 2'b00);

        // --- T4: Functional Max-Pooling over Interconnect ---
        $display("\n--- T4: Functional Max-Pooling over Interconnect ---");
        // Quad = {p0=10, p1=50, p2=30, p3=20} -> expected max=50, idx=1
        axi_write(POOLING_BASE + 32'h0C, {16'sd50, 16'sd10}, 8'h0F, resp);
        check("T4: Write REG_DATA_IN_P01 OKAY", resp == 2'b00);

        axi_write(POOLING_BASE + 32'h10, {16'sd20, 16'sd30}, 8'h0F, resp);
        check("T4: Write REG_DATA_IN_P23 (trigger) OKAY", resp == 2'b00);

        repeat (3) @(posedge clk);
        axi_read(POOLING_BASE + 32'h14, rdata, resp);
        check("T4: Read pooled max result = 50", rdata[15:0] == 16'sd50);
        check("T4: Read argmax index = 1", rdata[17:16] == 2'd1);

        // --- T5: Functional Average-Pooling over Interconnect ---
        $display("\n--- T5: Functional Average-Pooling over Interconnect ---");
        // Set mode=AVG (01), enable=1 -> CTRL = 3
        axi_write(POOLING_BASE + 32'h00, 32'h03, 8'h0F, resp);
        check("T5: Set CTRL mode=AVG OKAY", resp == 2'b00);

        // Quad = {p0=1, p1=2, p2=3, p3=4} -> sum=10, rnd=12, >>2 = 3
        axi_write(POOLING_BASE + 32'h0C, {16'sd2, 16'sd1}, 8'h0F, resp);
        axi_write(POOLING_BASE + 32'h10, {16'sd4, 16'sd3}, 8'h0F, resp);

        repeat (3) @(posedge clk);
        axi_read(POOLING_BASE + 32'h14, rdata, resp);
        check("T5: Read pooled avg result = 3", rdata[15:0] == 16'sd3);

        // --- T6: Functional Min-Pooling over Interconnect ---
        $display("\n--- T6: Functional Min-Pooling over Interconnect ---");
        // Set mode=MIN (10), enable=1 -> CTRL = 5
        axi_write(POOLING_BASE + 32'h00, 32'h05, 8'h0F, resp);
        check("T6: Set CTRL mode=MIN OKAY", resp == 2'b00);

        // Quad = {p0=10, p1=50, p2=30, p3=20} -> expected min=10
        axi_write(POOLING_BASE + 32'h0C, {16'sd50, 16'sd10}, 8'h0F, resp);
        axi_write(POOLING_BASE + 32'h10, {16'sd20, 16'sd30}, 8'h0F, resp);

        repeat (3) @(posedge clk);
        axi_read(POOLING_BASE + 32'h14, rdata, resp);
        check("T6: Read pooled min result = 10", rdata[15:0] == 16'sd10);

        // --- T7: Interconnect Error Handling ---
        $display("\n--- T7: Interconnect Error Handling ---");
        axi_write(POOLING_BASE + 32'h18, 32'hDEADBEEF, 8'h0F, resp);
        check("T7: Write to RO ID register produces SLVERR", resp == 2'b10);

        // --- Summary ---
        $display("\n============================================================");
        $display("  Pooling Interconnect Integration Verification Summary");
        $display("  Total Passed: %0d", pass_count);
        $display("  Total Failed: %0d", fail_count);
        $display("============================================================");

        if (fail_count == 0 && pass_count >= 12) begin
            $display("\n*** ALL POOLING INTERCONNECT TESTS PASSED ***\n");
        end else begin
            $display("\n*** TEST FAILED ***\n");
        end

        $finish;
    end

endmodule

`default_nettype wire
