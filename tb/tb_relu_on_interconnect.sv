// =============================================================================
// File    : tb_relu_on_interconnect.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : System-level integration testbench for AXI ReLU Activation Unit
//           integrated onto axi_interconnect_wrap_3x8.
//
// Topology:
//   Interconnect : axi_interconnect_wrap_3x8 (3 Master ports x 8 Slave ports)
//   Master s00   : Testbench CPU stimulus driver (32/64-bit transactions)
//   Master s01   : Idle (tied off)
//   Master s02   : Idle (tied off)
//
//   Slave m00    : Dummy Slave 0  (0x0000_0000 .. 0x0FFF_FFFF)
//   Slave m01    : Dummy Slave 1  (0x1000_0000 .. 0x1FFF_FFFF)
//   Slave m02    : Dummy Slave 2  (0x2000_0000 .. 0x2FFF_FFFF)
//   Slave m03    : axi_relu DUT   (0x3000_0000 .. 0x3000_0FFF, 4 KiB)
//   Slave m04    : Dummy Slave 3  (0x4000_0000 .. 0x4FFF_FFFF)
//   Slave m05    : Dummy Slave 4  (0x5000_0000 .. 0x5FFF_FFFF)
//   Slave m06    : Dummy Slave 5  (0x6000_0000 .. 0x6FFF_FFFF)
//   Slave m07    : Dummy Slave 6  (0x7000_0000 .. 0x7FFF_FFFF)
//
// Test Suite:
//   T1  : Reset and peripheral identification (RELU_ID = "RELU", VER = v1.0)
//   T2  : Control and threshold register configuration
//   T3  : Standard positive ReLU activations
//   T4  : Negative input clamping & zero handling
//   T5  : Extreme boundary values (INT32_MAX, INT32_MIN, -1)
//   T6  : Bypass mode linear passthrough
//   T7  : Saturated clipping activation (ReLU6 / threshold saturation)
//   T8  : Sparsity & metrics counters with reset_stats pulse
//   T9  : Read-only register write protection (returns SLVERR)
//   T10 : Address error handling (reserved offset, out-of-aperture DECERR, misalignment)
//   T11 : Byte strobe (WSTRB) partial writes
//   T12 : Interconnect multi-slave access routing & independence
// =============================================================================

`timescale 1ns/1ps
`default_nettype none

// =============================================================================
// Lightweight Dummy AXI4 Slave for unused interconnect ports
// =============================================================================
module dummy_slave_relu_intg #(
    parameter integer DW  = 64,
    parameter integer AW  = 32,
    parameter integer IDW = 8
)(
    input  wire              clk,
    input  wire              rst,

    // AW
    input  wire [IDW-1:0]    awid,
    input  wire [AW-1:0]     awaddr,
    input  wire [7:0]        awlen,
    input  wire [2:0]        awsize,
    input  wire [1:0]        awburst,
    input  wire              awlock,
    input  wire [3:0]        awcache,
    input  wire [2:0]        awprot,
    input  wire [3:0]        awqos,
    input  wire [3:0]        awregion,
    input  wire              awvalid,
    output reg               awready,
    // W
    input  wire [DW-1:0]     wdata,
    input  wire [DW/8-1:0]   wstrb,
    input  wire              wlast,
    input  wire              wvalid,
    output reg               wready,
    // B
    output reg  [IDW-1:0]    bid,
    output reg  [1:0]        bresp,
    output reg               bvalid,
    input  wire              bready,
    // AR
    input  wire [IDW-1:0]    arid,
    input  wire [AW-1:0]     araddr,
    input  wire [7:0]        arlen,
    input  wire [2:0]        arsize,
    input  wire [1:0]        arburst,
    input  wire              arlock,
    input  wire [3:0]        arcache,
    input  wire [2:0]        arprot,
    input  wire [3:0]        arqos,
    input  wire [3:0]        arregion,
    input  wire              arvalid,
    output reg               arready,
    // R
    output reg  [IDW-1:0]    rid,
    output reg  [DW-1:0]     rdata,
    output reg  [1:0]        rresp,
    output reg               rlast,
    output reg               rvalid,
    input  wire              rready
);
    reg              aw_pend;
    reg [IDW-1:0]    aw_id_r;

    initial begin
        awready = 1'b0; wready = 1'b0; bvalid = 1'b0;
        bid = '0; bresp = 2'b00;
        arready = 1'b0; rvalid = 1'b0; rlast = 1'b0;
        rid = '0; rdata = '0; rresp = 2'b00;
        aw_pend = 1'b0; aw_id_r = '0;
    end

    always @(posedge clk) begin
        if (rst) begin
            awready <= 1'b1; wready <= 1'b1; bvalid <= 1'b0;
            arready <= 1'b0; rvalid <= 1'b0; rlast  <= 1'b0;
            aw_pend <= 1'b0;
        end else begin
            awready <= !aw_pend && !bvalid;
            wready  <= 1'b1;

            if (awvalid && awready) begin
                aw_pend <= 1'b1;
                aw_id_r <= awid;
            end

            if (aw_pend && wvalid && wready && !bvalid) begin
                bvalid  <= 1'b1;
                bid     <= aw_id_r;
                bresp   <= 2'b00;
                aw_pend <= 1'b0;
            end else if (bvalid && bready) begin
                bvalid  <= 1'b0;
            end

            arready <= !rvalid;
            if (arvalid && arready) begin
                rvalid <= 1'b1;
                rid    <= arid;
                rdata  <= 64'hA5A5_5A5A_0000_0000 | araddr;
                rresp  <= 2'b00;
                rlast  <= 1'b1;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
            end
        end
    end
endmodule

// =============================================================================
// Top-Level Interconnect Testbench for ReLU Unit
// =============================================================================
module tb_relu_on_interconnect;

    // Bus parameters
    localparam integer DW         = 64;
    localparam integer AW         = 32;
    localparam integer IDW        = 8;
    localparam integer SW         = DW / 8;
    localparam integer CLK_PERIOD = 10;

    // Memory map
    localparam [AW-1:0] RELU_BASE = 32'h3000_0000;

    localparam [AW-1:0] REG_ADDR_CTRL     = RELU_BASE + 32'h00;
    localparam [AW-1:0] REG_ADDR_STATUS   = RELU_BASE + 32'h04;
    localparam [AW-1:0] REG_ADDR_DIN      = RELU_BASE + 32'h08;
    localparam [AW-1:0] REG_ADDR_DOUT     = RELU_BASE + 32'h0C;
    localparam [AW-1:0] REG_ADDR_CLIP_MAX = RELU_BASE + 32'h10;
    localparam [AW-1:0] REG_ADDR_ZERO_CNT = RELU_BASE + 32'h14;
    localparam [AW-1:0] REG_ADDR_TOT_CNT  = RELU_BASE + 32'h18;
    localparam [AW-1:0] REG_ADDR_ID       = RELU_BASE + 32'h1C;
    localparam [AW-1:0] REG_ADDR_VER      = RELU_BASE + 32'h20;

    localparam [31:0] RELU_ID_VALUE      = 32'h5245_4C55;  // "RELU"
    localparam [31:0] RELU_VERSION_VALUE = 32'h0001_0000;  // v1.0

    // AXI response codes
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // ---------------------------------------------------------
    // Clock / Reset
    // ---------------------------------------------------------
    reg clk;
    reg rst;   // active-high

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ---------------------------------------------------------
    // TB AXI Master Signals (drives interconnect s00)
    // ---------------------------------------------------------
    reg  [IDW-1:0]  m_awid;
    reg  [AW-1:0]   m_awaddr;
    reg  [7:0]      m_awlen;
    reg  [2:0]      m_awsize;
    reg  [1:0]      m_awburst;
    reg             m_awlock;
    reg  [3:0]      m_awcache;
    reg  [2:0]      m_awprot;
    reg  [3:0]      m_awqos;
    reg  [0:0]      m_awuser;
    reg             m_awvalid;
    wire            m_awready;

    reg  [DW-1:0]   m_wdata;
    reg  [SW-1:0]   m_wstrb;
    reg             m_wlast;
    reg  [0:0]      m_wuser;
    reg             m_wvalid;
    wire            m_wready;

    wire [IDW-1:0]  m_bid;
    wire [1:0]      m_bresp;
    wire [0:0]      m_buser;
    wire            m_bvalid;
    reg             m_bready;

    reg  [IDW-1:0]  m_arid;
    reg  [AW-1:0]   m_araddr;
    reg  [7:0]      m_arlen;
    reg  [2:0]      m_arsize;
    reg  [1:0]      m_arburst;
    reg             m_arlock;
    reg  [3:0]      m_arcache;
    reg  [2:0]      m_arprot;
    reg  [3:0]      m_arqos;
    reg  [0:0]      m_aruser;
    reg             m_arvalid;
    wire            m_arready;

    wire [IDW-1:0]  m_rid;
    wire [DW-1:0]   m_rdata;
    wire [1:0]      m_rresp;
    wire            m_rlast;
    wire [0:0]      m_ruser;
    wire            m_rvalid;
    reg             m_rready;

    // Last write response recorded by task
    reg [1:0]       last_write_resp;

    // ---------------------------------------------------------
    // Idle master ports s01 and s02
    // ---------------------------------------------------------
    wire [IDW-1:0]  s1_awid  = '0; wire [AW-1:0]  s1_awaddr  = '0;
    wire [7:0]      s1_awlen = '0; wire [2:0]      s1_awsize  = 3'b011;
    wire [1:0]      s1_awburst= 2'b01; wire         s1_awlock  = 1'b0;
    wire [3:0]      s1_awcache= '0; wire [2:0]     s1_awprot  = '0;
    wire [3:0]      s1_awqos = '0; wire [0:0]      s1_awuser  = '0;
    wire            s1_awvalid= 1'b0;
    wire [DW-1:0]   s1_wdata = '0; wire [SW-1:0]   s1_wstrb   = '0;
    wire            s1_wlast = 1'b0; wire [0:0]    s1_wuser   = '0;
    wire            s1_wvalid= 1'b0; wire          s1_bready  = 1'b0;
    wire [IDW-1:0]  s1_arid  = '0; wire [AW-1:0]  s1_araddr  = '0;
    wire [7:0]      s1_arlen = '0; wire [2:0]      s1_arsize  = 3'b011;
    wire [1:0]      s1_arburst= 2'b01; wire         s1_arlock  = 1'b0;
    wire [3:0]      s1_arcache= '0; wire [2:0]     s1_arprot  = '0;
    wire [3:0]      s1_arqos = '0; wire [0:0]      s1_aruser  = '0;
    wire            s1_arvalid= 1'b0; wire          s1_rready  = 1'b0;

    wire [IDW-1:0]  s2_awid  = '0; wire [AW-1:0]  s2_awaddr  = '0;
    wire [7:0]      s2_awlen = '0; wire [2:0]      s2_awsize  = 3'b011;
    wire [1:0]      s2_awburst= 2'b01; wire         s2_awlock  = 1'b0;
    wire [3:0]      s2_awcache= '0; wire [2:0]     s2_awprot  = '0;
    wire [3:0]      s2_awqos = '0; wire [0:0]      s2_awuser  = '0;
    wire            s2_awvalid= 1'b0;
    wire [DW-1:0]   s2_wdata = '0; wire [SW-1:0]   s2_wstrb   = '0;
    wire            s2_wlast = 1'b0; wire [0:0]    s2_wuser   = '0;
    wire            s2_wvalid= 1'b0; wire          s2_bready  = 1'b0;
    wire [IDW-1:0]  s2_arid  = '0; wire [AW-1:0]  s2_araddr  = '0;
    wire [7:0]      s2_arlen = '0; wire [2:0]      s2_arsize  = 3'b011;
    wire [1:0]      s2_arburst= 2'b01; wire         s2_arlock  = 1'b0;
    wire [3:0]      s2_arcache= '0; wire [2:0]     s2_arprot  = '0;
    wire [3:0]      s2_arqos = '0; wire [0:0]      s2_aruser  = '0;
    wire            s2_arvalid= 1'b0; wire          s2_rready  = 1'b0;

    // ---------------------------------------------------------
    // Interconnect master port m03 (connected to axi_relu)
    // ---------------------------------------------------------
    wire [IDW-1:0]  ic_relu_awid;
    wire [AW-1:0]   ic_relu_awaddr;
    wire [7:0]      ic_relu_awlen;
    wire [2:0]      ic_relu_awsize;
    wire [1:0]      ic_relu_awburst;
    wire            ic_relu_awlock;
    wire [3:0]      ic_relu_awcache;
    wire [2:0]      ic_relu_awprot;
    wire [3:0]      ic_relu_awqos;
    wire [3:0]      ic_relu_awregion;
    wire [0:0]      ic_relu_awuser;
    wire            ic_relu_awvalid;
    wire            ic_relu_awready;

    wire [DW-1:0]   ic_relu_wdata;
    wire [SW-1:0]   ic_relu_wstrb;
    wire            ic_relu_wlast;
    wire [0:0]      ic_relu_wuser;
    wire            ic_relu_wvalid;
    wire            ic_relu_wready;

    wire [IDW-1:0]  ic_relu_bid;
    wire [1:0]      ic_relu_bresp;
    wire [0:0]      ic_relu_buser;
    wire            ic_relu_bvalid;
    wire            ic_relu_bready;

    wire [IDW-1:0]  ic_relu_arid;
    wire [AW-1:0]   ic_relu_araddr;
    wire [7:0]      ic_relu_arlen;
    wire [2:0]      ic_relu_arsize;
    wire [1:0]      ic_relu_arburst;
    wire            ic_relu_arlock;
    wire [3:0]      ic_relu_arcache;
    wire [2:0]      ic_relu_arprot;
    wire [3:0]      ic_relu_arqos;
    wire [3:0]      ic_relu_arregion;
    wire [0:0]      ic_relu_aruser;
    wire            ic_relu_arvalid;
    wire            ic_relu_arready;

    wire [IDW-1:0]  ic_relu_rid;
    wire [DW-1:0]   ic_relu_rdata;
    wire [1:0]      ic_relu_rresp;
    wire            ic_relu_rlast;
    wire [0:0]      ic_relu_ruser;
    wire            ic_relu_rvalid;
    wire            ic_relu_rready;

    // ---------------------------------------------------------
    // Dummy slave wires for 7 unused ports (m00,m01,m02,m04,m05,m06,m07)
    // ---------------------------------------------------------
    wire [IDW-1:0]  ds_awid    [0:6];
    wire [AW-1:0]   ds_awaddr  [0:6];
    wire [7:0]      ds_awlen   [0:6];
    wire [2:0]      ds_awsize  [0:6];
    wire [1:0]      ds_awburst [0:6];
    wire            ds_awlock  [0:6];
    wire [3:0]      ds_awcache [0:6];
    wire [2:0]      ds_awprot  [0:6];
    wire [3:0]      ds_awqos   [0:6];
    wire [3:0]      ds_awregion[0:6];
    wire [0:0]      ds_awuser  [0:6];
    wire            ds_awvalid [0:6];
    wire            ds_awready [0:6];

    wire [DW-1:0]   ds_wdata   [0:6];
    wire [SW-1:0]   ds_wstrb   [0:6];
    wire            ds_wlast   [0:6];
    wire [0:0]      ds_wuser   [0:6];
    wire            ds_wvalid  [0:6];
    wire            ds_wready  [0:6];

    wire [IDW-1:0]  ds_bid     [0:6];
    wire [1:0]      ds_bresp   [0:6];
    wire [0:0]      ds_buser   [0:6];
    wire            ds_bvalid  [0:6];
    wire            ds_bready  [0:6];

    wire [IDW-1:0]  ds_arid    [0:6];
    wire [AW-1:0]   ds_araddr  [0:6];
    wire [7:0]      ds_arlen   [0:6];
    wire [2:0]      ds_arsize  [0:6];
    wire [1:0]      ds_arburst [0:6];
    wire            ds_arlock  [0:6];
    wire [3:0]      ds_arcache [0:6];
    wire [2:0]      ds_arprot  [0:6];
    wire [3:0]      ds_arqos   [0:6];
    wire [3:0]      ds_arregion[0:6];
    wire [0:0]      ds_aruser  [0:6];
    wire            ds_arvalid [0:6];
    wire            ds_arready [0:6];

    wire [IDW-1:0]  ds_rid     [0:6];
    wire [DW-1:0]   ds_rdata   [0:6];
    wire [1:0]      ds_rresp   [0:6];
    wire            ds_rlast   [0:6];
    wire [0:0]      ds_ruser   [0:6];
    wire            ds_rvalid  [0:6];
    wire            ds_rready  [0:6];

    assign ds_buser[0] = 1'b0; assign ds_buser[1] = 1'b0;
    assign ds_buser[2] = 1'b0; assign ds_buser[3] = 1'b0;
    assign ds_buser[4] = 1'b0; assign ds_buser[5] = 1'b0;
    assign ds_buser[6] = 1'b0;
    assign ds_ruser[0] = 1'b0; assign ds_ruser[1] = 1'b0;
    assign ds_ruser[2] = 1'b0; assign ds_ruser[3] = 1'b0;
    assign ds_ruser[4] = 1'b0; assign ds_ruser[5] = 1'b0;
    assign ds_ruser[6] = 1'b0;

    // Instantiate dummy slaves
    genvar gi;
    generate
        for (gi = 0; gi < 7; gi++) begin : gen_ds
            dummy_slave_relu_intg #(.DW(DW),.AW(AW),.IDW(IDW)) u_ds (
                .clk     (clk),
                .rst     (rst),
                .awid    (ds_awid[gi]),
                .awaddr  (ds_awaddr[gi]),
                .awlen   (ds_awlen[gi]),
                .awsize  (ds_awsize[gi]),
                .awburst (ds_awburst[gi]),
                .awlock  (ds_awlock[gi]),
                .awcache (ds_awcache[gi]),
                .awprot  (ds_awprot[gi]),
                .awqos   (ds_awqos[gi]),
                .awregion(ds_awregion[gi]),
                .awvalid (ds_awvalid[gi]),
                .awready (ds_awready[gi]),
                .wdata   (ds_wdata[gi]),
                .wstrb   (ds_wstrb[gi]),
                .wlast   (ds_wlast[gi]),
                .wvalid  (ds_wvalid[gi]),
                .wready  (ds_wready[gi]),
                .bid     (ds_bid[gi]),
                .bresp   (ds_bresp[gi]),
                .bvalid  (ds_bvalid[gi]),
                .bready  (ds_bready[gi]),
                .arid    (ds_arid[gi]),
                .araddr  (ds_araddr[gi]),
                .arlen   (ds_arlen[gi]),
                .arsize  (ds_arsize[gi]),
                .arburst (ds_arburst[gi]),
                .arlock  (ds_arlock[gi]),
                .arcache (ds_arcache[gi]),
                .arprot  (ds_arprot[gi]),
                .arqos   (ds_arqos[gi]),
                .arregion(ds_arregion[gi]),
                .arvalid (ds_arvalid[gi]),
                .arready (ds_arready[gi]),
                .rid     (ds_rid[gi]),
                .rdata   (ds_rdata[gi]),
                .rresp   (ds_rresp[gi]),
                .rlast   (ds_rlast[gi]),
                .rvalid  (ds_rvalid[gi]),
                .rready  (ds_rready[gi])
            );
        end
    endgenerate

    // ---------------------------------------------------------
    // AXI Interconnect 3x8
    // ---------------------------------------------------------
    axi_interconnect_wrap_3x8 #(
        .DATA_WIDTH     (DW),
        .ADDR_WIDTH     (AW),
        .STRB_WIDTH     (SW),
        .ID_WIDTH       (IDW),
        .AWUSER_ENABLE  (1),
        .AWUSER_WIDTH   (1),
        .WUSER_ENABLE   (1),
        .WUSER_WIDTH    (1),
        .BUSER_ENABLE   (1),
        .BUSER_WIDTH    (1),
        .ARUSER_ENABLE  (1),
        .ARUSER_WIDTH   (1),
        .RUSER_ENABLE   (1),
        .RUSER_WIDTH    (1),
        .FORWARD_ID     (1),
        .M_REGIONS      (1),

        .M00_BASE_ADDR   (32'h0000_0000), .M00_ADDR_WIDTH (32'd28),
        .M01_BASE_ADDR   (32'h1000_0000), .M01_ADDR_WIDTH (32'd28),
        .M02_BASE_ADDR   (32'h2000_0000), .M02_ADDR_WIDTH (32'd28),
        .M03_BASE_ADDR   (32'h3000_0000), .M03_ADDR_WIDTH (32'd28),  // ReLU
        .M04_BASE_ADDR   (32'h4000_0000), .M04_ADDR_WIDTH (32'd28),
        .M05_BASE_ADDR   (32'h5000_0000), .M05_ADDR_WIDTH (32'd28),
        .M06_BASE_ADDR   (32'h6000_0000), .M06_ADDR_WIDTH (32'd28),
        .M07_BASE_ADDR   (32'h7000_0000), .M07_ADDR_WIDTH (32'd28)
    ) u_ic (
        .clk(clk), .rst(rst),

        // Slave Port 0: Testbench Master
        .s00_axi_awid    (m_awid),    .s00_axi_awaddr  (m_awaddr),
        .s00_axi_awlen   (m_awlen),   .s00_axi_awsize  (m_awsize),
        .s00_axi_awburst (m_awburst), .s00_axi_awlock  (m_awlock),
        .s00_axi_awcache (m_awcache), .s00_axi_awprot  (m_awprot),
        .s00_axi_awqos   (m_awqos),   .s00_axi_awuser  (m_awuser),
        .s00_axi_awvalid (m_awvalid), .s00_axi_awready (m_awready),
        .s00_axi_wdata   (m_wdata),   .s00_axi_wstrb   (m_wstrb),
        .s00_axi_wlast   (m_wlast),   .s00_axi_wuser   (m_wuser),
        .s00_axi_wvalid  (m_wvalid),  .s00_axi_wready  (m_wready),
        .s00_axi_bid     (m_bid),     .s00_axi_bresp   (m_bresp),
        .s00_axi_buser   (m_buser),   .s00_axi_bvalid  (m_bvalid),
        .s00_axi_bready  (m_bready),
        .s00_axi_arid    (m_arid),    .s00_axi_araddr  (m_araddr),
        .s00_axi_arlen   (m_arlen),   .s00_axi_arsize  (m_arsize),
        .s00_axi_arburst (m_arburst), .s00_axi_arlock  (m_arlock),
        .s00_axi_arcache (m_arcache), .s00_axi_arprot  (m_arprot),
        .s00_axi_arqos   (m_arqos),   .s00_axi_aruser  (m_aruser),
        .s00_axi_arvalid (m_arvalid), .s00_axi_arready (m_arready),
        .s00_axi_rid     (m_rid),     .s00_axi_rdata   (m_rdata),
        .s00_axi_rresp   (m_rresp),   .s00_axi_rlast   (m_rlast),
        .s00_axi_ruser   (m_ruser),   .s00_axi_rvalid  (m_rvalid),
        .s00_axi_rready  (m_rready),

        // Slave Port 1: Idle
        .s01_axi_awid    (s1_awid),   .s01_axi_awaddr  (s1_awaddr),
        .s01_axi_awlen   (s1_awlen),  .s01_axi_awsize  (s1_awsize),
        .s01_axi_awburst (s1_awburst),.s01_axi_awlock  (s1_awlock),
        .s01_axi_awcache (s1_awcache),.s01_axi_awprot  (s1_awprot),
        .s01_axi_awqos   (s1_awqos),  .s01_axi_awuser  (s1_awuser),
        .s01_axi_awvalid (s1_awvalid),.s01_axi_awready (),
        .s01_axi_wdata   (s1_wdata),  .s01_axi_wstrb   (s1_wstrb),
        .s01_axi_wlast   (s1_wlast),  .s01_axi_wuser   (s1_wuser),
        .s01_axi_wvalid  (s1_wvalid), .s01_axi_wready  (),
        .s01_axi_bid     (),          .s01_axi_bresp   (),
        .s01_axi_buser   (),          .s01_axi_bvalid  (),
        .s01_axi_bready  (s1_bready),
        .s01_axi_arid    (s1_arid),   .s01_axi_araddr  (s1_araddr),
        .s01_axi_arlen   (s1_arlen),  .s01_axi_arsize  (s1_arsize),
        .s01_axi_arburst (s1_arburst),.s01_axi_arlock  (s1_arlock),
        .s01_axi_arcache (s1_arcache),.s01_axi_arprot  (s1_arprot),
        .s01_axi_arqos   (s1_arqos),  .s01_axi_aruser  (s1_aruser),
        .s01_axi_arvalid (s1_arvalid),.s01_axi_arready (),
        .s01_axi_rid     (),          .s01_axi_rdata   (),
        .s01_axi_rresp   (),          .s01_axi_rlast   (),
        .s01_axi_ruser   (),          .s01_axi_rvalid  (),
        .s01_axi_rready  (s1_rready),

        // Slave Port 2: Idle
        .s02_axi_awid    (s2_awid),   .s02_axi_awaddr  (s2_awaddr),
        .s02_axi_awlen   (s2_awlen),  .s02_axi_awsize  (s2_awsize),
        .s02_axi_awburst (s2_awburst),.s02_axi_awlock  (s2_awlock),
        .s02_axi_awcache (s2_awcache),.s02_axi_awprot  (s2_awprot),
        .s02_axi_awqos   (s2_awqos),  .s02_axi_awuser  (s2_awuser),
        .s02_axi_awvalid (s2_awvalid),.s02_axi_awready (),
        .s02_axi_wdata   (s2_wdata),  .s02_axi_wstrb   (s2_wstrb),
        .s02_axi_wlast   (s2_wlast),  .s02_axi_wuser   (s2_wuser),
        .s02_axi_wvalid  (s2_wvalid), .s02_axi_wready  (),
        .s02_axi_bid     (),          .s02_axi_bresp   (),
        .s02_axi_buser   (),          .s02_axi_bvalid  (),
        .s02_axi_bready  (s2_bready),
        .s02_axi_arid    (s2_arid),   .s02_axi_araddr  (s2_araddr),
        .s02_axi_arlen   (s2_arlen),  .s02_axi_arsize  (s2_arsize),
        .s02_axi_arburst (s2_arburst),.s02_axi_arlock  (s2_arlock),
        .s02_axi_arcache (s2_arcache),.s02_axi_arprot  (s2_arprot),
        .s02_axi_arqos   (s2_arqos),  .s02_axi_aruser  (s2_aruser),
        .s02_axi_arvalid (s2_arvalid),.s02_axi_arready (),
        .s02_axi_rid     (),          .s02_axi_rdata   (),
        .s02_axi_rresp   (),          .s02_axi_rlast   (),
        .s02_axi_ruser   (),          .s02_axi_rvalid  (),
        .s02_axi_rready  (s2_rready),

        // Master Port 0: Dummy Slave 0
        .m00_axi_awid    (ds_awid[0]),  .m00_axi_awaddr  (ds_awaddr[0]),
        .m00_axi_awlen   (ds_awlen[0]), .m00_axi_awsize  (ds_awsize[0]),
        .m00_axi_awburst (ds_awburst[0]),.m00_axi_awlock  (ds_awlock[0]),
        .m00_axi_awcache (ds_awcache[0]),.m00_axi_awprot  (ds_awprot[0]),
        .m00_axi_awqos   (ds_awqos[0]), .m00_axi_awregion(ds_awregion[0]),
        .m00_axi_awuser  (ds_awuser[0]),.m00_axi_awvalid (ds_awvalid[0]),
        .m00_axi_awready (ds_awready[0]),
        .m00_axi_wdata   (ds_wdata[0]), .m00_axi_wstrb   (ds_wstrb[0]),
        .m00_axi_wlast   (ds_wlast[0]), .m00_axi_wuser   (ds_wuser[0]),
        .m00_axi_wvalid  (ds_wvalid[0]),.m00_axi_wready  (ds_wready[0]),
        .m00_axi_bid     (ds_bid[0]),   .m00_axi_bresp   (ds_bresp[0]),
        .m00_axi_buser   (ds_buser[0]), .m00_axi_bvalid  (ds_bvalid[0]),
        .m00_axi_bready  (ds_bready[0]),
        .m00_axi_arid    (ds_arid[0]),  .m00_axi_araddr  (ds_araddr[0]),
        .m00_axi_arlen   (ds_arlen[0]), .m00_axi_arsize  (ds_arsize[0]),
        .m00_axi_arburst (ds_arburst[0]),.m00_axi_arlock  (ds_arlock[0]),
        .m00_axi_arcache (ds_arcache[0]),.m00_axi_arprot  (ds_arprot[0]),
        .m00_axi_arqos   (ds_arqos[0]), .m00_axi_arregion(ds_arregion[0]),
        .m00_axi_aruser  (ds_aruser[0]),.m00_axi_arvalid (ds_arvalid[0]),
        .m00_axi_arready (ds_arready[0]),
        .m00_axi_rid     (ds_rid[0]),   .m00_axi_rdata   (ds_rdata[0]),
        .m00_axi_rresp   (ds_rresp[0]), .m00_axi_rlast   (ds_rlast[0]),
        .m00_axi_ruser   (ds_ruser[0]), .m00_axi_rvalid  (ds_rvalid[0]),
        .m00_axi_rready  (ds_rready[0]),

        // Master Port 1: Dummy Slave 1
        .m01_axi_awid    (ds_awid[1]),  .m01_axi_awaddr  (ds_awaddr[1]),
        .m01_axi_awlen   (ds_awlen[1]), .m01_axi_awsize  (ds_awsize[1]),
        .m01_axi_awburst (ds_awburst[1]),.m01_axi_awlock  (ds_awlock[1]),
        .m01_axi_awcache (ds_awcache[1]),.m01_axi_awprot  (ds_awprot[1]),
        .m01_axi_awqos   (ds_awqos[1]), .m01_axi_awregion(ds_awregion[1]),
        .m01_axi_awuser  (ds_awuser[1]),.m01_axi_awvalid (ds_awvalid[1]),
        .m01_axi_awready (ds_awready[1]),
        .m01_axi_wdata   (ds_wdata[1]), .m01_axi_wstrb   (ds_wstrb[1]),
        .m01_axi_wlast   (ds_wlast[1]), .m01_axi_wuser   (ds_wuser[1]),
        .m01_axi_wvalid  (ds_wvalid[1]),.m01_axi_wready  (ds_wready[1]),
        .m01_axi_bid     (ds_bid[1]),   .m01_axi_bresp   (ds_bresp[1]),
        .m01_axi_buser   (ds_buser[1]), .m01_axi_bvalid  (ds_bvalid[1]),
        .m01_axi_bready  (ds_bready[1]),
        .m01_axi_arid    (ds_arid[1]),  .m01_axi_araddr  (ds_araddr[1]),
        .m01_axi_arlen   (ds_arlen[1]), .m01_axi_arsize  (ds_arsize[1]),
        .m01_axi_arburst (ds_arburst[1]),.m01_axi_arlock  (ds_arlock[1]),
        .m01_axi_arcache (ds_arcache[1]),.m01_axi_arprot  (ds_arprot[1]),
        .m01_axi_arqos   (ds_arqos[1]), .m01_axi_arregion(ds_arregion[1]),
        .m01_axi_aruser  (ds_aruser[1]),.m01_axi_arvalid (ds_arvalid[1]),
        .m01_axi_arready (ds_arready[1]),
        .m01_axi_rid     (ds_rid[1]),   .m01_axi_rdata   (ds_rdata[1]),
        .m01_axi_rresp   (ds_rresp[1]), .m01_axi_rlast   (ds_rlast[1]),
        .m01_axi_ruser   (ds_ruser[1]), .m01_axi_rvalid  (ds_rvalid[1]),
        .m01_axi_rready  (ds_rready[1]),

        // Master Port 2: Dummy Slave 2
        .m02_axi_awid    (ds_awid[2]),  .m02_axi_awaddr  (ds_awaddr[2]),
        .m02_axi_awlen   (ds_awlen[2]), .m02_axi_awsize  (ds_awsize[2]),
        .m02_axi_awburst (ds_awburst[2]),.m02_axi_awlock  (ds_awlock[2]),
        .m02_axi_awcache (ds_awcache[2]),.m02_axi_awprot  (ds_awprot[2]),
        .m02_axi_awqos   (ds_awqos[2]), .m02_axi_awregion(ds_awregion[2]),
        .m02_axi_awuser  (ds_awuser[2]),.m02_axi_awvalid (ds_awvalid[2]),
        .m02_axi_awready (ds_awready[2]),
        .m02_axi_wdata   (ds_wdata[2]), .m02_axi_wstrb   (ds_wstrb[2]),
        .m02_axi_wlast   (ds_wlast[2]), .m02_axi_wuser   (ds_wuser[2]),
        .m02_axi_wvalid  (ds_wvalid[2]),.m02_axi_wready  (ds_wready[2]),
        .m02_axi_bid     (ds_bid[2]),   .m02_axi_bresp   (ds_bresp[2]),
        .m02_axi_buser   (ds_buser[2]), .m02_axi_bvalid  (ds_bvalid[2]),
        .m02_axi_bready  (ds_bready[2]),
        .m02_axi_arid    (ds_arid[2]),  .m02_axi_araddr  (ds_araddr[2]),
        .m02_axi_arlen   (ds_arlen[2]), .m02_axi_arsize  (ds_arsize[2]),
        .m02_axi_arburst (ds_arburst[2]),.m02_axi_arlock  (ds_arlock[2]),
        .m02_axi_arcache (ds_arcache[2]),.m02_axi_arprot  (ds_arprot[2]),
        .m02_axi_arqos   (ds_arqos[2]), .m02_axi_arregion(ds_arregion[2]),
        .m02_axi_aruser  (ds_aruser[2]),.m02_axi_arvalid (ds_arvalid[2]),
        .m02_axi_arready (ds_arready[2]),
        .m02_axi_rid     (ds_rid[2]),   .m02_axi_rdata   (ds_rdata[2]),
        .m02_axi_rresp   (ds_rresp[2]), .m02_axi_rlast   (ds_rlast[2]),
        .m02_axi_ruser   (ds_ruser[2]), .m02_axi_rvalid  (ds_rvalid[2]),
        .m02_axi_rready  (ds_rready[2]),

        // Master Port 3: axi_relu (DUT)
        .m03_axi_awid    (ic_relu_awid),    .m03_axi_awaddr  (ic_relu_awaddr),
        .m03_axi_awlen   (ic_relu_awlen),   .m03_axi_awsize  (ic_relu_awsize),
        .m03_axi_awburst (ic_relu_awburst), .m03_axi_awlock  (ic_relu_awlock),
        .m03_axi_awcache (ic_relu_awcache), .m03_axi_awprot  (ic_relu_awprot),
        .m03_axi_awqos   (ic_relu_awqos),   .m03_axi_awregion(ic_relu_awregion),
        .m03_axi_awuser  (ic_relu_awuser),  .m03_axi_awvalid (ic_relu_awvalid),
        .m03_axi_awready (ic_relu_awready),
        .m03_axi_wdata   (ic_relu_wdata),   .m03_axi_wstrb   (ic_relu_wstrb),
        .m03_axi_wlast   (ic_relu_wlast),   .m03_axi_wuser   (ic_relu_wuser),
        .m03_axi_wvalid  (ic_relu_wvalid),  .m03_axi_wready  (ic_relu_wready),
        .m03_axi_bid     (ic_relu_bid),     .m03_axi_bresp   (ic_relu_bresp),
        .m03_axi_buser   (ic_relu_buser),   .m03_axi_bvalid  (ic_relu_bvalid),
        .m03_axi_bready  (ic_relu_bready),
        .m03_axi_arid    (ic_relu_arid),    .m03_axi_araddr  (ic_relu_araddr),
        .m03_axi_arlen   (ic_relu_arlen),   .m03_axi_arsize  (ic_relu_arsize),
        .m03_axi_arburst (ic_relu_arburst), .m03_axi_arlock  (ic_relu_arlock),
        .m03_axi_arcache (ic_relu_arcache), .m03_axi_arprot  (ic_relu_arprot),
        .m03_axi_arqos   (ic_relu_arqos),   .m03_axi_arregion(ic_relu_arregion),
        .m03_axi_aruser  (ic_relu_aruser),  .m03_axi_arvalid (ic_relu_arvalid),
        .m03_axi_arready (ic_relu_arready),
        .m03_axi_rid     (ic_relu_rid),     .m03_axi_rdata   (ic_relu_rdata),
        .m03_axi_rresp   (ic_relu_rresp),   .m03_axi_rlast   (ic_relu_rlast),
        .m03_axi_ruser   (ic_relu_ruser),   .m03_axi_rvalid  (ic_relu_rvalid),
        .m03_axi_rready  (ic_relu_rready),

        // Master Port 4: Dummy Slave 3
        .m04_axi_awid    (ds_awid[3]),  .m04_axi_awaddr  (ds_awaddr[3]),
        .m04_axi_awlen   (ds_awlen[3]), .m04_axi_awsize  (ds_awsize[3]),
        .m04_axi_awburst (ds_awburst[3]),.m04_axi_awlock  (ds_awlock[3]),
        .m04_axi_awcache (ds_awcache[3]),.m04_axi_awprot  (ds_awprot[3]),
        .m04_axi_awqos   (ds_awqos[3]), .m04_axi_awregion(ds_awregion[3]),
        .m04_axi_awuser  (ds_awuser[3]),.m04_axi_awvalid (ds_awvalid[3]),
        .m04_axi_awready (ds_awready[3]),
        .m04_axi_wdata   (ds_wdata[3]), .m04_axi_wstrb   (ds_wstrb[3]),
        .m04_axi_wlast   (ds_wlast[3]), .m04_axi_wuser   (ds_wuser[3]),
        .m04_axi_wvalid  (ds_wvalid[3]),.m04_axi_wready  (ds_wready[3]),
        .m04_axi_bid     (ds_bid[3]),   .m04_axi_bresp   (ds_bresp[3]),
        .m04_axi_buser   (ds_buser[3]), .m04_axi_bvalid  (ds_bvalid[3]),
        .m04_axi_bready  (ds_bready[3]),
        .m04_axi_arid    (ds_arid[3]),  .m04_axi_araddr  (ds_araddr[3]),
        .m04_axi_arlen   (ds_arlen[3]), .m04_axi_arsize  (ds_arsize[3]),
        .m04_axi_arburst (ds_arburst[3]),.m04_axi_arlock  (ds_arlock[3]),
        .m04_axi_arcache (ds_arcache[3]),.m04_axi_arprot  (ds_arprot[3]),
        .m04_axi_arqos   (ds_arqos[3]), .m04_axi_arregion(ds_arregion[3]),
        .m04_axi_aruser  (ds_aruser[3]),.m04_axi_arvalid (ds_arvalid[3]),
        .m04_axi_arready (ds_arready[3]),
        .m04_axi_rid     (ds_rid[3]),   .m04_axi_rdata   (ds_rdata[3]),
        .m04_axi_rresp   (ds_rresp[3]), .m04_axi_rlast   (ds_rlast[3]),
        .m04_axi_ruser   (ds_ruser[3]), .m04_axi_rvalid  (ds_rvalid[3]),
        .m04_axi_rready  (ds_rready[3]),

        // Master Port 5: Dummy Slave 4
        .m05_axi_awid    (ds_awid[4]),  .m05_axi_awaddr  (ds_awaddr[4]),
        .m05_axi_awlen   (ds_awlen[4]), .m05_axi_awsize  (ds_awsize[4]),
        .m05_axi_awburst (ds_awburst[4]),.m05_axi_awlock  (ds_awlock[4]),
        .m05_axi_awcache (ds_awcache[4]),.m05_axi_awprot  (ds_awprot[4]),
        .m05_axi_awqos   (ds_awqos[4]), .m05_axi_awregion(ds_awregion[4]),
        .m05_axi_awuser  (ds_awuser[4]),.m05_axi_awvalid (ds_awvalid[4]),
        .m05_axi_awready (ds_awready[4]),
        .m05_axi_wdata   (ds_wdata[4]), .m05_axi_wstrb   (ds_wstrb[4]),
        .m05_axi_wlast   (ds_wlast[4]), .m05_axi_wuser   (ds_wuser[4]),
        .m05_axi_wvalid  (ds_wvalid[4]),.m05_axi_wready  (ds_wready[4]),
        .m05_axi_bid     (ds_bid[4]),   .m05_axi_bresp   (ds_bresp[4]),
        .m05_axi_buser   (ds_buser[4]), .m05_axi_bvalid  (ds_bvalid[4]),
        .m05_axi_bready  (ds_bready[4]),
        .m05_axi_arid    (ds_arid[4]),  .m05_axi_araddr  (ds_araddr[4]),
        .m05_axi_arlen   (ds_arlen[4]), .m05_axi_arsize  (ds_arsize[4]),
        .m05_axi_arburst (ds_arburst[4]),.m05_axi_arlock  (ds_arlock[4]),
        .m05_axi_arcache (ds_arcache[4]),.m05_axi_arprot  (ds_arprot[4]),
        .m05_axi_arqos   (ds_arqos[4]), .m05_axi_arregion(ds_arregion[4]),
        .m05_axi_aruser  (ds_aruser[4]),.m05_axi_arvalid (ds_arvalid[4]),
        .m05_axi_arready (ds_arready[4]),
        .m05_axi_rid     (ds_rid[4]),   .m05_axi_rdata   (ds_rdata[4]),
        .m05_axi_rresp   (ds_rresp[4]), .m05_axi_rlast   (ds_rlast[4]),
        .m05_axi_ruser   (ds_ruser[4]), .m05_axi_rvalid  (ds_rvalid[4]),
        .m05_axi_rready  (ds_rready[4]),

        // Master Port 6: Dummy Slave 5
        .m06_axi_awid    (ds_awid[5]),  .m06_axi_awaddr  (ds_awaddr[5]),
        .m06_axi_awlen   (ds_awlen[5]), .m06_axi_awsize  (ds_awsize[5]),
        .m06_axi_awburst (ds_awburst[5]),.m06_axi_awlock  (ds_awlock[5]),
        .m06_axi_awcache (ds_awcache[5]),.m06_axi_awprot  (ds_awprot[5]),
        .m06_axi_awqos   (ds_awqos[5]), .m06_axi_awregion(ds_awregion[5]),
        .m06_axi_awuser  (ds_awuser[5]),.m06_axi_awvalid (ds_awvalid[5]),
        .m06_axi_awready (ds_awready[5]),
        .m06_axi_wdata   (ds_wdata[5]), .m06_axi_wstrb   (ds_wstrb[5]),
        .m06_axi_wlast   (ds_wlast[5]), .m06_axi_wuser   (ds_wuser[5]),
        .m06_axi_wvalid  (ds_wvalid[5]),.m06_axi_wready  (ds_wready[5]),
        .m06_axi_bid     (ds_bid[5]),   .m06_axi_bresp   (ds_bresp[5]),
        .m06_axi_buser   (ds_buser[5]), .m06_axi_bvalid  (ds_bvalid[5]),
        .m06_axi_bready  (ds_bready[5]),
        .m06_axi_arid    (ds_arid[5]),  .m06_axi_araddr  (ds_araddr[5]),
        .m06_axi_arlen   (ds_arlen[5]), .m06_axi_arsize  (ds_arsize[5]),
        .m06_axi_arburst (ds_arburst[5]),.m06_axi_arlock  (ds_arlock[5]),
        .m06_axi_arcache (ds_arcache[5]),.m06_axi_arprot  (ds_arprot[5]),
        .m06_axi_arqos   (ds_arqos[5]), .m06_axi_arregion(ds_arregion[5]),
        .m06_axi_aruser  (ds_aruser[5]),.m06_axi_arvalid (ds_arvalid[5]),
        .m06_axi_arready (ds_arready[5]),
        .m06_axi_rid     (ds_rid[5]),   .m06_axi_rdata   (ds_rdata[5]),
        .m06_axi_rresp   (ds_rresp[5]), .m06_axi_rlast   (ds_rlast[5]),
        .m06_axi_ruser   (ds_ruser[5]), .m06_axi_rvalid  (ds_rvalid[5]),
        .m06_axi_rready  (ds_rready[5]),

        // Master Port 7: Dummy Slave 6
        .m07_axi_awid    (ds_awid[6]),  .m07_axi_awaddr  (ds_awaddr[6]),
        .m07_axi_awlen   (ds_awlen[6]), .m07_axi_awsize  (ds_awsize[6]),
        .m07_axi_awburst (ds_awburst[6]),.m07_axi_awlock  (ds_awlock[6]),
        .m07_axi_awcache (ds_awcache[6]),.m07_axi_awprot  (ds_awprot[6]),
        .m07_axi_awqos   (ds_awqos[6]), .m07_axi_awregion(ds_awregion[6]),
        .m07_axi_awuser  (ds_awuser[6]),.m07_axi_awvalid (ds_awvalid[6]),
        .m07_axi_awready (ds_awready[6]),
        .m07_axi_wdata   (ds_wdata[6]), .m07_axi_wstrb   (ds_wstrb[6]),
        .m07_axi_wlast   (ds_wlast[6]), .m07_axi_wuser   (ds_wuser[6]),
        .m07_axi_wvalid  (ds_wvalid[6]),.m07_axi_wready  (ds_wready[6]),
        .m07_axi_bid     (ds_bid[6]),   .m07_axi_bresp   (ds_bresp[6]),
        .m07_axi_buser   (ds_buser[6]), .m07_axi_bvalid  (ds_bvalid[6]),
        .m07_axi_bready  (ds_bready[6]),
        .m07_axi_arid    (ds_arid[6]),  .m07_axi_araddr  (ds_araddr[6]),
        .m07_axi_arlen   (ds_arlen[6]), .m07_axi_arsize  (ds_arsize[6]),
        .m07_axi_arburst (ds_arburst[6]),.m07_axi_arlock  (ds_arlock[6]),
        .m07_axi_arcache (ds_arcache[6]),.m07_axi_arprot  (ds_arprot[6]),
        .m07_axi_arqos   (ds_arqos[6]), .m07_axi_arregion(ds_arregion[6]),
        .m07_axi_aruser  (ds_aruser[6]),.m07_axi_arvalid (ds_arvalid[6]),
        .m07_axi_arready (ds_arready[6]),
        .m07_axi_rid     (ds_rid[6]),   .m07_axi_rdata   (ds_rdata[6]),
        .m07_axi_rresp   (ds_rresp[6]), .m07_axi_rlast   (ds_rlast[6]),
        .m07_axi_ruser   (ds_ruser[6]), .m07_axi_rvalid  (ds_rvalid[6]),
        .m07_axi_rready  (ds_rready[6])
    );

    // ---------------------------------------------------------
    // Instantiate axi_relu on interconnect port m03
    // ---------------------------------------------------------
    axi_relu #(
        .DATA_WIDTH     (32),
        .AXI_DATA_WIDTH (DW),
        .AXI_ADDR_WIDTH (AW),
        .AXI_ID_WIDTH   (IDW),
        .REG_ADDR_WIDTH (12),
        .BASE_ADDR      (RELU_BASE)
    ) u_relu (
        .clk             (clk),
        .rst             (rst),

        .s_axi_awid      (ic_relu_awid),
        .s_axi_awaddr    (ic_relu_awaddr),
        .s_axi_awlen     (ic_relu_awlen),
        .s_axi_awsize    (ic_relu_awsize),
        .s_axi_awburst   (ic_relu_awburst),
        .s_axi_awlock    (ic_relu_awlock),
        .s_axi_awcache   (ic_relu_awcache),
        .s_axi_awprot    (ic_relu_awprot),
        .s_axi_awqos     (ic_relu_awqos),
        .s_axi_awregion  (ic_relu_awregion),
        .s_axi_awvalid   (ic_relu_awvalid),
        .s_axi_awready   (ic_relu_awready),

        .s_axi_wdata     (ic_relu_wdata),
        .s_axi_wstrb     (ic_relu_wstrb),
        .s_axi_wlast     (ic_relu_wlast),
        .s_axi_wvalid    (ic_relu_wvalid),
        .s_axi_wready    (ic_relu_wready),

        .s_axi_bid       (ic_relu_bid),
        .s_axi_bresp     (ic_relu_bresp),
        .s_axi_bvalid    (ic_relu_bvalid),
        .s_axi_bready    (ic_relu_bready),

        .s_axi_arid      (ic_relu_arid),
        .s_axi_araddr    (ic_relu_araddr),
        .s_axi_arlen     (ic_relu_arlen),
        .s_axi_arsize    (ic_relu_arsize),
        .s_axi_arburst   (ic_relu_arburst),
        .s_axi_arlock    (ic_relu_arlock),
        .s_axi_arcache   (ic_relu_arcache),
        .s_axi_arprot    (ic_relu_arprot),
        .s_axi_arqos     (ic_relu_arqos),
        .s_axi_arregion  (ic_relu_arregion),
        .s_axi_arvalid   (ic_relu_arvalid),
        .s_axi_arready   (ic_relu_arready),

        .s_axi_rid       (ic_relu_rid),
        .s_axi_rdata     (ic_relu_rdata),
        .s_axi_rresp     (ic_relu_rresp),
        .s_axi_rlast     (ic_relu_rlast),
        .s_axi_rvalid    (ic_relu_rvalid),
        .s_axi_rready    (ic_relu_rready)
    );

    // ---------------------------------------------------------
    // Test tracking & reporting
    // ---------------------------------------------------------
    integer pass_cnt;
    integer fail_cnt;

    task pass_msg;
        input [511:0] msg;
        begin
            $display("  [PASS] %0s", msg);
            pass_cnt = pass_cnt + 1;
        end
    endtask

    task fail_msg;
        input [511:0] msg;
        begin
            $display("  [FAIL] %0s", msg);
            fail_cnt = fail_cnt + 1;
        end
    endtask

    // ---------------------------------------------------------
    // AXI Bus Driver Tasks
    // ---------------------------------------------------------
    task axi_write_64;
        input [AW-1:0] addr;
        input [63:0]   data;
        input [7:0]    strb;
        begin
            @(posedge clk); #1;

            m_awid     <= 8'h01;
            m_awaddr   <= addr;
            m_awlen    <= 8'h00;
            m_awsize   <= 3'b011;   // 8 bytes
            m_awburst  <= 2'b01;
            m_awlock   <= 1'b0;
            m_awcache  <= 4'b0000;
            m_awprot   <= 3'b000;
            m_awqos    <= 4'b0000;
            m_awuser   <= 1'b0;
            m_awvalid  <= 1'b1;

            m_wdata    <= data;
            m_wstrb    <= strb;
            m_wlast    <= 1'b1;
            m_wuser    <= 1'b0;
            m_wvalid   <= 1'b1;
            m_bready   <= 1'b1;

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
            last_write_resp = m_bresp;
            @(posedge clk); #1;
            m_bready <= 1'b0;

            repeat (2) @(posedge clk);
        end
    endtask

    task axi_write_32;
        input [AW-1:0]  addr;
        input [31:0]    data;
        input [7:0]     strb;
        begin
            axi_write_64(addr, {32'h0, data}, strb);
        end
    endtask

    task axi_write_32_resp;
        input [AW-1:0]  addr;
        input [31:0]    data;
        input [7:0]     strb;
        output [1:0]    resp;
        begin
            axi_write_64(addr, {32'h0, data}, strb);
            resp = last_write_resp;
        end
    endtask

    task axi_read_64;
        input  [AW-1:0] addr;
        output [63:0]   data;
        output [1:0]    resp;
        begin
            @(posedge clk); #1;

            m_arid    <= 8'h01;
            m_araddr  <= addr;
            m_arlen   <= 8'h00;
            m_arsize  <= 3'b011;
            m_arburst <= 2'b01;
            m_arlock  <= 1'b0;
            m_arcache <= 4'b0000;
            m_arprot  <= 3'b000;
            m_arqos   <= 4'b0000;
            m_aruser  <= 1'b0;
            m_arvalid <= 1'b1;
            m_rready  <= 1'b1;

            wait (m_arready); @(posedge clk); #1;
            m_arvalid <= 1'b0;

            wait (m_rvalid);
            data = m_rdata;
            resp = m_rresp;
            @(posedge clk); #1;
            m_rready <= 1'b0;

            repeat (2) @(posedge clk);
        end
    endtask

    task axi_read_32;
        input  [AW-1:0] addr;
        output [31:0]   data;
        output [1:0]    resp;
        reg [63:0] d64;
        reg [1:0]  r;
        begin
            axi_read_64(addr, d64, r);
            data = d64[31:0];
            resp = r;
        end
    endtask

    // =========================================================================
    // Test Suites
    // =========================================================================

    // T1: Reset & Peripheral ID
    task t1_reset_and_id;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T1: Reset & Peripheral Identification via Interconnect ==");

            rst = 1'b1;
            repeat (10) @(posedge clk);
            rst = 1'b0;
            repeat (20) @(posedge clk);

            // Read CTRL: default enable=1 (0x0000_0001)
            axi_read_32(REG_ADDR_CTRL, rd, resp);
            if (resp === RESP_OKAY && rd === 32'h0000_0001)
                pass_msg("RELU_CTRL reset default = 0x0000_0001 (enabled)");
            else
                fail_msg("RELU_CTRL reset default incorrect");

            // Read STATUS: default 0
            axi_read_32(REG_ADDR_STATUS, rd, resp);
            if (resp === RESP_OKAY && rd === 32'h0)
                pass_msg("RELU_STATUS reset default = 0x0000_0000");
            else
                fail_msg("RELU_STATUS reset default incorrect");

            // Read ID: "RELU" (0x5245_4C55)
            axi_read_32(REG_ADDR_ID, rd, resp);
            if (resp === RESP_OKAY && rd === RELU_ID_VALUE)
                pass_msg("RELU_ID = 'RELU' (0x52454C55)");
            else
                fail_msg("RELU_ID mismatch");

            // Read VERSION: 0x0001_0000
            axi_read_32(REG_ADDR_VER, rd, resp);
            if (resp === RESP_OKAY && rd === RELU_VERSION_VALUE)
                pass_msg("RELU_VERSION = 0x00010000 (v1.0)");
            else
                fail_msg("RELU_VERSION mismatch");
        end
    endtask

    // T2: Control & Configuration Register R/W
    task t2_register_rw;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T2: Control & Configuration Register R/W ==");

            // Enable bypass and clipping: CTRL = 0x07 (enable=1, bypass=1, clip_en=1)
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0007, 8'h0F);
            axi_read_32(REG_ADDR_CTRL, rd, resp);
            if (resp === RESP_OKAY && rd[2:0] === 3'b111)
                pass_msg("RELU_CTRL readback = 0x07 (en, bypass, clip_en)");
            else
                fail_msg("RELU_CTRL readback mismatch");

            // Configure CLIP_MAX = 500
            axi_write_32(REG_ADDR_CLIP_MAX, 32'd500, 8'h0F);
            axi_read_32(REG_ADDR_CLIP_MAX, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd500)
                pass_msg("RELU_CLIP_MAX readback = 500");
            else
                fail_msg("RELU_CLIP_MAX readback mismatch");

            // Restore CTRL to standard ReLU (enable=1, bypass=0, clip_en=0)
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0001, 8'h0F);
            axi_read_32(REG_ADDR_CTRL, rd, resp);
            if (resp === RESP_OKAY && rd === 32'h0000_0001)
                pass_msg("RELU_CTRL restored to standard mode (0x01)");
            else
                fail_msg("RELU_CTRL restore mismatch");
        end
    endtask

    // T3: Standard Positive Activations
    task t3_positive_activation;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T3: Standard Positive Activations ==");

            // Positive input: 42 -> DOUT = 42, last_active=1, last_zero=0
            axi_write_32(REG_ADDR_DIN, 32'd42, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd42)
                pass_msg("DIN=42 -> DOUT=42");
            else
                fail_msg("DIN=42 DOUT mismatch");

            axi_read_32(REG_ADDR_STATUS, rd, resp);
            if (resp === RESP_OKAY && rd[2] === 1'b1 && rd[1] === 1'b0)
                pass_msg("STATUS reflects last_active=1, last_zero=0 for positive input");
            else
                fail_msg("STATUS flags incorrect for positive input");

            // Another positive input: 12345
            axi_write_32(REG_ADDR_DIN, 32'd12345, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd12345)
                pass_msg("DIN=12345 -> DOUT=12345");
            else
                fail_msg("DIN=12345 DOUT mismatch");
        end
    endtask

    // T4: Negative Input Clamping & Zero Handling
    task t4_negative_clamping;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T4: Negative Input Clamping & Zero Handling ==");

            // Negative input: -100 -> DOUT = 0, last_zero=1, last_active=0
            axi_write_32(REG_ADDR_DIN, -32'd100, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd0)
                pass_msg("DIN=-100 -> DOUT=0 (clamped to zero)");
            else
                fail_msg("DIN=-100 DOUT not clamped to 0");

            axi_read_32(REG_ADDR_STATUS, rd, resp);
            if (resp === RESP_OKAY && rd[1] === 1'b1 && rd[2] === 1'b0)
                pass_msg("STATUS reflects last_zero=1, last_active=0 for negative input");
            else
                fail_msg("STATUS flags incorrect for negative input");

            // Zero input: 0 -> DOUT = 0, last_zero=1
            axi_write_32(REG_ADDR_DIN, 32'd0, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd0)
                pass_msg("DIN=0 -> DOUT=0");
            else
                fail_msg("DIN=0 DOUT mismatch");

            axi_read_32(REG_ADDR_STATUS, rd, resp);
            if (resp === RESP_OKAY && rd[1] === 1'b1)
                pass_msg("STATUS reflects last_zero=1 for zero input");
            else
                fail_msg("STATUS flags incorrect for zero input");
        end
    endtask

    // T5: Extreme Boundary Values
    task t5_boundary_values;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T5: Extreme Boundary Values ==");

            // Maximum positive 32-bit signed integer: 0x7FFF_FFFF (+2147483647)
            axi_write_32(REG_ADDR_DIN, 32'h7FFF_FFFF, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'h7FFF_FFFF)
                pass_msg("DIN=0x7FFFFFFF -> DOUT=0x7FFFFFFF (maximum positive signed)");
            else
                fail_msg("DIN=0x7FFFFFFF DOUT mismatch");

            // Minimum negative 32-bit signed integer: 0x8000_0000 (-2147483648)
            axi_write_32(REG_ADDR_DIN, 32'h8000_0000, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd0)
                pass_msg("DIN=0x80000000 -> DOUT=0 (minimum negative signed clamped)");
            else
                fail_msg("DIN=0x80000000 DOUT mismatch");

            // -1 (0xFFFF_FFFF) -> clamped to 0
            axi_write_32(REG_ADDR_DIN, 32'hFFFF_FFFF, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd0)
                pass_msg("DIN=-1 (0xFFFFFFFF) -> DOUT=0");
            else
                fail_msg("DIN=-1 DOUT mismatch");
        end
    endtask

    // T6: Bypass Mode Linear Passthrough
    task t6_bypass_mode;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T6: Bypass Mode Linear Passthrough ==");

            // Set bypass: CTRL = 0x03 (enable=1, bypass=1)
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0003, 8'h0F);

            // Negative input in bypass mode should NOT be clamped: -75 -> DOUT = -75
            axi_write_32(REG_ADDR_DIN, -32'd75, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === (-32'd75))
                pass_msg("Bypass mode: DIN=-75 -> DOUT=-75 (unclamped passthrough)");
            else
                fail_msg("Bypass mode failed to pass negative value unmodified");

            // Positive input in bypass mode: +88 -> DOUT = +88
            axi_write_32(REG_ADDR_DIN, 32'd88, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd88)
                pass_msg("Bypass mode: DIN=88 -> DOUT=88");
            else
                fail_msg("Bypass mode failed to pass positive value");

            // Restore standard mode
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0001, 8'h0F);
        end
    endtask

    // T7: Saturated Clipping Activation (Clipped ReLU / ReLU6)
    task t7_clipping_saturation;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T7: Saturated Clipping Activation (Clipped ReLU) ==");

            // Configure clip threshold to 60 (e.g. scaled ReLU6)
            axi_write_32(REG_ADDR_CLIP_MAX, 32'd60, 8'h0F);

            // Enable clipping: CTRL = 0x05 (enable=1, clip_en=1)
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0005, 8'h0F);

            // Input above clip threshold: 150 -> DOUT should saturate at 60
            axi_write_32(REG_ADDR_DIN, 32'd150, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd60)
                pass_msg("DIN=150 with CLIP_MAX=60 -> DOUT=60 (saturated)");
            else
                fail_msg("Clipping failed to saturate at threshold");

            axi_read_32(REG_ADDR_STATUS, rd, resp);
            if (resp === RESP_OKAY && rd[3] === 1'b1)
                pass_msg("STATUS reflects last_clipped=1");
            else
                fail_msg("STATUS flag last_clipped not asserted");

            // Input below clip threshold: 45 -> DOUT = 45 (unclipped active)
            axi_write_32(REG_ADDR_DIN, 32'd45, 8'h0F);
            axi_read_32(REG_ADDR_DOUT, rd, resp);
            if (resp === RESP_OKAY && rd === 32'd45)
                pass_msg("DIN=45 with CLIP_MAX=60 -> DOUT=45 (linear active)");
            else
                fail_msg("Clipping erroneously modified below-threshold value");

            axi_read_32(REG_ADDR_STATUS, rd, resp);
            if (resp === RESP_OKAY && rd[3] === 1'b0 && rd[2] === 1'b1)
                pass_msg("STATUS reflects last_clipped=0, last_active=1");
            else
                fail_msg("STATUS flag incorrect for unclipped value");

            // Restore standard mode
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0001, 8'h0F);
        end
    endtask

    // T8: Sparsity & Metrics Counters with reset_stats
    task t8_sparsity_counters;
        reg [31:0] rd_zero;
        reg [31:0] rd_tot;
        reg [1:0]  resp;
        begin
            $display("\n== T8: Sparsity & Metrics Counters with reset_stats ==");

            // Reset stats by writing bit 3 of CTRL: 0x09 (enable=1, reset_stats=1)
            axi_write_32(REG_ADDR_CTRL, 32'h0000_0009, 8'h0F);

            // Verify counters cleared to 0
            axi_read_32(REG_ADDR_ZERO_CNT, rd_zero, resp);
            axi_read_32(REG_ADDR_TOT_CNT,  rd_tot,  resp);
            if (resp === RESP_OKAY && rd_zero === 32'd0 && rd_tot === 32'd0)
                pass_msg("reset_stats pulse cleared ZERO_COUNT and TOTAL_COUNT to 0");
            else
                fail_msg("reset_stats failed to clear counters");

            // Process 4 inputs: 2 positive, 2 negative/zero
            axi_write_32(REG_ADDR_DIN, 32'd10,  8'h0F);   // active
            axi_write_32(REG_ADDR_DIN, -32'd20, 8'h0F);   // clamped
            axi_write_32(REG_ADDR_DIN, 32'd30,  8'h0F);   // active
            axi_write_32(REG_ADDR_DIN, 32'd0,   8'h0F);   // clamped (zero)

            axi_read_32(REG_ADDR_ZERO_CNT, rd_zero, resp);
            axi_read_32(REG_ADDR_TOT_CNT,  rd_tot,  resp);

            if (rd_tot === 32'd4)
                pass_msg("TOTAL_COUNT = 4 after 4 inputs");
            else
                fail_msg("TOTAL_COUNT mismatch");

            if (rd_zero === 32'd2)
                pass_msg("ZERO_COUNT = 2 (50% activation sparsity)");
            else
                fail_msg("ZERO_COUNT mismatch");
        end
    endtask

    // T9: Read-Only Register Write Protection (SLVERR)
    task t9_ro_protection;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T9: Read-Only Register Write Protection (SLVERR) ==");

            // Attempt write to RELU_STATUS (RO)
            axi_write_32_resp(REG_ADDR_STATUS, 32'hFFFF_FFFF, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Write to RELU_STATUS returned SLVERR");
            else
                fail_msg("Write to RELU_STATUS did not return SLVERR");

            // Attempt write to RELU_DOUT (RO)
            axi_write_32_resp(REG_ADDR_DOUT, 32'h1234_5678, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Write to RELU_DOUT returned SLVERR");
            else
                fail_msg("Write to RELU_DOUT did not return SLVERR");

            // Attempt write to RELU_ZERO_COUNT (RO)
            axi_write_32_resp(REG_ADDR_ZERO_CNT, 32'hFFFF_FFFF, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Write to RELU_ZERO_COUNT returned SLVERR");
            else
                fail_msg("Write to RELU_ZERO_COUNT did not return SLVERR");

            // Attempt write to RELU_TOTAL_COUNT (RO)
            axi_write_32_resp(REG_ADDR_TOT_CNT, 32'hFFFF_FFFF, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Write to RELU_TOTAL_COUNT returned SLVERR");
            else
                fail_msg("Write to RELU_TOTAL_COUNT did not return SLVERR");

            // Attempt write to RELU_ID (RO)
            axi_write_32_resp(REG_ADDR_ID, 32'hFFFF_FFFF, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Write to RELU_ID returned SLVERR");
            else
                fail_msg("Write to RELU_ID did not return SLVERR");

            // Verify RELU_ID was not modified
            axi_read_32(REG_ADDR_ID, rd, resp);
            if (rd === RELU_ID_VALUE)
                pass_msg("RELU_ID unchanged after write attempt");
            else
                fail_msg("RELU_ID was modified");

            // Attempt write to RELU_VERSION (RO)
            axi_write_32_resp(REG_ADDR_VER, 32'hFFFF_FFFF, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Write to RELU_VERSION returned SLVERR");
            else
                fail_msg("Write to RELU_VERSION did not return SLVERR");
        end
    endtask

    // T10: Address Decoding & Error Handling
    task t10_address_errors;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T10: Address Decoding & Error Handling ==");

            // Reserved offset within aperture: 0x30
            axi_write_32_resp(RELU_BASE + 32'h030, 32'hCAFE_BABE, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Reserved offset 0x30 write returns SLVERR");
            else
                fail_msg("Reserved offset 0x30 write did not return SLVERR");

            axi_read_32(RELU_BASE + 32'h030, rd, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Reserved offset 0x30 read returns SLVERR");
            else
                fail_msg("Reserved offset 0x30 read did not return SLVERR");

            // Out-of-aperture within port m03 slice (base + 0x2000, 4 KiB aperture)
            axi_write_32_resp(RELU_BASE + 32'h2000, 32'hDEAD_BEEF, 8'h0F, resp);
            if (resp === RESP_DECERR)
                pass_msg("Out-of-aperture write returns DECERR");
            else
                fail_msg("Out-of-aperture write did not return DECERR");

            axi_read_32(RELU_BASE + 32'h2000, rd, resp);
            if (resp === RESP_DECERR)
                pass_msg("Out-of-aperture read returns DECERR");
            else
                fail_msg("Out-of-aperture read did not return DECERR");

            // Misaligned address: 0x02
            axi_write_32_resp(RELU_BASE + 32'h002, 32'h1234_5678, 8'h0F, resp);
            if (resp === RESP_SLVERR)
                pass_msg("Misaligned address write returns SLVERR");
            else
                fail_msg("Misaligned address write did not return SLVERR");
        end
    endtask

    // T11: WSTRB Partial Writes
    task t11_wstrb;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T11: WSTRB Partial Writes ==");

            // Full write to CLIP_MAX: 0xDEAD_BEEF
            axi_write_32(REG_ADDR_CLIP_MAX, 32'hDEAD_BEEF, 8'h0F);
            axi_read_32(REG_ADDR_CLIP_MAX, rd, resp);
            if (rd === 32'hDEAD_BEEF)
                pass_msg("CLIP_MAX full write = 0xDEAD_BEEF");
            else
                fail_msg("CLIP_MAX full write mismatch");

            // Write only byte 0 with 0x11 (strb=0x01) -> 0xDEAD_BE11
            axi_write_32(REG_ADDR_CLIP_MAX, 32'h0000_0011, 8'h01);
            axi_read_32(REG_ADDR_CLIP_MAX, rd, resp);
            if (rd === 32'hDEAD_BE11)
                pass_msg("CLIP_MAX byte 0 write = 0xDEAD_BE11");
            else
                fail_msg("CLIP_MAX byte 0 write mismatch");

            // Write only byte 1 with 0x22 (strb=0x02) -> 0xDEAD_2211
            axi_write_32(REG_ADDR_CLIP_MAX, 32'h0000_2200, 8'h02);
            axi_read_32(REG_ADDR_CLIP_MAX, rd, resp);
            if (rd === 32'hDEAD_2211)
                pass_msg("CLIP_MAX byte 1 write = 0xDEAD_2211");
            else
                fail_msg("CLIP_MAX byte 1 write mismatch");
        end
    endtask

    // T12: Interconnect Multi-Slave Routing Independence
    task t12_interconnect_routing;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T12: Interconnect Multi-Slave Routing Independence ==");

            // Access dummy slave on port m02 (0x2000_0000)
            axi_read_32(32'h2000_0000, rd, resp);
            if (resp === RESP_OKAY)
                pass_msg("Interconnect properly routes to slave m02 (0x2000_0000)");
            else
                fail_msg("Interconnect routing to m02 failed");

            // Access axi_relu on port m03 (0x3000_0000)
            axi_read_32(REG_ADDR_CTRL, rd, resp);
            if (resp === RESP_OKAY && rd[0] === 1'b1)
                pass_msg("Interconnect routes to axi_relu on m03 (0x3000_0000)");
            else
                fail_msg("Interconnect routing to axi_relu failed");

            // Access dummy slave on port m04 (0x4000_0000)
            axi_read_32(32'h4000_0000, rd, resp);
            if (resp === RESP_OKAY)
                pass_msg("Interconnect properly routes to slave m04 (0x4000_0000)");
            else
                fail_msg("Interconnect routing to m04 failed");
        end
    endtask

    // ---------------------------------------------------------
    // Main initial test execution
    // ---------------------------------------------------------
    initial begin
        rst = 1'b1;
        pass_cnt = 0;
        fail_cnt = 0;

        m_awid    = '0; m_awaddr  = '0; m_awlen   = '0;
        m_awsize  = 3'b011; m_awburst = 2'b01;
        m_awlock  = '0; m_awcache = '0; m_awprot  = '0;
        m_awqos   = '0; m_awuser  = '0; m_awvalid = '0;
        m_wdata   = '0; m_wstrb   = '0; m_wlast   = '0;
        m_wuser   = '0; m_wvalid  = '0; m_bready  = '0;
        m_arid    = '0; m_araddr  = '0; m_arlen   = '0;
        m_arsize  = 3'b011; m_arburst = 2'b01;
        m_arlock  = '0; m_arcache = '0; m_arprot  = '0;
        m_arqos   = '0; m_aruser  = '0; m_arvalid = '0;
        m_rready  = '0;

        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (10) @(posedge clk);

        t1_reset_and_id();
        t2_register_rw();
        t3_positive_activation();
        t4_negative_clamping();
        t5_boundary_values();
        t6_bypass_mode();
        t7_clipping_saturation();
        t8_sparsity_counters();
        t9_ro_protection();
        t10_address_errors();
        t11_wstrb();
        t12_interconnect_routing();

        $display("\n");
        $display("============================================================");
        $display("        ReLU ↔ AXI INTERCONNECT INTEGRATION RESULTS");
        $display("============================================================");
        $display("  PASS : %0d", pass_cnt);
        $display("  FAIL : %0d", fail_cnt);
        $display("============================================================");
        if (fail_cnt == 0) begin
            $display("  *** ALL TESTS PASSED ***");
        end else begin
            $display("  *** %0d TEST(S) FAILED ***", fail_cnt);
        end
        $display("============================================================\n");

        #1000;
        $finish;
    end

    // Watchdog
    initial begin
        #10_000_000;
        $display("\n[WATCHDOG] Simulation timed out!");
        $finish;
    end

endmodule

`default_nettype wire
