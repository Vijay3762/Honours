// =============================================================================
// File    : tb_cnn_on_interconnect.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
//
// Purpose : Integration testbench — verifies the axi_cnn_conv hanging off port
//           m03 of the 3x8 AXI interconnect.
//
// Topology
// --------
//   TB AXI master (s00)
//      |
//   axi_interconnect_wrap_3x8
//      |-- m00..m02, m04..m07 → dummy_slave_intg instances
//      |-- m03                 → axi_cnn_conv
//
// Test plan
// ---------
//   T1  Reset defaults        : CTRL=0, STATUS=0, ID=0x434E4E56, VERSION=0x00010000
//   T2  Register write/readback: CTRL, IN_BASE, OUT_BASE, IMG_W, IMG_H, K00, K11, BIAS
//   T3  Convolution computation: identity kernel, all-ones input → result=1
//   T4  Interrupt generation   : irq_en=1, done → IRQ asserted
//   T5  RO register writes     : STATUS, ID, VERSION → SLVERR
//   T6  Reserved offset        : 0x70 → SLVERR
//   T7  Out-of-aperture        : 0x3001_0000 → DECERR
//   T8  Misaligned address     : 0x3000_0005 → SLVERR
//   T9  WSTRB partial write    : byte strobe control
//
// Compile (xvlog):
//   xvlog -sv -i rtl/uart/include \
//       rtl/cnn/axi_cnn_conv.sv \
//       rtl/interconnect/axi_interconnect.v \
//       rtl/interconnect/arbiter.v \
//       rtl/interconnect/priority_encoder.v \
//       rtl/interconnect/axi_interconnect_wrap_3x8.v \
//       rtl/axi_mem.sv \
//       rtl/axi_dummy_slave.sv \
//       rtl/simple_cpu_master.sv \
//       rtl/soc_top.sv \
//       tb/tb_cnn_on_interconnect.sv
//
// Simulate:
//   xsim tb_cnn_on_interconnect -runall
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

// ---------------------------------------------------------------------------
// Minimal dummy slave – accepts every single-beat write/read, returns a
// deterministic data pattern.  Used for m00..m02, m04..m07.
// ---------------------------------------------------------------------------
module dummy_slave_intg #(
    parameter integer DW  = 64,
    parameter integer AW  = 32,
    parameter integer IDW = 8
)(
    input  wire              clk,
    input  wire              rst,

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
    input  wire [DW-1:0]     wdata,
    input  wire [DW/8-1:0]   wstrb,
    input  wire              wlast,
    input  wire              wvalid,
    output reg               wready,
    output reg  [IDW-1:0]    bid,
    output reg  [1:0]        bresp,
    output reg               bvalid,
    input  wire              bready,
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
            wready  <= !bvalid;

            if (awvalid && awready) begin
                aw_pend <= 1'b1;
                aw_id_r <= awid;
            end

            if (wvalid && wready && aw_pend) begin
                aw_pend <= 1'b0;
                bid     <= aw_id_r;
                bresp   <= 2'b00;
                bvalid  <= 1'b1;
            end

            if (bvalid && bready)
                bvalid <= 1'b0;

            arready <= !rvalid;
            if (arvalid && arready) begin
                rid    <= arid;
                rdata  <= {64{1'b0}} | {32'hCAFE_0000, araddr[31:0]};
                rresp  <= 2'b00;
                rlast  <= 1'b1;
                rvalid <= 1'b1;
            end

            if (rvalid && rready) begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
            end
        end
    end
endmodule


// ---------------------------------------------------------------------------
// Top-level testbench
// ---------------------------------------------------------------------------
module tb_cnn_on_interconnect;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam integer DW   = 64;
    localparam integer AW   = 32;
    localparam integer IDW  = 8;
    localparam integer SW   = DW/8;   // 8

    localparam integer CLK_PERIOD = 10;   // 10 ns  →  100 MHz

    // CNN base address on interconnect  (m03 → 0x3000_0000)
    localparam [AW-1:0] CNN_BASE = 32'h3000_0000;

    // CNN register offsets (from CNN_BASE)
    localparam [AW-1:0] CNN_CTRL     = CNN_BASE + 32'h000;
    localparam [AW-1:0] CNN_STATUS   = CNN_BASE + 32'h004;
    localparam [AW-1:0] CNN_IN_BASE  = CNN_BASE + 32'h008;
    localparam [AW-1:0] CNN_OUT_BASE = CNN_BASE + 32'h00C;
    localparam [AW-1:0] CNN_IMG_W    = CNN_BASE + 32'h010;
    localparam [AW-1:0] CNN_IMG_H    = CNN_BASE + 32'h014;
    localparam [AW-1:0] CNN_K00      = CNN_BASE + 32'h018;
    localparam [AW-1:0] CNN_K01      = CNN_BASE + 32'h01C;
    localparam [AW-1:0] CNN_K02      = CNN_BASE + 32'h020;
    localparam [AW-1:0] CNN_K10      = CNN_BASE + 32'h024;
    localparam [AW-1:0] CNN_K11      = CNN_BASE + 32'h028;
    localparam [AW-1:0] CNN_K12      = CNN_BASE + 32'h02C;
    localparam [AW-1:0] CNN_K20      = CNN_BASE + 32'h030;
    localparam [AW-1:0] CNN_K21      = CNN_BASE + 32'h034;
    localparam [AW-1:0] CNN_K22      = CNN_BASE + 32'h038;
    localparam [AW-1:0] CNN_BIAS     = CNN_BASE + 32'h03C;
    localparam [AW-1:0] CNN_PX00     = CNN_BASE + 32'h048;
    localparam [AW-1:0] CNN_PX01     = CNN_BASE + 32'h04C;
    localparam [AW-1:0] CNN_PX02     = CNN_BASE + 32'h050;
    localparam [AW-1:0] CNN_PX10     = CNN_BASE + 32'h054;
    localparam [AW-1:0] CNN_PX11     = CNN_BASE + 32'h058;
    localparam [AW-1:0] CNN_PX12     = CNN_BASE + 32'h05C;
    localparam [AW-1:0] CNN_PX20     = CNN_BASE + 32'h060;
    localparam [AW-1:0] CNN_PX21     = CNN_BASE + 32'h064;
    localparam [AW-1:0] CNN_PX22     = CNN_BASE + 32'h068;
    localparam [AW-1:0] CNN_RESULT   = CNN_BASE + 32'h06C;
    localparam [AW-1:0] CNN_ID_REG   = CNN_BASE + 32'h040;
    localparam [AW-1:0] CNN_VERSION  = CNN_BASE + 32'h044;

    // Fixed constants from axi_cnn_conv
    localparam [31:0] CNN_ID_VALUE      = 32'h434E4E56;  // "CNNV"
    localparam [31:0] CNN_VERSION_VALUE = 32'h00010000;  // v1.0

    // =========================================================================
    // Clock / reset
    // =========================================================================
    reg clk;
    reg rst;   // active-high

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // TB master AXI signals  (drives interconnect s00)
    // =========================================================================
    reg  [IDW-1:0]  m_awid;
    reg  [AW-1:0]   m_awaddr;
    reg  [7:0]      m_awlen;
    reg  [2:0]      m_awsize;
    reg  [1:0]      m_awburst;
    reg             m_awlock;
    reg  [3:0]      m_awcache;
    reg  [2:0]      m_awprot;
    reg  [3:0]      m_awqos;
    reg             m_awvalid;
    wire            m_awready;

    reg  [DW-1:0]   m_wdata;
    reg  [SW-1:0]   m_wstrb;
    reg             m_wlast;
    reg             m_wvalid;
    wire            m_wready;

    wire [IDW-1:0]  m_bid;
    wire [1:0]      m_bresp;
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
    reg             m_arvalid;
    wire            m_arready;

    wire [IDW-1:0]  m_rid;
    wire [DW-1:0]   m_rdata;
    wire [1:0]      m_rresp;
    wire            m_rlast;
    wire            m_rvalid;
    reg             m_rready;

    // =========================================================================
    // Idle-driven master ports s01 and s02 (not used)
    // =========================================================================
    wire [IDW-1:0]  s1_awid  = '0; wire [AW-1:0]  s1_awaddr  = '0;
    wire [7:0]      s1_awlen = '0; wire [2:0]      s1_awsize  = 3'b010;
    wire [1:0]      s1_awburst= 2'b01; wire         s1_awlock  = 1'b0;
    wire [3:0]      s1_awcache= '0; wire [2:0]     s1_awprot  = '0;
    wire [3:0]      s1_awqos = '0;
    wire            s1_awvalid= 1'b0;
    wire [DW-1:0]   s1_wdata = '0; wire [SW-1:0]   s1_wstrb   = '0;
    wire            s1_wlast = 1'b0; wire            s1_wvalid= 1'b0;
    wire            s1_bready  = 1'b1;
    wire [IDW-1:0]  s1_arid  = '0; wire [AW-1:0]   s1_araddr  = '0;
    wire [7:0]      s1_arlen = '0; wire [2:0]      s1_arsize  = 3'b010;
    wire [1:0]      s1_arburst= 2'b01; wire         s1_arlock  = 1'b0;
    wire [3:0]      s1_arcache= '0; wire [2:0]     s1_arprot  = '0;
    wire [3:0]      s1_arqos = '0;
    wire            s1_arvalid= 1'b0; wire           s1_rready  = 1'b1;

    wire [IDW-1:0]  s2_awid  = '0; wire [AW-1:0]  s2_awaddr  = '0;
    wire [7:0]      s2_awlen = '0; wire [2:0]      s2_awsize  = 3'b010;
    wire [1:0]      s2_awburst= 2'b01; wire         s2_awlock  = 1'b0;
    wire [3:0]      s2_awcache= '0; wire [2:0]     s2_awprot  = '0;
    wire [3:0]      s2_awqos = '0;
    wire            s2_awvalid= 1'b0;
    wire [DW-1:0]   s2_wdata = '0; wire [SW-1:0]   s2_wstrb   = '0;
    wire            s2_wlast = 1'b0; wire            s2_wvalid= 1'b0;
    wire            s2_bready  = 1'b1;
    wire [IDW-1:0]  s2_arid  = '0; wire [AW-1:0]   s2_araddr  = '0;
    wire [7:0]      s2_arlen = '0; wire [2:0]      s2_arsize  = 3'b010;
    wire [1:0]      s2_arburst= 2'b01; wire         s2_arlock  = 1'b0;
    wire [3:0]      s2_arcache= '0; wire [2:0]     s2_arprot  = '0;
    wire [3:0]      s2_arqos = '0;
    wire            s2_arvalid= 1'b0; wire           s2_rready  = 1'b1;

    wire [IDW-1:0]  s1_bid, s2_bid;
    wire [1:0]      s1_bresp, s2_bresp;
    wire            s1_bvalid, s2_bvalid;
    wire [IDW-1:0]  s1_rid, s2_rid;
    wire [DW-1:0]   s1_rdata, s2_rdata;
    wire [1:0]      s1_rresp, s2_rresp;
    wire            s1_rlast, s2_rlast;
    wire            s1_rvalid, s2_rvalid;
    wire            s1_awready, s2_awready;
    wire            s1_wready, s2_wready;
    wire            s1_arready, s2_arready;

    // =========================================================================
    // Inter-module wires: interconnect m03 <-> axi_cnn_conv
    // =========================================================================
    wire [IDW-1:0]  ic_cnn_awid;
    wire [AW-1:0]   ic_cnn_awaddr;
    wire [7:0]      ic_cnn_awlen;
    wire [2:0]      ic_cnn_awsize;
    wire [1:0]      ic_cnn_awburst;
    wire            ic_cnn_awlock;
    wire [3:0]      ic_cnn_awcache;
    wire [2:0]      ic_cnn_awprot;
    wire [3:0]      ic_cnn_awqos;
    wire [3:0]      ic_cnn_awregion;
    wire            ic_cnn_awvalid;
    wire            ic_cnn_awready;

    wire [DW-1:0]   ic_cnn_wdata;
    wire [SW-1:0]   ic_cnn_wstrb;
    wire            ic_cnn_wlast;
    wire            ic_cnn_wvalid;
    wire            ic_cnn_wready;

    wire [IDW-1:0]  ic_cnn_bid;
    wire [1:0]      ic_cnn_bresp;
    wire            ic_cnn_bvalid;
    wire            ic_cnn_bready;

    wire [IDW-1:0]  ic_cnn_arid;
    wire [AW-1:0]   ic_cnn_araddr;
    wire [7:0]      ic_cnn_arlen;
    wire [2:0]      ic_cnn_arsize;
    wire [1:0]      ic_cnn_arburst;
    wire            ic_cnn_arlock;
    wire [3:0]      ic_cnn_arcache;
    wire [2:0]      ic_cnn_arprot;
    wire [3:0]      ic_cnn_arqos;
    wire [3:0]      ic_cnn_arregion;
    wire            ic_cnn_arvalid;
    wire            ic_cnn_arready;

    wire [IDW-1:0]  ic_cnn_rid;
    wire [DW-1:0]   ic_cnn_rdata;
    wire [1:0]      ic_cnn_rresp;
    wire            ic_cnn_rlast;
    wire            ic_cnn_rvalid;
    wire            ic_cnn_rready;

    // =========================================================================
    // CNN interrupt signal
    // =========================================================================
    wire            cnn_irq_o;

    // =========================================================================
    // Dummy slave interconnect wires  (m00..m02, m04..m07) = 7 dummies
    // Slave index mapping:
    //   0 = m00 (IMEM)   1 = m01 (DMEM)   2 = m02 (DMA)
    //   3 = m04 (UART)   4 = m05 (TIMER)   5 = m06 (GPIO)
    //   6 = m07 (7-SEG)
    // (m03 = CNN, handled separately above)
    // =========================================================================
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
    wire            ds_awvalid [0:6];
    wire            ds_awready [0:6];

    wire [DW-1:0]   ds_wdata   [0:6];
    wire [SW-1:0]   ds_wstrb   [0:6];
    wire            ds_wlast   [0:6];
    wire            ds_wvalid  [0:6];
    wire            ds_wready  [0:6];

    wire [IDW-1:0]  ds_bid     [0:6];
    wire [1:0]      ds_bresp   [0:6];
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
    wire            ds_arvalid [0:6];
    wire            ds_arready [0:6];

    wire [IDW-1:0]  ds_rid     [0:6];
    wire [DW-1:0]   ds_rdata   [0:6];
    wire [1:0]      ds_rresp   [0:6];
    wire            ds_rlast   [0:6];
    wire            ds_rvalid  [0:6];
    wire            ds_rready  [0:6];

    // =========================================================================
    // Generate 7 dummy_slave_intg instances  (m00..m02, m04..m07)
    // =========================================================================
    genvar gi;
    generate
        for (gi = 0; gi < 7; gi++) begin : gen_ds
            dummy_slave_intg #(.DW(DW),.AW(AW),.IDW(IDW)) u_ds (
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

    // =========================================================================
    // AXI Interconnect DUT  (3 masters × 8 slaves)
    // =========================================================================
    axi_interconnect_wrap_3x8 #(
        .DATA_WIDTH          (DW),
        .ADDR_WIDTH          (AW),
        .STRB_WIDTH          (SW),
        .ID_WIDTH            (IDW),
        // CNN slave  m03 = 0x3000_0000 .. 0x3FFF_FFFF
        .M03_BASE_ADDR       (32'h3000_0000),
        .M03_ADDR_WIDTH      ({1{32'd28}}),
        .M03_CONNECT_READ    (3'b001),
        .M03_CONNECT_WRITE   (3'b001),
        // IMEM slave  m00 = 0x0000_0000
        .M00_BASE_ADDR       (32'h0000_0000),
        .M00_ADDR_WIDTH      ({1{32'd28}}),
        .M00_CONNECT_READ    (3'b111),
        .M00_CONNECT_WRITE   (3'b111),
        // DMEM slave  m01 = 0x1000_0000
        .M01_BASE_ADDR       (32'h1000_0000),
        .M01_ADDR_WIDTH      ({1{32'd28}}),
        .M01_CONNECT_READ    (3'b111),
        .M01_CONNECT_WRITE   (3'b111),
        // DMA slave   m02 = 0x2000_0000
        .M02_BASE_ADDR       (32'h2000_0000),
        .M02_ADDR_WIDTH      ({1{32'd28}}),
        .M02_CONNECT_READ    (3'b111),
        .M02_CONNECT_WRITE   (3'b111),
        // UART slave  m04 = 0x4000_0000
        .M04_BASE_ADDR       (32'h4000_0000),
        .M04_ADDR_WIDTH      ({1{32'd28}}),
        .M04_CONNECT_READ    (3'b111),
        .M04_CONNECT_WRITE   (3'b111),
        // TIMER slave m05 = 0x5000_0000
        .M05_BASE_ADDR       (32'h5000_0000),
        .M05_ADDR_WIDTH      ({1{32'd28}}),
        .M05_CONNECT_READ    (3'b111),
        .M05_CONNECT_WRITE   (3'b111),
        // GPIO slave  m06 = 0x6000_0000
        .M06_BASE_ADDR       (32'h6000_0000),
        .M06_ADDR_WIDTH      ({1{32'd28}}),
        .M06_CONNECT_READ    (3'b111),
        .M06_CONNECT_WRITE   (3'b111),
        // 7-SEG slave m07 = 0x7000_0000
        .M07_BASE_ADDR       (32'h7000_0000),
        .M07_ADDR_WIDTH      ({1{32'd28}}),
        .M07_CONNECT_READ    (3'b111),
        .M07_CONNECT_WRITE   (3'b111)
    ) u_ic (
        .clk (clk),
        .rst (rst),

        // ---- Master port 0  (TB driver) ----
        .s00_axi_awid    (m_awid),
        .s00_axi_awaddr  (m_awaddr),
        .s00_axi_awlen   (m_awlen),
        .s00_axi_awsize  (m_awsize),
        .s00_axi_awburst (m_awburst),
        .s00_axi_awlock  (m_awlock),
        .s00_axi_awcache (m_awcache),
        .s00_axi_awprot  (m_awprot),
        .s00_axi_awqos   (m_awqos),
        .s00_axi_awvalid (m_awvalid),
        .s00_axi_awready (m_awready),
        .s00_axi_wdata   (m_wdata),
        .s00_axi_wstrb   (m_wstrb),
        .s00_axi_wlast   (m_wlast),
        .s00_axi_wvalid  (m_wvalid),
        .s00_axi_wready  (m_wready),
        .s00_axi_bid     (m_bid),
        .s00_axi_bresp   (m_bresp),
        .s00_axi_bvalid  (m_bvalid),
        .s00_axi_bready  (m_bready),
        .s00_axi_arid    (m_arid),
        .s00_axi_araddr  (m_araddr),
        .s00_axi_arlen   (m_arlen),
        .s00_axi_arsize  (m_arsize),
        .s00_axi_arburst (m_arburst),
        .s00_axi_arlock  (m_arlock),
        .s00_axi_arcache (m_arcache),
        .s00_axi_arprot  (m_arprot),
        .s00_axi_arqos   (m_arqos),
        .s00_axi_arvalid (m_arvalid),
        .s00_axi_arready (m_arready),
        .s00_axi_rid     (m_rid),
        .s00_axi_rdata   (m_rdata),
        .s00_axi_rresp   (m_rresp),
        .s00_axi_rlast   (m_rlast),
        .s00_axi_rvalid  (m_rvalid),
        .s00_axi_rready  (m_rready),

        // ---- Master port 1  (idle) ----
        .s01_axi_awid    (s1_awid),    .s01_axi_awaddr  (s1_awaddr),
        .s01_axi_awlen   (s1_awlen),   .s01_axi_awsize  (s1_awsize),
        .s01_axi_awburst (s1_awburst), .s01_axi_awlock  (s1_awlock),
        .s01_axi_awcache (s1_awcache), .s01_axi_awprot  (s1_awprot),
        .s01_axi_awqos   (s1_awqos),
        .s01_axi_awvalid (s1_awvalid), .s01_axi_awready (s1_awready),
        .s01_axi_wdata   (s1_wdata),   .s01_axi_wstrb   (s1_wstrb),
        .s01_axi_wlast   (s1_wlast),   .s01_axi_wvalid  (s1_wvalid),
        .s01_axi_wready  (s1_wready),
        .s01_axi_bid     (s1_bid),     .s01_axi_bresp   (s1_bresp),
        .s01_axi_bvalid  (s1_bvalid),  .s01_axi_bready  (s1_bready),
        .s01_axi_arid    (s1_arid),    .s01_axi_araddr  (s1_araddr),
        .s01_axi_arlen   (s1_arlen),   .s01_axi_arsize  (s1_arsize),
        .s01_axi_arburst (s1_arburst), .s01_axi_arlock  (s1_arlock),
        .s01_axi_arcache (s1_arcache), .s01_axi_arprot  (s1_arprot),
        .s01_axi_arqos   (s1_arqos),
        .s01_axi_arvalid (s1_arvalid), .s01_axi_arready (s1_arready),
        .s01_axi_rid     (s1_rid),     .s01_axi_rdata   (s1_rdata),
        .s01_axi_rresp   (s1_rresp),   .s01_axi_rlast   (s1_rlast),
        .s01_axi_rvalid  (s1_rvalid),  .s01_axi_rready  (s1_rready),

        // ---- Master port 2  (idle) ----
        .s02_axi_awid    (s2_awid),    .s02_axi_awaddr  (s2_awaddr),
        .s02_axi_awlen   (s2_awlen),   .s02_axi_awsize  (s2_awsize),
        .s02_axi_awburst (s2_awburst), .s02_axi_awlock  (s2_awlock),
        .s02_axi_awcache (s2_awcache), .s02_axi_awprot  (s2_awprot),
        .s02_axi_awqos   (s2_awqos),
        .s02_axi_awvalid (s2_awvalid), .s02_axi_awready (s2_awready),
        .s02_axi_wdata   (s2_wdata),   .s02_axi_wstrb   (s2_wstrb),
        .s02_axi_wlast   (s2_wlast),   .s02_axi_wvalid  (s2_wvalid),
        .s02_axi_wready  (s2_wready),
        .s02_axi_bid     (s2_bid),     .s02_axi_bresp   (s2_bresp),
        .s02_axi_bvalid  (s2_bvalid),  .s02_axi_bready  (s2_bready),
        .s02_axi_arid    (s2_arid),    .s02_axi_araddr  (s2_araddr),
        .s02_axi_arlen   (s2_arlen),   .s02_axi_arsize  (s2_arsize),
        .s02_axi_arburst (s2_arburst), .s02_axi_arlock  (s2_arlock),
        .s02_axi_arcache (s2_arcache), .s02_axi_arprot  (s2_arprot),
        .s02_axi_arqos   (s2_arqos),
        .s02_axi_arvalid (s2_arvalid), .s02_axi_arready (s2_arready),
        .s02_axi_rid     (s2_rid),     .s02_axi_rdata   (s2_rdata),
        .s02_axi_rresp   (s2_rresp),   .s02_axi_rlast   (s2_rlast),
        .s02_axi_rvalid  (s2_rvalid),  .s02_axi_rready  (s2_rready),

        // ---- Slave port 0 : m00 IMEM ----
        .m00_axi_awid    (ds_awid[0]),  .m00_axi_awaddr  (ds_awaddr[0]),
        .m00_axi_awlen   (ds_awlen[0]), .m00_axi_awsize  (ds_awsize[0]),
        .m00_axi_awburst (ds_awburst[0]),.m00_axi_awlock  (ds_awlock[0]),
        .m00_axi_awcache (ds_awcache[0]),.m00_axi_awprot  (ds_awprot[0]),
        .m00_axi_awqos   (ds_awqos[0]), .m00_axi_awregion(ds_awregion[0]),
        .m00_axi_awvalid (ds_awvalid[0]),.m00_axi_awready (ds_awready[0]),
        .m00_axi_wdata   (ds_wdata[0]), .m00_axi_wstrb   (ds_wstrb[0]),
        .m00_axi_wlast   (ds_wlast[0]), .m00_axi_wvalid  (ds_wvalid[0]),
        .m00_axi_wready  (ds_wready[0]),
        .m00_axi_bid     (ds_bid[0]),   .m00_axi_bresp   (ds_bresp[0]),
        .m00_axi_bvalid  (ds_bvalid[0]), .m00_axi_bready  (ds_bready[0]),
        .m00_axi_arid    (ds_arid[0]),  .m00_axi_araddr  (ds_araddr[0]),
        .m00_axi_arlen   (ds_arlen[0]), .m00_axi_arsize  (ds_arsize[0]),
        .m00_axi_arburst (ds_arburst[0]),.m00_axi_arlock  (ds_arlock[0]),
        .m00_axi_arcache (ds_arcache[0]),.m00_axi_arprot  (ds_arprot[0]),
        .m00_axi_arqos   (ds_arqos[0]), .m00_axi_arregion(ds_arregion[0]),
        .m00_axi_arvalid (ds_arvalid[0]),.m00_axi_arready (ds_arready[0]),
        .m00_axi_rid     (ds_rid[0]),   .m00_axi_rdata   (ds_rdata[0]),
        .m00_axi_rresp   (ds_rresp[0]), .m00_axi_rlast   (ds_rlast[0]),
        .m00_axi_rvalid  (ds_rvalid[0]), .m00_axi_rready  (ds_rready[0]),

        // ---- Slave port 1 : m01 DMEM ----
        .m01_axi_awid    (ds_awid[1]),  .m01_axi_awaddr  (ds_awaddr[1]),
        .m01_axi_awlen   (ds_awlen[1]), .m01_axi_awsize  (ds_awsize[1]),
        .m01_axi_awburst (ds_awburst[1]),.m01_axi_awlock  (ds_awlock[1]),
        .m01_axi_awcache (ds_awcache[1]),.m01_axi_awprot  (ds_awprot[1]),
        .m01_axi_awqos   (ds_awqos[1]), .m01_axi_awregion(ds_awregion[1]),
        .m01_axi_awvalid (ds_awvalid[1]),.m01_axi_awready (ds_awready[1]),
        .m01_axi_wdata   (ds_wdata[1]), .m01_axi_wstrb   (ds_wstrb[1]),
        .m01_axi_wlast   (ds_wlast[1]), .m01_axi_wvalid  (ds_wvalid[1]),
        .m01_axi_wready  (ds_wready[1]),
        .m01_axi_bid     (ds_bid[1]),   .m01_axi_bresp   (ds_bresp[1]),
        .m01_axi_bvalid  (ds_bvalid[1]), .m01_axi_bready  (ds_bready[1]),
        .m01_axi_arid    (ds_arid[1]),  .m01_axi_araddr  (ds_araddr[1]),
        .m01_axi_arlen   (ds_arlen[1]), .m01_axi_arsize  (ds_arsize[1]),
        .m01_axi_arburst (ds_arburst[1]),.m01_axi_arlock  (ds_arlock[1]),
        .m01_axi_arcache (ds_arcache[1]),.m01_axi_arprot  (ds_arprot[1]),
        .m01_axi_arqos   (ds_arqos[1]), .m01_axi_arregion(ds_arregion[1]),
        .m01_axi_arvalid (ds_arvalid[1]),.m01_axi_arready (ds_arready[1]),
        .m01_axi_rid     (ds_rid[1]),   .m01_axi_rdata   (ds_rdata[1]),
        .m01_axi_rresp   (ds_rresp[1]), .m01_axi_rlast   (ds_rlast[1]),
        .m01_axi_rvalid  (ds_rvalid[1]), .m01_axi_rready  (ds_rready[1]),

        // ---- Slave port 2 : m02 DMA ----
        .m02_axi_awid    (ds_awid[2]),  .m02_axi_awaddr  (ds_awaddr[2]),
        .m02_axi_awlen   (ds_awlen[2]), .m02_axi_awsize  (ds_awsize[2]),
        .m02_axi_awburst (ds_awburst[2]),.m02_axi_awlock  (ds_awlock[2]),
        .m02_axi_awcache (ds_awcache[2]),.m02_axi_awprot  (ds_awprot[2]),
        .m02_axi_awqos   (ds_awqos[2]), .m02_axi_awregion(ds_awregion[2]),
        .m02_axi_awvalid (ds_awvalid[2]),.m02_axi_awready (ds_awready[2]),
        .m02_axi_wdata   (ds_wdata[2]), .m02_axi_wstrb   (ds_wstrb[2]),
        .m02_axi_wlast   (ds_wlast[2]), .m02_axi_wvalid  (ds_wvalid[2]),
        .m02_axi_wready  (ds_wready[2]),
        .m02_axi_bid     (ds_bid[2]),   .m02_axi_bresp   (ds_bresp[2]),
        .m02_axi_bvalid  (ds_bvalid[2]), .m02_axi_bready  (ds_bready[2]),
        .m02_axi_arid    (ds_arid[2]),  .m02_axi_araddr  (ds_araddr[2]),
        .m02_axi_arlen   (ds_arlen[2]), .m02_axi_arsize  (ds_arsize[2]),
        .m02_axi_arburst (ds_arburst[2]),.m02_axi_arlock  (ds_arlock[2]),
        .m02_axi_arcache (ds_arcache[2]),.m02_axi_arprot  (ds_arprot[2]),
        .m02_axi_arqos   (ds_arqos[2]), .m02_axi_arregion(ds_arregion[2]),
        .m02_axi_arvalid (ds_arvalid[2]),.m02_axi_arready (ds_arready[2]),
        .m02_axi_rid     (ds_rid[2]),   .m02_axi_rdata   (ds_rdata[2]),
        .m02_axi_rresp   (ds_rresp[2]), .m02_axi_rlast   (ds_rlast[2]),
        .m02_axi_rvalid  (ds_rvalid[2]), .m02_axi_rready  (ds_rready[2]),

        // ---- Slave port 4 : m04 UART ----
        .m04_axi_awid    (ds_awid[3]),  .m04_axi_awaddr  (ds_awaddr[3]),
        .m04_axi_awlen   (ds_awlen[3]), .m04_axi_awsize  (ds_awsize[3]),
        .m04_axi_awburst (ds_awburst[3]),.m04_axi_awlock  (ds_awlock[3]),
        .m04_axi_awcache (ds_awcache[3]),.m04_axi_awprot  (ds_awprot[3]),
        .m04_axi_awqos   (ds_awqos[3]), .m04_axi_awregion(ds_awregion[3]),
        .m04_axi_awvalid (ds_awvalid[3]),.m04_axi_awready (ds_awready[3]),
        .m04_axi_wdata   (ds_wdata[3]), .m04_axi_wstrb   (ds_wstrb[3]),
        .m04_axi_wlast   (ds_wlast[3]), .m04_axi_wvalid  (ds_wvalid[3]),
        .m04_axi_wready  (ds_wready[3]),
        .m04_axi_bid     (ds_bid[3]),   .m04_axi_bresp   (ds_bresp[3]),
        .m04_axi_bvalid  (ds_bvalid[3]), .m04_axi_bready  (ds_bready[3]),
        .m04_axi_arid    (ds_arid[3]),  .m04_axi_araddr  (ds_araddr[3]),
        .m04_axi_arlen   (ds_arlen[3]), .m04_axi_arsize  (ds_arsize[3]),
        .m04_axi_arburst (ds_arburst[3]),.m04_axi_arlock  (ds_arlock[3]),
        .m04_axi_arcache (ds_arcache[3]),.m04_axi_arprot  (ds_arprot[3]),
        .m04_axi_arqos   (ds_arqos[3]), .m04_axi_arregion(ds_arregion[3]),
        .m04_axi_arvalid (ds_arvalid[3]),.m04_axi_arready (ds_arready[3]),
        .m04_axi_rid     (ds_rid[3]),   .m04_axi_rdata   (ds_rdata[3]),
        .m04_axi_rresp   (ds_rresp[3]), .m04_axi_rlast   (ds_rlast[3]),
        .m04_axi_rvalid  (ds_rvalid[3]), .m04_axi_rready  (ds_rready[3]),

        // ---- Slave port 5 : m05 TIMER ----
        .m05_axi_awid    (ds_awid[4]),  .m05_axi_awaddr  (ds_awaddr[4]),
        .m05_axi_awlen   (ds_awlen[4]), .m05_axi_awsize  (ds_awsize[4]),
        .m05_axi_awburst (ds_awburst[4]),.m05_axi_awlock  (ds_awlock[4]),
        .m05_axi_awcache (ds_awcache[4]),.m05_axi_awprot  (ds_awprot[4]),
        .m05_axi_awqos   (ds_awqos[4]), .m05_axi_awregion(ds_awregion[4]),
        .m05_axi_awvalid (ds_awvalid[4]),.m05_axi_awready (ds_awready[4]),
        .m05_axi_wdata   (ds_wdata[4]), .m05_axi_wstrb   (ds_wstrb[4]),
        .m05_axi_wlast   (ds_wlast[4]), .m05_axi_wvalid  (ds_wvalid[4]),
        .m05_axi_wready  (ds_wready[4]),
        .m05_axi_bid     (ds_bid[4]),   .m05_axi_bresp   (ds_bresp[4]),
        .m05_axi_bvalid  (ds_bvalid[4]), .m05_axi_bready  (ds_bready[4]),
        .m05_axi_arid    (ds_arid[4]),  .m05_axi_araddr  (ds_araddr[4]),
        .m05_axi_arlen   (ds_arlen[4]), .m05_axi_arsize  (ds_arsize[4]),
        .m05_axi_arburst (ds_arburst[4]),.m05_axi_arlock  (ds_arlock[4]),
        .m05_axi_arcache (ds_arcache[4]),.m05_axi_arprot  (ds_arprot[4]),
        .m05_axi_arqos   (ds_arqos[4]), .m05_axi_arregion(ds_arregion[4]),
        .m05_axi_arvalid (ds_arvalid[4]),.m05_axi_arready (ds_arready[4]),
        .m05_axi_rid     (ds_rid[4]),   .m05_axi_rdata   (ds_rdata[4]),
        .m05_axi_rresp   (ds_rresp[4]), .m05_axi_rlast   (ds_rlast[4]),
        .m05_axi_rvalid  (ds_rvalid[4]), .m05_axi_rready  (ds_rready[4]),

        // ---- Slave port 6 : m06 GPIO ----
        .m06_axi_awid    (ds_awid[5]),  .m06_axi_awaddr  (ds_awaddr[5]),
        .m06_axi_awlen   (ds_awlen[5]), .m06_axi_awsize  (ds_awsize[5]),
        .m06_axi_awburst (ds_awburst[5]),.m06_axi_awlock  (ds_awlock[5]),
        .m06_axi_awcache (ds_awcache[5]),.m06_axi_awprot  (ds_awprot[5]),
        .m06_axi_awqos   (ds_awqos[5]), .m06_axi_awregion(ds_awregion[5]),
        .m06_axi_awvalid (ds_awvalid[5]),.m06_axi_awready (ds_awready[5]),
        .m06_axi_wdata   (ds_wdata[5]), .m06_axi_wstrb   (ds_wstrb[5]),
        .m06_axi_wlast   (ds_wlast[5]), .m06_axi_wvalid  (ds_wvalid[5]),
        .m06_axi_wready  (ds_wready[5]),
        .m06_axi_bid     (ds_bid[5]),   .m06_axi_bresp   (ds_bresp[5]),
        .m06_axi_bvalid  (ds_bvalid[5]), .m06_axi_bready  (ds_bready[5]),
        .m06_axi_arid    (ds_arid[5]),  .m06_axi_araddr  (ds_araddr[5]),
        .m06_axi_arlen   (ds_arlen[5]), .m06_axi_arsize  (ds_arsize[5]),
        .m06_axi_arburst (ds_arburst[5]),.m06_axi_arlock  (ds_arlock[5]),
        .m06_axi_arcache (ds_arcache[5]),.m06_axi_arprot  (ds_arprot[5]),
        .m06_axi_arqos   (ds_arqos[5]), .m06_axi_arregion(ds_arregion[5]),
        .m06_axi_arvalid (ds_arvalid[5]),.m06_axi_arready (ds_arready[5]),
        .m06_axi_rid     (ds_rid[5]),   .m06_axi_rdata   (ds_rdata[5]),
        .m06_axi_rresp   (ds_rresp[5]), .m06_axi_rlast   (ds_rlast[5]),
        .m06_axi_rvalid  (ds_rvalid[5]), .m06_axi_rready  (ds_rready[5]),

        // ---- Slave port 7 : m07 7-SEG ----
        .m07_axi_awid    (ds_awid[6]),  .m07_axi_awaddr  (ds_awaddr[6]),
        .m07_axi_awlen   (ds_awlen[6]), .m07_axi_awsize  (ds_awsize[6]),
        .m07_axi_awburst (ds_awburst[6]),.m07_axi_awlock  (ds_awlock[6]),
        .m07_axi_awcache (ds_awcache[6]),.m07_axi_awprot  (ds_awprot[6]),
        .m07_axi_awqos   (ds_awqos[6]), .m07_axi_awregion(ds_awregion[6]),
        .m07_axi_awvalid (ds_awvalid[6]),.m07_axi_awready (ds_awready[6]),
        .m07_axi_wdata   (ds_wdata[6]), .m07_axi_wstrb   (ds_wstrb[6]),
        .m07_axi_wlast   (ds_wlast[6]), .m07_axi_wvalid  (ds_wvalid[6]),
        .m07_axi_wready  (ds_wready[6]),
        .m07_axi_bid     (ds_bid[6]),   .m07_axi_bresp   (ds_bresp[6]),
        .m07_axi_bvalid  (ds_bvalid[6]), .m07_axi_bready  (ds_bready[6]),
        .m07_axi_arid    (ds_arid[6]),  .m07_axi_araddr  (ds_araddr[6]),
        .m07_axi_arlen   (ds_arlen[6]), .m07_axi_arsize  (ds_arsize[6]),
        .m07_axi_arburst (ds_arburst[6]),.m07_axi_arlock  (ds_arlock[6]),
        .m07_axi_arcache (ds_arcache[6]),.m07_axi_arprot  (ds_arprot[6]),
        .m07_axi_arqos   (ds_arqos[6]), .m07_axi_arregion(ds_arregion[6]),
        .m07_axi_arvalid (ds_arvalid[6]),.m07_axi_arready (ds_arready[6]),
        .m07_axi_rid     (ds_rid[6]),   .m07_axi_rdata   (ds_rdata[6]),
        .m07_axi_rresp   (ds_rresp[6]), .m07_axi_rlast   (ds_rlast[6]),
        .m07_axi_rvalid  (ds_rvalid[6]), .m07_axi_rready  (ds_rready[6])
    );

    // =========================================================================
    // axi_cnn_conv DUT
    // =========================================================================
    axi_cnn_conv #(
        .DATA_WIDTH       (DW),
        .AXI_DATA_WIDTH   (DW),
        .AXI_ADDR_WIDTH   (AW),
        .AXI_ID_WIDTH     (IDW),
        .AXI_STRB_WIDTH   (SW),
        .BASE_ADDR        (CNN_BASE),
        .CNN_ID_VALUE     (CNN_ID_VALUE),
        .CNN_VERSION_VALUE(CNN_VERSION_VALUE)
    ) u_cnn (
        .clk                (clk),
        .rst                (rst),
        .s_axi_awid         (ic_cnn_awid),
        .s_axi_awaddr       (ic_cnn_awaddr),
        .s_axi_awlen        (ic_cnn_awlen),
        .s_axi_awsize       (ic_cnn_awsize),
        .s_axi_awburst      (ic_cnn_awburst),
        .s_axi_awlock       (ic_cnn_awlock),
        .s_axi_awcache      (ic_cnn_awcache),
        .s_axi_awprot       (ic_cnn_awprot),
        .s_axi_awqos        (ic_cnn_awqos),
        .s_axi_awregion     (ic_cnn_awregion),
        .s_axi_awvalid      (ic_cnn_awvalid),
        .s_axi_awready      (ic_cnn_awready),
        .s_axi_wdata        (ic_cnn_wdata),
        .s_axi_wstrb        (ic_cnn_wstrb),
        .s_axi_wlast        (ic_cnn_wlast),
        .s_axi_wvalid       (ic_cnn_wvalid),
        .s_axi_wready       (ic_cnn_wready),
        .s_axi_bid          (ic_cnn_bid),
        .s_axi_bresp        (ic_cnn_bresp),
        .s_axi_bvalid       (ic_cnn_bvalid),
        .s_axi_bready       (ic_cnn_bready),
        .s_axi_arid         (ic_cnn_arid),
        .s_axi_araddr       (ic_cnn_araddr),
        .s_axi_arlen        (ic_cnn_arlen),
        .s_axi_arsize       (ic_cnn_arsize),
        .s_axi_arburst      (ic_cnn_arburst),
        .s_axi_arlock       (ic_cnn_arlock),
        .s_axi_arcache      (ic_cnn_arcache),
        .s_axi_arprot       (ic_cnn_arprot),
        .s_axi_arqos        (ic_cnn_arqos),
        .s_axi_arregion     (ic_cnn_arregion),
        .s_axi_arvalid      (ic_cnn_arvalid),
        .s_axi_arready      (ic_cnn_arready),
        .s_axi_rid          (ic_cnn_rid),
        .s_axi_rdata        (ic_cnn_rdata),
        .s_axi_rresp        (ic_cnn_rresp),
        .s_axi_rlast        (ic_cnn_rlast),
        .s_axi_rvalid       (ic_cnn_rvalid),
        .s_axi_rready       (ic_cnn_rready),
        .cnn_irq_o          (cnn_irq_o)
    );

    // =========================================================================
    // Test scoreboard
    // =========================================================================
    integer pass_cnt;
    integer fail_cnt;

    task pass_msg;
        input string msg;
        begin
            $display("  [PASS] %s", msg);
            pass_cnt = pass_cnt + 1;
        end
    endtask

    task fail_msg;
        input string msg;
        begin
            $display("  [FAIL] %s", msg);
            fail_cnt = fail_cnt + 1;
        end
    endtask

    // =========================================================================
    // AXI write helper  (full-width 64-bit bus, single beat)
    // =========================================================================
    task axi_write_64;
        input [AW-1:0]  addr;
        input [63:0]    data;
        input [7:0]     strb;
        output [1:0]    resp;

        begin
            // Ensure clean gap from previous transaction
            repeat (2) @(posedge clk);
            @(posedge clk); #1;

            m_awid     <= 8'h01;
            m_awaddr   <= addr;
            m_awlen    <= 8'h00;
            m_awsize   <= 3'b011;
            m_awburst  <= 2'b01;
            m_awlock   <= 1'b0;
            m_awcache  <= 4'b0000;
            m_awprot   <= 3'b000;
            m_awqos    <= 4'b0000;
            m_awvalid  <= 1'b1;

            m_wdata    <= data;
            m_wstrb    <= strb;
            m_wlast    <= 1'b1;
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
            resp = m_bresp;
            @(posedge clk); #1;
            m_bready <= 1'b0;

            repeat (4) @(posedge clk);
        end
    endtask

    task axi_write_32;
        input [AW-1:0]  addr;
        input [31:0]    data;
        output [1:0]    resp;
        begin
            axi_write_64(addr, {32'h0, data}, 8'h0F, resp);
        end
    endtask

    // =========================================================================
    // AXI read helper  (single beat)
    // =========================================================================
    task axi_read_64;
        input  [AW-1:0] addr;
        output [63:0]   data;
        output [1:0]    resp;

        begin
            @(posedge clk); #1;

            m_arid    <= 8'h02;
            m_araddr  <= addr;
            m_arlen   <= 8'h00;
            m_arsize  <= 3'b011;
            m_arburst <= 2'b01;
            m_arlock  <= 1'b0;
            m_arcache <= 4'b0000;
            m_arprot  <= 3'b000;
            m_arqos   <= 4'b0000;
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

    task master_idle;
        begin
            m_awvalid <= 1'b0; m_wvalid <= 1'b0; m_bready <= 1'b0;
            m_arvalid <= 1'b0; m_rready <= 1'b0;
        end
    endtask

    // =========================================================================
    // T E S T   T A S K S
    // =========================================================================

    task t1_reset;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T1: Reset defaults ==");

            rst = 1'b1;
            repeat (10) @(posedge clk);
            rst = 1'b0;
            repeat (5) @(posedge clk);

            axi_read_32(CNN_CTRL, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_CTRL = 0 after reset");
            else begin
                $display("  [FAIL] CNN_CTRL after reset: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_STATUS, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_STATUS = 0 after reset");
            else begin
                $display("  [FAIL] CNN_STATUS after reset: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_IN_BASE, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_IN_BASE = 0 after reset");
            else begin
                $display("  [FAIL] CNN_IN_BASE after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_OUT_BASE, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_OUT_BASE = 0 after reset");
            else begin
                $display("  [FAIL] CNN_OUT_BASE after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_IMG_W, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_IMG_W = 0 after reset");
            else begin
                $display("  [FAIL] CNN_IMG_W after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_IMG_H, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_IMG_H = 0 after reset");
            else begin
                $display("  [FAIL] CNN_IMG_H after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_K00, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_K00 = 0 after reset");
            else begin
                $display("  [FAIL] CNN_K00 after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_K11, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_K11 = 0 after reset");
            else begin
                $display("  [FAIL] CNN_K11 after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_K22, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_K22 = 0 after reset");
            else begin
                $display("  [FAIL] CNN_K22 after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_BIAS, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("CNN_BIAS = 0 after reset");
            else begin
                $display("  [FAIL] CNN_BIAS after reset: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_ID_REG, rd, resp);
            if (rd === CNN_ID_VALUE && resp === 2'b00)
                pass_msg("CNN_ID = 0x434E4E56");
            else begin
                $display("  [FAIL] CNN_ID: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(CNN_VERSION, rd, resp);
            if (rd === CNN_VERSION_VALUE && resp === 2'b00)
                pass_msg("CNN_VERSION = 0x00010000");
            else begin
                $display("  [FAIL] CNN_VERSION: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            check_bit("T1 cnn_irq_o = 0", cnn_irq_o, 1'b0);
            $display("  T1 done.");
        end
    endtask

    task t2_register_write_readback;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T2: Register write/readback ==");

            axi_write_32(CNN_CTRL, 32'h7, resp_r);  // start+irq_en+reset_acc
            axi_read_32(CNN_CTRL, rd, resp_r);
            if (rd[2:0] === 3'b111)
                pass_msg("CNN_CTRL readback = 0x7 (bits[2:0])");
            else begin
                $display("  [FAIL] CNN_CTRL readback: got 0x%08h resp %b", rd, resp_r);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_IN_BASE,  32'h1000_0000, resp_r);
            axi_read_32(CNN_IN_BASE, rd, resp_r);
            if (rd === 32'h1000_0000)
                pass_msg("CNN_IN_BASE readback = 0x10000000");
            else begin
                $display("  [FAIL] CNN_IN_BASE readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_OUT_BASE, 32'h2000_0000, resp_r);
            axi_read_32(CNN_OUT_BASE, rd, resp_r);
            if (rd === 32'h2000_0000)
                pass_msg("CNN_OUT_BASE readback = 0x20000000");
            else begin
                $display("  [FAIL] CNN_OUT_BASE readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_IMG_W, 32'd28, resp_r);
            axi_read_32(CNN_IMG_W, rd, resp_r);
            if (rd === 32'd28)
                pass_msg("CNN_IMG_W readback = 28");
            else begin
                $display("  [FAIL] CNN_IMG_W readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_IMG_H, 32'd28, resp_r);
            axi_read_32(CNN_IMG_H, rd, resp_r);
            if (rd === 32'd28)
                pass_msg("CNN_IMG_H readback = 28");
            else begin
                $display("  [FAIL] CNN_IMG_H readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_K00, 32'd1, resp_r);
            axi_read_32(CNN_K00, rd, resp_r);
            if (rd === 32'd1)
                pass_msg("CNN_K00 readback = 1");
            else begin
                $display("  [FAIL] CNN_K00 readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_K11, 32'd1, resp_r);
            axi_read_32(CNN_K11, rd, resp_r);
            if (rd === 32'd1)
                pass_msg("CNN_K11 readback = 1");
            else begin
                $display("  [FAIL] CNN_K11 readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(CNN_BIAS, 32'd5, resp_r);
            axi_read_32(CNN_BIAS, rd, resp_r);
            if (rd === 32'd5)
                pass_msg("CNN_BIAS readback = 5");
            else begin
                $display("  [FAIL] CNN_BIAS readback: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            // Clean up
            axi_write(CNN_CTRL, 32'h0, 4'hF, resp_r);
            $display("  T2 done.");
        end
    endtask

    task t3_convolution_computation;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T3: Convolution computation ==");

            // Stop any running convolution from T2
            axi_write(CNN_CTRL, 32'h0, 4'hF, resp_r);
            repeat (5) @(posedge clk);  // let conv_running go to 0

            axi_write(CNN_IN_BASE,  32'h1000_0000, 4'hF, resp_r);
            axi_write(CNN_OUT_BASE, 32'h2000_0000, 4'hF, resp_r);
            axi_write(CNN_IMG_W,    32'd3,   4'hF, resp_r);
            axi_write(CNN_IMG_H,    32'd3,   4'hF, resp_r);
            // Identity kernel: only K11=1
            axi_write(CNN_K00, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K01, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K02, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K10, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K11, 32'd1, 4'hF, resp_r);
            axi_write(CNN_K12, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K20, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K21, 32'd0, 4'hF, resp_r);
            axi_write(CNN_K22, 32'd0, 4'hF, resp_r);
            axi_write(CNN_BIAS,   32'd0, 4'hF, resp_r);

            // Write input pixels (all ones)
            axi_write(CNN_PX00, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX01, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX02, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX10, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX11, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX12, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX20, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX21, 32'd1, 4'hF, resp_r);
            axi_write(CNN_PX22, 32'd1, 4'hF, resp_r);

            // Start convolution
            axi_write(CNN_CTRL, 32'h1, 4'hF, resp_r);  // start=1

            // Wait for done
            repeat (20) @(posedge clk);

            axi_read(CNN_STATUS, rd, resp_r);
            if (rd[STATUS_DONE] === 1'b1)
                pass_msg("T3 STATUS done");
            else begin
                $display("  [FAIL] T3 STATUS done: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            check_bit("T3 cnn_irq_o (irq_en=0)", cnn_irq_o, 1'b0);

            // Read convolution result
            axi_read(CNN_RESULT, rd, resp_r);
            if (rd === 32'd1)
                pass_msg("T3 result (center pixel = 1)");
            else begin
                $display("  [FAIL] T3 result: got 0x%08h, expected 1", rd);
                fail_cnt = fail_cnt + 1;
            end

            $display("  T3 done.");
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    reg [1:0]               resp_r;
    reg [31:0]              rd_r;

    initial begin
        pass_cnt = 0;
        fail_cnt = 0;
        resp_r   = 2'b00;
        rd_r     = 32'h0;

        $display("");
        $display("============================================================");
        $display("  tb_axi_cnn_conv — CNN Convolution IP Testbench");
        $display("============================================================");
        $display("");

        rst = 1'b1;
        idle_bus();
        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // Run tests
        t1_reset();
        t2_register_write_readback();
        t3_convolution_computation();
        // T4-T7 would go here

        // Summary
        $display("");
        $display("============================================================");
        $display("  Test Summary");
        $display("============================================================");
        $display("  Passed: %0d", pass_cnt);
        $display("  Failed: %0d", fail_cnt);
        $display("============================================================");
        $display("");

        if (fail_cnt > 0) begin
            $display("*** TESTBENCH FAILED ***");
            $fatal(1, "tb_axi_cnn_conv: %0d checks failed", fail_cnt);
        end else begin
            $display("*** ALL TESTS PASSED ***");
            $finish;
        end
    end

    // Watchdog — catch deadlocks
    initial begin
        #(CLK_PERIOD * 200000);
        $display("");
        $display("  [FAIL] WATCHDOG: simulation timeout (deadlock detected)");
        $fatal(2, "tb_axi_cnn_conv: watchdog timeout");
    end

endmodule

`default_nettype wire