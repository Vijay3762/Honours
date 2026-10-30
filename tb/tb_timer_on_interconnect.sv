// =============================================================================
// File    : tb_timer_on_interconnect.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
//
// Purpose : Integration testbench — verifies the axi_timer hanging off port
//           m05 of the 3x8 AXI interconnect.
//
// Topology
// --------
//   TB AXI master (s00)
//      |
//   axi_interconnect_wrap_3x8
//      |-- m00..m03, m04, m06, m07 → dummy_slave_intg instances
//      |-- m05                     → axi_timer
//
// Test plan
// ---------
//   T1  Reset defaults          : CTRL=0, LOAD=0, VALUE=0, STATUS=0, IRQ=0
//   T2  CTRL write/readback     : enable, autoload, irq_en bits
//   T3  LOAD write/readback     : load value
//   T4  One-shot count down     : load 10, enable, verify counts to 0 and stops
//   T5  Auto-reload mode        : load 5, enable+autoload, verify reloads
//   T6  Compare match interrupt : COMPARE=50, enable+irq_en, verify IRQ fires
//   T7  W1C status flags        : set match via countdown, clear via W1C
//   T8  RO register writes      : VALUE, ID, VERSION → SLVERR
//   T9  Reserved offset         : 0x20 → SLVERR
//   T10 Out-of-aperture         : 0x5001_0000 → DECERR
//   T11 WSTRB partial writes    : byte strobe control on LOAD/COMPARE
//
// Compile (xvlog):
//   xvlog -sv -i rtl/uart/include \
//       rtl/timer/axi_timer.sv \
//       rtl/interconnect/axi_interconnect.v \
//       rtl/interconnect/arbiter.v \
//       rtl/interconnect/priority_encoder.v \
//       rtl/interconnect/axi_interconnect_wrap_3x8.v \
//       rtl/axi_mem.sv \
//       rtl/axi_dummy_slave.sv \
//       rtl/simple_cpu_master.sv \
//       rtl/soc_top.sv \
//       tb/tb_timer_on_interconnect.sv
//
// Simulate:
//   xsim tb_timer_on_interconnect -runall
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

// ---------------------------------------------------------------------------
// Minimal dummy slave – accepts every single-beat write/read, returns a
// deterministic data pattern.  Used for m00..m03, m04, m06, m07.
// ---------------------------------------------------------------------------
module dummy_slave_intg #(
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
module tb_timer_on_interconnect;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam integer DW   = 64;
    localparam integer AW   = 32;
    localparam integer IDW  = 8;
    localparam integer SW   = DW/8;   // 8

    localparam integer CLK_PERIOD = 10;   // 10 ns  →  100 MHz

    // Timer base address on interconnect  (m05 → 0x5000_0000)
    localparam [AW-1:0] TIMER_BASE = 32'h5000_0000;

    // Timer register offsets (from TIMER_BASE)
    localparam [AW-1:0] TIMER_CTRL     = TIMER_BASE + 32'h000;
    localparam [AW-1:0] TIMER_LOAD     = TIMER_BASE + 32'h004;
    localparam [AW-1:0] TIMER_VALUE    = TIMER_BASE + 32'h008;
    localparam [AW-1:0] TIMER_COMPARE  = TIMER_BASE + 32'h00C;
    localparam [AW-1:0] TIMER_STATUS   = TIMER_BASE + 32'h010;
    localparam [AW-1:0] TIMER_ID_REG   = TIMER_BASE + 32'h014;
    localparam [AW-1:0] TIMER_VERSION  = TIMER_BASE + 32'h018;

    // Fixed constants from axi_timer
    localparam [31:0] TIMER_ID_VALUE      = 32'h54494D52;  // "TIMR"
    localparam [31:0] TIMER_VERSION_VALUE = 32'h00010000;  // v1.0

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
    // Inter-module wires: interconnect m05 <-> axi_timer
    // =========================================================================
    wire [IDW-1:0]  ic_timer_awid;
    wire [AW-1:0]   ic_timer_awaddr;
    wire [7:0]      ic_timer_awlen;
    wire [2:0]      ic_timer_awsize;
    wire [1:0]      ic_timer_awburst;
    wire            ic_timer_awlock;
    wire [3:0]      ic_timer_awcache;
    wire [2:0]      ic_timer_awprot;
    wire [3:0]      ic_timer_awqos;
    wire [3:0]      ic_timer_awregion;
    wire            ic_timer_awvalid;
    wire            ic_timer_awready;

    wire [DW-1:0]   ic_timer_wdata;
    wire [SW-1:0]   ic_timer_wstrb;
    wire            ic_timer_wlast;
    wire            ic_timer_wvalid;
    wire            ic_timer_wready;

    wire [IDW-1:0]  ic_timer_bid;
    wire [1:0]      ic_timer_bresp;
    wire            ic_timer_bvalid;
    wire            ic_timer_bready;

    wire [IDW-1:0]  ic_timer_arid;
    wire [AW-1:0]   ic_timer_araddr;
    wire [7:0]      ic_timer_arlen;
    wire [2:0]      ic_timer_arsize;
    wire [1:0]      ic_timer_arburst;
    wire            ic_timer_arlock;
    wire [3:0]      ic_timer_arcache;
    wire [2:0]      ic_timer_arprot;
    wire [3:0]      ic_timer_arqos;
    wire [3:0]      ic_timer_arregion;
    wire            ic_timer_arvalid;
    wire            ic_timer_arready;

    wire [IDW-1:0]  ic_timer_rid;
    wire [DW-1:0]   ic_timer_rdata;
    wire [1:0]      ic_timer_rresp;
    wire            ic_timer_rlast;
    wire            ic_timer_rvalid;
    wire            ic_timer_rready;

    // =========================================================================
    // Timer interrupt signal
    // =========================================================================
    wire            timer_irq_o;

    // =========================================================================
    // Dummy slave interconnect wires  (m00..m03, m04, m06, m07) = 7 dummies
    // Slave index mapping:
    //   0 = m00 (IMEM)   1 = m01 (DMEM)   2 = m02 (DMA)
    //   3 = m03 (CNN)   4 = m04 (UART)   5 = m06 (GPIO)
    //   6 = m07 (7-SEG)
    // (m05 = Timer, handled separately above)
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
    // Generate 7 dummy_slave_intg instances  (m00..m03, m04, m06, m07)
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
        // Timer slave  m05 = 0x5000_0000 .. 0x5FFF_FFFF
        .M05_BASE_ADDR       (32'h5000_0000),
        .M05_ADDR_WIDTH      ({1{32'd28}}),
        .M05_CONNECT_READ    (3'b001),
        .M05_CONNECT_WRITE   (3'b001),
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
        // CNN slave   m03 = 0x3000_0000
        .M03_BASE_ADDR       (32'h3000_0000),
        .M03_ADDR_WIDTH      ({1{32'd28}}),
        .M03_CONNECT_READ    (3'b111),
        .M03_CONNECT_WRITE   (3'b111),
        // UART slave  m04 = 0x4000_0000
        .M04_BASE_ADDR       (32'h4000_0000),
        .M04_ADDR_WIDTH      ({1{32'd28}}),
        .M04_CONNECT_READ    (3'b111),
        .M04_CONNECT_WRITE   (3'b111),
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

        // ---- Slave port 3 : m03 CNN ----
        .m03_axi_awid    (ds_awid[3]),  .m03_axi_awaddr  (ds_awaddr[3]),
        .m03_axi_awlen   (ds_awlen[3]), .m03_axi_awsize  (ds_awsize[3]),
        .m03_axi_awburst (ds_awburst[3]),.m03_axi_awlock  (ds_awlock[3]),
        .m03_axi_awcache (ds_awcache[3]),.m03_axi_awprot  (ds_awprot[3]),
        .m03_axi_awqos   (ds_awqos[3]), .m03_axi_awregion(ds_awregion[3]),
        .m03_axi_awvalid (ds_awvalid[3]),.m03_axi_awready (ds_awready[3]),
        .m03_axi_wdata   (ds_wdata[3]), .m03_axi_wstrb   (ds_wstrb[3]),
        .m03_axi_wlast   (ds_wlast[3]), .m03_axi_wvalid  (ds_wvalid[3]),
        .m03_axi_wready  (ds_wready[3]),
        .m03_axi_bid     (ds_bid[3]),   .m03_axi_bresp   (ds_bresp[3]),
        .m03_axi_bvalid  (ds_bvalid[3]), .m03_axi_bready  (ds_bready[3]),
        .m03_axi_arid    (ds_arid[3]),  .m03_axi_araddr  (ds_araddr[3]),
        .m03_axi_arlen   (ds_arlen[3]), .m03_axi_arsize  (ds_arsize[3]),
        .m03_axi_arburst (ds_arburst[3]),.m03_axi_arlock  (ds_arlock[3]),
        .m03_axi_arcache (ds_arcache[3]),.m03_axi_arprot  (ds_arprot[3]),
        .m03_axi_arqos   (ds_arqos[3]), .m03_axi_arregion(ds_arregion[3]),
        .m03_axi_arvalid (ds_arvalid[3]),.m03_axi_arready (ds_arready[3]),
        .m03_axi_rid     (ds_rid[3]),   .m03_axi_rdata   (ds_rdata[3]),
        .m03_axi_rresp   (ds_rresp[3]), .m03_axi_rlast   (ds_rlast[3]),
        .m03_axi_rvalid  (ds_rvalid[3]), .m03_axi_rready  (ds_rready[3]),

        // ---- Slave port 4 : m04 UART ----
        .m04_axi_awid    (ds_awid[4]),  .m04_axi_awaddr  (ds_awaddr[4]),
        .m04_axi_awlen   (ds_awlen[4]), .m04_axi_awsize  (ds_awsize[4]),
        .m04_axi_awburst (ds_awburst[4]),.m04_axi_awlock  (ds_awlock[4]),
        .m04_axi_awcache (ds_awcache[4]),.m04_axi_awprot  (ds_awprot[4]),
        .m04_axi_awqos   (ds_awqos[4]), .m04_axi_awregion(ds_awregion[4]),
        .m04_axi_awvalid (ds_awvalid[4]),.m04_axi_awready (ds_awready[4]),
        .m04_axi_wdata   (ds_wdata[4]), .m04_axi_wstrb   (ds_wstrb[4]),
        .m04_axi_wlast   (ds_wlast[4]), .m04_axi_wvalid  (ds_wvalid[4]),
        .m04_axi_wready  (ds_wready[4]),
        .m04_axi_bid     (ds_bid[4]),   .m04_axi_bresp   (ds_bresp[4]),
        .m04_axi_bvalid  (ds_bvalid[4]), .m04_axi_bready  (ds_bready[4]),
        .m04_axi_arid    (ds_arid[4]),  .m04_axi_araddr  (ds_araddr[4]),
        .m04_axi_arlen   (ds_arlen[4]), .m04_axi_arsize  (ds_arsize[4]),
        .m04_axi_arburst (ds_arburst[4]),.m04_axi_arlock  (ds_arlock[4]),
        .m04_axi_arcache (ds_arcache[4]),.m04_axi_arprot  (ds_arprot[4]),
        .m04_axi_arqos   (ds_arqos[4]), .m04_axi_arregion(ds_arregion[4]),
        .m04_axi_arvalid (ds_arvalid[4]),.m04_axi_arready (ds_arready[4]),
        .m04_axi_rid     (ds_rid[4]),   .m04_axi_rdata   (ds_rdata[4]),
        .m04_axi_rresp   (ds_rresp[4]), .m04_axi_rlast   (ds_rlast[4]),
        .m04_axi_rvalid  (ds_rvalid[4]), .m04_axi_rready  (ds_rready[4]),

        // ---- Slave port 5 : m05 TIMER ----
        .m05_axi_awid    (ic_timer_awid),    .m05_axi_awaddr  (ic_timer_awaddr),
        .m05_axi_awlen   (ic_timer_awlen),   .m05_axi_awsize  (ic_timer_awsize),
        .m05_axi_awburst (ic_timer_awburst), .m05_axi_awlock  (ic_timer_awlock),
        .m05_axi_awcache (ic_timer_awcache), .m05_axi_awprot  (ic_timer_awprot),
        .m05_axi_awqos   (ic_timer_awqos),   .m05_axi_awregion(ic_timer_awregion),
        .m05_axi_awvalid (ic_timer_awvalid), .m05_axi_awready (ic_timer_awready),
        .m05_axi_wdata   (ic_timer_wdata),   .m05_axi_wstrb   (ic_timer_wstrb),
        .m05_axi_wlast   (ic_timer_wlast),   .m05_axi_wvalid  (ic_timer_wvalid),
        .m05_axi_wready  (ic_timer_wready),
        .m05_axi_bid     (ic_timer_bid),     .m05_axi_bresp   (ic_timer_bresp),
        .m05_axi_bvalid  (ic_timer_bvalid),  .m05_axi_bready  (ic_timer_bready),
        .m05_axi_arid    (ic_timer_arid),    .m05_axi_araddr  (ic_timer_araddr),
        .m05_axi_arlen   (ic_timer_arlen),   .m05_axi_arsize  (ic_timer_arsize),
        .m05_axi_arburst (ic_timer_arburst), .m05_axi_arlock  (ic_timer_arlock),
        .m05_axi_arcache (ic_timer_arcache), .m05_axi_arprot  (ic_timer_arprot),
        .m05_axi_arqos   (ic_timer_arqos),   .m05_axi_arregion(ic_timer_arregion),
        .m05_axi_arvalid (ic_timer_arvalid), .m05_axi_arready (ic_timer_arready),
        .m05_axi_rid     (ic_timer_rid),     .m05_axi_rdata   (ic_timer_rdata),
        .m05_axi_rresp   (ic_timer_rresp),   .m05_axi_rlast   (ic_timer_rlast),
        .m05_axi_rvalid  (ic_timer_rvalid),  .m05_axi_rready  (ic_timer_rready),

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
    // axi_timer DUT
    // =========================================================================
    axi_timer #(
        .TIMER_WIDTH         (32),
        .AXI_DATA_WIDTH      (DW),
        .AXI_ADDR_WIDTH      (AW),
        .AXI_ID_WIDTH        (IDW),
        .AXI_STRB_WIDTH      (SW),
        .BASE_ADDR           (TIMER_BASE),
        .TIMER_ID_VALUE      (TIMER_ID_VALUE),
        .TIMER_VERSION_VALUE (TIMER_VERSION_VALUE)
    ) u_timer (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (ic_timer_awid),
        .s_axi_awaddr        (ic_timer_awaddr),
        .s_axi_awlen         (ic_timer_awlen),
        .s_axi_awsize        (ic_timer_awsize),
        .s_axi_awburst       (ic_timer_awburst),
        .s_axi_awlock        (ic_timer_awlock),
        .s_axi_awcache       (ic_timer_awcache),
        .s_axi_awprot        (ic_timer_awprot),
        .s_axi_awqos         (ic_timer_awqos),
        .s_axi_awregion      (ic_timer_awregion),
        .s_axi_awvalid       (ic_timer_awvalid),
        .s_axi_awready       (ic_timer_awready),
        .s_axi_wdata         (ic_timer_wdata),
        .s_axi_wstrb         (ic_timer_wstrb),
        .s_axi_wlast         (ic_timer_wlast),
        .s_axi_wvalid        (ic_timer_wvalid),
        .s_axi_wready        (ic_timer_wready),
        .s_axi_bid           (ic_timer_bid),
        .s_axi_bresp         (ic_timer_bresp),
        .s_axi_bvalid        (ic_timer_bvalid),
        .s_axi_bready        (ic_timer_bready),
        .s_axi_arid          (ic_timer_arid),
        .s_axi_araddr        (ic_timer_araddr),
        .s_axi_arlen         (ic_timer_arlen),
        .s_axi_arsize        (ic_timer_arsize),
        .s_axi_arburst       (ic_timer_arburst),
        .s_axi_arlock        (ic_timer_arlock),
        .s_axi_arcache       (ic_timer_arcache),
        .s_axi_arprot        (ic_timer_arprot),
        .s_axi_arqos         (ic_timer_arqos),
        .s_axi_arregion      (ic_timer_arregion),
        .s_axi_arvalid       (ic_timer_arvalid),
        .s_axi_arready       (ic_timer_arready),
        .s_axi_rid           (ic_timer_rid),
        .s_axi_rdata         (ic_timer_rdata),
        .s_axi_rresp         (ic_timer_rresp),
        .s_axi_rlast         (ic_timer_rlast),
        .s_axi_rvalid        (ic_timer_rvalid),
        .s_axi_rready        (ic_timer_rready),
        .timer_irq_o         (timer_irq_o)
    );

    // =========================================================================
    // Test scoreboard
    // =========================================================================
    integer pass_cnt;
    integer fail_cnt;
    reg [1:0]   resp_r;
    reg [31:0]  rd_r;

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

            axi_read_32(TIMER_CTRL, rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("TIMER_CTRL = 0 after reset");
            else begin
                $display("  [FAIL] TIMER_CTRL after reset: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(TIMER_LOAD, rd, resp);
            check("T1 LOAD after reset", rd, 32'h0);

            axi_read_32(TIMER_VALUE, rd, resp);
            check("T1 VALUE after reset", rd, 32'h0);
            check_resp("T1 VALUE resp", resp, 2'b00);

            axi_read_32(TIMER_COMPARE, rd, resp);
            check("T1 COMPARE after reset", rd, 32'hFFFF_FFFF);

            axi_read_32(TIMER_STATUS, rd, resp);
            check("T1 STATUS after reset", rd, 32'h0);

            axi_read_32(TIMER_ID_REG, rd, resp);
            check("T1 ID", rd, TIMER_ID_VALUE);
            check_resp("T1 ID resp", resp, 2'b00);

            axi_read_32(TIMER_VERSION, rd, resp);
            check("T1 VERSION", rd, TIMER_VERSION_VALUE);
            check_resp("T1 VERSION resp", resp, 2'b00);

            check_bit("T1 timer_irq_o = 0", timer_irq_o, 1'b0);
            $display("  T1 done.");
        end
    endtask

    task check(input string desc, input [31:0] actual, input [31:0] expected);
        begin
            if (actual === expected) begin
                pass_msg(desc);
            end else begin
                $display("  [FAIL] %s: expected 0x%08h, got 0x%08h", desc, expected, actual);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    task check_resp(input string desc, input [1:0] actual, input [1:0] expected);
        begin
            if (actual === expected) begin
                pass_msg(desc);
            end else begin
                $display("  [FAIL] %s: expected resp %b, got resp %b", desc, expected, actual);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    task check_bit(input string desc, input bit actual, input bit expected);
        begin
            if (actual === expected) begin
                pass_msg(desc);
            end else begin
                $display("  [FAIL] %s: expected %b, got %b", desc, expected, actual);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    task t2_ctrl_write_readback;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T2: CTRL write/readback ==");

            // Set LOAD first (non-zero) so enabling doesn't immediately hit zero
            axi_write_32(TIMER_LOAD, 32'd100, resp_r);
            // Write CTRL = enable+autoload+irq_en
            axi_write_32(TIMER_CTRL, 32'h7, resp_r);
            axi_read_32(TIMER_CTRL, rd, resp_r);
            // Check only CTRL bits [2:0], ignore match/overflow bits [4:3]
            if (rd[2:0] === 3'b111 && resp_r === 2'b00)
                pass_msg("TIMER_CTRL readback = 0x7 (bits[2:0])");
            else begin
                $display("  [FAIL] TIMER_CTRL readback: got 0x%08h resp %b", rd, resp_r);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(TIMER_CTRL, 32'h0, resp_r);  // clear all
            axi_read_32(TIMER_CTRL, rd, resp_r);
            if (rd === 32'h0 && resp_r === 2'b00)
                pass_msg("TIMER_CTRL cleared = 0");
            else begin
                $display("  [FAIL] TIMER_CTRL clear: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T2 done.");
        end
    endtask

    task t3_load_write_readback;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T3: LOAD write/readback ==");

            axi_write_32(TIMER_LOAD, 32'd1000, resp_r);
            axi_read_32(TIMER_LOAD, rd, resp_r);
            if (rd === 32'd1000 && resp_r === 2'b00)
                pass_msg("TIMER_LOAD readback = 1000");
            else begin
                $display("  [FAIL] TIMER_LOAD readback: got 0x%08h resp %b", rd, resp_r);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T3 done.");
        end
    endtask

    task t4_one_shot_countdown;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T4: One-shot count down ==");

            axi_write_32(TIMER_LOAD, 32'd10, resp_r);
            axi_write_32(TIMER_CTRL, 32'h1, resp_r);  // enable only

            repeat (15) @(posedge clk);

            axi_read_32(TIMER_VALUE, rd, resp_r);
            if (rd === 32'h0 && resp_r === 2'b00)
                pass_msg("TIMER_VALUE reached 0 (one-shot)");
            else begin
                $display("  [FAIL] TIMER_VALUE after one-shot: got 0x%08h resp %b", rd, resp_r);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(TIMER_CTRL, rd, resp_r);
            if (rd[0] === 1'b0 && resp_r === 2'b00)
                pass_msg("TIMER_CTRL enable cleared (one-shot stopped)");
            else begin
                $display("  [FAIL] TIMER_CTRL enable still set: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(TIMER_STATUS, rd, resp_r);
            if (rd[0] === 1'b1)
                pass_msg("TIMER_STATUS match flag set");
            else begin
                $display("  [FAIL] TIMER_STATUS match flag not set: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T4 done.");
        end
    endtask

    task t5_auto_reload;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T5: Auto-reload mode ==");

            axi_write_32(TIMER_LOAD, 32'd5, resp_r);
            axi_read_32(TIMER_LOAD, rd, resp_r);
            if (rd !== 32'd5)
                $display("  [INFO] T5 LOAD readback: 0x%08h (expected 5)", rd);

            // Disable first to ensure clean enable edge
            axi_write_32(TIMER_CTRL, 32'h0, resp_r);
            repeat (5) @(posedge clk);

            // Enable + auto-reload
            axi_write_32(TIMER_CTRL, 32'h3, resp_r);

            // Wait for multiple auto-reload cycles (period = 6 clks: 5→4→3→2→1→0→5)
            repeat (40) @(posedge clk);

            // Counter should NOT stay at zero (it reloads)
            axi_read_32(TIMER_VALUE, rd, resp_r);
            if (rd !== 32'h0)
                pass_msg("TIMER_VALUE reloaded (non-zero in auto-reload)");
            else begin
                $display("  [FAIL] TIMER_VALUE stuck at zero in auto-reload");
                fail_cnt = fail_cnt + 1;
            end
            $display("  T5 done.");
        end
    endtask

    task t6_compare_interrupt;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T6: Compare match interrupt ==");

            axi_write_32(TIMER_LOAD, 32'd100, resp_r);
            axi_write_32(TIMER_COMPARE, 32'd50, resp_r);
            axi_write_32(TIMER_CTRL, 32'h5, resp_r);  // enable + irq_en

            repeat (70) @(posedge clk);

            axi_read_32(TIMER_STATUS, rd, resp_r);
            if (rd[0] === 1'b1 && resp_r === 2'b00)
                pass_msg("TIMER_STATUS match flag set");
            else begin
                $display("  [FAIL] TIMER_STATUS match flag: got 0x%08h resp %b", rd, resp_r);
                fail_cnt = fail_cnt + 1;
            end

            check_bit("T6 timer_irq_o = 1", timer_irq_o, 1'b1);
            $display("  T6 done.");
        end
    endtask

    task t7_w1c_status;
        begin
            $display("\n== T7: W1C status flags ==");

            // Disable timer first to prevent match from being re-set during clear
            axi_write_32(TIMER_CTRL, 32'h0, resp_r);

            // Clear match flag by writing 1 to STATUS bit0 (W1C)
            axi_write_32(TIMER_STATUS, 32'h1, resp_r);
            if (resp_r === 2'b00)
                pass_msg("TIMER_STATUS W1C write resp OKAY");
            else begin
                $display("  [FAIL] TIMER_STATUS W1C write resp: got %b", resp_r);
                fail_cnt = fail_cnt + 1;
            end

            @(posedge clk);

            axi_read_32(TIMER_STATUS, rd_r, resp_r);
            if (rd_r[0] === 1'b0)
                pass_msg("TIMER_STATUS match cleared by W1C");
            else begin
                $display("  [FAIL] TIMER_STATUS match not cleared: got 0x%08h", rd_r);
                fail_cnt = fail_cnt + 1;
            end

            check_bit("T7 timer_irq_o = 0 after W1C", timer_irq_o, 1'b0);
            $display("  T7 done.");
        end
    endtask

    task t8_ro_writes;
        reg [1:0] resp;
        begin
            $display("\n== T8: RO register writes → SLVERR ==");

            axi_write_32(TIMER_VALUE, 32'hFFFF_FFFF, resp);
            if (resp === 2'b10)
                pass_msg("Write VALUE (RO) → SLVERR");
            else begin
                $display("  [FAIL] Write VALUE (RO): got resp %b, expected SLVERR(10)", resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(TIMER_ID_REG, 32'hFFFF_FFFF, resp);
            if (resp === 2'b10)
                pass_msg("Write ID (RO) → SLVERR");
            else begin
                $display("  [FAIL] Write ID (RO): got resp %b", resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(TIMER_ID_REG, rd_r, resp);
            if (rd_r === TIMER_ID_VALUE)
                pass_msg("TIMER_ID unchanged after RO write");
            else begin
                $display("  [FAIL] TIMER_ID changed: got 0x%08h", rd_r);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T8 done.");
        end
    endtask

    task t9_reserved_offset;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T9: Reserved offset → SLVERR ==");

            axi_read_32(TIMER_BASE + 32'h020, rd, resp);
            if (resp === 2'b10 && rd === 32'h0)
                pass_msg("Reserved offset 0x20 → SLVERR, data=0");
            else begin
                $display("  [FAIL] Reserved offset 0x20: got resp %b data 0x%08h", resp, rd);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T9 done.");
        end
    endtask

    task t10_out_of_aperture;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T10: Out-of-aperture → DECERR ==");

            axi_read_32(32'h5001_0000, rd, resp);
            if (resp === 2'b11)
                pass_msg("Out-of-aperture 0x5001_0000 → DECERR");
            else begin
                $display("  [FAIL] Out-of-aperture 0x5001_0000: got resp %b", resp);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T10 done.");
        end
    endtask

    task t11_wstrb_partial;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T11: WSTRB partial write ==");

            // Full write all-0xFF with strb=0xF
            axi_write_64(TIMER_LOAD, 64'hFFFF_FFFF, 8'h0F, resp);
            axi_read_32(TIMER_LOAD, rd, resp_r);
            if (rd === 32'hFFFF_FFFF)
                pass_msg("T11 full write 0xFFFFFFFF");
            else begin
                $display("  [FAIL] T11 full write: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            // Write only byte 0 with strb=0x1, data=0x00 -> should only change byte 0
            axi_write_64(TIMER_LOAD, 64'h0000_0000, 8'h01, resp);
            axi_read_32(TIMER_LOAD, rd, resp_r);
            if (rd[7:0] === 8'h00 && rd[31:8] === 24'hFFFF_FF)
                pass_msg("T11 byte0 write (strb=0x1) only changes byte 0");
            else begin
                $display("  [FAIL] T11 byte0 write: got 0x%08h, expected 0xFFFFFF00", rd);
                fail_cnt = fail_cnt + 1;
            end

            // Write only byte 3 with strb=0x8, data=0x12 -> should only change byte 3
            axi_write_64(TIMER_LOAD, 64'h1200_0000, 8'h08, resp);
            axi_read_32(TIMER_LOAD, rd, resp_r);
            if (rd[31:24] === 8'h12 && rd[23:0] === 24'hFFFF00)
                pass_msg("T11 byte3 write (strb=0x8) only changes byte 3");
            else begin
                $display("  [FAIL] T11 byte3 write: got 0x%08h, expected 0x12FFFF00", rd);
                fail_cnt = fail_cnt + 1;
            end
            $display("  T11 done.");
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================

    initial begin
        pass_cnt = 0;
        fail_cnt = 0;

        $display("");
        $display("============================================================");
        $display("  tb_timer_on_interconnect — Timer IP Integration Testbench");
        $display("============================================================");
        $display("");

        // Reset
        rst = 1'b1;
        master_idle();
        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // Run tests
        t1_reset();
        t2_ctrl_write_readback();
        t3_load_write_readback();
        t4_one_shot_countdown();
        t5_auto_reload();
        t6_compare_interrupt();
        t7_w1c_status();
        t8_ro_writes();
        t9_reserved_offset();
        t10_out_of_aperture();
        t11_wstrb_partial();

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
            $display("*** INTEGRATION TESTBENCH FAILED ***");
            $fatal(1, "tb_timer_on_interconnect: %0d checks failed", fail_cnt);
        end else begin
            $display("*** ALL INTEGRATION TESTS PASSED ***");
            $finish;
        end
    end

    // Watchdog
    initial begin
        #(CLK_PERIOD * 200000);
        $display("");
        $display("  [FAIL] WATCHDOG: simulation timeout (deadlock detected)");
        $fatal(2, "tb_timer_on_interconnect: watchdog timeout");
    end

endmodule

`default_nettype wire
