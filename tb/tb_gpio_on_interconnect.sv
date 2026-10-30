// =============================================================================
// File    : tb_gpio_on_interconnect.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
//
// Purpose : Integration testbench — verifies the axi_gpio hanging off port
//           m06 of the 3x8 AXI interconnect.
//
// Topology
// --------
//   TB AXI master (s00)
//      |
//   axi_interconnect_wrap_3x8
//      |-- m00..m03, m04, m05, m07  → dummy_slave_intg instances
//      |-- m06                      → axi_gpio
//                                    gpio_pads_i ← testbench stimulus
//                                    gpio_pads_o → observable
//                                    gpio_pads_oen → observable
//
// Test plan
// ---------
//   T1  Reset defaults        : OUT=0, DIR=0, IN=0 after reset
//   T2  OUT write/readback    : write GPIO_OUT, read back same value
//   T3  DIR write/readback    : write GPIO_DIR, read back, check gpio_pads_oen
//   T4  SET action register   : SET bits 0,3,7 then read OUT
//   T5  CLR action register   : CLR bits 0,7 then read OUT
//   T6  TOGGLE action register: TOGGLE bit 3 then read OUT
//   T7  GPIO_ID / VERSION     : read fixed ID and version registers
//   T8  RO register write     : write GPIO_IN (0x00) → SLVERR
//   T9  Reserved offset       : read 0x20 → SLRERR
//   T10 Out-of-aperture       : read 0x1_0000_0000 → DECERR
//   T11 GPIO_WIDTH masking    : write 0xFFFF_FFFF, read back 0x0000_FFFF
//   T12 WSTRB partial write   : byte strobe control
//   T13 Physical output       : set DIR+OUT, check gpio_pads_o follows
//
// Compile (xvlog):
//   xvlog -sv -i rtl/uart/include \
//       rtl/gpio/axi_gpio.sv \
//       rtl/interconnect/axi_interconnect.v \
//       rtl/interconnect/arbiter.v \
//       rtl/interconnect/priority_encoder.v \
//       rtl/interconnect/axi_interconnect_wrap_3x8.v \
//       rtl/axi_mem.sv \
//       rtl/axi_dummy_slave.sv \
//       rtl/simple_cpu_master.sv \
//       rtl/soc_top.sv \
//       tb/tb_gpio_on_interconnect.sv
//
// Simulate:
//   xsim tb_gpio_on_interconnect -runall
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

// ---------------------------------------------------------------------------
// Minimal dummy slave – accepts every single-beat write/read, returns a
// deterministic data pattern.  Used for m00..m03, m04, m05, m07.
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
            // Write address
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

            // Read address
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
module tb_gpio_on_interconnect;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam integer DW   = 64;
    localparam integer AW   = 32;
    localparam integer IDW  = 8;
    localparam integer SW   = DW/8;   // 8

    localparam integer CLK_PERIOD = 10;   // 10 ns  →  100 MHz

    // GPIO base address on interconnect  (m06 → 0x6000_0000)
    localparam [AW-1:0] GPIO_BASE = 32'h6000_0000;

    // GPIO register offsets (from GPIO_BASE)
    localparam [AW-1:0] GPIO_IN       = GPIO_BASE + 32'h000;
    localparam [AW-1:0] GPIO_OUT      = GPIO_BASE + 32'h004;
    localparam [AW-1:0] GPIO_DIR      = GPIO_BASE + 32'h008;
    localparam [AW-1:0] GPIO_OUT_SET  = GPIO_BASE + 32'h00C;
    localparam [AW-1:0] GPIO_OUT_CLR  = GPIO_BASE + 32'h010;
    localparam [AW-1:0] GPIO_OUT_TOGG = GPIO_BASE + 32'h014;
    localparam [AW-1:0] GPIO_ID       = GPIO_BASE + 32'h018;
    localparam [AW-1:0] GPIO_VERSION  = GPIO_BASE + 32'h01C;

    // Fixed constants from axi_gpio
    localparam [31:0] GPIO_ID_VALUE      = 32'h4749504F;
    localparam [31:0] GPIO_VERSION_VALUE = 32'h00010000;

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
    // Inter-module wires: interconnect m06 <-> axi_gpio
    // =========================================================================
    wire [IDW-1:0]  ic_gpio_awid;
    wire [AW-1:0]   ic_gpio_awaddr;
    wire [7:0]      ic_gpio_awlen;
    wire [2:0]      ic_gpio_awsize;
    wire [1:0]      ic_gpio_awburst;
    wire            ic_gpio_awlock;
    wire [3:0]      ic_gpio_awcache;
    wire [2:0]      ic_gpio_awprot;
    wire [3:0]      ic_gpio_awqos;
    wire [3:0]      ic_gpio_awregion;
    wire            ic_gpio_awvalid;
    wire            ic_gpio_awready;

    wire [DW-1:0]   ic_gpio_wdata;
    wire [SW-1:0]   ic_gpio_wstrb;
    wire            ic_gpio_wlast;
    wire            ic_gpio_wvalid;
    wire            ic_gpio_wready;

    wire [IDW-1:0]  ic_gpio_bid;
    wire [1:0]      ic_gpio_bresp;
    wire            ic_gpio_bvalid;
    wire            ic_gpio_bready;

    wire [IDW-1:0]  ic_gpio_arid;
    wire [AW-1:0]   ic_gpio_araddr;
    wire [7:0]      ic_gpio_arlen;
    wire [2:0]      ic_gpio_arsize;
    wire [1:0]      ic_gpio_arburst;
    wire            ic_gpio_arlock;
    wire [3:0]      ic_gpio_arcache;
    wire [2:0]      ic_gpio_arprot;
    wire [3:0]      ic_gpio_arqos;
    wire [3:0]      ic_gpio_arregion;
    wire            ic_gpio_arvalid;
    wire            ic_gpio_arready;

    wire [IDW-1:0]  ic_gpio_rid;
    wire [DW-1:0]   ic_gpio_rdata;
    wire [1:0]      ic_gpio_rresp;
    wire            ic_gpio_rlast;
    wire            ic_gpio_rvalid;
    wire            ic_gpio_rready;

    // =========================================================================
    // GPIO physical signals
    // =========================================================================
    reg  [15:0]     gpio_pads_i;
    wire [15:0]     gpio_pads_o;
    wire [15:0]     gpio_pads_oen;

    // =========================================================================
    // Dummy slave interconnect wires  (m00..m03, m04, m05, m07) = 7 dummies
    // Slave index mapping:
    //   0 = m00 (IMEM)   1 = m01 (DMEM)   2 = m02 (DMA)
    //   3 = m03 (CNN)   4 = m04 (UART)   5 = m05 (TIMER)
    //   6 = m07 (7-SEG)
    // (m06 = GPIO, handled separately above)
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
    // Generate 7 dummy_slave_intg instances  (m00..m03, m04, m05, m07)
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
        // GPIO slave  m06 = 0x6000_0000 .. 0x6FFF_FFFF
        .M06_BASE_ADDR       (32'h6000_0000),
        .M06_ADDR_WIDTH      ({1{32'd28}}),
        .M06_CONNECT_READ    (3'b001),
        .M06_CONNECT_WRITE   (3'b001),
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
        // TIMER slave m05 = 0x5000_0000
        .M05_BASE_ADDR       (32'h5000_0000),
        .M05_ADDR_WIDTH      ({1{32'd28}}),
        .M05_CONNECT_READ    (3'b111),
        .M05_CONNECT_WRITE   (3'b111),
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
        .m05_axi_awid    (ds_awid[5]),  .m05_axi_awaddr  (ds_awaddr[5]),
        .m05_axi_awlen   (ds_awlen[5]), .m05_axi_awsize  (ds_awsize[5]),
        .m05_axi_awburst (ds_awburst[5]),.m05_axi_awlock  (ds_awlock[5]),
        .m05_axi_awcache (ds_awcache[5]),.m05_axi_awprot  (ds_awprot[5]),
        .m05_axi_awqos   (ds_awqos[5]), .m05_axi_awregion(ds_awregion[5]),
        .m05_axi_awvalid (ds_awvalid[5]),.m05_axi_awready (ds_awready[5]),
        .m05_axi_wdata   (ds_wdata[5]), .m05_axi_wstrb   (ds_wstrb[5]),
        .m05_axi_wlast   (ds_wlast[5]), .m05_axi_wvalid  (ds_wvalid[5]),
        .m05_axi_wready  (ds_wready[5]),
        .m05_axi_bid     (ds_bid[5]),   .m05_axi_bresp   (ds_bresp[5]),
        .m05_axi_bvalid  (ds_bvalid[5]), .m05_axi_bready  (ds_bready[5]),
        .m05_axi_arid    (ds_arid[5]),  .m05_axi_araddr  (ds_araddr[5]),
        .m05_axi_arlen   (ds_arlen[5]), .m05_axi_arsize  (ds_arsize[5]),
        .m05_axi_arburst (ds_arburst[5]),.m05_axi_arlock  (ds_arlock[5]),
        .m05_axi_arcache (ds_arcache[5]),.m05_axi_arprot  (ds_arprot[5]),
        .m05_axi_arqos   (ds_arqos[5]), .m05_axi_arregion(ds_arregion[5]),
        .m05_axi_arvalid (ds_arvalid[5]),.m05_axi_arready (ds_arready[5]),
        .m05_axi_rid     (ds_rid[5]),   .m05_axi_rdata   (ds_rdata[5]),
        .m05_axi_rresp   (ds_rresp[5]), .m05_axi_rlast   (ds_rlast[5]),
        .m05_axi_rvalid  (ds_rvalid[5]), .m05_axi_rready  (ds_rready[5]),

        // ---- Slave port 6 : m06 GPIO ----
        .m06_axi_awid    (ic_gpio_awid),    .m06_axi_awaddr  (ic_gpio_awaddr),
        .m06_axi_awlen   (ic_gpio_awlen),   .m06_axi_awsize  (ic_gpio_awsize),
        .m06_axi_awburst (ic_gpio_awburst), .m06_axi_awlock  (ic_gpio_awlock),
        .m06_axi_awcache (ic_gpio_awcache), .m06_axi_awprot  (ic_gpio_awprot),
        .m06_axi_awqos   (ic_gpio_awqos),   .m06_axi_awregion(ic_gpio_awregion),
        .m06_axi_awvalid (ic_gpio_awvalid), .m06_axi_awready (ic_gpio_awready),
        .m06_axi_wdata   (ic_gpio_wdata),   .m06_axi_wstrb   (ic_gpio_wstrb),
        .m06_axi_wlast   (ic_gpio_wlast),   .m06_axi_wvalid  (ic_gpio_wvalid),
        .m06_axi_wready  (ic_gpio_wready),
        .m06_axi_bid     (ic_gpio_bid),     .m06_axi_bresp   (ic_gpio_bresp),
        .m06_axi_bvalid  (ic_gpio_bvalid),  .m06_axi_bready  (ic_gpio_bready),
        .m06_axi_arid    (ic_gpio_arid),    .m06_axi_araddr  (ic_gpio_araddr),
        .m06_axi_arlen   (ic_gpio_arlen),   .m06_axi_arsize  (ic_gpio_arsize),
        .m06_axi_arburst (ic_gpio_arburst), .m06_axi_arlock  (ic_gpio_arlock),
        .m06_axi_arcache (ic_gpio_arcache), .m06_axi_arprot  (ic_gpio_arprot),
        .m06_axi_arqos   (ic_gpio_arqos),   .m06_axi_arregion(ic_gpio_arregion),
        .m06_axi_arvalid (ic_gpio_arvalid), .m06_axi_arready (ic_gpio_arready),
        .m06_axi_rid     (ic_gpio_rid),     .m06_axi_rdata   (ic_gpio_rdata),
        .m06_axi_rresp   (ic_gpio_rresp),   .m06_axi_rlast   (ic_gpio_rlast),
        .m06_axi_rvalid  (ic_gpio_rvalid),  .m06_axi_rready  (ic_gpio_rready),

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
    // axi_gpio DUT
    // =========================================================================
    axi_gpio #(
        .GPIO_WIDTH         (16),
        .AXI_DATA_WIDTH     (DW),
        .AXI_ADDR_WIDTH     (AW),
        .AXI_ID_WIDTH       (IDW),
        .AXI_STRB_WIDTH     (SW),
        .BASE_ADDR          (GPIO_BASE)
    ) u_gpio (
        .clk                (clk),
        .rst                (rst),
        .s_axi_awid         (ic_gpio_awid),
        .s_axi_awaddr       (ic_gpio_awaddr),
        .s_axi_awlen        (ic_gpio_awlen),
        .s_axi_awsize       (ic_gpio_awsize),
        .s_axi_awburst      (ic_gpio_awburst),
        .s_axi_awlock       (ic_gpio_awlock),
        .s_axi_awcache      (ic_gpio_awcache),
        .s_axi_awprot       (ic_gpio_awprot),
        .s_axi_awqos        (ic_gpio_awqos),
        .s_axi_awregion     (ic_gpio_awregion),
        .s_axi_awvalid      (ic_gpio_awvalid),
        .s_axi_awready      (ic_gpio_awready),
        .s_axi_wdata        (ic_gpio_wdata),
        .s_axi_wstrb        (ic_gpio_wstrb),
        .s_axi_wlast        (ic_gpio_wlast),
        .s_axi_wvalid       (ic_gpio_wvalid),
        .s_axi_wready       (ic_gpio_wready),
        .s_axi_bid          (ic_gpio_bid),
        .s_axi_bresp        (ic_gpio_bresp),
        .s_axi_bvalid       (ic_gpio_bvalid),
        .s_axi_bready       (ic_gpio_bready),
        .s_axi_arid         (ic_gpio_arid),
        .s_axi_araddr       (ic_gpio_araddr),
        .s_axi_arlen        (ic_gpio_arlen),
        .s_axi_arsize       (ic_gpio_arsize),
        .s_axi_arburst      (ic_gpio_arburst),
        .s_axi_arlock       (ic_gpio_arlock),
        .s_axi_arcache      (ic_gpio_arcache),
        .s_axi_arprot       (ic_gpio_arprot),
        .s_axi_arqos        (ic_gpio_arqos),
        .s_axi_arregion     (ic_gpio_arregion),
        .s_axi_arvalid      (ic_gpio_arvalid),
        .s_axi_arready      (ic_gpio_arready),
        .s_axi_rid          (ic_gpio_rid),
        .s_axi_rdata        (ic_gpio_rdata),
        .s_axi_rresp        (ic_gpio_rresp),
        .s_axi_rlast        (ic_gpio_rlast),
        .s_axi_rvalid       (ic_gpio_rvalid),
        .s_axi_rready       (ic_gpio_rready),
        .gpio_i             (gpio_pads_i),
        .gpio_o             (gpio_pads_o),
        .gpio_oen           (gpio_pads_oen)
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

        begin
            @(posedge clk); #1;

            m_awid     <= 8'h01;
            m_awaddr   <= addr;
            m_awlen    <= 8'h00;          // single beat
            m_awsize   <= 3'b011;         // 8 bytes
            m_awburst  <= 2'b01;          // INCR
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

            // Hold AW+W until both accepted
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

            // Wait for write response
            wait (m_bvalid);
            @(posedge clk); #1;
            m_bready <= 1'b0;

            repeat (2) @(posedge clk);
        end
    endtask

    // Convenience: write only lower 32 bits
    task axi_write_32;
        input [AW-1:0]  addr;
        input [31:0]    data;
        begin
            axi_write_64(addr, {32'h0, data}, 8'h0F);
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

    // =========================================================================
    // Deassert all master signals to safe idle values
    // =========================================================================
    task master_idle;
        begin
            m_awvalid <= 1'b0; m_wvalid <= 1'b0; m_bready <= 1'b0;
            m_arvalid <= 1'b0; m_rready <= 1'b0;
        end
    endtask

    // =========================================================================
    // T E S T   T A S K S
    // =========================================================================

    // -------------------------------------------------------------------------
    // T1 : Reset defaults
    // -------------------------------------------------------------------------
    task t1_reset;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T1: Reset defaults ==");

            rst = 1'b1;
            repeat (10) @(posedge clk);
            rst = 1'b0;
            repeat (5) @(posedge clk);

            axi_read_32(GPIO_OUT,  rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("GPIO_OUT = 0 after reset");
            else begin
                $display("  [FAIL] GPIO_OUT after reset: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(GPIO_DIR,  rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("GPIO_DIR = 0 after reset");
            else begin
                $display("  [FAIL] GPIO_DIR after reset: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(GPIO_IN,   rd, resp);
            if (rd === 32'h0 && resp === 2'b00)
                pass_msg("GPIO_IN = 0 after reset (gpio_pads_i=0)");
            else begin
                $display("  [FAIL] GPIO_IN after reset: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            if (gpio_pads_oen === 16'h0)
                pass_msg("gpio_pads_oen = 0 after reset");
            else begin
                $display("  [FAIL] gpio_pads_oen after reset: got 0x%04h", gpio_pads_oen);
                fail_cnt = fail_cnt + 1;
            end

            if (gpio_pads_o === 16'h0)
                pass_msg("gpio_pads_o = 0 after reset");
            else begin
                $display("  [FAIL] gpio_pads_o after reset: got 0x%04h", gpio_pads_o);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T2 : OUT write/readback
    // -------------------------------------------------------------------------
    task t2_out_write_readback;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T2: GPIO_OUT write/readback ==");

            axi_write_32(GPIO_OUT, 32'h0000_1234);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_1234 && resp === 2'b00)
                pass_msg("GPIO_OUT readback = 0x1234");
            else begin
                $display("  [FAIL] GPIO_OUT readback: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(GPIO_OUT, 32'h0000_F0F0);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_F0F0 && resp === 2'b00)
                pass_msg("GPIO_OUT readback = 0xF0F0");
            else begin
                $display("  [FAIL] GPIO_OUT readback: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T3 : DIR write/readback + physical oen
    // -------------------------------------------------------------------------
    task t3_dir_write_readback;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T3: GPIO_DIR write/readback ==");

            axi_write_32(GPIO_DIR, 32'h0000_ABCD);
            axi_read_32(GPIO_DIR, rd, resp);
            if (rd === 32'h0000_ABCD && resp === 2'b00)
                pass_msg("GPIO_DIR readback = 0xABCD");
            else begin
                $display("  [FAIL] GPIO_DIR readback: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            if (gpio_pads_oen === 16'hABCD)
                pass_msg("gpio_pads_oen matches DIR = 0xABCD");
            else begin
                $display("  [FAIL] gpio_pads_oen = 0x%04h, expected 0xABCD", gpio_pads_oen);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(GPIO_DIR, 32'h0000_0000);
            axi_read_32(GPIO_DIR, rd, resp);
            if (rd === 32'h0000_0000 && resp === 2'b00)
                pass_msg("GPIO_DIR cleared to 0");
            else begin
                $display("  [FAIL] GPIO_DIR clear: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            if (gpio_pads_oen === 16'h0000)
                pass_msg("gpio_pads_oen = 0 after DIR clear");
            else begin
                $display("  [FAIL] gpio_pads_oen = 0x%04h after DIR clear", gpio_pads_oen);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T4 : SET action register
    // -------------------------------------------------------------------------
    task t4_set_action;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T4: SET action register ==");

            axi_write_32(GPIO_DIR, 32'h0000_FFFF);   // all outputs
            axi_write_32(GPIO_OUT, 32'h0000_0000);   // start from 0

            // SET bits 0, 3, 7 → 0b10001001 = 0x89
            axi_write_32(GPIO_OUT_SET, 32'h0000_0089);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_0089 && resp === 2'b00)
                pass_msg("GPIO_OUT after SET 0x89 = 0x0089");
            else begin
                $display("  [FAIL] GPIO_OUT after SET: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(GPIO_OUT_SET, rd, resp);
            if (rd === 32'h0000_0000)
                pass_msg("GPIO_OUT_SET reads as 0 (write-only)");
            else begin
                $display("  [FAIL] GPIO_OUT_SET read: got 0x%08h, expected 0", rd);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T5 : CLR action register
    // -------------------------------------------------------------------------
    task t5_clr_action;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T5: CLR action register ==");

            axi_write_32(GPIO_DIR, 32'h0000_FFFF);
            axi_write_32(GPIO_OUT, 32'h0000_FFFF);   // all ones

            // CLR bits 0 and 7 → 0b10000001 = 0x81
            axi_write_32(GPIO_OUT_CLR, 32'h0000_0081);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_FF7E && resp === 2'b00)
                pass_msg("GPIO_OUT after CLR 0x81 = 0xFF7E");
            else begin
                $display("  [FAIL] GPIO_OUT after CLR: got 0x%08h, expected 0xFF7E", rd);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T6 : TOGGLE action register
    // -------------------------------------------------------------------------
    task t6_toggle_action;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T6: TOGGLE action register ==");

            axi_write_32(GPIO_DIR, 32'h0000_FFFF);
            axi_write_32(GPIO_OUT, 32'h0000_0008);   // bit 3 = 1

            // TOGGLE bit 3 → should clear it
            axi_write_32(GPIO_OUT_TOGG, 32'h0000_0008);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_0000 && resp === 2'b00)
                pass_msg("GPIO_OUT after TOGGLE bit3 = 0x0000");
            else begin
                $display("  [FAIL] GPIO_OUT after TOGGLE: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            // TOGGLE bit 3 again → should set it back
            axi_write_32(GPIO_OUT_TOGG, 32'h0000_0008);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_0008 && resp === 2'b00)
                pass_msg("GPIO_OUT after TOGGLE bit3 again = 0x0008");
            else begin
                $display("  [FAIL] GPIO_OUT after TOGGLE again: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T7 : GPIO_ID / GPIO_VERSION
    // -------------------------------------------------------------------------
    task t7_id_version;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T7: GPIO_ID / GPIO_VERSION ==");

            axi_read_32(GPIO_ID, rd, resp);
            if (rd === GPIO_ID_VALUE && resp === 2'b00)
                pass_msg("GPIO_ID = 0x4749504F");
            else begin
                $display("  [FAIL] GPIO_ID: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end

            axi_read_32(GPIO_VERSION, rd, resp);
            if (rd === GPIO_VERSION_VALUE && resp === 2'b00)
                pass_msg("GPIO_VERSION = 0x00010000");
            else begin
                $display("  [FAIL] GPIO_VERSION: got 0x%08h resp %b", rd, resp);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T8 : RO register write → SLVERR
    // -------------------------------------------------------------------------
    task t8_ro_write_slverr;
        reg [1:0] resp;
        begin
            $display("\n== T8: Read-only register writes → SLVERR ==");

            // Write to GPIO_IN (0x00) - RO register → SLVERR
            @(posedge clk); #1;
            m_awid     <= 8'h01;
            m_awaddr   <= GPIO_IN;
            m_awlen    <= 8'h00;
            m_awsize   <= 3'b011;
            m_awburst  <= 2'b01;
            m_awlock   <= 1'b0;
            m_awcache  <= 4'b0000;
            m_awprot   <= 3'b000;
            m_awqos    <= 4'b0000;
            m_awvalid  <= 1'b1;
            m_wdata    <= {32'h0, 32'hFFFF_FFFF};
            m_wstrb    <= 8'h0F;
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

            if (resp === 2'b10)
                pass_msg("Write GPIO_IN (RO) → SLVERR");
            else begin
                $display("  [FAIL] Write GPIO_IN (RO): got resp %b, expected SLVERR(10)", resp);
                fail_cnt = fail_cnt + 1;
            end

            repeat (2) @(posedge clk);
        end
    endtask

    // -------------------------------------------------------------------------
    // T9 : Reserved offset → SLVERR
    // -------------------------------------------------------------------------
    task t9_reserved_offset;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T9: Reserved offset → SLVERR ==");

            axi_read_32(GPIO_BASE + 32'h020, rd, resp);
            if (resp === 2'b10 && rd === 32'h0)
                pass_msg("Reserved offset 0x20 → SLVERR, data=0");
            else begin
                $display("  [FAIL] Reserved offset 0x20: got resp %b data 0x%08h", resp, rd);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T10 : Out-of-aperture → DECERR
    // -------------------------------------------------------------------------
    task t10_out_of_aperture;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T10: Out-of-aperture address → DECERR ==");

            axi_read_32(32'h6001_0000, rd, resp);
            if (resp === 2'b11)
                pass_msg("Out-of-aperture 0x6001_0000 → DECERR");
            else begin
                $display("  [FAIL] Out-of-aperture 0x6001_0000: got resp %b, expected DECERR(11)", resp);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T11 : GPIO_WIDTH masking
    // -------------------------------------------------------------------------
    task t11_width_masking;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T11: GPIO_WIDTH masking ==");

            axi_write_32(GPIO_OUT, 32'hFFFF_FFFF);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_FFFF && resp === 2'b00)
                pass_msg("GPIO_OUT all-ones masked to 0x0000_FFFF");
            else begin
                $display("  [FAIL] GPIO_OUT all-ones: got 0x%08h, expected 0x0000_FFFF", rd);
                fail_cnt = fail_cnt + 1;
            end

            axi_write_32(GPIO_DIR, 32'hFFFF_FFFF);
            axi_read_32(GPIO_DIR, rd, resp);
            if (rd === 32'h0000_FFFF && resp === 2'b00)
                pass_msg("GPIO_DIR all-ones masked to 0x0000_FFFF");
            else begin
                $display("  [FAIL] GPIO_DIR all-ones: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T12 : WSTRB partial write
    // -------------------------------------------------------------------------
    task t12_wstrb_partial;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T12: WSTRB partial write ==");

            axi_write_32(GPIO_OUT, 32'h0000_DEAD);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_DEAD)
                pass_msg("GPIO_OUT full write = 0xDEAD");
            else begin
                $display("  [FAIL] GPIO_OUT full write: got 0x%08h", rd);
                fail_cnt = fail_cnt + 1;
            end

            // Write only byte 0 (strb=0x1): 0xBE → DEBE
            axi_write_64(GPIO_OUT, 64'h0000_00BE, 8'h01);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_DEBE)
                pass_msg("GPIO_OUT byte0 write (strb=0x1) = 0xDEBE");
            else begin
                $display("  [FAIL] GPIO_OUT byte0 write: got 0x%08h, expected 0xDEBE", rd);
                fail_cnt = fail_cnt + 1;
            end

            // Write only byte 1 (strb=0x2): 0xAD → ADBE
            axi_write_64(GPIO_OUT, 64'h0000_AD00, 8'h02);
            axi_read_32(GPIO_OUT, rd, resp);
            if (rd === 32'h0000_ADBE)
                pass_msg("GPIO_OUT byte1 write (strb=0x2) = 0xADBE");
            else begin
                $display("  [FAIL] GPIO_OUT byte1 write: got 0x%08h, expected 0xADBE", rd);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // T13 : Physical output follows DIR & OUT
    // -------------------------------------------------------------------------
    task t13_physical_output;
        reg [31:0] rd;
        reg [1:0]  resp;
        begin
            $display("\n== T13: Physical output ==");

            axi_write_32(GPIO_DIR, 32'h0000_F00F);   // bits 0-3, 12-15 output
            axi_write_32(GPIO_OUT, 32'h0000_0F00);   // bits 8-11 high

            // gpio_o = DIR & OUT → bits 8-11 high only (but masked by DIR)
            // DIR bits 8-11 = 0, so gpio_o bits 8-11 = 0
            // gpio_o bits 0-3 = OUT bits 0-3 = 0
            // gpio_o bits 12-15 = OUT bits 12-15 = 0
            if (gpio_pads_o === 16'h0000)
                pass_msg("gpio_pads_o = 0x0000 (DIR masks OUT)");
            else begin
                $display("  [FAIL] gpio_pads_o = 0x%04h, expected 0x0000", gpio_pads_o);
                fail_cnt = fail_cnt + 1;
            end

            // Now set OUT bits that overlap DIR
            axi_write_32(GPIO_OUT, 32'h0000_F00F);
            if (gpio_pads_o === 16'hF00F)
                pass_msg("gpio_pads_o = 0xF00F follows DIR&OUT");
            else begin
                $display("  [FAIL] gpio_pads_o = 0x%04h, expected 0xF00F", gpio_pads_o);
                fail_cnt = fail_cnt + 1;
            end

            if (gpio_pads_oen === 16'hF00F)
                pass_msg("gpio_pads_oen = 0xF00F matches DIR");
            else begin
                $display("  [FAIL] gpio_pads_oen = 0x%04h, expected 0xF00F", gpio_pads_oen);
                fail_cnt = fail_cnt + 1;
            end
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
        $display("  tb_gpio_on_interconnect — GPIO IP Integration Testbench");
        $display("============================================================");
        $display("");

        // Drive gpio_pads_i low initially
        gpio_pads_i = 16'h0000;

        // Reset
        rst = 1'b1;
        master_idle();
        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // Run tests
        t1_reset();
        t2_out_write_readback();
        t3_dir_write_readback();
        t4_set_action();
        t5_clr_action();
        t6_toggle_action();
        t7_id_version();
        t8_ro_write_slverr();
        t9_reserved_offset();
        t10_out_of_aperture();
        t11_width_masking();
        t12_wstrb_partial();
        t13_physical_output();

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
            $fatal(1, "tb_gpio_on_interconnect: %0d checks failed", fail_cnt);
        end else begin
            $display("*** ALL INTEGRATION TESTS PASSED ***");
            $finish;
        end
    end

    // Watchdog — catch deadlocks
    initial begin
        #(CLK_PERIOD * 200000);
        $display("");
        $display("  [FAIL] WATCHDOG: simulation timeout (deadlock detected)");
        $fatal(2, "tb_gpio_on_interconnect: watchdog timeout");
    end

endmodule

`default_nettype wire
