// ============================================================================
// File    : soc_top.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Top-level SoC for CPU-to-UART read/write testing
//
// Topology:
//   simple_cpu_master  -> interconnect s00
//   interconnect m00   -> IMEM  (0x8000_0000)
//   interconnect m01   -> DMEM  (0x1000_0000)
//   interconnect m02   -> DMA   (0x2000_0000) [dummy]
//   interconnect m03   -> CNN   (0x3000_0000) [dummy]
//   interconnect m04   -> UART  (0x4000_0000) via axi_uart_bridge
//   interconnect m05   -> TIMER (0x5000_0000) [dummy]
//   interconnect m06   -> GPIO  (0x6000_0000) [dummy]
//   interconnect m07   -> 7-SEG (0x7000_0000) [dummy]
//
// UART tx is looped back to rx internally for self-contained simulation.
// ============================================================================

`timescale 1ns/1ps

module soc_top #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter ID_WIDTH   = 8,
    parameter TEST_MODE  = "UART"
)(
    input  wire                    clk,
    input  wire                    rst,          // active-high system reset

    // UART external
    output wire                    uart_tx_o,
    input  wire                    uart_rx_i,
    output wire                    uart_irq_o,

    // GPIO external
    input  wire [15:0]             gpio_pads_i,
    output wire [15:0]             gpio_pads_o,
    output wire [15:0]             gpio_pads_oen,

    // Timer external
    output wire                    timer_irq_o,

    // CNN external
    output wire                    cnn_irq_o,

    // 7-Segment display external (Basys3)
    output wire [6:0]              seven_seg_seg_o,
    output wire                    seven_seg_dp_o,
    output wire [3:0]              seven_seg_an_o,

    // CPU status external
    output wire                    cpu_done_o,
    output wire                    cpu_pass_o
);

    // ------------------------------------------------------------------
    // Simple CPU master AXI4 interface (connects to interconnect s00)
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
    // Interconnect master-side signals (m00..m07)
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     m_axi_awid   [0:7];
    wire [ADDR_WIDTH-1:0]   m_axi_awaddr [0:7];
    wire [7:0]              m_axi_awlen  [0:7];
    wire [2:0]              m_axi_awsize [0:7];
    wire [1:0]              m_axi_awburst[0:7];
    wire                    m_axi_awlock [0:7];
    wire [3:0]              m_axi_awcache[0:7];
    wire [2:0]              m_axi_awprot [0:7];
    wire [3:0]              m_axi_awqos  [0:7];
    wire [3:0]              m_axi_awregion[0:7];
    wire                    m_axi_awvalid[0:7];
    wire                    m_axi_awready[0:7];

    wire [DATA_WIDTH-1:0]   m_axi_wdata  [0:7];
    wire [STRB_WIDTH-1:0]   m_axi_wstrb  [0:7];
    wire                    m_axi_wlast  [0:7];
    wire                    m_axi_wvalid [0:7];
    wire                    m_axi_wready [0:7];

    wire [ID_WIDTH-1:0]     m_axi_bid    [0:7];
    wire [1:0]              m_axi_bresp  [0:7];
    wire                    m_axi_bvalid [0:7];
    wire                    m_axi_bready [0:7];

    wire [ID_WIDTH-1:0]     m_axi_arid   [0:7];
    wire [ADDR_WIDTH-1:0]   m_axi_araddr [0:7];
    wire [7:0]              m_axi_arlen  [0:7];
    wire [2:0]              m_axi_arsize [0:7];
    wire [1:0]              m_axi_arburst[0:7];
    wire                    m_axi_arlock [0:7];
    wire [3:0]              m_axi_arcache[0:7];
    wire [2:0]              m_axi_arprot [0:7];
    wire [3:0]              m_axi_arqos  [0:7];
    wire [3:0]              m_axi_arregion[0:7];
    wire                    m_axi_arvalid[0:7];
    wire                    m_axi_arready[0:7];

    wire [ID_WIDTH-1:0]     m_axi_rid    [0:7];
    wire [DATA_WIDTH-1:0]   m_axi_rdata  [0:7];
    wire [1:0]              m_axi_rresp  [0:7];
    wire                    m_axi_rlast  [0:7];
    wire                    m_axi_rvalid [0:7];
    wire                    m_axi_rready [0:7];

    // ------------------------------------------------------------------
    // Interconnect slave-side signals (s00)
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     s_axi_awid   [0:0];
    wire [ADDR_WIDTH-1:0]   s_axi_awaddr [0:0];
    wire [7:0]              s_axi_awlen  [0:0];
    wire [2:0]              s_axi_awsize [0:0];
    wire [1:0]              s_axi_awburst[0:0];
    wire                    s_axi_awlock [0:0];
    wire [3:0]              s_axi_awcache[0:0];
    wire [2:0]              s_axi_awprot [0:0];
    wire [3:0]              s_axi_awqos  [0:0];
    wire                    s_axi_awvalid[0:0];
    wire                    s_axi_awready[0:0];

    wire [DATA_WIDTH-1:0]   s_axi_wdata  [0:0];
    wire [STRB_WIDTH-1:0]   s_axi_wstrb  [0:0];
    wire                    s_axi_wlast  [0:0];
    wire                    s_axi_wvalid [0:0];
    wire                    s_axi_wready [0:0];

    wire [ID_WIDTH-1:0]     s_axi_bid    [0:0];
    wire [1:0]              s_axi_bresp  [0:0];
    wire                    s_axi_bvalid [0:0];
    wire                    s_axi_bready [0:0];

    wire [ID_WIDTH-1:0]     s_axi_arid   [0:0];
    wire [ADDR_WIDTH-1:0]   s_axi_araddr [0:0];
    wire [7:0]              s_axi_arlen  [0:0];
    wire [2:0]              s_axi_arsize [0:0];
    wire [1:0]              s_axi_arburst[0:0];
    wire                    s_axi_arlock [0:0];
    wire [3:0]              s_axi_arcache[0:0];
    wire [2:0]              s_axi_arprot [0:0];
    wire [3:0]              s_axi_arqos  [0:0];
    wire                    s_axi_arvalid[0:0];
    wire                    s_axi_arready[0:0];

    wire [ID_WIDTH-1:0]     s_axi_rid    [0:0];
    wire [DATA_WIDTH-1:0]   s_axi_rdata  [0:0];
    wire [1:0]              s_axi_rresp  [0:0];
    wire                    s_axi_rlast  [0:0];
    wire                    s_axi_rvalid [0:0];
    wire                    s_axi_rready [0:0];

    // ------------------------------------------------------------------
    // Connect s00 to simple_cpu_master
    // ------------------------------------------------------------------
    assign s_axi_awid[0]    = cpu_awid;
    assign s_axi_awaddr[0]  = cpu_awaddr;
    assign s_axi_awlen[0]   = cpu_awlen;
    assign s_axi_awsize[0]  = cpu_awsize;
    assign s_axi_awburst[0] = cpu_awburst;
    assign s_axi_awlock[0]  = cpu_awlock;
    assign s_axi_awcache[0] = cpu_awcache;
    assign s_axi_awprot[0]  = cpu_awprot;
    assign s_axi_awqos[0]   = cpu_awqos;
    assign s_axi_awvalid[0] = cpu_awvalid;
    assign cpu_awready      = s_axi_awready[0];

    assign s_axi_wdata[0]   = cpu_wdata;
    assign s_axi_wstrb[0]   = cpu_wstrb;
    assign s_axi_wlast[0]   = cpu_wlast;
    assign s_axi_wvalid[0]  = cpu_wvalid;
    assign cpu_wready       = s_axi_wready[0];

    assign cpu_bvalid       = s_axi_bvalid[0];
    assign cpu_bresp        = s_axi_bresp[0];
    assign cpu_bid          = s_axi_bid[0];
    assign s_axi_bready[0]  = cpu_bready;

    assign s_axi_arid[0]    = cpu_arid;
    assign s_axi_araddr[0]  = cpu_araddr;
    assign s_axi_arlen[0]   = cpu_arlen;
    assign s_axi_arsize[0]  = cpu_arsize;
    assign s_axi_arburst[0] = cpu_arburst;
    assign s_axi_arlock[0]  = cpu_arlock;
    assign s_axi_arcache[0] = cpu_arcache;
    assign s_axi_arprot[0]  = cpu_arprot;
    assign s_axi_arqos[0]   = cpu_arqos;
    assign s_axi_arvalid[0] = cpu_arvalid;
    assign cpu_arready      = s_axi_arready[0];

    assign cpu_rvalid       = s_axi_rvalid[0];
    assign cpu_rdata        = s_axi_rdata[0];
    assign cpu_rresp        = s_axi_rresp[0];
    assign cpu_rid          = s_axi_rid[0];
    assign cpu_rlast        = s_axi_rlast[0];
    assign s_axi_rready[0]  = cpu_rready;

    // ------------------------------------------------------------------
    // Instantiate Simple CPU Master
    // ------------------------------------------------------------------
    simple_cpu_master #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .STRB_WIDTH(STRB_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .TEST_MODE (TEST_MODE)
    ) cpu_inst (
        .clk            (clk),
        .rst            (rst),

        // AXI4 master — drives the cpu_* wires which feed interconnect s00
        .m_axi_awid     (cpu_awid),         .m_axi_awready (cpu_awready),
        .m_axi_awaddr   (cpu_awaddr),       .m_axi_awvalid (cpu_awvalid),
        .m_axi_awlen    (cpu_awlen),        .m_axi_awsize  (cpu_awsize),
        .m_axi_awburst  (cpu_awburst),      .m_axi_awlock  (cpu_awlock),
        .m_axi_awcache  (cpu_awcache),      .m_axi_awprot  (cpu_awprot),
        .m_axi_awqos    (cpu_awqos),

        .m_axi_wdata    (cpu_wdata),        .m_axi_wready  (cpu_wready),
        .m_axi_wstrb    (cpu_wstrb),        .m_axi_wvalid  (cpu_wvalid),
        .m_axi_wlast    (cpu_wlast),

        .m_axi_bid      (cpu_bid),          .m_axi_bresp   (cpu_bresp),
        .m_axi_bvalid   (cpu_bvalid),       .m_axi_bready  (cpu_bready),

        .m_axi_arid     (cpu_arid),         .m_axi_arready (cpu_arready),
        .m_axi_araddr   (cpu_araddr),       .m_axi_arvalid (cpu_arvalid),
        .m_axi_arlen    (cpu_arlen),        .m_axi_arsize  (cpu_arsize),
        .m_axi_arburst  (cpu_arburst),      .m_axi_arlock  (cpu_arlock),
        .m_axi_arcache  (cpu_arcache),      .m_axi_arprot  (cpu_arprot),
        .m_axi_arqos    (cpu_arqos),

        .m_axi_rid      (cpu_rid),          .m_axi_rdata   (cpu_rdata),
        .m_axi_rresp    (cpu_rresp),        .m_axi_rlast   (cpu_rlast),
        .m_axi_rvalid   (cpu_rvalid),       .m_axi_rready  (cpu_rready),

        .cpu_done       (cpu_done),
        .cpu_pass       (cpu_pass)
    );

    assign cpu_done_o = cpu_done;
    assign cpu_pass_o = cpu_pass;

    // ------------------------------------------------------------------
    // Instantiate AXI Interconnect (1 master x 8 slaves)
    // ------------------------------------------------------------------
    axi_interconnect #(
        .S_COUNT         (1),
        .M_COUNT         (8),
        .DATA_WIDTH      (DATA_WIDTH),
        .ADDR_WIDTH      (ADDR_WIDTH),
        .STRB_WIDTH      (STRB_WIDTH),
        .ID_WIDTH        (ID_WIDTH),
        .FORWARD_ID      (0),
        .M_REGIONS       (1),
        // M_BASE_ADDR: LSB = port 0, MSB = port 7 (Verilog concatenation)
        // Must match soc_top port connections: port0=IMEM, port1=DMEM, ..., port4=UART, ...
        .M_BASE_ADDR     ({
            32'h7000_0000,  // m07 7-SEG  (IC port 7)
            32'h6000_0000,  // m06 GPIO   (IC port 6)
            32'h5000_0000,  // m05 TIMER  (IC port 5)
            32'h4000_0000,  // m04 UART   (IC port 4)
            32'h3000_0000,  // m03 CNN    (IC port 3)
            32'h2000_0000,  // m02 DMA    (IC port 2)
            32'h1000_0000,  // m01 DMEM   (IC port 1)
            32'h8000_0000   // m00 IMEM   (IC port 0)
        }),
        .M_ADDR_WIDTH    ({
            {32{32'd28}}, {32{32'd28}}, {32{32'd28}}, {32{32'd28}},
            {32{32'd28}}, {32{32'd28}}, {32{32'd28}}, {32{32'd28}}
        }),
        // S_COUNT=1, so each field is 1 bit: all 8 ports accessible from slave 0
        .M_CONNECT_READ  ({8{1'b1}}),
        .M_CONNECT_WRITE ({8{1'b1}})
    ) ic_inst (
        .clk (clk),
        .rst (rst),

        // Slave interface (s00) - from CPU master
        .s_axi_awid   ({s_axi_awid[0]}),
        .s_axi_awaddr ({s_axi_awaddr[0]}),
        .s_axi_awlen  ({s_axi_awlen[0]}),
        .s_axi_awsize ({s_axi_awsize[0]}),
        .s_axi_awburst({s_axi_awburst[0]}),
        .s_axi_awlock ({s_axi_awlock[0]}),
        .s_axi_awcache({s_axi_awcache[0]}),
        .s_axi_awprot ({s_axi_awprot[0]}),
        .s_axi_awqos  ({s_axi_awqos[0]}),
        .s_axi_awvalid({s_axi_awvalid[0]}),
        .s_axi_awready({s_axi_awready[0]}),
        .s_axi_wdata  ({s_axi_wdata[0]}),
        .s_axi_wstrb  ({s_axi_wstrb[0]}),
        .s_axi_wlast  ({s_axi_wlast[0]}),
        .s_axi_wvalid ({s_axi_wvalid[0]}),
        .s_axi_wready ({s_axi_wready[0]}),
        .s_axi_bid    ({s_axi_bid[0]}),
        .s_axi_bresp  ({s_axi_bresp[0]}),
        .s_axi_bvalid ({s_axi_bvalid[0]}),
        .s_axi_bready ({s_axi_bready[0]}),
        .s_axi_arid   ({s_axi_arid[0]}),
        .s_axi_araddr ({s_axi_araddr[0]}),
        .s_axi_arlen  ({s_axi_arlen[0]}),
        .s_axi_arsize ({s_axi_arsize[0]}),
        .s_axi_arburst({s_axi_arburst[0]}),
        .s_axi_arlock ({s_axi_arlock[0]}),
        .s_axi_arcache({s_axi_arcache[0]}),
        .s_axi_arprot ({s_axi_arprot[0]}),
        .s_axi_arqos  ({s_axi_arqos[0]}),
        .s_axi_arvalid({s_axi_arvalid[0]}),
        .s_axi_arready({s_axi_arready[0]}),
        .s_axi_rid    ({s_axi_rid[0]}),
        .s_axi_rdata  ({s_axi_rdata[0]}),
        .s_axi_rresp  ({s_axi_rresp[0]}),
        .s_axi_rlast  ({s_axi_rlast[0]}),
        .s_axi_rvalid ({s_axi_rvalid[0]}),
        .s_axi_rready ({s_axi_rready[0]}),

        // Master interfaces (m00..m07) - to slaves
        .m_axi_awid   ({m_axi_awid[7],   m_axi_awid[6],   m_axi_awid[5],   m_axi_awid[4],
                        m_axi_awid[3],   m_axi_awid[2],   m_axi_awid[1],   m_axi_awid[0]}),
        .m_axi_awaddr ({m_axi_awaddr[7], m_axi_awaddr[6], m_axi_awaddr[5], m_axi_awaddr[4],
                        m_axi_awaddr[3], m_axi_awaddr[2], m_axi_awaddr[1], m_axi_awaddr[0]}),
        .m_axi_awlen  ({m_axi_awlen[7],  m_axi_awlen[6],  m_axi_awlen[5],  m_axi_awlen[4],
                        m_axi_awlen[3],  m_axi_awlen[2],  m_axi_awlen[1],  m_axi_awlen[0]}),
        .m_axi_awsize ({m_axi_awsize[7], m_axi_awsize[6], m_axi_awsize[5], m_axi_awsize[4],
                        m_axi_awsize[3], m_axi_awsize[2], m_axi_awsize[1], m_axi_awsize[0]}),
        .m_axi_awburst({m_axi_awburst[7],m_axi_awburst[6],m_axi_awburst[5],m_axi_awburst[4],
                        m_axi_awburst[3],m_axi_awburst[2],m_axi_awburst[1],m_axi_awburst[0]}),
        .m_axi_awlock ({m_axi_awlock[7], m_axi_awlock[6], m_axi_awlock[5], m_axi_awlock[4],
                        m_axi_awlock[3], m_axi_awlock[2], m_axi_awlock[1], m_axi_awlock[0]}),
        .m_axi_awcache({m_axi_awcache[7],m_axi_awcache[6],m_axi_awcache[5],m_axi_awcache[4],
                        m_axi_awcache[3],m_axi_awcache[2],m_axi_awcache[1],m_axi_awcache[0]}),
        .m_axi_awprot ({m_axi_awprot[7], m_axi_awprot[6], m_axi_awprot[5], m_axi_awprot[4],
                        m_axi_awprot[3], m_axi_awprot[2], m_axi_awprot[1], m_axi_awprot[0]}),
        .m_axi_awqos  ({m_axi_awqos[7],  m_axi_awqos[6],  m_axi_awqos[5],  m_axi_awqos[4],
                        m_axi_awqos[3],  m_axi_awqos[2],  m_axi_awqos[1],  m_axi_awqos[0]}),
        .m_axi_awvalid({m_axi_awvalid[7],m_axi_awvalid[6],m_axi_awvalid[5],m_axi_awvalid[4],
                        m_axi_awvalid[3],m_axi_awvalid[2],m_axi_awvalid[1],m_axi_awvalid[0]}),
        .m_axi_awready({m_axi_awready[7],m_axi_awready[6],m_axi_awready[5],m_axi_awready[4],
                        m_axi_awready[3],m_axi_awready[2],m_axi_awready[1],m_axi_awready[0]}),
        .m_axi_wdata  ({m_axi_wdata[7],  m_axi_wdata[6],  m_axi_wdata[5],  m_axi_wdata[4],
                        m_axi_wdata[3],  m_axi_wdata[2],  m_axi_wdata[1],  m_axi_wdata[0]}),
        .m_axi_wstrb  ({m_axi_wstrb[7],  m_axi_wstrb[6],  m_axi_wstrb[5],  m_axi_wstrb[4],
                        m_axi_wstrb[3],  m_axi_wstrb[2],  m_axi_wstrb[1],  m_axi_wstrb[0]}),
        .m_axi_wlast  ({m_axi_wlast[7],  m_axi_wlast[6],  m_axi_wlast[5],  m_axi_wlast[4],
                        m_axi_wlast[3],  m_axi_wlast[2],  m_axi_wlast[1],  m_axi_wlast[0]}),
        .m_axi_wvalid ({m_axi_wvalid[7], m_axi_wvalid[6], m_axi_wvalid[5], m_axi_wvalid[4],
                        m_axi_wvalid[3], m_axi_wvalid[2], m_axi_wvalid[1], m_axi_wvalid[0]}),
        .m_axi_wready ({m_axi_wready[7], m_axi_wready[6], m_axi_wready[5], m_axi_wready[4],
                        m_axi_wready[3], m_axi_wready[2], m_axi_wready[1], m_axi_wready[0]}),
        .m_axi_bid    ({m_axi_bid[7],    m_axi_bid[6],    m_axi_bid[5],    m_axi_bid[4],
                        m_axi_bid[3],    m_axi_bid[2],    m_axi_bid[1],    m_axi_bid[0]}),
        .m_axi_bresp  ({m_axi_bresp[7],  m_axi_bresp[6],  m_axi_bresp[5],  m_axi_bresp[4],
                        m_axi_bresp[3],  m_axi_bresp[2],  m_axi_bresp[1],  m_axi_bresp[0]}),
        .m_axi_bvalid ({m_axi_bvalid[7], m_axi_bvalid[6], m_axi_bvalid[5], m_axi_bvalid[4],
                        m_axi_bvalid[3], m_axi_bvalid[2], m_axi_bvalid[1], m_axi_bvalid[0]}),
        .m_axi_bready ({m_axi_bready[7], m_axi_bready[6], m_axi_bready[5], m_axi_bready[4],
                        m_axi_bready[3], m_axi_bready[2], m_axi_bready[1], m_axi_bready[0]}),
        .m_axi_arid   ({m_axi_arid[7],   m_axi_arid[6],   m_axi_arid[5],   m_axi_arid[4],
                        m_axi_arid[3],   m_axi_arid[2],   m_axi_arid[1],   m_axi_arid[0]}),
        .m_axi_araddr ({m_axi_araddr[7], m_axi_araddr[6], m_axi_araddr[5], m_axi_araddr[4],
                        m_axi_araddr[3], m_axi_araddr[2], m_axi_araddr[1], m_axi_araddr[0]}),
        .m_axi_arlen  ({m_axi_arlen[7],  m_axi_arlen[6],  m_axi_arlen[5],  m_axi_arlen[4],
                        m_axi_arlen[3],  m_axi_arlen[2],  m_axi_arlen[1],  m_axi_arlen[0]}),
        .m_axi_arsize ({m_axi_arsize[7], m_axi_arsize[6], m_axi_arsize[5], m_axi_arsize[4],
                        m_axi_arsize[3], m_axi_arsize[2], m_axi_arsize[1], m_axi_arsize[0]}),
        .m_axi_arburst({m_axi_arburst[7],m_axi_arburst[6],m_axi_arburst[5],m_axi_arburst[4],
                        m_axi_arburst[3],m_axi_arburst[2],m_axi_arburst[1],m_axi_arburst[0]}),
        .m_axi_arlock ({m_axi_arlock[7], m_axi_arlock[6], m_axi_arlock[5], m_axi_arlock[4],
                        m_axi_arlock[3], m_axi_arlock[2], m_axi_arlock[1], m_axi_arlock[0]}),
        .m_axi_arcache({m_axi_arcache[7],m_axi_arcache[6],m_axi_arcache[5],m_axi_arcache[4],
                        m_axi_arcache[3],m_axi_arcache[2],m_axi_arcache[1],m_axi_arcache[0]}),
        .m_axi_arprot ({m_axi_arprot[7], m_axi_arprot[6], m_axi_arprot[5], m_axi_arprot[4],
                        m_axi_arprot[3], m_axi_arprot[2], m_axi_arprot[1], m_axi_arprot[0]}),
        .m_axi_arqos  ({m_axi_arqos[7],  m_axi_arqos[6],  m_axi_arqos[5],  m_axi_arqos[4],
                        m_axi_arqos[3],  m_axi_arqos[2],  m_axi_arqos[1],  m_axi_arqos[0]}),
        .m_axi_arvalid({m_axi_arvalid[7],m_axi_arvalid[6],m_axi_arvalid[5],m_axi_arvalid[4],
                        m_axi_arvalid[3],m_axi_arvalid[2],m_axi_arvalid[1],m_axi_arvalid[0]}),
        .m_axi_arready({m_axi_arready[7],m_axi_arready[6],m_axi_arready[5],m_axi_arready[4],
                        m_axi_arready[3],m_axi_arready[2],m_axi_arready[1],m_axi_arready[0]}),
        .m_axi_rid    ({m_axi_rid[7],    m_axi_rid[6],    m_axi_rid[5],    m_axi_rid[4],
                        m_axi_rid[3],    m_axi_rid[2],    m_axi_rid[1],    m_axi_rid[0]}),
        .m_axi_rdata  ({m_axi_rdata[7],  m_axi_rdata[6],  m_axi_rdata[5],  m_axi_rdata[4],
                        m_axi_rdata[3],  m_axi_rdata[2],  m_axi_rdata[1],  m_axi_rdata[0]}),
        .m_axi_rresp  ({m_axi_rresp[7],  m_axi_rresp[6],  m_axi_rresp[5],  m_axi_rresp[4],
                        m_axi_rresp[3],  m_axi_rresp[2],  m_axi_rresp[1],  m_axi_rresp[0]}),
        .m_axi_rlast  ({m_axi_rlast[7],  m_axi_rlast[6],  m_axi_rlast[5],  m_axi_rlast[4],
                        m_axi_rlast[3],  m_axi_rlast[2],  m_axi_rlast[1],  m_axi_rlast[0]}),
        .m_axi_rvalid ({m_axi_rvalid[7], m_axi_rvalid[6], m_axi_rvalid[5], m_axi_rvalid[4],
                        m_axi_rvalid[3], m_axi_rvalid[2], m_axi_rvalid[1], m_axi_rvalid[0]}),
        .m_axi_rready ({m_axi_rready[7], m_axi_rready[6], m_axi_rready[5], m_axi_rready[4],
                        m_axi_rready[3], m_axi_rready[2], m_axi_rready[1], m_axi_rready[0]})
    );

    // ------------------------------------------------------------------
    // m00 = IMEM (simple AXI memory at 0x8000_0000)
    // ------------------------------------------------------------------
    axi_mem #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .BASE_ADDR (32'h8000_0000),
        .MEM_SIZE  (32'h0000_0800)
    ) imem (
        .clk   (clk),
        .rst   (rst),
        .awid  (m_axi_awid[0]),   .awaddr (m_axi_awaddr[0]),
        .awlen (m_axi_awlen[0]),  .awsize (m_axi_awsize[0]),
        .awburst(m_axi_awburst[0]),.awvalid(m_axi_awvalid[0]), .awready(m_axi_awready[0]),
        .wdata (m_axi_wdata[0]),  .wstrb (m_axi_wstrb[0]),
        .wlast (m_axi_wlast[0]),  .wvalid(m_axi_wvalid[0]),    .wready(m_axi_wready[0]),
        .bid   (m_axi_bid[0]),    .bresp (m_axi_bresp[0]),
        .bvalid(m_axi_bvalid[0]), .bready(m_axi_bready[0]),
        .arid  (m_axi_arid[0]),   .araddr (m_axi_araddr[0]),
        .arlen (m_axi_arlen[0]),  .arsize (m_axi_arsize[0]),
        .arburst(m_axi_arburst[0]),.arvalid(m_axi_arvalid[0]), .arready(m_axi_arready[0]),
        .rid   (m_axi_rid[0]),    .rdata (m_axi_rdata[0]),
        .rresp (m_axi_rresp[0]),  .rlast (m_axi_rlast[0]),
        .rvalid(m_axi_rvalid[0]), .rready(m_axi_rready[0])
    );

    // ------------------------------------------------------------------
    // m01 = DMEM (simple AXI memory at 0x1000_0000)
    // ------------------------------------------------------------------
    axi_mem #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .BASE_ADDR (32'h1000_0000),
        .MEM_SIZE  (32'h0000_0800)
    ) dmem (
        .clk   (clk),
        .rst   (rst),
        .awid  (m_axi_awid[1]),   .awaddr (m_axi_awaddr[1]),
        .awlen (m_axi_awlen[1]),  .awsize (m_axi_awsize[1]),
        .awburst(m_axi_awburst[1]),.awvalid(m_axi_awvalid[1]), .awready(m_axi_awready[1]),
        .wdata (m_axi_wdata[1]),  .wstrb (m_axi_wstrb[1]),
        .wlast (m_axi_wlast[1]),  .wvalid(m_axi_wvalid[1]),    .wready(m_axi_wready[1]),
        .bid   (m_axi_bid[1]),    .bresp (m_axi_bresp[1]),
        .bvalid(m_axi_bvalid[1]), .bready(m_axi_bready[1]),
        .arid  (m_axi_arid[1]),   .araddr (m_axi_araddr[1]),
        .arlen (m_axi_arlen[1]),  .arsize (m_axi_arsize[1]),
        .arburst(m_axi_arburst[1]),.arvalid(m_axi_arvalid[1]), .arready(m_axi_arready[1]),
        .rid   (m_axi_rid[1]),    .rdata (m_axi_rdata[1]),
        .rresp (m_axi_rresp[1]),  .rlast (m_axi_rlast[1]),
        .rvalid(m_axi_rvalid[1]), .rready(m_axi_rready[1])
    );

    // ------------------------------------------------------------------
    // m02-m03, m05-m07 = dummy slaves
    // ------------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 2; gi < 8; gi = gi + 1) begin : gen_dummy
            if (gi != 3 && gi != 4 && gi != 5 && gi != 6 && gi != 7) begin : dummy_slave
                axi_dummy_slave #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ADDR_WIDTH(ADDR_WIDTH),
                    .ID_WIDTH  (ID_WIDTH)
                ) dum (
                    .clk   (clk),
                    .rst   (rst),
                    .awid  (m_axi_awid[gi]),   .awaddr (m_axi_awaddr[gi]),
                    .awlen (m_axi_awlen[gi]),  .awsize (m_axi_awsize[gi]),
                    .awburst(m_axi_awburst[gi]),.awvalid(m_axi_awvalid[gi]), .awready(m_axi_awready[gi]),
                    .wdata (m_axi_wdata[gi]),  .wstrb (m_axi_wstrb[gi]),
                    .wlast (m_axi_wlast[gi]),  .wvalid(m_axi_wvalid[gi]),    .wready(m_axi_wready[gi]),
                    .bid   (m_axi_bid[gi]),    .bresp (m_axi_bresp[gi]),
                    .bvalid(m_axi_bvalid[gi]), .bready(m_axi_bready[gi]),
                    .arid  (m_axi_arid[gi]),   .araddr (m_axi_araddr[gi]),
                    .arlen (m_axi_arlen[gi]),  .arsize (m_axi_arsize[gi]),
                    .arburst(m_axi_arburst[gi]),.arvalid(m_axi_arvalid[gi]), .arready(m_axi_arready[gi]),
                    .rid   (m_axi_rid[gi]),    .rdata (m_axi_rdata[gi]),
                    .rresp (m_axi_rresp[gi]),  .rlast (m_axi_rlast[gi]),
                    .rvalid(m_axi_rvalid[gi]), .rready(m_axi_rready[gi])
                );
            end
        end
    endgenerate

    // ------------------------------------------------------------------
    // m04 = UART via bridge (0x4000_0000)
    // Interconnect is master, bridge is slave
    // Interconnect drives AW/W/AR to bridge; bridge drives B/R back
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     uart_awid, uart_arid, uart_rid, uart_bid;
    wire [ADDR_WIDTH-1:0]   uart_awaddr, uart_araddr;
    wire [7:0]              uart_awlen, uart_arlen;
    wire [2:0]              uart_awsize, uart_arsize;
    wire [1:0]              uart_awburst, uart_arburst;
    wire                    uart_awlock, uart_arlock;
    wire [3:0]              uart_awcache, uart_arcache;
    wire [2:0]              uart_awprot, uart_arprot;
    wire [3:0]              uart_awqos, uart_arqos;
    wire [3:0]              uart_awregion, uart_arregion;
    wire                    uart_awvalid, uart_awready;
    wire                    uart_wvalid, uart_wready;
    wire [DATA_WIDTH-1:0]   uart_wdata;
    wire [STRB_WIDTH-1:0]   uart_wstrb;
    wire                    uart_wlast;
    wire                    uart_bvalid, uart_bready;
    wire [1:0]              uart_bresp;
    wire                    uart_arvalid, uart_arready;
    wire                    uart_rvalid, uart_rready;
    wire [DATA_WIDTH-1:0]   uart_rdata;
    wire [1:0]              uart_rresp;
    wire                    uart_rlast;

    // Interconnect (master) -> UART bridge (slave): AW channel
    assign uart_awid     = m_axi_awid[4];
    assign uart_awaddr   = m_axi_awaddr[4];
    assign uart_awlen    = m_axi_awlen[4];
    assign uart_awsize   = m_axi_awsize[4];
    assign uart_awburst  = m_axi_awburst[4];
    assign uart_awlock   = m_axi_awlock[4];
    assign uart_awcache  = m_axi_awcache[4];
    assign uart_awprot   = m_axi_awprot[4];
    assign uart_awqos    = m_axi_awqos[4];
    assign uart_awregion = m_axi_awregion[4];
    assign uart_awvalid  = m_axi_awvalid[4];
    assign m_axi_awready[4] = uart_awready;

    // Interconnect (master) -> UART bridge (slave): W channel
    assign uart_wdata   = m_axi_wdata[4];
    assign uart_wstrb   = m_axi_wstrb[4];
    assign uart_wlast   = m_axi_wlast[4];
    assign uart_wvalid  = m_axi_wvalid[4];
    assign m_axi_wready[4] = uart_wready;

    // UART bridge (slave) -> Interconnect (master): B channel
    assign m_axi_bid[4]    = uart_bid;
    assign m_axi_bresp[4]  = uart_bresp;
    assign m_axi_bvalid[4] = uart_bvalid;
    assign uart_bready     = m_axi_bready[4];

    // Interconnect (master) -> UART bridge (slave): AR channel
    assign uart_arid     = m_axi_arid[4];
    assign uart_araddr   = m_axi_araddr[4];
    assign uart_arlen    = m_axi_arlen[4];
    assign uart_arsize   = m_axi_arsize[4];
    assign uart_arburst  = m_axi_arburst[4];
    assign uart_arlock   = m_axi_arlock[4];
    assign uart_arcache  = m_axi_arcache[4];
    assign uart_arprot   = m_axi_arprot[4];
    assign uart_arqos    = m_axi_arqos[4];
    assign uart_arregion = m_axi_arregion[4];
    assign uart_arvalid  = m_axi_arvalid[4];
    assign m_axi_arready[4] = uart_arready;

    // UART bridge (slave) -> Interconnect (master): R channel
    // Bridge drives these back to the interconnect
    assign m_axi_rid[4]      = uart_rid;
    assign m_axi_rdata[4]    = uart_rdata;
    assign m_axi_rresp[4]    = uart_rresp;
    assign m_axi_rlast[4]    = uart_rlast;
    assign m_axi_rvalid[4]   = uart_rvalid;
    assign uart_rready       = m_axi_rready[4];

    axi_uart_bridge #(
        .IC_DATA_WIDTH (DATA_WIDTH),
        .IC_ADDR_WIDTH (ADDR_WIDTH),
        .IC_ID_WIDTH   (ID_WIDTH)
    ) uart_bridge (
        .clk    (clk),
        .rst    (rst),
        .s_axi_awid   (uart_awid),
        .s_axi_awaddr (uart_awaddr),
        .s_axi_awlen  (uart_awlen),
        .s_axi_awsize (uart_awsize),
        .s_axi_awburst(uart_awburst),
        .s_axi_awlock (uart_awlock),
        .s_axi_awcache(uart_awcache),
        .s_axi_awprot (uart_awprot),
        .s_axi_awqos  (uart_awqos),
        .s_axi_awregion(uart_awregion),
        .s_axi_awvalid(uart_awvalid),
        .s_axi_awready(uart_awready),
        .s_axi_wdata  (uart_wdata),
        .s_axi_wstrb  (uart_wstrb),
        .s_axi_wlast  (uart_wlast),
        .s_axi_wvalid (uart_wvalid),
        .s_axi_wready (uart_wready),
        .s_axi_bid    (uart_bid),
        .s_axi_bresp  (uart_bresp),
        .s_axi_bvalid (uart_bvalid),
        .s_axi_bready (uart_bready),
        .s_axi_arid   (uart_arid),
        .s_axi_araddr (uart_araddr),
        .s_axi_arlen  (uart_arlen),
        .s_axi_arsize (uart_arsize),
        .s_axi_arburst(uart_arburst),
        .s_axi_arlock (uart_arlock),
        .s_axi_arcache(uart_arcache),
        .s_axi_arprot (uart_arprot),
        .s_axi_arqos  (uart_arqos),
        .s_axi_arregion(uart_arregion),
        .s_axi_arvalid(uart_arvalid),
        .s_axi_arready(uart_arready),
        .s_axi_rid    (uart_rid),
        .s_axi_rdata  (uart_rdata),
        .s_axi_rresp  (uart_rresp),
        .s_axi_rlast  (uart_rlast),
        .s_axi_rvalid (uart_rvalid),
        .s_axi_rready (uart_rready),
        .uart_tx_o    (uart_tx_o),
        .uart_rx_i    (uart_rx_i),
        .uart_irq_o   (uart_irq_o)
    );

    // ------------------------------------------------------------------
    // m06 = GPIO (0x6000_0000)
    // Interconnect (master) -> axi_gpio (slave)
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     gpio_awid, gpio_arid, gpio_rid, gpio_bid;
    wire [ADDR_WIDTH-1:0]   gpio_awaddr, gpio_araddr;
    wire [7:0]              gpio_awlen, gpio_arlen;
    wire [2:0]              gpio_awsize, gpio_arsize;
    wire [1:0]              gpio_awburst, gpio_arburst;
    wire                    gpio_awlock, gpio_arlock;
    wire [3:0]              gpio_awcache, gpio_arcache;
    wire [2:0]              gpio_awprot, gpio_arprot;
    wire [3:0]              gpio_awqos, gpio_arqos;
    wire [3:0]              gpio_awregion, gpio_arregion;
    wire                    gpio_awvalid, gpio_awready;
    wire                    gpio_wvalid, gpio_wready;
    wire [DATA_WIDTH-1:0]   gpio_wdata;
    wire [STRB_WIDTH-1:0]   gpio_wstrb;
    wire                    gpio_wlast;
    wire                    gpio_bvalid, gpio_bready;
    wire [1:0]              gpio_bresp;
    wire                    gpio_arvalid, gpio_arready;
    wire                    gpio_rvalid, gpio_rready;
    wire [DATA_WIDTH-1:0]   gpio_rdata;
    wire [1:0]              gpio_rresp;
    wire                    gpio_rlast;

    // AW channel: interconnect -> GPIO
    assign gpio_awid      = m_axi_awid[6];
    assign gpio_awaddr    = m_axi_awaddr[6];
    assign gpio_awlen     = m_axi_awlen[6];
    assign gpio_awsize    = m_axi_awsize[6];
    assign gpio_awburst   = m_axi_awburst[6];
    assign gpio_awlock    = m_axi_awlock[6];
    assign gpio_awcache   = m_axi_awcache[6];
    assign gpio_awprot    = m_axi_awprot[6];
    assign gpio_awqos     = m_axi_awqos[6];
    assign gpio_awregion  = m_axi_awregion[6];
    assign gpio_awvalid   = m_axi_awvalid[6];
    assign m_axi_awready[6] = gpio_awready;

    // W channel: interconnect -> GPIO
    assign gpio_wdata   = m_axi_wdata[6];
    assign gpio_wstrb   = m_axi_wstrb[6];
    assign gpio_wlast   = m_axi_wlast[6];
    assign gpio_wvalid  = m_axi_wvalid[6];
    assign m_axi_wready[6] = gpio_wready;

    // B channel: GPIO -> interconnect
    assign m_axi_bid[6]    = gpio_bid;
    assign m_axi_bresp[6]  = gpio_bresp;
    assign m_axi_bvalid[6] = gpio_bvalid;
    assign gpio_bready     = m_axi_bready[6];

    // AR channel: interconnect -> GPIO
    assign gpio_arid      = m_axi_arid[6];
    assign gpio_araddr    = m_axi_araddr[6];
    assign gpio_arlen     = m_axi_arlen[6];
    assign gpio_arsize    = m_axi_arsize[6];
    assign gpio_arburst   = m_axi_arburst[6];
    assign gpio_arlock    = m_axi_arlock[6];
    assign gpio_arcache   = m_axi_arcache[6];
    assign gpio_arprot    = m_axi_arprot[6];
    assign gpio_arqos     = m_axi_arqos[6];
    assign gpio_arregion  = m_axi_arregion[6];
    assign gpio_arvalid   = m_axi_arvalid[6];
    assign m_axi_arready[6] = gpio_arready;

    // R channel: GPIO -> interconnect
    assign m_axi_rid[6]      = gpio_rid;
    assign m_axi_rdata[6]    = gpio_rdata;
    assign m_axi_rresp[6]    = gpio_rresp;
    assign m_axi_rlast[6]    = gpio_rlast;
    assign m_axi_rvalid[6]   = gpio_rvalid;
    assign gpio_rready       = m_axi_rready[6];

    axi_gpio #(
        .GPIO_WIDTH         (16),
        .AXI_DATA_WIDTH     (DATA_WIDTH),
        .AXI_ADDR_WIDTH     (ADDR_WIDTH),
        .AXI_ID_WIDTH       (ID_WIDTH),
        .AXI_STRB_WIDTH     (STRB_WIDTH),
        .BASE_ADDR          (32'h6000_0000)
    ) gpio_inst (
        .clk                (clk),
        .rst                (rst),
        .s_axi_awid         (gpio_awid),
        .s_axi_awaddr       (gpio_awaddr),
        .s_axi_awlen        (gpio_awlen),
        .s_axi_awsize       (gpio_awsize),
        .s_axi_awburst      (gpio_awburst),
        .s_axi_awlock       (gpio_awlock),
        .s_axi_awcache      (gpio_awcache),
        .s_axi_awprot       (gpio_awprot),
        .s_axi_awqos        (gpio_awqos),
        .s_axi_awregion     (gpio_awregion),
        .s_axi_awvalid      (gpio_awvalid),
        .s_axi_awready      (gpio_awready),
        .s_axi_wdata        (gpio_wdata),
        .s_axi_wstrb        (gpio_wstrb),
        .s_axi_wlast        (gpio_wlast),
        .s_axi_wvalid       (gpio_wvalid),
        .s_axi_wready       (gpio_wready),
        .s_axi_bid          (gpio_bid),
        .s_axi_bresp        (gpio_bresp),
        .s_axi_bvalid       (gpio_bvalid),
        .s_axi_bready       (gpio_bready),
        .s_axi_arid         (gpio_arid),
        .s_axi_araddr       (gpio_araddr),
        .s_axi_arlen        (gpio_arlen),
        .s_axi_arsize       (gpio_arsize),
        .s_axi_arburst      (gpio_arburst),
        .s_axi_arlock       (gpio_arlock),
        .s_axi_arcache      (gpio_arcache),
        .s_axi_arprot       (gpio_arprot),
        .s_axi_arqos        (gpio_arqos),
        .s_axi_arregion     (gpio_arregion),
        .s_axi_arvalid      (gpio_arvalid),
        .s_axi_arready      (gpio_arready),
        .s_axi_rid          (gpio_rid),
        .s_axi_rdata        (gpio_rdata),
        .s_axi_rresp        (gpio_rresp),
        .s_axi_rlast        (gpio_rlast),
        .s_axi_rvalid       (gpio_rvalid),
        .s_axi_rready       (gpio_rready),
        .gpio_i             (gpio_pads_i),
        .gpio_o             (gpio_pads_o),
        .gpio_oen           (gpio_pads_oen)
    );

    // ------------------------------------------------------------------
    // m05 = Timer (0x5000_0000)
    // Interconnect (master) -> axi_timer (slave)
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     timer_awid, timer_arid, timer_rid, timer_bid;
    wire [ADDR_WIDTH-1:0]   timer_awaddr, timer_araddr;
    wire [7:0]              timer_awlen, timer_arlen;
    wire [2:0]              timer_awsize, timer_arsize;
    wire [1:0]              timer_awburst, timer_arburst;
    wire                    timer_awlock, timer_arlock;
    wire [3:0]              timer_awcache, timer_arcache;
    wire [2:0]              timer_awprot, timer_arprot;
    wire [3:0]              timer_awqos, timer_arqos;
    wire [3:0]              timer_awregion, timer_arregion;
    wire                    timer_awvalid, timer_awready;
    wire                    timer_wvalid, timer_wready;
    wire [DATA_WIDTH-1:0]   timer_wdata;
    wire [STRB_WIDTH-1:0]   timer_wstrb;
    wire                    timer_wlast;
    wire                    timer_bvalid, timer_bready;
    wire [1:0]              timer_bresp;
    wire                    timer_arvalid, timer_arready;
    wire                    timer_rvalid, timer_rready;
    wire [DATA_WIDTH-1:0]   timer_rdata;
    wire [1:0]              timer_rresp;
    wire                    timer_rlast;

    // AW channel: interconnect -> Timer
    assign timer_awid      = m_axi_awid[5];
    assign timer_awaddr    = m_axi_awaddr[5];
    assign timer_awlen     = m_axi_awlen[5];
    assign timer_awsize    = m_axi_awsize[5];
    assign timer_awburst   = m_axi_awburst[5];
    assign timer_awlock    = m_axi_awlock[5];
    assign timer_awcache   = m_axi_awcache[5];
    assign timer_awprot    = m_axi_awprot[5];
    assign timer_awqos     = m_axi_awqos[5];
    assign timer_awregion  = m_axi_awregion[5];
    assign timer_awvalid   = m_axi_awvalid[5];
    assign m_axi_awready[5] = timer_awready;

    // W channel: interconnect -> Timer
    assign timer_wdata   = m_axi_wdata[5];
    assign timer_wstrb   = m_axi_wstrb[5];
    assign timer_wlast   = m_axi_wlast[5];
    assign timer_wvalid  = m_axi_wvalid[5];
    assign m_axi_wready[5] = timer_wready;

    // B channel: Timer -> interconnect
    assign m_axi_bid[5]    = timer_bid;
    assign m_axi_bresp[5]  = timer_bresp;
    assign m_axi_bvalid[5] = timer_bvalid;
    assign timer_bready    = m_axi_bready[5];

    // AR channel: interconnect -> Timer
    assign timer_arid      = m_axi_arid[5];
    assign timer_araddr    = m_axi_araddr[5];
    assign timer_arlen     = m_axi_arlen[5];
    assign timer_arsize    = m_axi_arsize[5];
    assign timer_arburst   = m_axi_arburst[5];
    assign timer_arlock    = m_axi_arlock[5];
    assign timer_arcache   = m_axi_arcache[5];
    assign timer_arprot    = m_axi_arprot[5];
    assign timer_arqos     = m_axi_arqos[5];
    assign timer_arregion  = m_axi_arregion[5];
    assign timer_arvalid   = m_axi_arvalid[5];
    assign m_axi_arready[5] = timer_arready;

    // R channel: Timer -> interconnect
    assign m_axi_rid[5]      = timer_rid;
    assign m_axi_rdata[5]    = timer_rdata;
    assign m_axi_rresp[5]    = timer_rresp;
    assign m_axi_rlast[5]    = timer_rlast;
    assign m_axi_rvalid[5]   = timer_rvalid;
    assign timer_rready      = m_axi_rready[5];

    axi_timer #(
        .TIMER_WIDTH         (32),
        .AXI_DATA_WIDTH      (DATA_WIDTH),
        .AXI_ADDR_WIDTH      (ADDR_WIDTH),
        .AXI_ID_WIDTH        (ID_WIDTH),
        .AXI_STRB_WIDTH      (STRB_WIDTH),
        .BASE_ADDR           (32'h5000_0000),
        .TIMER_ID_VALUE      (32'h54494D52),  // "TIMR"
        .TIMER_VERSION_VALUE (32'h00010000)
    ) timer_inst (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (timer_awid),
        .s_axi_awaddr        (timer_awaddr),
        .s_axi_awlen         (timer_awlen),
        .s_axi_awsize        (timer_awsize),
        .s_axi_awburst       (timer_awburst),
        .s_axi_awlock        (timer_awlock),
        .s_axi_awcache       (timer_awcache),
        .s_axi_awprot        (timer_awprot),
        .s_axi_awqos         (timer_awqos),
        .s_axi_awregion      (timer_awregion),
        .s_axi_awvalid       (timer_awvalid),
        .s_axi_awready       (timer_awready),
        .s_axi_wdata         (timer_wdata),
        .s_axi_wstrb         (timer_wstrb),
        .s_axi_wlast         (timer_wlast),
        .s_axi_wvalid        (timer_wvalid),
        .s_axi_wready        (timer_wready),
        .s_axi_bid           (timer_bid),
        .s_axi_bresp         (timer_bresp),
        .s_axi_bvalid        (timer_bvalid),
        .s_axi_bready        (timer_bready),
        .s_axi_arid          (timer_arid),
        .s_axi_araddr        (timer_araddr),
        .s_axi_arlen         (timer_arlen),
        .s_axi_arsize        (timer_arsize),
        .s_axi_arburst       (timer_arburst),
        .s_axi_arlock        (timer_arlock),
        .s_axi_arcache       (timer_arcache),
        .s_axi_arprot        (timer_arprot),
        .s_axi_arqos         (timer_arqos),
        .s_axi_arregion      (timer_arregion),
        .s_axi_arvalid       (timer_arvalid),
        .s_axi_arready       (timer_arready),
        .s_axi_rid           (timer_rid),
        .s_axi_rdata         (timer_rdata),
        .s_axi_rresp         (timer_rresp),
        .s_axi_rlast         (timer_rlast),
        .s_axi_rvalid        (timer_rvalid),
        .s_axi_rready        (timer_rready),
        .timer_irq_o         (timer_irq_o)
    );

    // ------------------------------------------------------------------
    // m03 = CNN Convolution Accelerator (0x3000_0000)
    // Interconnect (master) -> axi_cnn_conv (slave)
    // ------------------------------------------------------------------
    wire [ID_WIDTH-1:0]     cnn_awid, cnn_arid, cnn_rid, cnn_bid;
    wire [ADDR_WIDTH-1:0]   cnn_awaddr, cnn_araddr;
    wire [7:0]              cnn_awlen, cnn_arlen;
    wire [2:0]              cnn_awsize, cnn_arsize;
    wire [1:0]              cnn_awburst, cnn_arburst;
    wire                    cnn_awlock, cnn_arlock;
    wire [3:0]              cnn_awcache, cnn_arcache;
    wire [2:0]              cnn_awprot, cnn_arprot;
    wire [3:0]              cnn_awqos, cnn_arqos;
    wire [3:0]              cnn_awregion, cnn_arregion;
    wire                    cnn_awvalid, cnn_awready;
    wire                    cnn_wvalid, cnn_wready;
    wire [DATA_WIDTH-1:0]   cnn_wdata;
    wire [STRB_WIDTH-1:0]   cnn_wstrb;
    wire                    cnn_wlast;
    wire                    cnn_bvalid, cnn_bready;
    wire [1:0]              cnn_bresp;
    wire                    cnn_arvalid, cnn_arready;
    wire                    cnn_rvalid, cnn_rready;
    wire [DATA_WIDTH-1:0]   cnn_rdata;
    wire [1:0]              cnn_rresp;
    wire                    cnn_rlast;

    // AW channel: interconnect -> CNN
    assign cnn_awid      = m_axi_awid[3];
    assign cnn_awaddr    = m_axi_awaddr[3];
    assign cnn_awlen     = m_axi_awlen[3];
    assign cnn_awsize    = m_axi_awsize[3];
    assign cnn_awburst   = m_axi_awburst[3];
    assign cnn_awlock    = m_axi_awlock[3];
    assign cnn_awcache   = m_axi_awcache[3];
    assign cnn_awprot    = m_axi_awprot[3];
    assign cnn_awqos     = m_axi_awqos[3];
    assign cnn_awregion  = m_axi_awregion[3];
    assign cnn_awvalid   = m_axi_awvalid[3];
    assign m_axi_awready[3] = cnn_awready;

    // W channel: interconnect -> CNN
    assign cnn_wdata   = m_axi_wdata[3];
    assign cnn_wstrb   = m_axi_wstrb[3];
    assign cnn_wlast   = m_axi_wlast[3];
    assign cnn_wvalid  = m_axi_wvalid[3];
    assign m_axi_wready[3] = cnn_wready;

    // B channel: CNN -> interconnect
    assign m_axi_bid[3]    = cnn_bid;
    assign m_axi_bresp[3]  = cnn_bresp;
    assign m_axi_bvalid[3] = cnn_bvalid;
    assign cnn_bready     = m_axi_bready[3];

    // AR channel: interconnect -> CNN
    assign cnn_arid      = m_axi_arid[3];
    assign cnn_araddr    = m_axi_araddr[3];
    assign cnn_arlen     = m_axi_arlen[3];
    assign cnn_arsize    = m_axi_arsize[3];
    assign cnn_arburst   = m_axi_arburst[3];
    assign cnn_arlock    = m_axi_arlock[3];
    assign cnn_arcache   = m_axi_arcache[3];
    assign cnn_arprot    = m_axi_arprot[3];
    assign cnn_arqos     = m_axi_arqos[3];
    assign cnn_arregion  = m_axi_arregion[3];
    assign cnn_arvalid   = m_axi_arvalid[3];
    assign m_axi_arready[3] = cnn_arready;

    // R channel: CNN -> interconnect
    assign m_axi_rid[3]      = cnn_rid;
    assign m_axi_rdata[3]    = cnn_rdata;
    assign m_axi_rresp[3]    = cnn_rresp;
    assign m_axi_rlast[3]    = cnn_rlast;
    assign m_axi_rvalid[3]   = cnn_rvalid;
    assign cnn_rready       = m_axi_rready[3];

    axi_cnn_conv #(
        .DATA_WIDTH       (DATA_WIDTH),
        .AXI_DATA_WIDTH   (DATA_WIDTH),
        .AXI_ADDR_WIDTH   (ADDR_WIDTH),
        .AXI_ID_WIDTH     (ID_WIDTH),
        .AXI_STRB_WIDTH   (STRB_WIDTH),
        .BASE_ADDR        (32'h3000_0000),
        .CNN_ID_VALUE     (32'h434E4E56),  // "CNNV"
        .CNN_VERSION_VALUE(32'h00010000)
    ) cnn_inst (
        .clk                (clk),
        .rst                (rst),
        .s_axi_awid         (cnn_awid),
        .s_axi_awaddr       (cnn_awaddr),
        .s_axi_awlen        (cnn_awlen),
        .s_axi_awsize       (cnn_awsize),
        .s_axi_awburst      (cnn_awburst),
        .s_axi_awlock       (cnn_awlock),
        .s_axi_awcache      (cnn_awcache),
        .s_axi_awprot       (cnn_awprot),
        .s_axi_awqos        (cnn_awqos),
        .s_axi_awregion     (cnn_awregion),
        .s_axi_awvalid      (cnn_awvalid),
        .s_axi_awready      (cnn_awready),
        .s_axi_wdata        (cnn_wdata),
        .s_axi_wstrb        (cnn_wstrb),
        .s_axi_wlast        (cnn_wlast),
        .s_axi_wvalid       (cnn_wvalid),
        .s_axi_wready       (cnn_wready),
        .s_axi_bid          (cnn_bid),
        .s_axi_bresp        (cnn_bresp),
        .s_axi_bvalid       (cnn_bvalid),
        .s_axi_bready       (cnn_bready),
        .s_axi_arid         (cnn_arid),
        .s_axi_araddr       (cnn_araddr),
        .s_axi_arlen        (cnn_arlen),
        .s_axi_arsize       (cnn_arsize),
        .s_axi_arburst      (cnn_arburst),
        .s_axi_arlock       (cnn_arlock),
        .s_axi_arcache      (cnn_arcache),
        .s_axi_arprot       (cnn_arprot),
        .s_axi_arqos        (cnn_arqos),
        .s_axi_arregion     (cnn_arregion),
        .s_axi_arvalid      (cnn_arvalid),
        .s_axi_arready      (cnn_arready),
        .s_axi_rid          (cnn_rid),
        .s_axi_rdata        (cnn_rdata),
        .s_axi_rresp        (cnn_rresp),
        .s_axi_rlast        (cnn_rlast),
        .s_axi_rvalid       (cnn_rvalid),
        .s_axi_rready       (cnn_rready),
        .cnn_irq_o          (cnn_irq_o)
    );

    // ------------------------------------------------------------------
    // m07 = 7-Segment Display Controller (0x7000_0000)
    // ------------------------------------------------------------------
    axi_seven_seg #(
        .AXI_DATA_WIDTH      (DATA_WIDTH),
        .AXI_ADDR_WIDTH      (ADDR_WIDTH),
        .AXI_ID_WIDTH        (ID_WIDTH),
        .AXI_STRB_WIDTH      (STRB_WIDTH),
        .BASE_ADDR           (32'h7000_0000)
    ) seven_seg_inst (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (m_axi_awid[7]),
        .s_axi_awaddr        (m_axi_awaddr[7]),
        .s_axi_awlen         (m_axi_awlen[7]),
        .s_axi_awsize        (m_axi_awsize[7]),
        .s_axi_awburst       (m_axi_awburst[7]),
        .s_axi_awlock        (m_axi_awlock[7]),
        .s_axi_awcache       (m_axi_awcache[7]),
        .s_axi_awprot        (m_axi_awprot[7]),
        .s_axi_awqos         (m_axi_awqos[7]),
        .s_axi_awregion      (m_axi_awregion[7]),
        .s_axi_awvalid       (m_axi_awvalid[7]),
        .s_axi_awready       (m_axi_awready[7]),
        .s_axi_wdata         (m_axi_wdata[7]),
        .s_axi_wstrb         (m_axi_wstrb[7]),
        .s_axi_wlast         (m_axi_wlast[7]),
        .s_axi_wvalid        (m_axi_wvalid[7]),
        .s_axi_wready        (m_axi_wready[7]),
        .s_axi_bid           (m_axi_bid[7]),
        .s_axi_bresp         (m_axi_bresp[7]),
        .s_axi_bvalid        (m_axi_bvalid[7]),
        .s_axi_bready        (m_axi_bready[7]),
        .s_axi_arid          (m_axi_arid[7]),
        .s_axi_araddr        (m_axi_araddr[7]),
        .s_axi_arlen         (m_axi_arlen[7]),
        .s_axi_arsize        (m_axi_arsize[7]),
        .s_axi_arburst       (m_axi_arburst[7]),
        .s_axi_arlock        (m_axi_arlock[7]),
        .s_axi_arcache       (m_axi_arcache[7]),
        .s_axi_arprot        (m_axi_arprot[7]),
        .s_axi_arqos         (m_axi_arqos[7]),
        .s_axi_arregion      (m_axi_arregion[7]),
        .s_axi_arvalid       (m_axi_arvalid[7]),
        .s_axi_arready       (m_axi_arready[7]),
        .s_axi_rid           (m_axi_rid[7]),
        .s_axi_rdata         (m_axi_rdata[7]),
        .s_axi_rresp         (m_axi_rresp[7]),
        .s_axi_rlast         (m_axi_rlast[7]),
        .s_axi_rvalid        (m_axi_rvalid[7]),
        .s_axi_rready        (m_axi_rready[7]),
        .seg_o               (seven_seg_seg_o),
        .dp_o                (seven_seg_dp_o),
        .an_o                (seven_seg_an_o)
    );

endmodule