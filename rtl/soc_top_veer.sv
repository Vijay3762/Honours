// ============================================================================
// File    : soc_top_veer.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : SoC top level with the real VeeR EL2 processor core, complete
//           3-master x 10-slave AXI4 crossbar, DMA engine, and CNN accelerators.
//
// Topology:
//   VeeR EL2 IFU (AXI4 master 0) -----\
//   VeeR EL2 LSU (AXI4 master 1) ------+--> axi_interconnect (3 masters x 10 slaves)
//   DMA Master   (AXI4 master 2) -----/            |
//                                                  |-- m00  0x8000_0000  axi_mem         (IMEM, firmware)
//                                                  |-- m01  0x1000_0000  axi_mem         (DMEM)
//                                                  |-- m02  0x2000_0000  axi_dma_ctrl    (DMA regs)
//                                                  |-- m03  0x3000_0000  axi_cnn_conv    (CNN conv regs)
//                                                  |-- m04  0x4000_0000  axi_uart_bridge -> axi_uart_top
//                                                  |-- m05  0x5000_0000  axi_timer       (Timer)
//                                                  |-- m06  0x6000_0000  axi_gpio        (GPIO)
//                                                  |-- m07  0x7000_0000  axi_seven_seg   (7-segment)
//                                                  |-- m08  0x9000_0000  axi_relu        (ReLU unit)
//                                                  |-- m09  0xA000_0000  axi_pooling     (Pooling unit)
//
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module soc_top_veer #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter ID_WIDTH   = 8,
    // Firmware image loaded into the IMEM window (64 bit words)
    parameter IMEM_HEX   = "",
    // Firmware image loaded into the ICCM of the core (32 bit words)
    parameter ICCM_HEX   = ""
)(
    input  wire                    clk,
    input  wire                    rst,          // active-high system reset
    output wire                    uart_tx_o,
    input  wire                    uart_rx_i,
    output wire                    uart_irq_o,

    // GPIO external
    input  wire [15:0]             gpio_pads_i,
    output wire [15:0]             gpio_pads_o,
    output wire [15:0]             gpio_pads_oen,

    // Timer external
    output wire                    timer_irq_o,

    // DMA external interrupt
    output wire                    dma_irq_o,

    // CNN external interrupt
    output wire                    cnn_irq_o,

    // 7-Segment display external (Basys3)
    output wire [6:0]              seven_seg_seg_o,
    output wire                    seven_seg_dp_o,
    output wire [3:0]              seven_seg_an_o,

    // Optional external AXI master port (for UVM VIP / testbench verification)
    input  wire                    ext_master_en,
    input  wire [ID_WIDTH-1:0]     ext_m_axi_awid,
    input  wire [ADDR_WIDTH-1:0]   ext_m_axi_awaddr,
    input  wire [7:0]              ext_m_axi_awlen,
    input  wire [2:0]              ext_m_axi_awsize,
    input  wire [1:0]              ext_m_axi_awburst,
    input  wire                    ext_m_axi_awlock,
    input  wire [3:0]              ext_m_axi_awcache,
    input  wire [2:0]              ext_m_axi_awprot,
    input  wire [3:0]              ext_m_axi_awqos,
    input  wire                    ext_m_axi_awvalid,
    output wire                    ext_m_axi_awready,
    input  wire [DATA_WIDTH-1:0]   ext_m_axi_wdata,
    input  wire [STRB_WIDTH-1:0]   ext_m_axi_wstrb,
    input  wire                    ext_m_axi_wlast,
    input  wire                    ext_m_axi_wvalid,
    output wire                    ext_m_axi_wready,
    output wire [ID_WIDTH-1:0]     ext_m_axi_bid,
    output wire [1:0]              ext_m_axi_bresp,
    output wire                    ext_m_axi_bvalid,
    input  wire                    ext_m_axi_bready,
    input  wire [ID_WIDTH-1:0]     ext_m_axi_arid,
    input  wire [ADDR_WIDTH-1:0]   ext_m_axi_araddr,
    input  wire [7:0]              ext_m_axi_arlen,
    input  wire [2:0]              ext_m_axi_arsize,
    input  wire [1:0]              ext_m_axi_arburst,
    input  wire                    ext_m_axi_arlock,
    input  wire [3:0]              ext_m_axi_arcache,
    input  wire [2:0]              ext_m_axi_arprot,
    input  wire [3:0]              ext_m_axi_arqos,
    input  wire                    ext_m_axi_arvalid,
    output wire                    ext_m_axi_arready,
    output wire [ID_WIDTH-1:0]     ext_m_axi_rid,
    output wire [DATA_WIDTH-1:0]   ext_m_axi_rdata,
    output wire [1:0]              ext_m_axi_rresp,
    output wire                    ext_m_axi_rlast,
    output wire                    ext_m_axi_rvalid,
    input  wire                    ext_m_axi_rready
);

    localparam integer S_COUNT = 3;   // IFU + LSU + DMA Master
    localparam integer M_COUNT = 10;  // IMEM, DMEM, DMA, CNN, UART, TIMER, GPIO, 7-SEG, RELU, POOL

    // ------------------------------------------------------------------
    // Interrupt lines to PIC
    // ------------------------------------------------------------------
    wire [31:1] extintsrc_req;
    assign extintsrc_req[1]    = dma_irq_o;
    assign extintsrc_req[2]    = cnn_irq_o;
    assign extintsrc_req[31:3] = '0;

    // ------------------------------------------------------------------
    // VeeR EL2 core
    // ------------------------------------------------------------------
    wire        core_rst_l   = ~rst;

    // LSU AXI4 master
    wire                    lsu_axi_awvalid;
    wire                    lsu_axi_awready;
    wire [2:0]              lsu_axi_awid;
    wire [31:0]             lsu_axi_awaddr;
    wire [3:0]              lsu_axi_awregion;
    wire [7:0]              lsu_axi_awlen;
    wire [2:0]              lsu_axi_awsize;
    wire [1:0]              lsu_axi_awburst;
    wire                    lsu_axi_awlock;
    wire [3:0]              lsu_axi_awcache;
    wire [2:0]              lsu_axi_awprot;
    wire [3:0]              lsu_axi_awqos;
    wire                    lsu_axi_wvalid;
    wire                    lsu_axi_wready;
    wire [63:0]             lsu_axi_wdata;
    wire [7:0]              lsu_axi_wstrb;
    wire                    lsu_axi_wlast;
    wire                    lsu_axi_bvalid;
    wire                    lsu_axi_bready;
    wire [1:0]              lsu_axi_bresp;
    wire [2:0]              lsu_axi_bid;
    wire                    lsu_axi_arvalid;
    wire                    lsu_axi_arready;
    wire [2:0]              lsu_axi_arid;
    wire [31:0]             lsu_axi_araddr;
    wire [3:0]              lsu_axi_arregion;
    wire [7:0]              lsu_axi_arlen;
    wire [2:0]              lsu_axi_arsize;
    wire [1:0]              lsu_axi_arburst;
    wire                    lsu_axi_arlock;
    wire [3:0]              lsu_axi_arcache;
    wire [2:0]              lsu_axi_arprot;
    wire [3:0]              lsu_axi_arqos;
    wire                    lsu_axi_rvalid;
    wire                    lsu_axi_rready;
    wire [2:0]              lsu_axi_rid;
    wire [63:0]             lsu_axi_rdata;
    wire [1:0]              lsu_axi_rresp;
    wire                    lsu_axi_rlast;

    // IFU AXI4 master
    wire                    ifu_axi_awvalid;
    wire                    ifu_axi_awready;
    wire [2:0]              ifu_axi_awid;
    wire [31:0]             ifu_axi_awaddr;
    wire [3:0]              ifu_axi_awregion;
    wire [7:0]              ifu_axi_awlen;
    wire [2:0]              ifu_axi_awsize;
    wire [1:0]              ifu_axi_awburst;
    wire                    ifu_axi_awlock;
    wire [3:0]              ifu_axi_awcache;
    wire [2:0]              ifu_axi_awprot;
    wire [3:0]              ifu_axi_awqos;
    wire                    ifu_axi_wvalid;
    wire                    ifu_axi_wready;
    wire [63:0]             ifu_axi_wdata;
    wire [7:0]              ifu_axi_wstrb;
    wire                    ifu_axi_wlast;
    wire                    ifu_axi_bvalid;
    wire                    ifu_axi_bready;
    wire [1:0]              ifu_axi_bresp;
    wire [2:0]              ifu_axi_bid;
    wire                    ifu_axi_arvalid;
    wire                    ifu_axi_arready;
    wire [2:0]              ifu_axi_arid;
    wire [31:0]             ifu_axi_araddr;
    wire [3:0]              ifu_axi_arregion;
    wire [7:0]              ifu_axi_arlen;
    wire [2:0]              ifu_axi_arsize;
    wire [1:0]              ifu_axi_arburst;
    wire                    ifu_axi_arlock;
    wire [3:0]              ifu_axi_arcache;
    wire [2:0]              ifu_axi_arprot;
    wire [3:0]              ifu_axi_arqos;
    wire                    ifu_axi_rvalid;
    wire                    ifu_axi_rready;
    wire [2:0]              ifu_axi_rid;
    wire [63:0]             ifu_axi_rdata;
    wire [1:0]              ifu_axi_rresp;
    wire                    ifu_axi_rlast;

    // DMA AXI4 master
    wire [ID_WIDTH-1:0]     dma_m_axi_awid;
    wire [ADDR_WIDTH-1:0]   dma_m_axi_awaddr;
    wire [7:0]              dma_m_axi_awlen;
    wire [2:0]              dma_m_axi_awsize;
    wire [1:0]              dma_m_axi_awburst;
    wire                    dma_m_axi_awlock;
    wire [3:0]              dma_m_axi_awcache;
    wire [2:0]              dma_m_axi_awprot;
    wire [3:0]              dma_m_axi_awqos;
    wire [3:0]              dma_m_axi_awregion;
    wire                    dma_m_axi_awvalid;
    wire                    dma_m_axi_awready;
    wire [DATA_WIDTH-1:0]   dma_m_axi_wdata;
    wire [STRB_WIDTH-1:0]   dma_m_axi_wstrb;
    wire                    dma_m_axi_wlast;
    wire                    dma_m_axi_wvalid;
    wire                    dma_m_axi_wready;
    wire [ID_WIDTH-1:0]     dma_m_axi_bid;
    wire [1:0]              dma_m_axi_bresp;
    wire                    dma_m_axi_bvalid;
    wire                    dma_m_axi_bready;
    wire [ID_WIDTH-1:0]     dma_m_axi_arid;
    wire [ADDR_WIDTH-1:0]   dma_m_axi_araddr;
    wire [7:0]              dma_m_axi_arlen;
    wire [2:0]              dma_m_axi_arsize;
    wire [1:0]              dma_m_axi_arburst;
    wire                    dma_m_axi_arlock;
    wire [3:0]              dma_m_axi_arcache;
    wire [2:0]              dma_m_axi_arprot;
    wire [3:0]              dma_m_axi_arqos;
    wire [3:0]              dma_m_axi_arregion;
    wire                    dma_m_axi_arvalid;
    wire                    dma_m_axi_arready;
    wire [ID_WIDTH-1:0]     dma_m_axi_rid;
    wire [DATA_WIDTH-1:0]   dma_m_axi_rdata;
    wire [1:0]              dma_m_axi_rresp;
    wire                    dma_m_axi_rlast;
    wire                    dma_m_axi_rvalid;
    wire                    dma_m_axi_rready;

    veer_core_wrapper #(
        .ICCM_HEX       (ICCM_HEX)
    ) core_inst (
        .clk            (clk),
        .rst_l          (core_rst_l),
        .dbg_rst_l      (core_rst_l),
        .rst_vec        (31'h7700_0000),   // 0xEE00_0000 (ICCM window)
        .nmi_int        (1'b0),
        .nmi_vec        (31'h0),
        .jtag_id        (31'h0),

        .lsu_axi_awvalid(lsu_axi_awvalid),
        .lsu_axi_awready(lsu_axi_awready),
        .lsu_axi_awid   (lsu_axi_awid),
        .lsu_axi_awaddr (lsu_axi_awaddr),
        .lsu_axi_awregion(lsu_axi_awregion),
        .lsu_axi_awlen  (lsu_axi_awlen),
        .lsu_axi_awsize (lsu_axi_awsize),
        .lsu_axi_awburst(lsu_axi_awburst),
        .lsu_axi_awlock (lsu_axi_awlock),
        .lsu_axi_awcache(lsu_axi_awcache),
        .lsu_axi_awprot (lsu_axi_awprot),
        .lsu_axi_awqos  (lsu_axi_awqos),
        .lsu_axi_wvalid (lsu_axi_wvalid),
        .lsu_axi_wready (lsu_axi_wready),
        .lsu_axi_wdata  (lsu_axi_wdata),
        .lsu_axi_wstrb  (lsu_axi_wstrb),
        .lsu_axi_wlast  (lsu_axi_wlast),
        .lsu_axi_bvalid (lsu_axi_bvalid),
        .lsu_axi_bready (lsu_axi_bready),
        .lsu_axi_bresp  (lsu_axi_bresp),
        .lsu_axi_bid    (lsu_axi_bid),
        .lsu_axi_arvalid(lsu_axi_arvalid),
        .lsu_axi_arready(lsu_axi_arready),
        .lsu_axi_arid   (lsu_axi_arid),
        .lsu_axi_araddr (lsu_axi_araddr),
        .lsu_axi_arregion(lsu_axi_arregion),
        .lsu_axi_arlen  (lsu_axi_arlen),
        .lsu_axi_arsize (lsu_axi_arsize),
        .lsu_axi_arburst(lsu_axi_arburst),
        .lsu_axi_arlock (lsu_axi_arlock),
        .lsu_axi_arcache(lsu_axi_arcache),
        .lsu_axi_arprot (lsu_axi_arprot),
        .lsu_axi_arqos  (lsu_axi_arqos),
        .lsu_axi_rvalid (lsu_axi_rvalid),
        .lsu_axi_rready (lsu_axi_rready),
        .lsu_axi_rid    (lsu_axi_rid),
        .lsu_axi_rdata  (lsu_axi_rdata),
        .lsu_axi_rresp  (lsu_axi_rresp),
        .lsu_axi_rlast  (lsu_axi_rlast),

        .ifu_axi_awvalid(ifu_axi_awvalid),
        .ifu_axi_awready(ifu_axi_awready),
        .ifu_axi_awid   (ifu_axi_awid),
        .ifu_axi_awaddr (ifu_axi_awaddr),
        .ifu_axi_awregion(ifu_axi_awregion),
        .ifu_axi_awlen  (ifu_axi_awlen),
        .ifu_axi_awsize (ifu_axi_awsize),
        .ifu_axi_awburst(ifu_axi_awburst),
        .ifu_axi_awlock (ifu_axi_awlock),
        .ifu_axi_awcache(ifu_axi_awcache),
        .ifu_axi_awprot (ifu_axi_awprot),
        .ifu_axi_awqos  (ifu_axi_awqos),
        .ifu_axi_wvalid (ifu_axi_wvalid),
        .ifu_axi_wready (ifu_axi_wready),
        .ifu_axi_wdata  (ifu_axi_wdata),
        .ifu_axi_wstrb  (ifu_axi_wstrb),
        .ifu_axi_wlast  (ifu_axi_wlast),
        .ifu_axi_bvalid (ifu_axi_bvalid),
        .ifu_axi_bready (ifu_axi_bready),
        .ifu_axi_bresp  (ifu_axi_bresp),
        .ifu_axi_bid    (ifu_axi_bid),
        .ifu_axi_arvalid(ifu_axi_arvalid),
        .ifu_axi_arready(ifu_axi_arready),
        .ifu_axi_arid   (ifu_axi_arid),
        .ifu_axi_araddr (ifu_axi_araddr),
        .ifu_axi_arregion(ifu_axi_arregion),
        .ifu_axi_arlen  (ifu_axi_arlen),
        .ifu_axi_arsize (ifu_axi_arsize),
        .ifu_axi_arburst(ifu_axi_arburst),
        .ifu_axi_arlock (ifu_axi_arlock),
        .ifu_axi_arcache(ifu_axi_arcache),
        .ifu_axi_arprot (ifu_axi_arprot),
        .ifu_axi_arqos  (ifu_axi_arqos),
        .ifu_axi_rvalid (ifu_axi_rvalid),
        .ifu_axi_rready (ifu_axi_rready),
        .ifu_axi_rid    (ifu_axi_rid),
        .ifu_axi_rdata  (ifu_axi_rdata),
        .ifu_axi_rresp  (ifu_axi_rresp),
        .ifu_axi_rlast  (ifu_axi_rlast),

        .timer_int      (timer_irq_o),
        .soft_int       (uart_irq_o),
        .extintsrc_req  (extintsrc_req),
        .core_id        (28'd0)
    );

    // ------------------------------------------------------------------
    // Interconnect slave side (3 masters: 0 = IFU, 1 = LSU, 2 = DMA)
    // ------------------------------------------------------------------
    wire [S_COUNT*ID_WIDTH-1:0]     s_axi_awid;
    wire [S_COUNT*ADDR_WIDTH-1:0]   s_axi_awaddr;
    wire [S_COUNT*8-1:0]            s_axi_awlen;
    wire [S_COUNT*3-1:0]            s_axi_awsize;
    wire [S_COUNT*2-1:0]            s_axi_awburst;
    wire [S_COUNT-1:0]              s_axi_awlock;
    wire [S_COUNT*4-1:0]            s_axi_awcache;
    wire [S_COUNT*3-1:0]            s_axi_awprot;
    wire [S_COUNT*4-1:0]            s_axi_awqos;
    wire [S_COUNT-1:0]              s_axi_awvalid;
    wire [S_COUNT-1:0]              s_axi_awready;

    wire [S_COUNT*DATA_WIDTH-1:0]   s_axi_wdata;
    wire [S_COUNT*STRB_WIDTH-1:0]   s_axi_wstrb;
    wire [S_COUNT-1:0]              s_axi_wlast;
    wire [S_COUNT-1:0]              s_axi_wvalid;
    wire [S_COUNT-1:0]              s_axi_wready;

    wire [S_COUNT*ID_WIDTH-1:0]     s_axi_bid;
    wire [S_COUNT*2-1:0]            s_axi_bresp;
    wire [S_COUNT-1:0]              s_axi_bvalid;
    wire [S_COUNT-1:0]              s_axi_bready;

    wire [S_COUNT*ID_WIDTH-1:0]     s_axi_arid;
    wire [S_COUNT*ADDR_WIDTH-1:0]   s_axi_araddr;
    wire [S_COUNT*8-1:0]            s_axi_arlen;
    wire [S_COUNT*3-1:0]            s_axi_arsize;
    wire [S_COUNT*2-1:0]            s_axi_arburst;
    wire [S_COUNT-1:0]              s_axi_arlock;
    wire [S_COUNT*4-1:0]            s_axi_arcache;
    wire [S_COUNT*3-1:0]            s_axi_arprot;
    wire [S_COUNT*4-1:0]            s_axi_arqos;
    wire [S_COUNT-1:0]              s_axi_arvalid;
    wire [S_COUNT-1:0]              s_axi_arready;

    wire [S_COUNT*ID_WIDTH-1:0]     s_axi_rid;
    wire [S_COUNT*DATA_WIDTH-1:0]   s_axi_rdata;
    wire [S_COUNT*2-1:0]            s_axi_rresp;
    wire [S_COUNT-1:0]              s_axi_rlast;
    wire [S_COUNT-1:0]              s_axi_rvalid;
    wire [S_COUNT-1:0]              s_axi_rready;

    // ---- master 0: IFU ---------------------------------------------------
    assign s_axi_awid   [ID_WIDTH-1:0]   = {{(ID_WIDTH-$bits(ifu_axi_awid)){1'b0}}, ifu_axi_awid};
    assign s_axi_awaddr [ADDR_WIDTH-1:0] = ifu_axi_awaddr;
    assign s_axi_awlen  [7:0]            = ifu_axi_awlen;
    assign s_axi_awsize [2:0]            = ifu_axi_awsize;
    assign s_axi_awburst[1:0]            = ifu_axi_awburst;
    assign s_axi_awlock [0]              = ifu_axi_awlock;
    assign s_axi_awcache[3:0]            = ifu_axi_awcache;
    assign s_axi_awprot [2:0]            = ifu_axi_awprot;
    assign s_axi_awqos  [3:0]            = ifu_axi_awqos;
    assign s_axi_awvalid[0]              = ifu_axi_awvalid;
    assign ifu_axi_awready                = s_axi_awready[0];

    assign s_axi_wdata  [DATA_WIDTH-1:0] = ifu_axi_wdata;
    assign s_axi_wstrb  [STRB_WIDTH-1:0] = ifu_axi_wstrb;
    assign s_axi_wlast  [0]              = ifu_axi_wlast;
    assign s_axi_wvalid [0]              = ifu_axi_wvalid;
    assign ifu_axi_wready                = s_axi_wready[0];

    assign ifu_axi_bid                   = s_axi_bid    [ID_WIDTH-1:0];
    assign ifu_axi_bresp                 = s_axi_bresp  [1:0];
    assign ifu_axi_bvalid                = s_axi_bvalid [0];
    assign s_axi_bready[0]               = ifu_axi_bready;

    assign s_axi_arid   [ID_WIDTH-1:0]   = {{(ID_WIDTH-$bits(ifu_axi_arid)){1'b0}}, ifu_axi_arid};
    assign s_axi_araddr [ADDR_WIDTH-1:0] = ifu_axi_araddr;
    assign s_axi_arlen  [7:0]            = ifu_axi_arlen;
    assign s_axi_arsize [2:0]            = ifu_axi_arsize;
    assign s_axi_arburst[1:0]            = ifu_axi_arburst;
    assign s_axi_arlock [0]              = ifu_axi_arlock;
    assign s_axi_arcache[3:0]            = ifu_axi_arcache;
    assign s_axi_arprot [2:0]            = ifu_axi_arprot;
    assign s_axi_arqos  [3:0]            = ifu_axi_arqos;
    assign s_axi_arvalid[0]              = ifu_axi_arvalid;
    assign ifu_axi_arready                = s_axi_arready[0];

    assign ifu_axi_rid                   = s_axi_rid    [ID_WIDTH-1:0];
    assign ifu_axi_rdata                 = s_axi_rdata  [DATA_WIDTH-1:0];
    assign ifu_axi_rresp                 = s_axi_rresp  [1:0];
    assign ifu_axi_rlast                 = s_axi_rlast  [0];
    assign ifu_axi_rvalid                = s_axi_rvalid [0];
    assign s_axi_rready[0]               = ifu_axi_rready;

    // ---- master 1: LSU ---------------------------------------------------
    assign s_axi_awid   [2*ID_WIDTH-1:ID_WIDTH]   = {{(ID_WIDTH-$bits(lsu_axi_awid)){1'b0}}, lsu_axi_awid};
    assign s_axi_awaddr [2*ADDR_WIDTH-1:ADDR_WIDTH] = lsu_axi_awaddr;
    assign s_axi_awlen  [15:8]           = lsu_axi_awlen;
    assign s_axi_awsize [5:3]            = lsu_axi_awsize;
    assign s_axi_awburst[3:2]            = lsu_axi_awburst;
    assign s_axi_awlock [1]              = lsu_axi_awlock;
    assign s_axi_awcache[7:4]            = lsu_axi_awcache;
    assign s_axi_awprot [5:3]            = lsu_axi_awprot;
    assign s_axi_awqos  [7:4]            = lsu_axi_awqos;
    assign s_axi_awvalid[1]              = lsu_axi_awvalid;
    assign lsu_axi_awready                = s_axi_awready[1];

    assign s_axi_wdata  [2*DATA_WIDTH-1:DATA_WIDTH] = lsu_axi_wdata;
    assign s_axi_wstrb  [2*STRB_WIDTH-1:STRB_WIDTH] = lsu_axi_wstrb;
    assign s_axi_wlast  [1]              = lsu_axi_wlast;
    assign s_axi_wvalid [1]              = lsu_axi_wvalid;
    assign lsu_axi_wready                = s_axi_wready[1];

    assign lsu_axi_bid                   = s_axi_bid    [2*ID_WIDTH-1:ID_WIDTH];
    assign lsu_axi_bresp                 = s_axi_bresp  [3:2];
    assign lsu_axi_bvalid                = s_axi_bvalid [1];
    assign s_axi_bready[1]               = lsu_axi_bready;

    assign s_axi_arid   [2*ID_WIDTH-1:ID_WIDTH]   = {{(ID_WIDTH-$bits(lsu_axi_arid)){1'b0}}, lsu_axi_arid};
    assign s_axi_araddr [2*ADDR_WIDTH-1:ADDR_WIDTH] = lsu_axi_araddr;
    assign s_axi_arlen  [15:8]           = lsu_axi_arlen;
    assign s_axi_arsize [5:3]            = lsu_axi_arsize;
    assign s_axi_arburst[3:2]            = lsu_axi_arburst;
    assign s_axi_arlock [1]              = lsu_axi_arlock;
    assign s_axi_arcache[7:4]            = lsu_axi_arcache;
    assign s_axi_arprot [5:3]            = lsu_axi_arprot;
    assign s_axi_arqos  [7:4]            = lsu_axi_arqos;
    assign s_axi_arvalid[1]              = lsu_axi_arvalid;
    assign lsu_axi_arready                = s_axi_arready[1];

    assign lsu_axi_rid                   = s_axi_rid    [2*ID_WIDTH-1:ID_WIDTH];
    assign lsu_axi_rdata                 = s_axi_rdata  [2*DATA_WIDTH-1:DATA_WIDTH];
    assign lsu_axi_rresp                 = s_axi_rresp  [3:2];
    assign lsu_axi_rlast                 = s_axi_rlast  [1];
    assign lsu_axi_rvalid                = s_axi_rvalid [1];
    assign s_axi_rready[1]               = lsu_axi_rready;

    // ---- master 2: DMA Master / External AXI Master (Muxed) ---------------
    assign s_axi_awid   [3*ID_WIDTH-1:2*ID_WIDTH]     = ext_master_en ? ext_m_axi_awid    : dma_m_axi_awid;
    assign s_axi_awaddr [3*ADDR_WIDTH-1:2*ADDR_WIDTH] = ext_master_en ? ext_m_axi_awaddr  : dma_m_axi_awaddr;
    assign s_axi_awlen  [23:16]                       = ext_master_en ? ext_m_axi_awlen   : dma_m_axi_awlen;
    assign s_axi_awsize [8:6]                         = ext_master_en ? ext_m_axi_awsize  : dma_m_axi_awsize;
    assign s_axi_awburst[5:4]                         = ext_master_en ? ext_m_axi_awburst : dma_m_axi_awburst;
    assign s_axi_awlock [2]                           = ext_master_en ? ext_m_axi_awlock  : dma_m_axi_awlock;
    assign s_axi_awcache[11:8]                        = ext_master_en ? ext_m_axi_awcache : dma_m_axi_awcache;
    assign s_axi_awprot [8:6]                         = ext_master_en ? ext_m_axi_awprot  : dma_m_axi_awprot;
    assign s_axi_awqos  [11:8]                        = ext_master_en ? ext_m_axi_awqos   : dma_m_axi_awqos;
    assign s_axi_awvalid[2]                           = ext_master_en ? ext_m_axi_awvalid : dma_m_axi_awvalid;
    assign dma_m_axi_awready                          = ext_master_en ? 1'b0              : s_axi_awready[2];
    assign ext_m_axi_awready                          = ext_master_en ? s_axi_awready[2]  : 1'b0;

    assign s_axi_wdata  [3*DATA_WIDTH-1:2*DATA_WIDTH] = ext_master_en ? ext_m_axi_wdata   : dma_m_axi_wdata;
    assign s_axi_wstrb  [3*STRB_WIDTH-1:2*STRB_WIDTH] = ext_master_en ? ext_m_axi_wstrb   : dma_m_axi_wstrb;
    assign s_axi_wlast  [2]                           = ext_master_en ? ext_m_axi_wlast   : dma_m_axi_wlast;
    assign s_axi_wvalid [2]                           = ext_master_en ? ext_m_axi_wvalid  : dma_m_axi_wvalid;
    assign dma_m_axi_wready                           = ext_master_en ? 1'b0              : s_axi_wready[2];
    assign ext_m_axi_wready                           = ext_master_en ? s_axi_wready[2]   : 1'b0;

    assign dma_m_axi_bid                              = s_axi_bid    [3*ID_WIDTH-1:2*ID_WIDTH];
    assign dma_m_axi_bresp                            = s_axi_bresp  [5:4];
    assign dma_m_axi_bvalid                           = ext_master_en ? 1'b0              : s_axi_bvalid [2];
    assign ext_m_axi_bid                              = s_axi_bid    [3*ID_WIDTH-1:2*ID_WIDTH];
    assign ext_m_axi_bresp                            = s_axi_bresp  [5:4];
    assign ext_m_axi_bvalid                           = ext_master_en ? s_axi_bvalid [2]  : 1'b0;
    assign s_axi_bready[2]                            = ext_master_en ? ext_m_axi_bready  : dma_m_axi_bready;

    assign s_axi_arid   [3*ID_WIDTH-1:2*ID_WIDTH]     = ext_master_en ? ext_m_axi_arid    : dma_m_axi_arid;
    assign s_axi_araddr [3*ADDR_WIDTH-1:2*ADDR_WIDTH] = ext_master_en ? ext_m_axi_araddr  : dma_m_axi_araddr;
    assign s_axi_arlen  [23:16]                       = ext_master_en ? ext_m_axi_arlen   : dma_m_axi_arlen;
    assign s_axi_arsize [8:6]                         = ext_master_en ? ext_m_axi_arsize  : dma_m_axi_arsize;
    assign s_axi_arburst[5:4]                         = ext_master_en ? ext_m_axi_arburst : dma_m_axi_arburst;
    assign s_axi_arlock [2]                           = ext_master_en ? ext_m_axi_arlock  : dma_m_axi_arlock;
    assign s_axi_arcache[11:8]                        = ext_master_en ? ext_m_axi_arcache : dma_m_axi_arcache;
    assign s_axi_arprot [8:6]                         = ext_master_en ? ext_m_axi_arprot  : dma_m_axi_arprot;
    assign s_axi_arqos  [11:8]                        = ext_master_en ? ext_m_axi_arqos   : dma_m_axi_arqos;
    assign s_axi_arvalid[2]                           = ext_master_en ? ext_m_axi_arvalid : dma_m_axi_arvalid;
    assign dma_m_axi_arready                          = ext_master_en ? 1'b0              : s_axi_arready[2];
    assign ext_m_axi_arready                          = ext_master_en ? s_axi_arready[2]  : 1'b0;

    assign dma_m_axi_rid                              = s_axi_rid    [3*ID_WIDTH-1:2*ID_WIDTH];
    assign dma_m_axi_rdata                            = s_axi_rdata  [3*DATA_WIDTH-1:2*DATA_WIDTH];
    assign dma_m_axi_rresp                            = s_axi_rresp  [5:4];
    assign dma_m_axi_rlast                            = s_axi_rlast  [2];
    assign dma_m_axi_rvalid                           = ext_master_en ? 1'b0              : s_axi_rvalid [2];
    assign ext_m_axi_rid                              = s_axi_rid    [3*ID_WIDTH-1:2*ID_WIDTH];
    assign ext_m_axi_rdata                            = s_axi_rdata  [3*DATA_WIDTH-1:2*DATA_WIDTH];
    assign ext_m_axi_rresp                            = s_axi_rresp  [5:4];
    assign ext_m_axi_rlast                            = s_axi_rlast  [2];
    assign ext_m_axi_rvalid                           = ext_master_en ? s_axi_rvalid [2]  : 1'b0;
    assign s_axi_rready[2]                            = ext_master_en ? ext_m_axi_rready  : dma_m_axi_rready;

    // ------------------------------------------------------------------
    // Interconnect master side (10 slaves)
    // ------------------------------------------------------------------
    wire [M_COUNT*ID_WIDTH-1:0]     m_axi_awid;
    wire [M_COUNT*ADDR_WIDTH-1:0]   m_axi_awaddr;
    wire [M_COUNT*8-1:0]            m_axi_awlen;
    wire [M_COUNT*3-1:0]            m_axi_awsize;
    wire [M_COUNT*2-1:0]            m_axi_awburst;
    wire [M_COUNT-1:0]              m_axi_awlock;
    wire [M_COUNT*4-1:0]            m_axi_awcache;
    wire [M_COUNT*3-1:0]            m_axi_awprot;
    wire [M_COUNT*4-1:0]            m_axi_awqos;
    wire [M_COUNT*4-1:0]            m_axi_awregion;
    wire [M_COUNT-1:0]              m_axi_awvalid;
    wire [M_COUNT-1:0]              m_axi_awready;

    wire [M_COUNT*DATA_WIDTH-1:0]   m_axi_wdata;
    wire [M_COUNT*STRB_WIDTH-1:0]   m_axi_wstrb;
    wire [M_COUNT-1:0]              m_axi_wlast;
    wire [M_COUNT-1:0]              m_axi_wvalid;
    wire [M_COUNT-1:0]              m_axi_wready;

    wire [M_COUNT*ID_WIDTH-1:0]     m_axi_bid;
    wire [M_COUNT*2-1:0]            m_axi_bresp;
    wire [M_COUNT-1:0]              m_axi_bvalid;
    wire [M_COUNT-1:0]              m_axi_bready;

    wire [M_COUNT*ID_WIDTH-1:0]     m_axi_arid;
    wire [M_COUNT*ADDR_WIDTH-1:0]   m_axi_araddr;
    wire [M_COUNT*8-1:0]            m_axi_arlen;
    wire [M_COUNT*3-1:0]            m_axi_arsize;
    wire [M_COUNT*2-1:0]            m_axi_arburst;
    wire [M_COUNT-1:0]              m_axi_arlock;
    wire [M_COUNT*4-1:0]            m_axi_arcache;
    wire [M_COUNT*3-1:0]            m_axi_arprot;
    wire [M_COUNT*4-1:0]            m_axi_arqos;
    wire [M_COUNT*4-1:0]            m_axi_arregion;
    wire [M_COUNT-1:0]              m_axi_arvalid;
    wire [M_COUNT-1:0]              m_axi_arready;

    wire [M_COUNT*ID_WIDTH-1:0]     m_axi_rid;
    wire [M_COUNT*DATA_WIDTH-1:0]   m_axi_rdata;
    wire [M_COUNT*2-1:0]            m_axi_rresp;
    wire [M_COUNT-1:0]              m_axi_rlast;
    wire [M_COUNT-1:0]              m_axi_rvalid;
    wire [M_COUNT-1:0]              m_axi_rready;

    // Per-port views used by the slaves (0..9)
    wire [ID_WIDTH-1:0]     p_awid   [0:M_COUNT-1];
    wire [ADDR_WIDTH-1:0]   p_awaddr [0:M_COUNT-1];
    wire [7:0]              p_awlen  [0:M_COUNT-1];
    wire [2:0]              p_awsize [0:M_COUNT-1];
    wire [1:0]              p_awburst[0:M_COUNT-1];
    wire                    p_awlock [0:M_COUNT-1];
    wire [3:0]              p_awcache[0:M_COUNT-1];
    wire [2:0]              p_awprot [0:M_COUNT-1];
    wire [3:0]              p_awqos  [0:M_COUNT-1];
    wire                    p_awvalid[0:M_COUNT-1];
    wire                    p_awready[0:M_COUNT-1];
    wire [DATA_WIDTH-1:0]   p_wdata  [0:M_COUNT-1];
    wire [STRB_WIDTH-1:0]   p_wstrb  [0:M_COUNT-1];
    wire                    p_wlast  [0:M_COUNT-1];
    wire                    p_wvalid [0:M_COUNT-1];
    wire                    p_wready [0:M_COUNT-1];
    wire [ID_WIDTH-1:0]     p_bid    [0:M_COUNT-1];
    wire [1:0]              p_bresp  [0:M_COUNT-1];
    wire                    p_bvalid [0:M_COUNT-1];
    wire                    p_bready [0:M_COUNT-1];
    wire [ID_WIDTH-1:0]     p_arid   [0:M_COUNT-1];
    wire [ADDR_WIDTH-1:0]   p_araddr [0:M_COUNT-1];
    wire [7:0]              p_arlen  [0:M_COUNT-1];
    wire [2:0]              p_arsize [0:M_COUNT-1];
    wire [1:0]              p_arburst[0:M_COUNT-1];
    wire                    p_arlock [0:M_COUNT-1];
    wire [3:0]              p_arcache[0:M_COUNT-1];
    wire [2:0]              p_arprot [0:M_COUNT-1];
    wire [3:0]              p_arqos  [0:M_COUNT-1];
    wire                    p_arvalid[0:M_COUNT-1];
    wire                    p_arready[0:M_COUNT-1];
    wire [ID_WIDTH-1:0]     p_rid    [0:M_COUNT-1];
    wire [DATA_WIDTH-1:0]   p_rdata  [0:M_COUNT-1];
    wire [1:0]              p_rresp  [0:M_COUNT-1];
    wire                    p_rlast  [0:M_COUNT-1];
    wire                    p_rvalid [0:M_COUNT-1];
    wire                    p_rready [0:M_COUNT-1];

    // Unpack the flattened interconnect master ports
    genvar gi;
    generate
        for (gi = 0; gi < M_COUNT; gi = gi + 1) begin : gen_unpack
            assign p_awid   [gi] = m_axi_awid   [gi*ID_WIDTH +: ID_WIDTH];
            assign p_awaddr [gi] = m_axi_awaddr [gi*ADDR_WIDTH +: ADDR_WIDTH];
            assign p_awlen  [gi] = m_axi_awlen  [gi*8 +: 8];
            assign p_awsize [gi] = m_axi_awsize [gi*3 +: 3];
            assign p_awburst[gi] = m_axi_awburst[gi*2 +: 2];
            assign p_awlock [gi] = m_axi_awlock [gi];
            assign p_awcache[gi] = m_axi_awcache[gi*4 +: 4];
            assign p_awprot [gi] = m_axi_awprot [gi*3 +: 3];
            assign p_awqos  [gi] = m_axi_awqos  [gi*4 +: 4];
            assign p_awvalid[gi] = m_axi_awvalid[gi];
            assign m_axi_awready[gi] = p_awready[gi];

            assign p_wdata  [gi] = m_axi_wdata  [gi*DATA_WIDTH +: DATA_WIDTH];
            assign p_wstrb  [gi] = m_axi_wstrb  [gi*STRB_WIDTH +: STRB_WIDTH];
            assign p_wlast  [gi] = m_axi_wlast  [gi];
            assign p_wvalid [gi] = m_axi_wvalid [gi];
            assign m_axi_wready[gi] = p_wready[gi];

            assign m_axi_bid    [gi*ID_WIDTH +: ID_WIDTH] = p_bid    [gi];
            assign m_axi_bresp  [gi*2 +: 2]               = p_bresp  [gi];
            assign m_axi_bvalid [gi]                      = p_bvalid [gi];
            assign p_bready    [gi] = m_axi_bready[gi];

            assign p_arid   [gi] = m_axi_arid   [gi*ID_WIDTH +: ID_WIDTH];
            assign p_araddr [gi] = m_axi_araddr [gi*ADDR_WIDTH +: ADDR_WIDTH];
            assign p_arlen  [gi] = m_axi_arlen  [gi*8 +: 8];
            assign p_arsize [gi] = m_axi_arsize [gi*3 +: 3];
            assign p_arburst[gi] = m_axi_arburst[gi*2 +: 2];
            assign p_arlock [gi] = m_axi_arlock [gi];
            assign p_arcache[gi] = m_axi_arcache[gi*4 +: 4];
            assign p_arprot [gi] = m_axi_arprot [gi*3 +: 3];
            assign p_arqos  [gi] = m_axi_arqos  [gi*4 +: 4];
            assign p_arvalid[gi] = m_axi_arvalid[gi];
            assign m_axi_arready[gi] = p_arready[gi];

            assign m_axi_rid    [gi*ID_WIDTH +: ID_WIDTH] = p_rid    [gi];
            assign m_axi_rdata  [gi*DATA_WIDTH +: DATA_WIDTH] = p_rdata[gi];
            assign m_axi_rresp  [gi*2 +: 2]               = p_rresp  [gi];
            assign m_axi_rlast  [gi]                      = p_rlast  [gi];
            assign m_axi_rvalid [gi]                      = p_rvalid [gi];
            assign p_rready    [gi] = m_axi_rready[gi];
        end
    endgenerate

    // ------------------------------------------------------------------
    // AXI interconnect (3 masters x 10 slaves)
    // ------------------------------------------------------------------
    axi_interconnect #(
        .S_COUNT         (S_COUNT),
        .M_COUNT         (M_COUNT),
        .DATA_WIDTH      (DATA_WIDTH),
        .ADDR_WIDTH      (ADDR_WIDTH),
        .STRB_WIDTH      (STRB_WIDTH),
        .ID_WIDTH        (ID_WIDTH),
        .FORWARD_ID      (0),
        .M_REGIONS       (1),
        // LSB = port 0 (IMEM), MSB = port 9 (POOLING)
        .M_BASE_ADDR     ({
            32'hA000_0000,  // m09 POOLING
            32'h9000_0000,  // m08 RELU
            32'h7000_0000,  // m07 7-SEG
            32'h6000_0000,  // m06 GPIO
            32'h5000_0000,  // m05 TIMER
            32'h4000_0000,  // m04 UART
            32'h3000_0000,  // m03 CNN
            32'h2000_0000,  // m02 DMA
            32'h1000_0000,  // m01 DMEM
            32'h8000_0000   // m00 IMEM
        }),
        .M_ADDR_WIDTH    ({10{32'd28}}),
        // S_COUNT = 3 -> every slave is reachable from all 3 masters
        .M_CONNECT_READ  ({10{3'b111}}),
        .M_CONNECT_WRITE ({10{3'b111}})
    ) ic_inst (
        .clk (clk),
        .rst (rst),

        // Slave interfaces (s00 = IFU, s01 = LSU, s02 = DMA)
        .s_axi_awid   (s_axi_awid),
        .s_axi_awaddr (s_axi_awaddr),
        .s_axi_awlen  (s_axi_awlen),
        .s_axi_awsize (s_axi_awsize),
        .s_axi_awburst(s_axi_awburst),
        .s_axi_awlock (s_axi_awlock),
        .s_axi_awcache(s_axi_awcache),
        .s_axi_awprot (s_axi_awprot),
        .s_axi_awqos  (s_axi_awqos),
        .s_axi_awuser ({S_COUNT{1'b0}}),
        .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready),
        .s_axi_wdata  (s_axi_wdata),
        .s_axi_wstrb  (s_axi_wstrb),
        .s_axi_wlast  (s_axi_wlast),
        .s_axi_wuser  ({S_COUNT{1'b0}}),
        .s_axi_wvalid (s_axi_wvalid),
        .s_axi_wready (s_axi_wready),
        .s_axi_bid    (s_axi_bid),
        .s_axi_bresp  (s_axi_bresp),
        .s_axi_buser (),
        .s_axi_bvalid (s_axi_bvalid),
        .s_axi_bready (s_axi_bready),
        .s_axi_arid   (s_axi_arid),
        .s_axi_araddr (s_axi_araddr),
        .s_axi_arlen  (s_axi_arlen),
        .s_axi_arsize (s_axi_arsize),
        .s_axi_arburst(s_axi_arburst),
        .s_axi_arlock (s_axi_arlock),
        .s_axi_arcache(s_axi_arcache),
        .s_axi_arprot (s_axi_arprot),
        .s_axi_arqos  (s_axi_arqos),
        .s_axi_aruser ({S_COUNT{1'b0}}),
        .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready),
        .s_axi_rid    (s_axi_rid),
        .s_axi_rdata  (s_axi_rdata),
        .s_axi_rresp  (s_axi_rresp),
        .s_axi_rlast  (s_axi_rlast),
        .s_axi_ruser (),
        .s_axi_rvalid (s_axi_rvalid),
        .s_axi_rready (s_axi_rready),

        // Master interfaces (m00..m09) - to slaves
        .m_axi_awid   (m_axi_awid),
        .m_axi_awaddr (m_axi_awaddr),
        .m_axi_awlen  (m_axi_awlen),
        .m_axi_awsize (m_axi_awsize),
        .m_axi_awburst(m_axi_awburst),
        .m_axi_awlock (m_axi_awlock),
        .m_axi_awcache(m_axi_awcache),
        .m_axi_awprot (m_axi_awprot),
        .m_axi_awqos  (m_axi_awqos),
        .m_axi_awregion (),
        .m_axi_awuser (),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata  (m_axi_wdata),
        .m_axi_wstrb  (m_axi_wstrb),
        .m_axi_wlast  (m_axi_wlast),
        .m_axi_wuser (),
        .m_axi_wvalid (m_axi_wvalid),
        .m_axi_wready (m_axi_wready),
        .m_axi_bid    (m_axi_bid),
        .m_axi_bresp  (m_axi_bresp),
        .m_axi_buser  ({M_COUNT{1'b0}}),
        .m_axi_bvalid (m_axi_bvalid),
        .m_axi_bready (m_axi_bready),
        .m_axi_arid   (m_axi_arid),
        .m_axi_araddr (m_axi_araddr),
        .m_axi_arlen  (m_axi_arlen),
        .m_axi_arsize (m_axi_arsize),
        .m_axi_arburst(m_axi_arburst),
        .m_axi_arlock (m_axi_arlock),
        .m_axi_arcache(m_axi_arcache),
        .m_axi_arprot (m_axi_arprot),
        .m_axi_arqos  (m_axi_arqos),
        .m_axi_arregion (),
        .m_axi_aruser (),
        .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid    (m_axi_rid),
        .m_axi_rdata  (m_axi_rdata),
        .m_axi_rresp  (m_axi_rresp),
        .m_axi_rlast  (m_axi_rlast),
        .m_axi_ruser  ({M_COUNT{1'b0}}),
        .m_axi_rvalid (m_axi_rvalid),
        .m_axi_rready (m_axi_rready)
    );

    // ------------------------------------------------------------------
    // m00 = IMEM (0x8000_0000) - holds the firmware
    // ------------------------------------------------------------------
    axi_mem #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .BASE_ADDR (32'h8000_0000),
        .MEM_SIZE  (32'h0000_1000),
        .MEM_HEX   (IMEM_HEX)
    ) imem (
        .clk   (clk),  .rst (rst),
        .awid  (p_awid[0]),   .awaddr (p_awaddr[0]),
        .awlen (p_awlen[0]),  .awsize (p_awsize[0]),
        .awburst(p_awburst[0]), .awvalid(p_awvalid[0]), .awready(p_awready[0]),
        .wdata (p_wdata[0]),  .wstrb (p_wstrb[0]),
        .wlast (p_wlast[0]),  .wvalid(p_wvalid[0]),   .wready(p_wready[0]),
        .bid   (p_bid[0]),    .bresp (p_bresp[0]),
        .bvalid(p_bvalid[0]), .bready(p_bready[0]),
        .arid  (p_arid[0]),   .araddr (p_araddr[0]),
        .arlen (p_arlen[0]),  .arsize (p_arsize[0]),
        .arburst(p_arburst[0]), .arvalid(p_arvalid[0]), .arready(p_arready[0]),
        .rid   (p_rid[0]),    .rdata (p_rdata[0]),
        .rresp (p_rresp[0]),  .rlast (p_rlast[0]),
        .rvalid(p_rvalid[0]), .rready(p_rready[0])
    );

    // ------------------------------------------------------------------
    // m01 = DMEM (0x1000_0000)
    // ------------------------------------------------------------------
    axi_mem #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .ID_WIDTH  (ID_WIDTH),
        .BASE_ADDR (32'h1000_0000),
        .MEM_SIZE  (32'h0000_4000)    // 16 KiB DMEM for images, weights, and features
    ) dmem (
        .clk   (clk),  .rst (rst),
        .awid  (p_awid[1]),   .awaddr (p_awaddr[1]),
        .awlen (p_awlen[1]),  .awsize (p_awsize[1]),
        .awburst(p_awburst[1]), .awvalid(p_awvalid[1]), .awready(p_awready[1]),
        .wdata (p_wdata[1]),  .wstrb (p_wstrb[1]),
        .wlast (p_wlast[1]),  .wvalid(p_wvalid[1]),   .wready(p_wready[1]),
        .bid   (p_bid[1]),    .bresp (p_bresp[1]),
        .bvalid(p_bvalid[1]), .bready(p_bready[1]),
        .arid  (p_arid[1]),   .araddr (p_araddr[1]),
        .arlen (p_arlen[1]),  .arsize (p_arsize[1]),
        .arburst(p_arburst[1]), .arvalid(p_arvalid[1]), .arready(p_arready[1]),
        .rid   (p_rid[1]),    .rdata (p_rdata[1]),
        .rresp (p_rresp[1]),  .rlast (p_rlast[1]),
        .rvalid(p_rvalid[1]), .rready(p_rready[1])
    );

    // ------------------------------------------------------------------
    // m02 = DMA Controller (0x2000_0000)
    // ------------------------------------------------------------------
    axi_dma_ctrl #(
        .DATA_WIDTH       (DATA_WIDTH),
        .ADDR_WIDTH       (ADDR_WIDTH),
        .ID_WIDTH         (ID_WIDTH),
        .STRB_WIDTH       (STRB_WIDTH),
        .BASE_ADDR        (32'h2000_0000)
    ) dma_inst (
        .clk              (clk),
        .rst              (rst),
        // Slave port (CPU programming)
        .s_axi_awid       (p_awid[2]),
        .s_axi_awaddr     (p_awaddr[2]),
        .s_axi_awlen      (p_awlen[2]),
        .s_axi_awsize     (p_awsize[2]),
        .s_axi_awburst    (p_awburst[2]),
        .s_axi_awlock     (p_awlock[2]),
        .s_axi_awcache    (p_awcache[2]),
        .s_axi_awprot     (p_awprot[2]),
        .s_axi_awqos      (p_awqos[2]),
        .s_axi_awregion   (4'b0),
        .s_axi_awvalid    (p_awvalid[2]),
        .s_axi_awready    (p_awready[2]),
        .s_axi_wdata      (p_wdata[2]),
        .s_axi_wstrb      (p_wstrb[2]),
        .s_axi_wlast      (p_wlast[2]),
        .s_axi_wvalid     (p_wvalid[2]),
        .s_axi_wready     (p_wready[2]),
        .s_axi_bid        (p_bid[2]),
        .s_axi_bresp      (p_bresp[2]),
        .s_axi_bvalid     (p_bvalid[2]),
        .s_axi_bready     (p_bready[2]),
        .s_axi_arid       (p_arid[2]),
        .s_axi_araddr     (p_araddr[2]),
        .s_axi_arlen      (p_arlen[2]),
        .s_axi_arsize     (p_arsize[2]),
        .s_axi_arburst    (p_arburst[2]),
        .s_axi_arlock     (p_arlock[2]),
        .s_axi_arcache    (p_arcache[2]),
        .s_axi_arprot     (p_arprot[2]),
        .s_axi_arqos      (p_arqos[2]),
        .s_axi_arregion   (4'b0),
        .s_axi_arvalid    (p_arvalid[2]),
        .s_axi_arready    (p_arready[2]),
        .s_axi_rid        (p_rid[2]),
        .s_axi_rdata      (p_rdata[2]),
        .s_axi_rresp      (p_rresp[2]),
        .s_axi_rlast      (p_rlast[2]),
        .s_axi_rvalid     (p_rvalid[2]),
        .s_axi_rready     (p_rready[2]),
        // Master port (high-throughput data movement)
        .m_axi_awid       (dma_m_axi_awid),
        .m_axi_awaddr     (dma_m_axi_awaddr),
        .m_axi_awlen      (dma_m_axi_awlen),
        .m_axi_awsize     (dma_m_axi_awsize),
        .m_axi_awburst    (dma_m_axi_awburst),
        .m_axi_awlock     (dma_m_axi_awlock),
        .m_axi_awcache    (dma_m_axi_awcache),
        .m_axi_awprot     (dma_m_axi_awprot),
        .m_axi_awqos      (dma_m_axi_awqos),
        .m_axi_awregion   (dma_m_axi_awregion),
        .m_axi_awvalid    (dma_m_axi_awvalid),
        .m_axi_awready    (dma_m_axi_awready),
        .m_axi_wdata      (dma_m_axi_wdata),
        .m_axi_wstrb      (dma_m_axi_wstrb),
        .m_axi_wlast      (dma_m_axi_wlast),
        .m_axi_wvalid     (dma_m_axi_wvalid),
        .m_axi_wready     (dma_m_axi_wready),
        .m_axi_bid        (dma_m_axi_bid),
        .m_axi_bresp      (dma_m_axi_bresp),
        .m_axi_bvalid     (dma_m_axi_bvalid),
        .m_axi_bready     (dma_m_axi_bready),
        .m_axi_arid       (dma_m_axi_arid),
        .m_axi_araddr     (dma_m_axi_araddr),
        .m_axi_arlen      (dma_m_axi_arlen),
        .m_axi_arsize     (dma_m_axi_arsize),
        .m_axi_arburst    (dma_m_axi_arburst),
        .m_axi_arlock     (dma_m_axi_arlock),
        .m_axi_arcache    (dma_m_axi_arcache),
        .m_axi_arprot     (dma_m_axi_arprot),
        .m_axi_arqos      (dma_m_axi_arqos),
        .m_axi_arregion   (dma_m_axi_arregion),
        .m_axi_arvalid    (dma_m_axi_arvalid),
        .m_axi_arready    (dma_m_axi_arready),
        .m_axi_rid        (dma_m_axi_rid),
        .m_axi_rdata      (dma_m_axi_rdata),
        .m_axi_rresp      (dma_m_axi_rresp),
        .m_axi_rlast      (dma_m_axi_rlast),
        .m_axi_rvalid     (dma_m_axi_rvalid),
        .m_axi_rready     (dma_m_axi_rready),
        .dma_irq_o        (dma_irq_o)
    );

    // ------------------------------------------------------------------
    // m03 = CNN Convolution Accelerator (0x3000_0000)
    // ------------------------------------------------------------------
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
        .clk              (clk),
        .rst              (rst),
        .s_axi_awid       (p_awid[3]),
        .s_axi_awaddr     (p_awaddr[3]),
        .s_axi_awlen      (p_awlen[3]),
        .s_axi_awsize     (p_awsize[3]),
        .s_axi_awburst    (p_awburst[3]),
        .s_axi_awlock     (p_awlock[3]),
        .s_axi_awcache    (p_awcache[3]),
        .s_axi_awprot     (p_awprot[3]),
        .s_axi_awqos      (p_awqos[3]),
        .s_axi_awregion   (4'b0),
        .s_axi_awvalid    (p_awvalid[3]),
        .s_axi_awready    (p_awready[3]),
        .s_axi_wdata      (p_wdata[3]),
        .s_axi_wstrb      (p_wstrb[3]),
        .s_axi_wlast      (p_wlast[3]),
        .s_axi_wvalid     (p_wvalid[3]),
        .s_axi_wready     (p_wready[3]),
        .s_axi_bid        (p_bid[3]),
        .s_axi_bresp      (p_bresp[3]),
        .s_axi_bvalid     (p_bvalid[3]),
        .s_axi_bready     (p_bready[3]),
        .s_axi_arid       (p_arid[3]),
        .s_axi_araddr     (p_araddr[3]),
        .s_axi_arlen      (p_arlen[3]),
        .s_axi_arsize     (p_arsize[3]),
        .s_axi_arburst    (p_arburst[3]),
        .s_axi_arlock     (p_arlock[3]),
        .s_axi_arcache    (p_arcache[3]),
        .s_axi_arprot     (p_arprot[3]),
        .s_axi_arqos      (p_arqos[3]),
        .s_axi_arregion   (4'b0),
        .s_axi_arvalid    (p_arvalid[3]),
        .s_axi_arready    (p_arready[3]),
        .s_axi_rid        (p_rid[3]),
        .s_axi_rdata      (p_rdata[3]),
        .s_axi_rresp      (p_rresp[3]),
        .s_axi_rlast      (p_rlast[3]),
        .s_axi_rvalid     (p_rvalid[3]),
        .s_axi_rready     (p_rready[3]),
        .cnn_irq_o        (cnn_irq_o)
    );

    // ------------------------------------------------------------------
    // m04 = UART (0x4000_0000) through the bridge
    // ------------------------------------------------------------------
    axi_uart_bridge #(
        .IC_DATA_WIDTH (DATA_WIDTH),
        .IC_ADDR_WIDTH (ADDR_WIDTH),
        .IC_ID_WIDTH   (ID_WIDTH),
        // CPU64: 8-byte strided register window for 64-bit AXI masters
        .MAP_MODE      (1)
    ) uart_bridge (
        .clk          (clk),
        .rst          (rst),
        .s_axi_awid   (p_awid[4]),   .s_axi_awaddr (p_awaddr[4]),
        .s_axi_awlen  (p_awlen[4]),  .s_axi_awsize (p_awsize[4]),
        .s_axi_awburst(p_awburst[4]),.s_axi_awlock(p_awlock[4]),
        .s_axi_awcache(p_awcache[4]),.s_axi_awprot(p_awprot[4]),
        .s_axi_awqos  (p_awqos[4]),  .s_axi_awregion(4'b0),
        .s_axi_awvalid(p_awvalid[4]),.s_axi_awready(p_awready[4]),
        .s_axi_wdata  (p_wdata[4]),  .s_axi_wstrb (p_wstrb[4]),
        .s_axi_wlast  (p_wlast[4]),  .s_axi_wvalid(p_wvalid[4]),
        .s_axi_wready (p_wready[4]),
        .s_axi_bid    (p_bid[4]),    .s_axi_bresp (p_bresp[4]),
        .s_axi_bvalid (p_bvalid[4]), .s_axi_bready(p_bready[4]),
        .s_axi_arid   (p_arid[4]),   .s_axi_araddr(p_araddr[4]),
        .s_axi_arlen  (p_arlen[4]),  .s_axi_arsize(p_arsize[4]),
        .s_axi_arburst(p_arburst[4]),.s_axi_arlock(p_arlock[4]),
        .s_axi_arcache(p_arcache[4]),.s_axi_arprot(p_arprot[4]),
        .s_axi_arqos  (p_arqos[4]),  .s_axi_arregion(4'b0),
        .s_axi_arvalid(p_arvalid[4]),.s_axi_arready(p_arready[4]),
        .s_axi_rid    (p_rid[4]),    .s_axi_rdata (p_rdata[4]),
        .s_axi_rresp  (p_rresp[4]),  .s_axi_rlast (p_rlast[4]),
        .s_axi_rvalid (p_rvalid[4]), .s_axi_rready(p_rready[4]),
        .uart_tx_o    (uart_tx_o),
        .uart_rx_i    (uart_rx_i),
        .uart_irq_o   (uart_irq_o)
    );

    // ------------------------------------------------------------------
    // m05 = Timer (0x5000_0000)
    // ------------------------------------------------------------------
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
        .s_axi_awid          (p_awid[5]),
        .s_axi_awaddr        (p_awaddr[5]),
        .s_axi_awlen         (p_awlen[5]),
        .s_axi_awsize        (p_awsize[5]),
        .s_axi_awburst       (p_awburst[5]),
        .s_axi_awlock        (p_awlock[5]),
        .s_axi_awcache       (p_awcache[5]),
        .s_axi_awprot        (p_awprot[5]),
        .s_axi_awqos         (p_awqos[5]),
        .s_axi_awregion      (4'b0),
        .s_axi_awvalid       (p_awvalid[5]),
        .s_axi_awready       (p_awready[5]),
        .s_axi_wdata         (p_wdata[5]),
        .s_axi_wstrb         (p_wstrb[5]),
        .s_axi_wlast         (p_wlast[5]),
        .s_axi_wvalid        (p_wvalid[5]),
        .s_axi_wready        (p_wready[5]),
        .s_axi_bid           (p_bid[5]),
        .s_axi_bresp         (p_bresp[5]),
        .s_axi_bvalid        (p_bvalid[5]),
        .s_axi_bready        (p_bready[5]),
        .s_axi_arid          (p_arid[5]),
        .s_axi_araddr        (p_araddr[5]),
        .s_axi_arlen         (p_arlen[5]),
        .s_axi_arsize        (p_arsize[5]),
        .s_axi_arburst       (p_arburst[5]),
        .s_axi_arlock        (p_arlock[5]),
        .s_axi_arcache       (p_arcache[5]),
        .s_axi_arprot        (p_arprot[5]),
        .s_axi_arqos         (p_arqos[5]),
        .s_axi_arregion      (4'b0),
        .s_axi_arvalid       (p_arvalid[5]),
        .s_axi_arready       (p_arready[5]),
        .s_axi_rid           (p_rid[5]),
        .s_axi_rdata         (p_rdata[5]),
        .s_axi_rresp         (p_rresp[5]),
        .s_axi_rlast         (p_rlast[5]),
        .s_axi_rvalid        (p_rvalid[5]),
        .s_axi_rready        (p_rready[5]),
        .timer_irq_o         (timer_irq_o)
    );

    // ------------------------------------------------------------------
    // m06 = GPIO (0x6000_0000)
    // ------------------------------------------------------------------
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
        .s_axi_awid         (p_awid[6]),
        .s_axi_awaddr       (p_awaddr[6]),
        .s_axi_awlen        (p_awlen[6]),
        .s_axi_awsize       (p_awsize[6]),
        .s_axi_awburst      (p_awburst[6]),
        .s_axi_awlock       (p_awlock[6]),
        .s_axi_awcache      (p_awcache[6]),
        .s_axi_awprot       (p_awprot[6]),
        .s_axi_awqos        (p_awqos[6]),
        .s_axi_awregion     (4'b0),
        .s_axi_awvalid      (p_awvalid[6]),
        .s_axi_awready      (p_awready[6]),
        .s_axi_wdata        (p_wdata[6]),
        .s_axi_wstrb        (p_wstrb[6]),
        .s_axi_wlast        (p_wlast[6]),
        .s_axi_wvalid       (p_wvalid[6]),
        .s_axi_wready       (p_wready[6]),
        .s_axi_bid          (p_bid[6]),
        .s_axi_bresp        (p_bresp[6]),
        .s_axi_bvalid       (p_bvalid[6]),
        .s_axi_bready       (p_bready[6]),
        .s_axi_arid         (p_arid[6]),
        .s_axi_araddr       (p_araddr[6]),
        .s_axi_arlen        (p_arlen[6]),
        .s_axi_arsize       (p_arsize[6]),
        .s_axi_arburst      (p_arburst[6]),
        .s_axi_arlock       (p_arlock[6]),
        .s_axi_arcache      (p_arcache[6]),
        .s_axi_arprot       (p_arprot[6]),
        .s_axi_arqos        (p_arqos[6]),
        .s_axi_arregion     (4'b0),
        .s_axi_arvalid      (p_arvalid[6]),
        .s_axi_arready      (p_arready[6]),
        .s_axi_rid          (p_rid[6]),
        .s_axi_rdata        (p_rdata[6]),
        .s_axi_rresp        (p_rresp[6]),
        .s_axi_rlast        (p_rlast[6]),
        .s_axi_rvalid       (p_rvalid[6]),
        .s_axi_rready       (p_rready[6]),
        .gpio_i             (gpio_pads_i),
        .gpio_o             (gpio_pads_o),
        .gpio_oen           (gpio_pads_oen)
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
        .s_axi_awid          (p_awid[7]),
        .s_axi_awaddr        (p_awaddr[7]),
        .s_axi_awlen         (p_awlen[7]),
        .s_axi_awsize        (p_awsize[7]),
        .s_axi_awburst       (p_awburst[7]),
        .s_axi_awlock        (p_awlock[7]),
        .s_axi_awcache       (p_awcache[7]),
        .s_axi_awprot        (p_awprot[7]),
        .s_axi_awqos         (p_awqos[7]),
        .s_axi_awregion      (4'b0),
        .s_axi_awvalid       (p_awvalid[7]),
        .s_axi_awready       (p_awready[7]),
        .s_axi_wdata         (p_wdata[7]),
        .s_axi_wstrb         (p_wstrb[7]),
        .s_axi_wlast         (p_wlast[7]),
        .s_axi_wvalid        (p_wvalid[7]),
        .s_axi_wready        (p_wready[7]),
        .s_axi_bid           (p_bid[7]),
        .s_axi_bresp         (p_bresp[7]),
        .s_axi_bvalid        (p_bvalid[7]),
        .s_axi_bready        (p_bready[7]),
        .s_axi_arid          (p_arid[7]),
        .s_axi_araddr        (p_araddr[7]),
        .s_axi_arlen         (p_arlen[7]),
        .s_axi_arsize        (p_arsize[7]),
        .s_axi_arburst       (p_arburst[7]),
        .s_axi_arlock        (p_arlock[7]),
        .s_axi_arcache       (p_arcache[7]),
        .s_axi_arprot        (p_arprot[7]),
        .s_axi_arqos         (p_arqos[7]),
        .s_axi_arregion      (4'b0),
        .s_axi_arvalid       (p_arvalid[7]),
        .s_axi_arready       (p_arready[7]),
        .s_axi_rid           (p_rid[7]),
        .s_axi_rdata         (p_rdata[7]),
        .s_axi_rresp         (p_rresp[7]),
        .s_axi_rlast         (p_rlast[7]),
        .s_axi_rvalid        (p_rvalid[7]),
        .s_axi_rready        (p_rready[7]),
        .seg_o               (seven_seg_seg_o),
        .dp_o                (seven_seg_dp_o),
        .an_o                (seven_seg_an_o)
    );

    // ------------------------------------------------------------------
    // m08 = ReLU Activation Unit (0x9000_0000)
    // ------------------------------------------------------------------
    axi_relu #(
        .DATA_WIDTH          (32),
        .AXI_DATA_WIDTH      (DATA_WIDTH),
        .AXI_ADDR_WIDTH      (ADDR_WIDTH),
        .AXI_ID_WIDTH        (ID_WIDTH),
        .AXI_STRB_WIDTH      (STRB_WIDTH),
        .REG_ADDR_WIDTH      (12),
        .BASE_ADDR           (32'h9000_0000)
    ) relu_inst (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (p_awid[8]),
        .s_axi_awaddr        (p_awaddr[8]),
        .s_axi_awlen         (p_awlen[8]),
        .s_axi_awsize        (p_awsize[8]),
        .s_axi_awburst       (p_awburst[8]),
        .s_axi_awlock        (p_awlock[8]),
        .s_axi_awcache       (p_awcache[8]),
        .s_axi_awprot        (p_awprot[8]),
        .s_axi_awqos         (p_awqos[8]),
        .s_axi_awregion      (4'b0),
        .s_axi_awvalid       (p_awvalid[8]),
        .s_axi_awready       (p_awready[8]),
        .s_axi_wdata         (p_wdata[8]),
        .s_axi_wstrb         (p_wstrb[8]),
        .s_axi_wlast         (p_wlast[8]),
        .s_axi_wvalid        (p_wvalid[8]),
        .s_axi_wready        (p_wready[8]),
        .s_axi_bid           (p_bid[8]),
        .s_axi_bresp         (p_bresp[8]),
        .s_axi_bvalid        (p_bvalid[8]),
        .s_axi_bready        (p_bready[8]),
        .s_axi_arid          (p_arid[8]),
        .s_axi_araddr        (p_araddr[8]),
        .s_axi_arlen         (p_arlen[8]),
        .s_axi_arsize        (p_arsize[8]),
        .s_axi_arburst       (p_arburst[8]),
        .s_axi_arlock        (p_arlock[8]),
        .s_axi_arcache       (p_arcache[8]),
        .s_axi_arprot        (p_arprot[8]),
        .s_axi_arqos         (p_arqos[8]),
        .s_axi_arregion      (4'b0),
        .s_axi_arvalid       (p_arvalid[8]),
        .s_axi_arready       (p_arready[8]),
        .s_axi_rid           (p_rid[8]),
        .s_axi_rdata         (p_rdata[8]),
        .s_axi_rresp         (p_rresp[8]),
        .s_axi_rlast         (p_rlast[8]),
        .s_axi_rvalid        (p_rvalid[8]),
        .s_axi_rready        (p_rready[8])
    );

    // ------------------------------------------------------------------
    // m09 = 2x2 Spatial Pooling Engine (0xA000_0000)
    // ------------------------------------------------------------------
    axi_pooling #(
        .AXI_DATA_WIDTH      (DATA_WIDTH),
        .AXI_ADDR_WIDTH      (ADDR_WIDTH),
        .AXI_ID_WIDTH        (ID_WIDTH),
        .AXI_STRB_WIDTH      (STRB_WIDTH),
        .REG_ADDR_WIDTH      (12),
        .BASE_ADDR           (32'hA000_0000)
    ) pooling_inst (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (p_awid[9]),
        .s_axi_awaddr        (p_awaddr[9]),
        .s_axi_awlen         (p_awlen[9]),
        .s_axi_awsize        (p_awsize[9]),
        .s_axi_awburst       (p_awburst[9]),
        .s_axi_awlock        (p_awlock[9]),
        .s_axi_awcache       (p_awcache[9]),
        .s_axi_awprot        (p_awprot[9]),
        .s_axi_awqos         (p_awqos[9]),
        .s_axi_awregion      (4'b0),
        .s_axi_awvalid       (p_awvalid[9]),
        .s_axi_awready       (p_awready[9]),
        .s_axi_wdata         (p_wdata[9]),
        .s_axi_wstrb         (p_wstrb[9]),
        .s_axi_wlast         (p_wlast[9]),
        .s_axi_wvalid        (p_wvalid[9]),
        .s_axi_wready        (p_wready[9]),
        .s_axi_bid           (p_bid[9]),
        .s_axi_bresp         (p_bresp[9]),
        .s_axi_bvalid        (p_bvalid[9]),
        .s_axi_bready        (p_bready[9]),
        .s_axi_arid          (p_arid[9]),
        .s_axi_araddr        (p_araddr[9]),
        .s_axi_arlen         (p_arlen[9]),
        .s_axi_arsize        (p_arsize[9]),
        .s_axi_arburst       (p_arburst[9]),
        .s_axi_arlock        (p_arlock[9]),
        .s_axi_arcache       (p_arcache[9]),
        .s_axi_arprot        (p_arprot[9]),
        .s_axi_arqos         (p_arqos[9]),
        .s_axi_arregion      (4'b0),
        .s_axi_arvalid       (p_arvalid[9]),
        .s_axi_arready       (p_arready[9]),
        .s_axi_rid           (p_rid[9]),
        .s_axi_rdata         (p_rdata[9]),
        .s_axi_rresp         (p_rresp[9]),
        .s_axi_rlast         (p_rlast[9]),
        .s_axi_rvalid        (p_rvalid[9]),
        .s_axi_rready        (p_rready[9])
    );

endmodule

`default_nettype wire
