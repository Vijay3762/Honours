// ============================================================================
// File    : soc_top_veer.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : SoC top level with the real VeeR EL2 processor core.
//
// Topology:
//   VeeR EL2 IFU (AXI4 master 0) --\
//                                   +--> axi_interconnect (2 masters x 8 slaves)
//   VeeR EL2 LSU (AXI4 master 1) --/            |
//                                            |-- m00  0x8000_0000  axi_mem  (IMEM, firmware)
//                                            |-- m01  0x1000_0000  axi_mem  (DMEM)
//                                            |-- m02  0x2000_0000  dummy    (DMA regs)
//                                            |-- m03  0x3000_0000  dummy    (CNN regs)
//                                            |-- m04  0x4000_0000  axi_uart_bridge -> axi_uart_top
//                                            |-- m05  0x5000_0000  dummy    (timer)
//                                            |-- m06  0x6000_0000  dummy    (GPIO)
//                                            |-- m07  0x7000_0000  dummy    (7-seg)
//
// The core boots from the reset vector 0x8000_0000 which is the IMEM window,
// so the firmware hex file is pre-loaded into the IMEM axi_mem model.
//
// The UART is reached through the bridge in CPU64 mode (8-byte strided
// register window) because the LSU issues 64-bit beats with 8-byte aligned
// addresses - see rtl/axi_uart_bridge.v for the details.
//
// UART tx is looped back to rx internally for self-contained simulation.
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
    output wire                    uart_irq_o
);

    localparam integer S_COUNT = 2;   // IFU + LSU
    localparam integer M_COUNT = 8;   // peripherals

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

        .timer_int      (1'b0),
        .soft_int       (uart_irq_o),      // UART interrupt -> software IRQ
        .extintsrc_req  ('0),
        .core_id        (28'd0)
    );

    // ------------------------------------------------------------------
    // Interconnect slave side (2 masters: 0 = IFU, 1 = LSU)
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

    // B channel: the interconnect drives the response back to the master
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

    // R channel: the interconnect drives the response back to the master
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

    // B channel: the interconnect drives the response back to the master
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

    // R channel: the interconnect drives the response back to the master
    assign lsu_axi_rid                   = s_axi_rid    [2*ID_WIDTH-1:ID_WIDTH];
    assign lsu_axi_rdata                 = s_axi_rdata  [2*DATA_WIDTH-1:DATA_WIDTH];
    assign lsu_axi_rresp                 = s_axi_rresp  [3:2];
    assign lsu_axi_rlast                 = s_axi_rlast  [1];
    assign lsu_axi_rvalid                = s_axi_rvalid [1];
    assign s_axi_rready[1]               = lsu_axi_rready;

    // ------------------------------------------------------------------
    // Interconnect master side (8 slaves)
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

    // Per-port views used by the slaves
    wire [ID_WIDTH-1:0]     p_awid   [0:7];
    wire [ADDR_WIDTH-1:0]   p_awaddr [0:7];
    wire [7:0]              p_awlen  [0:7];
    wire [2:0]              p_awsize [0:7];
    wire [1:0]              p_awburst[0:7];
    wire                    p_awlock [0:7];
    wire [3:0]              p_awcache[0:7];
    wire [2:0]              p_awprot [0:7];
    wire [3:0]              p_awqos  [0:7];
    wire                    p_awvalid[0:7];
    wire                    p_awready[0:7];
    wire [DATA_WIDTH-1:0]   p_wdata  [0:7];
    wire [STRB_WIDTH-1:0]   p_wstrb  [0:7];
    wire                    p_wlast  [0:7];
    wire                    p_wvalid [0:7];
    wire                    p_wready [0:7];
    wire [ID_WIDTH-1:0]     p_bid    [0:7];
    wire [1:0]              p_bresp  [0:7];
    wire                    p_bvalid [0:7];
    wire                    p_bready [0:7];
    wire [ID_WIDTH-1:0]     p_arid   [0:7];
    wire [ADDR_WIDTH-1:0]   p_araddr [0:7];
    wire [7:0]              p_arlen  [0:7];
    wire [2:0]              p_arsize [0:7];
    wire [1:0]              p_arburst[0:7];
    wire                    p_arlock [0:7];
    wire [3:0]              p_arcache[0:7];
    wire [2:0]              p_arprot [0:7];
    wire [3:0]              p_arqos  [0:7];
    wire                    p_arvalid[0:7];
    wire                    p_arready[0:7];
    wire [ID_WIDTH-1:0]     p_rid    [0:7];
    wire [DATA_WIDTH-1:0]   p_rdata  [0:7];
    wire [1:0]              p_rresp  [0:7];
    wire                    p_rlast  [0:7];
    wire                    p_rvalid [0:7];
    wire                    p_rready [0:7];

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

            // Write response: the slave drives the interconnect inputs
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

            // Read data: the slave drives the interconnect inputs
            assign m_axi_rid    [gi*ID_WIDTH +: ID_WIDTH] = p_rid    [gi];
            assign m_axi_rdata  [gi*DATA_WIDTH +: DATA_WIDTH] = p_rdata[gi];
            assign m_axi_rresp  [gi*2 +: 2]               = p_rresp  [gi];
            assign m_axi_rlast  [gi]                      = p_rlast  [gi];
            assign m_axi_rvalid [gi]                      = p_rvalid [gi];
            assign p_rready    [gi] = m_axi_rready[gi];
        end
    endgenerate

    // ------------------------------------------------------------------
    // AXI interconnect (2 masters x 8 slaves)
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
        // LSB = port 0 (IMEM), MSB = port 7 (7-SEG)
        .M_BASE_ADDR     ({
            32'h7000_0000,  // m07 7-SEG
            32'h6000_0000,  // m06 GPIO
            32'h5000_0000,  // m05 TIMER
            32'h4000_0000,  // m04 UART
            32'h3000_0000,  // m03 CNN
            32'h2000_0000,  // m02 DMA
            32'h1000_0000,  // m01 DMEM
            32'h8000_0000   // m00 IMEM
        }),
        .M_ADDR_WIDTH    ({
            {32{32'd28}}, {32{32'd28}}, {32{32'd28}}, {32{32'd28}},
            {32{32'd28}}, {32{32'd28}}, {32{32'd28}}, {32{32'd28}}
        }),
        // S_COUNT = 2 -> every slave is reachable from both masters
        .M_CONNECT_READ  ({8{2'b11}}),
        .M_CONNECT_WRITE ({8{2'b11}})
    ) ic_inst (
        .clk (clk),
        .rst (rst),

        // Slave interface (s00 = IFU, s01 = LSU)
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

        // Master interfaces (m00..m07) - to slaves
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
        .MEM_SIZE  (32'h0000_1000)
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
    // m02, m03, m05, m06, m07 = dummy slaves
    // ------------------------------------------------------------------
    generate
        for (gi = 2; gi < 8; gi = gi + 1) begin : gen_dummy
            if (gi != 4) begin : dummy_slave
                axi_dummy_slave #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .ADDR_WIDTH(ADDR_WIDTH),
                    .ID_WIDTH  (ID_WIDTH)
                ) dum (
                    .clk   (clk),  .rst (rst),
                    .awid  (p_awid[gi]),   .awaddr (p_awaddr[gi]),
                    .awlen (p_awlen[gi]),  .awsize (p_awsize[gi]),
                    .awburst(p_awburst[gi]), .awvalid(p_awvalid[gi]), .awready(p_awready[gi]),
                    .wdata (p_wdata[gi]),  .wstrb (p_wstrb[gi]),
                    .wlast (p_wlast[gi]),  .wvalid(p_wvalid[gi]),   .wready(p_wready[gi]),
                    .bid   (p_bid[gi]),    .bresp (p_bresp[gi]),
                    .bvalid(p_bvalid[gi]), .bready(p_bready[gi]),
                    .arid  (p_arid[gi]),   .araddr (p_araddr[gi]),
                    .arlen (p_arlen[gi]),  .arsize (p_arsize[gi]),
                    .arburst(p_arburst[gi]), .arvalid(p_arvalid[gi]), .arready(p_arready[gi]),
                    .rid   (p_rid[gi]),    .rdata (p_rdata[gi]),
                    .rresp (p_rresp[gi]),  .rlast (p_rlast[gi]),
                    .rvalid(p_rvalid[gi]), .rready(p_rready[gi])
                );
            end
        end
    endgenerate

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
        .clk    (clk),
        .rst    (rst),
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

endmodule

`default_nettype wire
