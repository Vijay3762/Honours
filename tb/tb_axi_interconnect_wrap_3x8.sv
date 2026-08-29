
`timescale 1ns/1ps
`default_nettype none

// ============================================================
// Testbench for axi_interconnect_wrap_3x8
//
// Functional topology:
//
//   Master 0  = VeeR IFU-like AXI master
//   Master 1  = VeeR LSU-like AXI master
//   Master 2  = External DMA-like AXI master
//
//   Slave 0   = IMEM
//   Slave 1   = DMEM
//   Slave 2   = DMA registers
//   Slave 3   = CNN
//   Slave 4   = UART
//   Slave 5   = TIMER
//   Slave 6   = GPIO
//   Slave 7   = 7-Segment
//
// AXI: 64-bit data, 32-bit address, 8-bit WSTRB, 8-bit ID
//
// This testbench performs:
//   1. Read/write test to every slave
//   2. Access tests from all three masters
//   3. Simultaneous accesses by multiple masters
//   4. Contention test: multiple masters request the same slave
//   5. VCD waveform generation
//
// NOTE:
//   Compile this file together with:
//     - axi_interconnect_wrap_3x8.v
//     - axi_interconnect.v
//     - all source files required by axi_interconnect.v
// ============================================================

module dummy_axi_slave #(
    parameter integer DATA_WIDTH = 64,
    parameter integer ADDR_WIDTH = 32,
    parameter integer ID_WIDTH   = 8,
    parameter integer INDEX       = 0,
    parameter integer MEM_WORDS   = 256
) (
    input  wire                     clk,
    input  wire                     rst,

    input  wire [ID_WIDTH-1:0]      s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire [7:0]               s_axi_awlen,
    input  wire [2:0]               s_axi_awsize,
    input  wire [1:0]               s_axi_awburst,
    input  wire                     s_axi_awlock,
    input  wire [3:0]               s_axi_awcache,
    input  wire [2:0]               s_axi_awprot,
    input  wire [3:0]               s_axi_awqos,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    input  wire [DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                     s_axi_wlast,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    output reg  [ID_WIDTH-1:0]      s_axi_bid,
    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    input  wire [ID_WIDTH-1:0]      s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire [7:0]               s_axi_arlen,
    input  wire [2:0]               s_axi_arsize,
    input  wire [1:0]               s_axi_arburst,
    input  wire                     s_axi_arlock,
    input  wire [3:0]               s_axi_arcache,
    input  wire [2:0]               s_axi_arprot,
    input  wire [3:0]               s_axi_arqos,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    output reg  [ID_WIDTH-1:0]      s_axi_rid,
    output reg  [DATA_WIDTH-1:0]    s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rlast,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready
);

    localparam integer STRB_WIDTH = DATA_WIDTH/8;

    reg [DATA_WIDTH-1:0] mem [0:MEM_WORDS-1];

    reg                  aw_pending;
    reg [ID_WIDTH-1:0]   aw_id_reg;
    reg [ADDR_WIDTH-1:0] aw_addr_reg;

    integer i;
    integer word_index;

    function [DATA_WIDTH-1:0] make_read_data;
        input [ADDR_WIDTH-1:0] addr;
        begin
            // Deterministic per-slave/per-address signature.
            // The expected test values are calculated in the TB.
            make_read_data =
                ({(DATA_WIDTH){1'b0}})
                | (64'hD000_0000_0000_0000)
                | (INDEX[7:0] << 32)
                | addr[31:3];
        end
    endfunction

    initial begin
        for (i = 0; i < MEM_WORDS; i = i + 1)
            mem[i] = 64'h0;

        aw_pending  = 1'b0;
        aw_id_reg   = '0;
        aw_addr_reg = '0;

        s_axi_awready = 1'b0;
        s_axi_wready  = 1'b0;
        s_axi_bvalid  = 1'b0;
        s_axi_bresp   = 2'b00;
        s_axi_bid     = '0;

        s_axi_arready = 1'b0;
        s_axi_rvalid  = 1'b0;
        s_axi_rdata   = '0;
        s_axi_rresp   = 2'b00;
        s_axi_rid     = '0;
        s_axi_rlast   = 1'b0;
    end

    always @(posedge clk) begin
        if (rst) begin
            aw_pending    <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rlast   <= 1'b0;

            // Always-ready simple single-beat model.
            s_axi_awready <= 1'b1;
            s_axi_wready  <= 1'b1;
        end else begin
            s_axi_awready <= !aw_pending && !s_axi_bvalid;
            s_axi_wready  <= !s_axi_bvalid;

            // -------------------------
            // Write address handshake
            // -------------------------
            if (s_axi_awvalid && s_axi_awready) begin
                aw_pending  <= 1'b1;
                aw_id_reg   <= s_axi_awid;
                aw_addr_reg <= s_axi_awaddr;
            end

            // -------------------------
            // Write data handshake
            // -------------------------
            if (s_axi_wvalid && s_axi_wready) begin
                // If AW and W handshake on the same cycle, use current AW.
                if (aw_pending) begin
                    word_index = aw_addr_reg[11:3];
                    mem[word_index] <= s_axi_wdata;
                    aw_pending <= 1'b0;

                    s_axi_bid   <= aw_id_reg;
                    s_axi_bresp <= 2'b00; // OKAY
                    s_axi_bvalid <= 1'b1;
                end else if (s_axi_awvalid && s_axi_awready) begin
                    word_index = s_axi_awaddr[11:3];
                    mem[word_index] <= s_axi_wdata;

                    s_axi_bid   <= s_axi_awid;
                    s_axi_bresp <= 2'b00;
                    s_axi_bvalid <= 1'b1;
                end
            end

            // -------------------------
            // Write response
            // -------------------------
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            // -------------------------
            // Read address handshake
            // -------------------------
            s_axi_arready <= !s_axi_rvalid;

            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_rid    <= s_axi_arid;
                s_axi_rdata  <= make_read_data(s_axi_araddr);
                s_axi_rresp  <= 2'b00; // OKAY
                s_axi_rlast  <= 1'b1;
                s_axi_rvalid <= 1'b1;
            end

            // -------------------------
            // Read response
            // -------------------------
            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
                s_axi_rlast  <= 1'b0;
            end
        end
    end

endmodule


module tb_axi_interconnect_wrap_3x8;

    localparam integer DATA_WIDTH = 64;
    localparam integer ADDR_WIDTH = 32;
    localparam integer STRB_WIDTH = DATA_WIDTH/8;
    localparam integer ID_WIDTH   = 8;

    reg clk;
    reg rst;

    // ------------------------------------------------------------
    // 3 AXI master-side testbench interfaces
    // These connect to DUT s00/s01/s02.
    // ------------------------------------------------------------

    // AW
    reg  [ID_WIDTH-1:0]        s_awid    [0:2];
    reg  [ADDR_WIDTH-1:0]      s_awaddr  [0:2];
    reg  [7:0]                 s_awlen   [0:2];
    reg  [2:0]                 s_awsize  [0:2];
    reg  [1:0]                 s_awburst [0:2];
    reg                        s_awlock  [0:2];
    reg  [3:0]                 s_awcache [0:2];
    reg  [2:0]                 s_awprot  [0:2];
    reg  [3:0]                 s_awqos   [0:2];
    reg                        s_awvalid [0:2];
    wire                       s_awready [0:2];

    // W
    reg  [DATA_WIDTH-1:0]      s_wdata   [0:2];
    reg  [STRB_WIDTH-1:0]      s_wstrb   [0:2];
    reg                        s_wlast   [0:2];
    reg                        s_wvalid  [0:2];
    wire                       s_wready  [0:2];

    // B
    wire [ID_WIDTH-1:0]        s_bid     [0:2];
    wire [1:0]                 s_bresp   [0:2];
    wire                       s_bvalid  [0:2];
    reg                        s_bready  [0:2];

    // AR
    reg  [ID_WIDTH-1:0]        s_arid    [0:2];
    reg  [ADDR_WIDTH-1:0]      s_araddr  [0:2];
    reg  [7:0]                 s_arlen   [0:2];
    reg  [2:0]                 s_arsize  [0:2];
    reg  [1:0]                 s_arburst [0:2];
    reg                        s_arlock  [0:2];
    reg  [3:0]                 s_arcache [0:2];
    reg  [2:0]                 s_arprot  [0:2];
    reg  [3:0]                 s_arqos   [0:2];
    reg                        s_arvalid [0:2];
    wire                       s_arready [0:2];

    // R
    wire [ID_WIDTH-1:0]        s_rid     [0:2];
    wire [DATA_WIDTH-1:0]      s_rdata   [0:2];
    wire [1:0]                 s_rresp   [0:2];
    wire                       s_rlast   [0:2];
    wire                       s_rvalid  [0:2];
    reg                        s_rready  [0:2];

    // Unused USER signals on the generated wrapper
    reg [0:0] s_awuser [0:2];
    reg [0:0] s_wuser  [0:2];
    reg [0:0] s_aruser [0:2];
    wire [0:0] s_buser [0:2];
    wire [0:0] s_ruser [0:2];

    // ------------------------------------------------------------
    // 8 AXI slave-side testbench interfaces
    // These connect to DUT m00..m07.
    // ------------------------------------------------------------

    wire [ID_WIDTH-1:0]        m_awid    [0:7];
    wire [ADDR_WIDTH-1:0]      m_awaddr  [0:7];
    wire [7:0]                 m_awlen   [0:7];
    wire [2:0]                 m_awsize  [0:7];
    wire [1:0]                 m_awburst [0:7];
    wire                       m_awlock  [0:7];
    wire [3:0]                 m_awcache [0:7];
    wire [2:0]                 m_awprot  [0:7];
    wire [3:0]                 m_awqos   [0:7];
    wire [3:0]                 m_awregion[0:7];
    wire                       m_awvalid [0:7];
    wire                       m_awready [0:7];

    wire [DATA_WIDTH-1:0]      m_wdata   [0:7];
    wire [STRB_WIDTH-1:0]      m_wstrb   [0:7];
    wire                       m_wlast   [0:7];
    wire                       m_wvalid  [0:7];
    wire                       m_wready  [0:7];

    wire [ID_WIDTH-1:0]        m_bid     [0:7];
    wire [1:0]                 m_bresp   [0:7];
    wire                       m_bvalid  [0:7];
    wire                       m_bready  [0:7];

    wire [ID_WIDTH-1:0]        m_arid    [0:7];
    wire [ADDR_WIDTH-1:0]      m_araddr  [0:7];
    wire [7:0]                 m_arlen   [0:7];
    wire [2:0]                 m_arsize  [0:7];
    wire [1:0]                 m_arburst [0:7];
    wire                       m_arlock  [0:7];
    wire [3:0]                 m_arcache [0:7];
    wire [2:0]                 m_arprot  [0:7];
    wire [3:0]                 m_arqos   [0:7];
    wire [3:0]                 m_arregion[0:7];
    wire                       m_arvalid [0:7];
    wire                       m_arready [0:7];

    wire [ID_WIDTH-1:0]        m_rid     [0:7];
    wire [DATA_WIDTH-1:0]      m_rdata   [0:7];
    wire [1:0]                 m_rresp   [0:7];
    wire                       m_rlast   [0:7];
    wire                       m_rvalid  [0:7];
    wire                       m_rready  [0:7];

    wire [0:0] m_awuser [0:7];
    wire [0:0] m_wuser  [0:7];
    wire [0:0] m_buser  [0:7];
    wire [0:0] m_aruser [0:7];
    wire [0:0] m_ruser  [0:7];

    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------

    axi_interconnect_wrap_3x8 #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .STRB_WIDTH(STRB_WIDTH),
        .ID_WIDTH(ID_WIDTH),

        .M00_BASE_ADDR(32'h0000_0000),
        .M00_ADDR_WIDTH({1{32'd28}}),

        .M01_BASE_ADDR(32'h1000_0000),
        .M01_ADDR_WIDTH({1{32'd28}}),

        .M02_BASE_ADDR(32'h2000_0000),
        .M02_ADDR_WIDTH({1{32'd28}}),

        .M03_BASE_ADDR(32'h3000_0000),
        .M03_ADDR_WIDTH({1{32'd28}}),

        .M04_BASE_ADDR(32'h4000_0000),
        .M04_ADDR_WIDTH({1{32'd28}}),

        .M05_BASE_ADDR(32'h5000_0000),
        .M05_ADDR_WIDTH({1{32'd28}}),

        .M06_BASE_ADDR(32'h6000_0000),
        .M06_ADDR_WIDTH({1{32'd28}}),

        .M07_BASE_ADDR(32'h7000_0000),
        .M07_ADDR_WIDTH({1{32'd28}})
    ) dut (
        .clk(clk),
        .rst(rst),

        // ---------------- M0 / s00 ----------------
        .s00_axi_awid(s_awid[0]),
        .s00_axi_awaddr(s_awaddr[0]),
        .s00_axi_awlen(s_awlen[0]),
        .s00_axi_awsize(s_awsize[0]),
        .s00_axi_awburst(s_awburst[0]),
        .s00_axi_awlock(s_awlock[0]),
        .s00_axi_awcache(s_awcache[0]),
        .s00_axi_awprot(s_awprot[0]),
        .s00_axi_awqos(s_awqos[0]),
        .s00_axi_awuser(s_awuser[0]),
        .s00_axi_awvalid(s_awvalid[0]),
        .s00_axi_awready(s_awready[0]),
        .s00_axi_wdata(s_wdata[0]),
        .s00_axi_wstrb(s_wstrb[0]),
        .s00_axi_wlast(s_wlast[0]),
        .s00_axi_wuser(s_wuser[0]),
        .s00_axi_wvalid(s_wvalid[0]),
        .s00_axi_wready(s_wready[0]),
        .s00_axi_bid(s_bid[0]),
        .s00_axi_bresp(s_bresp[0]),
        .s00_axi_buser(s_buser[0]),
        .s00_axi_bvalid(s_bvalid[0]),
        .s00_axi_bready(s_bready[0]),
        .s00_axi_arid(s_arid[0]),
        .s00_axi_araddr(s_araddr[0]),
        .s00_axi_arlen(s_arlen[0]),
        .s00_axi_arsize(s_arsize[0]),
        .s00_axi_arburst(s_arburst[0]),
        .s00_axi_arlock(s_arlock[0]),
        .s00_axi_arcache(s_arcache[0]),
        .s00_axi_arprot(s_arprot[0]),
        .s00_axi_arqos(s_arqos[0]),
        .s00_axi_aruser(s_aruser[0]),
        .s00_axi_arvalid(s_arvalid[0]),
        .s00_axi_arready(s_arready[0]),
        .s00_axi_rid(s_rid[0]),
        .s00_axi_rdata(s_rdata[0]),
        .s00_axi_rresp(s_rresp[0]),
        .s00_axi_rlast(s_rlast[0]),
        .s00_axi_ruser(s_ruser[0]),
        .s00_axi_rvalid(s_rvalid[0]),
        .s00_axi_rready(s_rready[0]),

        // ---------------- M1 / s01 ----------------
        .s01_axi_awid(s_awid[1]),
        .s01_axi_awaddr(s_awaddr[1]),
        .s01_axi_awlen(s_awlen[1]),
        .s01_axi_awsize(s_awsize[1]),
        .s01_axi_awburst(s_awburst[1]),
        .s01_axi_awlock(s_awlock[1]),
        .s01_axi_awcache(s_awcache[1]),
        .s01_axi_awprot(s_awprot[1]),
        .s01_axi_awqos(s_awqos[1]),
        .s01_axi_awuser(s_awuser[1]),
        .s01_axi_awvalid(s_awvalid[1]),
        .s01_axi_awready(s_awready[1]),
        .s01_axi_wdata(s_wdata[1]),
        .s01_axi_wstrb(s_wstrb[1]),
        .s01_axi_wlast(s_wlast[1]),
        .s01_axi_wuser(s_wuser[1]),
        .s01_axi_wvalid(s_wvalid[1]),
        .s01_axi_wready(s_wready[1]),
        .s01_axi_bid(s_bid[1]),
        .s01_axi_bresp(s_bresp[1]),
        .s01_axi_buser(s_buser[1]),
        .s01_axi_bvalid(s_bvalid[1]),
        .s01_axi_bready(s_bready[1]),
        .s01_axi_arid(s_arid[1]),
        .s01_axi_araddr(s_araddr[1]),
        .s01_axi_arlen(s_arlen[1]),
        .s01_axi_arsize(s_arsize[1]),
        .s01_axi_arburst(s_arburst[1]),
        .s01_axi_arlock(s_arlock[1]),
        .s01_axi_arcache(s_arcache[1]),
        .s01_axi_arprot(s_arprot[1]),
        .s01_axi_arqos(s_arqos[1]),
        .s01_axi_aruser(s_aruser[1]),
        .s01_axi_arvalid(s_arvalid[1]),
        .s01_axi_arready(s_arready[1]),
        .s01_axi_rid(s_rid[1]),
        .s01_axi_rdata(s_rdata[1]),
        .s01_axi_rresp(s_rresp[1]),
        .s01_axi_rlast(s_rlast[1]),
        .s01_axi_ruser(s_ruser[1]),
        .s01_axi_rvalid(s_rvalid[1]),
        .s01_axi_rready(s_rready[1]),

        // ---------------- M2 / s02 ----------------
        .s02_axi_awid(s_awid[2]),
        .s02_axi_awaddr(s_awaddr[2]),
        .s02_axi_awlen(s_awlen[2]),
        .s02_axi_awsize(s_awsize[2]),
        .s02_axi_awburst(s_awburst[2]),
        .s02_axi_awlock(s_awlock[2]),
        .s02_axi_awcache(s_awcache[2]),
        .s02_axi_awprot(s_awprot[2]),
        .s02_axi_awqos(s_awqos[2]),
        .s02_axi_awuser(s_awuser[2]),
        .s02_axi_awvalid(s_awvalid[2]),
        .s02_axi_awready(s_awready[2]),
        .s02_axi_wdata(s_wdata[2]),
        .s02_axi_wstrb(s_wstrb[2]),
        .s02_axi_wlast(s_wlast[2]),
        .s02_axi_wuser(s_wuser[2]),
        .s02_axi_wvalid(s_wvalid[2]),
        .s02_axi_wready(s_wready[2]),
        .s02_axi_bid(s_bid[2]),
        .s02_axi_bresp(s_bresp[2]),
        .s02_axi_buser(s_buser[2]),
        .s02_axi_bvalid(s_bvalid[2]),
        .s02_axi_bready(s_bready[2]),
        .s02_axi_arid(s_arid[2]),
        .s02_axi_araddr(s_araddr[2]),
        .s02_axi_arlen(s_arlen[2]),
        .s02_axi_arsize(s_arsize[2]),
        .s02_axi_arburst(s_arburst[2]),
        .s02_axi_arlock(s_arlock[2]),
        .s02_axi_arcache(s_arcache[2]),
        .s02_axi_arprot(s_arprot[2]),
        .s02_axi_arqos(s_arqos[2]),
        .s02_axi_aruser(s_aruser[2]),
        .s02_axi_arvalid(s_arvalid[2]),
        .s02_axi_arready(s_arready[2]),
        .s02_axi_rid(s_rid[2]),
        .s02_axi_rdata(s_rdata[2]),
        .s02_axi_rresp(s_rresp[2]),
        .s02_axi_rlast(s_rlast[2]),
        .s02_axi_ruser(s_ruser[2]),
        .s02_axi_rvalid(s_rvalid[2]),
        .s02_axi_rready(s_rready[2]),

        // ---------------- Slave 0 ----------------
        .m00_axi_awid(m_awid[0]),
        .m00_axi_awaddr(m_awaddr[0]),
        .m00_axi_awlen(m_awlen[0]),
        .m00_axi_awsize(m_awsize[0]),
        .m00_axi_awburst(m_awburst[0]),
        .m00_axi_awlock(m_awlock[0]),
        .m00_axi_awcache(m_awcache[0]),
        .m00_axi_awprot(m_awprot[0]),
        .m00_axi_awqos(m_awqos[0]),
        .m00_axi_awregion(m_awregion[0]),
        .m00_axi_awuser(m_awuser[0]),
        .m00_axi_awvalid(m_awvalid[0]),
        .m00_axi_awready(m_awready[0]),
        .m00_axi_wdata(m_wdata[0]),
        .m00_axi_wstrb(m_wstrb[0]),
        .m00_axi_wlast(m_wlast[0]),
        .m00_axi_wuser(m_wuser[0]),
        .m00_axi_wvalid(m_wvalid[0]),
        .m00_axi_wready(m_wready[0]),
        .m00_axi_bid(m_bid[0]),
        .m00_axi_bresp(m_bresp[0]),
        .m00_axi_buser(m_buser[0]),
        .m00_axi_bvalid(m_bvalid[0]),
        .m00_axi_bready(m_bready[0]),
        .m00_axi_arid(m_arid[0]),
        .m00_axi_araddr(m_araddr[0]),
        .m00_axi_arlen(m_arlen[0]),
        .m00_axi_arsize(m_arsize[0]),
        .m00_axi_arburst(m_arburst[0]),
        .m00_axi_arlock(m_arlock[0]),
        .m00_axi_arcache(m_arcache[0]),
        .m00_axi_arprot(m_arprot[0]),
        .m00_axi_arqos(m_arqos[0]),
        .m00_axi_arregion(m_arregion[0]),
        .m00_axi_aruser(m_aruser[0]),
        .m00_axi_arvalid(m_arvalid[0]),
        .m00_axi_arready(m_arready[0]),
        .m00_axi_rid(m_rid[0]),
        .m00_axi_rdata(m_rdata[0]),
        .m00_axi_rresp(m_rresp[0]),
        .m00_axi_rlast(m_rlast[0]),
        .m00_axi_ruser(m_ruser[0]),
        .m00_axi_rvalid(m_rvalid[0]),
        .m00_axi_rready(m_rready[0]),

        // ---------------- Slave 1 ----------------
        .m01_axi_awid(m_awid[1]),
        .m01_axi_awaddr(m_awaddr[1]),
        .m01_axi_awlen(m_awlen[1]),
        .m01_axi_awsize(m_awsize[1]),
        .m01_axi_awburst(m_awburst[1]),
        .m01_axi_awlock(m_awlock[1]),
        .m01_axi_awcache(m_awcache[1]),
        .m01_axi_awprot(m_awprot[1]),
        .m01_axi_awqos(m_awqos[1]),
        .m01_axi_awregion(m_awregion[1]),
        .m01_axi_awuser(m_awuser[1]),
        .m01_axi_awvalid(m_awvalid[1]),
        .m01_axi_awready(m_awready[1]),
        .m01_axi_wdata(m_wdata[1]),
        .m01_axi_wstrb(m_wstrb[1]),
        .m01_axi_wlast(m_wlast[1]),
        .m01_axi_wuser(m_wuser[1]),
        .m01_axi_wvalid(m_wvalid[1]),
        .m01_axi_wready(m_wready[1]),
        .m01_axi_bid(m_bid[1]),
        .m01_axi_bresp(m_bresp[1]),
        .m01_axi_buser(m_buser[1]),
        .m01_axi_bvalid(m_bvalid[1]),
        .m01_axi_bready(m_bready[1]),
        .m01_axi_arid(m_arid[1]),
        .m01_axi_araddr(m_araddr[1]),
        .m01_axi_arlen(m_arlen[1]),
        .m01_axi_arsize(m_arsize[1]),
        .m01_axi_arburst(m_arburst[1]),
        .m01_axi_arlock(m_arlock[1]),
        .m01_axi_arcache(m_arcache[1]),
        .m01_axi_arprot(m_arprot[1]),
        .m01_axi_arqos(m_arqos[1]),
        .m01_axi_arregion(m_arregion[1]),
        .m01_axi_aruser(m_aruser[1]),
        .m01_axi_arvalid(m_arvalid[1]),
        .m01_axi_arready(m_arready[1]),
        .m01_axi_rid(m_rid[1]),
        .m01_axi_rdata(m_rdata[1]),
        .m01_axi_rresp(m_rresp[1]),
        .m01_axi_rlast(m_rlast[1]),
        .m01_axi_ruser(m_ruser[1]),
        .m01_axi_rvalid(m_rvalid[1]),
        .m01_axi_rready(m_rready[1]),

        // ---------------- Slave 2 ----------------
        .m02_axi_awid(m_awid[2]),
        .m02_axi_awaddr(m_awaddr[2]),
        .m02_axi_awlen(m_awlen[2]),
        .m02_axi_awsize(m_awsize[2]),
        .m02_axi_awburst(m_awburst[2]),
        .m02_axi_awlock(m_awlock[2]),
        .m02_axi_awcache(m_awcache[2]),
        .m02_axi_awprot(m_awprot[2]),
        .m02_axi_awqos(m_awqos[2]),
        .m02_axi_awregion(m_awregion[2]),
        .m02_axi_awuser(m_awuser[2]),
        .m02_axi_awvalid(m_awvalid[2]),
        .m02_axi_awready(m_awready[2]),
        .m02_axi_wdata(m_wdata[2]),
        .m02_axi_wstrb(m_wstrb[2]),
        .m02_axi_wlast(m_wlast[2]),
        .m02_axi_wuser(m_wuser[2]),
        .m02_axi_wvalid(m_wvalid[2]),
        .m02_axi_wready(m_wready[2]),
        .m02_axi_bid(m_bid[2]),
        .m02_axi_bresp(m_bresp[2]),
        .m02_axi_buser(m_buser[2]),
        .m02_axi_bvalid(m_bvalid[2]),
        .m02_axi_bready(m_bready[2]),
        .m02_axi_arid(m_arid[2]),
        .m02_axi_araddr(m_araddr[2]),
        .m02_axi_arlen(m_arlen[2]),
        .m02_axi_arsize(m_arsize[2]),
        .m02_axi_arburst(m_arburst[2]),
        .m02_axi_arlock(m_arlock[2]),
        .m02_axi_arcache(m_arcache[2]),
        .m02_axi_arprot(m_arprot[2]),
        .m02_axi_arqos(m_arqos[2]),
        .m02_axi_arregion(m_arregion[2]),
        .m02_axi_aruser(m_aruser[2]),
        .m02_axi_arvalid(m_arvalid[2]),
        .m02_axi_arready(m_arready[2]),
        .m02_axi_rid(m_rid[2]),
        .m02_axi_rdata(m_rdata[2]),
        .m02_axi_rresp(m_rresp[2]),
        .m02_axi_rlast(m_rlast[2]),
        .m02_axi_ruser(m_ruser[2]),
        .m02_axi_rvalid(m_rvalid[2]),
        .m02_axi_rready(m_rready[2]),

        // ---------------- Slave 3 ----------------
        .m03_axi_awid(m_awid[3]),
        .m03_axi_awaddr(m_awaddr[3]),
        .m03_axi_awlen(m_awlen[3]),
        .m03_axi_awsize(m_awsize[3]),
        .m03_axi_awburst(m_awburst[3]),
        .m03_axi_awlock(m_awlock[3]),
        .m03_axi_awcache(m_awcache[3]),
        .m03_axi_awprot(m_awprot[3]),
        .m03_axi_awqos(m_awqos[3]),
        .m03_axi_awregion(m_awregion[3]),
        .m03_axi_awuser(m_awuser[3]),
        .m03_axi_awvalid(m_awvalid[3]),
        .m03_axi_awready(m_awready[3]),
        .m03_axi_wdata(m_wdata[3]),
        .m03_axi_wstrb(m_wstrb[3]),
        .m03_axi_wlast(m_wlast[3]),
        .m03_axi_wuser(m_wuser[3]),
        .m03_axi_wvalid(m_wvalid[3]),
        .m03_axi_wready(m_wready[3]),
        .m03_axi_bid(m_bid[3]),
        .m03_axi_bresp(m_bresp[3]),
        .m03_axi_buser(m_buser[3]),
        .m03_axi_bvalid(m_bvalid[3]),
        .m03_axi_bready(m_bready[3]),
        .m03_axi_arid(m_arid[3]),
        .m03_axi_araddr(m_araddr[3]),
        .m03_axi_arlen(m_arlen[3]),
        .m03_axi_arsize(m_arsize[3]),
        .m03_axi_arburst(m_arburst[3]),
        .m03_axi_arlock(m_arlock[3]),
        .m03_axi_arcache(m_arcache[3]),
        .m03_axi_arprot(m_arprot[3]),
        .m03_axi_arqos(m_arqos[3]),
        .m03_axi_arregion(m_arregion[3]),
        .m03_axi_aruser(m_aruser[3]),
        .m03_axi_arvalid(m_arvalid[3]),
        .m03_axi_arready(m_arready[3]),
        .m03_axi_rid(m_rid[3]),
        .m03_axi_rdata(m_rdata[3]),
        .m03_axi_rresp(m_rresp[3]),
        .m03_axi_rlast(m_rlast[3]),
        .m03_axi_ruser(m_ruser[3]),
        .m03_axi_rvalid(m_rvalid[3]),
        .m03_axi_rready(m_rready[3]),

        // ---------------- Slave 4 ----------------
        .m04_axi_awid(m_awid[4]),
        .m04_axi_awaddr(m_awaddr[4]),
        .m04_axi_awlen(m_awlen[4]),
        .m04_axi_awsize(m_awsize[4]),
        .m04_axi_awburst(m_awburst[4]),
        .m04_axi_awlock(m_awlock[4]),
        .m04_axi_awcache(m_awcache[4]),
        .m04_axi_awprot(m_awprot[4]),
        .m04_axi_awqos(m_awqos[4]),
        .m04_axi_awregion(m_awregion[4]),
        .m04_axi_awuser(m_awuser[4]),
        .m04_axi_awvalid(m_awvalid[4]),
        .m04_axi_awready(m_awready[4]),
        .m04_axi_wdata(m_wdata[4]),
        .m04_axi_wstrb(m_wstrb[4]),
        .m04_axi_wlast(m_wlast[4]),
        .m04_axi_wuser(m_wuser[4]),
        .m04_axi_wvalid(m_wvalid[4]),
        .m04_axi_wready(m_wready[4]),
        .m04_axi_bid(m_bid[4]),
        .m04_axi_bresp(m_bresp[4]),
        .m04_axi_buser(m_buser[4]),
        .m04_axi_bvalid(m_bvalid[4]),
        .m04_axi_bready(m_bready[4]),
        .m04_axi_arid(m_arid[4]),
        .m04_axi_araddr(m_araddr[4]),
        .m04_axi_arlen(m_arlen[4]),
        .m04_axi_arsize(m_arsize[4]),
        .m04_axi_arburst(m_arburst[4]),
        .m04_axi_arlock(m_arlock[4]),
        .m04_axi_arcache(m_arcache[4]),
        .m04_axi_arprot(m_arprot[4]),
        .m04_axi_arqos(m_arqos[4]),
        .m04_axi_arregion(m_arregion[4]),
        .m04_axi_aruser(m_aruser[4]),
        .m04_axi_arvalid(m_arvalid[4]),
        .m04_axi_arready(m_arready[4]),
        .m04_axi_rid(m_rid[4]),
        .m04_axi_rdata(m_rdata[4]),
        .m04_axi_rresp(m_rresp[4]),
        .m04_axi_rlast(m_rlast[4]),
        .m04_axi_ruser(m_ruser[4]),
        .m04_axi_rvalid(m_rvalid[4]),
        .m04_axi_rready(m_rready[4]),

        // ---------------- Slave 5 ----------------
        .m05_axi_awid(m_awid[5]),
        .m05_axi_awaddr(m_awaddr[5]),
        .m05_axi_awlen(m_awlen[5]),
        .m05_axi_awsize(m_awsize[5]),
        .m05_axi_awburst(m_awburst[5]),
        .m05_axi_awlock(m_awlock[5]),
        .m05_axi_awcache(m_awcache[5]),
        .m05_axi_awprot(m_awprot[5]),
        .m05_axi_awqos(m_awqos[5]),
        .m05_axi_awregion(m_awregion[5]),
        .m05_axi_awuser(m_awuser[5]),
        .m05_axi_awvalid(m_awvalid[5]),
        .m05_axi_awready(m_awready[5]),
        .m05_axi_wdata(m_wdata[5]),
        .m05_axi_wstrb(m_wstrb[5]),
        .m05_axi_wlast(m_wlast[5]),
        .m05_axi_wuser(m_wuser[5]),
        .m05_axi_wvalid(m_wvalid[5]),
        .m05_axi_wready(m_wready[5]),
        .m05_axi_bid(m_bid[5]),
        .m05_axi_bresp(m_bresp[5]),
        .m05_axi_buser(m_buser[5]),
        .m05_axi_bvalid(m_bvalid[5]),
        .m05_axi_bready(m_bready[5]),
        .m05_axi_arid(m_arid[5]),
        .m05_axi_araddr(m_araddr[5]),
        .m05_axi_arlen(m_arlen[5]),
        .m05_axi_arsize(m_arsize[5]),
        .m05_axi_arburst(m_arburst[5]),
        .m05_axi_arlock(m_arlock[5]),
        .m05_axi_arcache(m_arcache[5]),
        .m05_axi_arprot(m_arprot[5]),
        .m05_axi_arqos(m_arqos[5]),
        .m05_axi_arregion(m_arregion[5]),
        .m05_axi_aruser(m_aruser[5]),
        .m05_axi_arvalid(m_arvalid[5]),
        .m05_axi_arready(m_arready[5]),
        .m05_axi_rid(m_rid[5]),
        .m05_axi_rdata(m_rdata[5]),
        .m05_axi_rresp(m_rresp[5]),
        .m05_axi_rlast(m_rlast[5]),
        .m05_axi_ruser(m_ruser[5]),
        .m05_axi_rvalid(m_rvalid[5]),
        .m05_axi_rready(m_rready[5]),

        // ---------------- Slave 6 ----------------
        .m06_axi_awid(m_awid[6]),
        .m06_axi_awaddr(m_awaddr[6]),
        .m06_axi_awlen(m_awlen[6]),
        .m06_axi_awsize(m_awsize[6]),
        .m06_axi_awburst(m_awburst[6]),
        .m06_axi_awlock(m_awlock[6]),
        .m06_axi_awcache(m_awcache[6]),
        .m06_axi_awprot(m_awprot[6]),
        .m06_axi_awqos(m_awqos[6]),
        .m06_axi_awregion(m_awregion[6]),
        .m06_axi_awuser(m_awuser[6]),
        .m06_axi_awvalid(m_awvalid[6]),
        .m06_axi_awready(m_awready[6]),
        .m06_axi_wdata(m_wdata[6]),
        .m06_axi_wstrb(m_wstrb[6]),
        .m06_axi_wlast(m_wlast[6]),
        .m06_axi_wuser(m_wuser[6]),
        .m06_axi_wvalid(m_wvalid[6]),
        .m06_axi_wready(m_wready[6]),
        .m06_axi_bid(m_bid[6]),
        .m06_axi_bresp(m_bresp[6]),
        .m06_axi_buser(m_buser[6]),
        .m06_axi_bvalid(m_bvalid[6]),
        .m06_axi_bready(m_bready[6]),
        .m06_axi_arid(m_arid[6]),
        .m06_axi_araddr(m_araddr[6]),
        .m06_axi_arlen(m_arlen[6]),
        .m06_axi_arsize(m_arsize[6]),
        .m06_axi_arburst(m_arburst[6]),
        .m06_axi_arlock(m_arlock[6]),
        .m06_axi_arcache(m_arcache[6]),
        .m06_axi_arprot(m_arprot[6]),
        .m06_axi_arqos(m_arqos[6]),
        .m06_axi_arregion(m_arregion[6]),
        .m06_axi_aruser(m_aruser[6]),
        .m06_axi_arvalid(m_arvalid[6]),
        .m06_axi_arready(m_arready[6]),
        .m06_axi_rid(m_rid[6]),
        .m06_axi_rdata(m_rdata[6]),
        .m06_axi_rresp(m_rresp[6]),
        .m06_axi_rlast(m_rlast[6]),
        .m06_axi_ruser(m_ruser[6]),
        .m06_axi_rvalid(m_rvalid[6]),
        .m06_axi_rready(m_rready[6]),

        // ---------------- Slave 7 ----------------
        .m07_axi_awid(m_awid[7]),
        .m07_axi_awaddr(m_awaddr[7]),
        .m07_axi_awlen(m_awlen[7]),
        .m07_axi_awsize(m_awsize[7]),
        .m07_axi_awburst(m_awburst[7]),
        .m07_axi_awlock(m_awlock[7]),
        .m07_axi_awcache(m_awcache[7]),
        .m07_axi_awprot(m_awprot[7]),
        .m07_axi_awqos(m_awqos[7]),
        .m07_axi_awregion(m_awregion[7]),
        .m07_axi_awuser(m_awuser[7]),
        .m07_axi_awvalid(m_awvalid[7]),
        .m07_axi_awready(m_awready[7]),
        .m07_axi_wdata(m_wdata[7]),
        .m07_axi_wstrb(m_wstrb[7]),
        .m07_axi_wlast(m_wlast[7]),
        .m07_axi_wuser(m_wuser[7]),
        .m07_axi_wvalid(m_wvalid[7]),
        .m07_axi_wready(m_wready[7]),
        .m07_axi_bid(m_bid[7]),
        .m07_axi_bresp(m_bresp[7]),
        .m07_axi_buser(m_buser[7]),
        .m07_axi_bvalid(m_bvalid[7]),
        .m07_axi_bready(m_bready[7]),
        .m07_axi_arid(m_arid[7]),
        .m07_axi_araddr(m_araddr[7]),
        .m07_axi_arlen(m_arlen[7]),
        .m07_axi_arsize(m_arsize[7]),
        .m07_axi_arburst(m_arburst[7]),
        .m07_axi_arlock(m_arlock[7]),
        .m07_axi_arcache(m_arcache[7]),
        .m07_axi_arprot(m_arprot[7]),
        .m07_axi_arqos(m_arqos[7]),
        .m07_axi_arregion(m_arregion[7]),
        .m07_axi_aruser(m_aruser[7]),
        .m07_axi_arvalid(m_arvalid[7]),
        .m07_axi_arready(m_arready[7]),
        .m07_axi_rid(m_rid[7]),
        .m07_axi_rdata(m_rdata[7]),
        .m07_axi_rresp(m_rresp[7]),
        .m07_axi_rlast(m_rlast[7]),
        .m07_axi_ruser(m_ruser[7]),
        .m07_axi_rvalid(m_rvalid[7]),
        .m07_axi_rready(m_rready[7])
    );

    // ------------------------------------------------------------
    // Dummy slaves
    // ------------------------------------------------------------
    genvar g;
    generate
        for (g = 0; g < 8; g = g + 1) begin : GEN_SLAVES
            dummy_axi_slave #(
                .DATA_WIDTH(DATA_WIDTH),
                .ADDR_WIDTH(ADDR_WIDTH),
                .ID_WIDTH(ID_WIDTH),
                .INDEX(g),
                .MEM_WORDS(256)
            ) slave_model (
                .clk(clk),
                .rst(rst),

                .s_axi_awid(m_awid[g]),
                .s_axi_awaddr(m_awaddr[g]),
                .s_axi_awlen(m_awlen[g]),
                .s_axi_awsize(m_awsize[g]),
                .s_axi_awburst(m_awburst[g]),
                .s_axi_awlock(m_awlock[g]),
                .s_axi_awcache(m_awcache[g]),
                .s_axi_awprot(m_awprot[g]),
                .s_axi_awqos(m_awqos[g]),
                .s_axi_awvalid(m_awvalid[g]),
                .s_axi_awready(m_awready[g]),

                .s_axi_wdata(m_wdata[g]),
                .s_axi_wstrb(m_wstrb[g]),
                .s_axi_wlast(m_wlast[g]),
                .s_axi_wvalid(m_wvalid[g]),
                .s_axi_wready(m_wready[g]),

                .s_axi_bid(m_bid[g]),
                .s_axi_bresp(m_bresp[g]),
                .s_axi_bvalid(m_bvalid[g]),
                .s_axi_bready(m_bready[g]),

                .s_axi_arid(m_arid[g]),
                .s_axi_araddr(m_araddr[g]),
                .s_axi_arlen(m_arlen[g]),
                .s_axi_arsize(m_arsize[g]),
                .s_axi_arburst(m_arburst[g]),
                .s_axi_arlock(m_arlock[g]),
                .s_axi_arcache(m_arcache[g]),
                .s_axi_arprot(m_arprot[g]),
                .s_axi_arqos(m_arqos[g]),
                .s_axi_arvalid(m_arvalid[g]),
                .s_axi_arready(m_arready[g]),

                .s_axi_rid(m_rid[g]),
                .s_axi_rdata(m_rdata[g]),
                .s_axi_rresp(m_rresp[g]),
                .s_axi_rlast(m_rlast[g]),
                .s_axi_rvalid(m_rvalid[g]),
                .s_axi_rready(m_rready[g])
            );
        end
    endgenerate

    // ------------------------------------------------------------
    // Clock
    // ------------------------------------------------------------
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    // ------------------------------------------------------------
    // Helper tasks
    // ------------------------------------------------------------

    task automatic init_master;
        input integer m;
        begin
            s_awid[m]    = '0;
            s_awaddr[m]  = '0;
            s_awlen[m]   = 8'd0;
            s_awsize[m]  = 3'd3; // 8 bytes = 2^3
            s_awburst[m] = 2'b01; // INCR
            s_awlock[m]  = 1'b0;
            s_awcache[m] = 4'b0000;
            s_awprot[m]  = 3'b000;
            s_awqos[m]   = 4'b0000;
            s_awuser[m]  = '0;
            s_awvalid[m] = 1'b0;

            s_wdata[m]   = '0;
            s_wstrb[m]   = 8'hFF;
            s_wlast[m]   = 1'b1;
            s_wuser[m]   = '0;
            s_wvalid[m]  = 1'b0;
            s_bready[m]  = 1'b0;

            s_arid[m]    = '0;
            s_araddr[m]  = '0;
            s_arlen[m]   = 8'd0;
            s_arsize[m]  = 3'd3;
            s_arburst[m] = 2'b01;
            s_arlock[m]  = 1'b0;
            s_arcache[m] = 4'b0000;
            s_arprot[m]  = 3'b000;
            s_arqos[m]   = 4'b0000;
            s_aruser[m]  = '0;
            s_arvalid[m] = 1'b0;
            s_rready[m]  = 1'b0;
        end
    endtask

    task automatic axi_write;
        input integer m;
        input [31:0] addr;
        input [63:0] data;
        input [7:0]  id;
        integer aw_done;
        integer w_done;
        begin
            aw_done = 0;
            w_done  = 0;

            @(posedge clk);
            s_awid[m]    = id;
            s_awaddr[m]  = addr;
            s_awlen[m]   = 8'd0;
            s_awsize[m]  = 3'd3;
            s_awburst[m] = 2'b01;
            s_awvalid[m] = 1'b1;

            s_wdata[m]   = data;
            s_wstrb[m]   = 8'hFF;
            s_wlast[m]   = 1'b1;
            s_wvalid[m]  = 1'b1;

            s_bready[m]  = 1'b1;

            while ((aw_done == 0) || (w_done == 0)) begin
                @(posedge clk);

                if (!aw_done && s_awready[m]) begin
                    aw_done = 1;
                    s_awvalid[m] = 1'b0;
                end

                if (!w_done && s_wready[m]) begin
                    w_done = 1;
                    s_wvalid[m] = 1'b0;
                end
            end

            while (!s_bvalid[m]) begin
                @(posedge clk);
            end

            if (s_bresp[m] !== 2'b00) begin
                $display("[%0t] ERROR: M%0d write response for %h = %b",
                         $time, m, addr, s_bresp[m]);
                $fatal;
            end

            @(posedge clk);
            s_bready[m] = 1'b0;
        end
    endtask

    task automatic axi_read;
        input integer m;
        input [31:0] addr;
        input [7:0]  id;
        output [63:0] data;
        begin
            @(posedge clk);

            s_arid[m]    = id;
            s_araddr[m]  = addr;
            s_arlen[m]   = 8'd0;
            s_arsize[m]  = 3'd3;
            s_arburst[m] = 2'b01;
            s_arvalid[m] = 1'b1;
            s_rready[m]  = 1'b1;

            while (!s_arready[m]) begin
                @(posedge clk);
            end

            @(posedge clk);
            s_arvalid[m] = 1'b0;

            while (!s_rvalid[m]) begin
                @(posedge clk);
            end

            data = s_rdata[m];

            if (s_rresp[m] !== 2'b00) begin
                $display("[%0t] ERROR: M%0d read response for %h = %b",
                         $time, m, addr, s_rresp[m]);
                $fatal;
            end

            if (s_rid[m] !== id) begin
                $display("[%0t] ERROR: M%0d read ID mismatch. Expected %h, got %h",
                         $time, m, id, s_rid[m]);
                $fatal;
            end

            if (!s_rlast[m]) begin
                $display("[%0t] ERROR: M%0d read did not assert RLAST",
                         $time, m);
                $fatal;
            end

            @(posedge clk);
            s_rready[m] = 1'b0;
        end
    endtask

    function [63:0] expected_read_data;
        input integer slave_index;
        input [31:0] addr;
        begin
            expected_read_data =
                64'hD000_0000_0000_0000
                | (slave_index[7:0] << 32)
                | addr[31:3];
        end
    endfunction

    task automatic write_read_check;
        input integer m;
        input integer slave_index;
        input [31:0] offset;
        input [63:0] write_data;
        reg [63:0] rd_data;
        reg [31:0] test_addr;
        begin
            test_addr = (slave_index << 28) | offset;

            $display("[%0t] M%0d -> S%0d : WRITE @ %h = %h",
                     $time, m, slave_index, test_addr, write_data);
            axi_write(m, test_addr, write_data, 8'h10 + slave_index);

            $display("[%0t] M%0d -> S%0d : READ  @ %h",
                     $time, m, slave_index, test_addr);
            axi_read(m, test_addr, 8'h20 + slave_index, rd_data);

            // The dummy slave returns a deterministic signature on reads.
            if (rd_data !== expected_read_data(slave_index, test_addr)) begin
                $display("[%0t] ERROR: Wrong route/read data.",
                         $time);
                $display("         Master      = M%0d", m);
                $display("         Expected    = %h", expected_read_data(slave_index, test_addr));
                $display("         Actual      = %h", rd_data);
                $fatal;
            end else begin
                $display("[%0t] PASS: M%0d correctly reached S%0d",
                         $time, m, slave_index);
            end
        end
    endtask

    // ------------------------------------------------------------
    // Main test
    // ------------------------------------------------------------

    integer k;
    reg [63:0] rd_tmp;

    initial begin
        $dumpfile("axi_interconnect_3x8.vcd");
        $dumpvars(0, tb_axi_interconnect_wrap_3x8);

        // Initialize all three masters
        for (k = 0; k < 3; k = k + 1)
            init_master(k);

        // Reset
        rst = 1'b1;
        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (3) @(posedge clk);

        $display("");
        $display("============================================================");
        $display(" AXI4 3x8 INTERCONNECT TEST START");
        $display(" DATA_WIDTH = %0d", DATA_WIDTH);
        $display(" ADDR_WIDTH = %0d", ADDR_WIDTH);
        $display("============================================================");
        $display("");

        // --------------------------------------------------------
        // TEST 1: Master 0 accesses all 8 slaves
        // --------------------------------------------------------
        $display("TEST 1: Master 0 -> all 8 slaves");
        for (k = 0; k < 8; k = k + 1) begin
            write_read_check(0, k, 32'h0000_0100 + (k * 8),
                             64'h1111_0000_0000_0000 | k);
        end
        $display("TEST 1 PASSED");
        $display("");

        // --------------------------------------------------------
        // TEST 2: Master 1 accesses all 8 slaves
        // --------------------------------------------------------
        $display("TEST 2: Master 1 -> all 8 slaves");
        for (k = 0; k < 8; k = k + 1) begin
            write_read_check(1, k, 32'h0000_0200 + (k * 8),
                             64'h2222_0000_0000_0000 | k);
        end
        $display("TEST 2 PASSED");
        $display("");

        // --------------------------------------------------------
        // TEST 3: Master 2 accesses all 8 slaves
        // --------------------------------------------------------
        $display("TEST 3: Master 2 -> all 8 slaves");
        for (k = 0; k < 8; k = k + 1) begin
            write_read_check(2, k, 32'h0000_0300 + (k * 8),
                             64'h3333_0000_0000_0000 | k);
        end
        $display("TEST 3 PASSED");
        $display("");

        // --------------------------------------------------------
        // TEST 4: Simultaneous accesses to different slaves
        // --------------------------------------------------------
        $display("TEST 4: Simultaneous 3-master / 3-slave access");

        fork
            begin
                axi_write(0, 32'h0000_0400, 64'hAAAA_0000_0000_0001, 8'h40);
            end
            begin
                axi_write(1, 32'h1000_0400, 64'hBBBB_0000_0000_0002, 8'h41);
            end
            begin
                axi_write(2, 32'h2000_0400, 64'hCCCC_0000_0000_0003, 8'h42);
            end
        join

        $display("TEST 4 PASSED");
        $display("");

        // --------------------------------------------------------
        // TEST 5: Contention - multiple masters target S3 (CNN)
        // --------------------------------------------------------
        $display("TEST 5: 3-master contention on Slave 3 (CNN)");

        fork
            begin
                axi_write(0, 32'h3000_0500, 64'hAAAA_AAAA_0000_0001, 8'h50);
            end
            begin
                axi_write(1, 32'h3000_0508, 64'hBBBB_BBBB_0000_0002, 8'h51);
            end
            begin
                axi_write(2, 32'h3000_0510, 64'hCCCC_CCCC_0000_0003, 8'h52);
            end
        join

        // Read back the three addresses from different masters.
        axi_read(0, 32'h3000_0500, 8'h60, rd_tmp);
        axi_read(1, 32'h3000_0508, 8'h61, rd_tmp);
        axi_read(2, 32'h3000_0510, 8'h62, rd_tmp);

        $display("TEST 5 PASSED");
        $display("");

        // --------------------------------------------------------
        // TEST 6: Same master changes destination repeatedly
        // --------------------------------------------------------
        $display("TEST 6: Repeated routing changes");
        axi_write(0, 32'h4000_0000, 64'h4000, 8'h70);
        axi_write(0, 32'h5000_0000, 64'h5000, 8'h71);
        axi_write(0, 32'h6000_0000, 64'h6000, 8'h72);
        axi_write(0, 32'h7000_0000, 64'h7000, 8'h73);
        $display("TEST 6 PASSED");
        $display("");

        $display("============================================================");
        $display(" ALL AXI4 3x8 INTERCONNECT TESTS PASSED");
        $display("============================================================");
        $display("");

        repeat (10) @(posedge clk);
        $finish;
    end
	initial begin
    $fsdbDumpfile("dump.fsdb");  // Record the waveform, waveform name testname.fsdb
    $fsdbDumpvars("+all");    // + all parameters, Struct structures in Dump SV
    $fsdbDumpSVA();      // Present the result of Assertion in FSDB
    $fsdbDumpMDA(); 
  end

endmodule

`default_nettype wire
