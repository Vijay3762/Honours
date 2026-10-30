// =============================================================================
// File    : axi_dma_ctrl.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Parameterized AXI4 DMA Controller.
//           Acts as an AXI4 slave for CPU programming (register interface at
//           m02 / 0x2000_0000) and as an AXI4 master for data movement
//           (read from source, write to destination).
//
// Register map (32-bit registers, word-aligned, 4 KiB aperture):
//   Offset  Register         Access  Description
//   ------  ---------------  ------  ----------------------------------------
//   0x00    DMA_CTRL         RW      bit0: enable, bit1: start (auto-clear),
//                                    bit2: irq_enable, bit3: dir (0=mem2mem),
//                                    bit4: auto_restart
//   0x04    DMA_SRC_ADDR     RW      Source address (AXI master read addr)
//   0x08    DMA_DST_ADDR     RW      Destination address (AXI master write)
//   0x0C    DMA_LEN          RW      Transfer length in 64-bit beats (1..255)
//   0x10    DMA_STATUS       RO      bit0: busy, bit1: done, bit2: error
//   0x14    DMA_IRQ_STATUS   RW      bit0: done_irq (write-1-to-clear)
//   0x18    DMA_ID           RO      Fixed peripheral ID ("DMA ")
//   0x1C    DMA_VERSION      RO      Fixed version
//
// Reset: active-high (rst).
//   On reset: all regs=0, busy=0, done=0, irq=0.
//
// AXI response codes:
//   OKAY   = 2'b00  -- valid register access
//   SLVERR = 2'b10  -- reserved offset, write to RO register, misaligned
//   DECERR = 2'b11  -- address outside the valid 4 KiB aperture
//
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_dma_ctrl #(
    parameter integer DATA_WIDTH      = 64,
    parameter integer ADDR_WIDTH      = 32,
    parameter integer ID_WIDTH        = 8,
    parameter integer STRB_WIDTH      = DATA_WIDTH / 8,

    // Register aperture: 4 KiB (12 address bits)
    parameter integer REG_ADDR_WIDTH  = 12,

    // Base address of this DMA slave in the system memory map.
    parameter [ADDR_WIDTH-1:0]        BASE_ADDR = 32'h2000_0000,

    // Fixed peripheral ID and version
    parameter [31:0]                  DMA_ID_VALUE      = 32'h444D4120,  // "DMA "
    parameter [31:0]                  DMA_VERSION_VALUE = 32'h00010000   // v1.0
)(
    input  wire                        clk,
    input  wire                        rst,          // active-high

    // ---------------------------------------------------------
    // AXI4 Slave interface — CPU programming port (connects to
    // interconnect master port m02)
    // ---------------------------------------------------------
    // Write address
    input  wire [ID_WIDTH-1:0]         s_axi_awid,
    input  wire [ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [7:0]                  s_axi_awlen,
    input  wire [2:0]                  s_axi_awsize,
    input  wire [1:0]                  s_axi_awburst,
    input  wire                        s_axi_awlock,
    input  wire [3:0]                  s_axi_awcache,
    input  wire [2:0]                  s_axi_awprot,
    input  wire [3:0]                  s_axi_awqos,
    input  wire [3:0]                  s_axi_awregion,
    input  wire                        s_axi_awvalid,
    output wire                        s_axi_awready,
    // Write data
    input  wire [DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [STRB_WIDTH-1:0]       s_axi_wstrb,
    input  wire                        s_axi_wlast,
    input  wire                        s_axi_wvalid,
    output wire                        s_axi_wready,
    // Write response
    output wire [ID_WIDTH-1:0]         s_axi_bid,
    output wire [1:0]                  s_axi_bresp,
    output wire                        s_axi_bvalid,
    input  wire                        s_axi_bready,
    // Read address
    input  wire [ID_WIDTH-1:0]         s_axi_arid,
    input  wire [ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [7:0]                  s_axi_arlen,
    input  wire [2:0]                  s_axi_arsize,
    input  wire [1:0]                  s_axi_arburst,
    input  wire                        s_axi_arlock,
    input  wire [3:0]                  s_axi_arcache,
    input  wire [2:0]                  s_axi_arprot,
    input  wire [3:0]                  s_axi_arqos,
    input  wire [3:0]                  s_axi_arregion,
    input  wire                        s_axi_arvalid,
    output wire                        s_axi_arready,
    // Read data
    output wire [ID_WIDTH-1:0]         s_axi_rid,
    output wire [DATA_WIDTH-1:0]       s_axi_rdata,
    output wire [1:0]                  s_axi_rresp,
    output wire                        s_axi_rlast,
    output wire                        s_axi_rvalid,
    input  wire                        s_axi_rready,

    // ---------------------------------------------------------
    // AXI4 Master interface — data movement port
    // Reads from SRC_ADDR, writes to DST_ADDR
    // ---------------------------------------------------------
    // Master write address
    output reg  [ID_WIDTH-1:0]         m_axi_awid,
    output reg  [ADDR_WIDTH-1:0]       m_axi_awaddr,
    output reg  [7:0]                  m_axi_awlen,
    output reg  [2:0]                  m_axi_awsize,
    output reg  [1:0]                  m_axi_awburst,
    output reg                         m_axi_awlock,
    output reg  [3:0]                  m_axi_awcache,
    output reg  [2:0]                  m_axi_awprot,
    output reg  [3:0]                  m_axi_awqos,
    output reg  [3:0]                  m_axi_awregion,
    output reg                         m_axi_awvalid,
    input  wire                        m_axi_awready,
    // Master write data
    output reg  [DATA_WIDTH-1:0]       m_axi_wdata,
    output reg  [STRB_WIDTH-1:0]       m_axi_wstrb,
    output reg                         m_axi_wlast,
    output reg                         m_axi_wvalid,
    input  wire                        m_axi_wready,
    // Master write response
    input  wire [ID_WIDTH-1:0]         m_axi_bid,
    input  wire [1:0]                  m_axi_bresp,
    input  wire                        m_axi_bvalid,
    output reg                         m_axi_bready,
    // Master read address
    output reg  [ID_WIDTH-1:0]         m_axi_arid,
    output reg  [ADDR_WIDTH-1:0]       m_axi_araddr,
    output reg  [7:0]                  m_axi_arlen,
    output reg  [2:0]                  m_axi_arsize,
    output reg  [1:0]                  m_axi_arburst,
    output reg                         m_axi_arlock,
    output reg  [3:0]                  m_axi_arcache,
    output reg  [2:0]                  m_axi_arprot,
    output reg  [3:0]                  m_axi_arqos,
    output reg  [3:0]                  m_axi_arregion,
    output reg                         m_axi_arvalid,
    input  wire                        m_axi_arready,
    // Master read data
    input  wire [ID_WIDTH-1:0]         m_axi_rid,
    input  wire [DATA_WIDTH-1:0]       m_axi_rdata,
    input  wire [1:0]                  m_axi_rresp,
    input  wire                        m_axi_rlast,
    input  wire                        m_axi_rvalid,
    output reg                         m_axi_rready,

    // ---------------------------------------------------------
    // Interrupt output (active-high)
    // ---------------------------------------------------------
    output wire                        dma_irq_o
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam integer REG_DATA_WIDTH = 32;

    // AXI response codes
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // DMA CTRL bit fields
    localparam CTRL_ENABLE      = 0;
    localparam CTRL_START       = 1;
    localparam CTRL_IRQ_EN      = 2;
    localparam CTRL_DIR         = 3;   // 0 = mem-to-mem (master read+write)
    localparam CTRL_AUTO_RESTART= 4;

    // DMA STATUS bit fields
    localparam STATUS_BUSY      = 0;
    localparam STATUS_DONE      = 1;
    localparam STATUS_ERROR     = 2;

    // DMA IRQ_STATUS bit fields
    localparam IRQ_DONE         = 0;

    // DMA FSM states (data movement)
    localparam [2:0] DMA_IDLE      = 3'd0;
    localparam [2:0] DMA_READ_AR   = 3'd1;   // Issue read address
    localparam [2:0] DMA_READ_R    = 3'd2;   // Wait for read data
    localparam [2:0] DMA_WRITE_AW  = 3'd3;   // Issue write address
    localparam [2:0] DMA_WRITE_W   = 3'd4;   // Issue write data
    localparam [2:0] DMA_WRITE_B   = 3'd5;   // Wait for write response
    localparam [2:0] DMA_DONE      = 3'd6;

    // ---------------------------------------------------------
    // Address validation (combinational) — slave port
    // ---------------------------------------------------------
    wire [ADDR_WIDTH-1:0] aw_offset = s_axi_awaddr - BASE_ADDR;
    wire [ADDR_WIDTH-1:0] ar_offset = s_axi_araddr - BASE_ADDR;

    wire aw_in_aperture = (aw_offset[ADDR_WIDTH-1:REG_ADDR_WIDTH] == 0);
    wire ar_in_aperture = (ar_offset[ADDR_WIDTH-1:REG_ADDR_WIDTH] == 0);

    wire aw_aligned = (s_axi_awaddr[1:0] == 2'b00);
    wire ar_aligned = (s_axi_araddr[1:0] == 2'b00);

    wire aw_burst = (s_axi_awlen != 8'd0);
    wire ar_burst = (s_axi_arlen != 8'd0);

    wire [3:0] aw_reg_sel = aw_offset[5:2];
    wire [3:0] ar_reg_sel = ar_offset[5:2];

    wire aw_addr_dec_err = !aw_in_aperture;
    wire aw_addr_slv_err = aw_in_aperture && !aw_aligned;
    wire ar_addr_dec_err = !ar_in_aperture;
    wire ar_addr_slv_err = ar_in_aperture && !ar_aligned;

    // =========================================================================
    // DMA control registers (slave-programmable)
    // =========================================================================
    reg                    dma_enable;
    reg                    dma_start_pending;   // set when CTRL.start written, cleared when DMA starts
    reg                    dma_irq_en;
    reg                    dma_dir;        // 0 = mem-to-mem
    reg                    dma_auto_restart;

    reg [ADDR_WIDTH-1:0]   dma_src_addr;
    reg [ADDR_WIDTH-1:0]   dma_dst_addr;
    reg [7:0]              dma_len;        // number of beats

    reg                    dma_busy;
    reg                    dma_done;
    reg                    dma_error;

    reg                    dma_done_irq;   // latched, W1C

    // ---------------------------------------------------------
    // Data movement FSM (master side)
    // ---------------------------------------------------------
    reg [2:0]              dma_state;

    // Beat tracking
    reg [7:0]              beats_remaining;
    reg [ADDR_WIDTH-1:0]   cur_src_addr;
    reg [ADDR_WIDTH-1:0]   cur_dst_addr;

    // Data buffer between read and write
    reg [DATA_WIDTH-1:0]   dma_data_buf;
    reg                    dma_buf_valid;

    // Master handshake helpers
    wire m_aw_handshake = m_axi_awvalid & m_axi_awready;
    wire m_w_handshake  = m_axi_wvalid  & m_axi_wready;
    wire m_ar_handshake = m_axi_arvalid & m_axi_arready;
    wire m_r_handshake  = m_axi_rvalid  & m_axi_rready;
    wire m_b_handshake  = m_axi_bvalid  & m_axi_bready;

    // ---------------------------------------------------------
    // Slave write channel FSM
    // ---------------------------------------------------------
    reg [2:0]              s_wr_state;
    reg [ID_WIDTH-1:0]     s_wr_id_r;
    reg [3:0]              s_wr_reg_sel_r;
    reg [1:0]              s_wr_error_r;
    reg [31:0]             s_wr_data_r;
    reg [3:0]              s_wr_strb_r;
    reg [1:0]              s_wr_resp_r;
    reg                    s_b_valid_r;

    localparam [2:0] S_WR_IDLE    = 3'd0;
    localparam [2:0] S_WR_WAIT_W  = 3'd1;
    localparam [2:0] S_WR_WAIT_AW = 3'd2;
    localparam [2:0] S_WR_DRAIN   = 3'd3;
    localparam [2:0] S_WR_RESP    = 3'd4;

    // ---------------------------------------------------------
    // Slave read channel FSM
    // ---------------------------------------------------------
    reg [1:0]              s_rd_state;
    reg [ID_WIDTH-1:0]     s_rd_id_r;
    reg [DATA_WIDTH-1:0]   s_rd_data_r;
    reg [1:0]              s_rd_resp_r;
    reg                    s_rd_last_r;
    reg                    s_r_valid_r;

    localparam [1:0] S_RD_IDLE  = 2'd0;
    localparam [1:0] S_RD_RESP  = 2'd1;

    // =========================================================================
    // Combinational: slave write response
    // =========================================================================
    function [1:0] s_compute_write_resp;
        input [3:0] reg_sel;
        begin
            case (reg_sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5:
                    s_compute_write_resp = RESP_OKAY;   // CTRL, SRC, DST, LEN, STATUS(W1C), IRQ_STATUS(W1C)
                4'h6, 4'h7:
                    s_compute_write_resp = RESP_SLVERR; // RO registers (ID, VERSION)
                default:
                    s_compute_write_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    function [1:0] s_compute_read_resp;
        input [3:0] reg_sel;
        begin
            case (reg_sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6, 4'h7:
                    s_compute_read_resp = RESP_OKAY;
                default:
                    s_compute_read_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    // =========================================================================
    // Slave outputs (defaults)
    // =========================================================================
    assign s_axi_awready = s_axi_awvalid &&
                          (s_wr_state == S_WR_IDLE || s_wr_state == S_WR_WAIT_AW);

    assign s_axi_wready = s_axi_wvalid &&
                         (s_wr_state == S_WR_IDLE ||
                          s_wr_state == S_WR_WAIT_W ||
                          s_wr_state == S_WR_DRAIN);

    assign s_axi_arready = s_axi_arvalid && (s_rd_state == S_RD_IDLE);

    assign s_axi_bid    = s_wr_id_r;
    assign s_axi_bresp  = s_wr_resp_r;
    assign s_axi_bvalid = s_b_valid_r;

    assign s_axi_rid    = s_rd_id_r;
    assign s_axi_rdata  = s_rd_data_r;
    assign s_axi_rresp  = s_rd_resp_r;
    assign s_axi_rlast  = s_rd_last_r;
    assign s_axi_rvalid = s_r_valid_r;

    // =========================================================================
    // Slave write FSM
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            $display("DEBUG_DMA: BASE_ADDR = %h", BASE_ADDR);
            s_wr_state     <= S_WR_IDLE;
            s_wr_id_r      <= '0;
            s_wr_reg_sel_r <= '0;
            s_wr_error_r   <= RESP_OKAY;
            s_wr_data_r    <= '0;
            s_wr_strb_r    <= '0;
            s_wr_resp_r    <= RESP_OKAY;
            s_b_valid_r    <= 1'b0;

            dma_enable     <= 1'b0;
            dma_start_pending <= 1'b0;
            dma_irq_en     <= 1'b0;
            dma_dir        <= 1'b0;
            dma_auto_restart <= 1'b0;
            dma_src_addr   <= '0;
            dma_dst_addr   <= '0;
            dma_len        <= '0;
            dma_done_irq   <= 1'b0;
        end else begin
            // Default: clear W1C strobes (single-cycle pulses)
            // dma_start_pending is cleared when DMA starts (in DMA_IDLE)

            case (s_wr_state)
                // -------------------------------------------------------------
                // WR_IDLE: waiting for AW and/or W (independent arrival)
                // -------------------------------------------------------------
                S_WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready &&
                        s_axi_wvalid  && s_axi_wready && !aw_burst) begin
                        $display("DEBUG_DMA_WRITE: awaddr=0x%0h BASE=0x%0h aw_offset=0x%0h aw_in_aperture=%b aw_aligned=%b",
                                 s_axi_awaddr, BASE_ADDR, aw_offset, aw_in_aperture, aw_aligned);
                        s_wr_id_r      <= s_axi_awid;
                        s_wr_reg_sel_r <= aw_reg_sel;

                        if (!aw_in_aperture) begin
                            s_wr_error_r   <= RESP_DECERR;
                            s_wr_resp_r    <= RESP_DECERR;
                        end else if (!aw_aligned) begin
                            s_wr_error_r   <= RESP_SLVERR;
                            s_wr_resp_r    <= RESP_SLVERR;
                        end else begin
                            s_wr_error_r   <= RESP_OKAY;
                            s_wr_resp_r    <= s_compute_write_resp(aw_reg_sel);
                        end

                        // Handle register writes (only if no address error)
                        if (aw_in_aperture && aw_aligned) begin
                            case (aw_reg_sel)
                                4'h0: begin // DMA_CTRL — RW
                                    if (s_axi_wstrb[0]) begin
                                        dma_enable        <= s_axi_wdata[0]  & 1'b1;
                                        dma_start_pending <= s_axi_wdata[1]  & 1'b1;
                                        dma_irq_en        <= s_axi_wdata[2]  & 1'b1;
                                        dma_dir           <= s_axi_wdata[3]  & 1'b1;
                                        dma_auto_restart  <= s_axi_wdata[4]  & 1'b1;
                                    end
                                end
                                4'h1: begin // DMA_SRC_ADDR
                                    if (s_axi_wstrb[0]) dma_src_addr[7:0]   <= s_axi_wdata[7:0];
                                    if (s_axi_wstrb[1]) dma_src_addr[15:8]  <= s_axi_wdata[15:8];
                                    if (s_axi_wstrb[2]) dma_src_addr[23:16] <= s_axi_wdata[23:16];
                                    if (s_axi_wstrb[3]) dma_src_addr[31:24] <= s_axi_wdata[31:24];
                                end
                                4'h2: begin // DMA_DST_ADDR
                                    if (s_axi_wstrb[0]) dma_dst_addr[7:0]   <= s_axi_wdata[7:0];
                                    if (s_axi_wstrb[1]) dma_dst_addr[15:8]  <= s_axi_wdata[15:8];
                                    if (s_axi_wstrb[2]) dma_dst_addr[23:16] <= s_axi_wdata[23:16];
                                    if (s_axi_wstrb[3]) dma_dst_addr[31:24] <= s_axi_wdata[31:24];
                                end
                                4'h3: begin // DMA_LEN
                                    if (s_axi_wstrb[0]) dma_len <= s_axi_wdata[7:0];
                                end
                                4'h4: begin // DMA_STATUS — W1C (clear done/error)
                                    if (s_axi_wstrb[0]) begin
                                        if (s_axi_wdata[1]) dma_done   <= 1'b0;
                                        if (s_axi_wdata[2]) dma_error  <= 1'b0;
                                    end
                                end
                                4'h5: begin // DMA_IRQ_STATUS — W1C (clear done_irq)
                                    if (s_axi_wstrb[0]) begin
                                        if (s_axi_wdata[0]) dma_done_irq <= 1'b0;
                                    end
                                end
                                default: ; // no-op
                            endcase
                        end
                        s_wr_state  <= S_WR_RESP;
                        s_b_valid_r <= 1'b1;
                    end
                    else if (s_axi_awvalid && s_axi_awready && aw_burst) begin
                        s_wr_state     <= S_WR_DRAIN;
                        s_wr_id_r      <= s_axi_awid;
                        s_wr_error_r   <= RESP_SLVERR;
                        s_wr_reg_sel_r <= aw_reg_sel;
                    end
                    else if (s_axi_awvalid && s_axi_awready) begin
                        // AW arrived (single), W not yet
                        s_wr_state     <= S_WR_WAIT_W;
                        s_wr_id_r      <= s_axi_awid;
                        s_wr_reg_sel_r <= aw_reg_sel;
                        if (!aw_in_aperture) begin
                            s_wr_error_r   <= RESP_DECERR;
                            s_wr_resp_r    <= RESP_DECERR;
                        end else if (!aw_aligned) begin
                            s_wr_error_r   <= RESP_SLVERR;
                            s_wr_resp_r    <= RESP_SLVERR;
                        end else begin
                            s_wr_error_r   <= RESP_OKAY;
                            s_wr_resp_r    <= s_compute_write_resp(aw_reg_sel);
                        end
                    end
                    else if (s_axi_wvalid && s_axi_wready) begin
                        // W arrived before AW
                        s_wr_state  <= S_WR_WAIT_AW;
                        s_wr_data_r <= s_axi_wdata[31:0];
                        s_wr_strb_r <= s_axi_wstrb[3:0];
                    end
                end

                // -------------------------------------------------------------
                // WR_WAIT_W: AW accepted (single), waiting for W
                // -------------------------------------------------------------
                S_WR_WAIT_W: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        s_wr_resp_r <= (s_wr_error_r == RESP_OKAY) ?
                                      s_compute_write_resp(s_wr_reg_sel_r) : s_wr_error_r;

                        if (s_wr_error_r == RESP_OKAY) begin
                            case (s_wr_reg_sel_r)
                                4'h0: begin // DMA_CTRL — RW
                                    if (s_wr_strb_r[0]) begin
                                        dma_enable        <= s_wr_data_r[0]  & 1'b1;
                                        dma_start_pending <= s_wr_data_r[1]  & 1'b1;
                                        dma_irq_en        <= s_wr_data_r[2]  & 1'b1;
                                        dma_dir           <= s_wr_data_r[3]  & 1'b1;
                                        dma_auto_restart  <= s_wr_data_r[4]  & 1'b1;
                                    end
                                end
                                4'h1: begin // DMA_SRC_ADDR
                                    if (s_wr_strb_r[0]) dma_src_addr[7:0]   <= s_wr_data_r[7:0];
                                    if (s_wr_strb_r[1]) dma_src_addr[15:8]  <= s_wr_data_r[15:8];
                                    if (s_wr_strb_r[2]) dma_src_addr[23:16] <= s_wr_data_r[23:16];
                                    if (s_wr_strb_r[3]) dma_src_addr[31:24] <= s_wr_data_r[31:24];
                                end
                                4'h2: begin // DMA_DST_ADDR
                                    if (s_wr_strb_r[0]) dma_dst_addr[7:0]   <= s_wr_data_r[7:0];
                                    if (s_wr_strb_r[1]) dma_dst_addr[15:8]  <= s_wr_data_r[15:8];
                                    if (s_wr_strb_r[2]) dma_dst_addr[23:16] <= s_wr_data_r[23:16];
                                    if (s_wr_strb_r[3]) dma_dst_addr[31:24] <= s_wr_data_r[31:24];
                                end
                                4'h3: begin // DMA_LEN
                                    if (s_wr_strb_r[0]) dma_len <= s_wr_data_r[7:0];
                                end
                                4'h4: begin // DMA_STATUS — W1C
                                    if (s_wr_strb_r[0]) begin
                                        if (s_wr_data_r[1]) dma_done   <= 1'b0;
                                        if (s_wr_data_r[2]) dma_error  <= 1'b0;
                                    end
                                end
                                4'h5: begin // DMA_IRQ_STATUS — W1C
                                    if (s_wr_strb_r[0]) begin
                                        if (s_wr_data_r[0]) dma_done_irq <= 1'b0;
                                    end
                                end
                                default: ;
                            endcase
                        end
                        s_wr_state  <= S_WR_RESP;
                        s_b_valid_r <= 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // WR_WAIT_AW: W accepted (single), waiting for AW
                // -------------------------------------------------------------
                S_WR_WAIT_AW: begin
                    if (s_axi_awvalid && s_axi_awready) begin
                        s_wr_id_r      <= s_axi_awid;
                        s_wr_reg_sel_r <= aw_reg_sel;
                        if (aw_burst) begin
                            s_wr_state   <= S_WR_DRAIN;
                            s_wr_error_r <= RESP_SLVERR;
                        end else begin
                            if (!aw_in_aperture) begin
                                s_wr_error_r <= RESP_DECERR;
                                s_wr_resp_r  <= RESP_DECERR;
                            end else if (!aw_aligned) begin
                                s_wr_error_r <= RESP_SLVERR;
                                s_wr_resp_r  <= RESP_SLVERR;
                            end else begin
                                s_wr_error_r <= RESP_OKAY;
                                s_wr_resp_r  <= s_compute_write_resp(aw_reg_sel);
                            end
                            if (aw_in_aperture && aw_aligned) begin
                                case (aw_reg_sel)
                                    4'h0: begin // DMA_CTRL
                                        if (s_wr_strb_r[0]) begin
                                            dma_enable        <= s_wr_data_r[0]  & 1'b1;
                                            dma_start_pending <= s_wr_data_r[1]  & 1'b1;
                                            dma_irq_en        <= s_wr_data_r[2]  & 1'b1;
                                            dma_dir           <= s_wr_data_r[3]  & 1'b1;
                                            dma_auto_restart  <= s_wr_data_r[4]  & 1'b1;
                                        end
                                    end
                                    4'h1: begin // DMA_SRC_ADDR
                                        if (s_wr_strb_r[0]) dma_src_addr[7:0]   <= s_wr_data_r[7:0];
                                        if (s_wr_strb_r[1]) dma_src_addr[15:8]  <= s_wr_data_r[15:8];
                                        if (s_wr_strb_r[2]) dma_src_addr[23:16] <= s_wr_data_r[23:16];
                                        if (s_wr_strb_r[3]) dma_src_addr[31:24] <= s_wr_data_r[31:24];
                                    end
                                    4'h2: begin // DMA_DST_ADDR
                                        if (s_wr_strb_r[0]) dma_dst_addr[7:0]   <= s_wr_data_r[7:0];
                                        if (s_wr_strb_r[1]) dma_dst_addr[15:8]  <= s_wr_data_r[15:8];
                                        if (s_wr_strb_r[2]) dma_dst_addr[23:16] <= s_wr_data_r[23:16];
                                        if (s_wr_strb_r[3]) dma_dst_addr[31:24] <= s_wr_data_r[31:24];
                                    end
                                    4'h3: begin // DMA_LEN
                                        if (s_wr_strb_r[0]) dma_len <= s_wr_data_r[7:0];
                                    end
                                    4'h4: begin // DMA_STATUS — W1C
                                        if (s_wr_strb_r[0]) begin
                                            if (s_wr_data_r[1]) dma_done   <= 1'b0;
                                            if (s_wr_data_r[2]) dma_error  <= 1'b0;
                                        end
                                    end
                                    4'h5: begin // DMA_IRQ_STATUS — W1C
                                        if (s_wr_strb_r[0]) begin
                                            if (s_wr_data_r[0]) dma_done_irq <= 1'b0;
                                        end
                                    end
                                    default: ;
                                endcase
                            end
                            s_wr_state  <= S_WR_RESP;
                            s_b_valid_r <= 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // WR_DRAIN: burst write, accept all W beats until WLAST
                // -------------------------------------------------------------
                S_WR_DRAIN: begin
                    if (s_axi_wvalid && s_axi_wready && s_axi_wlast) begin
                        s_wr_state  <= S_WR_RESP;
                        s_wr_resp_r <= RESP_SLVERR;
                        s_b_valid_r <= 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // WR_RESP: B response pending, wait for BREADY
                // -------------------------------------------------------------
                S_WR_RESP: begin
                    if (s_b_valid_r && s_axi_bready) begin
                        s_b_valid_r <= 1'b0;
                        s_wr_state  <= S_WR_IDLE;
                    end
                end

                default: s_wr_state <= S_WR_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Slave read FSM
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s_rd_state  <= S_RD_IDLE;
            s_rd_id_r   <= '0;
            s_rd_data_r <= '0;
            s_rd_resp_r <= RESP_OKAY;
            s_rd_last_r <= 1'b0;
            s_r_valid_r <= 1'b0;
        end else begin
            case (s_rd_state)
                S_RD_IDLE: begin
                    if (s_axi_arvalid && s_axi_arready) begin
                        s_rd_id_r  <= s_axi_arid;

                        if (ar_burst) begin
                            s_rd_data_r <= {DATA_WIDTH{1'b0}};
                            s_rd_resp_r <= RESP_SLVERR;
                            s_r_valid_r <= 1'b1;
                            s_rd_last_r <= (s_axi_arlen == 8'd0);
                        end else begin
                            if (!ar_in_aperture) begin
                                s_rd_data_r <= {DATA_WIDTH{1'b0}};
                                s_rd_resp_r <= RESP_DECERR;
                            end else if (!ar_aligned) begin
                                s_rd_data_r <= {DATA_WIDTH{1'b0}};
                                s_rd_resp_r <= RESP_SLVERR;
                            end else begin
                                case (ar_reg_sel)
                                    4'h0:   s_rd_data_r = {28'b0, dma_error, dma_done, dma_busy,
                                                          dma_auto_restart, dma_dir, dma_irq_en, dma_start_pending, dma_enable};
                                    4'h1:   s_rd_data_r = {{(32-ADDR_WIDTH){1'b0}}, dma_src_addr};
                                    4'h2:   s_rd_data_r = {{(32-ADDR_WIDTH){1'b0}}, dma_dst_addr};
                                    4'h3:   s_rd_data_r = {24'b0, dma_len};
                                    4'h4:   s_rd_data_r = {29'b0, dma_error, dma_done, dma_busy};
                                    4'h5:   s_rd_data_r = {31'b0, dma_done_irq};
                                    4'h6:   s_rd_data_r = DMA_ID_VALUE;
                                    4'h7:   s_rd_data_r = DMA_VERSION_VALUE;
                                    default: s_rd_data_r = 32'h0;
                                endcase
                                s_rd_resp_r <= s_compute_read_resp(ar_reg_sel);
                            end
                            s_r_valid_r <= 1'b1;
                            s_rd_last_r <= 1'b1;
                        end
                        s_rd_state <= S_RD_RESP;
                    end
                end

                S_RD_RESP: begin
                    if (s_r_valid_r && s_axi_rready) begin
                        if (s_rd_last_r) begin
                            s_r_valid_r <= 1'b0;
                            s_rd_last_r <= 1'b0;
                            s_rd_state  <= S_RD_IDLE;
                        end
                    end
                end

                default: s_rd_state <= S_RD_IDLE;
            endcase
        end
    end

    // =========================================================================
    // DMA data movement FSM (master side)
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            dma_state       <= DMA_IDLE;
            beats_remaining <= '0;
            cur_src_addr    <= '0;
            cur_dst_addr    <= '0;
            dma_data_buf    <= '0;
            dma_buf_valid   <= 1'b0;
            dma_busy        <= 1'b0;
            dma_done        <= 1'b0;
            dma_error       <= 1'b0;
            m_axi_rready    = 1'b0;
            m_axi_bready    = 1'b0;
        end else begin
            // Default master signal deassertion (use blocking = for combinational output)
            m_axi_awvalid = 1'b0; m_axi_awid = '0; m_axi_awaddr = '0;
            m_axi_awlen = 8'd0; m_axi_awsize = 3'd3; m_axi_awburst = 2'b01;
            m_axi_awlock = 1'b0; m_axi_awcache = 4'b0; m_axi_awprot = 3'b0;
            m_axi_awqos = 4'b0; m_axi_awregion = 4'b0;

            m_axi_wvalid = 1'b0; m_axi_wdata = '0; m_axi_wstrb = '0;
            m_axi_wlast = 1'b0;

            m_axi_arvalid = 1'b0; m_axi_arid = '0; m_axi_araddr = '0;
            m_axi_arlen = 8'd0; m_axi_arsize = 3'd3; m_axi_arburst = 2'b01;
            m_axi_arlock = 1'b0; m_axi_arcache = 4'b0; m_axi_arprot = 3'b0;
            m_axi_arqos = 4'b0; m_axi_arregion = 4'b0;

            case (dma_state)
                DMA_IDLE: begin
                    dma_busy  <= 1'b0;
                    // dma_done is NOT cleared here - it stays set until cleared by SW via W1C
                    dma_error <= 1'b0;

                    if (dma_enable && dma_start_pending && (dma_len != 0)) begin
                        $display("DEBUG_DMA_START: state=IDLE->READ_AR, state=%0d, en=%b, start_pending=%b, len=%0d, src=0x%0h, dst=0x%0h",
                                 dma_state, dma_enable, dma_start_pending, dma_len, dma_src_addr, dma_dst_addr);
                        dma_state       <= DMA_READ_AR;
                        dma_busy        <= 1'b1;
                        beats_remaining <= dma_len;
                        cur_src_addr    <= dma_src_addr;
                        cur_dst_addr    <= dma_dst_addr;
                        dma_buf_valid   <= 1'b0;
                        dma_start_pending <= 1'b0;  // Clear start pending
                    end
                end

                // ---------------------------------------------------------
                // DMA_READ_AR: issue read address for next beat
                // ---------------------------------------------------------
                DMA_READ_AR: begin
                    m_axi_arvalid = 1'b1;
                    m_axi_arid    = 8'h10;
                    m_axi_araddr  = cur_src_addr;
                    m_axi_arlen   = 8'd0;
                    m_axi_arsize  = 3'd3;
                    m_axi_arburst = 2'b01;
                    m_axi_arlock  = 1'b0;
                    m_axi_arcache = 4'b0;
                    m_axi_arprot  = 3'b0;
                    m_axi_arqos   = 4'b0;
                    m_axi_arregion= 4'b0;

                    if (m_ar_handshake) begin
                        $display("DEBUG_DMA_READ_AR: addr=0x%0h, accepted", cur_src_addr);
                        dma_state <= DMA_READ_R;
                    end
                end

                // ---------------------------------------------------------
                // DMA_READ_R: wait for read data
                // ---------------------------------------------------------
                DMA_READ_R: begin
                    m_axi_rready = 1'b1;
                    if (m_r_handshake) begin
                        $display("DEBUG_DMA_READ_R: data=0x%0h", m_axi_rdata);
                        dma_data_buf  <= m_axi_rdata;
                        dma_buf_valid <= 1'b1;
                        dma_state     <= DMA_WRITE_AW;
                    end
                end

                // ---------------------------------------------------------
                // DMA_WRITE_AW: issue write address
                // ---------------------------------------------------------
                DMA_WRITE_AW: begin
                    m_axi_awvalid = 1'b1;
                    m_axi_awid    = 8'h20;
                    m_axi_awaddr  = cur_dst_addr;
                    m_axi_awlen   = 8'd0;
                    m_axi_awsize  = 3'd3;
                    m_axi_awburst = 2'b01;
                    m_axi_awlock  = 1'b0;
                    m_axi_awcache = 4'b0;
                    m_axi_awprot  = 3'b0;
                    m_axi_awqos   = 4'b0;
                    m_axi_awregion= 4'b0;

                    if (m_aw_handshake) begin
                        $display("DEBUG_DMA_WRITE_AW: addr=0x%0h, accepted", cur_dst_addr);
                        dma_state <= DMA_WRITE_W;
                    end
                end

                // ---------------------------------------------------------
                // DMA_WRITE_W: issue write data
                // ---------------------------------------------------------
                DMA_WRITE_W: begin
                    m_axi_wvalid = 1'b1;
                    m_axi_wdata  = dma_data_buf;
                    m_axi_wstrb  = {STRB_WIDTH{1'b1}};
                    m_axi_wlast  = 1'b1;

                    if (m_w_handshake) begin
                        $display("DEBUG_DMA_WRITE_W: data=0x%0h, accepted", dma_data_buf);
                        dma_state <= DMA_WRITE_B;
                    end
                end

                // ---------------------------------------------------------
                // DMA_WRITE_B: wait for write response
                // ---------------------------------------------------------
                DMA_WRITE_B: begin
                    m_axi_bready = 1'b1;
                    if (m_b_handshake) begin
                        $display("DEBUG_DMA_WRITE_B: resp=0x%0h", m_axi_bresp);
                        if (m_axi_bresp != 2'b00) begin
                            dma_error <= 1'b1;
                            dma_state <= DMA_DONE;
                        end else begin
                            beats_remaining <= beats_remaining - 1'b1;
                            cur_src_addr    <= cur_src_addr + DATA_WIDTH/8;
                            cur_dst_addr    <= cur_dst_addr + DATA_WIDTH/8;
                            dma_buf_valid   <= 1'b0;

                            if (beats_remaining == 1) begin
                                dma_state <= DMA_DONE;
                            end else begin
                                dma_state <= DMA_READ_AR;
                            end
                        end
                    end
                end

                // ---------------------------------------------------------
                // DMA_DONE: transfer complete
                // ---------------------------------------------------------
                DMA_DONE: begin
                    dma_busy <= 1'b0;
                    dma_done <= 1'b1;
                    $display("DEBUG_DMA_DONE: state=DMA_DONE, beats_remaining=%0d, dma_done=%b, dma_enable=%b", beats_remaining, dma_done, dma_enable);

                    if (dma_irq_en)
                        dma_done_irq <= 1'b1;

                    if (!dma_enable || !dma_start_pending) begin
                        dma_state <= DMA_IDLE;
                    end else if (dma_auto_restart && (dma_len != 0)) begin
                        dma_state       <= DMA_READ_AR;
                        dma_busy        <= 1'b1;
                        beats_remaining <= dma_len;
                        cur_src_addr    <= dma_src_addr;
                        cur_dst_addr    <= dma_dst_addr;
                        dma_buf_valid   <= 1'b0;
                        dma_done        <= 1'b0;
                    end
                end

                default: dma_state <= DMA_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Interrupt output
    // =========================================================================
    assign dma_irq_o = dma_done_irq & dma_irq_en;

endmodule

`default_nettype wire