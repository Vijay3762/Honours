// =============================================================================
// File    : axi_cnn_conv.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Parameterized AXI4 CNN Convolution Accelerator IP block.
//
//           Implements a 3 × 3 convolution with 9 parallel multipliers,
//           a 4-level adder tree, and bias addition. Delivers one output
//           pixel per clock cycle when running.
//
//           The AXI4 slave interface matches the existing SoC interconnect
//           (64-bit data, 32-bit address, 8-bit ID, 8-bit strobe).
//
// Register map (32-bit registers, word-aligned, 4 KiB aperture):
//   Offset  Register         Access  Description
//   ------  ---------------  ------  ----------------------------------------
//   0x00    CNN_CTRL         RW      bit0: start, bit1: irq_enable,
//                                   bit2: reset_acc
//   0x04    CNN_STATUS       RO      bit0: done, bit1: busy
//   0x08    CNN_IN_BASE      RW      Base address of input image in DMEM
//   0x0C    CNN_OUT_BASE     RW      Base address of output feature map
//   0x10    CNN_IMG_WIDTH    RW      Image width in pixels
//   0x14    CNN_IMG_HEIGHT   RW      Image height in pixels
//   0x18    CNN_K00 .. K08   RW      9 kernel weights (K00 at 0x18, K08 at 0x38)
//   0x3C    CNN_BIAS         RW      Bias value added to convolution result
//   0x40    CNN_ID           RO      Fixed peripheral ID ("CNNV")
//   0x44    CNN_VERSION      RO      Fixed version
//
// Reset: active-high (rst).
//   On reset: all registers cleared, idle state, IRQ=0.
//
// AXI response codes:
//   OKAY   = 2'b00  -- valid register access
//   SLVERR = 2'b10  -- reserved offset, write to RO register, or misaligned
//   DECERR = 2'b11  -- address outside the valid 4 KiB register aperture
//
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_cnn_conv #(
    parameter integer DATA_WIDTH = 32,

    // AXI bus widths -- match the interconnect defaults
    parameter integer AXI_DATA_WIDTH  = 64,
    parameter integer AXI_ADDR_WIDTH  = 32,
    parameter integer AXI_ID_WIDTH    = 8,
    parameter integer AXI_STRB_WIDTH  = AXI_DATA_WIDTH / 8,

    // Register aperture: 4 KiB (12 address bits)
    parameter integer REG_ADDR_WIDTH  = 12,

    // Base address of this CNN slave in the system memory map
    parameter [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h0,

    // Fixed peripheral ID and version constants
    parameter [31:0] CNN_ID_VALUE      = 32'h434E4E56,  // "CNNV" in ASCII
    parameter [31:0] CNN_VERSION_VALUE = 32'h00010000   // v1.0
)(
    input  wire                            clk,
    input  wire                            rst,          // active-high

    // AXI4 slave interface
    // Write address channel
    input  wire [AXI_ID_WIDTH-1:0]         s_axi_awid,
    input  wire [AXI_ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [7:0]                      s_axi_awlen,
    input  wire [2:0]                      s_axi_awsize,
    input  wire [1:0]                      s_axi_awburst,
    input  wire                            s_axi_awlock,
    input  wire [3:0]                      s_axi_awcache,
    input  wire [2:0]                      s_axi_awprot,
    input  wire [3:0]                      s_axi_awqos,
    input  wire [3:0]                      s_axi_awregion,
    input  wire                            s_axi_awvalid,
    output wire                            s_axi_awready,

    // Write data channel
    input  wire [AXI_DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [AXI_STRB_WIDTH-1:0]       s_axi_wstrb,
    input  wire                            s_axi_wlast,
    input  wire                            s_axi_wvalid,
    output wire                            s_axi_wready,

    // Write response channel
    output wire [AXI_ID_WIDTH-1:0]         s_axi_bid,
    output wire [1:0]                      s_axi_bresp,
    output wire                            s_axi_bvalid,
    input  wire                            s_axi_bready,

    // Read address channel
    input  wire [AXI_ID_WIDTH-1:0]         s_axi_arid,
    input  wire [AXI_ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [7:0]                      s_axi_arlen,
    input  wire [2:0]                      s_axi_arsize,
    input  wire [1:0]                      s_axi_arburst,
    input  wire                            s_axi_arlock,
    input  wire [3:0]                      s_axi_arcache,
    input  wire [2:0]                      s_axi_arprot,
    input  wire [3:0]                      s_axi_arqos,
    input  wire [3:0]                      s_axi_arregion,
    input  wire                            s_axi_arvalid,
    output wire                            s_axi_arready,

    // Read data channel
    output wire [AXI_ID_WIDTH-1:0]         s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0]       s_axi_rdata,
    output wire [1:0]                      s_axi_rresp,
    output wire                            s_axi_rlast,
    output wire                            s_axi_rvalid,
    input  wire                            s_axi_rready,

    // Convolution outputs
    output wire                            cnn_irq_o     // interrupt (active-high)
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam integer REG_DATA_WIDTH = 32;

    // Register select offsets (word index = byte_offset / 4)
    localparam [4:0] REG_CTRL     = 5'h00;
    localparam [4:0] REG_STATUS   = 5'h01;
    localparam [4:0] REG_IN_BASE  = 5'h02;
    localparam [4:0] REG_OUT_BASE = 5'h03;
    localparam [4:0] REG_IMG_W    = 5'h04;
    localparam [4:0] REG_IMG_H    = 5'h05;
    localparam [4:0] REG_K00      = 5'h06;
    localparam [4:0] REG_K01      = 5'h07;
    localparam [4:0] REG_K02      = 5'h08;
    localparam [4:0] REG_K10      = 5'h09;
    localparam [4:0] REG_K11      = 5'h0A;
    localparam [4:0] REG_K12      = 5'h0B;
    localparam [4:0] REG_K20      = 5'h0C;
    localparam [4:0] REG_K21      = 5'h0D;
    localparam [4:0] REG_K22      = 5'h0E;
    localparam [4:0] REG_BIAS     = 5'h0F;
    localparam [4:0] REG_ID       = 5'h10;
    localparam [4:0] REG_VERSION  = 5'h11;

    // AXI response codes
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // Write FSM states
    localparam [2:0] WR_IDLE    = 3'd0;
    localparam [2:0] WR_WAIT_W  = 3'd1;
    localparam [2:0] WR_WAIT_AW = 3'd2;
    localparam [2:0] WR_DRAIN   = 3'd3;
    localparam [2:0] WR_RESP    = 3'd4;

    // Read FSM states
    localparam [1:0] RD_IDLE  = 2'd0;
    localparam [1:0] RD_RESP  = 2'd1;

    // =========================================================================
    // Address validation (combinational)
    // =========================================================================
    wire [AXI_ADDR_WIDTH-1:0] aw_offset = s_axi_awaddr - BASE_ADDR;
    wire [AXI_ADDR_WIDTH-1:0] ar_offset = s_axi_araddr - BASE_ADDR;

    wire aw_in_aperture = (aw_offset[AXI_ADDR_WIDTH-1:REG_ADDR_WIDTH] == 0);
    wire ar_in_aperture = (ar_offset[AXI_ADDR_WIDTH-1:REG_ADDR_WIDTH] == 0);

    wire aw_aligned = (s_axi_awaddr[1:0] == 2'b00);
    wire ar_aligned = (s_axi_araddr[1:0] == 2'b00);

    wire aw_burst = (s_axi_awlen != 8'd0);
    wire ar_burst = (s_axi_arlen != 8'd0);

    // Register select from word offset (bits [6:2] of offset address)
    wire [4:0] aw_reg_sel = aw_offset[6:2];
    wire [4:0] ar_reg_sel = ar_offset[6:2];

    wire aw_addr_error = !aw_in_aperture || !aw_aligned;
    wire ar_addr_error = !ar_in_aperture || !ar_aligned;

    // Pixel input registers (0x48-0x64, words 18-26)
    localparam [4:0] REG_PX00 = 5'h12;
    localparam [4:0] REG_PX01 = 5'h13;
    localparam [4:0] REG_PX02 = 5'h14;
    localparam [4:0] REG_PX10 = 5'h15;
    localparam [4:0] REG_PX11 = 5'h16;
    localparam [4:0] REG_PX12 = 5'h17;
    localparam [4:0] REG_PX20 = 5'h18;
    localparam [4:0] REG_PX21 = 5'h19;
    localparam [4:0] REG_PX22 = 5'h1A;
    localparam [4:0] REG_RESULT = 5'h1B;

    // =========================================================================
    // Combinational: determine write response for a given reg_sel
    // =========================================================================
    function [1:0] compute_write_resp;
        input [4:0] reg_sel;
        begin
            case (reg_sel)
                REG_CTRL, REG_IN_BASE, REG_OUT_BASE, REG_IMG_W, REG_IMG_H,
                REG_K00, REG_K01, REG_K02, REG_K10, REG_K11, REG_K12,
                REG_K20, REG_K21, REG_K22, REG_BIAS,
                REG_PX00, REG_PX01, REG_PX02, REG_PX10, REG_PX11, REG_PX12,
                REG_PX20, REG_PX21, REG_PX22:
                    compute_write_resp = RESP_OKAY;
                REG_STATUS, REG_ID, REG_VERSION:
                    compute_write_resp = RESP_SLVERR; // RO registers
                default:
                    compute_write_resp = RESP_SLVERR; // reserved
            endcase
        end
    endfunction

    // =========================================================================
    // Combinational: determine read response for a given reg_sel
    // =========================================================================
    function [1:0] compute_read_resp;
        input [4:0] reg_sel;
        begin
            case (reg_sel)
                REG_CTRL, REG_STATUS, REG_IN_BASE, REG_OUT_BASE, REG_IMG_W,
                REG_IMG_H, REG_K00, REG_K01, REG_K02, REG_K10, REG_K11,
                REG_K12, REG_K20, REG_K21, REG_K22, REG_BIAS, REG_ID,
                REG_VERSION, REG_PX00, REG_PX01, REG_PX02, REG_PX10,
                REG_PX11, REG_PX12, REG_PX20, REG_PX21, REG_PX22,
                REG_BIAS, REG_ID, REG_VERSION, REG_RESULT:
                    compute_read_resp = RESP_OKAY;
                default:
                    compute_read_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    // =========================================================================
    // Control / status registers
    // =========================================================================
    reg                   cnn_start;      // CTRL bit0
    reg                   cnn_irq_en;     // CTRL bit1
    reg                   cnn_reset_acc;  // CTRL bit2
    reg                   cnn_done;       // STATUS bit0
    reg                   cnn_busy;       // STATUS bit1
    reg                   cnn_irq;        // interrupt output

    // Configuration registers
    reg [31:0] cnn_in_base;       // input image base address in DMEM
    reg [31:0] cnn_out_base;      // output feature map base address
    reg [31:0] cnn_img_width;     // image width
    reg [31:0] cnn_img_height;    // image height

    // Kernel weights (9) + bias
    reg [31:0] cnn_k00, cnn_k01, cnn_k02;
    reg [31:0] cnn_k10, cnn_k11, cnn_k12;
    reg [31:0] cnn_k20, cnn_k21, cnn_k22;
    reg [31:0] cnn_bias;

    // Pixel input registers (3x3 input window)
    reg [31:0] cnn_px00, cnn_px01, cnn_px02;
    reg [31:0] cnn_px10, cnn_px11, cnn_px12;
    reg [31:0] cnn_px20, cnn_px21, cnn_px22;

    // Connect pixel registers to convolution engine
    wire [31:0] px00 = cnn_px00;
    wire [31:0] px01 = cnn_px01;
    wire [31:0] px02 = cnn_px02;
    wire [31:0] px10 = cnn_px10;
    wire [31:0] px11 = cnn_px11;
    wire [31:0] px12 = cnn_px12;
    wire [31:0] px20 = cnn_px20;
    wire [31:0] px21 = cnn_px21;
    wire [31:0] px22 = cnn_px22;

    // Interrupt output
    assign cnn_irq_o = cnn_irq & cnn_irq_en & cnn_done;

    // =========================================================================
    // Convolution engine
    // =========================================================================
    // Internal state for running convolution
    reg [31:0] conv_in_base;
    reg [31:0] conv_out_base;
    reg [31:0] conv_img_w;
    reg [31:0] conv_img_h;
    reg [31:0] conv_k00, conv_k01, conv_k02;
    reg [31:0] conv_k10, conv_k11, conv_k12;
    reg [31:0] conv_k20, conv_k21, conv_k22;
    reg [31:0] conv_bias;
    reg [31:0] cnn_result_reg;    // latched convolution result for readback

    reg [31:0] conv_x, conv_y;        // current output pixel coordinates
    reg [31:0] conv_result;           // accumulated convolution result
    reg [31:0] conv_acc;              // adder tree accumulator
    reg        conv_running;          // convolution in progress
    reg        conv_done_pulse;       // one-cycle done pulse

    // 9 parallel multiplier products
    wire [63:0] prod00, prod01, prod02;
    wire [63:0] prod10, prod11, prod12;
    wire [63:0] prod20, prod21, prod22;

    assign prod00 = px00 * conv_k00;
    assign prod01 = px01 * conv_k01;
    assign prod02 = px02 * conv_k02;
    assign prod10 = px10 * conv_k10;
    assign prod11 = px11 * conv_k11;
    assign prod12 = px12 * conv_k12;
    assign prod20 = px20 * conv_k20;
    assign prod21 = px21 * conv_k21;
    assign prod22 = px22 * conv_k22;

    // 4-level adder tree
    wire [63:0] sum_l0 = prod00 + prod01;
    wire [63:0] sum_l1 = prod02 + prod10;
    wire [63:0] sum_l2 = prod11 + prod12;
    wire [63:0] sum_l3 = prod20 + prod21;
    wire [63:0] sum_m0 = sum_l0 + sum_l1;
    wire [63:0] sum_m1 = sum_l2 + sum_l3;
    wire [63:0] sum_m2 = prod22 + conv_bias;
    wire [63:0] sum_h0 = sum_m0 + sum_m1;
    wire [63:0] conv_sum = sum_h0 + sum_m2;

    // =========================================================================
    // Write channel FSM
    // =========================================================================
    reg [2:0]               wr_state;
    reg [AXI_ID_WIDTH-1:0]   wr_id_r;
    reg [4:0]                wr_reg_sel_r;
    reg [1:0]               wr_error_r;
    reg [31:0]              wr_data_r;
    reg [3:0]               wr_strb_r;
    reg [1:0]               wr_resp_r;
    reg                     b_valid_r;

    wire [31:0] w_data_lower = s_axi_wdata[31:0];
    wire [3:0]  w_strb_lower = s_axi_wstrb[3:0];

    assign s_axi_awready = s_axi_awvalid &&
                          (wr_state == WR_IDLE || wr_state == WR_WAIT_AW);
    assign s_axi_wready = s_axi_wvalid &&
                         (wr_state == WR_IDLE ||
                          wr_state == WR_WAIT_W ||
                          wr_state == WR_DRAIN);

    // Temporary registers
    reg [31:0] nxt_ctrl;

    always @(posedge clk) begin
        if (rst) begin
            wr_state     <= WR_IDLE;
            wr_id_r      <= '0;
            wr_reg_sel_r <= '0;
            wr_error_r   <= RESP_OKAY;
            wr_data_r    <= '0;
            wr_strb_r    <= '0;
            wr_resp_r    <= RESP_OKAY;
            b_valid_r    <= 1'b0;

            // Control registers
            cnn_start     <= 1'b0;
            cnn_irq_en    <= 1'b0;
            cnn_reset_acc <= 1'b0;
            cnn_done      <= 1'b0;
            cnn_busy      <= 1'b0;

            // Config registers
            cnn_in_base   <= '0;
            cnn_out_base  <= '0;
            cnn_img_width <= '0;
            cnn_img_height<= '0;

            // Kernel weights
            cnn_k00 <= '0; cnn_k01 <= '0; cnn_k02 <= '0;
            cnn_k10 <= '0; cnn_k11 <= '0; cnn_k12 <= '0;
            cnn_k20 <= '0; cnn_k21 <= '0; cnn_k22 <= '0;
            cnn_bias <= '0;
            cnn_result_reg <= '0;

            // Pixel input registers
            cnn_px00 <= '0; cnn_px01 <= '0; cnn_px02 <= '0;
            cnn_px10 <= '0; cnn_px11 <= '0; cnn_px12 <= '0;
            cnn_px20 <= '0; cnn_px21 <= '0; cnn_px22 <= '0;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready &&
                        s_axi_wvalid && s_axi_wready && !aw_burst) begin
                        wr_id_r      <= s_axi_awid;
                        wr_reg_sel_r <= aw_reg_sel;
                        if (!aw_in_aperture)
                            wr_error_r <= RESP_DECERR;
                        else if (!aw_aligned)
                            wr_error_r <= RESP_SLVERR;
                        else
                            wr_error_r <= RESP_OKAY;

                        wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                    compute_write_resp(aw_reg_sel) : wr_error_r;

                        if (wr_error_r == RESP_OKAY) begin
                            case (aw_reg_sel)
                                REG_CTRL: begin
                                    for (int i = 0; i < 4; i = i + 1) begin
                                        if (w_strb_lower[i]) begin
                                            case (i)
                                                0: cnn_start     <= w_data_lower[0];
                                                1: cnn_irq_en    <= w_data_lower[1];
                                                2: cnn_reset_acc <= w_data_lower[2];
                                            endcase
                                        end
                                    end
                                end
                                REG_IN_BASE:  begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_in_base[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_OUT_BASE: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_out_base[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_IMG_W: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_img_width[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_IMG_H: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_img_height[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K00: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k00[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K01: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k01[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K02: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k02[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K10: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k10[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K11: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k11[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K12: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k12[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K20: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k20[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K21: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k21[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K22: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k22[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_BIAS: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_bias[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX00: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px00[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX01: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px01[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX02: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px02[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX10: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px10[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX11: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px11[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX12: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px12[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX20: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px20[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX21: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px21[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX22: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px22[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                default: ; // RO registers: no-op
                            endcase
                        end
                        wr_state  <= WR_RESP;
                        b_valid_r <= 1'b1;
                    end
                    else if (s_axi_awvalid && s_axi_awready && aw_burst) begin
                        wr_state     <= WR_DRAIN;
                        wr_id_r      <= s_axi_awid;
                        wr_error_r   <= RESP_SLVERR;
                        wr_reg_sel_r <= aw_reg_sel;
                    end
                    else if (s_axi_awvalid && s_axi_awready) begin
                        wr_state     <= WR_WAIT_W;
                        wr_id_r      <= s_axi_awid;
                        wr_reg_sel_r <= aw_reg_sel;
                        if (!aw_in_aperture)
                            wr_error_r <= RESP_DECERR;
                        else if (!aw_aligned)
                            wr_error_r <= RESP_SLVERR;
                        else
                            wr_error_r <= RESP_OKAY;
                    end
                    else if (s_axi_wvalid && s_axi_wready) begin
                        wr_state  <= WR_WAIT_AW;
                        wr_data_r <= w_data_lower;
                        wr_strb_r <= w_strb_lower;
                    end
                end

                WR_WAIT_W: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                     compute_write_resp(wr_reg_sel_r) : wr_error_r;

                        if (wr_error_r == RESP_OKAY) begin
                            case (wr_reg_sel_r)
                                REG_CTRL: begin
                                    for (int i = 0; i < 4; i = i + 1) begin
                                        if (w_strb_lower[i]) begin
                                            case (i)
                                                0: cnn_start     <= w_data_lower[0];
                                                1: cnn_irq_en    <= w_data_lower[1];
                                                2: cnn_reset_acc <= w_data_lower[2];
                                            endcase
                                        end
                                    end
                                end
                                REG_IN_BASE:  begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_in_base[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_OUT_BASE: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_out_base[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_IMG_W: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_img_width[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_IMG_H: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_img_height[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K00: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k00[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K01: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k01[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K02: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k02[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K10: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k10[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K11: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k11[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K12: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k12[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K20: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k20[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K21: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k21[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_K22: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_k22[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_BIAS: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_bias[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX00: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px00[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX01: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px01[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX02: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px02[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX10: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px10[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX11: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px11[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX12: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px12[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX20: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px20[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX21: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px21[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                REG_PX22: begin
                                    for (int i = 0; i < 4; i = i + 1)
                                        if (w_strb_lower[i])
                                            cnn_px22[i*8 +: 8] <= w_data_lower[i*8 +: 8];
                                end
                                default: ;
                            endcase
                        end
                        wr_state  <= WR_RESP;
                        b_valid_r <= 1'b1;
                    end
                end

                WR_WAIT_AW: begin
                    if (s_axi_awvalid && s_axi_awready) begin
                        wr_id_r      <= s_axi_awid;
                        wr_reg_sel_r <= aw_reg_sel;
                        if (aw_burst) begin
                            wr_state   <= WR_DRAIN;
                            wr_error_r <= RESP_SLVERR;
                        end else begin
                            if (!aw_in_aperture)
                                wr_error_r <= RESP_DECERR;
                            else if (!aw_aligned)
                                wr_error_r <= RESP_SLVERR;
                            else
                                wr_error_r <= RESP_OKAY;

                            wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                        compute_write_resp(aw_reg_sel) : wr_error_r;

                            if (wr_error_r == RESP_OKAY) begin
                                case (aw_reg_sel)
                                    REG_CTRL: begin
                                        for (int i = 0; i < 4; i = i + 1) begin
                                            if (wr_strb_r[i]) begin
                                                case (i)
                                                    0: cnn_start     <= wr_data_r[0];
                                                    1: cnn_irq_en    <= wr_data_r[1];
                                                    2: cnn_reset_acc <= wr_data_r[2];
                                                endcase
                                            end
                                        end
                                    end
                                    REG_IN_BASE:  begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_in_base[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_OUT_BASE: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_out_base[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_IMG_W: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_img_width[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_IMG_H: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_img_height[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K00: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k00[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K01: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k01[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K02: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k02[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K10: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k10[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K11: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k11[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K12: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k12[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K20: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k20[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K21: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k21[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_K22: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_k22[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_BIAS: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_bias[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX00: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px00[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX01: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px01[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX02: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px02[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX10: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px10[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX11: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px11[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX12: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px12[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX20: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px20[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX21: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px21[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    REG_PX22: begin
                                        for (int i = 0; i < 4; i = i + 1)
                                            if (wr_strb_r[i])
                                                cnn_px22[i*8 +: 8] <= wr_data_r[i*8 +: 8];
                                    end
                                    default: ;
                                endcase
                            end
                            wr_state  <= WR_RESP;
                            b_valid_r <= 1'b1;
                        end
                    end
                end

                WR_DRAIN: begin
                    if (s_axi_wvalid && s_axi_wready && s_axi_wlast) begin
                        wr_state  <= WR_RESP;
                        wr_resp_r <= RESP_SLVERR;
                        b_valid_r <= 1'b1;
                    end
                end

                WR_RESP: begin
                    if (b_valid_r && s_axi_bready) begin
                        b_valid_r <= 1'b0;
                        wr_state  <= WR_IDLE;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    assign s_axi_bid    = wr_id_r;
    assign s_axi_bresp  = wr_resp_r;
    assign s_axi_bvalid = b_valid_r;

    // =========================================================================
    // Convolution engine control
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            conv_running   <= 1'b0;
            conv_done_pulse <= 1'b0;
            conv_x         <= '0;
            conv_y         <= '0;
            conv_result    <= '0;
            conv_acc       <= '0;

            // Config copy
            conv_in_base  <= '0;
            conv_out_base <= '0;
            conv_img_w    <= '0;
            conv_img_h    <= '0;
            conv_k00      <= '0; conv_k01 <= '0; conv_k02 <= '0;
            conv_k10      <= '0; conv_k11 <= '0; conv_k12 <= '0;
            conv_k20      <= '0; conv_k21 <= '0; conv_k22 <= '0;
            conv_bias     <= '0;
        end else begin
            conv_done_pulse <= 1'b0;  // default: deassert pulse

            // Latch config when start is asserted
            if (cnn_start && !conv_running) begin
                conv_in_base  <= cnn_in_base;
                conv_out_base <= cnn_out_base;
                conv_img_w    <= cnn_img_width;
                conv_img_h    <= cnn_img_height;
                conv_k00      <= cnn_k00; conv_k01 <= cnn_k01; conv_k02 <= cnn_k02;
                conv_k10      <= cnn_k10; conv_k11 <= cnn_k11; conv_k12 <= cnn_k12;
                conv_k20      <= cnn_k20; conv_k21 <= cnn_k21; conv_k22 <= cnn_k22;
                conv_bias     <= cnn_bias;

                conv_x        <= '0;
                conv_y        <= '0;
                conv_result   <= '0;
                conv_running  <= 1'b1;
            end

            // Convolution processing: one pixel per cycle
            if (conv_running) begin
                // Accumulate convolution sum for current pixel
                conv_acc <= conv_sum;

                // On next cycle, store result and advance
                conv_result <= conv_sum[31:0];  // truncate to 32 bits

                // Advance to next pixel
                if (conv_x == conv_img_w - 1) begin
                    conv_x <= '0;
                    if (conv_y == conv_img_h - 1) begin
                        conv_y <= '0;
                        conv_running <= 1'b0;  // done
                        conv_done_pulse <= 1'b1;
                        cnn_result_reg <= conv_result;
                        cnn_irq <= 1'b1;  // set interrupt flag
                    end else begin
                        conv_y <= conv_y + 1;
                    end
                end else begin
                    conv_x <= conv_x + 1;
                end
            end
        end
    end

    // =========================================================================
    // Status register update
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            cnn_done <= 1'b0;
            cnn_busy <= 1'b0;
            cnn_irq <= 1'b0;
        end else begin
            cnn_busy <= conv_running;
            if (conv_done_pulse)
                cnn_done <= 1'b1;
            else if (cnn_reset_acc) begin
                cnn_done <= 1'b0;
                cnn_irq <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Read channel FSM
    // =========================================================================
    reg [1:0]               rd_state;
    reg [AXI_ID_WIDTH-1:0]   rd_id_r;
    reg [AXI_DATA_WIDTH-1:0] rd_data_r;
    reg [1:0]               rd_resp_r;
    reg                     rd_last_r;
    reg                     r_valid_r;
    reg [7:0]               rd_len_r;

    assign s_axi_arready = s_axi_arvalid && (rd_state == RD_IDLE);

    // Read data mux (combinational from live registers)
    reg [31:0] reg_read_data;
    always @(*) begin
        case (ar_reg_sel)
            REG_CTRL:   reg_read_data = {29'b0, cnn_reset_acc, cnn_irq_en, cnn_start};
            REG_STATUS:   reg_read_data = {30'b0, cnn_busy, cnn_done};
            REG_IN_BASE:  reg_read_data = cnn_in_base;
            REG_OUT_BASE: reg_read_data = cnn_out_base;
            REG_IMG_W:    reg_read_data = cnn_img_width;
            REG_IMG_H:    reg_read_data = cnn_img_height;
            REG_K00:      reg_read_data = cnn_k00;
            REG_K01:      reg_read_data = cnn_k01;
            REG_K02:      reg_read_data = cnn_k02;
            REG_K10:      reg_read_data = cnn_k10;
            REG_K11:      reg_read_data = cnn_k11;
            REG_K12:      reg_read_data = cnn_k12;
            REG_K20:      reg_read_data = cnn_k20;
            REG_K21:      reg_read_data = cnn_k21;
            REG_K22:      reg_read_data = cnn_k22;
            REG_BIAS:     reg_read_data = cnn_bias;
            REG_PX00:     reg_read_data = cnn_px00;
            REG_PX01:     reg_read_data = cnn_px01;
            REG_PX02:     reg_read_data = cnn_px02;
            REG_PX10:     reg_read_data = cnn_px10;
            REG_PX11:     reg_read_data = cnn_px11;
            REG_PX12:     reg_read_data = cnn_px12;
            REG_PX20:     reg_read_data = cnn_px20;
            REG_PX21:     reg_read_data = cnn_px21;
            REG_PX22:     reg_read_data = cnn_px22;
            REG_RESULT:   reg_read_data = cnn_result_reg;
            REG_ID:       reg_read_data = CNN_ID_VALUE;
            REG_VERSION:  reg_read_data = CNN_VERSION_VALUE;
            default:      reg_read_data = 32'h0;
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            rd_state  <= RD_IDLE;
            rd_id_r   <= '0;
            rd_data_r <= '0;
            rd_resp_r <= RESP_OKAY;
            rd_last_r <= 1'b0;
            r_valid_r <= 1'b0;
            rd_len_r  <= 8'd0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (s_axi_arvalid && s_axi_arready) begin
                        rd_id_r  <= s_axi_arid;
                        rd_len_r <= s_axi_arlen;

                        if (ar_burst) begin
                            rd_data_r <= {AXI_DATA_WIDTH{1'b0}};
                            rd_resp_r <= RESP_SLVERR;
                            r_valid_r <= 1'b1;
                            rd_last_r <= (s_axi_arlen == 8'd0);
                        end else begin
                            if (ar_addr_error) begin
                                rd_data_r <= {AXI_DATA_WIDTH{1'b0}};
                                rd_resp_r <= ar_in_aperture ? RESP_SLVERR : RESP_DECERR;
                            end else begin
                                rd_data_r <= {AXI_DATA_WIDTH{1'b0}} | reg_read_data;
                                rd_resp_r <= compute_read_resp(ar_reg_sel);
                            end
                            r_valid_r <= 1'b1;
                            rd_last_r <= 1'b1;
                        end
                        rd_state <= RD_RESP;
                    end
                end

                RD_RESP: begin
                    if (r_valid_r && s_axi_rready) begin
                        if (rd_last_r) begin
                            r_valid_r <= 1'b0;
                            rd_last_r <= 1'b0;
                            rd_state  <= RD_IDLE;
                        end else begin
                            rd_len_r  <= rd_len_r - 8'd1;
                            rd_last_r <= (rd_len_r == 8'd1);
                        end
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

    assign s_axi_rid    = rd_id_r;
    assign s_axi_rdata  = rd_data_r;
    assign s_axi_rresp  = rd_resp_r;
    assign s_axi_rlast  = rd_last_r;
    assign s_axi_rvalid = r_valid_r;

endmodule

`default_nettype wire
