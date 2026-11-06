// =============================================================================
// File    : axi_relu.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Parameterized AXI4 Memory-Mapped ReLU Activation Unit IP block.
//
//           Wraps the streaming relu_unit core with an AXI4 slave interface,
//           allowing the RISC-V processor or DMA controller to configure,
//           trigger, and inspect ReLU activations and sparsity metrics.
//
//           The AXI4 slave interface matches the existing SoC interconnect
//           (64-bit data, 32-bit address, 8-bit ID, 8-bit strobe).
//           ReLU register data occupies the lower 32 bits of the 64-bit data
//           bus; the upper 32 bits read as zero and are ignored on writes.
//
// Register map (32-bit registers, word-aligned, 4 KiB aperture):
//   Offset  Register          Access  Description
//   ------  ----------------  ------  ----------------------------------------
//   0x00    RELU_CTRL         RW      bit0: enable (default 1)
//                                     bit1: bypass (default 0)
//                                     bit2: clip_enable (default 0)
//                                     bit3: reset_stats (W1C pulse)
//   0x04    RELU_STATUS       RO      bit0: busy
//                                     bit1: last_zero
//                                     bit2: last_active
//                                     bit3: last_clipped
//   0x08    RELU_DIN          RW      Input signed data to be activated
//   0x0C    RELU_DOUT         RO      Activated output data = max(0, DIN)
//   0x10    RELU_CLIP_MAX     RW      Clipping threshold (default: 0x7FFF_FFFF)
//   0x14    RELU_ZERO_COUNT   RO      Count of inputs clamped to 0 or zero
//   0x18    RELU_TOTAL_COUNT  RO      Total count of inputs processed
//   0x1C    RELU_ID           RO      Fixed peripheral ID ("RELU")
//   0x20    RELU_VERSION      RO      Fixed version (v1.0)
//
// Reset: active-high (rst).
//   On reset: CTRL=0x01 (enabled), DIN=0, DOUT=0, stats=0.
//
// AXI response codes:
//   OKAY   = 2'b00  -- valid register access
//   SLVERR = 2'b10  -- reserved offset, write to RO register, or misaligned
//   DECERR = 2'b11  -- address outside the valid 4 KiB register aperture
//
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_relu #(
    parameter integer DATA_WIDTH       = 32,

    // AXI bus widths -- match the interconnect defaults
    parameter integer AXI_DATA_WIDTH  = 64,
    parameter integer AXI_ADDR_WIDTH  = 32,
    parameter integer AXI_ID_WIDTH    = 8,
    parameter integer AXI_STRB_WIDTH  = AXI_DATA_WIDTH / 8,

    // Register aperture: 4 KiB (12 address bits)
    parameter integer REG_ADDR_WIDTH  = 12,

    // Base address in system memory map
    parameter [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h0,

    // Peripheral identification constants
    parameter [31:0] RELU_ID_VALUE      = 32'h52454C55,  // "RELU" in ASCII
    parameter [31:0] RELU_VERSION_VALUE = 32'h00010000   // v1.0
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
    input  wire                            s_axi_rready
);

    // =========================================================================
    // Local constants and register offsets
    // =========================================================================
    localparam [4:0] REG_CTRL        = 5'h00;  // 0x00
    localparam [4:0] REG_STATUS      = 5'h01;  // 0x04
    localparam [4:0] REG_DIN         = 5'h02;  // 0x08
    localparam [4:0] REG_DOUT        = 5'h03;  // 0x0C
    localparam [4:0] REG_CLIP_MAX    = 5'h04;  // 0x10
    localparam [4:0] REG_ZERO_COUNT  = 5'h05;  // 0x14
    localparam [4:0] REG_TOTAL_COUNT = 5'h06;  // 0x18
    localparam [4:0] REG_ID          = 5'h07;  // 0x1C
    localparam [4:0] REG_VERSION     = 5'h08;  // 0x20

    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // =========================================================================
    // Register storage
    // =========================================================================
    reg [31:0] relu_ctrl_reg;
    reg [31:0] relu_din_reg;
    reg [31:0] relu_clip_max_reg;
    reg [31:0] relu_zero_count_reg;
    reg [31:0] relu_total_count_reg;

    wire ctrl_enable      = relu_ctrl_reg[0];
    wire ctrl_bypass      = relu_ctrl_reg[1];
    wire ctrl_clip_en     = relu_ctrl_reg[2];

    // =========================================================================
    // ReLU Core Instantiation
    // =========================================================================
    reg                  core_valid_in;
    wire                 core_ready_out;
    wire [31:0]          core_data_out;
    wire                 core_valid_out;
    wire                 core_zero_flag;
    wire                 core_active_flag;
    wire                 core_clipped_flag;

    reg [31:0] relu_dout_reg;
    reg        status_last_zero;
    reg        status_last_active;
    reg        status_last_clipped;

    relu_unit #(
        .DATA_WIDTH     (32),
        .PIPELINE_STAGES(0),   // Combinational for immediate single-cycle register write-read
        .CLIPPING_ENABLE(1)
    ) u_relu_core (
        .clk            (clk),
        .rst            (rst),
        .enable_i       (ctrl_enable),
        .bypass_i       (ctrl_bypass),
        .clip_en_i      (ctrl_clip_en),
        .clip_max_i     (relu_clip_max_reg),
        .data_i         (relu_din_reg),
        .valid_i        (core_valid_in),
        .ready_o        (core_ready_out),
        .data_o         (core_data_out),
        .valid_o        (core_valid_out),
        .ready_i        (1'b1),
        .zero_flag_o    (core_zero_flag),
        .active_flag_o  (core_active_flag),
        .clipped_flag_o (core_clipped_flag)
    );

    // =========================================================================
    // Address decoding and aperture check
    // =========================================================================
    wire [AXI_ADDR_WIDTH-1:0] aw_offset = s_axi_awaddr - BASE_ADDR;
    wire [AXI_ADDR_WIDTH-1:0] ar_offset = s_axi_araddr - BASE_ADDR;

    wire aw_in_aperture = (s_axi_awaddr >= BASE_ADDR) &&
                          (aw_offset < (32'(1) << REG_ADDR_WIDTH));
    wire ar_in_aperture = (s_axi_araddr >= BASE_ADDR) &&
                          (ar_offset < (32'(1) << REG_ADDR_WIDTH));

    wire aw_aligned = (s_axi_awaddr[1:0] == 2'b00);
    wire ar_aligned = (s_axi_araddr[1:0] == 2'b00);

    wire [4:0] aw_reg_sel = aw_offset[6:2];
    wire [4:0] ar_reg_sel = ar_offset[6:2];

    function automatic [1:0] check_aw_error(
        input in_aperture,
        input aligned,
        input [4:0] reg_sel,
        input [7:0] awlen
    );
        if (!in_aperture) return RESP_DECERR;
        if (!aligned || (awlen != 8'h00)) return RESP_SLVERR;
        case (reg_sel)
            REG_CTRL,
            REG_DIN,
            REG_CLIP_MAX: return RESP_OKAY;
            // Write to Read-Only registers or undefined offsets gives SLVERR
            REG_STATUS,
            REG_DOUT,
            REG_ZERO_COUNT,
            REG_TOTAL_COUNT,
            REG_ID,
            REG_VERSION: return RESP_SLVERR;
            default:     return RESP_SLVERR;
        endcase
    endfunction

    function automatic [1:0] check_ar_error(
        input in_aperture,
        input aligned,
        input [4:0] reg_sel,
        input [7:0] arlen
    );
        if (!in_aperture) return RESP_DECERR;
        if (!aligned || (arlen != 8'h00)) return RESP_SLVERR;
        case (reg_sel)
            REG_CTRL,
            REG_STATUS,
            REG_DIN,
            REG_DOUT,
            REG_CLIP_MAX,
            REG_ZERO_COUNT,
            REG_TOTAL_COUNT,
            REG_ID,
            REG_VERSION: return RESP_OKAY;
            default:     return RESP_SLVERR;
        endcase
    endfunction

    // =========================================================================
    // Write FSM
    // =========================================================================
    localparam [2:0] WR_IDLE    = 3'd0;
    localparam [2:0] WR_WAIT_AW = 3'd1;
    localparam [2:0] WR_WAIT_W  = 3'd2;
    localparam [2:0] WR_RESP    = 3'd3;
    localparam [2:0] WR_DRAIN   = 3'd4;

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
    assign s_axi_wready  = s_axi_wvalid &&
                          (wr_state == WR_IDLE || wr_state == WR_WAIT_W || wr_state == WR_DRAIN);

    function automatic [31:0] apply_strb(
        input [31:0] old_val,
        input [31:0] new_val,
        input [3:0]  strb
    );
        apply_strb[7:0]   = strb[0] ? new_val[7:0]   : old_val[7:0];
        apply_strb[15:8]  = strb[1] ? new_val[15:8]  : old_val[15:8];
        apply_strb[23:16] = strb[2] ? new_val[23:16] : old_val[23:16];
        apply_strb[31:24] = strb[3] ? new_val[31:24] : old_val[31:24];
    endfunction

    task automatic do_reg_write(
        input [4:0]  sel,
        input [31:0] data,
        input [3:0]  strb
    );
        logic [31:0] new_din;
        logic signed [DATA_WIDTH-1:0] new_din_s;
        logic is_zero, is_active, is_clipped;
        logic signed [DATA_WIDTH-1:0] new_dout;
        begin
            case (sel)
                REG_CTRL: begin
                    relu_ctrl_reg <= apply_strb(relu_ctrl_reg, data, strb);
                    if (strb[0] && data[3]) begin
                        // Reset stats
                        relu_zero_count_reg  <= 32'd0;
                        relu_total_count_reg <= 32'd0;
                    end
                end
                REG_DIN: begin
                    new_din   = apply_strb(relu_din_reg, data, strb);
                    new_din_s = $signed(new_din);

                    if (ctrl_bypass) begin
                        new_dout   = new_din_s;
                        is_zero    = 1'b0;
                        is_active  = (new_din_s > 0);
                        is_clipped = 1'b0;
                    end else if (new_din_s <= 0) begin
                        new_dout   = '0;
                        is_zero    = 1'b1;
                        is_active  = 1'b0;
                        is_clipped = 1'b0;
                    end else if (ctrl_clip_en && (new_din_s > $signed(relu_clip_max_reg))) begin
                        new_dout   = relu_clip_max_reg;
                        is_zero    = 1'b0;
                        is_active  = 1'b1;
                        is_clipped = 1'b1;
                    end else begin
                        new_dout   = new_din_s;
                        is_zero    = 1'b0;
                        is_active  = 1'b1;
                        is_clipped = 1'b0;
                    end

                    relu_din_reg        <= new_din;
                    core_valid_in       <= 1'b1;
                    relu_dout_reg       <= new_dout;
                    status_last_zero    <= is_zero;
                    status_last_active  <= is_active;
                    status_last_clipped <= is_clipped;

                    // Update metrics
                    relu_total_count_reg <= relu_total_count_reg + 1'b1;
                    if (is_zero) begin
                        relu_zero_count_reg <= relu_zero_count_reg + 1'b1;
                    end
                end
                REG_CLIP_MAX: begin
                    relu_clip_max_reg <= apply_strb(relu_clip_max_reg, data, strb);
                end
                default: ;
            endcase
        end
    endtask

    always_ff @(posedge clk) begin
        if (rst) begin
            wr_state             <= WR_IDLE;
            wr_id_r              <= '0;
            wr_reg_sel_r         <= '0;
            wr_error_r           <= RESP_OKAY;
            wr_data_r            <= '0;
            wr_strb_r            <= '0;
            wr_resp_r            <= RESP_OKAY;
            b_valid_r            <= 1'b0;

            relu_ctrl_reg        <= 32'h0000_0001; // Enabled by default
            relu_din_reg         <= 32'd0;
            relu_dout_reg        <= 32'd0;
            relu_clip_max_reg    <= 32'h7FFF_FFFF;
            relu_zero_count_reg  <= 32'd0;
            relu_total_count_reg <= 32'd0;

            core_valid_in        <= 1'b0;
            status_last_zero     <= 1'b0;
            status_last_active   <= 1'b0;
            status_last_clipped  <= 1'b0;
        end else begin
            core_valid_in <= 1'b0;

            case (wr_state)
                WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        wr_id_r   <= s_axi_awid;
                        wr_resp_r <= check_aw_error(aw_in_aperture, aw_aligned, aw_reg_sel, s_axi_awlen);
                        if (check_aw_error(aw_in_aperture, aw_aligned, aw_reg_sel, s_axi_awlen) == RESP_OKAY) begin
                            do_reg_write(aw_reg_sel, w_data_lower, w_strb_lower);
                        end
                        if (s_axi_awlen > 8'h00 && !s_axi_wlast) begin
                            wr_state <= WR_DRAIN;
                        end else begin
                            b_valid_r <= 1'b1;
                            wr_state  <= WR_RESP;
                        end
                    end else if (s_axi_awvalid) begin
                        wr_id_r      <= s_axi_awid;
                        wr_reg_sel_r <= aw_reg_sel;
                        wr_error_r   <= check_aw_error(aw_in_aperture, aw_aligned, aw_reg_sel, s_axi_awlen);
                        wr_state     <= WR_WAIT_W;
                    end else if (s_axi_wvalid) begin
                        wr_data_r    <= w_data_lower;
                        wr_strb_r    <= w_strb_lower;
                        wr_state     <= WR_WAIT_AW;
                    end
                end

                WR_WAIT_W: begin
                    if (s_axi_wvalid) begin
                        wr_resp_r <= wr_error_r;
                        if (wr_error_r == RESP_OKAY) begin
                            do_reg_write(wr_reg_sel_r, w_data_lower, w_strb_lower);
                        end
                        if (!s_axi_wlast) begin
                            wr_state <= WR_DRAIN;
                        end else begin
                            b_valid_r <= 1'b1;
                            wr_state  <= WR_RESP;
                        end
                    end
                end

                WR_WAIT_AW: begin
                    if (s_axi_awvalid) begin
                        wr_id_r   <= s_axi_awid;
                        wr_resp_r <= check_aw_error(aw_in_aperture, aw_aligned, aw_reg_sel, s_axi_awlen);
                        if (check_aw_error(aw_in_aperture, aw_aligned, aw_reg_sel, s_axi_awlen) == RESP_OKAY) begin
                            do_reg_write(aw_reg_sel, wr_data_r, wr_strb_r);
                        end
                        b_valid_r <= 1'b1;
                        wr_state  <= WR_RESP;
                    end
                end

                WR_DRAIN: begin
                    if (s_axi_wvalid && s_axi_wlast) begin
                        b_valid_r <= 1'b1;
                        wr_state  <= WR_RESP;
                    end
                end

                WR_RESP: begin
                    if (s_axi_bready && b_valid_r) begin
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
    // Read FSM
    // =========================================================================
    localparam [1:0] RD_IDLE = 2'd0;
    localparam [1:0] RD_RESP = 2'd1;

    reg [1:0]               rd_state;
    reg [AXI_ID_WIDTH-1:0]   rd_id_r;
    reg [AXI_DATA_WIDTH-1:0] rd_data_r;
    reg [1:0]               rd_resp_r;
    reg                     rd_valid_r;
    reg                     rd_last_r;
    reg [7:0]               rd_len_r;
    reg [7:0]               rd_cnt_r;

    assign s_axi_arready = s_axi_arvalid && (rd_state == RD_IDLE);

    reg [31:0] reg_read_data;
    wire [31:0] relu_status_reg = {28'd0, status_last_clipped, status_last_active, status_last_zero, 1'b0};

    always_comb begin
        case (ar_reg_sel)
            REG_CTRL:        reg_read_data = relu_ctrl_reg;
            REG_STATUS:      reg_read_data = relu_status_reg;
            REG_DIN:         reg_read_data = relu_din_reg;
            REG_DOUT:        reg_read_data = relu_dout_reg;
            REG_CLIP_MAX:    reg_read_data = relu_clip_max_reg;
            REG_ZERO_COUNT:  reg_read_data = relu_zero_count_reg;
            REG_TOTAL_COUNT: reg_read_data = relu_total_count_reg;
            REG_ID:          reg_read_data = RELU_ID_VALUE;
            REG_VERSION:     reg_read_data = RELU_VERSION_VALUE;
            default:         reg_read_data = 32'd0;
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            rd_state   <= RD_IDLE;
            rd_id_r    <= '0;
            rd_data_r  <= '0;
            rd_resp_r  <= RESP_OKAY;
            rd_valid_r <= 1'b0;
            rd_last_r  <= 1'b0;
            rd_len_r   <= '0;
            rd_cnt_r   <= '0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    if (s_axi_arvalid && s_axi_arready) begin
                        rd_id_r    <= s_axi_arid;
                        rd_len_r   <= s_axi_arlen;
                        rd_cnt_r   <= 8'd0;
                        rd_resp_r  <= check_ar_error(ar_in_aperture, ar_aligned, ar_reg_sel, s_axi_arlen);
                        if (check_ar_error(ar_in_aperture, ar_aligned, ar_reg_sel, s_axi_arlen) == RESP_OKAY) begin
                            rd_data_r  <= {{ (AXI_DATA_WIDTH-32){1'b0} }, reg_read_data};
                        end else begin
                            rd_data_r  <= '0;
                        end
                        rd_valid_r <= 1'b1;
                        rd_last_r  <= (s_axi_arlen == 8'd0);
                        rd_state   <= RD_RESP;
                    end
                end

                RD_RESP: begin
                    if (rd_valid_r && s_axi_rready) begin
                        if (rd_last_r) begin
                            rd_valid_r <= 1'b0;
                            rd_last_r  <= 1'b0;
                            rd_state   <= RD_IDLE;
                        end else begin
                            rd_cnt_r   <= rd_cnt_r + 1'b1;
                            rd_last_r  <= ((rd_cnt_r + 1'b1) == rd_len_r);
                            rd_resp_r  <= RESP_SLVERR;
                            rd_data_r  <= '0;
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
    assign s_axi_rvalid = rd_valid_r;
    assign s_axi_rvalid = rd_valid_r;

endmodule

`default_nettype wire
