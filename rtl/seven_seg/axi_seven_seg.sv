// =============================================================================
// File    : axi_seven_seg.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Memory-Mapped AXI4 Seven-Segment Display Controller
//           Designed for the Basys3 FPGA 4-digit multiplexed display.
//
// Features:
//   - AXI4 64-bit/32-bit memory-mapped slave interface (BASE_ADDR = 0x7000_0000)
//   - 64-bit bus lane steering and duplicate read data for CPU master compatibility
//   - Hardware Hex/BCD to 7-segment active-low cathode decoder (0..F)
//   - Time-multiplexed 4-digit anode scanner with programmable refresh rate
//   - Individual digit blanking / enable mask (REG_DIGIT_EN)
//   - Per-digit decimal point control (REG_DECIMAL_PT)
//   - Raw 8-bit segment override register for arbitrary custom symbols (REG_RAW_SEG)
//   - Master display enable, polarity selection, and fast simulation refresh mode
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_seven_seg #(
    parameter integer AXI_DATA_WIDTH       = 64,
    parameter integer AXI_ADDR_WIDTH       = 32,
    parameter integer AXI_ID_WIDTH         = 8,
    parameter integer AXI_STRB_WIDTH       = AXI_DATA_WIDTH / 8,
    parameter integer REG_ADDR_WIDTH       = 12,           // 4 KiB aperture
    parameter [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h7000_0000,
    parameter integer DEFAULT_REFRESH_DIV  = 100_000,     // 100 MHz / 100,000 = 1 kHz refresh
    parameter [31:0] SEVEN_SEG_ID_VALUE    = 32'h37534547, // "7SEG" in ASCII
    parameter [31:0] SEVEN_SEG_VER_VALUE   = 32'h00010000  // v1.0
)(
    input  wire                            clk,
    input  wire                            rst,          // active-high system reset

    // AXI4 Slave Write Address Channel
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

    // AXI4 Slave Write Data Channel
    input  wire [AXI_DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [AXI_STRB_WIDTH-1:0]       s_axi_wstrb,
    input  wire                            s_axi_wlast,
    input  wire                            s_axi_wvalid,
    output wire                            s_axi_wready,

    // AXI4 Slave Write Response Channel
    output wire [AXI_ID_WIDTH-1:0]         s_axi_bid,
    output wire [1:0]                      s_axi_bresp,
    output wire                            s_axi_bvalid,
    input  wire                            s_axi_bready,

    // AXI4 Slave Read Address Channel
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

    // AXI4 Slave Read Data Channel
    output wire [AXI_ID_WIDTH-1:0]         s_axi_rid,
    output wire [AXI_DATA_WIDTH-1:0]       s_axi_rdata,
    output wire [1:0]                      s_axi_rresp,
    output wire                            s_axi_rlast,
    output wire                            s_axi_rvalid,
    input  wire                            s_axi_rready,

    // Basys3 Physical Seven-Segment Display Interface
    output wire [6:0]                      seg_o,        // Cathodes [6:0] = {g, f, e, d, c, b, a}
    output wire                            dp_o,         // Decimal point cathode
    output wire [3:0]                      an_o          // Anodes [3:0] for digits 3..0
);

    // =========================================================================
    // Local Constants
    // =========================================================================
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // Write FSM States
    localparam [2:0] WR_IDLE    = 3'd0;
    localparam [2:0] WR_WAIT_W  = 3'd1;
    localparam [2:0] WR_WAIT_AW = 3'd2;
    localparam [2:0] WR_DRAIN   = 3'd3;
    localparam [2:0] WR_RESP    = 3'd4;

    // Read FSM States
    localparam [1:0] RD_IDLE    = 2'd0;
    localparam [1:0] RD_RESP    = 2'd1;

    // =========================================================================
    // Address Decoding
    // =========================================================================
    wire [AXI_ADDR_WIDTH-1:0] aw_offset = s_axi_awaddr - BASE_ADDR;
    wire [AXI_ADDR_WIDTH-1:0] ar_offset = s_axi_araddr - BASE_ADDR;

    wire aw_in_aperture = (aw_offset[AXI_ADDR_WIDTH-1:REG_ADDR_WIDTH] == 0);
    wire ar_in_aperture = (ar_offset[AXI_ADDR_WIDTH-1:REG_ADDR_WIDTH] == 0);

    wire aw_aligned = (s_axi_awaddr[1:0] == 2'b00);
    wire ar_aligned = (s_axi_araddr[1:0] == 2'b00);

    wire aw_burst = (s_axi_awlen != 8'd0);
    wire ar_burst = (s_axi_arlen != 8'd0);

    wire [3:0] aw_reg_sel = aw_offset[5:2];
    wire [3:0] ar_reg_sel = ar_offset[5:2];

    wire aw_addr_error = !aw_in_aperture || !aw_aligned;
    wire ar_addr_error = !ar_in_aperture || !ar_aligned;

    // 64-bit bus lane steering
    wire w_hi_lane = (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ||
                     (aw_offset[2] && s_axi_wstrb[7:4] != 4'h0);
    wire [31:0] w_data_eff = w_hi_lane ? s_axi_wdata[63:32] : s_axi_wdata[31:0];
    wire [3:0]  w_strb_eff = w_hi_lane ? s_axi_wstrb[7:4]    : s_axi_wstrb[3:0];

    wire [3:0] aw_reg_sel_eff = (aw_offset[2:0] == 3'b000 && (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0)) ?
                                (aw_reg_sel | 4'h1) : aw_reg_sel;

    // =========================================================================
    // Control / Configuration Registers
    // =========================================================================
    reg [15:0] reg_digits;       // [15:12] D3, [11:8] D2, [7:4] D1, [3:0] D0
    reg [3:0]  reg_digit_en;     // Digits 3..0 enable mask (1 = enabled, 0 = blank)
    reg [3:0]  reg_decimal_pt;   // Digits 3..0 decimal point mask (1 = DP on)
    reg [31:0] reg_refresh_div;  // Refresh rate clock divisor
    reg [31:0] reg_raw_seg;      // Custom raw segments (8 bits per digit)
    reg [3:0]  reg_ctrl;         // bit 0: disp_en, bit 1: raw_mode, bit 2: invert, bit 3: fast_sim

    // =========================================================================
    // Register Read Logic
    // =========================================================================
    reg [31:0] reg_read_data;

    always @* begin
        case (ar_reg_sel)
            4'h0: reg_read_data = {16'h0, reg_digits};
            4'h1: reg_read_data = {28'h0, reg_digit_en};
            4'h2: reg_read_data = {28'h0, reg_decimal_pt};
            4'h3: reg_read_data = reg_refresh_div;
            4'h4: reg_read_data = reg_raw_seg;
            4'h5: reg_read_data = {28'h0, reg_ctrl};
            4'h6: reg_read_data = SEVEN_SEG_ID_VALUE;
            4'h7: reg_read_data = SEVEN_SEG_VER_VALUE;
            default: reg_read_data = 32'h0;
        endcase
    end

    function [1:0] compute_write_resp;
        input [3:0] sel;
        begin
            case (sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5:
                    compute_write_resp = RESP_OKAY;
                4'h6, 4'h7:
                    compute_write_resp = RESP_SLVERR; // RO registers
                default:
                    compute_write_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    function [1:0] compute_read_resp;
        input [3:0] sel;
        begin
            case (sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6, 4'h7:
                    compute_read_resp = RESP_OKAY;
                default:
                    compute_read_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    // =========================================================================
    // Write FSM
    // =========================================================================
    reg [2:0]              wr_state;
    reg [AXI_ID_WIDTH-1:0] wr_id_r;
    reg [3:0]              wr_reg_sel_r;
    reg [1:0]              wr_error_r;
    reg [31:0]             wr_data_r;
    reg [3:0]              wr_strb_r;
    reg [1:0]              wr_resp_r;
    reg                    b_valid_r;
    reg                    wr_aw_offset2_r;
    reg                    wr_hi_lane_r;

    assign s_axi_awready = s_axi_awvalid && (wr_state == WR_IDLE || wr_state == WR_WAIT_AW);
    assign s_axi_wready  = s_axi_wvalid  && (wr_state == WR_IDLE || wr_state == WR_WAIT_W || wr_state == WR_DRAIN);

    wire w_hi_wait_w = (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ||
                       (wr_aw_offset2_r && s_axi_wstrb[7:4] != 4'h0);
    wire [31:0] w_data_wait_w = w_hi_wait_w ? s_axi_wdata[63:32] : s_axi_wdata[31:0];
    wire [3:0]  w_strb_wait_w = w_hi_wait_w ? s_axi_wstrb[7:4]    : s_axi_wstrb[3:0];
    wire [3:0]  eff_reg_wait_w = (!wr_aw_offset2_r && (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0)) ?
                                 (wr_reg_sel_r | 4'h1) : wr_reg_sel_r;

    wire [3:0] eff_reg_wait_aw = (aw_offset[2:0] == 3'b000 && wr_hi_lane_r) ?
                                 (aw_reg_sel | 4'h1) : aw_reg_sel;

    task apply_reg_write(input [3:0] sel, input [31:0] data, input [3:0] strb);
        integer i;
        begin
            case (sel)
                4'h0: begin // REG_DIGITS
                    if (strb[0]) reg_digits[7:0]   <= data[7:0];
                    if (strb[1]) reg_digits[15:8]  <= data[15:8];
                end
                4'h1: begin // REG_DIGIT_EN
                    if (strb[0]) reg_digit_en <= data[3:0];
                end
                4'h2: begin // REG_DECIMAL_PT
                    if (strb[0]) reg_decimal_pt <= data[3:0];
                end
                4'h3: begin // REG_REFRESH_DIV
                    for (i = 0; i < 4; i = i + 1) begin
                        if (strb[i]) reg_refresh_div[i*8 +: 8] <= data[i*8 +: 8];
                    end
                end
                4'h4: begin // REG_RAW_SEG
                    for (i = 0; i < 4; i = i + 1) begin
                        if (strb[i]) reg_raw_seg[i*8 +: 8] <= data[i*8 +: 8];
                    end
                end
                4'h5: begin // REG_CTRL
                    if (strb[0]) reg_ctrl <= data[3:0];
                end
                default: ; // RO or reserved
            endcase
        end
    endtask

    always @(posedge clk) begin
        if (rst) begin
            wr_state        <= WR_IDLE;
            wr_id_r         <= '0;
            wr_reg_sel_r    <= '0;
            wr_error_r      <= RESP_OKAY;
            wr_data_r       <= '0;
            wr_strb_r       <= '0;
            wr_resp_r       <= RESP_OKAY;
            b_valid_r       <= 1'b0;
            wr_aw_offset2_r <= 1'b0;
            wr_hi_lane_r    <= 1'b0;

            // Default register resets
            reg_digits      <= 16'h0000;
            reg_digit_en    <= 4'hF;               // all 4 digits enabled
            reg_decimal_pt  <= 4'h0;               // all DPs off
            reg_refresh_div <= DEFAULT_REFRESH_DIV;
            reg_raw_seg     <= 32'h0000_0000;
            reg_ctrl        <= 4'h1;               // display enable = 1
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready &&
                        s_axi_wvalid && s_axi_wready && !aw_burst) begin
                        wr_id_r      <= s_axi_awid;
                        wr_reg_sel_r <= aw_reg_sel_eff;
                        if (!aw_in_aperture) begin
                            wr_error_r <= RESP_DECERR;
                            wr_resp_r  <= RESP_DECERR;
                        end else if (!aw_aligned) begin
                            wr_error_r <= RESP_SLVERR;
                            wr_resp_r  <= RESP_SLVERR;
                        end else begin
                            wr_error_r <= RESP_OKAY;
                            wr_resp_r  <= compute_write_resp(aw_reg_sel_eff);
                            apply_reg_write(aw_reg_sel_eff, w_data_eff, w_strb_eff);
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
                        wr_state        <= WR_WAIT_W;
                        wr_id_r         <= s_axi_awid;
                        wr_reg_sel_r    <= aw_reg_sel;
                        wr_aw_offset2_r <= aw_offset[2];
                        wr_error_r      <= !aw_in_aperture ? RESP_DECERR :
                                           !aw_aligned     ? RESP_SLVERR : RESP_OKAY;
                    end
                    else if (s_axi_wvalid && s_axi_wready) begin
                        wr_state     <= WR_WAIT_AW;
                        wr_hi_lane_r <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0);
                        wr_data_r    <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ?
                                        s_axi_wdata[63:32] : s_axi_wdata[31:0];
                        wr_strb_r    <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ?
                                        s_axi_wstrb[7:4]    : s_axi_wstrb[3:0];
                    end
                end

                WR_WAIT_W: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                     compute_write_resp(eff_reg_wait_w) : wr_error_r;
                        if (wr_error_r == RESP_OKAY) begin
                            apply_reg_write(eff_reg_wait_w, w_data_wait_w, w_strb_wait_w);
                        end
                        wr_state  <= WR_RESP;
                        b_valid_r <= 1'b1;
                    end
                end

                WR_WAIT_AW: begin
                    if (s_axi_awvalid && s_axi_awready) begin
                        wr_id_r      <= s_axi_awid;
                        wr_reg_sel_r <= eff_reg_wait_aw;
                        if (aw_burst) begin
                            wr_state   <= WR_DRAIN;
                            wr_error_r <= RESP_SLVERR;
                        end else begin
                            if (!aw_in_aperture) begin
                                wr_error_r <= RESP_DECERR;
                                wr_resp_r  <= RESP_DECERR;
                            end else if (!aw_aligned) begin
                                wr_error_r <= RESP_SLVERR;
                                wr_resp_r  <= RESP_SLVERR;
                            end else begin
                                wr_error_r <= RESP_OKAY;
                                wr_resp_r  <= compute_write_resp(eff_reg_wait_aw);
                                apply_reg_write(eff_reg_wait_aw, wr_data_r, wr_strb_r);
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
    // Read FSM
    // =========================================================================
    reg [1:0]              rd_state;
    reg [AXI_ID_WIDTH-1:0] rd_id_r;
    reg [AXI_DATA_WIDTH-1:0] rd_data_r;
    reg [1:0]              rd_resp_r;
    reg                    rd_last_r;
    reg                    r_valid_r;
    reg [7:0]              rd_len_r;

    assign s_axi_arready = s_axi_arvalid && (rd_state == RD_IDLE);

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
                                rd_data_r <= {reg_read_data, reg_read_data};
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

    // =========================================================================
    // 7-Segment Multiplexing & Display Engine
    // =========================================================================
    reg [31:0] refresh_counter;
    reg [1:0]  active_digit_idx;

    wire [31:0] refresh_limit = reg_ctrl[3] ? 32'd4 : (reg_refresh_div > 32'd1 ? reg_refresh_div : 32'd1);

    always @(posedge clk) begin
        if (rst) begin
            refresh_counter  <= 32'd0;
            active_digit_idx <= 2'b00;
        end else begin
            if (refresh_counter >= (refresh_limit - 1)) begin
                refresh_counter  <= 32'd0;
                active_digit_idx <= active_digit_idx + 2'b01;
            end else begin
                refresh_counter <= refresh_counter + 32'd1;
            end
        end
    end

    // Current digit nibble selection
    reg [3:0] current_nibble;
    reg       current_dp_en;
    reg       current_digit_en;
    reg [7:0] current_raw_pattern;

    always @* begin
        case (active_digit_idx)
            2'b00: begin
                current_nibble      = reg_digits[3:0];
                current_dp_en       = reg_decimal_pt[0];
                current_digit_en    = reg_digit_en[0];
                current_raw_pattern = reg_raw_seg[7:0];
            end
            2'b01: begin
                current_nibble      = reg_digits[7:4];
                current_dp_en       = reg_decimal_pt[1];
                current_digit_en    = reg_digit_en[1];
                current_raw_pattern = reg_raw_seg[15:8];
            end
            2'b10: begin
                current_nibble      = reg_digits[11:8];
                current_dp_en       = reg_decimal_pt[2];
                current_digit_en    = reg_digit_en[2];
                current_raw_pattern = reg_raw_seg[23:16];
            end
            2'b11: begin
                current_nibble      = reg_digits[15:12];
                current_dp_en       = reg_decimal_pt[3];
                current_digit_en    = reg_digit_en[3];
                current_raw_pattern = reg_raw_seg[31:24];
            end
        endcase
    end

    // Hex to 7-Segment Active-Low Cathode Decoder: {g, f, e, d, c, b, a}
    function [6:0] hex_to_7seg(input [3:0] hex);
        begin
            case (hex)
                4'h0: hex_to_7seg = 7'b100_0000; // 0
                4'h1: hex_to_7seg = 7'b111_1001; // 1
                4'h2: hex_to_7seg = 7'b010_0100; // 2
                4'h3: hex_to_7seg = 7'b011_0000; // 3
                4'h4: hex_to_7seg = 7'b001_1001; // 4
                4'h5: hex_to_7seg = 7'b001_0010; // 5
                4'h6: hex_to_7seg = 7'b000_0010; // 6
                4'h7: hex_to_7seg = 7'b111_1000; // 7
                4'h8: hex_to_7seg = 7'b000_0000; // 8
                4'h9: hex_to_7seg = 7'b001_0000; // 9
                4'hA: hex_to_7seg = 7'b000_1000; // A
                4'hB: hex_to_7seg = 7'b000_0011; // b
                4'hC: hex_to_7seg = 7'b100_0110; // C
                4'hD: hex_to_7seg = 7'b010_0001; // d
                4'hE: hex_to_7seg = 7'b000_0110; // E
                4'hF: hex_to_7seg = 7'b000_1110; // F
            endcase
        end
    endfunction

    // Output Generation
    reg [6:0] int_seg;
    reg       int_dp;
    reg [3:0] int_an;

    wire disp_master_en = reg_ctrl[0];
    wire raw_mode       = reg_ctrl[1];
    wire invert_pol     = reg_ctrl[2];

    always @* begin
        if (!disp_master_en || !current_digit_en) begin
            // Digit blanked
            int_seg = 7'b111_1111; // all segments OFF (active-low)
            int_dp  = 1'b1;        // DP OFF (active-low)
            int_an  = 4'b1111;     // all anodes OFF (active-low)
        end else begin
            // Anode activation (one active-low anode active)
            case (active_digit_idx)
                2'b00: int_an = 4'b1110;
                2'b01: int_an = 4'b1101;
                2'b10: int_an = 4'b1011;
                2'b11: int_an = 4'b0111;
            endcase

            if (raw_mode) begin
                int_seg = current_raw_pattern[6:0];
                int_dp  = ~current_raw_pattern[7]; // bit 7 = 1 turns ON DP
            end else begin
                int_seg = hex_to_7seg(current_nibble);
                int_dp  = ~current_dp_en;          // 1 = ON (active-low 0)
            end
        end
    end

    // Polarity control (default: active-low for Basys3 common anode)
    assign seg_o = invert_pol ? ~int_seg : int_seg;
    assign dp_o  = invert_pol ? ~int_dp  : int_dp;
    assign an_o  = invert_pol ? ~int_an  : int_an;

endmodule

`default_nettype wire
