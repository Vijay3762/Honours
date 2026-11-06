// =============================================================================
// File    : axi_pooling.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Memory-Mapped AXI4 Slave Wrapper for the 2x2 Pooling Engine
//
// Register Map:
//   Offset  Name             Access  Description
//   ------  ---------------  ------  ----------------------------------------
//   0x00    REG_CTRL         RW      [0] Enable, [2:1] Mode (0=Max, 1=Avg, 2=Min, 3=Bypass)
//   0x04    REG_STATUS       RO      [0] Busy, [1] Done, [2] Result Valid
//   0x08    REG_CONFIG       RW      [15:0] Target Quad Count
//   0x0C    REG_DATA_IN_P01  RW      [15:0] Pixel 0, [31:16] Pixel 1
//   0x10    REG_DATA_IN_P23  WO      [15:0] Pixel 2, [31:16] Pixel 3 (Triggers computation)
//   0x14    REG_DATA_OUT     RO      [15:0] Pooled result, [17:16] Argmax index
//   0x18    REG_ID           RO      Fixed Peripheral ID (0x504F4F4C = "POOL")
//   0x1C    REG_VERSION      RO      Fixed Version (0x00010000 = v1.0)
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_pooling #(
    parameter integer AXI_DATA_WIDTH       = 64,
    parameter integer AXI_ADDR_WIDTH       = 32,
    parameter integer AXI_ID_WIDTH         = 8,
    parameter integer AXI_STRB_WIDTH       = AXI_DATA_WIDTH / 8,
    parameter integer REG_ADDR_WIDTH       = 12,
    parameter [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h0000_0000,
    parameter [31:0] POOLING_ID_VALUE      = 32'h504F4F4C, // "POOL"
    parameter [31:0] POOLING_VER_VALUE     = 32'h00010000
)(
    input  wire                            clk,
    input  wire                            rst,

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
    input  wire                            s_axi_rready
);

    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    localparam [2:0] WR_IDLE    = 3'd0;
    localparam [2:0] WR_WAIT_W  = 3'd1;
    localparam [2:0] WR_WAIT_AW = 3'd2;
    localparam [2:0] WR_DRAIN   = 3'd3;
    localparam [2:0] WR_RESP    = 3'd4;

    localparam [1:0] RD_IDLE    = 2'd0;
    localparam [1:0] RD_RESP    = 2'd1;

    // Address Decode
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

    // Control & Data Registers
    reg [2:0]  reg_ctrl;        // [0] enable, [2:1] mode
    reg [15:0] reg_config;      // [15:0] target quad count
    reg [31:0] reg_data_p01;    // [15:0] p0, [31:16] p1
    reg [31:0] reg_data_p23;    // [15:0] p2, [31:16] p3
    reg        trigger_pool;

    // Output signals from pooling core
    wire                       core_valid_o;
    wire signed [15:0]         core_data_o;
    wire [1:0]                 core_max_idx_o;

    reg [31:0] reg_data_out;
    reg        reg_result_valid;

    // Pooling Core Instantiation
    pooling_unit #(
        .DATA_WIDTH (16),
        .PIPELINED  (1)
    ) pool_core (
        .clk        (clk),
        .rst        (rst),
        .mode_i     (reg_ctrl[2:1]),
        .enable_i   (reg_ctrl[0]),
        .valid_i    (trigger_pool),
        .ready_o    (),
        .p0_i       (reg_data_p01[15:0]),
        .p1_i       (reg_data_p01[31:16]),
        .p2_i       (reg_data_p23[15:0]),
        .p3_i       (reg_data_p23[31:16]),
        .valid_o    (core_valid_o),
        .ready_i    (1'b1),
        .data_o     (core_data_o),
        .max_idx_o  (core_max_idx_o)
    );

    reg reg_busy;

    always @(posedge clk) begin
        if (rst) begin
            reg_data_out     <= 32'h0;
            reg_result_valid <= 1'b0;
            reg_busy         <= 1'b0;
        end else begin
            if (trigger_pool) begin
                reg_result_valid <= 1'b0;
                reg_busy         <= 1'b1;
            end else if (core_valid_o) begin
                reg_data_out     <= {14'h0, core_max_idx_o, core_data_o};
                reg_result_valid <= 1'b1;
                reg_busy         <= 1'b0;
            end
        end
    end

    // Status word: [0] Busy, [1] Done, [2] Result Valid
    wire [31:0] reg_status = {29'h0, reg_result_valid, reg_result_valid, (reg_busy | trigger_pool)};

    // Register Read logic
    reg [31:0] reg_read_data;
    always @* begin
        case (ar_reg_sel)
            4'h0: reg_read_data = {29'h0, reg_ctrl};
            4'h1: reg_read_data = reg_status;
            4'h2: reg_read_data = {16'h0, reg_config};
            4'h3: reg_read_data = reg_data_p01;
            4'h4: reg_read_data = 32'h0; // WO register
            4'h5: reg_read_data = reg_data_out;
            4'h6: reg_read_data = POOLING_ID_VALUE;
            4'h7: reg_read_data = POOLING_VER_VALUE;
            default: reg_read_data = 32'h0;
        endcase
    end

    function [1:0] compute_write_resp;
        input [3:0] sel;
        begin
            case (sel)
                4'h0, 4'h2, 4'h3, 4'h4: compute_write_resp = RESP_OKAY;
                4'h1, 4'h5, 4'h6, 4'h7: compute_write_resp = RESP_SLVERR; // RO registers
                default: compute_write_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    function [1:0] compute_read_resp;
        input [3:0] sel;
        begin
            case (sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6, 4'h7: compute_read_resp = RESP_OKAY;
                default: compute_read_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    // Write FSM
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
                4'h0: begin // REG_CTRL
                    if (strb[0]) reg_ctrl <= data[2:0];
                end
                4'h2: begin // REG_CONFIG
                    if (strb[0]) reg_config[7:0]  <= data[7:0];
                    if (strb[1]) reg_config[15:8] <= data[15:8];
                end
                4'h3: begin // REG_DATA_IN_P01
                    for (i = 0; i < 4; i = i + 1) begin
                        if (strb[i]) reg_data_p01[i*8 +: 8] <= data[i*8 +: 8];
                    end
                end
                4'h4: begin // REG_DATA_IN_P23 -> triggers pool
                    for (i = 0; i < 4; i = i + 1) begin
                        if (strb[i]) reg_data_p23[i*8 +: 8] <= data[i*8 +: 8];
                    end
                    trigger_pool <= 1'b1;
                end
                default: ;
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

            reg_ctrl        <= 3'b001; // default: enable=1, mode=00 (Max)
            reg_config      <= 16'h0001;
            reg_data_p01    <= 32'h0;
            reg_data_p23    <= 32'h0;
            trigger_pool    <= 1'b0;
        end else begin
            trigger_pool <= 1'b0; // 1-cycle strobe

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

    // Read FSM
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

endmodule

`default_nettype wire
