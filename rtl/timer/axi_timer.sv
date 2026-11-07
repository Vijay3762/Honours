// =============================================================================
// File    : axi_timer.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Parameterized AXI4 Timer IP block.
//
//           Provides a 32-bit down-counter timer with programmable load value,
//           compare/match interrupt, auto-reload (continuous) mode, and
//           write-1-to-clear status flags.
//
//           The AXI4 slave interface matches the existing SoC interconnect
//           (64-bit data, 32-bit address, 8-bit ID, 8-bit strobe).
//           Timer register data occupies the lower 32 bits of the 64-bit
//           data bus; the upper 32 bits read as zero on reads and are
//           ignored on writes.
//
// Register map (32-bit registers, word-aligned):
//   Offset  Register         Access  Description
//   ------  ---------------  ------  ----------------------------------------
//   0x00    TIMER_CTRL       RW      bit0: enable, bit1: auto-reload,
//                                   bit2: interrupt enable
//   0x04    TIMER_LOAD       RW      Counter initial load value
//   0x08    TIMER_VALUE      RO      Current counter value
//   0x0C    TIMER_COMPARE    RW      Compare value for match interrupt
//   0x10    TIMER_STATUS     RW      bit0: match flag (W1C), bit1: overflow
//                                   flag (W1C)
//   0x14    TIMER_ID         RO      Fixed peripheral ID ("TIMR")
//   0x18    TIMER_VERSION    RO      Fixed version
//
// Reset: active-high (rst).
//   On reset: counter=0, CTRL=0, STATUS=0, IRQ=0.
//
// AXI response codes:
//   OKAY   = 2'b00  -- valid register access
//   SLVERR = 2'b10  -- reserved offset, write to RO register, or misaligned
//   DECERR = 2'b11  -- address outside the valid 4 KiB register aperture
//
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_timer #(
    parameter integer TIMER_WIDTH      = 32,

    // AXI bus widths -- match the interconnect defaults
    parameter integer AXI_DATA_WIDTH  = 64,
    parameter integer AXI_ADDR_WIDTH  = 32,
    parameter integer AXI_ID_WIDTH    = 8,
    parameter integer AXI_STRB_WIDTH  = AXI_DATA_WIDTH / 8,

    // Register aperture: 4 KiB (12 address bits)
    parameter integer REG_ADDR_WIDTH  = 12,

    // Base address of this timer slave in the system memory map.
    // When connected via an AXI interconnect that passes the full system
    // address through (no address translation), set this to the slave's
    // base address (e.g. 32'h5000_0000 for m05).
    parameter [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h0,

    // Fixed peripheral ID and version constants
    parameter [31:0] TIMER_ID_VALUE      = 32'h54494D52,  // "TIMR" in ASCII
    parameter [31:0] TIMER_VERSION_VALUE = 32'h00010000   // v1.0
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

    // Timer outputs
    output wire                            timer_irq_o   // interrupt (active-high)
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam integer REG_DATA_WIDTH = 32;

    // Timer mask: valid bits within a 32-bit register
    localparam [31:0] TIMER_MASK = (TIMER_WIDTH >= 32) ?
                                    32'hFFFF_FFFF :
                                    (32'(1) << TIMER_WIDTH) - 1;

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

    // Register select from word offset (bits [5:2] of offset address)
    wire [3:0] aw_reg_sel = aw_offset[5:2];
    wire [3:0] ar_reg_sel = ar_offset[5:2];

    wire aw_addr_error = !aw_in_aperture || !aw_aligned;
    wire ar_addr_error = !ar_in_aperture || !ar_aligned;

    // =========================================================================
    // Timer control registers
    // =========================================================================
    reg [TIMER_WIDTH-1:0] timer_counter;   // current count value
    reg                   timer_enable;    // CTRL bit0: counter enable
    reg                   timer_autoload;  // CTRL bit1: auto-reload at zero
    reg                   timer_irq_en;    // CTRL bit2: interrupt enable
    reg [TIMER_WIDTH-1:0] timer_load;      // LOAD register (initial value)
    reg [TIMER_WIDTH-1:0] timer_compare;   // COMPARE register
    reg                   timer_match;     // STATUS bit0: match flag (set by HW, W1C by SW)
    reg                   timer_overflow;  // STATUS bit1: overflow flag (set by HW, W1C by SW)
    reg                   timer_enable_prev; // Previous enable value (edge detect)

    // Counter condition flags
    wire counter_hit_zero = (timer_counter == {TIMER_WIDTH{1'b0}});
    wire counter_hit_cmp  = (timer_counter == timer_compare) &&
                            (timer_compare != {TIMER_WIDTH{1'b0}});
    wire counter_match    = counter_hit_zero || counter_hit_cmp;
    wire enable_rising_edge = timer_enable && !timer_enable_prev;

    // Interrupt output (active-high)
    assign timer_irq_o = timer_match & timer_irq_en;

    // Software clear flags from STATUS register write (W1C)
    reg sw_clear_match;
    reg sw_clear_overflow;

    // =========================================================================
    // Combinational: determine write response for a given reg_sel
    // =========================================================================
    function [1:0] compute_write_resp;
        input [3:0] reg_sel;
        begin
            case (reg_sel)
                4'h0, 4'h1, 4'h3, 4'h4:
                    compute_write_resp = RESP_OKAY; // CTRL, LOAD, COMPARE, STATUS
                4'h2, 4'h5, 4'h6:
                    compute_write_resp = RESP_SLVERR; // RO registers (VALUE, ID, VERSION)
                default:
                    compute_write_resp = RESP_SLVERR; // reserved
            endcase
        end
    endfunction

    // =========================================================================
    // Combinational: determine read response for a given reg_sel
    // =========================================================================
    function [1:0] compute_read_resp;
        input [3:0] reg_sel;
        begin
            case (reg_sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6:
                    compute_read_resp = RESP_OKAY;
                default:
                    compute_read_resp = RESP_SLVERR;
            endcase
        end
    endfunction

    // =========================================================================
    // Write channel FSM
    // =========================================================================
    reg [2:0]               wr_state;
    reg [AXI_ID_WIDTH-1:0]   wr_id_r;
    reg [3:0]               wr_reg_sel_r;
    reg [1:0]               wr_error_r;
    reg [31:0]              wr_data_r;
    reg [3:0]               wr_strb_r;
    reg [1:0]               wr_resp_r;
    reg                     b_valid_r;
    reg                     wr_aw_offset2_r;

    // 64-bit sub-word / lane steering
    wire w_hi_lane = (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ||
                     (aw_offset[2] && s_axi_wstrb[7:4] != 4'h0);
    wire [31:0] w_data_lower = w_hi_lane ? s_axi_wdata[63:32] : s_axi_wdata[31:0];
    wire [3:0]  w_strb_lower = w_hi_lane ? s_axi_wstrb[7:4]   : s_axi_wstrb[3:0];

    wire w_hi_wait_w = (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ||
                       (wr_aw_offset2_r && s_axi_wstrb[7:4] != 4'h0);
    wire [31:0] w_data_wait_w = w_hi_wait_w ? s_axi_wdata[63:32] : s_axi_wdata[31:0];
    wire [3:0]  w_strb_wait_w = w_hi_wait_w ? s_axi_wstrb[7:4]   : s_axi_wstrb[3:0];

    // AW ready: accept AW in IDLE or WAIT_AW states
    assign s_axi_awready = s_axi_awvalid &&
                          (wr_state == WR_IDLE || wr_state == WR_WAIT_AW);

    // W ready: accept W in IDLE, WAIT_W, or DRAIN
    assign s_axi_wready = s_axi_wvalid &&
                         (wr_state == WR_IDLE ||
                          wr_state == WR_WAIT_W ||
                          wr_state == WR_DRAIN);

    // Temporary registers to hold computed values
    reg [31:0] nxt_ctrl;

    // One-shot countdown completion condition (auto-clears timer_enable in write FSM)
    wire timer_one_shot_done = timer_enable && !enable_rising_edge && counter_hit_zero && !timer_autoload;

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

            // Timer registers
            timer_enable   <= 1'b0;
            timer_autoload <= 1'b0;
            timer_irq_en   <= 1'b0;
            timer_load     <= '0;
            timer_compare  <= {TIMER_WIDTH{1'b1}};

            sw_clear_match     <= 1'b0;
            sw_clear_overflow  <= 1'b0;
        end else begin
            // Default: deassert software clear strobes (single-cycle pulses)
            sw_clear_match     <= 1'b0;
            sw_clear_overflow  <= 1'b0;

            // One-shot timer auto-disable when counter reaches zero
            if (timer_one_shot_done)
                timer_enable <= 1'b0;

            case (wr_state)
                // -------------------------------------------------------------
                // WR_IDLE: waiting for AW and/or W (independent arrival)
                // -------------------------------------------------------------
                WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready &&
                        s_axi_wvalid && s_axi_wready && !aw_burst) begin
                        // Both AW and W in same cycle (single-beat)
                        wr_id_r         <= s_axi_awid;
                        wr_reg_sel_r    <= aw_reg_sel;
                        wr_aw_offset2_r <= aw_offset[2];
                        if (!aw_in_aperture)
                            wr_error_r <= RESP_DECERR;
                        else if (!aw_aligned)
                            wr_error_r <= RESP_SLVERR;
                        else
                            wr_error_r <= RESP_OKAY;

                        wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                    compute_write_resp(aw_reg_sel) : wr_error_r;

                        // Handle register writes (only if no address error)
                        if (wr_error_r == RESP_OKAY) begin
                            case (aw_reg_sel)
                                4'h0: begin // TIMER_CTRL — RW
                                    if (w_strb_lower[0]) begin
                                        timer_enable   <= w_data_lower[0] & 1'b1;
                                        timer_autoload <= w_data_lower[1] & 1'b1;
                                        timer_irq_en   <= w_data_lower[2] & 1'b1;
                                    end
                                end
                                4'h1: begin // TIMER_LOAD — RW
                                    for (int i = 0; i < 4; i = i + 1) begin
                                        if (w_strb_lower[i])
                                            timer_load[i*8 +: 8] <= w_data_lower[i*8 +: 8] & TIMER_MASK[i*8 +: 8];
                                    end
                                end
                                4'h3: begin // TIMER_COMPARE — RW
                                    for (int i = 0; i < 4; i = i + 1) begin
                                        if (w_strb_lower[i])
                                            timer_compare[i*8 +: 8] <= w_data_lower[i*8 +: 8] & TIMER_MASK[i*8 +: 8];
                                    end
                                end
                                4'h4: begin // TIMER_STATUS — W1C
                                    if (w_strb_lower[0]) begin
                                        if (w_data_lower[0]) sw_clear_match     <= 1'b1;
                                        if (w_data_lower[1]) sw_clear_overflow  <= 1'b1;
                                    end
                                end
                                default: ; // no-op
                            endcase
                        end
                        wr_state  <= WR_RESP;
                        b_valid_r <= 1'b1;
                    end
                    else if (s_axi_awvalid && s_axi_awready && aw_burst) begin
                        wr_state        <= WR_DRAIN;
                        wr_id_r         <= s_axi_awid;
                        wr_error_r      <= RESP_SLVERR;
                        wr_reg_sel_r    <= aw_reg_sel;
                        wr_aw_offset2_r <= aw_offset[2];
                    end
                    else if (s_axi_awvalid && s_axi_awready) begin
                        // AW arrived (single), W not yet
                        wr_state        <= WR_WAIT_W;
                        wr_id_r         <= s_axi_awid;
                        wr_reg_sel_r    <= aw_reg_sel;
                        wr_aw_offset2_r <= aw_offset[2];
                        if (!aw_in_aperture)
                            wr_error_r <= RESP_DECERR;
                        else if (!aw_aligned)
                            wr_error_r <= RESP_SLVERR;
                        else
                            wr_error_r <= RESP_OKAY;
                    end
                    else if (s_axi_wvalid && s_axi_wready) begin
                        // W arrived before AW
                        wr_state  <= WR_WAIT_AW;
                        wr_data_r <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ?
                                     s_axi_wdata[63:32] : s_axi_wdata[31:0];
                        wr_strb_r <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ?
                                     s_axi_wstrb[7:4] : s_axi_wstrb[3:0];
                    end
                end

                // -------------------------------------------------------------
                // WR_WAIT_W: AW accepted (single), waiting for W
                // -------------------------------------------------------------
                WR_WAIT_W: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                     compute_write_resp(wr_reg_sel_r) : wr_error_r;

                        if (wr_error_r == RESP_OKAY) begin
                            case (wr_reg_sel_r)
                                4'h0: begin // TIMER_CTRL — RW
                                    if (w_strb_wait_w[0]) begin
                                        timer_enable   <= w_data_wait_w[0] & 1'b1;
                                        timer_autoload <= w_data_wait_w[1] & 1'b1;
                                        timer_irq_en   <= w_data_wait_w[2] & 1'b1;
                                    end
                                end
                                4'h1: begin // TIMER_LOAD — RW
                                    for (int i = 0; i < 4; i = i + 1) begin
                                        if (w_strb_wait_w[i])
                                            timer_load[i*8 +: 8] <= w_data_wait_w[i*8 +: 8] & TIMER_MASK[i*8 +: 8];
                                    end
                                end
                                4'h3: begin // TIMER_COMPARE — RW
                                    for (int i = 0; i < 4; i = i + 1) begin
                                        if (w_strb_wait_w[i])
                                            timer_compare[i*8 +: 8] <= w_data_wait_w[i*8 +: 8] & TIMER_MASK[i*8 +: 8];
                                    end
                                end
                                4'h4: begin // TIMER_STATUS — W1C
                                    if (w_strb_wait_w[0]) begin
                                        if (w_data_wait_w[0]) sw_clear_match     <= 1'b1;
                                        if (w_data_wait_w[1]) sw_clear_overflow  <= 1'b1;
                                    end
                                end
                                default: ;
                            endcase
                        end
                        wr_state  <= WR_RESP;
                        b_valid_r <= 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // WR_WAIT_AW: W accepted (single), waiting for AW
                // -------------------------------------------------------------
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
                                    4'h0: begin // TIMER_CTRL — RW
                                        if (wr_strb_r[0]) begin
                                            timer_enable   <= wr_data_r[0] & 1'b1;
                                            timer_autoload <= wr_data_r[1] & 1'b1;
                                            timer_irq_en   <= wr_data_r[2] & 1'b1;
                                        end
                                    end
                                    4'h1: begin // TIMER_LOAD — RW
                                        for (int i = 0; i < 4; i = i + 1) begin
                                            if (wr_strb_r[i])
                                                timer_load[i*8 +: 8] <= wr_data_r[i*8 +: 8] & TIMER_MASK[i*8 +: 8];
                                        end
                                    end
                                    4'h3: begin // TIMER_COMPARE — RW
                                        for (int i = 0; i < 4; i = i + 1) begin
                                            if (wr_strb_r[i])
                                                timer_compare[i*8 +: 8] <= wr_data_r[i*8 +: 8] & TIMER_MASK[i*8 +: 8];
                                        end
                                    end
                                    4'h4: begin // TIMER_STATUS — W1C
                                        if (wr_strb_r[0]) begin
                                            if (wr_data_r[0]) sw_clear_match     <= 1'b1;
                                            if (wr_data_r[1]) sw_clear_overflow  <= 1'b1;
                                        end
                                    end
                                    default: ;
                                endcase
                            end
                            wr_state  <= WR_RESP;
                            b_valid_r <= 1'b1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // WR_DRAIN: burst write, accept all W beats until WLAST
                // -------------------------------------------------------------
                WR_DRAIN: begin
                    if (s_axi_wvalid && s_axi_wready && s_axi_wlast) begin
                        wr_state  <= WR_RESP;
                        wr_resp_r <= RESP_SLVERR;
                        b_valid_r <= 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // WR_RESP: B response pending, wait for BREADY
                // -------------------------------------------------------------
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
    // Timer counter and status logic
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            timer_counter  <= '0;
            timer_match    <= 1'b0;
            timer_overflow <= 1'b0;
            timer_enable_prev <= 1'b0;
        end else begin
            timer_enable_prev <= timer_enable;

            // Handle software clears (W1C) — these have priority for clearing
            if (sw_clear_match)
                timer_match <= 1'b0;
            if (sw_clear_overflow)
                timer_overflow <= 1'b0;

            // Counter operation
            if (timer_enable) begin
                if (enable_rising_edge) begin
                    // First cycle of enable: load from LOAD register
                    timer_counter <= timer_load;
                end else if (counter_hit_zero) begin
                    // Reached zero during counting
                    if (timer_autoload) begin
                        timer_counter <= timer_load;
                    end else begin
                        timer_counter <= '0;
                    end
                    timer_match    <= 1'b1;
                    timer_overflow <= 1'b1;
                end else if (counter_hit_cmp) begin
                    // Compare match (non-zero compare value): assert match flag and continue countdown
                    timer_match   <= 1'b1;
                    timer_counter <= timer_counter - 1'b1;
                end else begin
                    timer_counter <= timer_counter - 1'b1;
                end
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
            4'h0:   reg_read_data = {29'b0, timer_overflow, timer_match,
                                     timer_irq_en, timer_autoload, timer_enable};
            4'h1:   reg_read_data = {{(32-TIMER_WIDTH){1'b0}}, timer_load};
            4'h2:   reg_read_data = {{(32-TIMER_WIDTH){1'b0}}, timer_counter};
            4'h3:   reg_read_data = {{(32-TIMER_WIDTH){1'b0}}, timer_compare};
            4'h4:   reg_read_data = {{30{1'b0}}, timer_overflow, timer_match};
            4'h5:   reg_read_data = TIMER_ID_VALUE;
            4'h6:   reg_read_data = TIMER_VERSION_VALUE;
            default: reg_read_data = 32'h0;
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
