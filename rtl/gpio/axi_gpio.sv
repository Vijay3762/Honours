// =============================================================================
// File    : axi_gpio.sv
// Project : RISC-V CNN SoC Honours
// Purpose : Parameterized AXI4 GPIO IP block.
//
//           Provides a memory-mapped GPIO controller with per-bit direction
//           control, set/clear/toggle action registers, and two-flop
//           synchronizers on asynchronous GPIO inputs.
//
//           The AXI4 slave interface matches the existing SoC interconnect
//           (64-bit data, 32-bit address, 8-bit ID, 8-bit strobe).
//           GPIO register data occupies the lower 32 bits of the 64-bit
//           data bus; the upper 32 bits read as zero on reads and are
//           ignored on writes.
//
// AXI interface widths (match axi_interconnect_wrap_3x8 instantiation):
//   DATA_WIDTH = 64, ADDR_WIDTH = 32, ID_WIDTH = 8, STRB_WIDTH = 8
//
// Register map (32-bit registers, word-aligned):
//   Offset  Register         Access  Description
//   ------  ---------------  ------  ----------------------------------------
//   0x00    GPIO_IN          RO      Synchronized GPIO input values
//   0x04    GPIO_OUT         RW      Output data register
//   0x08    GPIO_DIR         RW      Direction: 1=output, 0=input
//   0x0C    GPIO_OUT_SET     WO      Set selected output bits (mask)
//   0x10    GPIO_OUT_CLR     WO      Clear selected output bits (mask)
//   0x14    GPIO_OUT_TOGGLE  WO      Toggle selected output bits (mask)
//   0x18    GPIO_ID          RO      Fixed peripheral ID
//   0x1C    GPIO_VERSION     RO      Fixed version
//
// Reset: active-high (rst).  Matches the interconnect reset convention.
//   On reset: OUT=0, DIR=0 (all inputs), synchronizers=0, AXI state cleared.
//
// AXI response codes:
//   OKAY   = 2'b00  -- valid register access
//   SLVERR = 2'b10  -- reserved offset, write to RO register, or misaligned
//   DECERR = 2'b11  -- address outside the valid 4 KiB register aperture
//
// Burst policy:
//   Single-beat transfers (len=0) are fully supported.
//   Burst writes: all data beats are drained (accepted via W channel through
//     WLAST); a single SLVERR B response is returned.
//   Burst reads: the full number of beats is returned, each with SLVERR,
//     and RLAST is asserted on the final beat.
//
// All accepted transactions eventually complete; no deadlock.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module axi_gpio #(
    // GPIO width (default 16)
    parameter integer GPIO_WIDTH = 16,

    // AXI bus widths -- match the interconnect defaults
    parameter integer AXI_DATA_WIDTH = 64,
    parameter integer AXI_ADDR_WIDTH = 32,
    parameter integer AXI_ID_WIDTH   = 8,
    parameter integer AXI_STRB_WIDTH = AXI_DATA_WIDTH / 8,

    // Register aperture: 4 KiB (12 address bits)
    parameter integer REG_ADDR_WIDTH = 12,

    // Base address of this GPIO slave in the system memory map.
    // When connected via an AXI interconnect that passes the full system
    // address through (no address translation), set this to the slave's
    // base address (e.g. 32'h6000_0000 for m06). The IP subtracts it from
    // incoming addresses before register decode.  Default 0 keeps standalone
    // behaviour unchanged (offsets 0x000–0xFFF).
    parameter [AXI_ADDR_WIDTH-1:0] BASE_ADDR = 32'h0,

    // Fixed peripheral ID and version constants
    parameter [31:0] GPIO_ID_VALUE      = 32'h4749504F,  // "GPIO" in ASCII
    parameter [31:0] GPIO_VERSION_VALUE = 32'h00010000    // v1.0
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

    // GPIO physical pins
    input  wire [GPIO_WIDTH-1:0]           gpio_i,       // async inputs
    output wire [GPIO_WIDTH-1:0]           gpio_o,       // output data
    output wire [GPIO_WIDTH-1:0]           gpio_oen      // output enable
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam integer REG_DATA_WIDTH = 32;

    // GPIO mask: valid bits within a 32-bit register
    localparam [31:0] GPIO_MASK = (GPIO_WIDTH >= 32) ?
                                    32'hFFFF_FFFF :
                                    (32'(1) << GPIO_WIDTH) - 1;

    // AXI response codes
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // Write FSM states
    localparam [2:0] WR_IDLE    = 3'd0;
    localparam [2:0] WR_WAIT_W  = 3'd1;  // AW accepted, waiting for W
    localparam [2:0] WR_WAIT_AW = 3'd2;  // W accepted, waiting for AW
    localparam [2:0] WR_DRAIN   = 3'd3;  // burst write, draining W beats
    localparam [2:0] WR_RESP    = 3'd4;  // B response pending

    // Read FSM states
    localparam [1:0] RD_IDLE  = 2'd0;
    localparam [1:0] RD_RESP  = 2'd1;

    // =========================================================================
    // Address validation (combinational)
    // =========================================================================
    // Offset from the slave's base address in the system memory map.
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
    // GPIO input synchronizers (two-flop)
    // =========================================================================
    reg [GPIO_WIDTH-1:0] gpio_sync1;
    reg [GPIO_WIDTH-1:0] gpio_sync2;

    always @(posedge clk) begin
        if (rst) begin
            gpio_sync1 <= '0;
            gpio_sync2 <= '0;
        end else begin
            gpio_sync1 <= gpio_i;
            gpio_sync2 <= gpio_sync1;
        end
    end

    wire [GPIO_WIDTH-1:0] gpio_in_sync = gpio_sync2;

    // =========================================================================
    // Register storage
    // =========================================================================
    reg [GPIO_WIDTH-1:0] gpio_out_reg;
    reg [GPIO_WIDTH-1:0] gpio_dir_reg;

    // =========================================================================
    // Physical output mux
    // =========================================================================
    assign gpio_o   = gpio_dir_reg & gpio_out_reg;
    assign gpio_oen = gpio_dir_reg;

    // =========================================================================
    // Combinational: compute next value of gpio_out_reg given a write
    // =========================================================================
    function [31:0] compute_next_out;
        input [3:0]            reg_sel;
        input [31:0]           wdata;
        input [3:0]            wstrb;
        input [GPIO_WIDTH-1:0] cur_out;
        reg [31:0] result;
        integer i;
        begin
            result = {{(32-GPIO_WIDTH){1'b0}}, cur_out};
            case (reg_sel)
                4'h1: begin // GPIO_OUT — RW
                    for (i = 0; i < 4; i = i + 1) begin
                        if (wstrb[i])
                            result[i*8 +: 8] = wdata[i*8 +: 8] & GPIO_MASK[i*8 +: 8];
                    end
                end
                4'h3: begin // SET: out |= (wdata & mask)
                    for (i = 0; i < 4; i = i + 1) begin
                        if (wstrb[i])
                            result[i*8 +: 8] = result[i*8 +: 8] |
                                (wdata[i*8 +: 8] & GPIO_MASK[i*8 +: 8]);
                    end
                end
                4'h4: begin // CLR: out &= ~(wdata & mask)
                    for (i = 0; i < 4; i = i + 1) begin
                        if (wstrb[i])
                            result[i*8 +: 8] = result[i*8 +: 8] &
                                ~(wdata[i*8 +: 8] & GPIO_MASK[i*8 +: 8]);
                    end
                end
                4'h5: begin // TOGGLE: out ^= (wdata & mask)
                    for (i = 0; i < 4; i = i + 1) begin
                        if (wstrb[i])
                            result[i*8 +: 8] = result[i*8 +: 8] ^
                                (wdata[i*8 +: 8] & GPIO_MASK[i*8 +: 8]);
                    end
                end
                default: ; // no change for RO or reserved
            endcase
            compute_next_out = result;
        end
    endfunction

    // Combinational: compute next value of gpio_dir_reg
    function [31:0] compute_next_dir;
        input [3:0]            reg_sel;
        input [31:0]           wdata;
        input [3:0]            wstrb;
        input [GPIO_WIDTH-1:0] cur_dir;
        reg [31:0] result;
        integer i;
        begin
            result = {{(32-GPIO_WIDTH){1'b0}}, cur_dir};
            if (reg_sel == 4'h2) begin // GPIO_DIR — RW
                for (i = 0; i < 4; i = i + 1) begin
                    if (wstrb[i])
                        result[i*8 +: 8] = wdata[i*8 +: 8] & GPIO_MASK[i*8 +: 8];
                end
            end
            compute_next_dir = result;
        end
    endfunction

    // Combinational: determine write response for a given reg_sel
    function [1:0] compute_write_resp;
        input [3:0] reg_sel;
        begin
            case (reg_sel)
                4'h1, 4'h2, 4'h3, 4'h4, 4'h5:
                    compute_write_resp = RESP_OKAY;
                4'h0, 4'h6, 4'h7:
                    compute_write_resp = RESP_SLVERR; // RO registers
                default:
                    compute_write_resp = RESP_SLVERR; // reserved
            endcase
        end
    endfunction

    // Combinational: determine read response for a given reg_sel
    function [1:0] compute_read_resp;
        input [3:0] reg_sel;
        begin
            case (reg_sel)
                4'h0, 4'h1, 4'h2, 4'h3, 4'h4, 4'h5, 4'h6, 4'h7:
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
    reg [1:0]               wr_error_r;   // RESP_OKAY / RESP_SLVERR / RESP_DECERR
    reg [31:0]              wr_data_r;
    reg [3:0]               wr_strb_r;
    reg [1:0]               wr_resp_r;
    reg                     b_valid_r;
    reg                     wr_aw_offset2_r;
    reg                     wr_hi_lane_r;

    // Temporary registers to hold function results before slicing
    // (some simulators do not support direct indexing of function returns)
    reg [31:0]              nxt_out;
    reg [31:0]              nxt_dir;

    // 64-bit sub-word / lane steering:
    // High lane [63:32] is selected if strobes are only in [7:4],
    // or if the address explicitly has bit 2 set and strobes are in [7:4].
    wire w_hi_lane = (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ||
                     (aw_offset[2] && s_axi_wstrb[7:4] != 4'h0);
    wire [31:0] w_data_eff = w_hi_lane ? s_axi_wdata[63:32] : s_axi_wdata[31:0];
    wire [3:0]  w_strb_eff = w_hi_lane ? s_axi_wstrb[7:4]    : s_axi_wstrb[3:0];

    // Effective register selector for 64-bit masters issuing 8-byte aligned addresses
    wire [3:0] aw_reg_sel_eff = (aw_offset[2:0] == 3'b000 && (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0)) ?
                                (aw_reg_sel | 4'h1) : aw_reg_sel;

    // AW ready: accept AW in IDLE or WAIT_AW states
    assign s_axi_awready = s_axi_awvalid &&
                          (wr_state == WR_IDLE || wr_state == WR_WAIT_AW);

    // W ready: accept W in IDLE, WAIT_W, or DRAIN
    assign s_axi_wready = s_axi_wvalid &&
                         (wr_state == WR_IDLE ||
                          wr_state == WR_WAIT_W ||
                          wr_state == WR_DRAIN);

    // Helper wires for WAIT_W and WAIT_AW states
    wire w_hi_wait_w = (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ||
                       (wr_aw_offset2_r && s_axi_wstrb[7:4] != 4'h0);
    wire [31:0] w_data_wait_w = w_hi_wait_w ? s_axi_wdata[63:32] : s_axi_wdata[31:0];
    wire [3:0]  w_strb_wait_w = w_hi_wait_w ? s_axi_wstrb[7:4]    : s_axi_wstrb[3:0];
    wire [3:0]  eff_reg_wait_w = (!wr_aw_offset2_r && (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0)) ?
                                 (wr_reg_sel_r | 4'h1) : wr_reg_sel_r;

    wire [3:0] eff_reg_wait_aw = (aw_offset[2:0] == 3'b000 && wr_hi_lane_r) ?
                                 (aw_reg_sel | 4'h1) : aw_reg_sel;

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
            gpio_out_reg    <= '0;
            gpio_dir_reg    <= '0;
        end else begin
            case (wr_state)
                // -------------------------------------------------------------
                // WR_IDLE: waiting for AW and/or W (independent arrival)
                // -------------------------------------------------------------
                WR_IDLE: begin
                    if (s_axi_awvalid && s_axi_awready &&
                        s_axi_wvalid && s_axi_wready && !aw_burst) begin
                        // Both AW and W in same cycle (single-beat)
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
                        end
                        if (aw_in_aperture && aw_aligned) begin
                            nxt_out = compute_next_out(aw_reg_sel_eff, w_data_eff, w_strb_eff, gpio_out_reg);
                            nxt_dir = compute_next_dir(aw_reg_sel_eff, w_data_eff, w_strb_eff, gpio_dir_reg);
                            gpio_out_reg <= nxt_out[GPIO_WIDTH-1:0];
                            gpio_dir_reg <= nxt_dir[GPIO_WIDTH-1:0];
                        end
                        wr_state  <= WR_RESP;
                        b_valid_r <= 1'b1;
                    end
                    else if (s_axi_awvalid && s_axi_awready && aw_burst) begin
                        // Burst write: enter drain mode
                        wr_state     <= WR_DRAIN;
                        wr_id_r      <= s_axi_awid;
                        wr_error_r   <= RESP_SLVERR;
                        wr_reg_sel_r <= aw_reg_sel;
                    end
                    else if (s_axi_awvalid && s_axi_awready) begin
                        // AW arrived (single), W not yet
                        wr_state        <= WR_WAIT_W;
                        wr_id_r         <= s_axi_awid;
                        wr_reg_sel_r    <= aw_reg_sel;
                        wr_aw_offset2_r <= aw_offset[2];
                        wr_error_r      <= !aw_in_aperture ? RESP_DECERR :
                                           !aw_aligned     ? RESP_SLVERR : RESP_OKAY;
                    end
                    else if (s_axi_wvalid && s_axi_wready) begin
                        // W arrived before AW
                        wr_state     <= WR_WAIT_AW;
                        wr_hi_lane_r <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0);
                        wr_data_r    <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ?
                                        s_axi_wdata[63:32] : s_axi_wdata[31:0];
                        wr_strb_r    <= (s_axi_wstrb[3:0] == 4'h0 && s_axi_wstrb[7:4] != 4'h0) ?
                                        s_axi_wstrb[7:4]    : s_axi_wstrb[3:0];
                    end
                end

                // -------------------------------------------------------------
                // WR_WAIT_W: AW accepted (single), waiting for W
                // -------------------------------------------------------------
                WR_WAIT_W: begin
                    if (s_axi_wvalid && s_axi_wready) begin
                        wr_resp_r <= (wr_error_r == RESP_OKAY) ?
                                     compute_write_resp(eff_reg_wait_w) : wr_error_r;
                        if (wr_error_r == RESP_OKAY) begin
                            nxt_out = compute_next_out(eff_reg_wait_w, w_data_wait_w, w_strb_wait_w, gpio_out_reg);
                            nxt_dir = compute_next_dir(eff_reg_wait_w, w_data_wait_w, w_strb_wait_w, gpio_dir_reg);
                            gpio_out_reg <= nxt_out[GPIO_WIDTH-1:0];
                            gpio_dir_reg <= nxt_dir[GPIO_WIDTH-1:0];
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
                            end
                            if (aw_in_aperture && aw_aligned) begin
                                nxt_out = compute_next_out(eff_reg_wait_aw, wr_data_r, wr_strb_r, gpio_out_reg);
                                nxt_dir = compute_next_dir(eff_reg_wait_aw, wr_data_r, wr_strb_r, gpio_dir_reg);
                                gpio_out_reg <= nxt_out[GPIO_WIDTH-1:0];
                                gpio_dir_reg <= nxt_dir[GPIO_WIDTH-1:0];
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

    // Read data mux (combinational from live address)
    reg [31:0] reg_read_data;
    always @(*) begin
        case (ar_reg_sel)
            4'h0:   reg_read_data = {{(32-GPIO_WIDTH){1'b0}}, gpio_in_sync};
            4'h1:   reg_read_data = {{(32-GPIO_WIDTH){1'b0}}, gpio_out_reg};
            4'h2:   reg_read_data = {{(32-GPIO_WIDTH){1'b0}}, gpio_dir_reg};
            4'h3:   reg_read_data = 32'h0;
            4'h4:   reg_read_data = 32'h0;
            4'h5:   reg_read_data = 32'h0;
            4'h6:   reg_read_data = GPIO_ID_VALUE;
            4'h7:   reg_read_data = GPIO_VERSION_VALUE;
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
                // -------------------------------------------------------------
                // RD_IDLE: waiting for AR
                // -------------------------------------------------------------
                RD_IDLE: begin
                    if (s_axi_arvalid && s_axi_arready) begin
                        rd_id_r  <= s_axi_arid;
                        rd_len_r <= s_axi_arlen;

                        if (ar_burst) begin
                            // Burst read: all beats are SLVERR
                            rd_data_r <= {AXI_DATA_WIDTH{1'b0}};
                            rd_resp_r <= RESP_SLVERR;
                            r_valid_r <= 1'b1;
                            rd_last_r <= (s_axi_arlen == 8'd0);
                        end else begin
                            // Single-beat read
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

                // -------------------------------------------------------------
                // RD_RESP: R response in progress
                // -------------------------------------------------------------
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
