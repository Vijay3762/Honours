// =============================================================================
// File    : pooling_unit.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Configurable 2x2 Spatial Pooling Engine (Max, Average, Min, Bypass)
//           Designed for CNN feature map downsampling (e.g. 26x26 -> 13x13).
//
// Operational Modes:
//   - Mode 0 (MAX)    : Output = max(p0, p1, p2, p3)  [Signed comparison]
//   - Mode 1 (AVG)    : Output = (p0 + p1 + p2 + p3 + 2) >>> 2 [Rounded signed division by 4]
//   - Mode 2 (MIN)    : Output = min(p0, p1, p2, p3)  [Signed comparison]
//   - Mode 3 (BYPASS) : Output = p0
//
// Dual Execution Interfaces:
//   1. Block / Parallel Mode: Processes a 2x2 quad {p3, p2, p1, p0} simultaneously
//   2. Pipelined or Combinational operation via PIPELINED parameter
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module pooling_unit #(
    parameter integer DATA_WIDTH = 16,    // Signed feature width (e.g. 16-bit Q8.8 or INT16)
    parameter bit     PIPELINED  = 1      // 0 = Combinational, 1 = 1-stage registered pipeline
)(
    input  wire                             clk,
    input  wire                             rst,           // active-high

    // Configuration
    input  wire [1:0]                       mode_i,        // 00=Max, 01=Avg, 10=Min, 11=Bypass
    input  wire                             enable_i,      // Module enable

    // Input 2x2 block
    input  wire                             valid_i,
    output wire                             ready_o,
    input  wire signed [DATA_WIDTH-1:0]     p0_i,          // Top-left pixel
    input  wire signed [DATA_WIDTH-1:0]     p1_i,          // Top-right pixel
    input  wire signed [DATA_WIDTH-1:0]     p2_i,          // Bottom-left pixel
    input  wire signed [DATA_WIDTH-1:0]     p3_i,          // Bottom-right pixel

    // Output pooled result
    output reg                              valid_o,
    input  wire                             ready_i,
    output reg  signed [DATA_WIDTH-1:0]     data_o,

    // Status flags
    output reg  [1:0]                       max_idx_o      // Index of maximum pixel (for backprop / argmax)
);

    localparam [1:0] MODE_MAX    = 2'b00;
    localparam [1:0] MODE_AVG    = 2'b01;
    localparam [1:0] MODE_MIN    = 2'b10;
    localparam [1:0] MODE_BYPASS = 2'b11;

    assign ready_o = !valid_o || ready_i;

    // -------------------------------------------------------------------------
    // Combinational Pooling Logic
    // -------------------------------------------------------------------------

    // Level 1: Pairwise Max Comparisons
    wire signed [DATA_WIDTH-1:0] max_p01     = (p0_i >= p1_i) ? p0_i : p1_i;
    wire [1:0]                   idx_p01     = (p0_i >= p1_i) ? 2'd0 : 2'd1;

    wire signed [DATA_WIDTH-1:0] max_p23     = (p2_i >= p3_i) ? p2_i : p3_i;
    wire [1:0]                   idx_p23     = (p2_i >= p3_i) ? 2'd2 : 2'd3;

    // Level 2: Final Max
    wire signed [DATA_WIDTH-1:0] max_result  = (max_p01 >= max_p23) ? max_p01 : max_p23;
    wire [1:0]                   max_idx_comb= (max_p01 >= max_p23) ? idx_p01 : idx_p23;

    // Level 1: Pairwise Min Comparisons
    wire signed [DATA_WIDTH-1:0] min_p01     = (p0_i <= p1_i) ? p0_i : p1_i;
    wire signed [DATA_WIDTH-1:0] min_p23     = (p2_i <= p3_i) ? p2_i : p3_i;
    wire signed [DATA_WIDTH-1:0] min_result  = (min_p01 <= min_p23) ? min_p01 : min_p23;

    // Average Pooling: Extended sum with rounding
    // 4 inputs of DATA_WIDTH -> sum needs DATA_WIDTH+2 bits
    wire signed [DATA_WIDTH+2:0] ext_sum     = (DATA_WIDTH+3)'(p0_i) +
                                               (DATA_WIDTH+3)'(p1_i) +
                                               (DATA_WIDTH+3)'(p2_i) +
                                               (DATA_WIDTH+3)'(p3_i);
    // Symmetric rounding (+2 before >> 2)
    wire signed [DATA_WIDTH+2:0] ext_sum_rnd = ext_sum + (DATA_WIDTH+3)'(2);
    wire signed [DATA_WIDTH-1:0] avg_result  = ext_sum_rnd[DATA_WIDTH+1:2];

    // Mode Multiplexer
    reg signed [DATA_WIDTH-1:0] result_comb;

    always @* begin
        if (!enable_i) begin
            result_comb = {DATA_WIDTH{1'b0}};
        end else begin
            case (mode_i)
                MODE_MAX:    result_comb = max_result;
                MODE_AVG:    result_comb = avg_result;
                MODE_MIN:    result_comb = min_result;
                MODE_BYPASS: result_comb = p0_i;
                default:     result_comb = max_result;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Output Stage (Pipelined vs Combinational)
    // -------------------------------------------------------------------------
    generate
        if (PIPELINED) begin : gen_pipelined
            always @(posedge clk) begin
                if (rst) begin
                    valid_o   <= 1'b0;
                    data_o    <= {DATA_WIDTH{1'b0}};
                    max_idx_o <= 2'b00;
                end else if (ready_o) begin
                    valid_o   <= valid_i && enable_i;
                    if (valid_i && enable_i) begin
                        data_o    <= result_comb;
                        max_idx_o <= max_idx_comb;
                    end else begin
                        data_o    <= {DATA_WIDTH{1'b0}};
                        max_idx_o <= 2'b00;
                    end
                end
            end
        end else begin : gen_comb
            always @* begin
                valid_o   = valid_i && enable_i;
                data_o    = result_comb;
                max_idx_o = max_idx_comb;
            end
        end
    endgenerate

endmodule

`default_nettype wire
