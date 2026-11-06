// =============================================================================
// File    : relu_unit.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Parameterized Streaming ReLU (Rectified Linear Unit) Activation Unit.
//
// Description:
//   Applies the activation function element-wise to incoming convolution or
//   feature-map data:
//
//       ReLU(x) = max(0, x)
//
//   Negative signed inputs are clamped to zero; positive inputs are retained.
//   An optional runtime clipping upper-bound is supported:
//
//       ReLU_clip(x) = min(max(0, x), clip_max)
//
// Parameters:
//   DATA_WIDTH          - Bit width of input and output data (default: 32).
//   PIPELINE_STAGES     - Latency in clock cycles (1: registered output, 0: combinational).
//   CLIPPING_ENABLE     - Hardware generation of upper-bound clipping comparator (default: 1).
//   DEFAULT_CLIP_MAX    - Default clipping limit if clipping is active.
//
// Ports:
//   clk, rst            - Clock and active-high synchronous/asynchronous reset.
//   enable_i            - Unit enable (1: active, 0: clock-gated / frozen).
//   bypass_i            - Pass-through mode (1: output = input, 0: apply ReLU).
//   clip_en_i           - Runtime clipping enable (active when CLIPPING_ENABLE=1).
//   clip_max_i          - Saturation upper-bound when clip_en_i=1.
//   data_i              - Signed input data beat.
//   valid_i             - Input data valid handshake.
//   ready_o             - Unit ready to accept new input beat.
//   data_o              - Activated output data beat.
//   valid_o             - Output data valid handshake.
//   ready_i             - Downstream consumer ready handshake.
//   zero_flag_o         - Flag: asserted when output is zero (clamped or zero input).
//   active_flag_o       - Flag: asserted when output is strictly positive (> 0).
//   clipped_flag_o      - Flag: asserted when positive input was clamped to clip_max.
//
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module relu_unit #(
    parameter integer DATA_WIDTH       = 32,
    parameter integer PIPELINE_STAGES  = 1,
    parameter integer CLIPPING_ENABLE  = 1,
    parameter [DATA_WIDTH-1:0] DEFAULT_CLIP_MAX = {1'b0, {(DATA_WIDTH-1){1'b1}}} // Max positive
)(
    input  wire                         clk,
    input  wire                         rst,

    // Control and configuration
    input  wire                         enable_i,
    input  wire                         bypass_i,
    input  wire                         clip_en_i,
    input  wire [DATA_WIDTH-1:0]        clip_max_i,

    // Streaming data input
    input  wire [DATA_WIDTH-1:0]        data_i,
    input  wire                         valid_i,
    output wire                         ready_o,

    // Streaming data output
    output wire [DATA_WIDTH-1:0]        data_o,
    output wire                         valid_o,
    input  wire                         ready_i,

    // Status / Sparsity flags
    output wire                         zero_flag_o,
    output wire                         active_flag_o,
    output wire                         clipped_flag_o
);

    // -------------------------------------------------------------------------
    // Sign and magnitude evaluation
    // -------------------------------------------------------------------------
    wire sign_bit = data_i[DATA_WIDTH-1];
    wire is_negative = sign_bit;
    wire is_zero_input = (data_i == {DATA_WIDTH{1'b0}});

    // Effective clipping threshold
    wire [DATA_WIDTH-1:0] effective_clip_max = (clip_max_i == {DATA_WIDTH{1'b0}}) ?
                                               DEFAULT_CLIP_MAX : clip_max_i;

    // Check if positive input exceeds clipping threshold
    wire exceeds_clip = (CLIPPING_ENABLE != 0) && clip_en_i &&
                        !is_negative &&
                        ($signed(data_i) > $signed(effective_clip_max));

    // -------------------------------------------------------------------------
    // Activation computation (combinational core)
    // -------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] activated_val;
    reg                  comb_zero_flag;
    reg                  comb_active_flag;
    reg                  comb_clipped_flag;

    always_comb begin
        if (bypass_i) begin
            // Bypass mode: output raw input untouched
            activated_val     = data_i;
            comb_zero_flag    = is_zero_input;
            comb_active_flag  = !is_negative && !is_zero_input;
            comb_clipped_flag = 1'b0;
        end else if (is_negative) begin
            // Negative input clamped to 0
            activated_val     = {DATA_WIDTH{1'b0}};
            comb_zero_flag    = 1'b1;
            comb_active_flag  = 1'b0;
            comb_clipped_flag = 1'b0;
        end else if (exceeds_clip) begin
            // Positive input clamped to clip_max
            activated_val     = effective_clip_max;
            comb_zero_flag    = (effective_clip_max == {DATA_WIDTH{1'b0}});
            comb_active_flag  = (effective_clip_max != {DATA_WIDTH{1'b0}});
            comb_clipped_flag = 1'b1;
        end else begin
            // Positive or zero input retained
            activated_val     = data_i;
            comb_zero_flag    = is_zero_input;
            comb_active_flag  = !is_zero_input;
            comb_clipped_flag = 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Output pipeline generation
    // -------------------------------------------------------------------------
    generate
        if (PIPELINE_STAGES == 0) begin : gen_comb_output
            // Combinational (zero cycle latency)
            assign ready_o        = ready_i && enable_i;
            assign valid_o        = valid_i && enable_i;
            assign data_o         = activated_val;
            assign zero_flag_o    = comb_zero_flag;
            assign active_flag_o  = comb_active_flag;
            assign clipped_flag_o = comb_clipped_flag;

        end else begin : gen_pipelined_output
            // Registered single-cycle pipeline stage
            reg [DATA_WIDTH-1:0] pipe_data_r;
            reg                  pipe_valid_r;
            reg                  pipe_zero_r;
            reg                  pipe_active_r;
            reg                  pipe_clipped_r;

            // Ready when output register is empty or downstream accepts current data
            wire pipe_ready = !pipe_valid_r || ready_i;
            assign ready_o  = pipe_ready && enable_i;

            always_ff @(posedge clk) begin
                if (rst) begin
                    pipe_valid_r   <= 1'b0;
                    pipe_data_r    <= {DATA_WIDTH{1'b0}};
                    pipe_zero_r    <= 1'b0;
                    pipe_active_r  <= 1'b0;
                    pipe_clipped_r <= 1'b0;
                end else if (enable_i) begin
                    if (pipe_ready) begin
                        if (valid_i) begin
                            pipe_valid_r   <= 1'b1;
                            pipe_data_r    <= activated_val;
                            pipe_zero_r    <= comb_zero_flag;
                            pipe_active_r  <= comb_active_flag;
                            pipe_clipped_r <= comb_clipped_flag;
                        end else begin
                            pipe_valid_r   <= 1'b0;
                            pipe_zero_r    <= 1'b0;
                            pipe_active_r  <= 1'b0;
                            pipe_clipped_r <= 1'b0;
                        end
                    end
                end
            end

            assign valid_o        = pipe_valid_r;
            assign data_o         = pipe_data_r;
            assign zero_flag_o    = pipe_zero_r;
            assign active_flag_o  = pipe_active_r;
            assign clipped_flag_o = pipe_clipped_r;

        end
    endgenerate

endmodule

`default_nettype wire
