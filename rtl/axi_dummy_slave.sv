// ============================================================================
// File    : axi_dummy_slave.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Simple dummy AXI4 slave that returns OKAY for all transactions
//           Used for unused interconnect master ports (m02,m03,m05,m06,m07)
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_dummy_slave #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 8
)(
    input  wire                    clk,
    input  wire                    rst,          // active-high

    // Write address
    input  wire [ID_WIDTH-1:0]     awid,
    input  wire [ADDR_WIDTH-1:0]   awaddr,
    input  wire [7:0]              awlen,
    input  wire [2:0]              awsize,
    input  wire [1:0]              awburst,
    input  wire                    awvalid,
    output reg                     awready,
    // Write data
    input  wire [DATA_WIDTH-1:0]   wdata,
    input  wire [DATA_WIDTH/8-1:0] wstrb,
    input  wire                    wlast,
    input  wire                    wvalid,
    output reg                     wready,
    // Write response
    output reg  [ID_WIDTH-1:0]     bid,
    output reg  [1:0]              bresp,
    output reg                     bvalid,
    input  wire                    bready,
    // Read address
    input  wire [ID_WIDTH-1:0]     arid,
    input  wire [ADDR_WIDTH-1:0]   araddr,
    input  wire [7:0]              arlen,
    input  wire [2:0]              arsize,
    input  wire [1:0]              arburst,
    input  wire                    arvalid,
    output reg                     arready,
    // Read data
    output reg  [ID_WIDTH-1:0]     rid,
    output reg  [DATA_WIDTH-1:0]   rdata,
    output reg  [1:0]              rresp,
    output reg                     rlast,
    output reg                     rvalid,
    input  wire                    rready
);

    reg aw_pend;
    reg [ID_WIDTH-1:0] aw_id_r;
    reg ar_pend;
    reg [ID_WIDTH-1:0] ar_id_r;
    reg [DATA_WIDTH-1:0] ar_data_r;

    initial begin
        awready = 1'b1; wready = 1'b1; bvalid = 1'b0;
        bid = '0; bresp = 2'b00;
        arready = 1'b1; rvalid = 1'b0; rlast = 1'b0;
        rid = '0; rdata = '0; rresp = 2'b00;
        aw_pend = 1'b0; ar_pend = 1'b0;
        aw_id_r = '0; ar_id_r = '0; ar_data_r = '0;
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            awready <= 1'b1;
            wready  <= 1'b1;
            bvalid  <= 1'b0;
            aw_pend <= 1'b0;
        end else begin
            // Write address
            if (awvalid && awready && ~aw_pend) begin
                aw_pend   <= 1'b1;
                aw_id_r   <= awid;
                awready   <= 1'b0;
            end else if (~awvalid) begin
                awready <= 1'b1;
            end

            // Write data
            if (wvalid && wready && aw_pend) begin
                wready <= 1'b0;
            end else if (~wvalid) begin
                wready <= 1'b1;
            end

            // Write response
            if (wvalid && wready && aw_pend && wlast && ~bvalid) begin
                bid   <= aw_id_r;
                bresp <= 2'b00;
                bvalid <= 1'b1;
                aw_pend <= 1'b0;
            end else if (bvalid && bready) begin
                bvalid <= 1'b0;
            end
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            arready <= 1'b1;
            rvalid  <= 1'b0;
            rlast   <= 1'b0;
            ar_pend <= 1'b0;
        end else begin
            // Read address
            if (arvalid && arready && ~ar_pend) begin
                ar_pend   <= 1'b1;
                ar_id_r   <= arid;
                ar_data_r <= {64{1'b0}} | {32'hCAFE_0000, araddr[31:0]};
                arready   <= 1'b0;
            end else if (~arvalid) begin
                arready <= 1'b1;
            end

            // Read data
            if (ar_pend && ~rvalid) begin
                rid   <= ar_id_r;
                rdata <= ar_data_r;
                rresp <= 2'b00;
                rlast <= 1'b1;
                rvalid <= 1'b1;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
                ar_pend <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire