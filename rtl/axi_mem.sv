// ============================================================================
// File    : axi_mem.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Simple AXI4 slave memory model with optional hex file initialization
//           Single-beat only, supports 64-bit data, 32-bit address
//           Memory window: BASE_ADDR to BASE_ADDR + MEM_SIZE
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module axi_mem #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter ID_WIDTH   = 8,
    parameter BASE_ADDR  = 32'h8000_0000,  // memory base address
    parameter MEM_SIZE   = 32'h0000_0800,  // 2 KB memory window
    parameter MEM_HEX    = ""              // hex file to load (empty = no load)
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

    localparam MEM_DEPTH = MEM_SIZE / (DATA_WIDTH/8);   // number of 64-bit entries
    localparam ADDR_LSB  = $clog2(DATA_WIDTH/8);        // 3 for 64-bit
    localparam MEM_IDX_BITS = $clog2(MEM_DEPTH);

    reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];

    // ------------------------------------------------------------------
    // Hex file loading
    // ------------------------------------------------------------------
    initial begin
        awready = 1'b0; wready = 1'b0; bvalid = 1'b0;
        bid = '0; bresp = 2'b00;
        arready = 1'b0; rvalid = 1'b0; rlast = 1'b0;
        rid = '0; rdata = '0; rresp = 2'b00;

        if (MEM_HEX != "") begin
            $display("axi_mem: loading hex file: %s at base 0x%0h", MEM_HEX, BASE_ADDR);
            $readmemh(MEM_HEX, mem);
        end
    end

    // address index within memory window
    wire [MEM_IDX_BITS-1:0] aw_idx = awaddr[ADDR_LSB + MEM_IDX_BITS - 1: ADDR_LSB];
    wire [MEM_IDX_BITS-1:0] ar_idx = araddr[ADDR_LSB + MEM_IDX_BITS - 1: ADDR_LSB];

    // ------------------------------------------------------------------
    // Write address channel
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            awready <= 1'b1;
        end else begin
            if (awvalid && awready)
                awready <= 1'b0;
            else if (~awvalid)
                awready <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Write data channel
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wready <= 1'b1;
        end else begin
            if (wvalid && wready)
                wready <= 1'b0;
            else if (~wvalid)
                wready <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Write address latch + memory write
    //
    // A master may present the address and the data in the same cycle, so
    // the write has to be accepted when the data arrives and an address
    // was seen - either in an earlier cycle or in the same one.
    // ------------------------------------------------------------------
    reg aw_pend;
    reg [ID_WIDTH-1:0] aw_id_r;

    wire aw_accept = awvalid && awready;
    wire wr_accept = wvalid && wready && wlast;
    wire wr_fire   = wr_accept && (aw_pend || aw_accept);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            aw_pend <= 1'b0;
        end else begin
            if (wr_accept) begin
                // the data has been taken, no address is pending any more
                aw_pend <= 1'b0;
            end else if (aw_accept && ~aw_pend) begin
                aw_pend <= 1'b1;
                aw_id_r <= awid;
            end
        end
    end

    always @(posedge clk) begin
        if (wr_fire) begin
            for (int i = 0; i < DATA_WIDTH/8; i = i + 1) begin
                if (wstrb[i])
                    mem[aw_idx][i*8 +: 8] <= wdata[i*8 +: 8];
            end
        end
    end

    // ------------------------------------------------------------------
    // Write response
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bvalid <= 1'b0;
            bid <= '0;
            bresp <= 2'b00;
        end else begin
            if (wr_fire && ~bvalid) begin
                bid   <= aw_accept ? awid : aw_id_r;
                bresp <= 2'b00;
                bvalid <= 1'b1;
            end else if (bvalid && bready) begin
                bvalid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // Read address channel
    // ------------------------------------------------------------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            arready <= 1'b1;
        end else begin
            if (arvalid && arready)
                arready <= 1'b0;
            else if (~arvalid)
                arready <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Read data (single beat)
    // ------------------------------------------------------------------
    reg ar_pend;
    reg [ID_WIDTH-1:0] ar_id_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ar_pend <= 1'b0;
        end else begin
            if (arvalid && arready && ~ar_pend) begin
                ar_pend <= 1'b1;
                ar_id_r <= arid;
            end else if (rvalid && rready) begin
                ar_pend <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (arvalid && arready && ~ar_pend) begin
            rdata <= mem[ar_idx];
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rvalid <= 1'b0;
            rlast  <= 1'b0;
            rid <= '0;
            rresp <= 2'b00;
        end else begin
            if (arvalid && arready && ~ar_pend && ~rvalid) begin
                rid   <= ar_id_r;
                rresp <= 2'b00;
                rlast <= 1'b1;
                rvalid <= 1'b1;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
                rlast  <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire