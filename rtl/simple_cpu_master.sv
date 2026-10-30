// ============================================================================
// File    : simple_cpu_master.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Simple CPU-like AXI4 master model for UART read/write testing
//           NOTE: axi_uart_top gates axi_bvalid_o on (axi_awvalid & axi_wvalid),
//           so AW+W must stay asserted until B response is received.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module simple_cpu_master #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter ID_WIDTH   = 8
)(
    input  wire                    clk,
    input  wire                    rst,

    output reg  [ID_WIDTH-1:0]     m_axi_awid,
    output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output reg  [7:0]              m_axi_awlen,
    output reg  [2:0]              m_axi_awsize,
    output reg  [1:0]              m_axi_awburst,
    output reg                     m_axi_awlock,
    output reg  [3:0]              m_axi_awcache,
    output reg  [2:0]              m_axi_awprot,
    output reg  [3:0]              m_axi_awqos,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,
    output reg  [DATA_WIDTH-1:0]   m_axi_wdata,
    output reg  [STRB_WIDTH-1:0]   m_axi_wstrb,
    output reg                     m_axi_wlast,
    output reg                     m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [ID_WIDTH-1:0]     m_axi_bid,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output reg                     m_axi_bready,
    output reg  [ID_WIDTH-1:0]     m_axi_arid,
    output reg  [ADDR_WIDTH-1:0]   m_axi_araddr,
    output reg  [7:0]              m_axi_arlen,
    output reg  [2:0]              m_axi_arsize,
    output reg  [1:0]              m_axi_arburst,
    output reg                     m_axi_arlock,
    output reg  [3:0]              m_axi_arcache,
    output reg  [2:0]              m_axi_arprot,
    output reg  [3:0]              m_axi_arqos,
    output reg                     m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [ID_WIDTH-1:0]     m_axi_rid,
    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output reg                     m_axi_rready,

    output reg                     cpu_done,
    output reg                     cpu_pass
);

    localparam [ADDR_WIDTH-1:0] UART_BASE  = 32'h4000_0000;
    localparam [ADDR_WIDTH-1:0] UART_THR   = UART_BASE + 32'h00;
    localparam [ADDR_WIDTH-1:0] UART_BAUD  = UART_BASE + 32'h08;
    localparam [ADDR_WIDTH-1:0] UART_LCR   = UART_BASE + 32'h0C;
    localparam [ADDR_WIDTH-1:0] UART_LSR   = UART_BASE + 32'h14;

    typedef enum logic [4:0] {
        ST_IDLE,

        // Write transactions: drive AW+W simultaneously, hold until B response
        ST_W1_DRIVE_WAIT_B,
        ST_W2_DRIVE_WAIT_B,
        ST_W3_DRIVE_WAIT_B,
        ST_W4_DRIVE_WAIT_B,
        ST_W5_DRIVE_WAIT_B,

        // Read transactions: drive AR, then wait for R
        ST_R1_AR,      ST_R1_R,
        ST_R2_AR,      ST_R2_R,

        ST_DONE
    } state_t;

    state_t state, next_state;

    reg [31:0] rx_data_lsr, rx_data_rbr;

    // --------------------------------------------------------------
    // Sequential: state + captured read data
    // --------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state       <= ST_IDLE;
            rx_data_lsr <= 0;
            rx_data_rbr <= 0;
        end else begin
            state <= next_state;

            if (m_axi_rvalid && m_axi_rready) begin
                case (m_axi_rid)
                    8'h10: rx_data_lsr <= m_axi_rdata[31:0];
                    8'h20: rx_data_rbr <= m_axi_rdata[31:0];
                endcase
            end
        end
    end

    // --------------------------------------------------------------
    // Combinational: next state + AXI signal drive
    // --------------------------------------------------------------
    always_comb begin
        // ---- Defaults ----
        next_state = state;

        m_axi_awvalid = 1'b0; m_axi_awid = '0; m_axi_awaddr = '0;
        m_axi_awlen = 8'h00; m_axi_awsize = 3'b011; m_axi_awburst = 2'b01;
        m_axi_awlock = 1'b0; m_axi_awcache = 4'b0; m_axi_awprot = 3'b0;
        m_axi_awqos = 4'b0;

        m_axi_wvalid = 1'b0; m_axi_wdata = '0; m_axi_wstrb = '0;
        m_axi_wlast = 1'b0;

        m_axi_bready = 1'b1;

        m_axi_arvalid = 1'b0; m_axi_arid = '0; m_axi_araddr = '0;
        m_axi_arlen = 8'h00; m_axi_arsize = 3'b011; m_axi_arburst = 2'b01;
        m_axi_arlock = 1'b0; m_axi_arcache = 4'b0; m_axi_arprot = 3'b0;
        m_axi_arqos = 4'b0;

        m_axi_rready = 1'b1;

        cpu_done = 1'b0;
        cpu_pass = 1'b0;

        // ---- State machine ----
        case (state)
            ST_IDLE: begin
                next_state = ST_W1_DRIVE_WAIT_B;
            end

            // ---------- Write 1: LCR = 0x83 (DLAB|8N1) ----------
            ST_W1_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_LCR; m_axi_awid = 8'h01;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000083};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) begin
                    next_state = ST_W2_DRIVE_WAIT_B;
                end
            end

            // ---------- Write 2: BAUD = 50 ----------
            ST_W2_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_BAUD; m_axi_awid = 8'h02;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'd50};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) begin
                    next_state = ST_W3_DRIVE_WAIT_B;
                end
            end

            // ---------- Write 3: LCR = 0x03 (8N1) ----------
            ST_W3_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_LCR; m_axi_awid = 8'h03;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000003};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) begin
                    next_state = ST_W4_DRIVE_WAIT_B;
                end
            end

            // ---------- Write 4: THR = 'A' (0x41) ----------
            ST_W4_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_THR; m_axi_awid = 8'h04;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000041};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) begin
                    next_state = ST_R1_AR;
                end
            end

            // ---------- Read 1: LSR ----------
            ST_R1_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = UART_LSR; m_axi_arid = 8'h10;
                if (m_axi_arready) begin
                    next_state = ST_R1_R;
                end
            end
            ST_R1_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) begin
                    next_state = ST_R2_AR;
                end
            end

            // ---------- Read 2: RBR ----------
            ST_R2_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = UART_THR; m_axi_arid = 8'h20;
                if (m_axi_arready) begin
                    next_state = ST_R2_R;
                end
            end
            ST_R2_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) begin
                    next_state = ST_W5_DRIVE_WAIT_B;
                end
            end

            // ---------- Write 5: THR = 'B' (0x42) ----------
            ST_W5_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_THR; m_axi_awid = 8'h05;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000042};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) begin
                    next_state = ST_DONE;
                end
            end

            ST_DONE: begin
                cpu_done = 1'b1;
                cpu_pass = (rx_data_rbr[7:0] == 8'h41);
            end

            default: next_state = ST_IDLE;
        endcase
    end

endmodule

`default_nettype wire