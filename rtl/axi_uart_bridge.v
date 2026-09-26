// =============================================================================
// File    : axi_uart_bridge.v
// Project : RISC-V SoC Honours
// Purpose : Protocol bridge connecting the AXI4 64-bit/32-bit interconnect
//           master port m04 to the AXI4-Lite 32-bit/5-bit UART IP core.
//
// Interface differences handled here:
//   DATA  : 64-bit  (interconnect) -> 32-bit  (UART)  lower 32 bits used
//   ADDR  : 32-bit  (interconnect) -> 5-bit   (UART)  lower 5 bits forwarded
//   ID    : 8-bit   (interconnect) -> 12-bit  (UART)  zero-extended in
//   STRB  : 8-byte  (interconnect) -> 4-byte  (UART)  lower 4 bits used
//   wlast : present (interconnect) -> absent  (UART)  dropped
//   rlast : absent  (UART)         -> driven 1'b1     always single-beat
//   awlen/arlen: present -> UART is single-beat only; interconnect must
//                issue len=0 transfers (guaranteed for peripheral access)
//
// The UART clock domains:
//   fixed_clk_i  : internal UART clock  - tied to the same system clock here
//   axi_aclk_i   : AXI bus clock        - the interconnect clock
// =============================================================================

`default_nettype none

module axi_uart_bridge #(
    // Interconnect-side widths
    parameter IC_DATA_WIDTH  = 64,
    parameter IC_ADDR_WIDTH  = 32,
    parameter IC_ID_WIDTH    = 8,
    parameter IC_STRB_WIDTH  = IC_DATA_WIDTH / 8,   // 8

    // UART-side widths (fixed by the IP)
    parameter UART_DATA_W    = 32,
    parameter UART_ADDR_W    = 5,
    parameter UART_ID_W      = 12,
    parameter UART_STRB_W    = UART_DATA_W / 8      // 4
)(
    // -------------------------------------------------------------------------
    // Global
    // -------------------------------------------------------------------------
    input  wire                      clk,
    input  wire                      rst,           // active-high

    // -------------------------------------------------------------------------
    // AXI4 slave port  (connects to interconnect m04_axi_*)
    // -------------------------------------------------------------------------
    // Write address channel
    input  wire [IC_ID_WIDTH-1:0]    s_axi_awid,
    input  wire [IC_ADDR_WIDTH-1:0]  s_axi_awaddr,
    input  wire [7:0]                s_axi_awlen,
    input  wire [2:0]                s_axi_awsize,
    input  wire [1:0]                s_axi_awburst,
    input  wire                      s_axi_awlock,
    input  wire [3:0]                s_axi_awcache,
    input  wire [2:0]                s_axi_awprot,
    input  wire [3:0]                s_axi_awqos,
    input  wire [3:0]                s_axi_awregion,
    input  wire                      s_axi_awvalid,
    output wire                      s_axi_awready,

    // Write data channel
    input  wire [IC_DATA_WIDTH-1:0]  s_axi_wdata,
    input  wire [IC_STRB_WIDTH-1:0]  s_axi_wstrb,
    input  wire                      s_axi_wlast,
    input  wire                      s_axi_wvalid,
    output wire                      s_axi_wready,

    // Write response channel
    output wire [IC_ID_WIDTH-1:0]    s_axi_bid,
    output wire [1:0]                s_axi_bresp,
    output wire                      s_axi_bvalid,
    input  wire                      s_axi_bready,

    // Read address channel
    input  wire [IC_ID_WIDTH-1:0]    s_axi_arid,
    input  wire [IC_ADDR_WIDTH-1:0]  s_axi_araddr,
    input  wire [7:0]                s_axi_arlen,
    input  wire [2:0]                s_axi_arsize,
    input  wire [1:0]                s_axi_arburst,
    input  wire                      s_axi_arlock,
    input  wire [3:0]                s_axi_arcache,
    input  wire [2:0]                s_axi_arprot,
    input  wire [3:0]                s_axi_arqos,
    input  wire [3:0]                s_axi_arregion,
    input  wire                      s_axi_arvalid,
    output wire                      s_axi_arready,

    // Read data channel
    output wire [IC_ID_WIDTH-1:0]    s_axi_rid,
    output wire [IC_DATA_WIDTH-1:0]  s_axi_rdata,
    output wire [1:0]                s_axi_rresp,
    output wire                      s_axi_rlast,
    output wire                      s_axi_rvalid,
    input  wire                      s_axi_rready,

    // -------------------------------------------------------------------------
    // UART TX / RX physical lines
    // -------------------------------------------------------------------------
    output wire                      uart_tx_o,
    input  wire                      uart_rx_i,
    output wire                      uart_irq_o
);

    // =========================================================================
    // Internal wires to/from axi_uart_top
    // =========================================================================
    wire                        aresetn = ~rst;   // UART uses active-low reset

    // Write address
    wire [UART_ID_W-1:0]        uart_awid;
    wire [UART_ADDR_W-1:0]      uart_awaddr;
    wire                        uart_awvalid;
    wire                        uart_awready;

    // Write data
    wire [UART_DATA_W-1:0]      uart_wdata;
    wire [UART_STRB_W-1:0]      uart_wstrb;
    wire                        uart_wvalid;
    wire                        uart_wready;

    // Write response
    wire [UART_ID_W-1:0]        uart_bid;
    wire [1:0]                  uart_bresp;
    wire                        uart_bvalid;
    // bready goes back

    // Read address
    wire [UART_ID_W-1:0]        uart_arid_unused; // latched as ar_id_lat in FSM
    wire [UART_ADDR_W-1:0]      uart_araddr_unused; // latched as ar_addr_lat in FSM
    reg                         uart_arvalid;    // driven by read adapter FSM
    wire                        uart_arready;

    // Read data
    wire [UART_ID_W-1:0]        uart_rid;
    wire [UART_DATA_W-1:0]      uart_rdata;
    wire [1:0]                  uart_rresp;
    wire                        uart_rvalid;
    // rready goes back

    // =========================================================================
    // Down-size mappings  (interconnect -> UART)
    // =========================================================================

    // IDs: truncate 8-bit IC ID to 8 low bits of 12-bit UART ID (write path)
    assign uart_awid   = {{(UART_ID_W-IC_ID_WIDTH){1'b0}}, s_axi_awid};
    // Read ID is latched into ar_id_lat by the AR FSM

    // Addresses: pass only the 5 LSBs (UART register offset within 0x4000_0000)
    assign uart_awaddr = s_axi_awaddr[UART_ADDR_W-1:0];
    // Read address is latched into ar_addr_lat by the AR FSM

    // Data: use the lower 32 bits of the 64-bit bus
    assign uart_wdata  = s_axi_wdata[UART_DATA_W-1:0];
    assign uart_wstrb  = s_axi_wstrb[UART_STRB_W-1:0];

    // Valid / handshake pass-through (write channels direct)
    assign uart_awvalid = s_axi_awvalid;
    assign uart_wvalid  = s_axi_wvalid;
    // uart_arvalid is driven by the read-adapter FSM below

    // =========================================================================
    // Read-channel adapter FSM
    // =========================================================================
    // The UART IP (axi_uart_top) gates rvalid_o with arvalid_i:
    //   assign axi_rvalid_o = (axi_arvalid_i & ~axi_sync_rden) ? axi_rvalid : 0;
    // This means arvalid must stay HIGH until rvalid is also asserted.
    // A standard AXI4 interconnect deasserts arvalid one cycle after arready,
    // so we latch the request here and hold uart_arvalid until rvalid fires.
    //
    //  States:
    //   AR_IDLE     : waiting for incoming arvalid from interconnect
    //   AR_PENDING  : arvalid latched; driving uart_arvalid=1, waiting for
    //                 uart_arready (=uart_rvalid in same cycle for this IP)
    //   AR_DONE     : rvalid seen; drive arready back to IC; clear next cycle
    // =========================================================================
    localparam AR_IDLE    = 2'd0;
    localparam AR_PENDING = 2'd1;
    localparam AR_DONE    = 2'd2;

    reg [1:0]              ar_state;
    reg [UART_ADDR_W-1:0]  ar_addr_lat;
    reg [UART_ID_W-1:0]    ar_id_lat;

    // Acknowledge to the interconnect: pulse for one cycle when we captured AR
    reg  s_arready_r;
    assign s_axi_arready = s_arready_r;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ar_state      <= AR_IDLE;
            uart_arvalid  <= 1'b0;
            ar_addr_lat   <= '0;
            ar_id_lat     <= '0;
            s_arready_r   <= 1'b0;
        end else begin
            s_arready_r  <= 1'b0;          // default: deassert each cycle

            case (ar_state)
                AR_IDLE: begin
                    if (s_axi_arvalid) begin
                        // Latch address+id, ack interconnect, start UART request
                        ar_addr_lat  <= s_axi_araddr[UART_ADDR_W-1:0];
                        ar_id_lat    <= {{(UART_ID_W-IC_ID_WIDTH){1'b0}}, s_axi_arid};
                        s_arready_r  <= 1'b1;   // one-cycle ack to interconnect
                        uart_arvalid <= 1'b1;   // begin UART read
                        ar_state     <= AR_PENDING;
                    end
                end

                AR_PENDING: begin
                    // Keep uart_arvalid high until UART fires rvalid
                    if (uart_rvalid) begin
                        uart_arvalid <= 1'b0;
                        ar_state     <= AR_IDLE;
                    end
                end

                default: ar_state <= AR_IDLE;
            endcase
        end
    end

    // Feed latched address/id to UART (override combinational pass-through)
    // We need to present the stored address while in AR_PENDING since the
    // interconnect may have already moved on.
    // The uart_araddr/uart_arid are directly driven from the latched regs.

    // =========================================================================
    // Up-size mappings  (UART -> interconnect)
    // =========================================================================

    // Ready signals pass straight back
    assign s_axi_awready = uart_awready;
    assign s_axi_wready  = uart_wready;
    // s_axi_arready is driven by the FSM above

    // IDs: take lower IC_ID_WIDTH bits of 12-bit UART response ID
    assign s_axi_bid  = uart_bid[IC_ID_WIDTH-1:0];
    assign s_axi_rid  = uart_rid[IC_ID_WIDTH-1:0];

    // Response codes pass through
    assign s_axi_bresp = uart_bresp;
    assign s_axi_bvalid = uart_bvalid;

    // Read data: zero-extend 32-bit UART data into 64-bit IC bus
    assign s_axi_rdata  = {{(IC_DATA_WIDTH-UART_DATA_W){1'b0}}, uart_rdata};
    assign s_axi_rresp  = uart_rresp;
    assign s_axi_rvalid = uart_rvalid;

    // UART is always single-beat; assert rlast constantly when valid
    assign s_axi_rlast  = uart_rvalid;

    // =========================================================================
    // axi_uart_top instantiation
    // =========================================================================
    axi_uart_top uart_top (
        // Clocks & reset
        .fixed_clk_i    (clk),
        .axi_aclk_i     (clk),
        .axi_aresetn_i  (aresetn),

        // Write address
        .axi_awid_i     (uart_awid),
        .axi_awaddr_i   (uart_awaddr),
        .axi_awvalid_i  (uart_awvalid),
        .axi_awready_o  (uart_awready),

        // Write data
        .axi_wdata_i    (uart_wdata),
        .axi_wstrb_i    (uart_wstrb),
        .axi_wvalid_i   (uart_wvalid),
        .axi_wready_o   (uart_wready),

        // Write response
        .axi_bid_o      (uart_bid),
        .axi_bresp_o    (uart_bresp),
        .axi_bvalid_o   (uart_bvalid),
        .axi_bready_i   (s_axi_bready),

        // Read address
        .axi_arid_i     (ar_id_lat),
        .axi_araddr_i   (ar_addr_lat),
        .axi_arvalid_i  (uart_arvalid),
        .axi_arready_o  (uart_arready),

        // Read data
        .axi_rid_o      (uart_rid),
        .axi_rdata_o    (uart_rdata),
        .axi_rresp_o    (uart_rresp),
        .axi_rvalid_o   (uart_rvalid),
        .axi_rready_i   (s_axi_rready),

        // UART physical
        .uart_rx_i      (uart_rx_i),
        .uart_tx_o      (uart_tx_o),

        // Interrupt
        .read_interrupt_o (uart_irq_o)
    );

endmodule

`default_nettype wire
