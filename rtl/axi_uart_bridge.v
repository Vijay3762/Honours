// =============================================================================
// File    : axi_uart_bridge.v
// Project : RISC-V SoC Honours
// Purpose : Protocol bridge connecting the AXI4 64-bit/32-bit interconnect
//           master port m04 to the AXI4-Lite 32-bit/5-bit UART IP core.
//
// Interface differences handled here:
//   DATA  : 64-bit  (interconnect) -> 32-bit  (UART)  lower 32 bits used
//   ADDR  : 32-bit  (interconnect) -> 5-bit   (UART)  register offset
//   ID    : 8-bit   (interconnect) -> 12-bit  (UART)  zero-extended in
//   STRB  : 8-byte  (interconnect) -> 4-byte  (UART)  byte lanes selected
//   wlast : present (interconnect) -> absent  (UART)  dropped
//   rlast : absent  (UART)         -> driven 1'b1     always single-beat
//   awlen/arlen: present -> UART is single-beat only; interconnect must
//                issue len=0 transfers (guaranteed for peripheral access)
//
// -----------------------------------------------------------------------------
// 64-bit AXI master support
// -----------------------------------------------------------------------------
// A 64-bit AXI master (the VeeR EL2 LSU for example) always issues
// 8-byte aligned addresses and carries the sub-word select in WSTRB
// (see el2_lsu_bus_buffer.sv: lsu_axi_awaddr = {addr[31:3],3'b0} and
// lsu_axi_wstrb = obuf_byteen).  Two consequences:
//
//  1) WRITE - the byte lane of the write tells us which half of the 64-bit
//     beat carries the data, and therefore which 32-bit register is being
//     written.  The bridge selects the half from WSTRB and rebuilds the
//     register offset from the (aligned) address + the lane half.
//
//  2) READ - there are no strobes on a read, so the half the master will
//     consume is NOT recoverable from the aligned address.  A slave must
//     therefore return the complete 64-bit beat.  The classic 16550
//     register map cannot express that (a 64-bit beat would have to
//     contain two registers, and the RBR read has a side effect), so a
//     dedicated CPU-friendly window is provided (MAP_MODE = 1):
//
//       SoC offset (CPU64)   UART IP offset (16550)
//         0x00  THR / RBR  ->  0x00
//         0x08  IER         ->  0x04
//         0x10  BAUD_DIV    ->  0x08
//         0x18  LCR         ->  0x0C
//         0x20  LSR         ->  0x14
//
//     Every CPU visible register is 8-byte aligned, so a 64-bit master
//     always looks at the LOW 32 bits of the beat, which is where the
//     bridge places the data.
//
// MAP_MODE = 0 (default, LEGACY) keeps the original behaviour (offset ==
// IP offset, data in the low half) so that the existing 32-bit style
// masters and testbenches keep working unchanged.
//
// -----------------------------------------------------------------------------
// AXI write channel decoupling
// -----------------------------------------------------------------------------
// axi_uart_top only starts a write when (axi_awvalid_i & axi_wvalid_i) are
// both high.  A standard AXI master is allowed to present the address and
// the data in different cycles, so the bridge captures AW and W separately
// and presents them to the UART IP together.
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
    parameter UART_STRB_W    = UART_DATA_W / 8,      // 4

    // Register map: 0 = LEGACY (16550 spacing), 1 = CPU64 (8-byte strides)
    parameter MAP_MODE       = 0,

    // Width of the register offset inside the bridge.  The CPU64 window
    // needs 6 bits (0x00 ... 0x20), the UART IP itself only 5.
    parameter OFF_W          = 6
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

    // UART TX / RX physical lines
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

    // Read address
    wire [UART_ID_W-1:0]        uart_arid;
    wire [UART_ADDR_W-1:0]      uart_araddr;
    reg                         uart_arvalid;    // driven by read adapter FSM
    wire                        uart_arready;

    // Read data
    wire [UART_ID_W-1:0]        uart_rid;
    wire [UART_DATA_W-1:0]      uart_rdata;
    wire [1:0]                  uart_rresp;
    wire                        uart_rvalid;

    // =========================================================================
    // Register map
    // =========================================================================
    // CPU64 window (8 byte strides) -> UART IP offsets (16550 spacing)
    function [UART_ADDR_W-1:0] map_cpu64(input [OFF_W-1:0] off);
        begin
            case (off)
                6'h00: map_cpu64 = 5'h00;   // THR / RBR
                6'h08: map_cpu64 = 5'h04;   // IER
                6'h10: map_cpu64 = 5'h08;   // BAUD divisor
                6'h18: map_cpu64 = 5'h0C;   // LCR
                6'h20: map_cpu64 = 5'h14;   // LSR
                default: map_cpu64 = off[UART_ADDR_W-1:0];
            endcase
        end
    endfunction

    // =========================================================================
    // WRITE CHANNEL
    // =========================================================================
    // AW and W are captured independently and presented to the UART IP
    // together (the IP only starts a write when both are valid).
    reg                        aw_buf_valid;
    reg [IC_ID_WIDTH-1:0]      aw_buf_id;
    reg [OFF_W-1:0]            aw_buf_addr;

    reg                        w_buf_valid;
    reg [IC_DATA_WIDTH-1:0]    w_buf_data;
    reg [IC_STRB_WIDTH-1:0]    w_buf_strb;
    reg                        w_buf_last;

    reg                        b_buf_valid;
    reg [IC_ID_WIDTH-1:0]      b_buf_id;
    reg [1:0]                  b_buf_resp;

    // --- sub-word decode -----------------------------------------------------
    // The active byte lanes tell us which half of the 64-bit beat holds the
    // data.  A master that uses exact byte addresses (LEGACY) always puts the
    // data in the low half, a 64-bit master that aligns the address puts it in
    // the lane half.
    wire                       w_hi_lane = (w_buf_strb[3:0] == 4'h0) &&
                                             (w_buf_strb[7:4] != 4'h0);

    // Register offset inside the 8-byte beat:
    //   aligned address  -> bit 2 comes from the lane half
    //   unaligned address-> the address already carries the offset
    wire [OFF_W-1:0]           aw_off_raw = aw_buf_addr;
    wire [OFF_W-1:0]           aw_off_sel = {aw_buf_addr[OFF_W-1:3], w_hi_lane,
                                               aw_buf_addr[1:0]};
    wire [OFF_W-1:0]           aw_off     = (aw_buf_addr[2:0] == 3'b000) ?
                                               aw_off_sel : aw_off_raw;
    wire [UART_ADDR_W-1:0]     aw_off_map = (MAP_MODE == 1) ?
                                               map_cpu64(aw_off) :
                                               aw_off[UART_ADDR_W-1:0];

    // --- drive the UART IP ---------------------------------------------------
    wire                       wr_launch = aw_buf_valid & w_buf_valid;

    assign uart_awaddr  = aw_off_map;
    assign uart_awid    = {{(UART_ID_W-IC_ID_WIDTH){1'b0}}, aw_buf_id};
    assign uart_awvalid = wr_launch;

    assign uart_wdata   = w_hi_lane ? w_buf_data[63:32] : w_buf_data[31:0];
    assign uart_wstrb   = w_hi_lane ? w_buf_strb[7:4]    : w_buf_strb[3:0];
    assign uart_wvalid  = wr_launch;

    // --- handshake with the interconnect --------------------------------------
    // Only one write may be in flight: the AW and W buffers plus the B
    // response register.
    assign s_axi_awready = (!aw_buf_valid) && (!b_buf_valid);
    assign s_axi_wready  = (!w_buf_valid)  && (!b_buf_valid);

    assign s_axi_bvalid  = b_buf_valid;
    assign s_axi_bid     = b_buf_id;
    assign s_axi_bresp   = b_buf_resp;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            aw_buf_valid <= 1'b0;
            aw_buf_id    <= '0;
            aw_buf_addr  <= '0;
            w_buf_valid  <= 1'b0;
            w_buf_data   <= '0;
            w_buf_strb   <= '0;
            w_buf_last   <= 1'b0;
            b_buf_valid  <= 1'b0;
            b_buf_id     <= '0;
            b_buf_resp   <= 2'b00;
        end else begin
            // ---- capture AW ----
            if (s_axi_awvalid && s_axi_awready) begin
                aw_buf_valid <= 1'b1;
                aw_buf_id    <= s_axi_awid;
                aw_buf_addr  <= s_axi_awaddr[OFF_W-1:0];
            end

            // ---- capture W ----
            if (s_axi_wvalid && s_axi_wready) begin
                w_buf_valid <= 1'b1;
                w_buf_data  <= s_axi_wdata;
                w_buf_strb  <= s_axi_wstrb;
                w_buf_last  <= s_axi_wlast;
            end

            // ---- UART IP answered -> publish the B response ----
            if (uart_bvalid && wr_launch) begin
                aw_buf_valid <= 1'b0;
                w_buf_valid  <= 1'b0;
                b_buf_valid  <= 1'b1;
                b_buf_id     <= uart_bid[IC_ID_WIDTH-1:0];
                b_buf_resp   <= uart_bresp;
            end

            // ---- interconnect took the B response ----
            if (b_buf_valid && s_axi_bready)
                b_buf_valid <= 1'b0;
        end
    end

    // =========================================================================
    // READ CHANNEL
    // =========================================================================
    // The UART IP gates rvalid_o with arvalid_i, so arvalid has to stay high
    // until rvalid fires.  A standard AXI interconnect deasserts arvalid one
    // cycle after arready, so the request is latched here and held.
    //
    //   AR_IDLE    : waiting for an incoming arvalid from the interconnect
    //   AR_PENDING : arvalid latched; driving uart_arvalid=1, waiting for the
    //                UART IP to answer
    //
    // The IP only implements reads for the RBR (offset 0x00) and the LSR
    // (offset 0x14).  Any other offset is answered locally with zeros so the
    // bus can never hang.
    localparam AR_IDLE    = 2'd0;
    localparam AR_PENDING = 2'd1;

    reg [1:0]              ar_state;
    reg [UART_ADDR_W-1:0]  ar_addr_lat;
    reg [UART_ID_W-1:0]    ar_id_lat;
    reg                    s_arready_r;

    reg                    r_buf_valid;
    reg [IC_DATA_WIDTH-1:0] r_buf_data;
    reg [IC_ID_WIDTH-1:0]  r_buf_id;

    // --- sub-word decode (read side) ----------------------------------------
    // Without strobes the lane half is unknown, so an aligned address always
    // resolves to the low half of the beat (bit 2 = 0).
    wire [OFF_W-1:0]        ar_off_raw = s_axi_araddr[OFF_W-1:0];
    wire [OFF_W-1:0]        ar_off_sel = {ar_off_raw[OFF_W-1:3], 1'b0,
                                            ar_off_raw[1:0]};
    wire [OFF_W-1:0]        ar_off     = (ar_off_raw[2:0] == 3'b000) ?
                                              ar_off_sel : ar_off_raw;
    wire [UART_ADDR_W-1:0]  ar_off_map = (MAP_MODE == 1) ?
                                              map_cpu64(ar_off) :
                                              ar_off[UART_ADDR_W-1:0];
    wire                    ar_supported = (ar_off_map == 5'h00) ||
                                           (ar_off_map == 5'h14);

    assign s_axi_arready = s_arready_r;

    assign s_axi_rvalid  = r_buf_valid;
    assign s_axi_rdata   = r_buf_data;
    assign s_axi_rid     = r_buf_id;
    assign s_axi_rresp   = 2'b00;
    assign s_axi_rlast   = r_buf_valid;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ar_state      <= AR_IDLE;
            uart_arvalid  <= 1'b0;
            ar_addr_lat   <= '0;
            ar_id_lat     <= '0;
            s_arready_r   <= 1'b0;
            r_buf_valid   <= 1'b0;
            r_buf_data    <= '0;
            r_buf_id      <= '0;
        end else begin
            s_arready_r <= 1'b0;          // default: deassert each cycle

            // ---- retire the R response ----
            if (r_buf_valid && s_axi_rready)
                r_buf_valid <= 1'b0;

            case (ar_state)
                AR_IDLE: begin
                    if (s_axi_arvalid && !r_buf_valid) begin
                        // Latch address+id, ack the interconnect
                        ar_addr_lat  <= ar_off_map;
                        ar_id_lat    <= {{(UART_ID_W-IC_ID_WIDTH){1'b0}}, s_axi_arid};
                        s_arready_r  <= 1'b1;   // one-cycle ack to interconnect
                        if (ar_supported) begin
                            uart_arvalid <= 1'b1;   // begin the UART read
                            ar_state     <= AR_PENDING;
                        end else begin
                            // Not implemented by the IP -> answer locally
                            r_buf_valid <= 1'b1;
                            r_buf_data  <= '0;
                            r_buf_id    <= s_axi_arid[IC_ID_WIDTH-1:0];
                            ar_state    <= AR_IDLE;
                        end
                    end
                end

                AR_PENDING: begin
                    // Keep uart_arvalid high until the UART fires rvalid
                    if (uart_rvalid) begin
                        uart_arvalid <= 1'b0;
                        r_buf_valid  <= 1'b1;
                        r_buf_data   <= {{(IC_DATA_WIDTH-UART_DATA_W){1'b0}}, uart_rdata};
                        r_buf_id     <= ar_id_lat[IC_ID_WIDTH-1:0];
                        ar_state     <= AR_IDLE;
                    end
                end

                default: ar_state <= AR_IDLE;
            endcase
        end
    end

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
