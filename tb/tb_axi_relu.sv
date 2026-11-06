// =============================================================================
// File    : tb_axi_relu.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Standalone self-checking testbench for axi_relu.sv.
//
// Test plan:
//   T1  Reset defaults          : CTRL=1, STATUS=0, DIN=0, DOUT=0, ID="RELU", VER=v1.0
//   T2  Register write/readback : CTRL, CLIP_MAX
//   T3  Positive activation     : DIN=100 -> DOUT=100, status active=1, zero=0
//   T4  Negative activation     : DIN=-100 -> DOUT=0, status zero=1, active=0
//   T5  Boundary values         : 0x7FFFFFFF -> 0x7FFFFFFF, 0x80000000 -> 0, -1 -> 0
//   T6  Bypass mode             : bypass=1, DIN=-50 -> DOUT=-50
//   T7  Clipping saturation     : clip_en=1, clip_max=50, DIN=120 -> DOUT=50, clipped=1
//   T8  Sparsity counters       : ZERO_COUNT, TOTAL_COUNT, reset_stats pulse
//   T9  RO register writes      : STATUS, DOUT, ID, VERSION -> SLVERR
//   T10 Reserved offset         : 0x30 -> SLVERR
//   T11 Out-of-aperture         : 0x1000 -> DECERR
//   T12 Misaligned address      : 0x02 -> SLVERR
//   T13 WSTRB partial write     : byte strobes on CLIP_MAX
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_axi_relu;

    localparam integer DATA_WIDTH     = 32;
    localparam integer AXI_DATA_WIDTH = 64;
    localparam integer AXI_ADDR_WIDTH = 32;
    localparam integer AXI_ID_WIDTH   = 8;
    localparam integer AXI_STRB_WIDTH = 8;

    localparam [31:0] RELU_ID_VALUE      = 32'h52454C55;  // "RELU"
    localparam [31:0] RELU_VERSION_VALUE = 32'h00010000;  // v1.0

    // Register offsets
    localparam [AXI_ADDR_WIDTH-1:0] RELU_BASE        = 32'h0;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_CTRL    = RELU_BASE + 32'h00;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_STATUS  = RELU_BASE + 32'h04;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_DIN     = RELU_BASE + 32'h08;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_DOUT    = RELU_BASE + 32'h0C;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_CLIP_MAX= RELU_BASE + 32'h10;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_ZERO_CNT= RELU_BASE + 32'h14;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_TOT_CNT = RELU_BASE + 32'h18;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_ID      = RELU_BASE + 32'h1C;
    localparam [AXI_ADDR_WIDTH-1:0] REG_ADDR_VER     = RELU_BASE + 32'h20;

    localparam integer CLK_PERIOD = 10;
    localparam integer TIMEOUT_CYCLES = 1000;

    // AXI response codes
    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    // Signals
    reg                              clk;
    reg                              rst;

    reg  [AXI_ID_WIDTH-1:0]          awid;
    reg  [AXI_ADDR_WIDTH-1:0]        awaddr;
    reg  [7:0]                       awlen;
    reg  [2:0]                       awsize;
    reg  [1:0]                       awburst;
    reg                              awlock;
    reg  [3:0]                       awcache;
    reg  [2:0]                       awprot;
    reg  [3:0]                       awqos;
    reg  [3:0]                       awregion;
    reg                              awvalid;
    wire                             awready;

    reg  [AXI_DATA_WIDTH-1:0]        wdata;
    reg  [AXI_STRB_WIDTH-1:0]        wstrb;
    reg                              wlast;
    reg                              wvalid;
    wire                             wready;

    wire [AXI_ID_WIDTH-1:0]          bid;
    wire [1:0]                       bresp;
    wire                             bvalid;
    reg                              bready;

    reg  [AXI_ID_WIDTH-1:0]          arid;
    reg  [AXI_ADDR_WIDTH-1:0]        araddr;
    reg  [7:0]                       arlen;
    reg  [2:0]                       arsize;
    reg  [1:0]                       arburst;
    reg                              arlock;
    reg  [3:0]                       arcache;
    reg  [2:0]                       arprot;
    reg  [3:0]                       arqos;
    reg  [3:0]                       arregion;
    reg                              arvalid;
    wire                             arready;

    wire [AXI_ID_WIDTH-1:0]          rid;
    wire [AXI_DATA_WIDTH-1:0]        rdata;
    wire [1:0]                       rresp;
    wire                             rlast;
    wire                             rvalid;
    reg                              rready;

    // DUT instantiation
    axi_relu #(
        .DATA_WIDTH          (DATA_WIDTH),
        .AXI_DATA_WIDTH      (AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH      (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH        (AXI_ID_WIDTH),
        .AXI_STRB_WIDTH      (AXI_STRB_WIDTH),
        .BASE_ADDR           (RELU_BASE),
        .RELU_ID_VALUE       (RELU_ID_VALUE),
        .RELU_VERSION_VALUE  (RELU_VERSION_VALUE)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (awid),
        .s_axi_awaddr        (awaddr),
        .s_axi_awlen         (awlen),
        .s_axi_awsize        (awsize),
        .s_axi_awburst       (awburst),
        .s_axi_awlock        (awlock),
        .s_axi_awcache       (awcache),
        .s_axi_awprot        (awprot),
        .s_axi_awqos         (awqos),
        .s_axi_awregion      (awregion),
        .s_axi_awvalid       (awvalid),
        .s_axi_awready       (awready),
        .s_axi_wdata         (wdata),
        .s_axi_wstrb         (wstrb),
        .s_axi_wlast         (wlast),
        .s_axi_wvalid        (wvalid),
        .s_axi_wready        (wready),
        .s_axi_bid           (bid),
        .s_axi_bresp         (bresp),
        .s_axi_bvalid        (bvalid),
        .s_axi_bready        (bready),
        .s_axi_arid          (arid),
        .s_axi_araddr        (araddr),
        .s_axi_arlen         (arlen),
        .s_axi_arsize        (arsize),
        .s_axi_arburst       (arburst),
        .s_axi_arlock        (arlock),
        .s_axi_arcache       (arcache),
        .s_axi_arprot        (arprot),
        .s_axi_arqos         (arqos),
        .s_axi_arregion      (arregion),
        .s_axi_arvalid       (arvalid),
        .s_axi_arready       (arready),
        .s_axi_rid           (rid),
        .s_axi_rdata         (rdata),
        .s_axi_rresp         (rresp),
        .s_axi_rlast         (rlast),
        .s_axi_rvalid        (rvalid),
        .s_axi_rready        (rready)
    );

    // Clock generator
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // Test accounting
    integer tests_passed = 0;
    integer tests_failed = 0;
    integer total_checks = 0;

    task automatic check(input string desc, input [31:0] actual, input [31:0] expected);
        total_checks++;
        if (actual === expected) begin
            tests_passed++;
        end else begin
            tests_failed++;
            $display("  [FAIL] %s: expected 0x%08h, got 0x%08h", desc, expected, actual);
        end
    endtask

    task automatic check_resp(input string desc, input [1:0] actual, input [1:0] expected);
        total_checks++;
        if (actual === expected) begin
            tests_passed++;
        end else begin
            tests_failed++;
            $display("  [FAIL] %s: expected resp %b, got resp %b", desc, expected, actual);
        end
    endtask

    // Bounded AXI write task
    task automatic axi_write(
        input  [AXI_ADDR_WIDTH-1:0] addr,
        input  [31:0]               data,
        input  [3:0]                strb,
        output [1:0]                resp
    );
        integer to_aw, to_w, to_b;
        begin
            awid     = 8'hA1;
            awaddr   = addr;
            awlen    = 8'h00;
            awsize   = 3'b010;
            awburst  = 2'b01;
            awlock   = 1'b0;
            awcache  = 4'b0000;
            awprot   = 3'b000;
            awqos    = 4'b0000;
            awregion = 4'b0000;
            awvalid  = 1'b1;

            wdata    = {32'h0, data};
            wstrb    = {4'h0, strb};
            wlast    = 1'b1;
            wvalid   = 1'b1;
            bready   = 1'b1;

            to_aw = 0;
            while (!awready && to_aw < TIMEOUT_CYCLES) begin
                @(posedge clk);
                to_aw++;
            end
            @(posedge clk);
            awvalid = 1'b0;

            to_w = 0;
            while (!wready && to_w < TIMEOUT_CYCLES) begin
                @(posedge clk);
                to_w++;
            end
            wvalid = 1'b0;

            to_b = 0;
            while (!bvalid && to_b < TIMEOUT_CYCLES) begin
                @(posedge clk);
                to_b++;
            end
            resp = bresp;
            @(posedge clk);
            bready = 1'b0;
        end
    endtask

    // Bounded AXI read task
    task automatic axi_read(
        input  [AXI_ADDR_WIDTH-1:0] addr,
        output [31:0]               data,
        output [1:0]                resp
    );
        integer to_ar, to_r;
        begin
            arid     = 8'hB2;
            araddr   = addr;
            arlen    = 8'h00;
            arsize   = 3'b010;
            arburst  = 2'b01;
            arlock   = 1'b0;
            arcache  = 4'b0000;
            arprot   = 3'b000;
            arqos    = 4'b0000;
            arregion = 4'b0000;
            arvalid  = 1'b1;
            rready   = 1'b1;

            to_ar = 0;
            while (!arready && to_ar < TIMEOUT_CYCLES) begin
                @(posedge clk);
                to_ar++;
            end
            @(posedge clk);
            arvalid = 1'b0;

            to_r = 0;
            while (!rvalid && to_r < TIMEOUT_CYCLES) begin
                @(posedge clk);
                to_r++;
            end
            data = rdata[31:0];
            resp = rresp;
            @(posedge clk);
            rready = 1'b0;
        end
    endtask

    // =========================================================================
    // Test execution
    // =========================================================================
    reg [31:0] rd_val;
    reg [1:0]  wr_resp, rd_resp;

    initial begin
        $display("============================================================");
        $display("  Starting AXI ReLU Standalone Verification (tb_axi_relu)   ");
        $display("============================================================");

        // Reset
        rst     = 1'b1;
        awvalid = 1'b0;
        wvalid  = 1'b0;
        bready  = 1'b0;
        arvalid = 1'b0;
        rready  = 1'b0;

        #(CLK_PERIOD * 3);
        @(negedge clk);
        rst = 1'b0;
        #(CLK_PERIOD * 2);

        // T1: Reset defaults
        $display("\n--- T1: Reset defaults ---");
        axi_read(REG_ADDR_CTRL, rd_val, rd_resp);
        check_resp("T1: CTRL read resp", rd_resp, RESP_OKAY);
        check("T1: CTRL default = 1 (enabled)", rd_val, 32'h0000_0001);

        axi_read(REG_ADDR_STATUS, rd_val, rd_resp);
        check("T1: STATUS default = 0", rd_val, 32'd0);

        axi_read(REG_ADDR_ID, rd_val, rd_resp);
        check("T1: ID == 'RELU'", rd_val, RELU_ID_VALUE);

        axi_read(REG_ADDR_VER, rd_val, rd_resp);
        check("T1: VERSION == 0x00010000", rd_val, RELU_VERSION_VALUE);

        axi_read(REG_ADDR_DIN, rd_val, rd_resp);
        check("T1: DIN default = 0", rd_val, 32'd0);

        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T1: DOUT default = 0", rd_val, 32'd0);

        // T2: Register write/readback
        $display("\n--- T2: Register Write / Readback ---");
        axi_write(REG_ADDR_CLIP_MAX, 32'h0000_1234, 4'hF, wr_resp);
        check_resp("T2: CLIP_MAX write resp", wr_resp, RESP_OKAY);
        axi_read(REG_ADDR_CLIP_MAX, rd_val, rd_resp);
        check("T2: CLIP_MAX readback", rd_val, 32'h0000_1234);

        // T3: Positive activation
        $display("\n--- T3: Positive Activation ---");
        axi_write(REG_ADDR_DIN, 32'd100, 4'hF, wr_resp);
        check_resp("T3: DIN write resp", wr_resp, RESP_OKAY);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T3: DOUT for DIN=100 (expect 100)", rd_val, 32'd100);
        axi_read(REG_ADDR_STATUS, rd_val, rd_resp);
        check("T3: STATUS active=1, zero=0 (0x04)", rd_val[3:1], 3'b010); // bit 2: active

        // T4: Negative activation (clamped to 0)
        $display("\n--- T4: Negative Activation Clamping ---");
        axi_write(REG_ADDR_DIN, -32'd100, 4'hF, wr_resp);
        check_resp("T4: DIN write resp", wr_resp, RESP_OKAY);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T4: DOUT for DIN=-100 (expect 0)", rd_val, 32'd0);
        axi_read(REG_ADDR_STATUS, rd_val, rd_resp);
        check("T4: STATUS active=0, zero=1 (0x02)", rd_val[3:1], 3'b001); // bit 1: zero

        // T5: Boundary values
        $display("\n--- T5: Boundary Values ---");
        // Max positive
        axi_write(REG_ADDR_DIN, 32'h7FFF_FFFF, 4'hF, wr_resp);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T5: Max positive preserved", rd_val, 32'h7FFF_FFFF);

        // Min negative (0x80000000)
        axi_write(REG_ADDR_DIN, 32'h8000_0000, 4'hF, wr_resp);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T5: Min negative clamped to 0", rd_val, 32'd0);

        // Negative one
        axi_write(REG_ADDR_DIN, 32'hFFFF_FFFF, 4'hF, wr_resp);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T5: Negative one (-1) clamped to 0", rd_val, 32'd0);

        // Zero
        axi_write(REG_ADDR_DIN, 32'd0, 4'hF, wr_resp);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T5: Zero input -> 0", rd_val, 32'd0);

        // T6: Bypass mode
        $display("\n--- T6: Bypass Mode ---");
        axi_write(REG_ADDR_CTRL, 32'h0000_0003, 4'hF, wr_resp); // enable=1, bypass=1
        axi_write(REG_ADDR_DIN, -32'd50, 4'hF, wr_resp);
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T6: Bypass negative (-50 passed)", rd_val, -32'd50);

        // Turn off bypass
        axi_write(REG_ADDR_CTRL, 32'h0000_0001, 4'hF, wr_resp);

        // T7: Saturation / Clipping Mode
        $display("\n--- T7: Saturation / Clipping Mode ---");
        axi_write(REG_ADDR_CLIP_MAX, 32'sd50, 4'hF, wr_resp);
        axi_write(REG_ADDR_CTRL, 32'h0000_0005, 4'hF, wr_resp); // enable=1, clip_en=1
        axi_write(REG_ADDR_DIN, 32'sd120, 4'hF, wr_resp);       // 120 > 50
        axi_read(REG_ADDR_DOUT, rd_val, rd_resp);
        check("T7: Saturation 120 capped at 50", rd_val, 32'sd50);
        axi_read(REG_ADDR_STATUS, rd_val, rd_resp);
        check("T7: STATUS clipped flag asserted", rd_val[3], 1'b1);

        // Disable clip
        axi_write(REG_ADDR_CTRL, 32'h0000_0001, 4'hF, wr_resp);

        // T8: Sparsity & Performance Counters
        $display("\n--- T8: Performance / Sparsity Counters ---");
        axi_read(REG_ADDR_TOT_CNT, rd_val, rd_resp);
        $display("  Total inputs processed so far: %0d", rd_val);
        if (rd_val > 0) tests_passed++; else tests_failed++;

        axi_read(REG_ADDR_ZERO_CNT, rd_val, rd_resp);
        $display("  Zero / Clamped inputs so far: %0d", rd_val);
        if (rd_val > 0) tests_passed++; else tests_failed++;

        // Reset stats pulse (bit 3)
        axi_write(REG_ADDR_CTRL, 32'h0000_0009, 4'hF, wr_resp); // enable=1, reset_stats=1
        axi_read(REG_ADDR_TOT_CNT, rd_val, rd_resp);
        check("T8: TOTAL_COUNT cleared", rd_val, 32'd0);
        axi_read(REG_ADDR_ZERO_CNT, rd_val, rd_resp);
        check("T8: ZERO_COUNT cleared", rd_val, 32'd0);

        // T9: RO Register Write Rejection (SLVERR)
        $display("\n--- T9: Read-Only Register Write Rejection ---");
        axi_write(REG_ADDR_STATUS, 32'hDEADBEEF, 4'hF, wr_resp);
        check_resp("T9: STATUS write -> SLVERR", wr_resp, RESP_SLVERR);

        axi_write(REG_ADDR_DOUT, 32'hDEADBEEF, 4'hF, wr_resp);
        check_resp("T9: DOUT write -> SLVERR", wr_resp, RESP_SLVERR);

        axi_write(REG_ADDR_ID, 32'hDEADBEEF, 4'hF, wr_resp);
        check_resp("T9: ID write -> SLVERR", wr_resp, RESP_SLVERR);

        axi_write(REG_ADDR_VER, 32'hDEADBEEF, 4'hF, wr_resp);
        check_resp("T9: VERSION write -> SLVERR", wr_resp, RESP_SLVERR);

        // T10: Reserved offset error handling
        $display("\n--- T10: Reserved Offset Access ---");
        axi_read(RELU_BASE + 32'h30, rd_val, rd_resp);
        check_resp("T10: Reserved offset read -> SLVERR", rd_resp, RESP_SLVERR);

        axi_write(RELU_BASE + 32'h30, 32'h0, 4'hF, wr_resp);
        check_resp("T10: Reserved offset write -> SLVERR", wr_resp, RESP_SLVERR);

        // T11: Out-of-aperture address (DECERR)
        $display("\n--- T11: Out-of-Aperture Address ---");
        axi_read(RELU_BASE + 32'h1000, rd_val, rd_resp);
        check_resp("T11: Aperture boundary read -> DECERR", rd_resp, RESP_DECERR);

        // T12: Misaligned address (SLVERR)
        $display("\n--- T12: Misaligned Address ---");
        axi_read(REG_ADDR_CTRL + 32'h02, rd_val, rd_resp);
        check_resp("T12: Misaligned read -> SLVERR", rd_resp, RESP_SLVERR);

        // T13: WSTRB partial write
        $display("\n--- T13: WSTRB Partial Write ---");
        axi_write(REG_ADDR_CLIP_MAX, 32'hAABBCCDD, 4'hF, wr_resp);
        // Overwrite only lower byte with 0x55
        axi_write(REG_ADDR_CLIP_MAX, 32'h00000055, 4'b0001, wr_resp);
        axi_read(REG_ADDR_CLIP_MAX, rd_val, rd_resp);
        check("T13: Partial strobe byte 0 modified", rd_val, 32'hAABBCC55);

        // =====================================================================
        // Final Summary
        // =====================================================================
        $display("\n============================================================");
        $display("  AXI ReLU Verification Summary");
        $display("  Total Checks : %0d", total_checks);
        $display("  PASSED       : %0d", tests_passed);
        $display("  FAILED       : %0d", tests_failed);
        $display("============================================================");

        if (tests_failed == 0) begin
            $display("  *** ALL AXI RELU TESTS PASSED ***\n");
        end else begin
            $display("  *** SOME AXI RELU TESTS FAILED ***\n");
        end

        $finish;
    end

endmodule

`default_nettype wire
