// =============================================================================
// File    : tb_axi_gpio.sv
// Project : RISC-V CNN SoC Honours
// Purpose : Standalone self-checking testbench for axi_gpio.sv.
//
//           Verifies:
//             - Reset defaults (OUT=0, DIR=0, IN=0 after sync)
//             - Input synchronisation (two-flop delay)
//             - Output and direction register writes / readback
//             - SET, CLR, and TOGGLE action-register behaviour
//             - Partial writes and WSTRB handling
//             - GPIO-width masking (bits above GPIO_WIDTH ignored)
//             - GPIO_ID and GPIO_VERSION values
//             - Read-only register writes (SLVERR)
//             - Invalid, reserved, and misaligned addresses
//             - Independent AW/W timing (both arrival orders)
//             - BREADY and RREADY backpressure
//             - Unsupported burst handling (WLAST drain / RLAST multi-beat)
//             - Reset recovery and absence of deadlocks
//
//           Uses bounded AXI read/write tasks with timeouts.
//           Prints a PASS/FAIL summary and terminates with $fatal on errors.
//
// Compile (Icarus):
//   iverilog -g2012 -o tb_axi_gpio.vvp rtl/gpio/axi_gpio.sv tb/tb_axi_gpio.sv
//   vvp tb_axi_gpio.vvp
//
// Compile (VCS):
//   vcs -full64 -sverilog -timescale=1ns/1ps -top tb_axi_gpio \
//       rtl/gpio/axi_gpio.sv tb/tb_axi_gpio.sv -o simv_gpio_tb
//   ./simv_gpio_tb
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_axi_gpio;

    // =========================================================================
    // Parameters (match RTL defaults)
    // =========================================================================
    localparam integer GPIO_WIDTH       = 16;
    localparam integer AXI_DATA_WIDTH  = 64;
    localparam integer AXI_ADDR_WIDTH  = 32;
    localparam integer AXI_ID_WIDTH    = 8;
    localparam integer AXI_STRB_WIDTH  = 8;
    localparam integer REG_DATA_WIDTH = 32;

    localparam [31:0] GPIO_ID_VALUE      = 32'h4749504F;
    localparam [31:0] GPIO_VERSION_VALUE = 32'h00010000;

    // Clock period
    localparam integer CLK_PERIOD = 10;  // 10 ns = 100 MHz

    // Timeout for AXI handshakes (in clock cycles)
    localparam integer TIMEOUT_CYCLES = 1000;

    // =========================================================================
    // Signals
    // =========================================================================
    reg                              clk;
    reg                              rst;

    // AW
    reg  [AXI_ID_WIDTH-1:0]          awid;
    reg  [AXI_ADDR_WIDTH-1:0]        awaddr;
    reg  [7:0]                        awlen;
    reg  [2:0]                        awsize;
    reg  [1:0]                        awburst;
    reg                              awlock;
    reg  [3:0]                        awcache;
    reg  [2:0]                        awprot;
    reg  [3:0]                        awqos;
    reg  [3:0]                        awregion;
    reg                              awvalid;
    wire                             awready;

    // W
    reg  [AXI_DATA_WIDTH-1:0]        wdata;
    reg  [AXI_STRB_WIDTH-1:0]        wstrb;
    reg                              wlast;
    reg                              wvalid;
    wire                             wready;

    // B
    wire [AXI_ID_WIDTH-1:0]          bid;
    wire [1:0]                        bresp;
    wire                             bvalid;
    reg                              bready;

    // AR
    reg  [AXI_ID_WIDTH-1:0]          arid;
    reg  [AXI_ADDR_WIDTH-1:0]        araddr;
    reg  [7:0]                        arlen;
    reg  [2:0]                        arsize;
    reg  [1:0]                        arburst;
    reg                              arlock;
    reg  [3:0]                        arcache;
    reg  [2:0]                        arprot;
    reg  [3:0]                        arqos;
    reg  [3:0]                        arregion;
    reg                              arvalid;
    wire                             arready;

    // R
    wire [AXI_ID_WIDTH-1:0]          rid;
    wire [AXI_DATA_WIDTH-1:0]       rdata;
    wire [1:0]                        rresp;
    wire                             rlast;
    wire                             rvalid;
    reg                              rready;

    // GPIO pins
    reg  [GPIO_WIDTH-1:0]            gpio_i;
    wire [GPIO_WIDTH-1:0]           gpio_o;
    wire [GPIO_WIDTH-1:0]           gpio_oen;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    axi_gpio #(
        .GPIO_WIDTH          (GPIO_WIDTH),
        .AXI_DATA_WIDTH     (AXI_DATA_WIDTH),
        .AXI_ADDR_WIDTH     (AXI_ADDR_WIDTH),
        .AXI_ID_WIDTH       (AXI_ID_WIDTH),
        .AXI_STRB_WIDTH     (AXI_STRB_WIDTH),
        .GPIO_ID_VALUE      (GPIO_ID_VALUE),
        .GPIO_VERSION_VALUE (GPIO_VERSION_VALUE)
    ) u_dut (
        .clk               (clk),
        .rst               (rst),
        .s_axi_awid        (awid),
        .s_axi_awaddr      (awaddr),
        .s_axi_awlen       (awlen),
        .s_axi_awsize      (awsize),
        .s_axi_awburst     (awburst),
        .s_axi_awlock      (awlock),
        .s_axi_awcache     (awcache),
        .s_axi_awprot      (awprot),
        .s_axi_awqos       (awqos),
        .s_axi_awregion    (awregion),
        .s_axi_awvalid     (awvalid),
        .s_axi_awready     (awready),
        .s_axi_wdata       (wdata),
        .s_axi_wstrb       (wstrb),
        .s_axi_wlast       (wlast),
        .s_axi_wvalid      (wvalid),
        .s_axi_wready      (wready),
        .s_axi_bid         (bid),
        .s_axi_bresp       (bresp),
        .s_axi_bvalid      (bvalid),
        .s_axi_bready      (bready),
        .s_axi_arid        (arid),
        .s_axi_araddr      (araddr),
        .s_axi_arlen       (arlen),
        .s_axi_arsize      (arsize),
        .s_axi_arburst     (arburst),
        .s_axi_arlock      (arlock),
        .s_axi_arcache     (arcache),
        .s_axi_arprot      (arprot),
        .s_axi_arqos       (arqos),
        .s_axi_arregion    (arregion),
        .s_axi_arvalid     (arvalid),
        .s_axi_arready     (arready),
        .s_axi_rid         (rid),
        .s_axi_rdata       (rdata),
        .s_axi_rresp       (rresp),
        .s_axi_rlast       (rlast),
        .s_axi_rvalid      (rvalid),
        .s_axi_rready      (rready),
        .gpio_i            (gpio_i),
        .gpio_o            (gpio_o),
        .gpio_oen          (gpio_oen)
    );

    // =========================================================================
    // Clock generation
    // =========================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // Test accounting
    // =========================================================================
    integer tests_passed = 0;
    integer tests_failed = 0;
    integer total_checks = 0;

    // =========================================================================
    // Helper: check and report
    // =========================================================================
    task automatic check(input string desc, input [31:0] actual, input [31:0] expected);
        begin
            total_checks = total_checks + 1;
            if (actual === expected) begin
                tests_passed = tests_passed + 1;
            end else begin
                tests_failed = tests_failed + 1;
                $display("  [FAIL] %s: expected 0x%08h, got 0x%08h", desc, expected, actual);
            end
        end
    endtask

    task automatic check_resp(input string desc, input [1:0] actual, input [1:0] expected);
        begin
            total_checks = total_checks + 1;
            if (actual === expected) begin
                tests_passed = tests_passed + 1;
            end else begin
                tests_failed = tests_failed + 1;
                $display("  [FAIL] %s: expected resp %b, got resp %b", desc, expected, actual);
            end
        end
    endtask

    // =========================================================================
    // AXI write task (single-beat, bounded with timeout)
    // Drives AW and W channels, waits for B response.
    // Returns bresp value.
    // =========================================================================
    task automatic axi_write(
        input  [AXI_ADDR_WIDTH-1:0] addr,
        input  [REG_DATA_WIDTH-1:0] data,
        input  [3:0]                strb,
        output [1:0]                resp
    );
        integer timeout;
        begin
            // Drive AW and W simultaneously
            awid     = 8'h01;
            awaddr   = addr;
            awlen    = 8'd0;
            awsize   = 3'd2;  // 4 bytes
            awburst  = 2'b01; // INCR
            awlock   = 1'b0;
            awcache  = 4'h0;
            awprot   = 3'h0;
            awqos    = 4'h0;
            awregion = 4'h0;
            awvalid  = 1'b1;

            wdata    = {32'h0, data};  // upper 32 bits zero
            wstrb    = {4'h0, strb};   // upper strobe bits zero
            wlast    = 1'b1;
            wvalid   = 1'b1;

            bready   = 1'b1;

            // Both awvalid and wvalid were asserted above (single-beat write).
            // Wait for both channels ready, then both handshakes occur at the
            // same posedge clk (DUT is in WR_IDLE, so both ready immediately).
            timeout = 0;
            while (!(awready && wready) && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_write: AW/W handshake timeout (addr=0x%08h)", addr);
                tests_failed = tests_failed + 1;
                awvalid = 1'b0;
                wvalid  = 1'b0;
                resp = 2'b11;
                return;
            end
            @(posedge clk);  // AW and W handshakes occur here simultaneously
            awvalid = 1'b0;
            wvalid  = 1'b0;

            // Wait for B response
            timeout = 0;
            while (!bvalid && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_write: B timeout (addr=0x%08h)", addr);
                tests_failed = tests_failed + 1;
                resp = 2'b11;
                return;
            end
            resp = bresp;

            // Clear bready after one cycle
            @(posedge clk);
            bready = 1'b0;
        end
    endtask

    // =========================================================================
    // AXI read task (single-beat, bounded with timeout)
    // Drives AR channel, waits for R response.
    // Returns lower 32 bits of rdata and rresp.
    // =========================================================================
    task automatic axi_read(
        input  [AXI_ADDR_WIDTH-1:0]  addr,
        output [REG_DATA_WIDTH-1:0] data,
        output [1:0]                 resp
    );
        integer timeout;
        begin
            arid     = 8'h02;
            araddr   = addr;
            arlen    = 8'd0;
            arsize   = 3'd2;
            arburst  = 2'b01;
            arlock   = 1'b0;
            arcache  = 4'h0;
            arprot   = 3'h0;
            arqos    = 4'h0;
            arregion = 4'h0;
            arvalid  = 1'b1;

            rready   = 1'b1;

            // Wait for AR handshake: assert AR, wait for ready (high when DUT
            // in RD_IDLE), then handshake occurs at next posedge clk.
            arvalid = 1'b1;
            timeout = 0;
            while (!arready && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_read: AR handshake timeout (addr=0x%08h)", addr);
                tests_failed = tests_failed + 1;
                arvalid = 1'b0;
                data = 32'h0;
                resp = 2'b11;
                return;
            end
            @(posedge clk);  // AR handshake occurs here
            arvalid = 1'b0;

            // Wait for R response
            timeout = 0;
            while (!rvalid && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_read: R timeout (addr=0x%08h)", addr);
                tests_failed = tests_failed + 1;
                data = 32'h0;
                resp = 2'b11;
                return;
            end
            data = rdata[31:0];
            resp = rresp;

            @(posedge clk);
            rready = 1'b0;
        end
    endtask

    // =========================================================================
    // AXI write task with independent AW/W timing
    //   w_first=1: drive W before AW (W arrives first)
    //   w_first=0: drive AW before W (AW arrives first, W delayed by gap cycles)
    // =========================================================================
    task automatic axi_write_split(
        input  [AXI_ADDR_WIDTH-1:0] addr,
        input  [REG_DATA_WIDTH-1:0] data,
        input  [3:0]                strb,
        input                       w_first,   // 1 = W before AW
        input integer               gap,       // cycles between first and second
        output [1:0]                resp
    );
        integer timeout;
        integer i;
        begin
            // Initialize all AXI signals to idle
            awid     = 8'h03;
            awaddr   = addr;
            awlen    = 8'd0;
            awsize   = 3'd2;
            awburst  = 2'b01;
            awlock   = 1'b0;
            awcache  = 4'h0;
            awprot   = 3'h0;
            awqos    = 4'h0;
            awregion = 4'h0;
            awvalid  = 1'b0;

            wdata    = {32'h0, data};
            wstrb    = {4'h0, strb};
            wlast    = 1'b1;
            wvalid   = 1'b0;

            bready   = 1'b1;

            if (w_first) begin
                // Drive W first
                wvalid = 1'b1;
                // Wait for W handshake: wait for ready, then handshake at posedge
                timeout = 0;
                while (!wready && timeout < TIMEOUT_CYCLES) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (timeout >= TIMEOUT_CYCLES) begin
                    $display("  [FAIL] axi_write_split(W-first): W timeout");
                    tests_failed = tests_failed + 1;
                    wvalid = 1'b0;
                    resp = 2'b11;
                    return;
                end
                @(posedge clk);  // W handshake occurs here
                wvalid = 1'b0;

                // Gap cycles
                for (i = 0; i < gap; i++) @(posedge clk);

                // Now drive AW
                awvalid = 1'b1;
                timeout = 0;
                while (!awready && timeout < TIMEOUT_CYCLES) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (timeout >= TIMEOUT_CYCLES) begin
                    $display("  [FAIL] axi_write_split(W-first): AW timeout");
                    tests_failed = tests_failed + 1;
                    awvalid = 1'b0;
                    resp = 2'b11;
                    return;
                end
                @(posedge clk);  // AW handshake occurs here
                awvalid = 1'b0;
            end else begin
                // Drive AW first
                awvalid = 1'b1;
                timeout = 0;
                while (!awready && timeout < TIMEOUT_CYCLES) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (timeout >= TIMEOUT_CYCLES) begin
                    $display("  [FAIL] axi_write_split(AW-first): AW timeout");
                    tests_failed = tests_failed + 1;
                    awvalid = 1'b0;
                    resp = 2'b11;
                    return;
                end
                @(posedge clk);  // AW handshake occurs here
                awvalid = 1'b0;

                // Gap cycles
                for (i = 0; i < gap; i++) @(posedge clk);

                // Now drive W
                wvalid = 1'b1;
                timeout = 0;
                while (!wready && timeout < TIMEOUT_CYCLES) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (timeout >= TIMEOUT_CYCLES) begin
                    $display("  [FAIL] axi_write_split(AW-first): W timeout");
                    tests_failed = tests_failed + 1;
                    wvalid = 1'b0;
                    resp = 2'b11;
                    return;
                end
                @(posedge clk);  // W handshake occurs here
                wvalid = 1'b0;
            end

            // Wait for B response
            timeout = 0;
            while (!bvalid && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_write_split: B timeout");
                tests_failed = tests_failed + 1;
                resp = 2'b11;
                return;
            end
            resp = bresp;

            @(posedge clk);
            bready = 1'b0;
        end
    endtask

    // =========================================================================
    // Burst write task (for testing burst drain)
    // =========================================================================
    task automatic axi_burst_write(
        input  [AXI_ADDR_WIDTH-1:0] addr,
        input  [7:0]                len,      // arlen (beats-1)
        output [1:0]                resp
    );
        integer timeout;
        integer beat;
        begin
            awid     = 8'h04;
            awaddr   = addr;
            awlen    = len;
            awsize   = 3'd2;
            awburst  = 2'b01;
            awlock   = 1'b0;
            awcache  = 4'h0;
            awprot   = 3'h0;
            awqos    = 4'h0;
            awregion = 4'h0;
            awvalid  = 1'b1;

            bready   = 1'b1;

            // Wait for AW handshake: wait for ready, then handshake at posedge
            timeout = 0;
            while (!awready && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_burst_write: AW timeout");
                tests_failed = tests_failed + 1;
                awvalid = 1'b0;
                resp = 2'b11;
                return;
            end
            @(posedge clk);  // AW handshake occurs here
            awvalid = 1'b0;

            // Drive W beats
            for (beat = 0; beat <= len; beat++) begin
                wdata  = {32'h0, 32'hDEAD_BEEF};
                wstrb  = 8'hFF;
                wlast  = (beat == len) ? 1'b1 : 1'b0;
                wvalid = 1'b1;

                timeout = 0;
                while (!wready && timeout < TIMEOUT_CYCLES) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (timeout >= TIMEOUT_CYCLES) begin
                    $display("  [FAIL] axi_burst_write: W beat %0d timeout", beat);
                    tests_failed = tests_failed + 1;
                    wvalid = 1'b0;
                    resp = 2'b11;
                    return;
                end
                @(posedge clk);  // W beat handshake occurs here
                wvalid = 1'b0;
            end
            wlast  = 1'b0;

            // Wait for B response
            timeout = 0;
            while (!bvalid && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_burst_write: B timeout");
                tests_failed = tests_failed + 1;
                resp = 2'b11;
                return;
            end
            resp = bresp;

            @(posedge clk);
            bready = 1'b0;
        end
    endtask

    // =========================================================================
    // Burst read task (for testing multi-beat R response)
    // =========================================================================
    task automatic axi_burst_read(
        input  [AXI_ADDR_WIDTH-1:0] addr,
        input  [7:0]                len,      // arlen (beats-1)
        output [1:0]                resp,
        output                      last_seen
    );
        integer timeout;
        integer beat;
        reg last_r;
        begin
            arid     = 8'h05;
            araddr   = addr;
            arlen    = len;
            arsize   = 3'd2;
            arburst  = 2'b01;
            arlock   = 1'b0;
            arcache  = 4'h0;
            arprot   = 3'h0;
            arqos    = 4'h0;
            arregion = 4'h0;
            arvalid  = 1'b1;

            rready   = 1'b1;
            last_seen = 1'b0;

            // Wait for AR handshake: wait for ready, then handshake at posedge
            arvalid = 1'b1;
            timeout = 0;
            while (!arready && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] axi_burst_read: AR timeout");
                tests_failed = tests_failed + 1;
                arvalid = 1'b0;
                resp = 2'b11;
                return;
            end
            @(posedge clk);  // AR handshake occurs here
            arvalid = 1'b0;

            // Collect R beats
            beat = 0;
            last_r = 1'b0;
            while (!last_r && beat < (len + 1) + 10) begin
                timeout = 0;
                while (!rvalid && timeout < TIMEOUT_CYCLES) begin
                    @(posedge clk);
                    timeout = timeout + 1;
                end
                if (timeout >= TIMEOUT_CYCLES) begin
                    $display("  [FAIL] axi_burst_read: R beat %0d timeout", beat);
                    tests_failed = tests_failed + 1;
                    resp = 2'b11;
                    return;
                end
                last_r = rlast;
                if (beat == 0) resp = rresp;
                last_seen = last_r;
                @(posedge clk);
                beat = beat + 1;
            end

            // Check we saw RLAST
            if (!last_r) begin
                $display("  [FAIL] axi_burst_read: RLAST not seen after %0d beats", beat);
                tests_failed = tests_failed + 1;
            end

            @(posedge clk);
            rready = 1'b0;
        end
    endtask

    // =========================================================================
    // Idle bus initialization
    // =========================================================================
    task automatic idle_bus;
        begin
            awid     = 8'h0;  awaddr   = 32'h0;  awlen  = 8'd0;
            awsize   = 3'd0;   awburst  = 2'b00;  awlock = 1'b0;
            awcache  = 4'h0;   awprot   = 3'h0;   awqos  = 4'h0;
            awregion = 4'h0;   awvalid  = 1'b0;
            wdata    = 64'h0;  wstrb    = 8'h0;   wlast  = 1'b0;
            wvalid   = 1'b0;   bready   = 1'b0;
            arid     = 8'h0;   araddr   = 32'h0;  arlen  = 8'd0;
            arsize   = 3'd0;   arburst  = 2'b00;  arlock = 1'b0;
            arcache  = 4'h0;   arprot   = 3'h0;   arqos  = 4'h0;
            arregion = 4'h0;   arvalid  = 1'b0;   rready = 1'b0;
        end
    endtask

    // =========================================================================
    // Reset task
    // =========================================================================
    task automatic do_reset;
        integer i;
        begin
            idle_bus();
            gpio_i = '0;
            rst = 1'b1;
            for (i = 0; i < 5; i++) @(posedge clk);
            @(negedge clk);
            rst = 1'b0;
            for (i = 0; i < 3; i++) @(posedge clk);
        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    reg [1:0]               resp_r;
    reg [31:0]              data_r;
    reg                     last_r;
    reg [GPIO_WIDTH-1:0]    expected_out;
    reg [GPIO_WIDTH-1:0]    expected_dir;

    initial begin
        // -------------------------------------------------------------------
        // Dump waveform (optional, for VCD)
        // -------------------------------------------------------------------
        // $dumpfile("tb_axi_gpio.vcd");
        // $dumpvars(0, tb_axi_gpio);

        $display("");
        $display("==============================================================");
        $display("  tb_axi_gpio — AXI GPIO IP Testbench");
        $display("==============================================================");
        $display("");

        // ===================================================================
        // T1: Reset defaults
        // ===================================================================
        $display("[T1] Reset defaults ...");
        do_reset();

        // After reset, all registers should be 0
        axi_read(32'h000, data_r, resp_r);
        check("T1 GPIO_IN after reset", data_r, 32'h0);
        check_resp("T1 GPIO_IN resp", resp_r, 2'b00);

        axi_read(32'h004, data_r, resp_r);
        check("T1 GPIO_OUT after reset", data_r, 32'h0);
        check_resp("T1 GPIO_OUT resp", resp_r, 2'b00);

        axi_read(32'h008, data_r, resp_r);
        check("T1 GPIO_DIR after reset", data_r, 32'h0);
        check_resp("T1 GPIO_DIR resp", resp_r, 2'b00);

        // Physical outputs
        check("T1 gpio_o == 0", {{(32-GPIO_WIDTH){1'b0}}, gpio_o}, 32'h0);
        check("T1 gpio_oen == 0", {{(32-GPIO_WIDTH){1'b0}}, gpio_oen}, 32'h0);
        $display("  T1 done.");

        // ===================================================================
        // T2: Input synchronisation (two-flop delay)
        // ===================================================================
        $display("[T2] Input synchronisation ...");
        do_reset();

        // Drive a pattern on gpio_i
        gpio_i = 16'hA5A5;
        // After reset, sync regs are 0.  Need 2 posedge clk to propagate.
        @(posedge clk);  // sync1 = A5A5, sync2 = 0
        // Check directly after 1 cycle (axi_read takes multiple cycles so
        // sync2 would already have propagated by then)
        check("T2 GPIO_IN after 1 cycle (still 0)",
              {{(32-GPIO_WIDTH){1'b0}}, u_dut.gpio_sync2}, 32'h0);

        @(posedge clk);  // sync1 = A5A5, sync2 = A5A5
        axi_read(32'h000, data_r, resp_r);
        check("T2 GPIO_IN after 2 cycles (A5A5)", data_r, 32'h0000A5A5);
        check_resp("T2 GPIO_IN resp", resp_r, 2'b00);

        // Change input and check again
        gpio_i = 16'h3C3C;
        @(posedge clk);
        @(posedge clk);
        axi_read(32'h000, data_r, resp_r);
        check("T2 GPIO_IN after change (3C3C)", data_r, 32'h00003C3C);
        $display("  T2 done.");

        // ===================================================================
        // T3: Output register write/readback
        // ===================================================================
        $display("[T3] Output register write/readback ...");
        do_reset();
        expected_out = 16'h0;

        // Write 0x1234 to GPIO_OUT
        axi_write(32'h004, 32'h0000_1234, 4'hF, resp_r);
        check_resp("T3 GPIO_OUT write resp", resp_r, 2'b00);
        expected_out = 16'h1234;

        axi_read(32'h004, data_r, resp_r);
        check("T3 GPIO_OUT readback", data_r, 32'h0000_1234);
        check_resp("T3 GPIO_OUT read resp", resp_r, 2'b00);

        // Write 0xFFFF (all bits, within GPIO_WIDTH=16)
        axi_write(32'h004, 32'h0000_FFFF, 4'hF, resp_r);
        expected_out = 16'hFFFF;
        axi_read(32'h004, data_r, resp_r);
        check("T3 GPIO_OUT all-ones", data_r, 32'h0000_FFFF);

        // Write 0x10000 (bit 16 — above GPIO_WIDTH, should be masked).
        // Use strb=0x4 so only byte 2 is written; bytes 0-1 stay 0xFF.
        axi_write(32'h004, 32'h0001_0000, 4'h4, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T3 GPIO_OUT bit-16 masked", data_r, 32'h0000_FFFF);
        $display("  T3 done.");

        // ===================================================================
        // T4: Direction register write/readback
        // ===================================================================
        $display("[T4] Direction register write/readback ...");
        do_reset();
        expected_dir = 16'h0;

        axi_write(32'h008, 32'h0000_ABCD, 4'hF, resp_r);
        check_resp("T4 GPIO_DIR write resp", resp_r, 2'b00);
        expected_dir = 16'hABCD;

        axi_read(32'h008, data_r, resp_r);
        check("T4 GPIO_DIR readback", data_r, 32'h0000_ABCD);

        // Physical oen should match dir
        check("T4 gpio_oen matches DIR", gpio_oen, 16'hABCD);

        // Bit above GPIO_WIDTH masked. Use strb=0x4 so only byte 2 is
        // written; bytes 0-1 stay 0xAB/0xCD.
        axi_write(32'h008, 32'h0001_0000, 4'h4, resp_r);
        axi_read(32'h008, data_r, resp_r);
        check("T4 GPIO_DIR bit-16 masked", data_r, 32'h0000_ABCD);
        $display("  T4 done.");

        // ===================================================================
        // T5: SET, CLR, TOGGLE action registers
        // ===================================================================
        $display("[T5] SET, CLR, TOGGLE ...");
        do_reset();
        expected_out = 16'h0;

        // SET bits 0, 3, 7
        axi_write(32'h00C, 32'h0000_0089, 4'hF, resp_r);  // 0b10001001
        check_resp("T5 SET resp", resp_r, 2'b00);
        expected_out = expected_out | 16'h0089;

        axi_read(32'h004, data_r, resp_r);
        check("T5 OUT after SET", data_r, 32'h0000_0089);

        // CLR bits 0 and 7
        axi_write(32'h010, 32'h0000_0081, 4'hF, resp_r);  // 0b10000001
        expected_out = expected_out & ~16'h0081;

        axi_read(32'h004, data_r, resp_r);
        check("T5 OUT after CLR", data_r, 32'h0000_0008);

        // TOGGLE bit 3
        axi_write(32'h014, 32'h0000_0008, 4'hF, resp_r);
        expected_out = expected_out ^ 16'h0008;

        axi_read(32'h004, data_r, resp_r);
        check("T5 OUT after TOGGLE", data_r, 32'h0000_0000);

        // TOGGLE again (should set it back)
        axi_write(32'h014, 32'h0000_0008, 4'hF, resp_r);
        expected_out = expected_out ^ 16'h0008;

        axi_read(32'h004, data_r, resp_r);
        check("T5 OUT after TOGGLE again", data_r, 32'h0000_0008);

        // SET above GPIO_WIDTH (should have no effect)
        axi_write(32'h00C, 32'h0001_0000, 4'hF, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T5 SET bit-16 no effect", data_r, 32'h0000_0008);

        // SET/CLR/TOGGLE read as 0 (write-only)
        axi_read(32'h00C, data_r, resp_r);
        check("T5 SET read-as-0", data_r, 32'h0);
        axi_read(32'h010, data_r, resp_r);
        check("T5 CLR read-as-0", data_r, 32'h0);
        axi_read(32'h014, data_r, resp_r);
        check("T5 TOGGLE read-as-0", data_r, 32'h0);
        $display("  T5 done.");

        // ===================================================================
        // T6: Partial writes and WSTRB handling
        // ===================================================================
        $display("[T6] Partial writes (WSTRB) ...");
        do_reset();

        // Write full word first
        axi_write(32'h004, 32'h0000_DEAD, 4'hF, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T6 full write", data_r, 32'h0000_DEAD);

        // Write only byte 0 (strb=0x1)
        axi_write(32'h004, 32'h0000_00BE, 4'h1, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T6 byte0 write (strb=0x1)", data_r, 32'h0000_DEBE);

        // Write only byte 1 (strb=0x2)
        axi_write(32'h004, 32'h0000_AD00, 4'h2, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T6 byte1 write (strb=0x2)", data_r, 32'h0000_ADBE);

        // Write bytes 0+1 (strb=0x3)
        axi_write(32'h004, 32'h0000_CAFE, 4'h3, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T6 bytes0-1 write (strb=0x3)", data_r, 32'h0000_CAFE);

        // Write byte 2 (strb=0x4) — above GPIO_WIDTH, but byte still written
        // For GPIO_WIDTH=16, byte 2 = bits [23:16], all above GPIO_WIDTH
        // The GPIO_MASK should prevent any change
        axi_write(32'h004, 32'h00FF_0000, 4'h4, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T6 byte2 write masked by GPIO_WIDTH", data_r, 32'h0000_CAFE);
        $display("  T6 done.");

        // ===================================================================
        // T7: GPIO_ID and GPIO_VERSION
        // ===================================================================
        $display("[T7] GPIO_ID and GPIO_VERSION ...");
        do_reset();

        axi_read(32'h018, data_r, resp_r);
        check("T7 GPIO_ID", data_r, GPIO_ID_VALUE);
        check_resp("T7 GPIO_ID resp", resp_r, 2'b00);

        axi_read(32'h01C, data_r, resp_r);
        check("T7 GPIO_VERSION", data_r, GPIO_VERSION_VALUE);
        check_resp("T7 GPIO_VERSION resp", resp_r, 2'b00);
        $display("  T7 done.");

        // ===================================================================
        // T8: Read-only register writes (SLVERR)
        // ===================================================================
        $display("[T8] Read-only register writes ...");
        do_reset();

        // Write to GPIO_IN (0x00) — should return SLVERR
        axi_write(32'h000, 32'hFFFF_FFFF, 4'hF, resp_r);
        check_resp("T8 write GPIO_IN → SLVERR", resp_r, 2'b10);

        // Write to GPIO_ID (0x18) — should return SLVERR
        axi_write(32'h018, 32'hFFFF_FFFF, 4'hF, resp_r);
        check_resp("T8 write GPIO_ID → SLVERR", resp_r, 2'b10);

        // Write to GPIO_VERSION (0x1C) — should return SLVERR
        axi_write(32'h01C, 32'hFFFF_FFFF, 4'hF, resp_r);
        check_resp("T8 write GPIO_VERSION → SLVERR", resp_r, 2'b10);

        // Verify registers unchanged
        axi_read(32'h018, data_r, resp_r);
        check("T8 GPIO_ID unchanged after write", data_r, GPIO_ID_VALUE);
        axi_read(32'h01C, data_r, resp_r);
        check("T8 GPIO_VERSION unchanged after write", data_r, GPIO_VERSION_VALUE);
        $display("  T8 done.");

        // ===================================================================
        // T9: Invalid, reserved, and misaligned addresses
        // ===================================================================
        $display("[T9] Invalid, reserved, misaligned addresses ...");
        do_reset();

        // Reserved offset 0x20 (within 4 KiB but no register) — SLVERR
        axi_write(32'h020, 32'h1234_5678, 4'hF, resp_r);
        check_resp("T9 reserved 0x20 write → SLVERR", resp_r, 2'b10);

        axi_read(32'h020, data_r, resp_r);
        check_resp("T9 reserved 0x20 read → SLVERR", resp_r, 2'b10);
        check("T9 reserved 0x20 read data 0", data_r, 32'h0);

        // Reserved offset 0xFFC (last word in 4 KiB) — SLVERR
        axi_read(32'hFFC, data_r, resp_r);
        check_resp("T9 reserved 0xFFC read → SLVERR", resp_r, 2'b10);

        // Out-of-aperture address 0x1000 (above 4 KiB) — DECERR
        axi_write(32'h1000, 32'hAAAA_AAAA, 4'hF, resp_r);
        check_resp("T9 out-of-aperture 0x1000 write → DECERR", resp_r, 2'b11);

        axi_read(32'h1000, data_r, resp_r);
        check_resp("T9 out-of-aperture 0x1000 read → DECERR", resp_r, 2'b11);

        // Out-of-aperture address 0x60000000 (full m06 base) — DECERR
        axi_read(32'h6000_0000, data_r, resp_r);
        check_resp("T9 out-of-aperture 0x60000000 → DECERR", resp_r, 2'b11);

        // Misaligned address 0x005 (not word-aligned) — SLVERR
        axi_write(32'h005, 32'h1234_5678, 4'hF, resp_r);
        check_resp("T9 misaligned 0x005 write → SLVERR", resp_r, 2'b10);

        axi_read(32'h003, data_r, resp_r);
        check_resp("T9 misaligned 0x003 read → SLVERR", resp_r, 2'b10);

        // Misaligned address 0x001 — SLVERR
        axi_read(32'h001, data_r, resp_r);
        check_resp("T9 misaligned 0x001 read → SLVERR", resp_r, 2'b10);
        $display("  T9 done.");

        // ===================================================================
        // T10: Independent AW/W timing (both arrival orders)
        // ===================================================================
        $display("[T10] Independent AW/W timing ...");
        do_reset();

        // W before AW (gap=2)
        axi_write_split(32'h004, 32'h0000_5678, 4'hF, 1'b1, 2, resp_r);
        check_resp("T10 W-first resp OKAY", resp_r, 2'b00);
        axi_read(32'h004, data_r, resp_r);
        check("T10 W-first data", data_r, 32'h0000_5678);

        // AW before W (gap=3)
        axi_write_split(32'h004, 32'h0000_9ABC, 4'hF, 1'b0, 3, resp_r);
        check_resp("T10 AW-first resp OKAY", resp_r, 2'b00);
        axi_read(32'h004, data_r, resp_r);
        check("T10 AW-first data", data_r, 32'h0000_9ABC);

        // W before AW (gap=0, immediately)
        axi_write_split(32'h004, 32'h0000_0F0F, 4'hF, 1'b1, 0, resp_r);
        check_resp("T10 W-first gap0 resp OKAY", resp_r, 2'b00);
        axi_read(32'h004, data_r, resp_r);
        check("T10 W-first gap0 data", data_r, 32'h0000_0F0F);

        // AW before W (gap=0)
        axi_write_split(32'h004, 32'h0000_F0F0, 4'hF, 1'b0, 0, resp_r);
        check_resp("T10 AW-first gap0 resp OKAY", resp_r, 2'b00);
        axi_read(32'h004, data_r, resp_r);
        check("T10 AW-first gap0 data", data_r, 32'h0000_F0F0);
        $display("  T10 done.");

        // ===================================================================
        // T11: BREADY backpressure
        // ===================================================================
        $display("[T11] BREADY backpressure ...");
        do_reset();

        // Initiate write but delay BREADY
        begin
            integer timeout;
            awid = 8'h0A; awaddr = 32'h004; awlen = 8'd0;
            awsize = 3'd2; awburst = 2'b01; awlock = 1'b0;
            awcache = 4'h0; awprot = 3'h0; awqos = 4'h0; awregion = 4'h0;
            awvalid = 1'b1;
            wdata = {32'h0, 32'h0000_BEEF}; wstrb = 8'h0F; wlast = 1'b1;
            wvalid = 1'b1;
            bready = 1'b0;  // Hold BREADY low

            // Wait for AW and W handshakes (both asserted together, DUT in IDLE
            // so both ready immediately; handshake occurs at next posedge)
            timeout = 0;
            while (!(awready && wready) && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= TIMEOUT_CYCLES) begin
                $display("  [FAIL] T11: AW/W handshake timeout");
                tests_failed = tests_failed + 1;
                awvalid = 1'b0;
                wvalid = 1'b0;
            end else begin
                @(posedge clk);  // Both handshakes occur here
                awvalid = 1'b0;
                wvalid = 1'b0;
            end

            // BVALID should be asserted but BREADY is low
            timeout = 0;
            while (!bvalid && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk); timeout = timeout + 1;
            end
            if (!bvalid) begin
                $display("  [FAIL] T11: BVALID not asserted");
                tests_failed = tests_failed + 1;
            end

            // Wait several cycles with BREADY low
            for (integer i = 0; i < 5; i++) @(posedge clk);

            // Now assert BREADY
            bready = 1'b1;
            @(posedge clk);
            bready = 1'b0;

            check_resp("T11 BREADY backpressure resp", bresp, 2'b00);
        end

        axi_read(32'h004, data_r, resp_r);
        check("T11 data after backpressure", data_r, 32'h0000_BEEF);
        $display("  T11 done.");

        // ===================================================================
        // T12: RREADY backpressure
        // ===================================================================
        $display("[T12] RREADY backpressure ...");
        do_reset();

        // Write a known value first
        axi_write(32'h004, 32'h0000_CAFE, 4'hF, resp_r);

        // Initiate read but delay RREADY
        begin
            integer timeout;
            arid = 8'h0B; araddr = 32'h004; arlen = 8'd0;
            arsize = 3'd2; arburst = 2'b01; arlock = 1'b0;
            arcache = 4'h0; arprot = 3'h0; arqos = 4'h0; arregion = 4'h0;
            arvalid = 1'b1;
            rready = 1'b0;

            // Wait for AR handshake: wait for ready, then handshake at posedge
            timeout = 0;
            while (!arready && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk); timeout = timeout + 1;
            end
            if (timeout < TIMEOUT_CYCLES) begin
                @(posedge clk);  // AR handshake occurs here
            end
            arvalid = 1'b0;

            // RVALID should be asserted but RREADY is low
            timeout = 0;
            while (!rvalid && timeout < TIMEOUT_CYCLES) begin
                @(posedge clk); timeout = timeout + 1;
            end
            if (!rvalid) begin
                $display("  [FAIL] T12: RVALID not asserted");
                tests_failed = tests_failed + 1;
            end

            // Hold RREADY low for several cycles, check RVALID stays high
            for (integer i = 0; i < 5; i++) begin
                if (!rvalid) begin
                    $display("  [FAIL] T12: RVALID dropped during backpressure");
                    tests_failed = tests_failed + 1;
                end
                @(posedge clk);
            end

            // Now assert RREADY
            rready = 1'b1;
            @(posedge clk);
            rready = 1'b0;

            check("T12 RREADY backpressure data", rdata[31:0], 32'h0000_CAFE);
            check_resp("T12 RREADY backpressure resp", rresp, 2'b00);
        end
        $display("  T12 done.");

        // ===================================================================
        // T13: Unsupported burst handling (WLAST drain / RLAST)
        // ===================================================================
        $display("[T13] Burst handling ...");
        do_reset();

        // Burst write (len=3 → 4 beats) — should drain and return SLVERR
        axi_burst_write(32'h004, 8'd3, resp_r);
        check_resp("T13 burst write → SLVERR", resp_r, 2'b10);

        // Verify GPIO_OUT was NOT modified
        axi_read(32'h004, data_r, resp_r);
        check("T13 OUT unchanged after burst write", data_r, 32'h0);

        // Burst read (len=2 → 3 beats) — should return SLVERR on all, RLAST on last
        axi_burst_read(32'h000, 8'd2, resp_r, last_r);
        check_resp("T13 burst read → SLVERR", resp_r, 2'b10);
        check("T13 burst read RLAST seen", last_r, 1'b1);

        // Burst read with larger len (len=7 → 8 beats)
        axi_burst_read(32'h004, 8'd7, resp_r, last_r);
        check_resp("T13 burst read 8-beat → SLVERR", resp_r, 2'b10);
        check("T13 burst read 8-beat RLAST seen", last_r, 1'b1);
        $display("  T13 done.");

        // ===================================================================
        // T14: Reset recovery
        // ===================================================================
        $display("[T14] Reset recovery ...");
        do_reset();

        // Write some non-zero values
        axi_write(32'h004, 32'h0000_AAAA, 4'hF, resp_r);
        axi_write(32'h008, 32'h0000_5555, 4'hF, resp_r);
        gpio_i = 16'hFFFF;
        @(posedge clk); @(posedge clk);

        // Verify they took effect
        axi_read(32'h004, data_r, resp_r);
        check("T14 OUT before reset", data_r, 32'h0000_AAAA);
        axi_read(32'h008, data_r, resp_r);
        check("T14 DIR before reset", data_r, 32'h0000_5555);

        // Assert reset
        do_reset();

        // Verify all back to defaults
        axi_read(32'h004, data_r, resp_r);
        check("T14 OUT after reset", data_r, 32'h0);
        axi_read(32'h008, data_r, resp_r);
        check("T14 DIR after reset", data_r, 32'h0);
        axi_read(32'h000, data_r, resp_r);
        check("T14 IN after reset (gpio_i=0)", data_r, 32'h0);

        // Verify bus is functional after reset (no deadlock)
        axi_write(32'h004, 32'h0000_1234, 4'hF, resp_r);
        check_resp("T14 write after reset OKAY", resp_r, 2'b00);
        axi_read(32'h004, data_r, resp_r);
        check("T14 read after reset", data_r, 32'h0000_1234);
        $display("  T14 done.");

        // ===================================================================
        // T15: GPIO-width masking comprehensive
        // ===================================================================
        $display("[T15] GPIO-width masking ...");
        do_reset();

        // Write all-ones to GPIO_OUT — should mask to GPIO_WIDTH bits
        axi_write(32'h004, 32'hFFFF_FFFF, 4'hF, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T15 OUT all-ones masked", data_r, 32'h0000_FFFF);

        // Write all-ones to GPIO_DIR
        axi_write(32'h008, 32'hFFFF_FFFF, 4'hF, resp_r);
        axi_read(32'h008, data_r, resp_r);
        check("T15 DIR all-ones masked", data_r, 32'h0000_FFFF);

        // Physical outputs: oen=0xFFFF, o = 0xFFFF (masked by dir)
        check("T15 gpio_oen all-ones", gpio_oen, 16'hFFFF);
        check("T15 gpio_o all-ones", gpio_o, 16'hFFFF);

        // SET all-ones (should only set lower 16)
        axi_write(32'h00C, 32'hFFFF_FFFF, 4'hF, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T15 SET all-ones masked", data_r, 32'h0000_FFFF);

        // CLR all-ones
        axi_write(32'h010, 32'hFFFF_FFFF, 4'hF, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T15 CLR all-ones masked", data_r, 32'h0000_0000);

        // TOGGLE all-ones (should toggle only lower 16)
        axi_write(32'h014, 32'hFFFF_FFFF, 4'hF, resp_r);
        axi_read(32'h004, data_r, resp_r);
        check("T15 TOGGLE all-ones masked", data_r, 32'h0000_FFFF);
        $display("  T15 done.");

        // ===================================================================
        // Summary
        // ===================================================================
        $display("");
        $display("==============================================================");
        $display("  Test Summary");
        $display("==============================================================");
        $display("  Total checks : %0d", total_checks);
        $display("  Passed       : %0d", tests_passed);
        $display("  Failed       : %0d", tests_failed);
        $display("==============================================================");
        $display("");

        if (tests_failed > 0) begin
            $display("*** TESTBENCH FAILED ***");
            $fatal(1, "tb_axi_gpio: %0d checks failed", tests_failed);
        end else begin
            $display("*** ALL TESTS PASSED ***");
            $finish;
        end
    end

    // =========================================================================
    // Watchdog timer — catch deadlocks
    // =========================================================================
    initial begin
        #(CLK_PERIOD * 100000);  // 100K cycles max
        $display("");
        $display("  [FAIL] WATCHDOG: simulation timeout (deadlock detected)");
        $fatal(2, "tb_axi_gpio: watchdog timeout");
    end

endmodule

`default_nettype wire
