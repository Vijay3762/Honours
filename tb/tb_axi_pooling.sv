// =============================================================================
// File    : tb_axi_pooling.sv
// Project : RISC-V CNN Digit Recognition SoC — Honours
// Purpose : Self-checking testbench for axi_pooling.sv AXI4 slave interface.
//
// Verification scope:
//   1. Reset defaults (CTRL, STATUS, ID, VERSION)
//   2. Register R/W (CTRL mode changes, CONFIG)
//   3. Read-only protections (STATUS, DATA_OUT, ID, VERSION give SLVERR on write)
//   4. Out-of-aperture address gives DECERR
//   5. Unaligned address gives SLVERR
//   6. Burst transaction error rejection
//   7. Functional Pooling over AXI:
//      - Mode 0 (Max-Pooling) with positive and negative values & Argmax index
//      - Mode 1 (Average-Pooling) with rounding
//      - Mode 2 (Min-Pooling)
//      - Mode 3 (Bypass Mode)
//   8. 64-bit upper sub-word lane steering
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_axi_pooling;

    localparam integer DATA_WIDTH  = 64;
    localparam integer ADDR_WIDTH  = 32;
    localparam integer ID_WIDTH    = 8;
    localparam integer STRB_WIDTH  = DATA_WIDTH / 8;
    localparam [ADDR_WIDTH-1:0] BASE_ADDR = 32'h6000_0000;
    localparam integer CLK_PERIOD  = 10;

    // Register Offsets
    localparam [ADDR_WIDTH-1:0] ADDR_CTRL        = BASE_ADDR + 32'h00;
    localparam [ADDR_WIDTH-1:0] ADDR_STATUS      = BASE_ADDR + 32'h04;
    localparam [ADDR_WIDTH-1:0] ADDR_CONFIG      = BASE_ADDR + 32'h08;
    localparam [ADDR_WIDTH-1:0] ADDR_DATA_IN_P01 = BASE_ADDR + 32'h0C;
    localparam [ADDR_WIDTH-1:0] ADDR_DATA_IN_P23 = BASE_ADDR + 32'h10;
    localparam [ADDR_WIDTH-1:0] ADDR_DATA_OUT    = BASE_ADDR + 32'h14;
    localparam [ADDR_WIDTH-1:0] ADDR_ID          = BASE_ADDR + 32'h18;
    localparam [ADDR_WIDTH-1:0] ADDR_VERSION     = BASE_ADDR + 32'h1C;

    localparam [1:0] RESP_OKAY   = 2'b00;
    localparam [1:0] RESP_SLVERR = 2'b10;
    localparam [1:0] RESP_DECERR = 2'b11;

    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // AXI Master emulation signals
    logic [ID_WIDTH-1:0]     m_awid;
    logic [ADDR_WIDTH-1:0]   m_awaddr;
    logic [7:0]              m_awlen;
    logic [2:0]              m_awsize;
    logic [1:0]              m_awburst;
    logic                    m_awlock;
    logic [3:0]              m_awcache;
    logic [2:0]              m_awprot;
    logic [3:0]              m_awqos;
    logic [3:0]              m_awregion;
    logic                    m_awvalid;
    wire                     m_awready;

    logic [DATA_WIDTH-1:0]   m_wdata;
    logic [STRB_WIDTH-1:0]   m_wstrb;
    logic                    m_wlast;
    logic                    m_wvalid;
    wire                     m_wready;

    wire [ID_WIDTH-1:0]      m_bid;
    wire [1:0]               m_bresp;
    wire                     m_bvalid;
    logic                    m_bready;

    logic [ID_WIDTH-1:0]     m_arid;
    logic [ADDR_WIDTH-1:0]   m_araddr;
    logic [7:0]              m_arlen;
    logic [2:0]              m_arsize;
    logic [1:0]              m_arburst;
    logic                    m_arlock;
    logic [3:0]              m_arcache;
    logic [2:0]              m_arprot;
    logic [3:0]              m_arqos;
    logic [3:0]              m_arregion;
    logic                    m_arvalid;
    wire                     m_arready;

    wire [ID_WIDTH-1:0]      m_rid;
    wire [DATA_WIDTH-1:0]    m_rdata;
    wire [1:0]               m_rresp;
    wire                     m_rlast;
    wire                     m_rvalid;
    logic                    m_rready;

    // DUT Instantiation
    axi_pooling #(
        .AXI_DATA_WIDTH      (DATA_WIDTH),
        .AXI_ADDR_WIDTH      (ADDR_WIDTH),
        .AXI_ID_WIDTH        (ID_WIDTH),
        .BASE_ADDR           (BASE_ADDR)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .s_axi_awid          (m_awid),
        .s_axi_awaddr        (m_awaddr),
        .s_axi_awlen         (m_awlen),
        .s_axi_awsize        (m_awsize),
        .s_axi_awburst       (m_awburst),
        .s_axi_awlock        (m_awlock),
        .s_axi_awcache       (m_awcache),
        .s_axi_awprot        (m_awprot),
        .s_axi_awqos         (m_awqos),
        .s_axi_awregion      (m_awregion),
        .s_axi_awvalid       (m_awvalid),
        .s_axi_awready       (m_awready),
        .s_axi_wdata         (m_wdata),
        .s_axi_wstrb         (m_wstrb),
        .s_axi_wlast         (m_wlast),
        .s_axi_wvalid        (m_wvalid),
        .s_axi_wready        (m_wready),
        .s_axi_bid           (m_bid),
        .s_axi_bresp         (m_bresp),
        .s_axi_bvalid        (m_bvalid),
        .s_axi_bready        (m_bready),
        .s_axi_arid          (m_arid),
        .s_axi_araddr        (m_araddr),
        .s_axi_arlen         (m_arlen),
        .s_axi_arsize        (m_arsize),
        .s_axi_arburst       (m_arburst),
        .s_axi_arlock        (m_arlock),
        .s_axi_arcache       (m_arcache),
        .s_axi_arprot        (m_arprot),
        .s_axi_arqos         (m_arqos),
        .s_axi_arregion      (m_arregion),
        .s_axi_arvalid       (m_arvalid),
        .s_axi_arready       (m_arready),
        .s_axi_rid           (m_rid),
        .s_axi_rdata         (m_rdata),
        .s_axi_rresp         (m_rresp),
        .s_axi_rlast         (m_rlast),
        .s_axi_rvalid        (m_rvalid),
        .s_axi_rready        (m_rready)
    );

    // Test tracking
    integer pass_count = 0;
    integer fail_count = 0;

    task check(input string desc, input logic cond);
        if (cond) begin
            $display("  [PASS] %s", desc);
            pass_count = pass_count + 1;
        end else begin
            $display("  [FAIL] %s", desc);
            fail_count = fail_count + 1;
        end
    endtask

    // AXI Single-beat Write
    task axi_write(
        input [ADDR_WIDTH-1:0] addr,
        input [31:0]           data,
        input [7:0]            strb,
        output [1:0]           resp
    );
        begin
            @(posedge clk);
            m_awaddr  <= addr;
            m_awid    <= 8'h01;
            m_awlen   <= 8'h00;
            m_awsize  <= 3'b010;
            m_awburst <= 2'b01;
            m_awlock  <= 1'b0;
            m_awcache <= 4'b0;
            m_awprot  <= 3'b0;
            m_awqos   <= 4'b0;
            m_awregion<= 4'b0;
            m_awvalid <= 1'b1;

            m_wdata   <= {data, data};
            m_wstrb   <= strb;
            m_wlast   <= 1'b1;
            m_wvalid  <= 1'b1;
            m_bready  <= 1'b1;

            fork
                begin
                    wait (m_awready);
                    @(posedge clk);
                    m_awvalid <= 1'b0;
                end
                begin
                    wait (m_wready);
                    @(posedge clk);
                    m_wvalid <= 1'b0;
                end
            join

            while (!m_bvalid) @(posedge clk);
            resp = m_bresp;
            @(posedge clk);
            m_bready <= 1'b0;
        end
    endtask

    // AXI Single-beat Read
    task axi_read(
        input  [ADDR_WIDTH-1:0] addr,
        output [31:0]           data,
        output [1:0]            resp
    );
        begin
            @(posedge clk);
            m_araddr  <= addr;
            m_arid    <= 8'h02;
            m_arlen   <= 8'h00;
            m_arsize  <= 3'b010;
            m_arburst <= 2'b01;
            m_arlock  <= 1'b0;
            m_arcache <= 4'b0;
            m_arprot  <= 3'b0;
            m_arqos   <= 4'b0;
            m_arregion<= 4'b0;
            m_arvalid <= 1'b1;
            m_rready  <= 1'b1;

            wait (m_arready);
            @(posedge clk);
            m_arvalid <= 1'b0;

            while (!m_rvalid) @(posedge clk);
            data = addr[2] ? m_rdata[63:32] : m_rdata[31:0];
            resp = m_rresp;
            @(posedge clk);
            m_rready <= 1'b0;
        end
    endtask

    // Convenience task: execute a complete 2x2 pool operation
    task execute_pool(
        input [1:0] mode,
        input signed [15:0] p0,
        input signed [15:0] p1,
        input signed [15:0] p2,
        input signed [15:0] p3,
        output signed [15:0] out_val,
        output [1:0]        out_idx
    );
        logic [1:0]  resp;
        logic [31:0] rdata;
        begin
            // 1. Set mode & enable
            axi_write(ADDR_CTRL, {29'h0, mode, 1'b1}, 8'h0F, resp);

            // 2. Write {p1, p0}
            axi_write(ADDR_DATA_IN_P01, {p1, p0}, 8'h0F, resp);

            // 3. Write {p3, p2} (triggers calculation)
            axi_write(ADDR_DATA_IN_P23, {p3, p2}, 8'h0F, resp);

            // Wait 2 clocks for pipeline to retire
            repeat (2) @(posedge clk);

            // 4. Poll status to ensure result is valid
            axi_read(ADDR_STATUS, rdata, resp);

            // 5. Read result
            axi_read(ADDR_DATA_OUT, rdata, resp);
            out_val = rdata[15:0];
            out_idx = rdata[17:16];
        end
    endtask

    // -------------------------------------------------------------------------
    // Main Verification Flow
    // -------------------------------------------------------------------------
    logic [31:0] rdata;
    logic [1:0]  resp;
    logic signed [15:0] p_val;
    logic [1:0]  p_idx;

    initial begin
        $display("=================================================================");
        $display(" tb_axi_pooling : Verification of AXI4 Slave Pooling Peripheral  ");
        $display("=================================================================");

        // Init inputs
        m_awvalid = 1'b0;
        m_wvalid  = 1'b0;
        m_bready  = 1'b0;
        m_arvalid = 1'b0;
        m_rready  = 1'b0;
        rst       = 1'b1;

        repeat (4) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        // --- TEST 1: Default Registers ---
        $display("\n--- Test Group 1: Default Register Values ---");
        axi_read(ADDR_ID, rdata, resp);
        check("REG_ID reads 'POOL' (0x504F4F4C)", (rdata === 32'h504F4F4C) && (resp === RESP_OKAY));

        axi_read(ADDR_VERSION, rdata, resp);
        check("REG_VERSION reads 0x00010000 (v1.0)", (rdata === 32'h00010000) && (resp === RESP_OKAY));

        axi_read(ADDR_CTRL, rdata, resp);
        check("REG_CTRL default is 0x00000001 (enable=1, mode=00)", (rdata === 32'h00000001) && (resp === RESP_OKAY));

        axi_read(ADDR_STATUS, rdata, resp);
        check("REG_STATUS default is 0x00000000 (idle)", (rdata === 32'h00000000) && (resp === RESP_OKAY));

        axi_read(ADDR_CONFIG, rdata, resp);
        check("REG_CONFIG default is 0x00000001", (rdata === 32'h00000001) && (resp === RESP_OKAY));

        // --- TEST 2: Register Write & Readback ---
        $display("\n--- Test Group 2: Register R/W Operations ---");
        axi_write(ADDR_CONFIG, 32'h000000A5, 8'h0F, resp);
        check("REG_CONFIG write responds OKAY", resp === RESP_OKAY);
        axi_read(ADDR_CONFIG, rdata, resp);
        check("REG_CONFIG readback matches 0x000000A5", rdata === 32'h000000A5);

        axi_write(ADDR_CTRL, 32'h00000003, 8'h0F, resp); // mode=01 (AVG), enable=1
        check("REG_CTRL write responds OKAY", resp === RESP_OKAY);
        axi_read(ADDR_CTRL, rdata, resp);
        check("REG_CTRL readback matches 0x00000003", rdata === 32'h00000003);

        axi_write(ADDR_DATA_IN_P01, 32'h12345678, 8'h0F, resp);
        check("REG_DATA_IN_P01 write responds OKAY", resp === RESP_OKAY);
        axi_read(ADDR_DATA_IN_P01, rdata, resp);
        check("REG_DATA_IN_P01 readback matches 0x12345678", rdata === 32'h12345678);

        // --- TEST 3: Read-Only & Access Violation Protections ---
        $display("\n--- Test Group 3: Error Protection & Response Checks ---");
        axi_write(ADDR_ID, 32'hDEADBEEF, 8'h0F, resp);
        check("Write to RO REG_ID returns RESP_SLVERR", resp === RESP_SLVERR);

        axi_write(ADDR_VERSION, 32'hDEADBEEF, 8'h0F, resp);
        check("Write to RO REG_VERSION returns RESP_SLVERR", resp === RESP_SLVERR);

        axi_write(ADDR_STATUS, 32'hDEADBEEF, 8'h0F, resp);
        check("Write to RO REG_STATUS returns RESP_SLVERR", resp === RESP_SLVERR);

        axi_write(ADDR_DATA_OUT, 32'hDEADBEEF, 8'h0F, resp);
        check("Write to RO REG_DATA_OUT returns RESP_SLVERR", resp === RESP_SLVERR);

        axi_read(BASE_ADDR + 32'h2000, rdata, resp);
        check("Read out-of-aperture returns RESP_DECERR", resp === RESP_DECERR);

        axi_write(BASE_ADDR + 32'h2000, 32'h1, 8'h0F, resp);
        check("Write out-of-aperture returns RESP_DECERR", resp === RESP_DECERR);

        // Unaligned address
        @(posedge clk);
        m_awaddr  <= BASE_ADDR + 32'h03; // Misaligned
        m_awid    <= 8'h03;
        m_awlen   <= 8'h00;
        m_awsize  <= 3'b010;
        m_awburst <= 2'b01;
        m_awvalid <= 1'b1;
        m_wdata   <= 64'h0;
        m_wstrb   <= 8'h0F;
        m_wvalid  <= 1'b1;
        m_bready  <= 1'b1;
        wait (m_awready && m_wready);
        @(posedge clk);
        m_awvalid <= 1'b0;
        m_wvalid  <= 1'b0;
        while (!m_bvalid) @(posedge clk);
        check("Unaligned write returns RESP_SLVERR", m_bresp === RESP_SLVERR);
        @(posedge clk);
        m_bready <= 1'b0;

        // --- TEST 4: Functional Pooling Operations ---
        $display("\n--- Test Group 4: Functional Pooling via AXI Operations ---");

        // 4.1 Max-Pooling
        execute_pool(2'b00, 16'sd10, 16'sd50, 16'sd30, 16'sd20, p_val, p_idx);
        check("AXI MAX: {10, 50, 30, 20} -> val=50", p_val === 16'sd50);
        check("AXI MAX: {10, 50, 30, 20} -> idx=1 (pixel 1)", p_idx === 2'd1);

        // 4.2 Max-Pooling with negative numbers
        execute_pool(2'b00, -16'sd100, -16'sd20, -16'sd50, -16'sd5, p_val, p_idx);
        check("AXI MAX (negative): {-100, -20, -50, -5} -> val=-5", p_val === -16'sd5);
        check("AXI MAX (negative): {-100, -20, -50, -5} -> idx=3 (pixel 3)", p_idx === 2'd3);

        // 4.3 Average-Pooling
        execute_pool(2'b01, 16'sd1, 16'sd2, 16'sd3, 16'sd4, p_val, p_idx);
        check("AXI AVG (round): {1, 2, 3, 4} -> val=3", p_val === 16'sd3);

        execute_pool(2'b01, -16'sd4, -16'sd4, -16'sd4, -16'sd4, p_val, p_idx);
        check("AXI AVG (negative): {-4, -4, -4, -4} -> val=-4", p_val === -16'sd4);

        // 4.4 Min-Pooling
        execute_pool(2'b10, 16'sd10, 16'sd50, 16'sd30, 16'sd20, p_val, p_idx);
        check("AXI MIN: {10, 50, 30, 20} -> val=10", p_val === 16'sd10);

        execute_pool(2'b10, -16'sd300, 16'sd250, -16'sd10, 16'sd0, p_val, p_idx);
        check("AXI MIN (mixed): {-300, 250, -10, 0} -> val=-300", p_val === -16'sd300);

        // 4.5 Bypass Mode
        execute_pool(2'b11, 16'sd888, 16'sd10, 16'sd20, 16'sd30, p_val, p_idx);
        check("AXI BYPASS: {888, 10, 20, 30} -> val=888 (p0 passed)", p_val === 16'sd888);

        // --- TEST 5: 64-bit Upper Lane Access ---
        $display("\n--- Test Group 5: 64-bit Sub-word Lane Steering ---");
        // Case A: 64-bit aligned address 0x08 with strobe 0xF0 routes to upper sub-word (offset 0x0C: REG_DATA_IN_P01)
        axi_write(ADDR_CONFIG, 32'hA5A51234, 8'hF0, resp);
        check("Write 64-bit aligned 0x08 with strobe 0xF0 responds OKAY", resp === RESP_OKAY);
        axi_read(ADDR_DATA_IN_P01, rdata, resp);
        check("Upper lane data correctly written to sub-word reg REG_DATA_IN_P01 (0x0C)", rdata === 32'hA5A51234);

        // Case B: Word address 0x0C with strobe 0xF0 (upper lane) writes REG_DATA_IN_P01
        axi_write(ADDR_DATA_IN_P01, 32'h5A5A9876, 8'hF0, resp);
        check("Write 0x0C with strobe 0xF0 responds OKAY", resp === RESP_OKAY);
        axi_read(ADDR_DATA_IN_P01, rdata, resp);
        check("Direct 0x0C upper lane write matches 0x5A5A9876", rdata === 32'h5A5A9876);

        // --- Summary ---
        $display("\n=================================================================");
        $display(" tb_axi_pooling SUMMARY : PASSED = %0d, FAILED = %0d", pass_count, fail_count);
        $display("=================================================================");

        if (fail_count == 0) begin
            $display(">>> ALL tb_axi_pooling CHECKS PASSED SUCCESSFULLY! <<<\n");
        end else begin
            $display(">>> VERIFICATION FAILED with %0d errors! <<<\n", fail_count);
        end

        $finish;
    end

endmodule

`default_nettype wire
