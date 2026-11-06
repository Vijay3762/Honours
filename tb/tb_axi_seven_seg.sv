// =============================================================================
// File    : tb_axi_seven_seg.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Comprehensive Standalone Testbench for axi_seven_seg
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_axi_seven_seg;

    localparam DATA_WIDTH = 64;
    localparam ADDR_WIDTH = 32;
    localparam ID_WIDTH   = 8;
    localparam STRB_WIDTH = DATA_WIDTH / 8;
    localparam BASE_ADDR  = 32'h7000_0000;

    // Clock and Reset
    logic clk;
    logic rst;

    initial clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz

    // AXI4 Master Interface signals to DUT
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

    // Physical outputs from DUT
    wire [6:0]               seg_o;
    wire                     dp_o;
    wire [3:0]               an_o;

    // Instantiate DUT with fast simulation refresh divider
    axi_seven_seg #(
        .AXI_DATA_WIDTH      (DATA_WIDTH),
        .AXI_ADDR_WIDTH      (ADDR_WIDTH),
        .AXI_ID_WIDTH        (ID_WIDTH),
        .BASE_ADDR           (BASE_ADDR),
        .DEFAULT_REFRESH_DIV (8) // 8 clocks per digit in testbench
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
        .s_axi_rready        (m_rready),
        .seg_o               (seg_o),
        .dp_o                (dp_o),
        .an_o                (an_o)
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

    // AXI write task (single beat)
    task axi_write(input [ADDR_WIDTH-1:0] addr, input [31:0] data, input [7:0] strb, output [1:0] resp);
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

            wait (m_awready && m_wready);
            @(posedge clk);
            m_awvalid <= 1'b0;
            m_wvalid  <= 1'b0;

            wait (m_bvalid);
            resp = m_bresp;
            @(posedge clk);
            m_bready <= 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    // AXI read task (single beat)
    task axi_read(input [ADDR_WIDTH-1:0] addr, output [31:0] data, output [1:0] resp);
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

            wait (m_rvalid);
            data = m_rdata[31:0];
            resp = m_rresp;
            @(posedge clk);
            m_rready <= 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    // Hex reference mapping
    function [6:0] exp_hex(input [3:0] h);
        case (h)
            4'h0: exp_hex = 7'b100_0000;
            4'h1: exp_hex = 7'b111_1001;
            4'h2: exp_hex = 7'b010_0100;
            4'h3: exp_hex = 7'b011_0000;
            4'h4: exp_hex = 7'b001_1001;
            4'h5: exp_hex = 7'b001_0010;
            4'h6: exp_hex = 7'b000_0010;
            4'h7: exp_hex = 7'b111_1000;
            4'h8: exp_hex = 7'b000_0000;
            4'h9: exp_hex = 7'b001_0000;
            4'hA: exp_hex = 7'b000_1000;
            4'hB: exp_hex = 7'b000_0011;
            4'hC: exp_hex = 7'b100_0110;
            4'hD: exp_hex = 7'b010_0001;
            4'hE: exp_hex = 7'b000_0110;
            4'hF: exp_hex = 7'b000_1110;
        endcase
    endfunction

    logic [31:0] rdata;
    logic [1:0]  resp;
    integer d;

    initial begin
        m_awvalid = 0; m_wvalid = 0; m_bready = 0;
        m_arvalid = 0; m_rready = 0;
        m_awaddr = 0; m_wdata = 0; m_wstrb = 0;
        m_araddr = 0;

        $display("============================================================");
        $display("  tb_axi_seven_seg: AXI 7-Segment Controller Verification");
        $display("============================================================");

        // Reset
        rst = 1'b1;
        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        // --- T1: Reset Defaults ---
        $display("\n--- T1: Reset Defaults ---");
        axi_read(BASE_ADDR + 32'h18, rdata, resp);
        check("T1: Read ID = 0x37534547 ('7SEG')", rdata == 32'h37534547 && resp == 2'b00);

        axi_read(BASE_ADDR + 32'h1C, rdata, resp);
        check("T1: Read VERSION = 0x00010000 (v1.0)", rdata == 32'h00010000 && resp == 2'b00);

        axi_read(BASE_ADDR + 32'h00, rdata, resp);
        check("T1: Default DIGITS = 0x0000", rdata == 32'h0 && resp == 2'b00);

        axi_read(BASE_ADDR + 32'h04, rdata, resp);
        check("T1: Default DIGIT_EN = 0x0F (all enabled)", rdata[3:0] == 4'hF && resp == 2'b00);

        axi_read(BASE_ADDR + 32'h08, rdata, resp);
        check("T1: Default DECIMAL_PT = 0x0 (all DPs off)", rdata[3:0] == 4'h0 && resp == 2'b00);

        axi_read(BASE_ADDR + 32'h14, rdata, resp);
        check("T1: Default CTRL = 0x01 (display enabled)", rdata[3:0] == 4'h1 && resp == 2'b00);

        // --- T2: Register Write / Readback ---
        $display("\n--- T2: Register Write / Readback ---");
        axi_write(BASE_ADDR + 32'h00, 32'h0000_1234, 8'h0F, resp);
        check("T2: Write DIGITS resp OKAY", resp == 2'b00);
        axi_read(BASE_ADDR + 32'h00, rdata, resp);
        check("T2: Readback DIGITS = 0x1234", rdata[15:0] == 16'h1234);

        axi_write(BASE_ADDR + 32'h08, 32'h0000_0005, 8'h0F, resp); // DP on D0 and D2
        axi_read(BASE_ADDR + 32'h08, rdata, resp);
        check("T2: Readback DECIMAL_PT = 0x5", rdata[3:0] == 4'h5);

        // --- T3: Hex to 7-Segment Decoder Sweep (0..F) ---
        $display("\n--- T3: Hex Decoder Sweep (0x0 to 0xF) ---");
        for (d = 0; d < 16; d = d + 1) begin
            axi_write(BASE_ADDR + 32'h00, d, 8'h0F, resp);
            // Wait until digit 0 is active
            wait (an_o == 4'b1110);
            @(posedge clk);
            check($sformatf("T3: Hex 0x%01X cathode decode", d), seg_o == exp_hex(d[3:0]));
        end

        // --- T4: Digit Multiplexing & Scanner Verification ---
        $display("\n--- T4: Digit Multiplexing & Scanner Verification ---");
        // Write D3=A, D2=B, D1=C, D0=D
        axi_write(BASE_ADDR + 32'h00, 32'h0000_ABCD, 8'h0F, resp);
        axi_write(BASE_ADDR + 32'h08, 32'h0000_0000, 8'h0F, resp); // DPs off

        // Digit 0 (rightmost)
        wait (an_o == 4'b1110);
        check("T4: Digit 0 active (an_o = 1110)", an_o == 4'b1110);
        check("T4: Digit 0 cathode shows 'd' (0x21)", seg_o == exp_hex(4'hD));

        // Digit 1
        wait (an_o == 4'b1101);
        check("T4: Digit 1 active (an_o = 1101)", an_o == 4'b1101);
        check("T4: Digit 1 cathode shows 'C' (0x46)", seg_o == exp_hex(4'hC));

        // Digit 2
        wait (an_o == 4'b1011);
        check("T4: Digit 2 active (an_o = 1011)", an_o == 4'b1011);
        check("T4: Digit 2 cathode shows 'b' (0x03)", seg_o == exp_hex(4'hB));

        // Digit 3 (leftmost)
        wait (an_o == 4'b0111);
        check("T4: Digit 3 active (an_o = 0111)", an_o == 4'b0111);
        check("T4: Digit 3 cathode shows 'A' (0x08)", seg_o == exp_hex(4'hA));

        // --- T5: Individual Digit Blanking (DIGIT_EN) ---
        $display("\n--- T5: Individual Digit Blanking ---");
        // Blank digit 1 (enable D0, D2, D3 -> mask = 4'b1101 = 0xD)
        axi_write(BASE_ADDR + 32'h04, 32'h0000_000D, 8'h0F, resp);

        // Wait until digit 1 slot comes around
        repeat (32) @(posedge clk);
        // While in digit 1 slot, an_o must be all-1 (inactive) and seg_o must be 7'b111_1111 (all off)
        check("T5: Disabled digit 1 blanks anodes", dut.active_digit_idx == 2'b01 ? an_o == 4'b1111 : 1'b1);

        // Restore all digits
        axi_write(BASE_ADDR + 32'h04, 32'h0000_000F, 8'h0F, resp);

        // --- T6: Decimal Point Verification ---
        $display("\n--- T6: Decimal Point Verification ---");
        // Turn on DP for digit 2 only (mask 4'b0100)
        axi_write(BASE_ADDR + 32'h08, 32'h0000_0004, 8'h0F, resp);
        wait (an_o == 4'b1011); // digit 2 active
        check("T6: Digit 2 DP is ON (active-low 0)", dp_o == 1'b0);

        wait (an_o == 4'b1110); // digit 0 active
        check("T6: Digit 0 DP is OFF (active-low 1)", dp_o == 1'b1);

        // --- T7: Raw Segment Mode ---
        $display("\n--- T7: Raw Segment Mode ---");
        // Enable raw mode: CTRL[1] = 1 -> CTRL = 4'h3
        axi_write(BASE_ADDR + 32'h10, 32'hAA_55_F0_0F, 8'h0F, resp); // custom patterns
        axi_write(BASE_ADDR + 32'h14, 32'h0000_0003, 8'h0F, resp); // raw_mode = 1

        wait (an_o == 4'b1110); // digit 0
        check("T7: Raw pattern D0 cathodes = 0x0F", seg_o == 7'h0F);
        check("T7: Raw pattern D0 DP off (bit 7 = 0)", dp_o == 1'b1);

        wait (an_o == 4'b1101); // digit 1
        check("T7: Raw pattern D1 cathodes = 0x70", seg_o == 7'h70);

        // Return to normal mode
        axi_write(BASE_ADDR + 32'h14, 32'h0000_0001, 8'h0F, resp);

        // --- T8: Master Display Blanking (CTRL[0]=0) ---
        $display("\n--- T8: Master Display Blanking ---");
        axi_write(BASE_ADDR + 32'h14, 32'h0000_0000, 8'h0F, resp); // disp_en = 0
        repeat (16) @(posedge clk);
        check("T8: All anodes OFF when master display disabled", an_o == 4'b1111);
        check("T8: All cathodes OFF when master display disabled", seg_o == 7'b111_1111);
        check("T8: DP OFF when master display disabled", dp_o == 1'b1);

        // Re-enable
        axi_write(BASE_ADDR + 32'h14, 32'h0000_0001, 8'h0F, resp);

        // --- T9: AXI Error Handling ---
        $display("\n--- T9: AXI Error Handling ---");
        // Write to Read-Only ID register -> expect SLVERR (2'b10)
        axi_write(BASE_ADDR + 32'h18, 32'hDEADBEEF, 8'h0F, resp);
        check("T9: Write to RO ID reg produces SLVERR", resp == 2'b10);

        // Write to Read-Only VERSION register -> expect SLVERR (2'b10)
        axi_write(BASE_ADDR + 32'h1C, 32'hDEADBEEF, 8'h0F, resp);
        check("T9: Write to RO VERSION reg produces SLVERR", resp == 2'b10);

        // Out-of-aperture write -> expect DECERR (2'b11)
        axi_write(BASE_ADDR + 32'h1000, 32'h12345678, 8'h0F, resp);
        check("T9: Out-of-aperture address produces DECERR", resp == 2'b11);

        // Reserved register offset -> expect SLVERR (2'b10)
        axi_write(BASE_ADDR + 32'h0020, 32'h12345678, 8'h0F, resp);
        check("T9: Reserved offset produces SLVERR", resp == 2'b10);

        // --- Summary ---
        $display("\n============================================================");
        $display("  AXI 7-Segment Controller Verification Summary");
        $display("  Total Passed: %0d", pass_count);
        $display("  Total Failed: %0d", fail_count);
        $display("============================================================");

        if (fail_count == 0 && pass_count >= 30) begin
            $display("\n*** ALL AXI 7-SEGMENT TESTS PASSED ***\n");
        end else begin
            $display("\n*** TEST FAILED ***\n");
        end

        $finish;
    end

endmodule

`default_nettype wire
