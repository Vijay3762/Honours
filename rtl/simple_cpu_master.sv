// ============================================================================
// File    : simple_cpu_master.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Simple CPU-like AXI4 master model for UART and GPIO testing
//           Supports:
//             TEST_MODE = "UART"  : UART read/write loopback test
//             TEST_MODE = "GPIO"  : GPIO full register/action/pin test
//             TEST_MODE = "ALL"   : UART test followed by GPIO test
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module simple_cpu_master #(
    parameter DATA_WIDTH = 64,
    parameter ADDR_WIDTH = 32,
    parameter STRB_WIDTH = DATA_WIDTH/8,
    parameter ID_WIDTH   = 8,
    parameter TEST_MODE  = "UART"
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

    // UART register map (base 0x4000_0000)
    localparam [ADDR_WIDTH-1:0] UART_BASE       = 32'h4000_0000;
    localparam [ADDR_WIDTH-1:0] UART_THR        = UART_BASE + 32'h00;
    localparam [ADDR_WIDTH-1:0] UART_BAUD       = UART_BASE + 32'h08;
    localparam [ADDR_WIDTH-1:0] UART_LCR        = UART_BASE + 32'h0C;
    localparam [ADDR_WIDTH-1:0] UART_LSR        = UART_BASE + 32'h14;

    // GPIO register map (base 0x6000_0000)
    localparam [ADDR_WIDTH-1:0] GPIO_BASE       = 32'h6000_0000;
    localparam [ADDR_WIDTH-1:0] GPIO_IN         = GPIO_BASE + 32'h00;
    localparam [ADDR_WIDTH-1:0] GPIO_OUT        = GPIO_BASE + 32'h04;
    localparam [ADDR_WIDTH-1:0] GPIO_DIR        = GPIO_BASE + 32'h08;
    localparam [ADDR_WIDTH-1:0] GPIO_OUT_SET    = GPIO_BASE + 32'h0C;
    localparam [ADDR_WIDTH-1:0] GPIO_OUT_CLR    = GPIO_BASE + 32'h10;
    localparam [ADDR_WIDTH-1:0] GPIO_OUT_TOGGLE = GPIO_BASE + 32'h14;
    localparam [ADDR_WIDTH-1:0] GPIO_ID         = GPIO_BASE + 32'h18;
    localparam [ADDR_WIDTH-1:0] GPIO_VERSION    = GPIO_BASE + 32'h1C;

    // Timer register map (base 0x5000_0000)
    localparam [ADDR_WIDTH-1:0] TIMER_BASE_REG   = 32'h5000_0000;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_CTRL   = TIMER_BASE_REG + 32'h00;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_LOAD   = TIMER_BASE_REG + 32'h04;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_VAL    = TIMER_BASE_REG + 32'h08;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_CMP    = TIMER_BASE_REG + 32'h0C;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_STAT   = TIMER_BASE_REG + 32'h10;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_ID     = TIMER_BASE_REG + 32'h14;
    localparam [ADDR_WIDTH-1:0] CPU_TIMER_VER    = TIMER_BASE_REG + 32'h18;

    typedef enum logic [5:0] {
        ST_IDLE,

        // UART Write transactions
        ST_UART_W1_DRIVE_WAIT_B,
        ST_UART_W2_DRIVE_WAIT_B,
        ST_UART_W3_DRIVE_WAIT_B,
        ST_UART_W4_DRIVE_WAIT_B,
        ST_UART_R1_AR, ST_UART_R1_R,
        ST_UART_R2_AR, ST_UART_R2_R,
        ST_UART_W5_DRIVE_WAIT_B,

        // GPIO transactions
        ST_GPIO_R_ID_AR,    ST_GPIO_R_ID_R,
        ST_GPIO_R_VER_AR,   ST_GPIO_R_VER_R,
        ST_GPIO_R_OUT0_AR,  ST_GPIO_R_OUT0_R,
        ST_GPIO_R_DIR0_AR,  ST_GPIO_R_DIR0_R,
        ST_GPIO_W_DIR_WAIT_B,
        ST_GPIO_R_DIR_AR,   ST_GPIO_R_DIR_R,
        ST_GPIO_W_OUT_WAIT_B,
        ST_GPIO_R_OUT_AR,   ST_GPIO_R_OUT_R,
        ST_GPIO_W_SET_WAIT_B,
        ST_GPIO_R_SET_AR,   ST_GPIO_R_SET_R,
        ST_GPIO_W_CLR_WAIT_B,
        ST_GPIO_R_CLR_AR,   ST_GPIO_R_CLR_R,
        ST_GPIO_W_TOG_WAIT_B,
        ST_GPIO_R_TOG_AR,   ST_GPIO_R_TOG_R,
        ST_GPIO_R_IN_AR,    ST_GPIO_R_IN_R,

        // Timer transactions
        ST_TIMER_R_ID_AR,       ST_TIMER_R_ID_R,
        ST_TIMER_R_VER_AR,      ST_TIMER_R_VER_R,
        ST_TIMER_W_LOAD_WAIT_B,
        ST_TIMER_R_LOAD_AR,     ST_TIMER_R_LOAD_R,
        ST_TIMER_W_CMP_WAIT_B,
        ST_TIMER_R_CMP_AR,      ST_TIMER_R_CMP_R,
        ST_TIMER_W_CTRL_WAIT_B,
        ST_TIMER_WAIT_COUNT,
        ST_TIMER_R_VAL_AR,      ST_TIMER_R_VAL_R,
        ST_TIMER_R_STAT_AR,     ST_TIMER_R_STAT_R,
        ST_TIMER_W_STAT_CLR_WAIT_B,
        ST_TIMER_R_STAT_CLR_AR, ST_TIMER_R_STAT_CLR_R,

        ST_DONE
    } state_t;

    state_t state, next_state;

    // Captured UART read data
    reg [31:0] rx_data_lsr, rx_data_rbr;

    // Captured GPIO read data
    reg [31:0] rx_gpio_id, rx_gpio_ver;
    reg [31:0] rx_gpio_out_init, rx_gpio_dir_init;
    reg [31:0] rx_gpio_dir, rx_gpio_out_wr;
    reg [31:0] rx_gpio_out_set, rx_gpio_out_clr, rx_gpio_out_tog;
    reg [31:0] rx_gpio_in;

    // Captured Timer read data
    reg [31:0] rx_timer_id, rx_timer_ver;
    reg [31:0] rx_timer_load, rx_timer_cmp;
    reg [31:0] rx_timer_val, rx_timer_stat_match, rx_timer_stat_cleared;
    reg [7:0]  timer_wait_cnt;

    // --------------------------------------------------------------
    // Sequential: state + captured read data
    // --------------------------------------------------------------
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            state                 <= ST_IDLE;
            rx_data_lsr           <= '0;
            rx_data_rbr           <= '0;
            rx_gpio_id            <= '0;
            rx_gpio_ver           <= '0;
            rx_gpio_out_init      <= '0;
            rx_gpio_dir_init      <= '0;
            rx_gpio_dir           <= '0;
            rx_gpio_out_wr        <= '0;
            rx_gpio_out_set       <= '0;
            rx_gpio_out_clr       <= '0;
            rx_gpio_out_tog       <= '0;
            rx_gpio_in            <= '0;
            rx_timer_id           <= '0;
            rx_timer_ver          <= '0;
            rx_timer_load         <= '0;
            rx_timer_cmp          <= '0;
            rx_timer_val          <= '0;
            rx_timer_stat_match   <= '0;
            rx_timer_stat_cleared <= '0;
            timer_wait_cnt        <= '0;
        end else begin
            state <= next_state;

            if (state == ST_TIMER_WAIT_COUNT)
                timer_wait_cnt <= timer_wait_cnt + 1'b1;
            else
                timer_wait_cnt <= '0;

            if (m_axi_rvalid && m_axi_rready) begin
                case (m_axi_rid)
                    8'h10: rx_data_lsr           <= m_axi_rdata[31:0];
                    8'h20: rx_data_rbr           <= m_axi_rdata[31:0];
                    8'h30: rx_gpio_id            <= m_axi_rdata[31:0];
                    8'h31: rx_gpio_ver           <= m_axi_rdata[31:0];
                    8'h32: rx_gpio_out_init      <= m_axi_rdata[31:0];
                    8'h33: rx_gpio_dir_init      <= m_axi_rdata[31:0];
                    8'h35: rx_gpio_dir           <= m_axi_rdata[31:0];
                    8'h37: rx_gpio_out_wr        <= m_axi_rdata[31:0];
                    8'h39: rx_gpio_out_set       <= m_axi_rdata[31:0];
                    8'h3B: rx_gpio_out_clr       <= m_axi_rdata[31:0];
                    8'h3D: rx_gpio_out_tog       <= m_axi_rdata[31:0];
                    8'h3E: rx_gpio_in            <= m_axi_rdata[31:0];
                    8'h50: rx_timer_id           <= m_axi_rdata[31:0];
                    8'h51: rx_timer_ver          <= m_axi_rdata[31:0];
                    8'h52: rx_timer_load         <= m_axi_rdata[31:0];
                    8'h53: rx_timer_cmp          <= m_axi_rdata[31:0];
                    8'h54: rx_timer_val          <= m_axi_rdata[31:0];
                    8'h55: rx_timer_stat_match   <= m_axi_rdata[31:0];
                    8'h56: rx_timer_stat_cleared <= m_axi_rdata[31:0];
                endcase
            end
        end
    end

    // --------------------------------------------------------------
    // Combinational: next state + AXI signal drive
    // --------------------------------------------------------------
    always @* begin
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
                if (TEST_MODE == "GPIO")
                    next_state = ST_GPIO_R_ID_AR;
                else if (TEST_MODE == "TIMER")
                    next_state = ST_TIMER_R_ID_AR;
                else
                    next_state = ST_UART_W1_DRIVE_WAIT_B;
            end

            // ==========================================================
            // UART Sequence
            // ==========================================================
            ST_UART_W1_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_LCR; m_axi_awid = 8'h01;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000083};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_UART_W2_DRIVE_WAIT_B;
            end

            ST_UART_W2_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_BAUD; m_axi_awid = 8'h02;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'd50};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_UART_W3_DRIVE_WAIT_B;
            end

            ST_UART_W3_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_LCR; m_axi_awid = 8'h03;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000003};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_UART_W4_DRIVE_WAIT_B;
            end

            ST_UART_W4_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_THR; m_axi_awid = 8'h04;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000041};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_UART_R1_AR;
            end

            ST_UART_R1_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = UART_LSR; m_axi_arid = 8'h10;
                if (m_axi_arready) next_state = ST_UART_R1_R;
            end
            ST_UART_R1_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_UART_R2_AR;
            end

            ST_UART_R2_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = UART_THR; m_axi_arid = 8'h20;
                if (m_axi_arready) next_state = ST_UART_R2_R;
            end
            ST_UART_R2_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_UART_W5_DRIVE_WAIT_B;
            end

            ST_UART_W5_DRIVE_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = UART_THR; m_axi_awid = 8'h05;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0, 32'h00000042};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) begin
                    if (TEST_MODE == "ALL")
                        next_state = ST_GPIO_R_ID_AR;
                    else
                        next_state = ST_DONE;
                end
            end

            // ==========================================================
            // GPIO Sequence
            // ==========================================================

            // 1) Read GPIO_ID (0x6000_0018) -> expect 0x4749504F ("GPIO")
            ST_GPIO_R_ID_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_ID; m_axi_arid = 8'h30;
                if (m_axi_arready) next_state = ST_GPIO_R_ID_R;
            end
            ST_GPIO_R_ID_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_R_VER_AR;
            end

            // 2) Read GPIO_VERSION (0x6000_001C) -> expect 0x00010000
            ST_GPIO_R_VER_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_VERSION; m_axi_arid = 8'h31;
                if (m_axi_arready) next_state = ST_GPIO_R_VER_R;
            end
            ST_GPIO_R_VER_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_R_OUT0_AR;
            end

            // 3) Read initial GPIO_OUT (0x6000_0004) -> expect 0
            ST_GPIO_R_OUT0_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_OUT; m_axi_arid = 8'h32;
                if (m_axi_arready) next_state = ST_GPIO_R_OUT0_R;
            end
            ST_GPIO_R_OUT0_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_R_DIR0_AR;
            end

            // 4) Read initial GPIO_DIR (0x6000_0008) -> expect 0
            ST_GPIO_R_DIR0_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_DIR; m_axi_arid = 8'h33;
                if (m_axi_arready) next_state = ST_GPIO_R_DIR0_R;
            end
            ST_GPIO_R_DIR0_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_W_DIR_WAIT_B;
            end

            // 5) Write GPIO_DIR = 0x00FF (pins [7:0] output, pins [15:8] input)
            ST_GPIO_W_DIR_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = GPIO_DIR; m_axi_awid = 8'h34;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_00FF, 32'h0000_00FF};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_GPIO_R_DIR_AR;
            end

            // 6) Readback GPIO_DIR -> expect 0x00FF
            ST_GPIO_R_DIR_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_DIR; m_axi_arid = 8'h35;
                if (m_axi_arready) next_state = ST_GPIO_R_DIR_R;
            end
            ST_GPIO_R_DIR_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_W_OUT_WAIT_B;
            end

            // 7) Write GPIO_OUT = 0x00A5
            ST_GPIO_W_OUT_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = GPIO_OUT; m_axi_awid = 8'h36;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_00A5, 32'h0000_00A5};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_GPIO_R_OUT_AR;
            end

            // 8) Readback GPIO_OUT -> expect 0x00A5
            ST_GPIO_R_OUT_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_OUT; m_axi_arid = 8'h37;
                if (m_axi_arready) next_state = ST_GPIO_R_OUT_R;
            end
            ST_GPIO_R_OUT_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_W_SET_WAIT_B;
            end

            // 9) Write GPIO_OUT_SET = 0x0050 (bits 4, 6: 0xA5 | 0x50 = 0xF5)
            ST_GPIO_W_SET_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = GPIO_OUT_SET; m_axi_awid = 8'h38;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_0050, 32'h0000_0050};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_GPIO_R_SET_AR;
            end

            // 10) Readback GPIO_OUT -> expect 0x00F5
            ST_GPIO_R_SET_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_OUT; m_axi_arid = 8'h39;
                if (m_axi_arready) next_state = ST_GPIO_R_SET_R;
            end
            ST_GPIO_R_SET_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_W_CLR_WAIT_B;
            end

            // 11) Write GPIO_OUT_CLR = 0x0005 (clear bits 0, 2: 0xF5 & ~0x05 = 0xF0)
            ST_GPIO_W_CLR_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = GPIO_OUT_CLR; m_axi_awid = 8'h3A;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_0005, 32'h0000_0005};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_GPIO_R_CLR_AR;
            end

            // 12) Readback GPIO_OUT -> expect 0x00F0
            ST_GPIO_R_CLR_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_OUT; m_axi_arid = 8'h3B;
                if (m_axi_arready) next_state = ST_GPIO_R_CLR_R;
            end
            ST_GPIO_R_CLR_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_W_TOG_WAIT_B;
            end

            // 13) Write GPIO_OUT_TOGGLE = 0x0033 (toggle bits 0,1,4,5: 0xF0 ^ 0x33 = 0xC3)
            ST_GPIO_W_TOG_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = GPIO_OUT_TOGGLE; m_axi_awid = 8'h3C;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_0033, 32'h0000_0033};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_GPIO_R_TOG_AR;
            end

            // 14) Readback GPIO_OUT -> expect 0x00C3
            ST_GPIO_R_TOG_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_OUT; m_axi_arid = 8'h3D;
                if (m_axi_arready) next_state = ST_GPIO_R_TOG_R;
            end
            ST_GPIO_R_TOG_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_GPIO_R_IN_AR;
            end

            // 15) Read GPIO_IN (0x6000_0000)
            ST_GPIO_R_IN_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = GPIO_IN; m_axi_arid = 8'h3E;
                if (m_axi_arready) next_state = ST_GPIO_R_IN_R;
            end
            ST_GPIO_R_IN_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_DONE;
            end

            // ==========================================================
            // Timer Sequence (0x5000_0000)
            // ==========================================================
            // 1) Read TIMER_ID (0x5000_0014)
            ST_TIMER_R_ID_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_ID; m_axi_arid = 8'h50;
                if (m_axi_arready) next_state = ST_TIMER_R_ID_R;
            end
            ST_TIMER_R_ID_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_TIMER_R_VER_AR;
            end

            // 2) Read TIMER_VERSION (0x5000_0018)
            ST_TIMER_R_VER_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_VER; m_axi_arid = 8'h51;
                if (m_axi_arready) next_state = ST_TIMER_R_VER_R;
            end
            ST_TIMER_R_VER_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_TIMER_W_LOAD_WAIT_B;
            end

            // 3) Write TIMER_LOAD = 30 (0x5000_0004)
            ST_TIMER_W_LOAD_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = CPU_TIMER_LOAD; m_axi_awid = 8'h52;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'd30, 32'd30};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_TIMER_R_LOAD_AR;
            end

            // 4) Readback TIMER_LOAD
            ST_TIMER_R_LOAD_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_LOAD; m_axi_arid = 8'h52;
                if (m_axi_arready) next_state = ST_TIMER_R_LOAD_R;
            end
            ST_TIMER_R_LOAD_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_TIMER_W_CMP_WAIT_B;
            end

            // 5) Write TIMER_COMPARE = 15 (0x5000_000C)
            ST_TIMER_W_CMP_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = CPU_TIMER_CMP; m_axi_awid = 8'h53;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'd15, 32'd15};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_TIMER_R_CMP_AR;
            end

            // 6) Readback TIMER_COMPARE
            ST_TIMER_R_CMP_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_CMP; m_axi_arid = 8'h53;
                if (m_axi_arready) next_state = ST_TIMER_R_CMP_R;
            end
            ST_TIMER_R_CMP_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_TIMER_W_CTRL_WAIT_B;
            end

            // 7) Write TIMER_CTRL = 5 (enable=1, irq_en=1, one-shot) (0x5000_0000)
            ST_TIMER_W_CTRL_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = CPU_TIMER_CTRL; m_axi_awid = 8'h54;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_0005, 32'h0000_0005};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_TIMER_WAIT_COUNT;
            end

            // 8) Wait for counter to count past 15 (40 cycles)
            ST_TIMER_WAIT_COUNT: begin
                if (timer_wait_cnt >= 8'd40)
                    next_state = ST_TIMER_R_VAL_AR;
            end

            // 9) Read TIMER_VALUE (0x5000_0008)
            ST_TIMER_R_VAL_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_VAL; m_axi_arid = 8'h54;
                if (m_axi_arready) next_state = ST_TIMER_R_VAL_R;
            end
            ST_TIMER_R_VAL_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_TIMER_R_STAT_AR;
            end

            // 10) Read TIMER_STATUS (0x5000_0010) - expect match flag set
            ST_TIMER_R_STAT_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_STAT; m_axi_arid = 8'h55;
                if (m_axi_arready) next_state = ST_TIMER_R_STAT_R;
            end
            ST_TIMER_R_STAT_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_TIMER_W_STAT_CLR_WAIT_B;
            end

            // 11) Write TIMER_STATUS = 1 (clear match flag via W1C)
            ST_TIMER_W_STAT_CLR_WAIT_B: begin
                m_axi_awvalid = 1'b1; m_axi_awaddr = CPU_TIMER_STAT; m_axi_awid = 8'h56;
                m_axi_wvalid  = 1'b1; m_axi_wdata = {32'h0000_0001, 32'h0000_0001};
                m_axi_wstrb   = 8'h0F; m_axi_wlast = 1'b1;
                if (m_axi_bvalid) next_state = ST_TIMER_R_STAT_CLR_AR;
            end

            // 12) Read back TIMER_STATUS - expect match flag cleared
            ST_TIMER_R_STAT_CLR_AR: begin
                m_axi_arvalid = 1'b1; m_axi_araddr = CPU_TIMER_STAT; m_axi_arid = 8'h56;
                if (m_axi_arready) next_state = ST_TIMER_R_STAT_CLR_R;
            end
            ST_TIMER_R_STAT_CLR_R: begin
                m_axi_rready = 1'b1;
                if (m_axi_rvalid) next_state = ST_DONE;
            end

            // ==========================================================
            // Done
            // ==========================================================
            ST_DONE: begin
                cpu_done = 1'b1;
                if (TEST_MODE == "GPIO") begin
                    cpu_pass = (rx_gpio_id       == 32'h4749504F) &&
                               (rx_gpio_ver      == 32'h00010000) &&
                               (rx_gpio_out_init == 32'h00000000) &&
                               (rx_gpio_dir_init == 32'h00000000) &&
                               (rx_gpio_dir      == 32'h000000FF) &&
                               (rx_gpio_out_wr   == 32'h000000A5) &&
                               (rx_gpio_out_set  == 32'h000000F5) &&
                               (rx_gpio_out_clr  == 32'h000000F0) &&
                               (rx_gpio_out_tog  == 32'h000000C3) &&
                               (rx_gpio_in[15:8] == 8'h5A);
                end else if (TEST_MODE == "TIMER") begin
                    cpu_pass = (rx_timer_id            == 32'h54494D52) &&
                               (rx_timer_ver           == 32'h00010000) &&
                               (rx_timer_load          == 32'd30) &&
                               (rx_timer_cmp           == 32'd15) &&
                               (rx_timer_val           < 32'd30) &&
                               (rx_timer_stat_match[0] == 1'b1) &&
                               (rx_timer_stat_cleared[0] == 1'b0);
                end else if (TEST_MODE == "ALL") begin
                    cpu_pass = (rx_data_rbr[7:0] == 8'h41) &&
                               (rx_gpio_id       == 32'h4749504F) &&
                               (rx_gpio_ver      == 32'h00010000) &&
                               (rx_gpio_dir      == 32'h000000FF) &&
                               (rx_gpio_out_tog  == 32'h000000C3) &&
                               (rx_timer_id      == 32'h54494D52) &&
                               (rx_timer_stat_match[0] == 1'b1);
                end else begin // "UART"
                    cpu_pass = (rx_data_rbr[7:0] == 8'h41);
                end
            end

            default: next_state = ST_IDLE;
        endcase
    end

endmodule

`default_nettype wire
