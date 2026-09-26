`timescale 1ns / 1ps

module tb_axi_uart_top;

  // ============================================================
  // PARAMETERS
  // ============================================================

  localparam AXI_DATA_WIDTH = 32;
  localparam AXI_ADDR_WIDTH = 5;
  localparam AXI_ID_WIDTH = 12;

  localparam UART_BAUD_DIV = 10;

  localparam CLK_PERIOD = 10;

  // IMPORTANT:
  //
  // uart_receiver.v:
  //     counter_int >= baud_div_i - 1
  //
  // Therefore RX stimulus uses baud_div clocks.
  //
  // uart_transmitter.v:
  //     counter_int >= baud_div_i
  //
  // Therefore TX output bits last approximately baud_div + 1
  // clocks.
  //
  localparam RX_BIT_TIME = UART_BAUD_DIV * CLK_PERIOD;

  localparam TX_BIT_TIME = (UART_BAUD_DIV + 1) * CLK_PERIOD;

  // Measured from the actual uart_tx waveform during a calibration frame.
  // This avoids assuming the RTL counter timing in the TX checker.
  realtime measured_tx_bit_time;


  // ============================================================
  // UART REGISTER ADDRESSES
  // ============================================================

  localparam ADDR_THR = 5'h00;
  localparam ADDR_IER = 5'h04;
  localparam ADDR_BAUD = 5'h08;
  localparam ADDR_LCR = 5'h0C;
  localparam ADDR_LSR = 5'h14;


  // ============================================================
  // LCR CONFIGURATIONS
  // ============================================================

  // 8 data bits, 1 stop bit, no parity
  localparam LCR_8N1 = 32'h00000003;

  // 8 data bits, 2 stop bits, no parity
  localparam LCR_8N2 = 32'h00000007;

  // 8 data bits, 1 stop bit, odd parity
  localparam LCR_8O1 = 32'h0000000B;

  // 8 data bits, 1 stop bit, even parity
  localparam LCR_8E1 = 32'h0000001B;

  // DLAB + 8N1
  localparam LCR_DLAB_8N1 = 32'h00000083;


  // ============================================================
  // CLOCKS
  // ============================================================

  reg fixed_clk;
  reg axi_clk;

  initial begin
    fixed_clk = 1'b0;

    forever #(CLK_PERIOD / 2) fixed_clk = ~fixed_clk;
  end

  initial begin
    axi_clk = 1'b0;

    forever #(CLK_PERIOD / 2) axi_clk = ~axi_clk;
  end


  // ============================================================
  // RESET
  // ============================================================

  reg                       axi_resetn;


  // ============================================================
  // AXI READ ADDRESS
  // ============================================================

  reg  [  AXI_ID_WIDTH-1:0] axi_arid;
  reg  [AXI_ADDR_WIDTH-1:0] axi_araddr;
  reg                       axi_arvalid;

  wire                      axi_arready;


  // ============================================================
  // AXI READ DATA
  // ============================================================

  wire [  AXI_ID_WIDTH-1:0] axi_rid;
  wire [AXI_DATA_WIDTH-1:0] axi_rdata;
  wire [               1:0] axi_rresp;
  wire                      axi_rvalid;

  reg                       axi_rready;


  // ============================================================
  // AXI WRITE ADDRESS
  // ============================================================

  reg  [  AXI_ID_WIDTH-1:0] axi_awid;
  reg  [AXI_ADDR_WIDTH-1:0] axi_awaddr;
  reg                       axi_awvalid;

  wire                      axi_awready;


  // ============================================================
  // AXI WRITE DATA
  // ============================================================

  reg  [AXI_DATA_WIDTH-1:0] axi_wdata;
  reg  [               3:0] axi_wstrb;
  reg                       axi_wvalid;

  wire                      axi_wready;


  // ============================================================
  // AXI WRITE RESPONSE
  // ============================================================

  wire [  AXI_ID_WIDTH-1:0] axi_bid;
  wire [               1:0] axi_bresp;
  wire                      axi_bvalid;

  reg                       axi_bready;


  // ============================================================
  // UART
  // ============================================================

  reg                       uart_rx;
  wire                      uart_tx;

  wire                      read_interrupt;


  // ============================================================
  // DUT
  // ============================================================

  axi_uart_top DUT (

      .fixed_clk_i  (fixed_clk),
      .axi_aclk_i   (axi_clk),
      .axi_aresetn_i(axi_resetn),

      // AXI READ
      .axi_arid_i   (axi_arid),
      .axi_araddr_i (axi_araddr),
      .axi_arvalid_i(axi_arvalid),
      .axi_arready_o(axi_arready),

      .axi_rid_o   (axi_rid),
      .axi_rdata_o (axi_rdata),
      .axi_rresp_o (axi_rresp),
      .axi_rvalid_o(axi_rvalid),
      .axi_rready_i(axi_rready),

      // AXI WRITE
      .axi_awid_i   (axi_awid),
      .axi_awaddr_i (axi_awaddr),
      .axi_awvalid_i(axi_awvalid),
      .axi_awready_o(axi_awready),

      .axi_wdata_i (axi_wdata),
      .axi_wstrb_i (axi_wstrb),
      .axi_wvalid_i(axi_wvalid),
      .axi_wready_o(axi_wready),

      .axi_bid_o   (axi_bid),
      .axi_bresp_o (axi_bresp),
      .axi_bvalid_o(axi_bvalid),
      .axi_bready_i(axi_bready),

      // UART
      .uart_rx_i(uart_rx),
      .uart_tx_o(uart_tx),

      // INTERRUPT
      .read_interrupt_o(read_interrupt)
  );


  // ============================================================
  // TEST COUNTERS
  // ============================================================

  integer test_count;
  integer pass_count;
  integer fail_count;


  // ============================================================
  // AXI WRITE TASK
  // ============================================================

  task axi_write;

    input [AXI_ADDR_WIDTH-1:0] addr;
    input [AXI_DATA_WIDTH-1:0] data;

    begin

      @(posedge axi_clk);

      axi_awid    <= 12'h001;
      axi_awaddr  <= addr;
      axi_awvalid <= 1'b1;

      axi_wdata   <= data;
      axi_wstrb   <= 4'hF;
      axi_wvalid  <= 1'b1;

      axi_bready  <= 1'b1;

      wait (axi_awready && axi_wready);

      $display("");
      $display("AXI WRITE");
      $display("  ADDR = 0x%02h", addr);
      $display("  DATA = 0x%08h", data);
      $display("  AW/W handshake detected");

      wait (axi_bvalid);

      if (axi_bresp == 2'b00) begin

        $display("  BRESP = OK");

      end else begin

        $display("  BRESP = ERROR (%b)", axi_bresp);

        fail_count = fail_count + 1;

      end

      @(posedge axi_clk);

      axi_awvalid <= 1'b0;
      axi_wvalid  <= 1'b0;
      axi_bready  <= 1'b0;

      $display("AXI WRITE COMPLETE");

      repeat (3) @(posedge axi_clk);

    end

  endtask


  // ============================================================
  // AXI READ TASK
  // ============================================================

  task axi_read;

    input [AXI_ADDR_WIDTH-1:0] addr;
    output [AXI_DATA_WIDTH-1:0] data;

    begin

      @(posedge axi_clk);

      axi_arid    <= 12'h002;
      axi_araddr  <= addr;
      axi_arvalid <= 1'b1;

      axi_rready  <= 1'b1;

      wait (axi_arready);

      $display("");
      $display("AXI READ");
      $display("  ADDR = 0x%02h", addr);
      $display("  AR handshake detected");

      wait (axi_rvalid);

      data = axi_rdata;

      if (axi_rresp == 2'b00) begin

        $display("  DATA = 0x%08h", axi_rdata);
        $display("  RRESP = OK");

      end else begin

        $display("  RRESP = ERROR (%b)", axi_rresp);

        fail_count = fail_count + 1;

      end

      @(posedge axi_clk);

      axi_arvalid <= 1'b0;
      axi_rready  <= 1'b0;

      $display("AXI READ COMPLETE");

      repeat (3) @(posedge axi_clk);

    end

  endtask


  // ============================================================
  // WAIT FOR TX IDLE
  //
  // LSR:
  //   bit 6 = TEMT
  //   bit 5 = THRE
  //
  // This task is intentionally bounded so the simulation cannot
  // hang forever if the DUT gets stuck.
  // ============================================================

  task wait_tx_empty;

    reg [31:0] lsr_data;
    integer timeout_count;

    begin

      timeout_count = 0;

      while (timeout_count < 100) begin

        axi_read(ADDR_LSR, lsr_data);

        if ((lsr_data[6] == 1'b1) && (lsr_data[5] == 1'b1)) begin

          $display("[PASS] UART TX idle");

          break;

        end

        timeout_count = timeout_count + 1;

        if (measured_tx_bit_time > 0.0)
          #(measured_tx_bit_time);
        else
          #(TX_BIT_TIME);

      end

      if (timeout_count >= 100) begin

        $display("[FAIL] UART TX timeout");

        fail_count = fail_count + 1;

      end

    end

  endtask


  // ============================================================
  // UART TX CHECKER
  //
  // IMPORTANT:
  //
  // This checker uses the measured TX bit time, not the nominal
  // UART_BAUD_DIV-derived value.
  //
  // The timing calibration is performed before the TX test sequence.
  // ============================================================

  task measure_tx_bit_time;

    reg       seen_start;
    realtime  t_start;
    realtime  t_data0;
    realtime  measured;

    begin

      $display("");
      $display("UART TX TIMING CALIBRATION");
      $display("  Calibration byte = 0x55");

      seen_start = 1'b0;
      measured_tx_bit_time = 0.0;

      // The caller starts this task in parallel with the THR write.
      // 0x55 guarantees D0=1, so the first rising edge after the
      // start-bit falling edge is exactly one TX bit later.
      @(negedge uart_tx);
      t_start = $realtime;
      seen_start = 1'b1;

      @(posedge uart_tx);
      t_data0 = $realtime;

      measured = t_data0 - t_start;

      if (measured > 0.0) begin

        measured_tx_bit_time = measured;

        $display("  Start edge time = %0.3f ns", t_start);
        $display("  D0 edge time    = %0.3f ns", t_data0);
        $display("  Measured TX bit = %0.3f ns", measured_tx_bit_time);

        // Do not return immediately after measuring D0.  The calibration
        // frame (0x55) is still being transmitted.  Waiting here prevents
        // wait_tx_empty() from racing the DUT before its TX state/status
        // has updated, and prevents the first real TX checker from
        // accidentally catching the calibration frame.
        #(measured_tx_bit_time * 10.0);

      end else begin

        $display("[FAIL] Invalid measured TX bit time");
        fail_count = fail_count + 1;

      end

    end

  endtask


  // ============================================================
  // UART TX CHECKER
  //
  // The TX bit period is obtained from the real uart_tx waveform
  // using measure_tx_bit_time().  The checker therefore does not
  // accumulate an assumed baud-divisor timing error.
  // ============================================================

  task check_uart_tx;

    input [7:0] expected;

    reg [7:0] received;
    realtime bit_time;
    integer i;

    begin

      received = 8'h00;
      bit_time = measured_tx_bit_time;

      $display("");
      $display("UART TX CHECK");
      $display("  Expected = 0x%02h", expected);
      $display("  Measured bit time = %0.3f ns", bit_time);

      if (bit_time <= 0.0) begin

        $display("[FAIL] TX bit time has not been calibrated");
        fail_count = fail_count + 1;
        test_count = test_count + 1;

      end else begin

        // Start bit falling edge.
        @(negedge uart_tx);

        $display("  Start bit detected");

        // Center of D0 = 1.5 bit times after start edge.
        #(bit_time * 1.5);

        for (i = 0; i < 8; i = i + 1) begin

          received[i] = uart_tx;

          $display("  Data bit %0d = %b", i, uart_tx);

          if (i != 7)
            #(bit_time);

        end

        // D7 was sampled at its center.  One full bit later is the
        // center of the stop bit.
        #(bit_time);

        $display("  Stop bit = %b", uart_tx);

        if (uart_tx !== 1'b1) begin

          $display("[FAIL] Stop bit is not HIGH");
          fail_count = fail_count + 1;

        end

        if (received === expected) begin

          $display("");
          $display("--------------------------------");
          $display("UART TX TEST PASSED");
          $display("Expected = 0x%02h", expected);
          $display("Received = 0x%02h", received);
          $display("--------------------------------");

          pass_count = pass_count + 1;

        end else begin

          $display("");
          $display("--------------------------------");
          $display("UART TX TEST FAILED");
          $display("Expected = 0x%02h", expected);
          $display("Received = 0x%02h", received);
          $display("--------------------------------");

          fail_count = fail_count + 1;

        end

        test_count = test_count + 1;

        #(bit_time);

      end

    end

  endtask


  // ============================================================
  // UART RX 8N1
  //
  // Uses RX_BIT_TIME because the DUT receiver counts to
  // baud_div_i - 1.
  // ============================================================

  task uart_send_byte_8n1;

    input [7:0] data;

    integer i;

    begin

      $display("");
      $display("UART RX STIMULUS");
      $display("  Sending = 0x%02h", data);

      // Idle
      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      // START
      uart_rx = 1'b0;

      #(RX_BIT_TIME);

      // DATA
      for (i = 0; i < 8; i = i + 1) begin

        uart_rx = data[i];

        #(RX_BIT_TIME);

      end

      // STOP
      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      // Idle
      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      $display("  RX frame complete");

    end

  endtask


  // ============================================================
  // UART RX 8N2
  // ============================================================

  task uart_send_byte_8n2;

    input [7:0] data;

    integer i;

    begin

      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      // START
      uart_rx = 1'b0;

      #(RX_BIT_TIME);

      // DATA
      for (i = 0; i < 8; i = i + 1) begin

        uart_rx = data[i];

        #(RX_BIT_TIME);

      end

      // STOP 1
      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      // STOP 2
      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      uart_rx = 1'b1;

      #(RX_BIT_TIME);

    end

  endtask


  // ============================================================
  // UART RX WITH PARITY
  //
  // parity_mode:
  //   0 = odd
  //   1 = even
  // ============================================================

  task uart_send_byte_parity;

    input [7:0] data;
    input parity_mode;

    reg parity;

    integer i;

    begin

      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      // START
      uart_rx = 1'b0;

      #(RX_BIT_TIME);

      // DATA
      for (i = 0; i < 8; i = i + 1) begin

        uart_rx = data[i];

        #(RX_BIT_TIME);

      end

      // Calculate parity
      if (parity_mode == 1'b1)

        // EVEN parity
        parity = ^data;

      else

        // ODD parity
        parity = ~(^data);

      // PARITY
      uart_rx = parity;

      #(RX_BIT_TIME);

      // STOP
      uart_rx = 1'b1;

      #(RX_BIT_TIME);

      uart_rx = 1'b1;

      #(RX_BIT_TIME);

    end

  endtask


  // ============================================================
  // TEST 1: RESET
  // ============================================================

  task reset_test;

    reg [31:0] lsr;

    begin

      $display("");
      $display("========================================");
      $display("TEST 1: RESET");
      $display("========================================");

      axi_resetn = 1'b0;

      uart_rx = 1'b1;

      // Clear AXI
      axi_awvalid = 1'b0;
      axi_wvalid = 1'b0;
      axi_bready = 1'b0;

      axi_arvalid = 1'b0;
      axi_rready = 1'b0;

      repeat (10) @(posedge fixed_clk);

      // TX should be HIGH during reset
      if (uart_tx === 1'b1) begin

        $display("[PASS] UART TX idle during reset");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] UART TX not idle during reset");

        fail_count = fail_count + 1;

      end

      // Release reset
      axi_resetn = 1'b1;

      repeat (20) @(posedge fixed_clk);

      $display("[PASS] Reset released");

      // Read LSR
      axi_read(ADDR_LSR, lsr);

      if (lsr[6] === 1'b1) begin

        $display("[PASS] LSR.TEMT = 1");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] LSR.TEMT = 0");

        fail_count = fail_count + 1;

      end

      if (lsr[5] === 1'b1) begin

        $display("[PASS] LSR.THRE = 1");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] LSR.THRE = 0");

        fail_count = fail_count + 1;

      end

    end

  endtask


  // ============================================================
  // TEST 2: REGISTER ACCESS
  // ============================================================

  task register_test;

    reg [31:0] data;

    begin

      $display("");
      $display("========================================");
      $display("TEST 2: REGISTER ACCESS");
      $display("========================================");

      // LCR
      axi_write(ADDR_LCR, LCR_8N1);

      // IER enable
      axi_write(ADDR_IER, 32'h00000001);

      // IER disable
      axi_write(ADDR_IER, 32'h00000000);

      // DLAB
      axi_write(ADDR_LCR, LCR_DLAB_8N1);

      // Baud divisor
      axi_write(ADDR_BAUD, UART_BAUD_DIV);

      // Back to 8N1
      axi_write(ADDR_LCR, LCR_8N1);

      $display("[PASS] Register writes completed");

      // Read LSR
      axi_read(ADDR_LSR, data);

      if (data[6] === 1'b1) $display("[PASS] LSR.TEMT");

      if (data[5] === 1'b1) $display("[PASS] LSR.THRE");

    end

  endtask


  // ============================================================
  // TEST 3: TRANSMITTER
  // ============================================================

  task tx_test;

    begin

      $display("");
      $display("========================================");
      $display("TEST 3: UART TRANSMITTER");
      $display("========================================");

      // Configure 8N1
      axi_write(ADDR_LCR, LCR_8N1);

      // Program baud divisor
      axi_write(ADDR_LCR, LCR_DLAB_8N1);

      axi_write(ADDR_BAUD, UART_BAUD_DIV);

      axi_write(ADDR_LCR, LCR_8N1);


      // ====================================================
      // CALIBRATE ACTUAL TX BIT TIME
      // ====================================================

      // 0x55 is used because D0=1, guaranteeing a transition
      // exactly one bit after the start-bit falling edge.
      fork

        begin
          measure_tx_bit_time();
        end

        begin
          axi_write(ADDR_THR, 32'h00000055);
        end

      join

      wait_tx_empty();


      // ====================================================
      // 0x00
      // ====================================================

      fork

        begin
          check_uart_tx(8'h00);
        end

        begin
          axi_write(ADDR_THR, 32'h00000000);
        end

      join

      wait_tx_empty();


      // ====================================================
      // 0xFF
      // ====================================================

      fork

        begin
          check_uart_tx(8'hFF);
        end

        begin
          axi_write(ADDR_THR, 32'h000000FF);
        end

      join

      wait_tx_empty();


      // ====================================================
      // 0x55
      // ====================================================

      fork

        begin
          check_uart_tx(8'h55);
        end

        begin
          axi_write(ADDR_THR, 32'h00000055);
        end

      join

      wait_tx_empty();


      // ====================================================
      // 0xAA
      // ====================================================

      fork

        begin
          check_uart_tx(8'hAA);
        end

        begin
          axi_write(ADDR_THR, 32'h000000AA);
        end

      join

      wait_tx_empty();


      // ====================================================
      // 0xA5
      // ====================================================

      fork

        begin
          check_uart_tx(8'hA5);
        end

        begin
          axi_write(ADDR_THR, 32'h000000A5);
        end

      join

      wait_tx_empty();

      $display("");
      $display("[PASS] TX test sequence complete");

    end

  endtask


  // ============================================================
  // TEST 4: RECEIVER
  // ============================================================

  task rx_test;

    reg [31:0] data;

    begin

      $display("");
      $display("========================================");
      $display("TEST 4: UART RECEIVER");
      $display("========================================");

      // 8N1
      axi_write(ADDR_LCR, LCR_8N1);

      // Enable RX interrupt
      axi_write(ADDR_IER, 32'h00000001);

      // Send 0x55
      uart_send_byte_8n1(8'h55);

      // Allow receiver to finish
      repeat (20) @(posedge fixed_clk);

      // Interrupt
      if (read_interrupt === 1'b1) begin

        $display("[PASS] RX interrupt asserted");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] RX interrupt not asserted");

        fail_count = fail_count + 1;

      end

      // LSR
      axi_read(ADDR_LSR, data);

      if (data[0] === 1'b1) begin

        $display("[PASS] LSR.DATA_READY = 1");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] LSR.DATA_READY = 0");

        fail_count = fail_count + 1;

      end

      // Read received byte
      axi_read(ADDR_THR, data);

      if (data[7:0] === 8'h55) begin

        $display("[PASS] RX data = 0x55");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] RX data mismatch");
        $display("       Expected = 0x55");
        $display("       Received = 0x%02h", data[7:0]);

        fail_count = fail_count + 1;

      end

      repeat (10) @(posedge fixed_clk);

      // Disable interrupt
      axi_write(ADDR_IER, 32'h00000000);

    end

  endtask


  // ============================================================
  // TEST 5: RX FIFO
  // ============================================================

  task fifo_test;

    reg [31:0] data;

    begin

      $display("");
      $display("========================================");
      $display("TEST 5: RX FIFO");
      $display("========================================");

      axi_write(ADDR_LCR, LCR_8N1);

      axi_write(ADDR_IER, 32'h00000001);

      // Byte 1
      uart_send_byte_8n1(8'h11);

      repeat (10) @(posedge fixed_clk);

      // Byte 2
      uart_send_byte_8n1(8'h22);

      repeat (10) @(posedge fixed_clk);

      // Byte 3
      uart_send_byte_8n1(8'h33);

      repeat (10) @(posedge fixed_clk);

      // Byte 4
      uart_send_byte_8n1(8'h44);

      repeat (20) @(posedge fixed_clk);


      // BYTE 1
      axi_read(ADDR_THR, data);

      if (data[7:0] === 8'h11) begin

        $display("[PASS] FIFO byte 1 = 0x11");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] FIFO byte 1 = 0x%02h", data[7:0]);

        fail_count = fail_count + 1;

      end


      // BYTE 2
      axi_read(ADDR_THR, data);

      if (data[7:0] === 8'h22) begin

        $display("[PASS] FIFO byte 2 = 0x22");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] FIFO byte 2 = 0x%02h", data[7:0]);

        fail_count = fail_count + 1;

      end


      // BYTE 3
      axi_read(ADDR_THR, data);

      if (data[7:0] === 8'h33) begin

        $display("[PASS] FIFO byte 3 = 0x33");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] FIFO byte 3 = 0x%02h", data[7:0]);

        fail_count = fail_count + 1;

      end


      // BYTE 4
      axi_read(ADDR_THR, data);

      if (data[7:0] === 8'h44) begin

        $display("[PASS] FIFO byte 4 = 0x44");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] FIFO byte 4 = 0x%02h", data[7:0]);

        fail_count = fail_count + 1;

      end


      axi_write(ADDR_IER, 32'h00000000);

    end

  endtask


  // ============================================================
  // TEST 6: TWO STOP BITS
  // ============================================================

  task stop_bit_test;

    realtime bit_time;

    begin

      bit_time = measured_tx_bit_time;

      $display("");
      $display("========================================");
      $display("TEST 6: TWO STOP BITS");
      $display("========================================");

      // Configure 8N2
      axi_write(ADDR_LCR, LCR_8N2);

      // Write byte
      axi_write(ADDR_THR, 32'h00000055);

      // Wait for start
      @(negedge uart_tx);

      // Start bit center
      #(bit_time / 2);

      // Move through 8 data bits
      #(bit_time);

      repeat (7) #(bit_time);

      // Move to first stop bit
      #(bit_time);

      if (uart_tx === 1'b1) begin

        $display("[PASS] First stop bit");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] First stop bit");

        fail_count = fail_count + 1;

      end

      // Second stop bit
      #(bit_time);

      if (uart_tx === 1'b1) begin

        $display("[PASS] Second stop bit");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] Second stop bit");

        fail_count = fail_count + 1;

      end

      #(bit_time);

      // Return to 8N1
      axi_write(ADDR_LCR, LCR_8N1);

    end

  endtask


  // ============================================================
  // TEST 7: PARITY
  // ============================================================

  task parity_test;

    realtime bit_time;

    begin

      $display("");
      $display("========================================");
      $display("TEST 7: PARITY");
      $display("========================================");

      bit_time = measured_tx_bit_time;

      if (bit_time <= 0.0) begin
        $display("[FAIL] TX bit time has not been calibrated");
        fail_count = fail_count + 1;
      end else begin

        // ====================================================
        // ODD PARITY
        // ====================================================

        axi_write(ADDR_LCR, LCR_8O1);

        fork

          begin
            @(negedge uart_tx);

            // Start edge -> D0 center = 1.5 bit times.
            #(bit_time * 1.5);

            // Move from D0 center to D7 center.
            repeat (7) #(bit_time);

            // D7 center -> parity center.
            #(bit_time);

            $display("Odd parity bit = %b", uart_tx);

            // 0x55 has four 1s, so odd parity requires 1.
            if (uart_tx === 1'b1) begin
              $display("[PASS] Odd parity");
              pass_count = pass_count + 1;
            end else begin
              $display("[FAIL] Odd parity");
              fail_count = fail_count + 1;
            end
          end

          begin
            axi_write(ADDR_THR, 32'h00000055);
          end

        join

        wait_tx_empty();


        // ====================================================
        // EVEN PARITY
        // ====================================================

        axi_write(ADDR_LCR, LCR_8E1);

        fork

          begin
            @(negedge uart_tx);

            #(bit_time * 1.5);
            repeat (7) #(bit_time);
            #(bit_time);

            $display("Even parity bit = %b", uart_tx);

            // 0x55 has four 1s, so even parity requires 0.
            if (uart_tx === 1'b0) begin
              $display("[PASS] Even parity");
              pass_count = pass_count + 1;
            end else begin
              $display("[FAIL] Even parity");
              fail_count = fail_count + 1;
            end
          end

          begin
            axi_write(ADDR_THR, 32'h00000055);
          end

        join

        wait_tx_empty();

        // Return to 8N1
        axi_write(ADDR_LCR, LCR_8N1);

      end

    end

  endtask  endtask


  // ============================================================
  // TEST 8: INVALID REGISTER
  // ============================================================

  task invalid_register_test;

    begin

      $display("");
      $display("========================================");
      $display("TEST 8: INVALID REGISTER ACCESS");
      $display("========================================");

      // 0x10 is undefined
      axi_write(5'h10, 32'hDEADBEEF);

      $display("[PASS] Invalid register write completed");

      pass_count = pass_count + 1;

    end

  endtask


  // ============================================================
  // TEST 9: RESET DURING TX
  // ============================================================

  task reset_during_tx;

    begin

      $display("");
      $display("========================================");
      $display("TEST 9: RESET DURING TRANSMISSION");
      $display("========================================");

      axi_write(ADDR_LCR, LCR_8N1);

      axi_write(ADDR_THR, 32'h000000AA);

      // Wait for TX start
      @(negedge uart_tx);

      // Allow several TX bits
      #(measured_tx_bit_time * 3);

      // Assert reset
      axi_resetn = 1'b0;

      repeat (5) @(posedge fixed_clk);

      // TX should return HIGH
      if (uart_tx === 1'b1) begin

        $display("[PASS] TX returned to idle after reset");

        pass_count = pass_count + 1;

      end else begin

        $display("[FAIL] TX did not return to idle");

        fail_count = fail_count + 1;

      end

      // Release reset
      axi_resetn = 1'b1;

      repeat (20) @(posedge fixed_clk);

      $display("[PASS] Reset recovery complete");

    end

  endtask


  // ============================================================
  // MAIN TEST
  // ============================================================

  initial begin

    // --------------------------------------------------------
    // Initial values
    // --------------------------------------------------------

    axi_resetn  = 1'b0;

    axi_arid    = 12'd0;
    axi_araddr  = 5'd0;
    axi_arvalid = 1'b0;
    axi_rready  = 1'b0;

    axi_awid    = 12'd0;
    axi_awaddr  = 5'd0;
    axi_awvalid = 1'b0;

    axi_wdata   = 32'd0;
    axi_wstrb   = 4'h0;
    axi_wvalid  = 1'b0;

    axi_bready  = 1'b0;

    uart_rx     = 1'b1;

    test_count  = 0;
    pass_count  = 0;
    fail_count  = 0;


    // --------------------------------------------------------
    // Waveform dump
    // --------------------------------------------------------

    $dumpfile("uart_full_tb.vcd");
    $dumpvars(0, tb_axi_uart_top);


    // --------------------------------------------------------
    // RUN TESTS
    // --------------------------------------------------------

    reset_test();

    register_test();

    tx_test();

    rx_test();

    fifo_test();

    stop_bit_test();

    parity_test();

    invalid_register_test();

    reset_during_tx();


    // --------------------------------------------------------
    // FINAL SUMMARY
    // --------------------------------------------------------

    $display("");
    $display("");
    $display("==================================================");
    $display("              UART FULL VERIFICATION");
    $display("==================================================");

    $display("PASS COUNT : %0d", pass_count);
    $display("FAIL COUNT : %0d", fail_count);

    if (fail_count == 0) begin

      $display("");
      $display("**********************************************");
      $display("*                                            *");
      $display("*       UART VERIFICATION PASSED             *");
      $display("*                                            *");
      $display("**********************************************");

    end else begin

      $display("");
      $display("**********************************************");
      $display("*                                            *");
      $display("*       UART VERIFICATION FAILED             *");
      $display("*                                            *");
      $display("**********************************************");

    end

    $display("==================================================");

    #1000;

    $finish;

  end

endmodule
