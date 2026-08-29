//==============================================================================
// tb_aes_core.sv
//
// Testbench for aes_cipher_top (encryption) and aes_inv_cipher_top (decryption)
// from the OpenCores AES-128 core.
//
// Connection topology (matches bench/verilog/test_bench_top.v):
//   - A single 'kld' pulse loads key+plaintext and triggers:
//       * aes_cipher_top.ld       → starts encryption
//       * aes_inv_cipher_top.kld  → starts key-schedule precomputation
//   - aes_cipher_top.done is wired to aes_inv_cipher_top.ld, so decryption
//     starts automatically when encryption completes (~12 clocks).
//   - Decryption finishes ~12 clocks after that; dec_text_out recovers plaintext.
//
// Sampling rule (RTL-specific):
//   Both enc_text_out and dec_text_out are registered on posedge clk by the RTL.
//   Their valid value is present ON the posedge where done=1; it changes on the
//   very next posedge.  Therefore outputs must be captured at the SAME posedge
//   as the done assertion, not one cycle later.
//
// Tests:
//   1. KAT Encryption     – 8 vectors from the project's own test_bench_top.v
//   2. KAT Round-Trip     – enc→dec must recover the original plaintext
//   3. Back-to-back       – 3 different plaintexts with the same key
//
// VCS compile + run (from Project/run):
//   export VCS_HOME=/home/student/snps_tools_target/vcs/U-2023.03
//   export PATH=$VCS_HOME/bin:$PATH
//   export SNPSLMD_LICENSE_FILE=27021@14.139.1.126
//   export VCS_ARCH_OVERRIDE=linux
//   RTL=/home/student/Documents/Vijay-058/IPs/aes_core-master/rtl/verilog
//   vcs -sverilog -full64 +incdir+$RTL                               \
//       $RTL/aes_sbox.v $RTL/aes_inv_sbox.v $RTL/aes_rcon.v         \
//       $RTL/aes_key_expand_128.v $RTL/aes_cipher_top.v              \
//       $RTL/aes_inv_cipher_top.v                                     \
//       ../tb/tb_aes_core.sv -top tb_aes_core -o simv_aes_core
//   ./simv_aes_core -no_save
//==============================================================================

`timescale 1ns/1ps

module tb_aes_core;

  //--------------------------------------------------------------------------
  // Parameters
  //--------------------------------------------------------------------------
  localparam CLK_HALF = 5;     // 10 ns → 100 MHz
  localparam N_KAT    = 8;

  //--------------------------------------------------------------------------
  // Signals
  //--------------------------------------------------------------------------
  logic          clk;
  logic          rst;
  logic          kld;           // one-cycle pulse: start encryption + load dec key-sched

  logic [127:0]  key;           // valid only while kld=1, X otherwise
  logic [127:0]  text_in;       // valid only while kld=1, X otherwise

  wire  [127:0]  enc_text_out;
  wire           enc_done;

  wire  [127:0]  dec_text_out;
  wire           dec_done;

  //--------------------------------------------------------------------------
  // DUT: Encryption
  //--------------------------------------------------------------------------
  aes_cipher_top u_enc (
    .clk      (clk),
    .rst      (rst),
    .ld       (kld),
    .done     (enc_done),
    .key      (key),
    .text_in  (text_in),
    .text_out (enc_text_out)
  );

  //--------------------------------------------------------------------------
  // DUT: Decryption
  // kld triggers key-schedule; enc_done triggers decryption of enc_text_out
  //--------------------------------------------------------------------------
  aes_inv_cipher_top u_dec (
    .clk      (clk),
    .rst      (rst),
    .kld      (kld),
    .ld       (enc_done),         // decrypt as soon as encryption is done
    .done     (dec_done),
    .key      (key),
    .text_in  (enc_text_out),     // feed ciphertext directly into decryptor
    .text_out (dec_text_out)
  );

  //--------------------------------------------------------------------------
  // Clock
  //--------------------------------------------------------------------------
  initial clk = 1'b0;
  always  #(CLK_HALF) clk = ~clk;

  //--------------------------------------------------------------------------
  // KAT vectors  {key[127:0], plaintext[127:0], expected_ciphertext[127:0]}
  // Taken from bench/verilog/test_bench_top.v (verified 0 errors by original bench)
  //--------------------------------------------------------------------------
  logic [383:0] kat [0:N_KAT-1];
  initial begin
    kat[0] = 384'h00000000000000000000000000000000_f34481ec3cc627bacd5dc3fb08f273e6_0336763e966d92595a567cc9ce537f5e;
    kat[1] = 384'h00000000000000000000000000000000_9798c4640bad75c7c3227db910174e72_a9a1631bf4996954ebc093957b234589;
    kat[2] = 384'h00000000000000000000000000000000_96ab5c2ff612d9dfaae8c31f30c42168_ff4f8391a6a40ca5b25d23bedd44a597;
    kat[3] = 384'h00000000000000000000000000000000_6a118a874519e64e9963798a503f1d35_dc43be40be0e53712f7e2bf5ca707209;
    kat[4] = 384'h10a58869d74be5a374cf867cfb473859_00000000000000000000000000000000_6d251e6944b051e04eaa6fb4dbf78465;
    kat[5] = 384'hcaea65cdbb75e9169ecd22ebe6e54675_00000000000000000000000000000000_6e29201190152df4ee058139def610bb;
    kat[6] = 384'h80000000000000000000000000000000_00000000000000000000000000000000_0edd33d3c621e546455bd8ba1418bec8;
    kat[7] = 384'hc0000000000000000000000000000000_00000000000000000000000000000000_4bc3f883450c113c64ca42e1112a9e87;
  end

  //--------------------------------------------------------------------------
  // Scoreboard
  //--------------------------------------------------------------------------
  int pass_cnt = 0;
  int fail_cnt = 0;

  task automatic check(
    input string        label,
    input logic [127:0] got,
    input logic [127:0] exp
  );
    if (got === exp) begin
      $display("  PASS  %-44s  result=%032h", label, got);
      pass_cnt++;
    end else begin
      $display("  FAIL  %-44s", label);
      $display("        expected: %032h", exp);
      $display("        got:      %032h", got);
      fail_cnt++;
    end
  endtask

  //--------------------------------------------------------------------------
  // run_block: pulse kld for one clock, wait for enc_done and dec_done.
  //
  // IMPORTANT: The RTL registers enc_text_out and dec_text_out on posedge clk.
  // Both outputs are valid ON the posedge where the corresponding done signal
  // is high.  One cycle later the registers update to the next state, so
  // outputs MUST be read during the same posedge-active time slot — this is
  // done by reading them immediately after @(posedge clk iff done) without
  // any additional clock advance.
  //--------------------------------------------------------------------------
  task automatic run_block(
    input  logic [127:0] in_key,
    input  logic [127:0] in_plain,
    output logic [127:0] out_cipher,
    output logic [127:0] out_plain
  );
    // Drive inputs just after a rising edge (#1 avoids delta-cycle conflicts)
    @(posedge clk); #1;
    kld     = 1'b1;
    key     = in_key;
    text_in = in_plain;

    @(posedge clk); #1;     // kld held for exactly one clock period
    kld     = 1'b0;
    key     = 128'hx;
    text_in = 128'hx;

    // Wait for encryption to complete; sample output at the SAME posedge
    @(posedge clk iff enc_done);
    out_cipher = enc_text_out;   // capture immediately at this posedge

    // Wait for decryption to complete; sample output at the SAME posedge
    @(posedge clk iff dec_done);
    out_plain = dec_text_out;    // capture immediately at this posedge
  endtask

  //--------------------------------------------------------------------------
  // Main stimulus
  //--------------------------------------------------------------------------
  initial begin
    kld     = 1'b0;
    key     = 128'hx;
    text_in = 128'hx;
    rst     = 1'b0;

    repeat(4) @(posedge clk);
    rst = 1'b1;
    repeat(20) @(posedge clk);

    $display("");
    $display("=============================================================");
    $display("  AES-128 Core Testbench  (tb_aes_core.sv)");
    $display("  DUT: aes_cipher_top + aes_inv_cipher_top");
    $display("=============================================================");

    // -----------------------------------------------------------------------
    // Test 1 – KAT Encryption
    // -----------------------------------------------------------------------
    $display("\n--- Test 1: NIST/KAT Encryption (%0d vectors) ---", N_KAT);
    begin
      logic [127:0] got_c, got_p;
      for (int i = 0; i < N_KAT; i++) begin
        run_block(kat[i][383:256], kat[i][255:128], got_c, got_p);
        check($sformatf("KAT enc[%0d]", i), got_c, kat[i][127:0]);
      end
    end

    // -----------------------------------------------------------------------
    // Test 2 – Encrypt → Decrypt Round-Trip
    // -----------------------------------------------------------------------
    $display("\n--- Test 2: Encrypt→Decrypt Round-Trip (%0d vectors) ---", N_KAT);
    begin
      logic [127:0] got_c, got_p;
      for (int i = 0; i < N_KAT; i++) begin
        run_block(kat[i][383:256], kat[i][255:128], got_c, got_p);
        check($sformatf("KAT round-trip[%0d]", i), got_p, kat[i][255:128]);
      end
    end

    // -----------------------------------------------------------------------
    // Test 3 – Back-to-back (3 blocks, same key, different plaintexts)
    // -----------------------------------------------------------------------
    $display("\n--- Test 3: Back-to-back Encrypt+Decrypt (3 blocks) ---");
    begin
      localparam logic [127:0] BB_KEY = 128'hDEAD_BEEF_1234_5678_DEAD_BEEF_1234_5678;
      logic [127:0] plains[2:0];
      logic [127:0] got_c, got_p;

      plains[0] = 128'h0123_4567_89AB_CDEF_FEDC_BA98_7654_3210;
      plains[1] = 128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
      plains[2] = 128'h0000_0000_0000_0000_0000_0000_0000_0001;

      for (int i = 0; i < 3; i++) begin
        run_block(BB_KEY, plains[i], got_c, got_p);
        check($sformatf("back-to-back[%0d]", i), got_p, plains[i]);
      end
    end

    // -----------------------------------------------------------------------
    // Summary
    // -----------------------------------------------------------------------
    $display("");
    $display("=============================================================");
    $display("  Results: %0d PASSED,  %0d FAILED", pass_cnt, fail_cnt);
    $display("=============================================================");
    if (fail_cnt == 0)
      $display("  *** ALL TESTS PASSED ***\n");
    else
      $display("  *** SOME TESTS FAILED – see details above ***\n");

    $finish;
  end

  //--------------------------------------------------------------------------
  // Watchdog (2 ms)
  //--------------------------------------------------------------------------
  initial begin
    #2_000_000;
    $display("ERROR: Simulation timeout!");
    $finish;
  end

  //--------------------------------------------------------------------------
  // Waveform dump (compile with +define+DUMP_WAVES to enable)
  //--------------------------------------------------------------------------
`ifdef DUMP_WAVES
  initial begin
    $dumpfile("tb_aes_core.vcd");
    $dumpvars(0, tb_aes_core);
  end
`endif

endmodule
