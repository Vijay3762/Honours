// ============================================================================
// File    : veer_iccm_ram.sv
// Project : RISC-V CNN Digit Recognition SoC
// Purpose : Behavioural model of the VeeR EL2 instruction cache memory
//           (ICCM) that sits behind the el2_mem_if SRAM interface.
//
//           The ICCM is the core's local instruction memory.  It is used
//           here to hold the boot program: the core fetches from the ICCM
//           (no AXI traffic), while all DATA accesses - and therefore the
//           whole UART test - go through the AXI interconnect.
//
//           The ICCM is organised as 4 banks of 32 bit words plus 7 ECC
//           bits per word.  The bank is selected by address bits [3:2],
//           the offset inside a bank by address bits [15:4].
//
//           The ECC bits written by the core are stored together with the
//           data, so the core's ECC check always matches.  Words that come
//           from the hex file get their ECC computed here.
//
//           The same module is the model that has to be replaced by real
//           block RAMs (or distributed RAM) in the FPGA implementation.
// ============================================================================

`timescale 1ns/1ps
`default_nettype none

module veer_iccm_ram #(
    parameter integer ICCM_NUM_BANKS      = 4,
    parameter integer ICCM_BITS           = 16,   // address bits of the ICCM
    parameter integer ICCM_BANK_INDEX_LO  = 4,    // first bit of the bank offset
    parameter integer ICCM_ECC_WIDTH      = 7,
    // hex file with the boot program (32 bit words, one per line)
    parameter         MEM_HEX             = ""
)(
    input  wire                                        clk,

    // ---- interface towards the core (el2_mem_if.veer_sram_src) ----------
    input  wire [ICCM_NUM_BANKS-1:0]                   clken,
    input  wire [ICCM_NUM_BANKS-1:0]                   wren_bank,
    input  wire [ICCM_NUM_BANKS-1:0][ICCM_BITS-1:ICCM_BANK_INDEX_LO] addr_bank,
    input  wire [ICCM_NUM_BANKS-1:0][31:0]             wr_data,
    input  wire [ICCM_NUM_BANKS-1:0][ICCM_ECC_WIDTH-1:0] wr_ecc,

    output wire [ICCM_NUM_BANKS-1:0][31:0]             rd_data,
    output wire [ICCM_NUM_BANKS-1:0][ICCM_ECC_WIDTH-1:0] rd_ecc
);

    localparam integer IDX_BITS = ICCM_BITS - ICCM_BANK_INDEX_LO;   // 12
    localparam integer IDX_DEPTH = 1 << IDX_BITS;                   // 4096

    // ------------------------------------------------------------------
    // The memory: one array per bank
    // ------------------------------------------------------------------
    logic [31:0]               mem_data [0:ICCM_NUM_BANKS-1][0:IDX_DEPTH-1];
    logic [ICCM_ECC_WIDTH-1:0] mem_ecc  [0:ICCM_NUM_BANKS-1][0:IDX_DEPTH-1];

    // ------------------------------------------------------------------
    // RISC-V style ECC (same algorithm as the VeeR testbench)
    // ------------------------------------------------------------------
    function automatic [ICCM_ECC_WIDTH-1:0] riscv_ecc32(input [31:0] data);
        reg [ICCM_ECC_WIDTH-1:0] synd;
        begin
            synd[0] = ^(data & 32'h56aa_ad5b);
            synd[1] = ^(data & 32'h9b33_366d);
            synd[2] = ^(data & 32'he3c3_c78e);
            synd[3] = ^(data & 32'h03fc_07f0);
            synd[4] = ^(data & 32'h03ff_f800);
            synd[5] = ^(data & 32'hfc00_0000);
            synd[6] = ^{data, synd[5:0]};
            riscv_ecc32 = synd;
        end
    endfunction

    // ------------------------------------------------------------------
    // Load the boot program
    //   The word index of the flat image is {offset, bank}, so the word
    //   with index w belongs to bank (w % 4) at offset (w / 4).
    // ------------------------------------------------------------------
    logic [31:0] flat [0:(IDX_DEPTH*ICCM_NUM_BANKS)-1];

    integer i, b;
    initial begin
        for (i = 0; i < IDX_DEPTH; i = i + 1) begin
            for (b = 0; b < ICCM_NUM_BANKS; b = b + 1) begin
                mem_data[b][i] = 32'h0000_0000;
                mem_ecc [b][i] = '0;
            end
        end
        for (i = 0; i < IDX_DEPTH*ICCM_NUM_BANKS; i = i + 1)
            flat[i] = 32'h0000_0000;

        if (MEM_HEX != "") begin
            $display("veer_iccm_ram: loading hex file: %s", MEM_HEX);
            $readmemh(MEM_HEX, flat);
            for (i = 0; i < IDX_DEPTH*ICCM_NUM_BANKS; i = i + 1) begin
                mem_data[i % ICCM_NUM_BANKS][i / ICCM_NUM_BANKS] = flat[i];
                mem_ecc [i % ICCM_NUM_BANKS][i / ICCM_NUM_BANKS] =
                                                        riscv_ecc32(flat[i]);
            end
        end
    end

    // ------------------------------------------------------------------
    // Write and read
    // ------------------------------------------------------------------
    reg [ICCM_NUM_BANKS-1:0][31:0]               rd_data_q;
    reg [ICCM_NUM_BANKS-1:0][ICCM_ECC_WIDTH-1:0] rd_ecc_q;

    always @(posedge clk) begin
        for (b = 0; b < ICCM_NUM_BANKS; b = b + 1) begin
            if (clken[b] && wren_bank[b]) begin
                mem_data[b][addr_bank[b]] <= wr_data[b];
                mem_ecc [b][addr_bank[b]] <= wr_ecc[b];
            end
            if (clken[b]) begin
                rd_data_q[b] <= mem_data[b][addr_bank[b]];
                rd_ecc_q [b] <= mem_ecc [b][addr_bank[b]];
            end
        end
    end

    assign rd_data = rd_data_q;
    assign rd_ecc  = rd_ecc_q;

endmodule

`default_nettype wire
