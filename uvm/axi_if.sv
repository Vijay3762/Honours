// =============================================================================
// File    : axi_if.sv
// Purpose : 64-bit / 32-bit AXI4 SystemVerilog Interface for Verification VIPs
// Standard: ARM AMBA AXI4 Protocol Specification (IHI0022E)
// =============================================================================

`ifndef AXI_IF_SV
`define AXI_IF_SV

interface axi_if #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 64,
    parameter ID_WIDTH   = 8,
    parameter USER_WIDTH = 1
)(
    input wire clk,
    input wire rst_n
);

  // ---------------------------------------------------------------------------
  // Write Address Channel (AW)
  // ---------------------------------------------------------------------------
  logic [ID_WIDTH-1:0]    awid;
  logic [ADDR_WIDTH-1:0]  awaddr;
  logic [7:0]             awlen;
  logic [2:0]             awsize;
  logic [1:0]             awburst;
  logic                   awlock;
  logic [3:0]             awcache;
  logic [2:0]             awprot;
  logic [3:0]             awqos;
  logic [3:0]             awregion;
  logic [USER_WIDTH-1:0]  awuser;
  logic                   awvalid;
  logic                   awready;

  // ---------------------------------------------------------------------------
  // Write Data Channel (W)
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH-1:0]  wdata;
  logic [(DATA_WIDTH/8)-1:0] wstrb;
  logic                   wlast;
  logic [USER_WIDTH-1:0]  wuser;
  logic                   wvalid;
  logic                   wready;

  // ---------------------------------------------------------------------------
  // Write Response Channel (B)
  // ---------------------------------------------------------------------------
  logic [ID_WIDTH-1:0]    bid;
  logic [1:0]             bresp;
  logic [USER_WIDTH-1:0]  buser;
  logic                   bvalid;
  logic                   bready;

  // ---------------------------------------------------------------------------
  // Read Address Channel (AR)
  // ---------------------------------------------------------------------------
  logic [ID_WIDTH-1:0]    arid;
  logic [ADDR_WIDTH-1:0]  araddr;
  logic [7:0]             arlen;
  logic [2:0]             arsize;
  logic [1:0]             arburst;
  logic                   arlock;
  logic [3:0]             arcache;
  logic [2:0]             arprot;
  logic [3:0]             arqos;
  logic [3:0]             arregion;
  logic [USER_WIDTH-1:0]  aruser;
  logic                   arvalid;
  logic                   arready;

  // ---------------------------------------------------------------------------
  // Read Data Channel (R)
  // ---------------------------------------------------------------------------
  logic [ID_WIDTH-1:0]    rid;
  logic [DATA_WIDTH-1:0]  rdata;
  logic [1:0]             rresp;
  logic                   rlast;
  logic [USER_WIDTH-1:0]  ruser;
  logic                   rvalid;
  logic                   rready;

  // Modport for Master VIP
  modport master (
    input  clk, rst_n,
    output awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser, awvalid,
    input  awready,
    output wdata, wstrb, wlast, wuser, wvalid,
    input  wready,
    input  bid, bresp, buser, bvalid,
    output bready,
    output arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser, arvalid,
    input  arready,
    input  rid, rdata, rresp, rlast, ruser, rvalid,
    output rready
  );

  // Modport for Slave VIP
  modport slave (
    input  clk, rst_n,
    input  awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser, awvalid,
    output awready,
    input  wdata, wstrb, wlast, wuser, wvalid,
    output wready,
    output bid, bresp, buser, bvalid,
    input  bready,
    input  arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser, arvalid,
    output arready,
    output rid, rdata, rresp, rlast, ruser, rvalid,
    input  rready
  );

  // Modport for Passive Monitor
  modport monitor (
    input clk, rst_n,
    input awid, awaddr, awlen, awsize, awburst, awlock, awcache, awprot, awqos, awregion, awuser, awvalid, awready,
    input wdata, wstrb, wlast, wuser, wvalid, wready,
    input bid, bresp, buser, bvalid, bready,
    input arid, araddr, arlen, arsize, arburst, arlock, arcache, arprot, arqos, arregion, aruser, arvalid, arready,
    input rid, rdata, rresp, rlast, ruser, rvalid, rready
  );

endinterface : axi_if

`endif // AXI_IF_SV
