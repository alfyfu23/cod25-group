`timescale 1ns/1ps
`default_nettype none

module rv32i_regfile (
  input  wire        clk,
  input  wire        rst,
  input  wire        we,
  input  wire [4:0]  waddr,
  input  wire [31:0] wdata,
  input  wire [4:0]  raddr1,
  input  wire [4:0]  raddr2,
  output logic [31:0] rdata1,
  output logic [31:0] rdata2
);

  logic [31:0] regs[31:0];
  integer i;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      for (i = 0; i < 32; i = i + 1) begin
        regs[i] <= 32'h0;
      end
    end else if (we && (waddr != 5'd0)) begin
      regs[waddr] <= wdata;
    end
  end

  always_comb begin
    rdata1 = (raddr1 == 5'd0) ? 32'h0 : regs[raddr1];
    rdata2 = (raddr2 == 5'd0) ? 32'h0 : regs[raddr2];
  end

endmodule

`default_nettype wire
