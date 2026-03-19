`timescale 1ns / 1ps
`default_nettype none

module uart_poll_cpu_tb;
  logic clk = 1'b0;
  logic rst = 1'b1;

  always #5 clk = ~clk;  // 100 MHz

  logic [31:0] wbm_adr;
  logic [31:0] wbm_dat_o;
  logic [31:0] wbm_dat_i;
  logic        wbm_we;
  logic [3:0]  wbm_sel;
  logic        wbm_cyc;
  logic        wbm_stb;
  logic        wbm_ack;
  logic        wbm_err;

  logic [31:0] dbg_pc;
  logic [31:0] dbg_instr;

  rv32i_core #(
    .RESET_VECTOR(32'h0000_0000)
  ) u_cpu (
    .clk       (clk),
    .rst       (rst),
    .wbm_adr_o (wbm_adr),
    .wbm_dat_o (wbm_dat_o),
    .wbm_dat_i (wbm_dat_i),
    .wbm_we_o  (wbm_we),
    .wbm_sel_o (wbm_sel),
    .wbm_cyc_o (wbm_cyc),
    .wbm_stb_o (wbm_stb),
    .wbm_ack_i (wbm_ack),
    .wbm_err_i (wbm_err),
    .dbg_pc    (dbg_pc),
    .dbg_instr (dbg_instr)
  );

  logic [31:0] last_pc;
  always_ff @(posedge clk) begin
    if (rst) begin
      last_pc <= 32'hFFFF_FFFF;
    end else if (dbg_pc != last_pc) begin
      $display("[%0t] PC=%08x instr=%08x", $time, dbg_pc, dbg_instr);
      last_pc <= dbg_pc;
    end
  end

  fake_uart_mem #(
      .MEM_WORDS(64)
  ) u_mem (
      .clk  (clk),
      .rst  (rst),
      .adr_i(wbm_adr),
      .dat_i(wbm_dat_o),
      .dat_o(wbm_dat_i),
      .we_i (wbm_we),
      .sel_i(wbm_sel),
      .cyc_i(wbm_cyc),
      .stb_i(wbm_stb),
      .ack_o(wbm_ack),
    .err_o(wbm_err),
    .dbg_pc_i(dbg_pc),
    .dbg_instr_i(dbg_instr)
  );

  initial begin
    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (2000) @(posedge clk);
    $display("[TB] Timeout waiting for simulation to finish");
    $finish;
  end
endmodule

// -----------------------------------------------------------------------------
// Wishbone memory with a fake UART that mimics the status polling semantics.
// -----------------------------------------------------------------------------
module fake_uart_mem #(
  parameter int MEM_WORDS = 64
) (
  input  wire        clk,
  input  wire        rst,
  input  wire [31:0] adr_i,
  input  wire [31:0] dat_i,
  output logic [31:0] dat_o,
  input  wire        we_i,
  input  wire [3:0]  sel_i,
  input  wire        cyc_i,
  input  wire        stb_i,
  output logic       ack_o,
  output logic       err_o,
  input  wire [31:0] dbg_pc_i,
  input  wire [31:0] dbg_instr_i
);
  localparam logic [31:0] UART_BASE   = 32'h1000_0000;
  localparam logic [31:0] UART_STATUS = 32'h1000_0005;

  logic [31:0] mem [0:MEM_WORDS-1];
  logic [31:0] read_data;
  logic [7:0]  uart_status;
  logic [15:0] uart_busy_cnt;

  wire access_uart_data   = (adr_i == UART_BASE);
  wire access_uart_status = (adr_i == UART_STATUS);
  wire mem_region         = (adr_i[31:16] == 16'h0000);
  wire [15:0] word_index  = adr_i[17:2];

  assign err_o = 1'b0;

  // ack is single-cycle
  always_ff @(posedge clk) begin
    if (rst)
      ack_o <= 1'b0;
    else
      ack_o <= cyc_i && stb_i;
  end

  always_ff @(posedge clk) begin
    if (!rst && cyc_i && stb_i && !we_i && access_uart_status) begin
      $display("[%0t] UART STATUS READ @%08x -> %02x", $time, adr_i, uart_status);
    end
  end

  // UART busy counter
  always_ff @(posedge clk) begin
    if (rst) begin
      uart_busy_cnt <= 16'd0;
      uart_status   <= 8'h20;  // ready by default
    end else begin
      if (uart_busy_cnt != 0) begin
        uart_busy_cnt <= uart_busy_cnt - 1'b1;
        if (uart_busy_cnt == 16'd1)
          uart_status[5] <= 1'b1;
      end

      if (cyc_i && stb_i && we_i && access_uart_data && sel_i[0]) begin
        uart_busy_cnt <= 16'd200;
        uart_status[5] <= 1'b0;
  $display("[%0t] UART WRITE char=%0d (%c) pc=%08x instr=%08x",
     $time, dat_i[7:0], dat_i[7:0], dbg_pc_i, dbg_instr_i);
      end
    end
  end

  // Memory write path
  always_ff @(posedge clk) begin
    if (rst) begin
      integer idx;
      for (idx = 0; idx < MEM_WORDS; idx++) begin
        mem[idx] <= 32'h00000013;  // NOP
      end
      // 完整的"done!"输出程序
      // 初始化
      mem[0]  <= 32'h100002B7;  // lui t0,0x10000  // UART基地址
      
      // 输出'd'
      mem[1]  <= 32'h06400513;  // addi a0,x0,'d'
      mem[2]  <= 32'h00A28023;  // sb a0,0(t0)
      mem[3]  <= 32'h00528303;  // lb t1,5(t0)
      mem[4]  <= 32'h02037313;  // andi t1,t1,0x20
      mem[5]  <= 32'hFE030CE3;  // beq t1,x0,-8  // 等待UART就绪 (回到lb指令)
      
      // 输出'o'
      mem[6]  <= 32'h06F00513;  // addi a0,x0,'o'
      mem[7]  <= 32'h00A28023;  // sb a0,0(t0)
      mem[8]  <= 32'h00528303;  // lb t1,5(t0)
      mem[9]  <= 32'h02037313;  // andi t1,t1,0x20
      mem[10] <= 32'hFE030CE3;  // beq t1,x0,-8  // 等待UART就绪 (回到lb指令)
      
      // 输出'n'
      mem[11] <= 32'h06E00513;  // addi a0,x0,'n'
      mem[12] <= 32'h00A28023;  // sb a0,0(t0)
      mem[13] <= 32'h00528303;  // lb t1,5(t0)
      mem[14] <= 32'h02037313;  // andi t1,t1,0x20
      mem[15] <= 32'hFE030CE3;  // beq t1,x0,-8  // 等待UART就绪 (回到lb指令)
      
      // 输出'e'
      mem[16] <= 32'h06500513;  // addi a0,x0,'e'
      mem[17] <= 32'h00A28023;  // sb a0,0(t0)
      mem[18] <= 32'h00528303;  // lb t1,5(t0)
      mem[19] <= 32'h02037313;  // andi t1,t1,0x20
      mem[20] <= 32'hFE030CE3;  // beq t1,x0,-8  // 等待UART就绪 (回到lb指令)
      
      // 输出'!'
      mem[21] <= 32'h02100513;  // addi a0,x0,'!'
      mem[22] <= 32'h00A28023;  // sb a0,0(t0)
      mem[23] <= 32'h00528303;  // lb t1,5(t0)
      mem[24] <= 32'h02037313;  // andi t1,t1,0x20
      mem[25] <= 32'hFE030CE3;  // beq t1,x0,-8  // 等待UART就绪 (回到lb指令)
      
      // 无限循环
      mem[26] <= 32'h0000006F;  // j .  // 无限循环
    end else if (cyc_i && stb_i && we_i && mem_region) begin
      if (word_index < MEM_WORDS) begin
        for (int b = 0; b < 4; b++) begin
          if (sel_i[b]) begin
            mem[word_index][8*b +: 8] <= dat_i[8*b +: 8];
          end
        end
      end
    end
  end

  // Read mux
  always_comb begin
    if (access_uart_status) begin
      read_data = {4{uart_status}};
    end else if (mem_region && word_index < MEM_WORDS) begin
      read_data = mem[word_index];
    end else begin
      read_data = 32'hDEAD_BEEF;
    end
  end

  assign dat_o = read_data;
endmodule

`default_nettype wire
