`timescale 1ns / 1ps
`default_nettype none

module simple_core_tb;
  logic clk = 1'b0;
  logic rst = 1'b1;

  // Wishbone wires
  logic [31:0] wbm_adr;
  logic [31:0] wbm_dat_o;
  logic [31:0] wbm_dat_i;
  logic        wbm_we;
  logic [3:0]  wbm_sel;
  logic        wbm_cyc;
  logic        wbm_stb;
  logic        wbm_ack;
  logic        wbm_err;

  // Debug wires
  logic [31:0] dbg_pc;
  logic [31:0] dbg_instr;

  // Clock generation
  always #5 clk = ~clk;  // 100 MHz equivalent

  // Reset sequencing
  initial begin
    repeat (5) @(posedge clk);
    rst = 1'b0;
    repeat (200) @(posedge clk);
    $display("[TB] Timeout reached without completion");
    $finish;
  end

  // DUT instance with reset vector at 0 so it fetches from our tiny memory
  rv32i_core #(
      .RESET_VECTOR(32'h0000_0000)
  ) u_dut (
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

  simple_wb_mem #(
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
      .err_o(wbm_err)
  );

  // Internal probes from the core to watch for traps/bus faults
  wire        trap_request_tap   = u_dut.trap_request;
  wire [31:0] trap_cause_tap     = u_dut.trap_cause_value;
  wire [31:0] trap_tval_tap      = u_dut.trap_tval_value;
  wire [31:0] csr_mepc_tap       = u_dut.csr_mepc;
  wire [31:0] csr_mstatus_tap    = u_dut.csr_mstatus;
  wire [31:0] csr_mcause_tap     = u_dut.csr_mcause;
  wire        id_illegal_tap     = u_dut.id_illegal;
  wire        instr_bus_err_tap  = u_dut.instr_bus_error;
  wire        data_bus_err_tap   = u_dut.mem_transaction_error;
  wire        ex_trap_valid_tap  = u_dut.ex_trap_valid;
  wire        mem_request_active_tap = u_dut.mem_request_active;
  wire        mem_busy_tap           = u_dut.mem_busy;
  wire [1:0]  bus_state_tap          = u_dut.bus_state;

  // Pipeline visibility to understand why stores might be missing
  wire        id_stage_mem_write  = u_dut.id_mem_write;
  wire        id_stage_mem_read   = u_dut.id_mem_read;
  wire        ex_stage_mem_write  = u_dut.id_ex_reg.mem_write;
  wire        ex_stage_mem_read   = u_dut.id_ex_reg.mem_read;
  wire        mem_stage_valid     = u_dut.ex_mem_reg.valid;
  wire        mem_stage_mem_write = u_dut.ex_mem_reg.mem_write;
  wire        mem_stage_mem_read  = u_dut.ex_mem_reg.mem_read;
  wire [31:0] mem_stage_addr      = u_dut.ex_mem_reg.mem_addr;
  wire [31:0] mem_stage_store_data= u_dut.ex_mem_reg.store_data;
  wire [3:0]  mem_stage_store_sel = u_dut.ex_mem_reg.store_sel;

  // Basic instrumentation: print each instruction issue and every completed bus access
  always_ff @(posedge clk) begin
    if (!rst) begin
      $display("[TB] PC=%08x INSTR=%08x CYC=%0b STB=%0b WE=%0b ACK=%0b memReq=%0b memBusy=%0b busState=%0d",
               dbg_pc, dbg_instr, wbm_cyc, wbm_stb, wbm_we, wbm_ack, mem_request_active_tap, mem_busy_tap, bus_state_tap);
      $display("[TB][PIPE] id{R=%0b,W=%0b} ex{R=%0b,W=%0b} mem{V=%0b,R=%0b,W=%0b addr=%08x data=%08x sel=%1x}",
               id_stage_mem_read, id_stage_mem_write,
               ex_stage_mem_read, ex_stage_mem_write,
               mem_stage_valid, mem_stage_mem_read, mem_stage_mem_write,
               mem_stage_addr, mem_stage_store_data, mem_stage_store_sel);
      if (wbm_cyc && wbm_stb && wbm_ack) begin
        if (wbm_we) begin
          $display("[TB][BUS] STORE addr=%08x data=%08x sel=%1x", wbm_adr, wbm_dat_o, wbm_sel);
        end else begin
          $display("[TB][BUS] LOAD  addr=%08x data=%08x", wbm_adr, wbm_dat_i);
        end
      end
    end
  end

  // Finish early once the first store completes (our mini-program writes once)
  integer store_count;
  always_ff @(posedge clk) begin
    if (rst) begin
      store_count <= 0;
    end else if (wbm_ack && wbm_we) begin
      store_count <= store_count + 1;
      if (store_count == 0) begin
        $display("[TB] Observed first store, ending simulation.");
        $finish;
      end
    end
  end

  // Trap / illegal instruction monitoring
  always_ff @(posedge clk) begin
    if (!rst) begin
      if (trap_request_tap) begin
        $display("[TB][TRAP] cause=%0d tval=%08x mepc=%08x mcause=%08x mstatus=%08x id_illegal=%0b ex_trap=%0b",
                 trap_cause_tap[4:0], trap_tval_tap, csr_mepc_tap, csr_mcause_tap, csr_mstatus_tap,
                 id_illegal_tap, ex_trap_valid_tap);
        $finish;
      end

      if (instr_bus_err_tap || data_bus_err_tap) begin
        $display("[TB][BUS-ERR] instr_err=%0b data_err=%0b adr=%08x", instr_bus_err_tap, data_bus_err_tap, wbm_adr);
        $finish;
      end
    end
  end
endmodule

// -----------------------------------------------------------------------------
// Extremely small Wishbone memory that responds in a single cycle.
// -----------------------------------------------------------------------------
module simple_wb_mem #(
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
  output logic        ack_o,
  output logic        err_o
);
  localparam int ADDR_WIDTH = $clog2(MEM_WORDS);

  logic [31:0] mem [0:MEM_WORDS-1];
  wire [ADDR_WIDTH-1:0] word_index = adr_i[ADDR_WIDTH+1:2];

  always_ff @(posedge clk) begin
    if (rst) begin
      ack_o <= 1'b0;
    end else begin
      ack_o <= cyc_i && stb_i;
      if (cyc_i && stb_i && we_i) begin
        for (int b = 0; b < 4; b++) begin
          if (sel_i[b]) begin
            mem[word_index][8*b +: 8] <= dat_i[8*b +: 8];
          end
        end
      end
    end
  end

  assign dat_o = mem[word_index];
  assign err_o = 1'b0;

  initial begin
    integer i;
    for (i = 0; i < MEM_WORDS; i++) begin
      mem[i] = 32'h00000013;  // NOP (addi x0, x0, 0)
    end
    mem[0] = 32'h05500093;  // addi x1, x0, 0x55
    mem[1] = 32'h0AA00113;  // addi x2, x0, 0xAA
    mem[2] = 32'h002081B3;  // add  x3, x1, x2
    mem[3] = 32'h00301023;  // sw   x3, 0(x0)
    mem[4] = 32'h0000006F;  // jal  x0, 0 (tight loop)
  end
endmodule

`default_nettype wire
