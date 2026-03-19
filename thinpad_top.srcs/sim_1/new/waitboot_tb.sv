`timescale 1ns / 1ps
`default_nettype none

module waitboot_tb;
  localparam string BASE_INIT_FILE       = "";  // override via -P waitboot_tb.BASE_INIT_FILE="/abs/path"
  localparam string BASE_INIT_FALLBACK_0 = "thinpad_top.srcs/sim_1/new/programs/waitboot_monitor.hex";
  localparam string BASE_INIT_FALLBACK_1 = "../../../../thinpad_top.srcs/sim_1/new/programs/waitboot_monitor.hex";
  localparam string BASE_INIT_FALLBACK_2 = "thinpad_top.srcs/sim_1/new/programs/waitboot_stub.hex";
  localparam string BASE_INIT_FALLBACK_3 = "../../../../thinpad_top.srcs/sim_1/new/programs/waitboot_stub.hex";
  localparam string EXT_INIT_FILE        = "";
  localparam int    TIMEOUT_CYCLES = 200000;  // ~20 us @ 10 MHz
  localparam string EXPECTED_BANNER = "WaitBoot";

  logic clk = 1'b0;
  logic rst = 1'b1;

  always #50 clk = ~clk;  // 10 MHz clock matches PLL output in top-level

  initial begin
    repeat (10) @(posedge clk);
    rst = 1'b0;
  end

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

  logic [31:0] dbg_pc;
  logic [31:0] dbg_instr;

  rv32i_core #(
      .RESET_VECTOR(32'h8000_0000)
  ) u_core (
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

  logic        uart_char_valid;
  logic [7:0]  uart_char;

  waitboot_env #(
    .BASE_INIT_FILE(BASE_INIT_FILE),
    .BASE_FALLBACK0(BASE_INIT_FALLBACK_0),
    .BASE_FALLBACK1(BASE_INIT_FALLBACK_1),
      .EXT_INIT_FILE (EXT_INIT_FILE)
  ) u_env (
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
      .uart_char_valid_o(uart_char_valid),
      .uart_char_o      (uart_char)
  );

  int cycle_counter;
  int match_index;
  int next_index_calc;
  int banner_len;
  bit banner_seen;

  // Core-internal probes to dump richer traces while debugging WaitBoot
  wire        trap_request_tap      = u_core.trap_request;
  wire [31:0] trap_cause_tap        = u_core.trap_cause_value;
  wire [31:0] trap_tval_tap         = u_core.trap_tval_value;
  wire [31:0] csr_mepc_tap          = u_core.csr_mepc;
  wire        mem_request_active_tap= u_core.mem_request_active;
  wire        mem_busy_tap          = u_core.mem_busy;
  wire [1:0]  bus_state_tap         = u_core.bus_state;
  wire        id_mem_read_tap       = u_core.id_mem_read;
  wire        id_mem_write_tap      = u_core.id_mem_write;
  wire        ex_mem_read_tap       = u_core.id_ex_reg.mem_read;
  wire        ex_mem_write_tap      = u_core.id_ex_reg.mem_write;

  initial banner_len = EXPECTED_BANNER.len();

  always_ff @(posedge clk) begin
    if (rst) begin
      cycle_counter <= 0;
      match_index   <= 0;
      banner_seen   <= 1'b0;
    end else begin
      cycle_counter <= cycle_counter + 1;
      if (uart_char_valid) begin
        if (uart_char >= 8'h20 && uart_char <= 8'h7E)
          $display("[UART] TX '%c' (0x%02x)", uart_char, uart_char);
        else
          $display("[UART] TX 0x%02x", uart_char);

        if (!banner_seen) begin
          byte expected_char;
          if (match_index < banner_len)
            expected_char = EXPECTED_BANNER[match_index];
          else
            expected_char = EXPECTED_BANNER[0];

          if ((match_index < banner_len) && (uart_char == expected_char)) begin
            next_index_calc = match_index + 1;
            match_index <= next_index_calc;
            if (next_index_calc == banner_len) begin
              banner_seen <= 1'b1;
              $display("[TB] WaitBoot banner observed at cycle %0d", cycle_counter);
            end
          end else if (uart_char == EXPECTED_BANNER[0]) begin
            match_index <= (banner_len > 1) ? 1 : 0;
          end else begin
            match_index <= 0;
          end
        end
      end
    end
  end

  // Wishbone-level instrumentation: dump every acknowledged transaction
  always_ff @(posedge clk) begin
    if (!rst && wbm_cyc && wbm_stb && wbm_ack) begin
      if (wbm_we) begin
        $display("[BUS] STORE addr=%08x data=%08x sel=%1x pc=%08x instr=%08x", wbm_adr, wbm_dat_o, wbm_sel, dbg_pc, dbg_instr);
      end else begin
        $display("[BUS] LOAD  addr=%08x data=%08x pc=%08x instr=%08x", wbm_adr, wbm_dat_i, dbg_pc, dbg_instr);
      end
    end
  end

  // Quick snapshot of pipeline demand each cycle (one line for easy sharing)
  always_ff @(posedge clk) begin
    if (!rst) begin
      $display("[PIPE] cyc=%0b stb=%0b we=%0b ack=%0b memReq=%0b memBusy=%0b busState=%0d id{R=%0b,W=%0b} ex{R=%0b,W=%0b} pc=%08x instr=%08x",
               wbm_cyc, wbm_stb, wbm_we, wbm_ack, mem_request_active_tap, mem_busy_tap, bus_state_tap,
               id_mem_read_tap, id_mem_write_tap, ex_mem_read_tap, ex_mem_write_tap,
               dbg_pc, dbg_instr);
    end
  end

  // Surface traps immediately so remote debugging has the cause
  always_ff @(posedge clk) begin
    if (!rst && trap_request_tap) begin
      $display("[TRAP] cause=%0d tval=%08x mepc=%08x (cycle=%0d)", trap_cause_tap[4:0], trap_tval_tap, csr_mepc_tap, cycle_counter);
    end
  end

  initial begin
    wait (!rst);
    repeat (TIMEOUT_CYCLES) @(posedge clk);
    if (!banner_seen) begin
      $fatal(1, "[TB] Timeout waiting for WaitBoot banner (matched %0d/%0d characters)", match_index, banner_len);
    end else begin
      $display("[TB] Simulation reached banner within timeout (cycles=%0d)", cycle_counter);
    end
    $finish;
  end

  always_ff @(posedge clk) begin
    if (!rst && banner_seen) begin
      $display("[TB] Test completed successfully at cycle %0d", cycle_counter);
      $finish;
    end
  end
endmodule

// -----------------------------------------------------------------------------
// Lightweight Wishbone environment that mimics BaseRAM/ExtRAM + UART window
// used by the WaitBoot/monitor test flow.
// -----------------------------------------------------------------------------
module waitboot_env #(
  parameter string BASE_INIT_FILE = "",
  parameter string BASE_FALLBACK0 = "",
  parameter string BASE_FALLBACK1 = "",
  parameter string EXT_INIT_FILE  = ""
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
  output logic        err_o,
  output logic        uart_char_valid_o,
  output logic [7:0]  uart_char_o
);
  localparam logic [31:0] BASE_START = 32'h8000_0000;
  localparam logic [31:0] EXT_START  = 32'h8040_0000;
  localparam logic [31:0] MEM_WINDOW = 32'h0040_0000; // 4 MB
  localparam logic [31:0] UART_ADDR  = 32'h1000_0000;

  localparam int BASE_WORDS = MEM_WINDOW >> 2;  // 1M words per bank

  logic [31:0] base_mem [0:BASE_WORDS-1];
  logic [31:0] ext_mem  [0:BASE_WORDS-1];

  typedef enum logic [0:0] {
    WB_IDLE,
    WB_RESP
  } wb_state_e;

  wb_state_e state;
  logic [31:0] latched_addr;
  logic [31:0] latched_wdata;
  logic [3:0]  latched_sel;
  logic        latched_we;

  logic [31:0] read_data_reg;
  logic        uart_char_valid;
  logic [7:0]  uart_char;

  assign dat_o = read_data_reg;
  assign uart_char_valid_o = uart_char_valid;
  assign uart_char_o       = uart_char;

  function automatic bit file_exists(input string path);
    int fd;
    if (path == "") begin
      return 1'b0;
    end
    fd = $fopen(path, "r");
    if (fd != 0) begin
      $fclose(fd);
      return 1'b1;
    end
    return 1'b0;
  endfunction

  function automatic string choose_image_path(
      input string preferred,
      input string fallback0,
      input string fallback1);
    if (file_exists(preferred)) begin
      return preferred;
    end
    if (file_exists(fallback0)) begin
      $display("[ENV] Preferred image '%s' missing, using fallback '%s'", preferred, fallback0);
      return fallback0;
    end
    if (file_exists(fallback1)) begin
      $display("[ENV] Preferred image '%s' missing, using second fallback '%s'", preferred, fallback1);
      return fallback1;
    end
    return "";
  endfunction

  function automatic string choose_base_image();
    string candidates[$];
    string chosen;
    bit    override;

    override = (BASE_INIT_FILE != "");

    candidates.push_back(BASE_INIT_FILE);
    candidates.push_back(BASE_INIT_FALLBACK_0);
    candidates.push_back(BASE_INIT_FALLBACK_1);
    candidates.push_back(BASE_INIT_FALLBACK_2);
    candidates.push_back(BASE_INIT_FALLBACK_3);

    foreach (candidates[idx]) begin
      if ((candidates[idx] != "") && file_exists(candidates[idx])) begin
        chosen = candidates[idx];
        if (override && (idx > 0)) begin
          $display("[ENV] Preferred image '%s' missing, using fallback '%s'", BASE_INIT_FILE, chosen);
        end else if (!override) begin
          if (idx <= 2) begin
            $display("[ENV] Using bundled monitor image '%s'", chosen);
          end else begin
            $display("[ENV] Using bundled stub image '%s'", chosen);
          end
        end
        return chosen;
      end
    end

    return "";
  endfunction

  function automatic logic in_base(input logic [31:0] addr);
    return (addr >= BASE_START) && (addr < (BASE_START + MEM_WINDOW));
  endfunction

  function automatic logic in_ext(input logic [31:0] addr);
    return (addr >= EXT_START) && (addr < (EXT_START + MEM_WINDOW));
  endfunction

  function automatic int unsigned base_index(input logic [31:0] addr);
    return (addr - BASE_START) >> 2;
  endfunction

  function automatic int unsigned ext_index(input logic [31:0] addr);
    return (addr - EXT_START) >> 2;
  endfunction

  task automatic write_mem(
      input logic [31:0] addr,
      input logic [31:0] data,
      input logic [3:0]  sel,
      output logic       err, 
      output logic       uart_valid,
      output logic [7:0] uart_data);
    err        = 1'b0;
    uart_valid = 1'b0;
    uart_data  = 8'h00;

    if (in_base(addr)) begin
      int unsigned idx = base_index(addr);
      if (idx < BASE_WORDS) begin
        for (int b = 0; b < 4; b++) begin
          if (sel[b]) begin
            base_mem[idx][8*b +: 8] = data[8*b +: 8];
          end
        end
      end else begin
        err = 1'b1;
      end
    end else if (in_ext(addr)) begin
      int unsigned idx = ext_index(addr);
      if (idx < BASE_WORDS) begin
        for (int b = 0; b < 4; b++) begin
          if (sel[b]) begin
            ext_mem[idx][8*b +: 8] = data[8*b +: 8];
          end
        end
      end else begin
        err = 1'b1;
      end
    end else if (addr[31:4] == UART_ADDR[31:4]) begin
      for (int b = 0; b < 4; b++) begin
        if (sel[b]) begin
          uart_valid = 1'b1;
          uart_data  = data[8*b +: 8];
        end
      end
    end else begin
      err = 1'b1;
    end
  endtask

  function automatic logic [31:0] read_mem(
      input logic [31:0] addr,
      output logic       err);
    logic [31:0] data;
    err = 1'b0;

    if (in_base(addr)) begin
      int unsigned idx = base_index(addr);
      if (idx < BASE_WORDS)
        data = base_mem[idx];
      else begin
        data = 32'hDEAD_BEEF;
        err = 1'b1;
      end
    end else if (in_ext(addr)) begin
      int unsigned idx = ext_index(addr);
      if (idx < BASE_WORDS)
        data = ext_mem[idx];
      else begin
        data = 32'hDEAD_BEEF;
        err = 1'b1;
      end
    end else if (addr[31:4] == UART_ADDR[31:4]) begin
      data = 32'h0000_0000;
    end else begin
      data = 32'hDEAD_BEEF;
      err  = 1'b1;
    end

    return data;
  endfunction

  always_ff @(posedge clk) begin
    if (rst) begin
  state            <= WB_IDLE;
  ack_o            <= 1'b0;
  err_o            <= 1'b0;
  read_data_reg    <= 32'h0;
  uart_char_valid  <= 1'b0;
  uart_char        <= 8'h00;
    end else begin
      ack_o           <= 1'b0;
      err_o           <= 1'b0;
      uart_char_valid <= 1'b0;

      case (state)
        WB_IDLE: begin
          if (cyc_i && stb_i) begin
            latched_addr  <= adr_i;
            latched_wdata <= dat_i;
            latched_sel   <= sel_i;
            latched_we    <= we_i;
            state         <= WB_RESP;
          end
        end

        WB_RESP: begin
          ack_o <= 1'b1;
          if (latched_we) begin
            logic write_err;
            logic char_valid;
            logic [7:0] char_data;
            write_mem(latched_addr, latched_wdata, latched_sel, write_err, char_valid, char_data);
            err_o           <= write_err;
            uart_char_valid <= char_valid;
            uart_char       <= char_data;
            read_data_reg   <= 32'h0000_0000;
          end else begin
            logic read_err;
            read_data_reg <= read_mem(latched_addr, read_err);
            err_o         <= read_err;
          end
          state <= WB_IDLE;
        end
      endcase
    end
  end

  initial begin
    string base_path;
    string ext_path;

    for (int i = 0; i < BASE_WORDS; i++) begin
      base_mem[i] = 32'h0000_0013;  // NOP
      ext_mem[i]  = 32'h0000_0013;
    end

    base_path = choose_base_image();
    if (base_path != "") begin
      $display("[ENV] Loading BaseRAM contents from %s", base_path);
      $readmemh(base_path, base_mem);
    end else begin
      $fatal(1, "[ENV] Unable to open any BaseRAM init file. Tried '%s', '%s', '%s'", BASE_INIT_FILE, BASE_FALLBACK0, BASE_FALLBACK1);
    end

    ext_path = choose_image_path(EXT_INIT_FILE, "", "");
    if ((EXT_INIT_FILE != "") && (ext_path == "")) begin
      $fatal(1, "[ENV] Unable to open ExtRAM init file '%s'", EXT_INIT_FILE);
    end else if (ext_path != "") begin
      $display("[ENV] Loading ExtRAM contents from %s", ext_path);
      $readmemh(ext_path, ext_mem);
    end
  end
endmodule

`default_nettype wire
