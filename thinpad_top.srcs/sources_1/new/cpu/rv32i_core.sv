// 时间单位和精度设置
`timescale 1ns/1ps
// 禁止默认网络类型，避免隐式声明
`default_nettype none

// =============================================================================
// RV32I CPU 核心实现
// =============================================================================
// 本模块实现了一个 5 级流水线的 RISC-V CPU 核心 (RV32I)。
// 主要特性：
// - 5 级流水线：取指(IF)、译码(ID)、执行(EX)、访存(MEM)、写回(WB)
// - 数据前推单元用于解决数据冒险
// - 冒险检测单元用于解决加载-使用冒险和控制冒险
// - 使用 BTB (分支目标缓冲) 进行分支预测
// - 带有 TLB 和硬件页表遍历器的 MMU (支持 Sv32 分页)
// - Wishbone 总线接口用于内存和 I/O 访问
// - 支持 M 模式和 U 模式的 CSR (控制和状态寄存器)
// - 异常和中断处理机制
// =============================================================================

// RV32I CPU核心模块
module rv32i_core #(
  // 复位向量，CPU启动时从该地址开始执行指令
  parameter [31:0] RESET_VECTOR = 32'h8000_0000
) (
  // 时钟和复位信号
  input  wire        clk,              // 时钟信号
  input  wire        rst,              // 复位信号

  // Wishbone主接口，用于与内存和外设通信
  output logic [31:0] wbm_adr_o,       // 地址输出
  output logic [31:0] wbm_dat_o,       // 数据输出
  input  wire [31:0] wbm_dat_i,       // 数据输入
  output logic        wbm_we_o,        // 写使能输出
  output logic [3:0]  wbm_sel_o,       // 字节选择输出
  output logic        wbm_cyc_o,       // 周期输出
  output logic        wbm_stb_o,       // 选通输出
  input  wire        wbm_ack_i,        // 应答输入
  input  wire        wbm_err_i,        // 错误输入

  // 中断输入信号
  input  wire        external_interrupt_i,  // 外部中断
  input  wire        timer_interrupt_i,     // 定时器中断

  // 调试接口
  output logic [31:0] dbg_pc,           // 当前程序计数器值
  output logic [31:0] dbg_instr         // 当前指令值
);

  // ------------------------------------------------------------
  // 枚举类型和辅助类型定义
  // ------------------------------------------------------------

  // ALU操作类型枚举
  typedef enum logic [5:0] {
    ALU_ADD,     // 加法
    ALU_SUB,     // 减法
    ALU_SLL,     // 逻辑左移
    ALU_SLT,     // 有符号比较小于
    ALU_SLTU,    // 无符号比较小于
    ALU_XOR,     // 异或
    ALU_SRL,     // 逻辑右移
    ALU_SRA,     // 算术右移
    ALU_OR,      // 或
    ALU_AND,     // 与
    ALU_PASS_B,  // 直接传递第二个操作数
    ALU_MINU,    // 无符号最小值
    ALU_XNOR,    // 同或
    ALU_CTZ      // 计算尾随零的数量
  } alu_op_e;

  // 分支类型枚举
  typedef enum logic [2:0] {
    BR_NONE,     // 无分支
    BR_EQ,       // 相等分支
    BR_NE,       // 不等分支
    BR_LT,       // 有符号小于分支
    BR_GE,       // 有符号大于等于分支
    BR_LTU,      // 无符号小于分支
    BR_GEU       // 无符号大于等于分支
  } branch_e;

  // ALU第一个操作数选择枚举
  typedef enum logic [1:0] {
    OP_A_RS1,    // 选择rs1寄存器值
    OP_A_PC,     // 选择程序计数器PC
    OP_A_ZERO    // 选择零值
  } op_a_sel_e;

  // ALU第二个操作数选择枚举
  typedef enum logic [2:0] {
    OP_B_RS2,    // 选择rs2寄存器值
    OP_B_IMM,    // 选择立即数
    OP_B_FOUR    // 选择常数4
  } op_b_sel_e;

  // 写回数据源选择枚举
  typedef enum logic [1:0] {
    WB_SRC_ALU,  // 来自ALU结果
    WB_SRC_MEM,  // 来自内存读取数据
    WB_SRC_PC4,  // 来自PC+4
    WB_SRC_CSR   // 来自CSR寄存器
  } wb_src_e;

  // 内存访问大小枚举
  typedef enum logic [1:0] {
    MEM_SIZE_BYTE,   // 字节访问
    MEM_SIZE_HALF,   // 半字访问
    MEM_SIZE_WORD    // 字访问
  } mem_size_e;

  // CSR操作类型枚举
  typedef enum logic [1:0] {
    CSR_OP_WRITE,    // 写入操作
    CSR_OP_SET,      // 置位操作
    CSR_OP_CLEAR     // 清位操作
  } csr_op_e;

  // 总线状态枚举
  typedef enum logic [2:0] {
    BUS_IDLE,        // 空闲状态
    BUS_IFETCH,      // 指令获取状态
    BUS_MEM,         // 内存访问状态
    BUS_WALK_L1,     // 页表第一级遍历
    BUS_WALK_L2      // 页表第二级遍历
  } bus_state_e;

  // IF/ID流水线寄存器结构体定义
  typedef struct packed {
    logic        valid;         // 有效标志
    logic [31:0] pc;           // 程序计数器
    logic [31:0] instr;        // 指令码
    logic        pred_taken;    // 分支预测是否跳转
    logic [31:0] pred_target;  // 预测的目标地址
  } if_id_reg_t;

  // ID/EX流水线寄存器结构体定义
  typedef struct packed {
    logic        valid;         // 有效标志
    logic [31:0] pc;           // 程序计数器
    logic [31:0] instr;        // 指令码
    logic [31:0] imm;          // 立即数
    logic [4:0]  rd;           // 目标寄存器地址
    logic [4:0]  rs1;          // 源寄存器1地址
    logic [4:0]  rs2;          // 源寄存器2地址
    logic [31:0] rs1_value;    // 源寄存器1的值
    logic [31:0] rs2_value;    // 源寄存器2的值
    op_a_sel_e   op_a_sel;     // ALU第一个操作数选择
    op_b_sel_e   op_b_sel;     // ALU第二个操作数选择
    alu_op_e     alu_op;       // ALU操作类型
    branch_e     branch;       // 分支类型
    logic        jump;         // 是否为跳转指令
    logic        jalr;         // 是否为JALR指令
    logic        mem_read;     // 是否需要内存读取
    logic        mem_write;    // 是否需要内存写入
    mem_size_e   mem_size;     // 内存访问大小
    logic        mem_unsigned; // 是否为无符号内存访问
    logic        reg_write;    // 是否需要寄存器写回
    wb_src_e     wb_src;       // 写回数据源
    logic        csr_en;       // CSR操作使能
    csr_op_e     csr_op;       // CSR操作类型
    logic        csr_imm;      // CSR操作是否使用立即数
    logic [11:0] csr_addr;     // CSR地址
    logic [31:0] csr_rdata;    // CSR读取数据
    logic [31:0] csr_operand;  // CSR操作数
    logic        is_ecall;     // 是否为ECALL指令
    logic        is_ebreak;    // 是否为EBREAK指令
    logic        is_mret;      // 是否为MRET指令
    logic        is_fencei;    // 是否为FENCE.I指令
    logic        is_sfence_vma; // 是否为SFENCE.VMA指令
    logic        illegal;      // 是否为非法指令
    logic        use_rs1;      // 是否使用rs1
    logic        use_rs2;      // 是否使用rs2
    logic        pred_taken;    // 分支预测是否跳转
    logic [31:0] pred_target;  // 预测的目标地址
  } id_ex_reg_t;

  // EX/MEM流水线寄存器结构体定义
  typedef struct packed {
    logic        valid;         // 有效标志
    logic [31:0] pc;           // 程序计数器
    logic [31:0] pc_plus4;     // PC+4值
    logic [4:0]  rd;           // 目标寄存器地址
    logic        reg_write;    // 是否需要寄存器写回
    wb_src_e     wb_src;       // 写回数据源
    logic        mem_read;     // 是否需要内存读取
    logic        mem_write;    // 是否需要内存写入
    mem_size_e   mem_size;     // 内存访问大小
    logic        mem_unsigned; // 是否为无符号内存访问
    logic [31:0] alu_result;   // ALU计算结果
    logic [31:0] rs2_value;    // 源寄存器2的值
    logic [31:0] branch_target; // 分支目标地址
    logic        branch_taken;  // 是否实际跳转
    logic        jump;         // 是否为跳转指令
    logic        jalr;         // 是否为JALR指令
    logic        csr_en;       // CSR操作使能
    logic        csr_write_en; // CSR写使能
    logic [31:0] csr_wdata;    // CSR写入数据
    logic [31:0] csr_rdata;    // CSR读取数据
    logic [11:0] csr_addr;     // CSR地址
    logic        is_ecall;     // 是否为ECALL指令
    logic        is_ebreak;    // 是否为EBREAK指令
    logic        is_mret;      // 是否为MRET指令
    logic        is_fencei;    // 是否为FENCE.I指令
    logic        is_sfence_vma; // 是否为SFENCE.VMA指令
    logic        illegal;      // 是否为非法指令
    logic        trap_valid;   // 是否有陷阱
    logic [4:0]  trap_cause;   // 陷阱原因
    logic [31:0] trap_value;  // 陷阱值
    logic [31:0] mem_addr;     // 内存访问地址
    logic [3:0]  store_sel;    // 存储字节选择
    logic [31:0] store_data;   // 存储数据
  } ex_mem_reg_t;

  // MEM/WB流水线寄存器结构体定义
  typedef struct packed {
    logic        valid;         // 有效标志
    logic [4:0]  rd;           // 目标寄存器地址
    logic        reg_write;    // 是否需要寄存器写回
    wb_src_e     wb_src;       // 写回数据源
    logic [31:0] alu_result;   // ALU计算结果
    logic [31:0] mem_data;     // 内存读取数据
    logic [31:0] pc_plus4;     // PC+4值
    logic [31:0] csr_data;     // CSR数据
  } mem_wb_reg_t;

  // 复位值在逻辑中用'0替代


  // ------------------------------------------------------------
  // 操作码和CSR常量的局部参数定义
  // ------------------------------------------------------------

  // RV32I指令操作码定义
  localparam [6:0] OPCODE_OP        = 7'b0110011;  // R-type算术逻辑指令
  localparam [6:0] OPCODE_OP_IMM    = 7'b0010011;  // I-type算术逻辑立即数指令
  localparam [6:0] OPCODE_LUI       = 7'b0110111;  // U-type加载立即数到高位指令
  localparam [6:0] OPCODE_AUIPC     = 7'b0010111;  // U-typePC相对加载立即数到高位指令
  localparam [6:0] OPCODE_JAL       = 7'b1101111;  // J-type跳转并链接指令
  localparam [6:0] OPCODE_JALR      = 7'b1100111;  // I-type寄存器跳转并链接指令
  localparam [6:0] OPCODE_BRANCH    = 7'b1100011;  // B-type分支指令
  localparam [6:0] OPCODE_LOAD      = 7'b0000011;  // I-type加载指令
  localparam [6:0] OPCODE_STORE     = 7'b0100011;  // S-type存储指令
  localparam [6:0] OPCODE_MISC_MEM  = 7'b0001111;  // 内存相关指令
  localparam [6:0] OPCODE_SYSTEM    = 7'b1110011;  // 系统指令和CSR指令

  // CSR MISA寄存器值，表示支持RV32I和扩展B(位操作)
  localparam [31:0] CSR_MISA_VALUE = 32'h4000_0102;  // RV32IB

  // 异常和中断原因代码定义
  localparam [4:0] CAUSE_INSTR_ADDR_MISALIGNED = 5'd0;  // 指令地址未对齐
  localparam [4:0] CAUSE_INSTR_ACCESS_FAULT    = 5'd1;  // 指令访问错误
  localparam [4:0] CAUSE_ILLEGAL_INSTRUCTION   = 5'd2;  // 非法指令
  localparam [4:0] CAUSE_BREAKPOINT            = 5'd3;  // 断点
  localparam [4:0] CAUSE_LOAD_ADDR_MISALIGNED  = 5'd4;  // 加载地址未对齐
  localparam [4:0] CAUSE_LOAD_ACCESS_FAULT     = 5'd5;  // 加载访问错误
  localparam [4:0] CAUSE_STORE_ADDR_MISALIGNED = 5'd6;  // 存储地址未对齐
  localparam [4:0] CAUSE_STORE_ACCESS_FAULT    = 5'd7;  // 存储访问错误
  localparam [4:0] CAUSE_M_TIMER_INT           = 5'd7;  // 机器定时器中断(中断位为1)
  localparam [4:0] CAUSE_M_EXTERNAL_INT        = 5'd11; // 机器外部中断
  // ECALL 根据当前特权级别有不同的原因码，这是RISC-V规范的要求
  localparam [4:0] CAUSE_ECALL_U               = 5'd8;   // U-mode环境调用
  localparam [4:0] CAUSE_ECALL_S               = 5'd9;   // S-mode环境调用
  localparam [4:0] CAUSE_ECALL_M               = 5'd11;  // M-mode环境调用
  localparam [4:0] CAUSE_INSTR_PAGE_FAULT      = 5'd12; // 指令页错误
  localparam [4:0] CAUSE_LOAD_PAGE_FAULT       = 5'd13; // 加载页错误
  localparam [4:0] CAUSE_STORE_PAGE_FAULT      = 5'd15; // 存储页错误

  // ------------------------------------------------------------
  // 流水线寄存器和状态
  // ------------------------------------------------------------

  // 流水线寄存器实例
  if_id_reg_t  if_id_reg;  // IF/ID流水线寄存器
  id_ex_reg_t  id_ex_reg;  // ID/EX流水线寄存器
  ex_mem_reg_t ex_mem_reg; // EX/MEM流水线寄存器
  mem_wb_reg_t mem_wb_reg; // MEM/WB流水线寄存器

  // 取指/PC状态
  logic [31:0] pc_reg;     // 当前程序计数器值

  // ------------------------------------------------------------
  // 指令缓存(I-Cache)
  // ------------------------------------------------------------
  
  // I-Cache参数定义
  localparam ICACHE_INDEX_BITS = 7; // 索引位宽，128行
  localparam ICACHE_TAG_BITS = 32 - 2 - ICACHE_INDEX_BITS; // 标签位宽
  
  // I-Cache存储数组
  logic [31:0] icache_data [0:(1<<ICACHE_INDEX_BITS)-1];      // 缓存数据
  logic [ICACHE_TAG_BITS-1:0] icache_tag [0:(1<<ICACHE_INDEX_BITS)-1]; // 缓存标签
  logic icache_valid [0:(1<<ICACHE_INDEX_BITS)-1];              // 缓存有效位
  
  // I-Cache控制信号
  logic [ICACHE_INDEX_BITS-1:0] icache_index;   // 当前访问的缓存索引
  logic [ICACHE_TAG_BITS-1:0]   icache_req_tag; // 当前请求的标签
  logic        icache_hit;                     // 缓存命中信号
  logic        flush_icache;                   // 缓存刷新信号

  // I-Cache地址解码和命中判断
  assign icache_index   = pc_reg[ICACHE_INDEX_BITS+1:2]; // 从PC中提取索引位
  assign icache_req_tag = pc_reg[31:ICACHE_INDEX_BITS+2]; // 从PC中提取标签位
  assign icache_hit     = icache_valid[icache_index] && (icache_tag[icache_index] == icache_req_tag); // 判断是否命中

  // ------------------------------------------------------------
  // 分支预测器(BTB)
  // ------------------------------------------------------------
  
  // BTB参数定义
  localparam BTB_SIZE = 64;      // BTB条目数
  localparam BTB_INDEX_BITS = 6; // BTB索引位宽
  localparam BTB_TAG_BITS = 32 - 2 - BTB_INDEX_BITS; // BTB标签位宽

  // BTB条目结构体定义
  typedef struct packed {
    logic        valid;                    // 有效标志
    logic [BTB_TAG_BITS-1:0] tag;         // 标签
    logic [31:0] target;                  // 目标地址
    logic [1:0]  counter;                 // 2位饱和计数器
  } btb_entry_t;

  // BTB存储数组
  btb_entry_t btb [BTB_SIZE-1:0]; // BTB条目数组

  // BTB控制信号
  logic [BTB_INDEX_BITS-1:0] if_btb_index;    // BTB索引
  logic [BTB_TAG_BITS-1:0]   if_btb_tag;      // BTB标签
  logic        btb_hit;                         // BTB命中信号
  logic        btb_pred_taken;                  // BTB预测跳转信号
  logic [31:0] btb_pred_target;                 // BTB预测目标地址

  // BTB地址解码和命中判断
  assign if_btb_index = pc_reg[BTB_INDEX_BITS+1:2]; // 从PC中提取BTB索引
  assign if_btb_tag   = pc_reg[31:BTB_INDEX_BITS+2]; // 从PC中提取BTB标签
  
  assign btb_hit = btb[if_btb_index].valid && (btb[if_btb_index].tag == if_btb_tag); // BTB命中判断
  assign btb_pred_taken = btb_hit && btb[if_btb_index].counter[1]; // 预测跳转(计数器最高位为1)
  assign btb_pred_target = btb[if_btb_index].target; // 预测目标地址

  // 取指/PC状态
  // logic [31:0] pc_reg; // 已移至顶部
  logic [31:0] fetch_pc_inflight;    // 正在取指的PC
  logic [31:0] fetch_data_instr;     // 取回的指令数据
  logic [31:0] fetch_data_pc;        // 取回指令对应的PC
  logic        fetch_data_valid;      // 取回指令有效标志
  logic        fetch_inflight;        // 正在取指标志
  logic        fetch_discard;         // 丢弃取回指令标志

  logic        fetch_pred_taken;      // 取指时的预测跳转标志
  logic [31:0] fetch_pred_target;    // 取指时的预测目标地址
  logic        fetch_data_pred_taken; // 取回指令的预测跳转标志
  logic [31:0] fetch_data_pred_target;// 取回指令的预测目标地址

  // TLB和页表遍历器
  logic        itlb_valid;            // 指令TLB有效标志
  logic [19:0] itlb_vpn;             // 指令TLB虚拟页号
  logic [19:0] itlb_ppn;             // 指令TLB物理页号
  logic        itlb_allow_user;      // 指令TLB允许用户模式访问
  logic        itlb_allow_exec;       // 指令TLB允许执行
  
  logic        dtlb_valid;            // 数据TLB有效标志
  logic [19:0] dtlb_vpn;             // 数据TLB虚拟页号
  logic [19:0] dtlb_ppn;             // 数据TLB物理页号
  logic        dtlb_allow_user;      // 数据TLB允许用户模式访问
  logic        dtlb_allow_read;       // 数据TLB允许读取
  logic        dtlb_allow_write;      // 数据TLB允许写入
  
  logic [31:0] walk_va;               // 页表遍历的虚拟地址
  logic        walk_source;           // 页表遍历来源(0:取指, 1:内存访问)
  logic [31:0] walk_pte_l1;           // 一级页表项
  logic        walk_write_access;     // 页表遍历写访问标志
  logic [31:0] walk_instr_pc;         // 页表遍历时的指令PC
  logic        page_fault;             // 页错误标志
  logic [4:0]  page_fault_cause;      // 页错误原因

  // Wishbone总线状态
  bus_state_e  bus_state;             // 总线状态
  logic [31:0] bus_addr_reg;          // 总线地址寄存器
  logic [31:0] bus_wdata_reg;         // 总线写数据寄存器
  logic [3:0]  bus_sel_reg;           // 总线字节选择寄存器
  logic        bus_we_reg;            // 总线写使能寄存器

  // Wishbone接口信号连接
  assign wbm_adr_o = bus_addr_reg;    // 地址输出
  assign wbm_dat_o = bus_wdata_reg;   // 数据输出
  assign wbm_sel_o = bus_sel_reg;     // 字节选择输出
  assign wbm_we_o  = bus_we_reg;      // 写使能输出
  assign wbm_cyc_o = (bus_state != BUS_IDLE); // 周期输出
  assign wbm_stb_o = (bus_state != BUS_IDLE); // 选通输出

  // 寄存器文件连接
  logic [4:0]  rs1_addr_id;           // ID阶段rs1地址
  logic [4:0]  rs2_addr_id;           // ID阶段rs2地址
  logic [31:0] rs1_data_id;           // ID阶段rs1数据
  logic [31:0] rs2_data_id;           // ID阶段rs2数据
  logic [31:0] regfile_rdata1;        // 寄存器文件读数据1
  logic [31:0] regfile_rdata2;        // 寄存器文件读数据2
  logic        regfile_we;            // 寄存器文件写使能
  logic [4:0]  regfile_waddr;         // 寄存器文件写地址
  logic [31:0] regfile_wdata;         // 寄存器文件写数据

  // 从指令中提取寄存器地址
  assign rs1_addr_id = if_id_reg.instr[19:15]; // rs1地址在指令的19:15位
  assign rs2_addr_id = if_id_reg.instr[24:20]; // rs2地址在指令的24:20位

  // 寄存器文件实例
  rv32i_regfile u_regfile (
      .clk   (clk),              // 时钟
      .rst   (rst),              // 复位
      .we    (regfile_we),       // 写使能
      .waddr (regfile_waddr),    // 写地址
      .wdata (regfile_wdata),    // 写数据
      .raddr1(rs1_addr_id),      // 读地址1
      .raddr2(rs2_addr_id),      // 读地址2
      .rdata1(regfile_rdata1),   // 读数据1
      .rdata2(regfile_rdata2)    // 读数据2
  );

  // 寄存器文件写控制
  assign regfile_we    = mem_wb_reg.valid && mem_wb_reg.reg_write && (mem_wb_reg.rd != 5'd0); // 有效且需要写回且目标寄存器非x0
  assign regfile_waddr = mem_wb_reg.rd; // 写地址为MEM/WB阶段的目标寄存器

  // WB -> ID前推
  always_comb begin
    // 默认从寄存器文件读取
    rs1_data_id = regfile_rdata1;
    // 如果正在写入且地址匹配且不是x0寄存器，则前推写入数据
    if (regfile_we && (regfile_waddr == rs1_addr_id) && (rs1_addr_id != 5'd0)) begin
      rs1_data_id = regfile_wdata;
    end

    // 默认从寄存器文件读取
    rs2_data_id = regfile_rdata2;
    // 如果正在写入且地址匹配且不是x0寄存器，则前推写入数据
    if (regfile_we && (regfile_waddr == rs2_addr_id) && (rs2_addr_id != 5'd0)) begin
      rs2_data_id = regfile_wdata;
    end
  end

  // 根据写回数据源选择寄存器写入数据
  always_comb begin
    case (mem_wb_reg.wb_src)
      WB_SRC_MEM: regfile_wdata = mem_wb_reg.mem_data;   // 来自内存
      WB_SRC_PC4: regfile_wdata = mem_wb_reg.pc_plus4;   // 来自PC+4
      WB_SRC_CSR: regfile_wdata = mem_wb_reg.csr_data;   // 来自CSR
      default:    regfile_wdata = mem_wb_reg.alu_result;  // 来自ALU(默认)
    endcase
  end

  // ------------------------------------------------------------
  // CSR(控制和状态寄存器)
  // ------------------------------------------------------------

  // CSR寄存器定义
  logic [31:0] csr_mstatus;    // 机器状态寄存器
  logic [31:0] csr_mtvec;      // 机器陷阱处理基地址寄存器
  logic [31:0] csr_mepc;       // 机器异常程序计数器
  logic [31:0] csr_mcause;     // 机器异常原因寄存器
  logic [31:0] csr_mscratch;    // 机器临时寄存器
  logic [31:0] csr_mie;        // 机器中断使能寄存器
  logic [31:0] csr_mip;        // 机器中断挂起寄存器
  logic [31:0] csr_mtval;      // 机器陷阱值寄存器
  logic [31:0] csr_mcounteren;  // 机器计数器使能寄存器
  logic [31:0] csr_mcycle;     // 机器周期计数器
  logic [31:0] csr_minstret;   // 机器指令退休计数器
  logic [31:0] csr_satp;       // 地址转换和保护寄存器

  // 当前特权模式
  logic [1:0] priv_mode;
  // 特权级别常量定义
  localparam PRIV_U = 2'b00;  // 用户模式
  localparam PRIV_S = 2'b01;  // 监管者模式
  localparam PRIV_M = 2'b11;  // 机器模式

  // ------------------------------------------------------------
  // 前推和控制信号
  // ------------------------------------------------------------

  // 寄存器值信号
  logic [31:0] id_rs1_value;      // ID阶段rs1的值
  logic [31:0] id_rs2_value;      // ID阶段rs2的值
  logic [31:0] ex_rs1_value;      // EX阶段rs1的值(可能经过前推)
  logic [31:0] ex_rs2_value;      // EX阶段rs2的值(可能经过前推)
  logic [31:0] alu_operand_a;     // ALU第一个操作数
  logic [31:0] alu_operand_b;     // ALU第二个操作数
  logic [31:0] alu_result;        // ALU结果

  // 流水线冒险和控制信号
  logic        load_use_hazard;    // 加载使用冒险
  logic        mem_busy;           // 内存忙标志
  logic        branch_flush;       // 分支刷新标志
  logic        trap_flush;         // 陷阱刷新标志
  logic        stall_if;           // IF阶段暂停标志
  logic        stall_id;           // ID阶段暂停标志
  logic        stall_ex;           // EX阶段暂停标志

  // 分支相关信号
  logic        branch_taken_ex;    // EX阶段分支实际跳转标志
  logic [31:0] branch_target_ex;  // EX阶段分支目标地址

  // 内存访问相关信号
  logic        mem_request_active; // 内存请求活跃标志
  logic        ex_mem_request_done; // EX/MEM阶段内存请求完成标志
  logic        mem_response_valid; // 内存响应有效标志
  logic [31:0] mem_response_data; // 内存响应数据
  logic        mem_response_error; // 内存响应错误标志
  logic        mem_transaction_done; // 内存事务完成标志
  logic        mem_transaction_error; // 内存事务错误标志
  logic [31:0] mem_transaction_data; // 内存事务数据
  logic        mem_active_is_load; // 当前活跃内存操作是否为加载
  logic [31:0] mem_active_pc;     // 当前活跃内存操作的PC
  logic [31:0] mem_active_addr;   // 当前活跃内存操作的地址
  logic        fetch_data_take;    // 取回数据被接受标志
  logic        mem_commit;         // 内存操作提交标志
  logic [31:0] mem_load_data;     // 内存加载数据
  logic [3:0]  store_sel_comb;    // 存储字节选择组合逻辑
  logic [31:0] store_data_comb;    // 存储数据组合逻辑

  // CSR相关信号
  logic [31:0] csr_write_value;    // CSR写入值
  logic        csr_write_enable_ex; // EX阶段CSR写使能
  logic [31:0] csr_operand_value;  // CSR操作数值
  logic [31:0] csr_operand_ex;    // EX阶段CSR操作数
  logic [31:0] pc_plus4_ex;       // EX阶段PC+4值

  // 指令总线错误信号
  logic        instr_bus_error;    // 指令总线错误标志
  logic [31:0] instr_error_pc;     // 指令错误PC

  // 陷阱相关信号
  logic        trap_request;       // 陷阱请求标志
  logic [31:0] trap_cause_value;   // 陷阱原因值
  logic [31:0] trap_tval_value;   // 陷阱值
  logic [31:0] trap_target_pc;     // 陷阱目标PC
  logic [31:0] trap_mepc_value;    // 陷阱MEPC值
  logic [31:0] mtvec_base;         // MTVEC基地址
  logic [31:0] csr_mstatus_trap_value; // 陷阱时的MSTATUS值
  logic [31:0] csr_mstatus_mret_value; // MRET时的MSTATUS值

  // 陷阱状态机
  typedef enum logic [1:0] {TRAP_IDLE, TRAP_WAIT, TRAP_FLUSH} trap_state_e;
  trap_state_e trap_state;         // 陷阱状态
  logic        trap_wait;          // 陷阱等待标志
  logic [31:0] trap_cause_reg;     // 陷阱原因寄存器
  logic [31:0] trap_tval_reg;      // 陷阱值寄存器
  logic [31:0] trap_target_pc_reg; // 陷阱目标PC寄存器
  logic [31:0] trap_mepc_reg;      // 陷阱MEPC寄存器
  logic        trap_active;         // 陷阱活跃标志

  // MTVEC基地址计算(对齐到4字节)
  assign mtvec_base = {csr_mtvec[31:2], 2'b00};

  always_comb begin
    csr_mstatus_trap_value = csr_mstatus;
    csr_mstatus_trap_value[7]  = csr_mstatus[3];  // MPIE <= MIE
    csr_mstatus_trap_value[3]  = 1'b0;            // MIE <= 0
    csr_mstatus_trap_value[12:11] = priv_mode;    // MPP <= priv_mode

    csr_mstatus_mret_value = csr_mstatus;
    csr_mstatus_mret_value[3]  = csr_mstatus[7];  // MIE <= MPIE
    csr_mstatus_mret_value[7]  = 1'b1;            // MPIE <= 1
    // 关于RISC-V中ret指令和MPP字段的说明：如果只支持M+U模式，MPP应设置为U-mode (0)
    // 如果MPP在mret时为U-mode (0)，则应返回U-mode，但MPP字段本身应保持不变
    // 在我们的实现中，mret时MPP应设置为U-mode
    // 根据规范："MPP is set to the least-privileged supported mode (U if U-mode is supported, else M)".
    // 因此我们设置为PRIV_U       csr_mstatus_mret_value[12:11] = PRIV_U;       // MPP <= U-mode
  end

  // 译码控制信号
  logic        id_reg_write;      // ID阶段寄存器写使能
  logic        id_mem_read;       // ID阶段内存读使能
  logic        id_mem_write;      // ID阶段内存写使能
  mem_size_e   id_mem_size;       // ID阶段内存访问大小
  logic        id_mem_unsigned;   // ID阶段无符号内存访问标志
  op_a_sel_e   id_op_a_sel;       // ID阶段ALU第一个操作数选择
  op_b_sel_e   id_op_b_sel;       // ID阶段ALU第二个操作数选择
  alu_op_e     id_alu_op;         // ID阶段ALU操作类型
  branch_e     id_branch;         // ID阶段分支类型
  logic        id_jump;           // ID阶段跳转标志
  logic        id_jalr;           // ID阶段JALR标志
  wb_src_e     id_wb_src;         // ID阶段写回数据源
  logic        id_csr_en;         // ID阶段CSR操作使能
  csr_op_e     id_csr_op;         // ID阶段CSR操作类型
  logic        id_csr_imm;        // ID阶段CSR立即数操作标志
  logic [11:0] id_csr_addr;      // ID阶段CSR地址
  logic [31:0] id_csr_rdata;     // ID阶段CSR读取数据
  logic        id_is_ecall;       // ID阶段ECALL指令标志
  logic        id_is_ebreak;      // ID阶段EBREAK指令标志
  logic        id_is_mret;        // ID阶段MRET指令标志
  logic        id_is_fencei;      // ID阶段FENCE.I指令标志
  logic        id_is_sfence_vma;  // ID阶段SFENCE.VMA指令标志
  logic        id_illegal;        // ID阶段非法指令标志
  logic        id_use_rs1;        // ID阶段使用rs1标志
  logic        id_use_rs2;        // ID阶段使用rs2标志
  logic [31:0] id_imm_value;     // ID阶段立即数值

  // ID阶段指令字段提取
  wire        id_valid  = if_id_reg.valid;        // 指令有效标志
  wire [31:0] id_instr  = if_id_reg.instr;       // 指令码
  wire [31:0] id_pc     = if_id_reg.pc;          // 程序计数器
  wire [6:0]  id_opcode = id_instr[6:0];         // 操作码
  wire [2:0]  id_funct3 = id_instr[14:12];       // 功能码3
  wire [6:0]  id_funct7 = id_instr[31:25];       // 功能码7
  wire [4:0]  id_shamt  = id_instr[24:20];       // 移位量
  wire [11:0] id_csr_field = id_instr[31:20];    // CSR字段

  // 寄存器值和CSR操作数赋值
  assign id_rs1_value = rs1_data_id;  // rs1值
  assign id_rs2_value = rs2_data_id;  // rs2值
  assign csr_operand_value = id_csr_imm ? {27'h0, id_instr[19:15]} : id_rs1_value; // CSR操作数(立即数或rs1)

  // 指令译码组合逻辑
  always_comb begin
    // 默认值初始化
    id_reg_write   = 1'b0;
    id_mem_read    = 1'b0;
    id_mem_write   = 1'b0;
    id_mem_size    = MEM_SIZE_WORD;
    id_mem_unsigned= 1'b0;
    id_op_a_sel    = OP_A_RS1;
    id_op_b_sel    = OP_B_RS2;
    id_alu_op      = ALU_ADD;
    id_branch      = BR_NONE;
    id_jump        = 1'b0;
    id_jalr        = 1'b0;
    id_wb_src      = WB_SRC_ALU;
    id_csr_en      = 1'b0;
    id_csr_op      = CSR_OP_WRITE;
    id_csr_imm     = 1'b0;
    id_csr_addr    = id_csr_field;
    id_csr_rdata   = csr_read(id_csr_addr);
    id_is_ecall    = 1'b0;
    id_is_ebreak   = 1'b0;
    id_is_mret     = 1'b0;
    id_is_fencei   = 1'b0;
    id_is_sfence_vma = 1'b0;
    id_illegal     = id_valid;  // 默认假设为非法，除非被覆盖
    id_use_rs1     = 1'b0;
    id_use_rs2     = 1'b0;
    id_imm_value   = 32'h0;

    if (!id_valid) begin
      id_illegal = 1'b0;
    end else begin
      // 根据操作码进行译码
      unique case (id_opcode)
        OPCODE_LUI: begin
          // LUI指令：加载立即数到高位
          id_reg_write = 1'b1;
          id_op_a_sel  = OP_A_ZERO;  // 第一个操作数为0
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数
          id_alu_op    = ALU_PASS_B; // ALU直接传递第二个操作数
          id_imm_value = imm_u(id_instr); // 提取U型立即数
          id_illegal   = 1'b0;
        end
        OPCODE_AUIPC: begin
          // AUIPC指令：PC相对加载立即数到高位
          id_reg_write = 1'b1;
          id_op_a_sel  = OP_A_PC;    // 第一个操作数为PC
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数
          id_alu_op    = ALU_ADD;    // ALU执行加法
          id_imm_value = imm_u(id_instr); // 提取U型立即数
          id_illegal   = 1'b0;
        end
        OPCODE_JAL: begin
          // JAL指令：跳转并链接
          id_reg_write = 1'b1;
          id_jump      = 1'b1;
          id_wb_src    = WB_SRC_PC4;  // 写回PC+4
          id_op_a_sel  = OP_A_PC;    // 第一个操作数为PC
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数
          id_alu_op    = ALU_ADD;    // ALU执行加法
          id_imm_value = imm_j(id_instr); // 提取J型立即数
          id_illegal   = 1'b0;
        end
        OPCODE_JALR: begin
          // JALR指令：寄存器跳转并链接
          id_reg_write = 1'b1;
          id_jump      = 1'b1;
          id_jalr      = 1'b1;
          id_wb_src    = WB_SRC_PC4;  // 写回PC+4
          id_op_a_sel  = OP_A_RS1;   // 第一个操作数为rs1
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数
          id_alu_op    = ALU_ADD;    // ALU执行加法
          id_imm_value = imm_i(id_instr); // 提取I型立即数
          id_use_rs1   = 1'b1;      // 使用rs1
          id_illegal   = 1'b0;
        end
        OPCODE_BRANCH: begin
          // 分支指令译码
          id_use_rs1   = 1'b1;      // 使用rs1
          id_use_rs2   = 1'b1;      // 使用rs2
          id_imm_value = imm_b(id_instr); // 提取B型立即数
          id_illegal   = 1'b0;
          // 根据funct3字段确定分支类型
          unique case (id_funct3)
            3'b000: id_branch = BR_EQ;  // BEQ: 相等分支
            3'b001: id_branch = BR_NE;  // BNE: 不等分支
            3'b100: id_branch = BR_LT;  // BLT: 有符号小于分支
            3'b101: id_branch = BR_GE;  // BGE: 有符号大于等于分支
            3'b110: id_branch = BR_LTU; // BLTU: 无符号小于分支
            3'b111: id_branch = BR_GEU; // BGEU: 无符号大于等于分支
            default: id_illegal = 1'b1;
          endcase
        end
        OPCODE_LOAD: begin
          // 加载指令译码
          id_reg_write = 1'b1;
          id_mem_read  = 1'b1;
          id_wb_src    = WB_SRC_MEM;  // 写回数据来自内存
          id_op_a_sel  = OP_A_RS1;   // 第一个操作数为rs1(基地址)
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数(偏移)
          id_alu_op    = ALU_ADD;    // ALU执行加法计算地址
          id_imm_value = imm_i(id_instr); // 提取I型立即数
          id_use_rs1   = 1'b1;      // 使用rs1
          id_illegal   = 1'b0;
          // 根据funct3字段确定加载大小和符号
          unique case (id_funct3)
            3'b000: begin id_mem_size = MEM_SIZE_BYTE; id_mem_unsigned = 1'b0; end  // LB: 有符号字节
            3'b001: begin id_mem_size = MEM_SIZE_HALF; id_mem_unsigned = 1'b0; end  // LH: 有符号半字
            3'b010: begin id_mem_size = MEM_SIZE_WORD; id_mem_unsigned = 1'b0; end  // LW: 有符号字
            3'b100: begin id_mem_size = MEM_SIZE_BYTE; id_mem_unsigned = 1'b1; end  // LBU: 无符号字节
            3'b101: begin id_mem_size = MEM_SIZE_HALF; id_mem_unsigned = 1'b1; end  // LHU: 无符号半字
            default: id_illegal = 1'b1;
          endcase
        end
        OPCODE_STORE: begin
          // 存储指令译码
          id_mem_write = 1'b1;
          id_op_a_sel  = OP_A_RS1;   // 第一个操作数为rs1(基地址)
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数(偏移)
          id_alu_op    = ALU_ADD;    // ALU执行加法计算地址
          id_imm_value = imm_s(id_instr); // 提取S型立即数
          id_use_rs1   = 1'b1;      // 使用rs1
          id_use_rs2   = 1'b1;      // 使用rs2(要存储的数据)
          id_illegal   = 1'b0;
          // 根据funct3字段确定存储大小
          unique case (id_funct3)
            3'b000: id_mem_size = MEM_SIZE_BYTE; // SB: 字节存储
            3'b001: id_mem_size = MEM_SIZE_HALF; // SH: 半字存储
            3'b010: id_mem_size = MEM_SIZE_WORD; // SW: 字存储
            default: id_illegal = 1'b1;
          endcase
        end
        OPCODE_OP_IMM: begin
          // 立即数算术逻辑指令译码
          id_reg_write = 1'b1;
          id_op_a_sel  = OP_A_RS1;   // 第一个操作数为rs1
          id_op_b_sel  = OP_B_IMM;   // 第二个操作数为立即数
          id_use_rs1   = 1'b1;      // 使用rs1
          id_imm_value = imm_i(id_instr); // 提取I型立即数
          id_illegal   = 1'b0;
          // 根据funct3字段确定ALU操作
          unique case (id_funct3)
            3'b000: id_alu_op = ALU_ADD;  // ADDI: 立即数加法
            3'b010: id_alu_op = ALU_SLT;  // SLTI: 立即数有符号小于比较
            3'b011: id_alu_op = ALU_SLTU; // SLTIU: 立即数无符号小于比较
            3'b100: id_alu_op = ALU_XOR;  // XORI: 立即数异或
            3'b110: id_alu_op = ALU_OR;   // ORI: 立即数或
            3'b111: id_alu_op = ALU_AND;  // ANDI: 立即数与
            3'b001: begin
              // 移位指令或CTZ(计算尾随零)
              if (id_funct7 == 7'b0000000) begin
                id_alu_op = ALU_SLL;  // SLLI: 立即数逻辑左移
              end else if ((id_funct7 == 7'b0110000) && (id_instr[24:20] == 5'b00001)) begin
                id_alu_op = ALU_CTZ;  // CTZ: 计算尾随零的数量(位操作扩展)
                id_illegal = 1'b0;
              end else begin
                id_illegal = 1'b1;
              end
            end
            3'b101: begin
              // 右移指令
              if (id_funct7 == 7'b0000000) id_alu_op = ALU_SRL; // SRLI: 立即数逻辑右移
              else if (id_funct7 == 7'b0100000) id_alu_op = ALU_SRA; // SRAI: 立即数算术右移
              else id_illegal = 1'b1;
            end
            default: id_illegal = 1'b1;
          endcase
        end
        OPCODE_OP: begin
          // 寄存器算术逻辑指令译码
          id_reg_write = 1'b1;
          id_op_a_sel  = OP_A_RS1;   // 第一个操作数为rs1
          id_op_b_sel  = OP_B_RS2;   // 第二个操作数为rs2
          id_use_rs1   = 1'b1;      // 使用rs1
          id_use_rs2   = 1'b1;      // 使用rs2
          id_illegal   = 1'b0;
          // 根据funct7和funct3字段确定ALU操作
          unique case ({id_funct7, id_funct3})
            10'b0000000_000: id_alu_op = ALU_ADD;  // ADD: 加法
            10'b0100000_000: id_alu_op = ALU_SUB;  // SUB: 减法
            10'b0000000_001: id_alu_op = ALU_SLL;  // SLL: 逻辑左移
            10'b0000000_010: id_alu_op = ALU_SLT;  // SLT: 有符号小于比较
            10'b0000000_011: id_alu_op = ALU_SLTU; // SLTU: 无符号小于比较
            10'b0000000_100: id_alu_op = ALU_XOR;  // XOR: 异或
            10'b0100000_100: id_alu_op = ALU_XNOR; // XNOR: 同或(位操作扩展)
            10'b0000000_101: id_alu_op = ALU_SRL; // SRL: 逻辑右移
            10'b0100000_101: id_alu_op = ALU_SRA; // SRA: 算术右移
            10'b0000000_110: id_alu_op = ALU_OR;   // OR: 或
            10'b0000000_111: id_alu_op = ALU_AND;  // AND: 与
            default: id_illegal = 1'b1;
          endcase

          // 位操作扩展覆盖
          if ((id_funct7 == 7'b0000101) && (id_funct3 == 3'b110)) begin
            id_alu_op  = ALU_MINU; // MINU: 无符号最小值(位操作扩展)
            id_illegal = 1'b0;
          end
        end
        OPCODE_MISC_MEM: begin
          // 内存相关指令译码
          if (id_funct3 == 3'b001) begin
             id_is_fencei = 1'b1; // FENCE.I: 指令同步屏障
             id_illegal = 1'b0;
          end else begin
             // fence treated as NOP // FENCE指令作为空操作处理
             id_illegal = 1'b0;
          end
        end
        OPCODE_SYSTEM: begin
          // 系统指令和CSR指令译码
          id_illegal = 1'b0;
          if (id_funct3 == 3'b000) begin
            // 系统调用和陷阱指令
            case (id_instr[31:20])
              12'h000: id_is_ecall  = 1'b1;  // ECALL: 环境调用
              12'h001: id_is_ebreak = 1'b1;  // EBREAK: 断点
              12'h302: begin
                 if (priv_mode != PRIV_M) id_illegal = 1'b1; // MRET只能在M模式执行
                 else id_is_mret = 1'b1; // MRET: 从机器异常返回
              end
              default: begin
                // SFENCE.VMA指令
                if (id_instr[31:25] == 7'b0001001) begin
                  if (priv_mode == PRIV_U) begin
                    id_illegal = 1'b1; // SFENCE.VMA不能在U模式执行
                  end else begin
                    id_is_sfence_vma = 1'b1; // SFENCE.VMA: 虚拟内存同步屏障
                    id_illegal = 1'b0;
                  end
                end else begin
                  id_illegal = 1'b1;
                end
              end
            endcase
          end else begin
            // CSR访问指令
            id_csr_en   = 1'b1;
            id_csr_addr = id_csr_field;
            id_csr_rdata= csr_read(id_csr_addr);
            if (!csr_legal(id_csr_addr)) begin
              id_illegal = 1'b1; // CSR地址非法
            end else begin
              logic [1:0] csr_priv = id_csr_addr[9:8];
              if (priv_mode < csr_priv) begin
                 id_illegal = 1'b1; // 特权级别不足
              end else begin
                  logic csr_write_attempt;

                  id_reg_write = 1'b1;
                  id_wb_src    = WB_SRC_CSR;  // 写回数据来自CSR
                  id_use_rs1   = (id_funct3[2] == 1'b0); // 根据指令确定是否使用rs1
                  id_csr_imm   = id_funct3[2]; // 确定是否使用立即数
                  // 根据funct3低2位确定CSR操作类型
                  unique case (id_funct3[1:0])
                    2'b01: id_csr_op = CSR_OP_WRITE; // CSRRW/CSRRWI: CSR读写
                    2'b10: id_csr_op = CSR_OP_SET;   // CSRRS/CSRRSI: CSR读并置位
                    2'b11: id_csr_op = CSR_OP_CLEAR; // CSRRC/CSRRCI: CSR读并清位
                    default: id_illegal = 1'b1;
                  endcase

                  // 检查是否有写操作
                  csr_write_attempt = 1'b0;
                  if (id_illegal == 1'b0) begin
                    if (id_funct3[1:0] == 2'b01) begin
                      csr_write_attempt = 1'b1;  // CSRRW/CSRRWI总是写
                    end else begin
                      if (id_csr_imm)
                        csr_write_attempt = |id_instr[19:15]; // 立即数非零则写
                      else
                        csr_write_attempt = (id_instr[19:15] != 5'd0); // rs1非零则写
                    end

                    // 检查是否尝试写入只读CSR
                    if (csr_is_readonly(id_csr_addr) && csr_write_attempt) begin
                      id_illegal = 1'b1;
                    end
                  end
              end
            end
          end
        end
        default: begin
          id_illegal = 1'b1;
        end
      endcase
    end
  end

  // 加载使用冒险检测：当EX阶段有加载指令且ID阶段需要使用其结果时
  assign load_use_hazard = id_ex_reg.valid && id_ex_reg.mem_read && (id_ex_reg.rd != 5'd0) &&
                           ((id_use_rs1 && (id_ex_reg.rd == if_id_reg.instr[19:15])) ||
                            (id_use_rs2 && (id_ex_reg.rd == if_id_reg.instr[24:20])));

  // 流水线暂停控制
  assign stall_ex = mem_busy || mem_request_active || trap_wait; // EX阶段暂停条件
  assign stall_id = stall_ex || load_use_hazard;              // ID阶段暂停条件
  assign stall_if = stall_id;                                 // IF阶段暂停条件

  // FENCE.I指令刷新控制
  logic fencei_flush;
  assign fencei_flush = ex_mem_reg.valid && ex_mem_reg.is_fencei && !ex_mem_reg.trap_valid;
  assign flush_icache = fencei_flush; // FENCE.I需要刷新指令缓存

  // MRET指令刷新控制
  logic mret_flush;
  assign mret_flush = ex_mem_reg.valid && ex_mem_reg.is_mret && !ex_mem_reg.trap_valid;

  // 分支预测错误检测
  logic branch_mispredicted;
  always_comb begin
    branch_mispredicted = 1'b0;
    if (id_ex_reg.valid) begin
      if (id_ex_reg.branch != BR_NONE || id_ex_reg.jump) begin
        // 对于分支或跳转指令，检查预测是否正确
        if (branch_taken_ex != id_ex_reg.pred_taken)
          branch_mispredicted = 1'b1; // 预测跳转与实际跳转不符
        else if (branch_taken_ex && (branch_target_ex != id_ex_reg.pred_target))
          branch_mispredicted = 1'b1; // 跳转但目标地址不符
      end else begin
        // 对于非分支指令，检查是否错误预测了跳转
        if (id_ex_reg.pred_taken)
          branch_mispredicted = 1'b1;
      end
    end
  end

  // 分支刷新和取指数据接受控制
  assign branch_flush = branch_mispredicted && !trap_flush;
  assign fetch_data_take = fetch_data_valid && !stall_id && !trap_flush && !branch_flush && !fencei_flush;

  // 数据前推
  logic [31:0] ex_mem_forward_value; // EX/MEM阶段前推值
  logic [31:0] mem_wb_forward_value; // MEM/WB阶段前推值

  // 前推值计算
  always_comb begin
    // 根据EX/MEM阶段的写回数据源计算前推值
    unique case (ex_mem_reg.wb_src)
      WB_SRC_MEM: ex_mem_forward_value = mem_load_data;       // 来自内存加载
      WB_SRC_PC4: ex_mem_forward_value = ex_mem_reg.pc_plus4; // 来自PC+4
      WB_SRC_CSR: ex_mem_forward_value = ex_mem_reg.csr_rdata; // 来自CSR
      default:    ex_mem_forward_value = ex_mem_reg.alu_result; // 来自ALU(默认)
    endcase

    // 根据MEM/WB阶段的写回数据源计算前推值
    unique case (mem_wb_reg.wb_src)
      WB_SRC_MEM: mem_wb_forward_value = mem_wb_reg.mem_data;   // 来自内存
      WB_SRC_PC4: mem_wb_forward_value = mem_wb_reg.pc_plus4; // 来自PC+4
      WB_SRC_CSR: mem_wb_forward_value = mem_wb_reg.csr_data; // 来自CSR
      default:    mem_wb_forward_value = mem_wb_reg.alu_result; // 来自ALU(默认)
    endcase
  end

  // 前推逻辑实现
  always_comb begin
    // 默认使用ID/EX阶段寄存器中的值
    ex_rs1_value = id_ex_reg.rs1_value;
    // 优先级：EX/MEM阶段 > MEM/WB阶段 > 寄存器文件
    if (id_ex_reg.use_rs1 && ex_mem_reg.valid && ex_mem_reg.reg_write && (ex_mem_reg.rd != 5'd0) &&
        (ex_mem_reg.rd == id_ex_reg.rs1)) begin
      ex_rs1_value = ex_mem_forward_value; // 从EX/MEM阶段前推
    end else if (id_ex_reg.use_rs1 && mem_wb_reg.valid && mem_wb_reg.reg_write && (mem_wb_reg.rd != 5'd0) &&
                 (mem_wb_reg.rd == id_ex_reg.rs1)) begin
      ex_rs1_value = mem_wb_forward_value; // 从MEM/WB阶段前推
    end

    // 默认使用ID/EX阶段寄存器中的值
    ex_rs2_value = id_ex_reg.rs2_value;
    // 优先级：EX/MEM阶段 > MEM/WB阶段 > 寄存器文件
    if (id_ex_reg.use_rs2 && ex_mem_reg.valid && ex_mem_reg.reg_write && (ex_mem_reg.rd != 5'd0) &&
        (ex_mem_reg.rd == id_ex_reg.rs2)) begin
      ex_rs2_value = ex_mem_forward_value; // 从EX/MEM阶段前推
    end else if (id_ex_reg.use_rs2 && mem_wb_reg.valid && mem_wb_reg.reg_write && (mem_wb_reg.rd != 5'd0) &&
                 (mem_wb_reg.rd == id_ex_reg.rs2)) begin
      ex_rs2_value = mem_wb_forward_value; // 从MEM/WB阶段前推
    end
  end

  // ALU操作数选择和计算
  always_comb begin
    // 根据操作数选择信号选择第一个操作数
    case (id_ex_reg.op_a_sel)
      OP_A_PC:   alu_operand_a = id_ex_reg.pc;    // 选择PC
      OP_A_ZERO: alu_operand_a = 32'h0;          // 选择零
      default:   alu_operand_a = ex_rs1_value;    // 选择rs1(默认)
    endcase

    // 根据操作数选择信号选择第二个操作数
    case (id_ex_reg.op_b_sel)
      OP_B_IMM:  alu_operand_b = id_ex_reg.imm;   // 选择立即数
      OP_B_FOUR: alu_operand_b = 32'd4;          // 选择常数4
      default:   alu_operand_b = ex_rs2_value;    // 选择rs2(默认)
    endcase

    // 根据ALU操作类型执行计算
    case (id_ex_reg.alu_op)
      ALU_ADD:   alu_result = alu_operand_a + alu_operand_b;                    // 加法
      ALU_SUB:   alu_result = alu_operand_a - alu_operand_b;                    // 减法
      ALU_SLL:   alu_result = alu_operand_a << alu_operand_b[4:0];          // 逻辑左移
      ALU_SLT:   alu_result = ($signed(alu_operand_a) < $signed(alu_operand_b)) ? 32'd1 : 32'd0; // 有符号比较小于
      ALU_SLTU:  alu_result = (alu_operand_a < alu_operand_b) ? 32'd1 : 32'd0;       // 无符号比较小于
      ALU_XOR:   alu_result = alu_operand_a ^ alu_operand_b;                    // 异或
      ALU_XNOR:  alu_result = ~(alu_operand_a ^ alu_operand_b);                   // 同或
      ALU_SRL:   alu_result = alu_operand_a >> alu_operand_b[4:0];          // 逻辑右移
      ALU_SRA:   alu_result = $signed(alu_operand_a) >>> alu_operand_b[4:0]; // 算术右移
      ALU_OR:    alu_result = alu_operand_a | alu_operand_b;                    // 或
      ALU_AND:   alu_result = alu_operand_a & alu_operand_b;                    // 与
      ALU_PASS_B:alu_result = alu_operand_b;                                   // 直接传递第二个操作数
      ALU_MINU:  alu_result = (alu_operand_a < alu_operand_b) ? alu_operand_a : alu_operand_b; // 无符号最小值
      ALU_CTZ:   alu_result = ctz32(ex_rs1_value);                             // 计算尾随零的数量
      default:   alu_result = 32'h0;                                           // 默认值
    endcase
  end

  // 存储数据字节选择和数据组合
  always_comb begin
    store_sel_comb  = 4'b0000;
    store_data_comb = 32'h0;
    case (id_ex_reg.mem_size)
      MEM_SIZE_BYTE: begin
        // 字节存储：根据地址低2位选择字节，数据扩展到4字节
        store_sel_comb  = 4'b0001 << alu_result[1:0]; // 字节选择
        store_data_comb = {4{ex_rs2_value[7:0]}} << (8 * alu_result[1:0]); // 数据扩展和对齐
      end
      MEM_SIZE_HALF: begin
        // 半字存储：根据地址第1位选择半字，数据扩展到4字节
        store_sel_comb  = alu_result[1] ? 4'b1100 : 4'b0011; // 半字选择
        store_data_comb = {2{ex_rs2_value[15:0]}} << (16 * alu_result[1]); // 数据扩展和对齐
      end
      MEM_SIZE_WORD: begin
        // 字存储：选择所有字节，数据不变
        store_sel_comb  = 4'b1111; // 选择所有字节
        store_data_comb = ex_rs2_value; // 数据不变
      end
    endcase
  end

  // CSR操作数和写入值计算
  always_comb begin
    // CSR操作数默认值
    csr_operand_ex      = id_ex_reg.csr_operand;
    // 如果不是立即数操作，则使用前推的rs1值
    if (!id_ex_reg.csr_imm)
      csr_operand_ex    = ex_rs1_value;

    // CSR写入值默认为当前CSR值
    csr_write_value     = id_ex_reg.csr_rdata;
    // CSR写使能默认为禁用
    csr_write_enable_ex = 1'b0;

    // 如果CSR操作使能
    if (id_ex_reg.csr_en) begin
      csr_write_enable_ex = 1'b1;
      // 根据CSR操作类型计算写入值
      unique case (id_ex_reg.csr_op)
        CSR_OP_WRITE: csr_write_value = csr_operand_ex;                           // 直接写入
        CSR_OP_SET:   csr_write_value = id_ex_reg.csr_rdata | csr_operand_ex;  // 置位操作
        CSR_OP_CLEAR: csr_write_value = id_ex_reg.csr_rdata & ~csr_operand_ex; // 清位操作
      endcase
      // 对于非写入操作且操作数为零的情况，禁用写使能(纯读取)
      if ((id_ex_reg.csr_op != CSR_OP_WRITE) && (csr_operand_ex == 32'h0)) begin
        csr_write_enable_ex = 1'b0;  // 操作数为零时为纯读取
      end
    end
  end

  // MRET指令目标特权级别
  logic [1:0] mret_target_priv;
  assign mret_target_priv = csr_mstatus[12:11];

  always_comb begin
    branch_taken_ex = 1'b0;
    branch_target_ex = id_ex_reg.pc + id_ex_reg.imm;

    case (id_ex_reg.branch)
      BR_EQ:  branch_taken_ex = (ex_rs1_value == ex_rs2_value);
      BR_NE:  branch_taken_ex = (ex_rs1_value != ex_rs2_value);
      BR_LT:  branch_taken_ex = ($signed(ex_rs1_value) < $signed(ex_rs2_value));
      BR_GE:  branch_taken_ex = ($signed(ex_rs1_value) >= $signed(ex_rs2_value));
      BR_LTU: branch_taken_ex = (ex_rs1_value < ex_rs2_value);
      BR_GEU: branch_taken_ex = (ex_rs1_value >= ex_rs2_value);
      default: ;
    endcase

    if (id_ex_reg.jump) begin
      branch_taken_ex = 1'b1;
      if (id_ex_reg.jalr)
        branch_target_ex = (ex_rs1_value + id_ex_reg.imm) & 32'hFFFF_FFFE;
      else
        branch_target_ex = id_ex_reg.pc + id_ex_reg.imm;
    end
  end

  assign pc_plus4_ex = id_ex_reg.pc + 32'd4;

  logic ex_trap_valid;
  logic [4:0] ex_trap_cause;
  logic [31:0] ex_trap_tval;

  wire instr_bus_trap_valid = instr_bus_error;
  wire mem_bus_trap_valid   = mem_transaction_error;
  wire [4:0] mem_bus_trap_cause = mem_active_is_load ? CAUSE_LOAD_ACCESS_FAULT : CAUSE_STORE_ACCESS_FAULT;
  wire [31:0] mem_bus_trap_tval = mem_active_addr;

  wire global_interrupt_enable = (priv_mode == PRIV_M) ? csr_mstatus[3] : 1'b1;
  // 实验兼容性：如果软件没有设置mie[7]，我们会在复位为0x88时强制设置
  // 但由于我们更改了复位值，csr_mie[7]应该为1
  // 中断挂起位由硬件自动设置，软件只能清除
  wire m_external_interrupt_pending = global_interrupt_enable && csr_mie[11] && csr_mip[11];
  wire m_timer_interrupt_pending    = global_interrupt_enable && csr_mie[7]  && csr_mip[7];

  // 中断时保存到mepc的PC值  // 需要保存触发中断的指令地址，而不是下一条指令的PC
  // 根据RISC-V规范，中断时保存发生中断的指令地址
  logic [31:0] interrupt_mepc;
  always_comb begin
    // 优先级：EX阶段 > ID阶段 > IF阶段 > 当前PC
    if (id_ex_reg.valid)
      interrupt_mepc = id_ex_reg.pc;
    else if (if_id_reg.valid)
      interrupt_mepc = if_id_reg.pc;
    else if (fetch_data_valid)
      interrupt_mepc = fetch_data_pc;
    else
      interrupt_mepc = pc_reg;
  end

  // 陷阱请求和值计算
  always_comb begin
    trap_request     = 1'b0;
    trap_cause_value = 32'h0;
    trap_tval_value  = 32'h0;
    trap_target_pc   = mtvec_base;
    trap_mepc_value  = 32'h0;

    // 根据RISC-V规范，中断优先级高于异常
    if (m_timer_interrupt_pending) begin
      trap_request     = 1'b1;
      trap_cause_value = {1'b1, 26'h0, CAUSE_M_TIMER_INT}; // 中断位为1
      trap_tval_value  = 32'h0;
      // 中断时保存发生中断的指令地址
      trap_mepc_value  = interrupt_mepc;
    end else if (m_external_interrupt_pending) begin
      trap_request     = 1'b1;
      trap_cause_value = {1'b1, 26'h0, CAUSE_M_EXTERNAL_INT}; // 中断位为1
      trap_tval_value  = 32'h0;
      trap_mepc_value  = interrupt_mepc;
    end else if (ex_trap_valid) begin
      trap_request     = 1'b1;
      trap_cause_value = {27'h0, ex_trap_cause}; // 异常位为0
      trap_tval_value  = ex_trap_tval;
      // 对于ECALL，保存触发ECALL的指令地址，而不是下一条指令
      // 对于其他异常，保存导致异常的指令地址
      trap_mepc_value = id_ex_reg.pc;
    end else if (instr_bus_trap_valid) begin
      trap_request     = 1'b1;
      trap_cause_value = {27'h0, CAUSE_INSTR_ACCESS_FAULT};
      trap_tval_value  = instr_error_pc;
      trap_mepc_value  = instr_error_pc;
    end else if (mem_bus_trap_valid) begin
      trap_request     = 1'b1;
      trap_cause_value = {27'h0, mem_bus_trap_cause};
      trap_tval_value  = mem_bus_trap_tval;
      trap_mepc_value  = mem_active_pc;
    end else if (page_fault) begin
      trap_request     = 1'b1;
      trap_cause_value = {27'h0, page_fault_cause};
      trap_tval_value  = walk_va;
      trap_mepc_value  = walk_instr_pc;
    end else begin
      trap_request = 1'b0;
    end
  end

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      trap_state         <= TRAP_IDLE;
      trap_cause_reg     <= 32'h0;
      trap_tval_reg      <= 32'h0;
      trap_target_pc_reg <= 32'h0;
      trap_mepc_reg      <= 32'h0;
    end else begin
      unique case (trap_state)
        TRAP_IDLE: begin
          if (trap_request) begin
            trap_state         <= TRAP_WAIT;
            trap_cause_reg     <= trap_cause_value;
            trap_tval_reg      <= trap_tval_value;
            trap_target_pc_reg <= trap_target_pc;
            trap_mepc_reg      <= trap_mepc_value;
          end
        end
        TRAP_WAIT: begin
          trap_state <= TRAP_FLUSH;
        end
        TRAP_FLUSH: begin
          trap_state <= TRAP_IDLE;
        end
        default: trap_state <= TRAP_IDLE;
      endcase
    end
  end

  assign trap_wait   = (trap_state == TRAP_WAIT);
  assign trap_active = (trap_state == TRAP_FLUSH);
  assign trap_flush  = trap_active;

  always_comb begin
    ex_trap_valid = 1'b0;
    ex_trap_cause = CAUSE_ILLEGAL_INSTRUCTION;
    ex_trap_tval  = 32'h0;

    if (id_ex_reg.valid) begin
      if (id_ex_reg.illegal) begin
        ex_trap_valid = 1'b1;
        ex_trap_cause = CAUSE_ILLEGAL_INSTRUCTION;
        ex_trap_tval  = id_ex_reg.instr;
      end else if (id_ex_reg.is_ecall) begin
        // 根据特权级别确定ECALL的原因码，这是RISC-V规范的要求
        ex_trap_valid = 1'b1;
        // 根据RISC-V规范，ECALL的原因码取决于当前特权级别
        case (priv_mode)
          PRIV_U: ex_trap_cause = CAUSE_ECALL_U;  // U-mode: mcause = 8
          PRIV_S: ex_trap_cause = CAUSE_ECALL_S;  // S-mode: mcause = 9
          default: ex_trap_cause = CAUSE_ECALL_M; // M-mode: mcause = 11
        endcase
        // 对于ECALL，tval未定义，根据RISC-V规范设为0
        ex_trap_tval  = 32'h0;
      end else if (id_ex_reg.is_ebreak) begin
        ex_trap_valid = 1'b1;
        ex_trap_cause = CAUSE_BREAKPOINT;
        ex_trap_tval  = id_ex_reg.pc;
      end else if (id_ex_reg.jump && id_ex_reg.jalr && |branch_target_ex[1:0]) begin
        ex_trap_valid = 1'b1;
        ex_trap_cause = CAUSE_INSTR_ADDR_MISALIGNED;
        ex_trap_tval  = branch_target_ex;
      end else if (id_ex_reg.mem_read || id_ex_reg.mem_write) begin
        logic misaligned;
        misaligned = 1'b0;
        case (id_ex_reg.mem_size)
          MEM_SIZE_HALF: misaligned = alu_result[0];
          MEM_SIZE_WORD: misaligned = |alu_result[1:0];
          default: misaligned = 1'b0;
        endcase
        if (misaligned) begin
          ex_trap_valid = 1'b1;
          ex_trap_cause = id_ex_reg.mem_read ? CAUSE_LOAD_ADDR_MISALIGNED : CAUSE_STORE_ADDR_MISALIGNED;
          ex_trap_tval  = alu_result;
        end
      end
    end
  end

  assign mem_request_active = ex_mem_reg.valid && (ex_mem_reg.mem_read || ex_mem_reg.mem_write) &&
                              !mem_busy && !ex_mem_reg.trap_valid && !ex_mem_request_done &&
                              !mem_transaction_done;

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      bus_state           <= BUS_IDLE;
      bus_addr_reg        <= 32'h0;
      bus_wdata_reg       <= 32'h0;
      bus_sel_reg         <= 4'h0;
      bus_we_reg          <= 1'b0;
      pc_reg              <= RESET_VECTOR;
      fetch_inflight      <= 1'b0;
      fetch_data_valid    <= 1'b0;
      fetch_discard       <= 1'b0;
      fetch_pred_taken    <= 1'b0;
      fetch_pred_target   <= 32'h0;
      fetch_data_pred_taken <= 1'b0;
      fetch_data_pred_target <= 32'h0;
      mem_busy            <= 1'b0;
      mem_transaction_done<= 1'b0;
      mem_transaction_error <= 1'b0;
      mem_response_valid  <= 1'b0;
      mem_response_error  <= 1'b0;
      instr_bus_error     <= 1'b0;
      mem_response_data   <= 32'h0;
      mem_transaction_data<= 32'h0;
      mem_active_is_load  <= 1'b0;
      mem_active_pc       <= 32'h0;
      mem_active_addr     <= 32'h0;

      itlb_valid       <= 1'b0;
      itlb_allow_user  <= 1'b0;
      itlb_allow_exec  <= 1'b0;
      dtlb_valid       <= 1'b0;
      dtlb_allow_user  <= 1'b0;
      dtlb_allow_read  <= 1'b0;
      dtlb_allow_write <= 1'b0;
      walk_va          <= 32'h0;
      walk_source      <= 1'b0;
      walk_pte_l1      <= 32'h0;
      walk_write_access<= 1'b0;
      walk_instr_pc    <= 32'h0;
      page_fault       <= 1'b0;
      page_fault_cause <= 5'h0;
    end else begin
      mem_transaction_done  <= 1'b0;
      mem_transaction_error <= 1'b0;
      mem_response_valid    <= 1'b0;
      mem_response_error    <= 1'b0;
      instr_bus_error       <= 1'b0;
      page_fault            <= 1'b0;
      page_fault_cause      <= 5'h0;

      if (ex_mem_reg.csr_en && ex_mem_reg.csr_write_en && mem_commit && !ex_mem_reg.trap_valid && ex_mem_reg.csr_addr == 12'h180) begin
        itlb_valid       <= 1'b0;
        itlb_allow_user  <= 1'b0;
        itlb_allow_exec  <= 1'b0;
        dtlb_valid       <= 1'b0;
        dtlb_allow_user  <= 1'b0;
        dtlb_allow_read  <= 1'b0;
        dtlb_allow_write <= 1'b0;
      end
      if (ex_mem_reg.valid && ex_mem_reg.is_fencei && !ex_mem_reg.trap_valid) begin
        itlb_valid       <= 1'b0;
        itlb_allow_user  <= 1'b0;
        itlb_allow_exec  <= 1'b0;
        dtlb_valid       <= 1'b0;
        dtlb_allow_user  <= 1'b0;
        dtlb_allow_read  <= 1'b0;
        dtlb_allow_write <= 1'b0;
      end
      if (ex_mem_reg.valid && ex_mem_reg.is_sfence_vma && !ex_mem_reg.trap_valid) begin
        itlb_valid       <= 1'b0;
        itlb_allow_user  <= 1'b0;
        itlb_allow_exec  <= 1'b0;
        dtlb_valid       <= 1'b0;
        dtlb_allow_user  <= 1'b0;
        dtlb_allow_read  <= 1'b0;
        dtlb_allow_write <= 1'b0;
      end

      // 处理陷阱刷新、分支刷新等特殊情况
      if (trap_flush) begin
        pc_reg <= trap_target_pc_reg;

        fetch_data_valid       <= 1'b0;
        fetch_discard          <= 1'b0;
        fetch_inflight         <= 1'b0;
        fetch_pred_taken       <= 1'b0;
        fetch_pred_target      <= 32'h0;
        fetch_data_pred_taken  <= 1'b0;
        fetch_data_pred_target <= 32'h0;
        bus_state <= BUS_IDLE;
        mem_busy  <= 1'b0;
      end else if (trap_wait) begin
        fetch_data_valid <= 1'b0;
        fetch_discard    <= fetch_inflight;
        fetch_inflight   <= 1'b0;
        bus_state        <= BUS_IDLE;
        mem_busy         <= 1'b0;
      end else if (fencei_flush) begin
        pc_reg <= ex_mem_reg.pc_plus4;
        fetch_data_valid <= 1'b0;
        fetch_discard <= fetch_inflight;
      end else if (mret_flush) begin
        pc_reg <= csr_mepc;
        fetch_data_valid <= 1'b0;
        fetch_discard <= fetch_inflight;
      end else if (branch_flush) begin
        if (branch_taken_ex)
          pc_reg <= branch_target_ex;
        else
          pc_reg <= id_ex_reg.pc + 32'd4;
        fetch_data_valid <= 1'b0;
        fetch_discard <= fetch_inflight;
      end else begin
        case (bus_state)
          BUS_IDLE: begin
            if (mem_request_active) begin
              logic [31:0] va;
              logic        translate;
              logic        user_mode;
              logic        violation;
              va        = ex_mem_reg.mem_addr;
              translate = (priv_mode != PRIV_M) && csr_satp[31];
              user_mode = (priv_mode == PRIV_U);
              violation = 1'b0;

              if (translate) begin
                if (dtlb_valid && (dtlb_vpn == va[31:12])) begin
                  if (user_mode && !dtlb_allow_user)
                    violation = 1'b1;
                  if (ex_mem_reg.mem_read && !dtlb_allow_read)
                    violation = 1'b1;
                  if (ex_mem_reg.mem_write && !dtlb_allow_write)
                    violation = 1'b1;

                  if (violation) begin
                    page_fault        <= 1'b1;
                    page_fault_cause  <= ex_mem_reg.mem_write ? CAUSE_STORE_PAGE_FAULT : CAUSE_LOAD_PAGE_FAULT;
                    walk_source       <= 1'b1;
                    walk_write_access <= ex_mem_reg.mem_write;
                    walk_va           <= va;
                    walk_instr_pc     <= ex_mem_reg.pc;
                    mem_busy          <= 1'b0;
                  end else begin
                    bus_state    <= BUS_MEM;
                    bus_addr_reg <= {dtlb_ppn, va[11:0]};
                    bus_wdata_reg<= ex_mem_reg.store_data;
                    bus_sel_reg  <= ex_mem_reg.store_sel;
                    bus_we_reg   <= ex_mem_reg.mem_write;
                    mem_busy     <= 1'b1;
                    mem_active_is_load <= ex_mem_reg.mem_read;
                    mem_active_pc      <= ex_mem_reg.pc;
                    mem_active_addr    <= va;
                  end
                end else begin
                  bus_state        <= BUS_WALK_L1;
                  walk_va          <= va;
                  walk_source      <= 1'b1;
                  walk_write_access<= ex_mem_reg.mem_write;
                  walk_instr_pc    <= ex_mem_reg.pc;
                  bus_addr_reg     <= {csr_satp[21:0], va[31:22], 2'b00};
                  bus_we_reg       <= 1'b0;
                  bus_sel_reg      <= 4'b1111;
                  mem_busy         <= 1'b1;
                end
              end else begin
                bus_state    <= BUS_MEM;
                bus_addr_reg <= va;
                bus_wdata_reg<= ex_mem_reg.store_data;
                bus_sel_reg  <= ex_mem_reg.store_sel;
                bus_we_reg   <= ex_mem_reg.mem_write;
                mem_busy     <= 1'b1;
                mem_active_is_load <= ex_mem_reg.mem_read;
                mem_active_pc      <= ex_mem_reg.pc;
                mem_active_addr    <= va;
              end
            end else if (!fetch_inflight && !fetch_data_valid && !trap_flush && !branch_flush && !fencei_flush) begin
              logic [31:0] va;
              logic        translate;
              logic        user_mode;
              logic        violation;
              va        = pc_reg;
              translate = (priv_mode != PRIV_M) && csr_satp[31];
              user_mode = (priv_mode == PRIV_U);
              violation = 1'b0;

              if (translate) begin
                if (itlb_valid && (itlb_vpn == va[31:12])) begin
                  if (!itlb_allow_exec)
                    violation = 1'b1;
                  if (user_mode && !itlb_allow_user)
                    violation = 1'b1;

                  if (violation) begin
                    page_fault        <= 1'b1;
                    page_fault_cause  <= CAUSE_INSTR_PAGE_FAULT;
                    walk_source       <= 1'b0;
                    walk_write_access <= 1'b0;
                    walk_va           <= va;
                    walk_instr_pc     <= va;
                  end else if (icache_hit) begin
                    fetch_data_instr <= icache_data[icache_index];
                    fetch_data_pc    <= pc_reg;
                    fetch_data_valid <= 1'b1;
                    if (btb_pred_taken) begin
                      pc_reg <= btb_pred_target;
                      fetch_pred_taken <= 1'b1;
                      fetch_pred_target <= btb_pred_target;
                    end else begin
                      pc_reg <= pc_reg + 32'd4;
                      fetch_pred_taken <= 1'b0;
                      fetch_pred_target <= 32'h0;
                    end
                    fetch_data_pred_taken   <= btb_pred_taken;
                    fetch_data_pred_target  <= btb_pred_target;
                  end else begin
                    bus_state        <= BUS_IFETCH;
                    bus_addr_reg     <= {itlb_ppn, va[11:0]};
                    bus_wdata_reg    <= 32'h0;
                    bus_sel_reg      <= 4'b1111;
                    bus_we_reg       <= 1'b0;
                    fetch_inflight   <= 1'b1;
                    fetch_pc_inflight<= pc_reg;
                    if (btb_pred_taken) begin
                      pc_reg <= btb_pred_target;
                      fetch_pred_taken <= 1'b1;
                      fetch_pred_target <= btb_pred_target;
                    end else begin
                      pc_reg <= pc_reg + 32'd4;
                      fetch_pred_taken <= 1'b0;
                      fetch_pred_target <= 32'h0;
                    end
                  end
                end else begin
                  bus_state        <= BUS_WALK_L1;
                  walk_va          <= va;
                  walk_source      <= 1'b0;
                  walk_write_access<= 1'b0;
                  walk_instr_pc    <= va;
                  bus_addr_reg     <= {csr_satp[21:0], va[31:22], 2'b00};
                  bus_we_reg       <= 1'b0;
                  bus_sel_reg      <= 4'b1111;
                end
              end else begin
                if (icache_hit) begin
                  fetch_data_instr <= icache_data[icache_index];
                  fetch_data_pc    <= pc_reg;
                  fetch_data_valid <= 1'b1;
                  if (btb_pred_taken) begin
                    pc_reg <= btb_pred_target;
                    fetch_pred_taken <= 1'b1;
                    fetch_pred_target <= btb_pred_target;
                  end else begin
                    pc_reg <= pc_reg + 32'd4;
                    fetch_pred_taken <= 1'b0;
                    fetch_pred_target <= 32'h0;
                  end
                  fetch_data_pred_taken  <= btb_pred_taken;
                  fetch_data_pred_target <= btb_pred_target;
                end else begin
                  bus_state        <= BUS_IFETCH;
                  bus_addr_reg     <= pc_reg;
                  bus_wdata_reg    <= 32'h0;
                  bus_sel_reg      <= 4'b1111;
                  bus_we_reg       <= 1'b0;
                  fetch_inflight   <= 1'b1;
                  fetch_pc_inflight<= pc_reg;
                  if (btb_pred_taken) begin
                    pc_reg <= btb_pred_target;
                    fetch_pred_taken <= 1'b1;
                    fetch_pred_target <= btb_pred_target;
                  end else begin
                    pc_reg <= pc_reg + 32'd4;
                    fetch_pred_taken <= 1'b0;
                    fetch_pred_target <= 32'h0;
                  end
                end
              end
            end
          end
          BUS_WALK_L1: begin
            logic [4:0] pf_cause;
            // 确定页错误原因：取指页错误、存储页错误或加载页错误
            pf_cause = (walk_source == 1'b0) ? CAUSE_INSTR_PAGE_FAULT :
                       (walk_write_access ? CAUSE_STORE_PAGE_FAULT : CAUSE_LOAD_PAGE_FAULT);
            if (wbm_ack_i) begin
              logic [31:0] pte;
              logic        leaf;
              logic        user_mode;
              logic        allow_user;
              logic        allow_read;
              logic        allow_write;
              logic        allow_exec;
              logic        accessed;
              logic        dirty;
              logic        violation;
              pte       = wbm_dat_i;
              leaf      = pte[1] || pte[3]; // R=1或X=1表示叶子页表项
              user_mode = (priv_mode == PRIV_U);

              // 检查PTE有效性：V=0或(R=0且W=1)为无效
              if (!pte[0] || (!pte[1] && pte[2])) begin
                bus_state       <= BUS_IDLE;
                page_fault      <= 1'b1;
                page_fault_cause<= pf_cause;
                if (walk_source) mem_busy <= 1'b0;
              end else if (leaf) begin
                // 处理叶子页表项
                allow_user  = pte[4]; // U bit
                allow_read  = pte[1]; // R bit
                allow_write = pte[2]; // W bit
                allow_exec  = pte[3]; // X bit
                accessed    = pte[6]; // A bit
                dirty       = pte[7]; // D bit
                violation   = 1'b0;

                // 检查超级页对齐(Sv32规范要求)：对于一级页表的叶子项(4MB大页)，PPN[0]必须为0
                if (|pte[19:10])
                  violation = 1'b1;

                // 权限检查
                if (user_mode && !allow_user)
                  violation = 1'b1; // 用户模式访问非用户页
                if (!accessed)
                  violation = 1'b1; // 访问位未置位(硬件不自动置位，需软件处理)

                if (walk_source == 1'b0) begin
                  // 取指检查
                  if (!allow_exec)
                    violation = 1'b1; // 不可执行
                end else begin
                  // 数据访问检查
                  if (!walk_write_access && !allow_read)
                    violation = 1'b1; // 读操作但不可读
                  if (walk_write_access && (!allow_write || !dirty))
                    violation = 1'b1; // 写操作但不可写或脏位未置位
                end

                if (violation) begin
                  // 权限检查失败，触发页错误
                  bus_state       <= BUS_IDLE;
                  page_fault      <= 1'b1;
                  page_fault_cause<= pf_cause;
                  if (walk_source) mem_busy <= 1'b0;
                end else begin
                  // 权限检查通过，更新TLB
                  // L1超级页(4MB): PPN = {pte[29:20], va[21:12]}
                  if (walk_source == 1'b0) begin
                    itlb_valid      <= 1'b1;
                    itlb_vpn        <= walk_va[31:12];
                    itlb_ppn        <= {pte[29:20], walk_va[21:12]};
                    itlb_allow_user <= allow_user;
                    itlb_allow_exec <= allow_exec;
                  end else begin
                    dtlb_valid       <= 1'b1;
                    dtlb_vpn         <= walk_va[31:12];
                    dtlb_ppn         <= {pte[29:20], walk_va[21:12]};
                    dtlb_allow_user  <= allow_user;
                    dtlb_allow_read  <= allow_read;
                    dtlb_allow_write <= allow_write && dirty;
                  end
                  bus_state <= BUS_IDLE;
                  if (walk_source) mem_busy <= 1'b0;
                end
              end else begin
                // 非叶子页表项，继续遍历下一级页表
                walk_pte_l1   <= pte;
                bus_state     <= BUS_WALK_L2;
                // 计算二级页表物理地址：Base(pte.ppn) + VPN[0] * 4
                bus_addr_reg  <= {pte[29:10], walk_va[21:12], 2'b00};
              end
            end
            if (wbm_err_i) begin
              bus_state       <= BUS_IDLE;
              page_fault      <= 1'b1;
              page_fault_cause<= pf_cause;
              if (walk_source) mem_busy <= 1'b0;
            end
          end
          BUS_WALK_L2: begin
            logic [4:0] pf_cause;
            // 确定页错误原因
            pf_cause = (walk_source == 1'b0) ? CAUSE_INSTR_PAGE_FAULT :
                       (walk_write_access ? CAUSE_STORE_PAGE_FAULT : CAUSE_LOAD_PAGE_FAULT);
            if (wbm_ack_i) begin
              logic [31:0] pte;
              logic        user_mode;
              logic        allow_user;
              logic        allow_read;
              logic        allow_write;
              logic        allow_exec;
              logic        accessed;
              logic        dirty;
              logic        violation;
              pte       = wbm_dat_i;
              user_mode = (priv_mode == PRIV_U);
              allow_user  = pte[4];
              allow_read  = pte[1];
              allow_write = pte[2];
              allow_exec  = pte[3];
              accessed    = pte[6];
              dirty       = pte[7];
              violation   = 1'b0;

              // 检查PTE有效性：V=0或(R=0且W=1)为无效
              if (!pte[0] || (!pte[1] && pte[2]))
                violation = 1'b1;
              // 二级页表项必须是叶子节点(R=1或X=1)
              if (!(pte[1] || pte[3]))
                violation = 1'b1;
              // 权限检查
              if (user_mode && !allow_user)
                violation = 1'b1; // 用户模式访问非用户页
              if (!accessed)
                violation = 1'b1; // 访问位未置位

              if (walk_source == 1'b0) begin
                // 取指检查
                if (!allow_exec)
                  violation = 1'b1;
              end else begin
                // 数据访问检查
                if (!walk_write_access && !allow_read)
                  violation = 1'b1;
                if (walk_write_access && (!allow_write || !dirty))
                  violation = 1'b1;
              end

              if (violation) begin
                // 权限检查失败，触发页错误
                bus_state       <= BUS_IDLE;
                page_fault      <= 1'b1;
                page_fault_cause<= pf_cause;
                if (walk_source) mem_busy <= 1'b0;
              end else begin
                // 权限检查通过，更新TLB
                if (walk_source == 1'b0) begin
                  itlb_valid      <= 1'b1;
                  itlb_vpn        <= walk_va[31:12];
                  itlb_ppn        <= pte[29:10];
                  itlb_allow_user <= allow_user;
                  itlb_allow_exec <= allow_exec;
                end else begin
                  dtlb_valid       <= 1'b1;
                  dtlb_vpn         <= walk_va[31:12];
                  dtlb_ppn         <= pte[29:10];
                  dtlb_allow_user  <= allow_user;
                  dtlb_allow_read  <= allow_read;
                  dtlb_allow_write <= allow_write && dirty;
                end
                bus_state <= BUS_IDLE;
                if (walk_source) mem_busy <= 1'b0;
              end
            end
            if (wbm_err_i) begin
              bus_state       <= BUS_IDLE;
              page_fault      <= 1'b1;
              page_fault_cause<= pf_cause;
              if (walk_source) mem_busy <= 1'b0;
            end
          end
          BUS_IFETCH: begin
            if (wbm_ack_i) begin
              bus_state    <= BUS_IDLE;
              fetch_inflight <= 1'b0;
              if (!fetch_discard) begin
                // 如果未被丢弃(例如因跳转而取消)，则保存取回的指令
                fetch_data_instr <= wbm_dat_i;
                fetch_data_pc    <= fetch_pc_inflight;
                fetch_data_valid <= 1'b1;
                fetch_data_pred_taken <= fetch_pred_taken;
                fetch_data_pred_target <= fetch_pred_target;
              end else begin
                fetch_discard <= 1'b0;
              end
            end
            if (wbm_err_i) begin
              // 总线错误处理
              bus_state      <= BUS_IDLE;
              fetch_inflight <= 1'b0;
              fetch_discard  <= 1'b0;
              instr_bus_error<= 1'b1;
              instr_error_pc <= fetch_pc_inflight;
            end
          end
          BUS_MEM: begin
            if (wbm_ack_i) begin
              bus_state           <= BUS_IDLE;
              mem_busy            <= 1'b0;
              mem_transaction_done<= 1'b1;
              if (!bus_we_reg) begin
                // 读操作完成，保存数据
                mem_transaction_data <= wbm_dat_i;
                mem_response_data    <= wbm_dat_i;
                mem_response_valid   <= 1'b1;
              end
            end
            if (wbm_err_i) begin
              // 总线错误处理
              bus_state            <= BUS_IDLE;
              mem_busy             <= 1'b0;
              mem_transaction_error<= 1'b1;
              mem_response_error   <= 1'b1;
            end
          end
        endcase
      end
      if (fetch_data_take) begin
        fetch_data_valid <= 1'b0;
      end
    end
  end

  // ------------------------------------------------------------
  // Pipeline register updates
  // ------------------------------------------------------------

  // IF/ID
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      if_id_reg <= '{default: '0};
    end else if (trap_flush || fencei_flush || mret_flush) begin
      if_id_reg.valid <= 1'b0;
    end else if (branch_flush) begin
      if_id_reg.valid <= 1'b0;
    end else if (!stall_id) begin
      if (fetch_data_take) begin
        if_id_reg.valid <= 1'b1;
        if_id_reg.pc    <= fetch_data_pc;
        if_id_reg.instr <= fetch_data_instr;
        if_id_reg.pred_taken <= fetch_data_pred_taken;
        if_id_reg.pred_target <= fetch_data_pred_target;
      end else begin
        if_id_reg.valid <= 1'b0;
      end
    end
  end

  // ID/EX
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      id_ex_reg <= '0;
    end else if (trap_flush || fencei_flush || mret_flush) begin
      id_ex_reg.valid <= 1'b0;
    end else if (!stall_ex) begin
      if (branch_flush) begin
        id_ex_reg <= '0;
      end else if (load_use_hazard) begin
        // 检测到加载使用冒险，暂停ID阶段并清除EX阶段
        id_ex_reg.valid <= 1'b0;
      end else begin
        if (if_id_reg.valid) begin
          id_ex_reg.valid       <= 1'b1;
          id_ex_reg.pc          <= id_pc;
          id_ex_reg.instr       <= id_instr;
          id_ex_reg.imm         <= id_imm_value;
          id_ex_reg.rd          <= id_instr[11:7];
          id_ex_reg.rs1         <= id_instr[19:15];
          id_ex_reg.rs2         <= id_instr[24:20];
          id_ex_reg.rs1_value   <= id_rs1_value;
          id_ex_reg.rs2_value   <= id_rs2_value;
          id_ex_reg.op_a_sel    <= id_op_a_sel;
          id_ex_reg.op_b_sel    <= id_op_b_sel;
          id_ex_reg.alu_op      <= id_alu_op;
          id_ex_reg.branch      <= id_branch;
          id_ex_reg.jump        <= id_jump;
          id_ex_reg.jalr        <= id_jalr;
          id_ex_reg.mem_read    <= id_mem_read;
          id_ex_reg.mem_write   <= id_mem_write;
          id_ex_reg.mem_size    <= id_mem_size;
          id_ex_reg.mem_unsigned<= id_mem_unsigned;
          id_ex_reg.reg_write   <= id_reg_write;
          id_ex_reg.wb_src      <= id_wb_src;
          id_ex_reg.csr_en      <= id_csr_en;
          id_ex_reg.csr_op      <= id_csr_op;
          id_ex_reg.csr_imm     <= id_csr_imm;
          id_ex_reg.csr_addr    <= id_csr_addr;
          id_ex_reg.csr_rdata   <= id_csr_rdata;
          id_ex_reg.csr_operand <= csr_operand_value;
          id_ex_reg.is_ecall    <= id_is_ecall;
          id_ex_reg.is_ebreak   <= id_is_ebreak;
          id_ex_reg.is_mret     <= id_is_mret;
          id_ex_reg.is_fencei   <= id_is_fencei;
          id_ex_reg.is_sfence_vma <= id_is_sfence_vma;
          id_ex_reg.illegal     <= id_illegal;
          id_ex_reg.use_rs1     <= id_use_rs1;
          id_ex_reg.use_rs2     <= id_use_rs2;
          id_ex_reg.pred_taken  <= if_id_reg.pred_taken;
          id_ex_reg.pred_target <= if_id_reg.pred_target;
        end else begin
          id_ex_reg <= '0;
        end
      end
    end
  end

  // EX/MEM
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      ex_mem_reg          <= '0;
      ex_mem_request_done <= 1'b0;
    end else if (trap_flush) begin
      if (ex_mem_reg.trap_valid || mem_bus_trap_valid || (page_fault && (walk_source == 1'b1))) begin
        ex_mem_reg.valid      <= 1'b0;
        ex_mem_reg.trap_valid <= 1'b0;
      end
      ex_mem_request_done   <= 1'b0;
    end else begin
      if (!mem_busy && !mem_request_active) begin
      ex_mem_reg.valid        <= id_ex_reg.valid;
      ex_mem_reg.pc           <= id_ex_reg.pc;
      ex_mem_reg.pc_plus4     <= pc_plus4_ex;
      ex_mem_reg.rd           <= id_ex_reg.rd;
      ex_mem_reg.reg_write    <= id_ex_reg.reg_write;
      ex_mem_reg.wb_src       <= id_ex_reg.wb_src;
      ex_mem_reg.mem_read     <= id_ex_reg.mem_read;
      ex_mem_reg.mem_write    <= id_ex_reg.mem_write;
      ex_mem_reg.mem_size     <= id_ex_reg.mem_size;
      ex_mem_reg.mem_unsigned <= id_ex_reg.mem_unsigned;
      ex_mem_reg.alu_result   <= alu_result;
      ex_mem_reg.rs2_value    <= ex_rs2_value;
      ex_mem_reg.branch_target<= branch_target_ex;
      ex_mem_reg.branch_taken <= branch_taken_ex;
      ex_mem_reg.jump         <= id_ex_reg.jump;
      ex_mem_reg.jalr         <= id_ex_reg.jalr;
      ex_mem_reg.csr_en       <= id_ex_reg.csr_en;
      ex_mem_reg.csr_write_en <= csr_write_enable_ex;
      ex_mem_reg.csr_wdata    <= csr_write_value;
      ex_mem_reg.csr_rdata    <= id_ex_reg.csr_rdata;
      ex_mem_reg.csr_addr     <= id_ex_reg.csr_addr;
      ex_mem_reg.is_ecall     <= id_ex_reg.is_ecall;
      ex_mem_reg.is_ebreak    <= id_ex_reg.is_ebreak;
      ex_mem_reg.is_mret      <= id_ex_reg.is_mret;
      ex_mem_reg.is_fencei    <= id_ex_reg.is_fencei;
      ex_mem_reg.is_sfence_vma<= id_ex_reg.is_sfence_vma;
      ex_mem_reg.illegal      <= id_ex_reg.illegal;
      ex_mem_reg.trap_valid   <= ex_trap_valid;
      ex_mem_reg.trap_cause   <= ex_trap_cause;
      ex_mem_reg.trap_value   <= ex_trap_tval;
      ex_mem_reg.mem_addr     <= alu_result;
      ex_mem_reg.store_sel    <= store_sel_comb;
      ex_mem_reg.store_data   <= store_data_comb;
      ex_mem_request_done     <= 1'b0;
      end

      if (mem_transaction_done) begin
        ex_mem_request_done <= 1'b1;
      end
    end
  end

  assign mem_commit = ex_mem_reg.valid &&
                      (
                          (!ex_mem_reg.mem_read && !ex_mem_reg.mem_write) ||
                          (ex_mem_reg.mem_read  && mem_response_valid)      ||
                          (ex_mem_reg.mem_write && mem_transaction_done)
                      );
  logic mret_commit;
  assign mret_commit = mem_commit && ex_mem_reg.is_mret && !ex_mem_reg.trap_valid;
  logic instr_retired;
  assign instr_retired = mem_commit && !ex_mem_reg.trap_valid;

  always_comb begin
    logic [7:0]  load_byte;
    logic [15:0] load_half;
    load_byte = mem_transaction_data >> (8 * ex_mem_reg.mem_addr[1:0]);
    load_half = mem_transaction_data >> (16 * ex_mem_reg.mem_addr[1]);
    mem_load_data = mem_transaction_data;
    case (ex_mem_reg.mem_size)
      MEM_SIZE_BYTE: begin
        if (ex_mem_reg.mem_unsigned)
          mem_load_data = {24'h0, load_byte};
        else
          mem_load_data = {{24{load_byte[7]}}, load_byte};
      end
      MEM_SIZE_HALF: begin
        if (ex_mem_reg.mem_unsigned)
          mem_load_data = {16'h0, load_half};
        else
          mem_load_data = {{16{load_half[15]}}, load_half};
      end
      default: mem_load_data = mem_transaction_data;
    endcase
  end

  // MEM/WB
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      mem_wb_reg <= '0;
    end else if (trap_flush && (mem_bus_trap_valid || (page_fault && walk_source == 1'b1))) begin
      // 如果MEM阶段有内存总线陷阱或页错误，刷新MEM/WB
      // 注意：EX阶段的ECALL陷阱在EX阶段处理，不会传播到MEM阶段
      mem_wb_reg.valid <= 1'b0;
    end else if (mem_commit) begin
      mem_wb_reg.valid     <= ex_mem_reg.valid && !ex_mem_reg.trap_valid;
      mem_wb_reg.rd        <= ex_mem_reg.rd;
      mem_wb_reg.reg_write <= ex_mem_reg.reg_write && !ex_mem_reg.trap_valid;
      mem_wb_reg.wb_src    <= ex_mem_reg.wb_src;
      mem_wb_reg.alu_result<= ex_mem_reg.alu_result;
      mem_wb_reg.pc_plus4  <= ex_mem_reg.pc_plus4;
      mem_wb_reg.csr_data  <= ex_mem_reg.csr_rdata;
      if (ex_mem_reg.mem_read) begin
        mem_wb_reg.mem_data <= mem_load_data;
      end else
        mem_wb_reg.mem_data <= ex_mem_reg.alu_result;
    end else if (!stall_ex) begin
      // 如果流水线前进但我们没有提交(例如气泡或已完成的非内存指令)，
      // 我们必须使mem_wb_reg无效，或者如果它是有效的非内存指令则更新它。
      // 注意，非内存指令立即设置mem_commit=1。
      // 所以这个分支只在mem_commit=0时执行。
      // 这意味着ex_mem_reg无效，或者它是一个等待的内存指令(但stall_ex=0??)。
      // 如果stall_ex=0，表示我们不在等待。
      // 所以它必须是无效的。
      mem_wb_reg.valid <= 1'b0;
    end
  end

  // CSR state update
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      csr_mstatus  <= 32'h0000_0000;
      csr_mtvec    <= RESET_VECTOR;
      csr_mepc     <= RESET_VECTOR;
      csr_mcause   <= 32'h0;
      csr_mtval    <= 32'h0;
      csr_mscratch <= 32'h0;
      csr_mie      <= 32'h0; // 默认禁用所有中断，让内核使能它们
      csr_mip      <= 32'h0;
      csr_mcounteren <= 32'h0;
      csr_mcycle   <= 32'h0;
      csr_minstret <= 32'h0;
      priv_mode    <= PRIV_M;
      csr_satp     <= 32'h0;
    end else begin
      csr_mcycle <= csr_mcycle + 32'd1;
      if (instr_retired)
        csr_minstret <= csr_minstret + 32'd1;
      
      // 根据RISC-V规范，mip.MTIP和mip.MEIP位由硬件自动设置
      // 软件只能清除这些位，不能设置它们
      csr_mip[11] <= external_interrupt_i;
      csr_mip[7]  <= timer_interrupt_i;

      if (trap_active) begin
        csr_mepc   <= trap_mepc_reg;
        csr_mcause <= trap_cause_reg;
        csr_mtval  <= trap_tval_reg;
        csr_mstatus<= csr_mstatus_trap_value;
        priv_mode  <= PRIV_M;
      end else if (mret_commit) begin
        csr_mstatus<= csr_mstatus_mret_value;
        priv_mode  <= mret_target_priv;
      end else if (ex_mem_reg.csr_en && ex_mem_reg.csr_write_en && mem_commit && !ex_mem_reg.trap_valid) begin
        unique case (ex_mem_reg.csr_addr)
          12'h300: csr_mstatus  <= ex_mem_reg.csr_wdata;
          12'h304: csr_mie      <= ex_mem_reg.csr_wdata;
          12'h305: csr_mtvec    <= ex_mem_reg.csr_wdata;
          12'h306: csr_mcounteren <= ex_mem_reg.csr_wdata;
          12'h340: csr_mscratch <= ex_mem_reg.csr_wdata;
          12'h341: csr_mepc     <= ex_mem_reg.csr_wdata;
          12'h342: csr_mcause   <= ex_mem_reg.csr_wdata;
          12'h343: csr_mtval    <= ex_mem_reg.csr_wdata;
          // ?????ip.MTIP (bit 7) ??mip.MEIP (bit 11) ???????????????
          // ??????????????????
          12'h344: csr_mip      <= (ex_mem_reg.csr_wdata & ~32'h880) |
                                     (csr_mip & 32'h880);
          12'hB00: csr_mcycle   <= ex_mem_reg.csr_wdata;
          12'hB02: csr_minstret <= ex_mem_reg.csr_wdata;
          12'h180: csr_satp     <= ex_mem_reg.csr_wdata;
          default: ;
        endcase
      end
    end
  end

  // ------------------------------------------------------------
  // BTB Update
  // ------------------------------------------------------------
  
  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      for (int i = 0; i < BTB_SIZE; i++) begin
        btb[i].valid <= 1'b0;
        btb[i].counter <= 2'b01; // 弱不跳转
      end
    end else if (id_ex_reg.valid && (id_ex_reg.branch != BR_NONE || id_ex_reg.jump)) begin
      logic [BTB_INDEX_BITS-1:0] update_index;
      logic [BTB_TAG_BITS-1:0]   update_tag;
      logic tag_match;
      
      update_index = id_ex_reg.pc[BTB_INDEX_BITS+1:2];
      update_tag   = id_ex_reg.pc[31:BTB_INDEX_BITS+2];
      tag_match    = btb[update_index].valid && (btb[update_index].tag == update_tag);
      
      btb[update_index].valid  <= 1'b1;
      btb[update_index].tag    <= update_tag;
      btb[update_index].target <= branch_target_ex;
      
      if (tag_match) begin
        if (branch_taken_ex) begin
          if (btb[update_index].counter != 2'b11)
            btb[update_index].counter <= btb[update_index].counter + 1;
        end else begin
          if (btb[update_index].counter != 2'b00)
            btb[update_index].counter <= btb[update_index].counter - 1;
        end
      end else begin
        // 新条目或冲突
        if (branch_taken_ex)
          btb[update_index].counter <= 2'b10; // 弱跳转
        else
          btb[update_index].counter <= 2'b01; // 弱不跳转
      end
    end
  end

  // ------------------------------------------------------------
  // I-Cache Update
  // ------------------------------------------------------------

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      for (int i = 0; i < (1<<ICACHE_INDEX_BITS); i++) begin
        icache_valid[i] <= 1'b0;
      end
    end else if (flush_icache) begin
      for (int i = 0; i < (1<<ICACHE_INDEX_BITS); i++) begin
        icache_valid[i] <= 1'b0;
      end
    end else if (bus_state == BUS_IFETCH && wbm_ack_i && !fetch_discard) begin
       icache_valid[fetch_pc_inflight[ICACHE_INDEX_BITS+1:2]] <= 1'b1;
       icache_tag[fetch_pc_inflight[ICACHE_INDEX_BITS+1:2]]   <= fetch_pc_inflight[31:ICACHE_INDEX_BITS+2];
       icache_data[fetch_pc_inflight[ICACHE_INDEX_BITS+1:2]]  <= wbm_dat_i;
    end
  end

  // ------------------------------------------------------------
  // Debug assignments
  // ------------------------------------------------------------

  assign dbg_pc    = if_id_reg.pc;
  assign dbg_instr = if_id_reg.instr;

  // ------------------------------------------------------------
  // Helper functions
  // ------------------------------------------------------------

  function automatic logic [31:0] imm_i(input logic [31:0] instr);
    imm_i = {{20{instr[31]}}, instr[31:20]};
  endfunction

  function automatic logic [31:0] imm_s(input logic [31:0] instr);
    imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  endfunction

  function automatic logic [31:0] imm_b(input logic [31:0] instr);
    imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
  endfunction

  function automatic logic [31:0] imm_u(input logic [31:0] instr);
    imm_u = {instr[31:12], 12'h000};
  endfunction

  function automatic logic [31:0] imm_j(input logic [31:0] instr);
    imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
  endfunction

  function automatic logic [31:0] ctz32(input logic [31:0] value);
    integer idx;
    logic [31:0] result;
    begin
      result = 32;
      for (idx = 0; idx < 32; idx = idx + 1) begin
        if ((result == 32) && value[idx]) begin
          result = idx;
        end
      end
      ctz32 = result;
    end
  endfunction

  function automatic logic csr_is_readonly(input logic [11:0] addr);
    case (addr)
      12'h301: csr_is_readonly = 1'b1;  // misa
      12'hF11, 12'hF12, 12'hF13, 12'hF14: csr_is_readonly = 1'b1;  // vendor info
      12'hC00, 12'hC01, 12'hC02: csr_is_readonly = 1'b1;  // cycle/time/instret (用户视图)
      12'hB80, 12'hB82: csr_is_readonly = 1'b1;  // 高位计数器(未实现)
      default: csr_is_readonly = 1'b0;
    endcase
  endfunction

  function automatic logic csr_legal(input logic [11:0] addr);
    case (addr)
      12'h300, 12'h301, 12'h304, 12'h305,
      12'h306,
      12'h340, 12'h341, 12'h342, 12'h343, 12'h344,
      12'h3A0, 12'h3B0, // pmpcfg0, pmpaddr0
      12'hB00, 12'hB02, 12'hB80, 12'hB82,
      12'hC00, 12'hC01, 12'hC02,
      12'hF11, 12'hF12, 12'hF13, 12'hF14,
      12'h180: csr_legal = 1'b1;
      default: csr_legal = 1'b0;
    endcase
  endfunction

  function automatic logic [31:0] csr_read(input logic [11:0] addr);
    case (addr)
      12'h300: csr_read = csr_mstatus;
      12'h301: csr_read = CSR_MISA_VALUE;
      12'h304: csr_read = csr_mie;
      12'h305: csr_read = csr_mtvec;
      12'h306: csr_read = csr_mcounteren;
      12'h340: csr_read = csr_mscratch;
      12'h341: csr_read = csr_mepc;
      12'h342: csr_read = csr_mcause;
      12'h343: csr_read = csr_mtval;
      12'h344: csr_read = csr_mip;
      12'h3A0: csr_read = 32'h0; // pmpcfg0
      12'h3B0: csr_read = 32'h0; // pmpaddr0
      12'hB00: csr_read = csr_mcycle;
      12'hB02: csr_read = csr_minstret;
      12'hB80: csr_read = 32'h0;
      12'hB82: csr_read = 32'h0;
      12'hC00: csr_read = csr_mcycle;
      12'hC01: csr_read = csr_mcycle;
      12'hC02: csr_read = csr_minstret;
      12'hF11: csr_read = 32'h0;  // mvendorid
      12'hF12: csr_read = 32'h0;  // marchid
      12'hF13: csr_read = 32'h0;  // mimpid
      12'hF14: csr_read = 32'h0;  // mhartid
      12'h180: csr_read = csr_satp;
      default: csr_read = 32'h0;
    endcase
  endfunction

endmodule

`default_nettype wire
