# RV32I 五级流水 CPU 设计概述

## 1. 总览
- **指令集**：完整 RV32I + 自定义 B 扩展指令 `MINU`, `CTZ`, `XNO` (alias XNOR)。
- **流水级**：IF → ID → EX → MEM → WB，所有阶段单周期，存在阻塞式内存访问。
- **总线协议**：Wishbone Classic，单主多从。CPU 作为主设备，通过 `wb_mux_3` 访问 BaseRAM、ExtRAM、UART。
- **复位入口**：`0x8000_0000`，与监控程序及测试脚本保持一致。

```
取指(IF) ─► 译码(ID) ─► 执行(EX) ─► 访存(MEM) ─► 回写(WB)
     └────── Hazard/Flush 控制 ──────►
```

## 2. 关键数据通路

| 信号/寄存器 | 产生阶段 | 使用阶段 | 说明 |
|--------------|---------|---------|------|
| `pc_reg` | IF | IF | 程序计数器，支持分支、跳转、异常跳转。|
| `if_id_reg` | IF | ID | 缓存指令/PC，支持阻塞与清空。|
| `id_ex_reg` | ID | EX | 包含寄存器值、立即数、控制信号（ALU、分支、CSR、访存）。|
| `ex_mem_reg` | EX | MEM | 存储 ALU 结果、访存参数（地址、字节使能、写数据）、CSR 写入、异常信息。|
| `mem_wb_reg` | MEM | WB | 选择写回源（ALU/访存/PC+4/CSR），驱动寄存器堆写入。|
| `csr_*` | MEM/WB | 全局 | 简化的 Machine 模式 CSR（mstatus/mtvec/mepc/mcause/mtval/mie/mip/mscratch）。|

### ALU/扩展运算
- RV32I 所有算术逻辑指令（含移位）在 EX 阶段完成。
- B 扩展：
  - `MINU`: `min(rs1, rs2)`。
  - `CTZ`: 通过优先编码实现，依赖 `rs1`，忽略 `rs2`。
  - `XNO`: 作为 `XNOR`。 

### 分支/跳转
- EX 阶段根据 `branch_e`/`jump` 判定。
- 发生分支/跳转或异常时，IF/ID 阶段被 flush，PC 选择目标地址。

### 访存
- 地址范围映射：
  - `0x8000_0000 ~ 0x803F_FFFF` → BaseRAM。
  - `0x8040_0000 ~ 0x807F_FFFF` → ExtRAM。
  - `0x1000_0000 ~ 0x1000_FFFF` → UART。
- 采用阻塞式访问：发起 Wishbone 事务后 pipeline 暂停，直到 `ACK` 返回。
- 字节/半字/字访问：在 EX 阶段生成 `store_sel` + 对齐数据，MEM 阶段根据 `mem_unsigned` 做符号扩展。

### CSR & 异常
- CSR 指令在 EX 阶段决策写入值，MEM 阶段提交。
- 支持 `ECALL`、`EBREAK`、非法指令、取指/访存异常、对齐异常。
- 异常入口：`mtvec[31:2]<<2`，保存 `mepc/mcause/mtval`，并清流水。
- 中断、`MRET` 暂未实现，满足当前需求。

## 3. Hazard 与 Forwarding

| 冲突类型 | 处理策略 |
|----------|----------|
| 结构冲突 | 单主 Wishbone，访存阶段阻塞 pipeline（IF/ID/EX 同步暂停）。|
| 数据冲突 | 
  - EX/MEM → EX 转发，优先级高。
  - MEM/WB → EX 转发。
  - Load-Use：检测 `id_ex_reg.mem_read` 与后继寄存器冲突，插入气泡并阻塞 IF/ID。|
| 控制冲突 | 分支/跳转在 EX 判定，立即 flush IF/ID，PC 重定向。异常同理。|

## 4. 后续扩展接口
- **中断**：`csr_mstatus.mie/mip/mie` 已预留，后续可接 UART 中断信号。
- **虚拟内存**：可在 IF/ID 前插入 TLB/MMU，CSR 中可扩展 satp 等寄存器。
- **多级缓存**：当前访存为直连 Wishbone，可通过在 MEM 阶段引入 cache 模块提升性能。

## 5. 调试建议
1. **单元仿真**：优先对 `rv32i_core` 做行为仿真，重点关注 hazard、跳转、异常路径。
2. **Wishbone 回环**：复用 `lab3/lab4` 中的 SRAM/UART 模型，通过 testbench 执行基础读写。
3. **软件验证**：先运行官方 `rv32ui`/`rv32um` 指令测试，再加载监控程序与实验 1 汇编。调用 `lab4_tb` 框架可快速复用 RAM/串口模型。

此文档将随代码迭代不断更新，如需更多细节（状态机、时序等），可在该目录扩展子章节。