# COD25 - Thinpad RV32I 五级流水线 CPU

清华大学计算机组成原理课程大作业，基于 Thinpad 实验平台实现 RV32I 处理器。

## CPU 特性

- **指令集**：完整 RV32I + B 扩展自定义指令（`MINU`, `CTZ`, `XNO`）
- **架构**：五级流水线（IF → ID → EX → MEM → WB）
- **总线**：Wishbone Classic 协议，单主多从
- **外设**：BaseRAM、ExtRAM、UART 串口
- **异常处理**：支持 ECALL、EBREAK、非法指令、对齐异常等
- **Hazard 处理**：EX/MEM 和 MEM/WB 数据转发 + Load-Use 气泡阻塞

## 工程结构

```
thinpad_top.srcs/
├── constrs_1/new/    # 引脚约束 (XDC)
├── sources_1/
│   ├── ip/           # IP 核 (PLL)
│   └── new/
│       ├── cpu/      # RV32I CPU 核心
│       ├── lab0/     # Lab0 顶层
│       ├── lab2/     # Lab2 顶层
│       ├── lab3/     # Lab3 SRAM 控制器
│       ├── lab4/     # Lab4 UART 控制器 + CLINT
│       ├── docs/     # 设计文档
│       └── thinpad_top.sv  # 顶层模块
└── sim_1/new/        # 仿真测试文件
```

## 使用说明

1. 使用 Vivado 打开 `thinpad_top.xpr`
2. 代码使用 UTF-8 编码，Windows 下可能出现乱码，请用外部编辑器转为 GBK
3. 综合、实现后生成比特流文件

## 开发环境

- Vivado 2024.2
- 目标器件：Artix-7 XC7A100TFGG484-1 (Thinpad)
