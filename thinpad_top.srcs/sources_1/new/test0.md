实验内容
设计并实现一个能够运行上述程序的多周期或流水线处理器，运行给定的汇编程序，能够从串口看到输出的效果。上述代码编译生成二进制文件之后通过工具装入到 BaseRAM 里面，启动时从 BaseRAM 的起始地址执行程序，按复位键开始运行。使用 Thinpad 云平台工具，观察程序的输出。

实验原理
本实验是支持多条指令的处理器。具体的多周期和流水线原理请参考计算机组成原理教材、课件和本实验指导书。

需要掌握基本的 CPU 设计方法。若设计流水线 CPU，则需要理解 CPU 中可能存在的结构冲突、数据冲突和控制冲突。对于冲突的处理只需要使用延迟冲突（即插入气泡）处理方法即可，不需要使用其它方式（例如数据旁路）来提高性能。

实验步骤
本实验比较复杂，是一个初步的处理器设计实验，作为设计可以执行监控程序的流水线处理器的铺垫。因此，在做本实验的过程中，需要比较深入理解处理器执行每一条指令的流程。

分析所编写的汇编代码，依据每一条指令所需要完成的功能（参考 RISC-V 指令集手册），划分每条指令的执行步骤，设计指令流程图。
实现 Wishbone Master，通过总线协议访问 SRAM 和 UART，二者通过不同的地址进行区分。
依据单周期的处理器设计方法把所需要的指令都加入到数据通路中，并标记各条线路上的信号名称，引出各条指令所需要的信号。
划分处理器的各功能部件和阶段设计，给出处理器的概要结构图，并标识出各主要信号及数据流向、阶段寄存器需要保存的各类信息。
细化各功能部件，设计出包含每个部件的外部控制信号以及数据信号的详细结构图，并根据指令流程图在该结构图上执行每条指令，检查指令执行是否正确。若设计流水线 CPU，使用流水线延迟（插入气泡）的方式对流水线冲突进行处理。
确认结构图中的每个功能部件的具体功能和外部连接信号，注意时序之间的配合。使用硬件描述语言设计实现每个功能部件并且使用软件进行仿真。仿真过程中尽量将各种情况的输入都加入，保证每个部件能够按照预定功能运行。
连接各个功能部件组成整体的 CPU，并对其进行软件仿真。
连接各个外设和 CPU 形成计算机系统，并对其进行软件仿真。
程序的二进制文件装入到 BaseRAM 中，将设计好的 CPU 装在到 FPGA 中，进行实际硬件调试。


需要支持的测试代码：

    addi t0, zero, 0     # loop variable
    addi t1, zero, 100   # loop upper bound
    addi t2, zero, 0     # sum
loop:
    addi t0, t0, 1
    add t2, t0, t2
    beq t0, t1, next # i == 100?
    beq zero, zero, loop

next:   
    # store result
    lui t0, 0x80000  # base ram address
    sw t2, 0x100(t0)

    lui t0, 0x10000  # serial address
.TESTW1:
    lb t1, 5(t0)
    andi t1, t1, 0x20
    beq t1, zero, .TESTW1 
    # do not write when serial is in used

    addi a0, zero, 'd'
    sb a0, 0(t0)

.TESTW2:
    lb t1, 5(t0)
    andi t1, t1, 0x20
    beq t1, zero, .TESTW2

    addi a0, zero, 'o'
    sb a0, 0(t0)

.TESTW3:
    lb t1, 5(t0)
    andi t1, t1, 0x20
    beq t1, zero, .TESTW3

    addi a0, zero, 'n'
    sb a0, 0(t0)

.TESTW4:
    lb t1, 5(t0)
    andi t1, t1, 0x20
    beq t1, zero, .TESTW4

    addi a0, zero, 'e'
    sb a0, 0(t0)

.TESTW5:
    lb t1, 5(t0)
    andi t1, t1, 0x20
    beq t1, zero, .TESTW5

    addi a0, zero, '!'
    sb a0, 0(t0)

end:
    beq zero, zero, end 
    # loop forever, let pc under control


预期的效果如下：

可以看到在串口上能够看到"done!"的输出，同时在 BaseRAM 的0x100地址处可以看到0x13ba=5050的结果，以小端方式存储。