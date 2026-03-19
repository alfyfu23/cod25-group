# Comparison Result: Current vs Reference

## 1. CPU Architecture
- **Current (`rv32i_core.sv`)**: 
  - **Type**: 5-Stage Pipeline (IF, ID, EX, MEM, WB).
  - **Complexity**: High. Requires complex hazard detection (Load-Use), forwarding (MEM->EX, WB->EX), and pipeline flushing (Branch).
  - **Risk**: Prone to subtle timing bugs, especially with high-latency I/O like UART. The failure to output "done!" (stuck after 'd') strongly suggests a **Load-Use Hazard** failure where the status register read (`LB`) following a write (`SB`) or loop is not correctly forwarded or stalled, causing the CPU to read '0' (Busy) indefinitely.
- **Reference (`ref/cpu.sv`)**:
  - **Type**: Multi-Cycle State Machine (FETCH, DECODE, EXEC, MEM, WB).
  - **Complexity**: Low. Instructions execute sequentially.
  - **Risk**: Very low. No hazards exist because only one instruction is active at a time.

## 2. UART Controller
- **Current (`lab4/uart_controller.sv`)**:
  - Modified to broadcast `REG_STATUS` to all byte lanes. This is a robust fix for alignment issues.
- **Reference (`ref/lab4/uart_controller.sv`)**:
  - Uses `wb_sel_i` to place data on specific byte lanes.
  - **Verdict**: Both should work. The current implementation is actually slightly more robust against CPU alignment quirks.

## 3. Top Level & Clocking
- **Current (`thinpad_top.sv`)**:
  - Uses `clk_50M` directly.
  - Removed PLL.
  - Reset via `reset_btn`.
- **Reference (`ref/thinpad_top.sv` & `ref/cpu_top.sv`)**:
  - Uses PLL (though CPU runs at 50MHz).
  - Reset uses `locked` signal.
  - **Verdict**: Direct 50MHz is fine. The PLL is not strictly necessary if the logic meets timing at 50MHz.

## Conclusion
The root cause of the `test0` failure is almost certainly within the **Pipeline Logic** of `rv32i_core.sv`. While the hazard detection logic appears present, pipeline CPUs are notoriously difficult to debug without cycle-accurate simulation waveforms.

The Reference implementation uses a **Multi-Cycle CPU**, which inherently avoids these hazards.

## Recommendation
To fix the issue immediately, **replace the Pipeline CPU (`rv32i_core`) with the Reference Multi-Cycle CPU (`multi_cycle_cpu`)** in `thinpad_top.sv`. This aligns with the "correct" reference provided.
