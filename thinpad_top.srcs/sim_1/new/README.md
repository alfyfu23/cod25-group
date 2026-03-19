# Simulation Entry Points

This folder now contains two standalone test benches that target the RV32I core directly. They provide a faster way to iterate on the CPU without rebuilding the entire Thinpad system.

## `simple_core_tb.sv`
Loads a tiny hand-written program out of a scratch memory (`simple_wb_mem`) and prints every Wishbone transaction. Use it when you only need to look at the pipeline behavior for a few instructions.

```
pwsh> iverilog -g2012 -o build/simple_core_tb.vvp \ 
         thinpad_top.srcs/sim_1/new/simple_core_tb.sv \ 
         thinpad_top.srcs/sources_1/new/cpu/rv32i_core.sv
pwsh> vvp build/simple_core_tb.vvp
```

## `waitboot_tb.sv`
Mimics the official "WaitBoot" monitor test:

- Provides a Wishbone environment with BaseRAM (0x8000_0000~0x803F_FFFF), ExtRAM (0x8040_0000~0x807F_FFFF) and the UART window at 0x1000_0000.
- Loads an instruction image from a Verilog hex file. By default it now points at `programs/waitboot_monitor.hex`, which was converted from the grader's actual BaseRAM snapshot (`chunk_00.bin`). If that file is missing it falls back to the legacy stub that only prints `WaitBoot`.
- Watches every byte written to the UART-mapped address and fails if the banner `WaitBoot` is not observed within 200,000 cycles. This reproduces the grader's `ERROR: timeout during WaitBoot` failure locally.

Run it with:

```
pwsh> iverilog -g2012 -o build/waitboot_tb.vvp \ 
         thinpad_top.srcs/sim_1/new/waitboot_tb.sv \ 
         thinpad_top.srcs/sources_1/new/cpu/rv32i_core.sv
pwsh> vvp build/waitboot_tb.vvp
```

### Real monitor image / overrides

`programs/waitboot_monitor.hex` was created from the grader's CBOR dump (`chunk_00.bin`). The helper script below regenerates it (and can be used for any other binary snapshot you obtain):

```
pwsh> python thinpad_top.srcs/sim_1/new/programs/bin2hex.py \
          --input thinpad_top.srcs/sources_1/new/data/decoded/chunk_00.bin \
          --output thinpad_top.srcs/sim_1/new/programs/waitboot_monitor.hex
```

If you want to try a different monitor/firmware image, convert your ELF/bin into Verilog hex and point the test bench at it:

```
# Example: dump ELF into a Verilog hex file
riscv64-unknown-elf-objcopy -O verilog kernel.elf waitboot.hex

# Override the default image path during compilation
iverilog -g2012 -P waitboot_tb.BASE_INIT_FILE="/abs/path/waitboot.hex" ...
```

Once the UART emits the string `WaitBoot`, the simulation ends automatically; otherwise it reports a timeout together with the number of matching characters that were seen. Use the UART log inside the transcript to compare with the grader's expectations.
