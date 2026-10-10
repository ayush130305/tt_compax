# tt_compax_memtrial: latch memory feasibility trial

A small Tiny Tapeout design whose only job is to answer one question for COMPAX: **does a latch-based 64-byte memory survive the Tiny Tapeout flow** (LibreLane hardening, precheck, gate-level test)?

Top module `tt_um_ayush130305_memtrial`, 25 MHz, 2x2 tiles (chosen so area cannot be the reason a run fails).

## What it is

`uart_rx` -> `cmd_fsm` -> `latch_mem` -> `uart_tx` (your verified UART blocks, unchanged).

`latch_mem` uses the standard clock-gating structure per row: an enable latch that is transparent while `clk` is low, an AND with `clk` to form a gated clock, and 8 data latches that are transparent while that gated clock is high. A write requested in cycle N is stored during the high phase of cycle N+1.

Commands: `W addr data` -> `K`; `R addr` -> data; `P addr n d...` -> `K` (page write, wraps at 64); `G addr n` -> n bytes; other -> `?`.

## Tests (`test/`)

Run `cd test && pip install -r requirements.txt && make` (needs iverilog). All via the UART pins.

1. reset: TX idles high
2. single write/read at first, last and a middle row
3. unknown command returns `?`
4. fill all 64 rows with three patterns (pattern, inverse, address) and read every row back
5. random single writes against a Python model (no cross-talk between rows)
6. page write that wraps from address 63 to 0

On RTL these pass, and breaking one row's decode makes test 4 fail. The suite takes about 2 minutes on RTL; the gate-level run will be slower.

## What to look at after pushing

- `test` green: RTL is fine.
- `gds` green: latches and gated clocks harden (placement, clock tree, hold repair).
- `gl_test` green: gate-level netlist behaves like the RTL. This is the real latch check.
- In the `gds` log, look for clock-tree warnings about the gated clocks, setup/hold lines, and the final utilization.

If it passes, build COMPAX on this memory. If `gl_test` or timing fails on the latches, fall back to flop storage (about 8.5k um2 more for 64 B).

Measured synthesized area (Yosys, sky130 hd tt): whole trial about 18.1k um2; 64 B latch memory alone about 12.0k; the same memory in flops about 20.5k.
