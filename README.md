# COMPAX BDI replacement files

This bundle updates the current 64-byte design. It does not change the metal
artwork, macro placement, or flow configuration. Keep src/config.json and macros/.

Renames:
- src/project.v -> src/compax_top.v
- src/cmd_fsm.v -> src/compax_controller.v
- src/latch_mem.v -> src/flop_mem.v
- top tt_um_ayush130305_memtrial -> tt_um_ayush130305_compax

New synthesizable modules: bdi_encoder and bdi_decoder. Both are instantiated
by compax_controller, which is instantiated by compax_top. The storage and UART
modules are also reachable from the synthesis top. compax_art remains a physical
black-box macro with a gate-level simulation stub.

Update info.yaml, test/Makefile, test/tb.v, test/test.py, and docs/info.md with
the included files. Remove the three obsolete source files after copying.
UART source files are included unchanged for reproducible standalone simulation.

Local Icarus verification: 10000 randomized codec round trips plus directed
format selection, and full UART integration including encode/decode, metadata,
repeat commands, write rejection, independent slots, raw reads/writes and wrap.
The original six cocotb tests are retained and two codec UART tests are added.
Cocotb, Verilator, synthesis, timing, DRC, LVS and gate-level execution must be
checked in CI; no successful physical implementation is claimed for this revision.

See docs/info.md for the complete protocol and fixed-slot capacity limitations.
The 128-bit working buffer adds 128 storage bits; four format/mask entries add
28 bits, plus controller registers and combinational arithmetic. Area and timing
will grow relative to the working memory-only design. Tile fit is unverified.

BDI reference: https://users.ece.cmu.edu/~omutlu/pub/bdi-compression_pact12.pdf
