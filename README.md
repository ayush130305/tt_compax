# tt_compax_memtrial: 128-byte flop memory trial

A UART-controlled 128-byte memory trial for Tiny Tapeout. This design does not yet implement COMPAX compression, allocation, or decompression.

Top module `tt_um_ayush130305_memtrial`, 25 MHz, 2x2 tiles. Physical fit and timing must be verified by a new GDS run.

## Storage and interface

`uart_rx` feeds `cmd_fsm`, which controls `latch_mem` and `uart_tx`. Despite its retained module name, `latch_mem` implements 128 x 8 bits of rising-edge flip-flop storage with synchronous writes and combinational reads. Storage has no reset; write rows before reading them.

Commands: `W addr data` replies `K`; `R addr` replies with data; `P addr n d...` writes n bytes and replies `K`; `G addr n` reads n bytes; unknown commands reply `?`.

Addresses use the low seven bits and wrap modulo 128. Page lengths use the full byte, 0 through 255. A zero-length write replies `K`; a zero-length read sends nothing. Transfers exceeding 128 bytes revisit rows.

UART timing and the previous lint fixes are preserved.

## Validation

Run `cd test && make` with Icarus Verilog and the packages from `test/requirements.txt`. Tests cover reset, boundary addresses including 64 and 127, unknown commands, three full-array patterns, random writes, wraparound from 127 to 0, zero-length transfers, 255-byte transfers, and address-bit masking.

Run the Tiny Tapeout GDS, precheck, and gate-level workflows to establish physical feasibility. The earlier 64-byte results do not validate this expanded version.
