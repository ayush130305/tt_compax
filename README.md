# COMPAX: 32-byte flop memory with a BDI codec (Tiny Tapeout, SKY130, 2x2)

A UART-controlled 32-byte flip-flop memory with a lossless 16-byte Base-Delta-Immediate
(BDI) codec. The memory is two fixed 16-byte slots. Commands: `W`, `R`, `P`, `G` (raw access),
`C`/`D` (compress/decompress a slot in place) and `M` (metadata). See `docs/info.md`.

Top module `tt_um_ayush130305_compax`, 25 MHz, 115200 baud, 2x2 tiles.

## Why 32 bytes and no block buffer

The first 64-byte version measured about 47k um^2 in Yosys and was too large for 2x2:
detailed routing took 30 minutes at about 79% utilization. The codec controller was over half
of the design because of a 128-bit staging buffer with wide multiplexers.

The codec now works directly in the flop memory, one byte per cycle, with only a 32-bit word
register, a 32-bit base register and two 16-bit delta registers. Yosys area is about 26.8k um^2
(controller 11.6k, memory 10.1k, UARTs 3.9k, ALU 1.1k). `src/config.json` uses a 40 ns clock period
(the chip runs at 25 MHz) and zero hold margin to reduce timing-repair buffers.
Physical fit is still unverified until the GDS action completes.

## Validation (Icarus)

- `cd test && make`: 8 cocotb tests through the UART pins (raw access, full-array patterns,
  all five BDI formats, write protection, invalid slots).
- `sequential_codec_tb.sv` + `bdi_reference_encoder.v`: 10000 randomized encode/store/decode
  operations compared against a reference encoder.
- `uart_bdi_tb.sv`: native full-UART regression.

No compression-ratio gain yet: there is no allocator, so compression is in place and the
padding space is not reused.
