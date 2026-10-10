# COMPAX sequential BDI update

Apply this bundle to the current artwork-free COMPAX repository.
UART protocol, four 16-byte slots, raw fallback, immediate masks, signed delta
formats and write protection are preserved. Physical memory remains 64 bytes.

The controller now scans and tests one word per clock, packing forwards and
unpacking backwards. A lookahead register preserves source values during
in-place buffer updates. One shared 32-bit add/subtract datapath handles both
encoding and decoding. The existing 128-bit staging buffer remains; this update
reduces arithmetic replication rather than adding a second block buffer.

Remove src/bdi_encoder.v and src/bdi_decoder.v from the repository; they are
replaced by src/bdi_word_alu.v and sequential states in compax_controller.v.
info.yaml and test/Makefile select the new source. The reference encoder under
test/ is simulation-only and is not listed in the synthesis source files.

Validation with Icarus:
- 10000 randomized complete encode/store/decode operations compare every payload
  byte, tag and immediate mask against the previous reference encoder.
- Directed signed delta boundaries, zero/repeated formats, mixed immediates and
  a first immediate followed by a non-immediate base all pass.
- Full UART regression passes with 361 responses: three full-memory patterns,
  five formats, repeated commands, metadata, write protection, invalid slots,
  independent slots, raw access and page wrapping.

No synthesis, Verilator or physical flow was available locally. The exact area
saving, timing and successful fit are unverified until the new CI run completes.
The 2x2 tile choice, current flow settings, UART and artwork-free top are unchanged.

The original combinational codec_tb.sv is obsolete and should be removed.
Use test/sequential_codec_tb.sv with test/bdi_reference_encoder.v for native
randomized verification. Existing cocotb tests remain unchanged in the repository.
