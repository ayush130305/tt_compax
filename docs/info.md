## How it works

COMPAX (Compressed On-chip Memory with Pipelined Array eXpansion) contains a
32-byte flip-flop memory and a lossless Base-Delta-Immediate (BDI) codec.
This implementation processes four little-endian 32-bit words per 16-byte block.
The codec processes one word per cycle using shared arithmetic. The UART
controller sequences scanning, encoding, decoding, and memory transfers.
The name does not imply that this revision has an allocator or a pipelined codec.

Commands at 115200 baud, 8N1, with a 25 MHz input clock:

- W (0x57), addr, data: write one raw byte; reply K (0x4B).
- R (0x52), addr: read one physical byte.
- P (0x50), addr, n, n data bytes: raw page write; reply K.
- G (0x47), addr, n: read n physical bytes.
- C (0x43), slot: compress a 16-byte block in place; reply K.
- D (0x44), slot: decompress a block in place; reply K.
- M (0x4D), slot: return two bytes: metadata and encoded payload length.
- Unknown commands or slot indices other than 0 and 1: reply ? (0x3F).

Slots 0 and 1 begin at byte addresses 0 and 16.
Metadata bits 2:0 are the format; bits 6:3 are the four immediate-selector
bits; bit 7 is zero. Selector bit i chooses zero as the base for word i.
Format 0 is raw (16 bytes); 1 is zero (0 payload bytes); 2 is repeated
32-bit word (4 bytes); 3 is base32 plus four signed delta8 values (8 bytes);
4 is base32 plus four signed delta16 values (12 bytes).
Delta payloads store a four-byte base followed by four deltas, little-endian.
The encoder selects the smallest supported lossless format. Other data stays raw.
The chosen base is the first word that does not fit as an immediate, or word zero
when all words fit. Word arithmetic wraps modulo 2^32.

C and D are idempotent for already compressed and already raw slots respectively.
Read commands expose the physical encoded bytes until D restores the original.
Compressed payloads occupy the beginning of the slot; unused bytes are zeroed.
Single-byte writes to a compressed slot return E (0x45) without writing.
Nonempty page writes return E without writing when any slot is compressed;
the controller still consumes the entire requested data sequence.
Decompress all slots before a page write. Writes to other raw slots remain allowed.

Addresses use their low five bits and wrap at 32. Page lengths use their low
seven bits (0 through 127); 128 therefore means zero. Zero-length writes reply K;
zero-length reads send nothing. Wait for a command's complete reply before
sending the next command. There is no receive FIFO or partial-command timeout.

Physical capacity is 32 bytes in two fixed slots. Metadata is separate from
payload storage. Compression does not yet allow additional logical blocks:
there is no allocator, compaction, or reuse of the padding space. This is a
16-byte, 32-bit-word BDI subset, not all formats of the original cache-line codec.
Reset clears format metadata without clearing the memory. Reload blocks after
reset; compressed data is not recoverable after its metadata is cleared.

## How to test

Connect a 3.3 V USB-serial adapter to ui[0] (RX) and uo[0] (TX).
Use 115200 8N1 and supply a 25 MHz clock.
Send P, 00, 10, followed by sixteen zero bytes; expect K.
Send C, 00; expect K. Send M, 00; expect 01, 00 (zero format).
Send D, 00; expect K. Send G, 00, 10; expect sixteen zero bytes.
Here numeric byte values are hexadecimal and letters denote ASCII commands.

## External hardware

3.3 V USB-serial adapter and a suitable clock source.
