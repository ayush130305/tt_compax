## How it works

A UART (115200 baud at 25 MHz) controls a 64-byte memory built from latches instead of flip-flops. Commands:

| Command | Bytes sent | Reply |
|---|---|---|
| `W` (0x57) | addr, data | `K` (0x4B) |
| `R` (0x52) | addr | the data byte |
| `P` (0x50) page write | addr, n, n data bytes | `K` |
| `G` (0x47) page read | addr, n | n data bytes |
| anything else | | `?` (0x3F) |

Addresses use the low 6 bits and wrap at 64. This is a feasibility trial for latch storage.

## How to test

Connect a 3.3 V USB-serial adapter to `ui[0]` (chip RX) and `uo[0]` (chip TX), 115200 8N1. Send `57 05 A5`, expect `4B`; send `52 05`, expect `A5`.

## External hardware

3.3 V USB-serial adapter.
