# test.py
# cocotb tests for tt_um_ayush130305_memtrial: a UART-controlled 64-byte latch memory.
# Every test drives the chip only through the UART pins, exactly like a real host would.

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

CLK_NS = 40        # 25 MHz
BIT_CLKS = 217     # 25 MHz / 115200 baud
DEPTH = 64


def tx_bit(dut):
    """Return the UART TX pin as '0', '1' or something else if it is X."""
    s = str(dut.uo_out.value)
    return s[-1]


dut_q = []   # bytes decoded from the chip's TX pin, shared by monitor() and recv_byte()


async def start(dut):
    """Start the clock, idle the pins, apply reset and start the background UART receiver."""
    dut_q.clear()
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 1          # UART RX idles high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 6)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)
    cocotb.start_soon(monitor(dut, dut_q))


async def send_byte(dut, data):
    """Send one UART byte (8N1) on ui_in[0]."""
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, BIT_CLKS)
    for i in range(8):
        dut.ui_in.value = (data >> i) & 1
        await ClockCycles(dut.clk, BIT_CLKS)
    dut.ui_in.value = 1
    await ClockCycles(dut.clk, BIT_CLKS)


async def monitor(dut, q):
    """Background UART receiver: decodes every frame on uo_out[0] into the list q."""
    while True:
        await RisingEdge(dut.clk)
        if tx_bit(dut) == '0':
            await ClockCycles(dut.clk, BIT_CLKS + BIT_CLKS // 2)
            v = 0
            for i in range(8):
                v |= int(tx_bit(dut)) << i
                await ClockCycles(dut.clk, BIT_CLKS)
            q.append(v)


async def recv_byte(dut, timeout_bits=400):
    """Pop the next byte decoded by the monitor; fail the test if none arrives in time."""
    for _ in range(timeout_bits * BIT_CLKS):
        if dut_q:
            return dut_q.pop(0)
        await RisingEdge(dut.clk)
    raise AssertionError("no UART byte received")


async def write_byte(dut, addr, data):
    for b in (0x57, addr, data):
        await send_byte(dut, b)
    assert await recv_byte(dut) == 0x4B, "missing 'K' after write"


async def read_byte(dut, addr):
    for b in (0x52, addr):
        await send_byte(dut, b)
    return await recv_byte(dut)


async def write_page(dut, addr, data):
    for b in [0x50, addr, len(data)] + list(data):
        await send_byte(dut, b)
    assert await recv_byte(dut) == 0x4B, "missing 'K' after page write"


async def read_page(dut, addr, n):
    for b in (0x47, addr, n):
        await send_byte(dut, b)
    return [await recv_byte(dut) for _ in range(n)]


@cocotb.test()
async def test_01_reset(dut):
    """UART TX idles high after reset."""
    await start(dut)
    assert tx_bit(dut) == '1'


@cocotb.test()
async def test_02_single_write_read(dut):
    """Single-byte write/read at the first, last and a middle address."""
    await start(dut)
    for addr, val in ((0, 0xA5), (63, 0x5A), (21, 0xFF)):
        await write_byte(dut, addr, val)
    for addr, val in ((0, 0xA5), (63, 0x5A), (21, 0xFF)):
        got = await read_byte(dut, addr)
        assert got == val, f"addr {addr}: wrote {val:#x}, read {got:#x}"


@cocotb.test()
async def test_03_unknown_command(dut):
    """An unknown command byte is answered with '?'."""
    await start(dut)
    await send_byte(dut, 0x41)
    assert await recv_byte(dut) == 0x3F


@cocotb.test()
async def test_04_full_array_patterns(dut):
    """Fill all 64 rows with three different patterns; every row must read back exactly."""
    await start(dut)
    pat_a = [(i * 37 + 11) & 0xFF for i in range(DEPTH)]
    patterns = [pat_a, [(~x) & 0xFF for x in pat_a], list(range(DEPTH))]
    for k, pat in enumerate(patterns):
        await write_page(dut, 0, pat)
        got = await read_page(dut, 0, DEPTH)
        bad = [i for i in range(DEPTH) if got[i] != pat[i]]
        assert not bad, f"pattern {k}: rows {bad} differ (got {[hex(got[i]) for i in bad]})"


@cocotb.test()
async def test_05_no_crosstalk(dut):
    """Writing one row must not disturb any other row (random single writes against a model)."""
    await start(dut)
    rng = random.Random(5)
    model = [0] * DEPTH
    await write_page(dut, 0, model)
    for _ in range(12):
        addr, val = rng.randrange(DEPTH), rng.getrandbits(8)
        await write_byte(dut, addr, val)
        model[addr] = val
    got = await read_page(dut, 0, DEPTH)
    assert got == model, "array differs from model after random single writes"


@cocotb.test()
async def test_06_page_wrap(dut):
    """A page write starting near the end wraps to address 0."""
    await start(dut)
    await write_page(dut, 0, [0] * DEPTH)
    await write_page(dut, 62, [0x11, 0x22, 0x33, 0x44])
    assert await read_page(dut, 62, 2) == [0x11, 0x22]
    assert await read_page(dut, 0, 2) == [0x33, 0x44]
