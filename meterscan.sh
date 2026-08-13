#!/usr/bin/env python3
"""Scan Modbus input registers (3x / function 0x04) and decode 32-bit floats.

Usage: sudo python meterscan.py [dev] [baud] [slave] [start] [count]
Default: /dev/ttyS1 9600 11 0 40
"""
import sys, time, struct, serial

dev   = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS1'
baud  = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
slave = int(sys.argv[3]) if len(sys.argv) > 3 else 11
start = int(sys.argv[4]) if len(sys.argv) > 4 else 0
count = int(sys.argv[5]) if len(sys.argv) > 5 else 40


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def read(p, fn, addr, n):
    f = bytes([slave, fn, addr >> 8, addr & 0xFF, n >> 8, n & 0xFF])
    c = crc16(f)
    p.reset_input_buffer()
    p.write(f + bytes([c & 0xFF, c >> 8]))
    p.flush()
    time.sleep(0.05)
    r = p.read(5 + n * 2)
    if len(r) < 5 or r[0] != slave or (r[1] & 0x80):
        return None
    nb = r[2]
    return [(r[3 + i * 2] << 8) | r[4 + i * 2] for i in range(nb // 2)]


def f32(lo, hi):
    """Two 16-bit words -> float, both word orders."""
    a = struct.unpack('>f', struct.pack('>HH', hi, lo))[0]   # LO_HI (word-swapped)
    b = struct.unpack('>f', struct.pack('>HH', lo, hi))[0]   # HI_LO (big-endian)
    return a, b


with serial.Serial(dev, baud, bytesize=8, parity=serial.PARITY_NONE,
                   stopbits=1, timeout=0.4) as p:
    regs = {}
    print(f'input registers (fn 0x04), slave {slave} @ {baud} 8N1\n')
    # Read in blocks of 8 to cut round trips on a half-duplex bus.
    a = start
    while a < start + count:
        n = min(8, start + count - a)
        vals = read(p, 4, a, n)
        if vals:
            for i, v in enumerate(vals):
                regs[a + i] = v
        a += n

    if not regs:
        print('  no response - check baud, slave id, or wiring')
        sys.exit(2)

    print(f'{"reg":>5} {"3x":>7} {"raw":>7} {"hex":>7}   {"float LO_HI":>14} {"float HI_LO":>14}')
    for r in sorted(regs):
        line = f'{r:>5} {30001 + r:>7} {regs[r]:>7} 0x{regs[r]:04x}'
        if r % 2 == 0 and (r + 1) in regs:
            lo_hi, hi_lo = f32(regs[r], regs[r + 1])
            plaus = ''
            for v in (lo_hi, hi_lo):
                if 0.01 < abs(v) < 1e6:
                    plaus = '  <-- plausible'
                    break
            line += f'   {lo_hi:>14.4f} {hi_lo:>14.4f}{plaus}'
        print(line)