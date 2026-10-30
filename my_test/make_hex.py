#!/usr/bin/env python3
"""
make_hex.py -- convert an ELF into a plain $readmemh word list.

The VeeR EL2 fetches 64-bit beats, so the instruction memory of the AXI SoC
has to be loaded as 64-bit words (two 32-bit instructions per line).

    objcopy -O verilog --verilog-data-width=8 <elf> out.hex

produces exactly that, but it also emits "@<word-address>" records.
$readmemh would then place the words at those absolute addresses, which are
outside the memory array of axi_mem (the array is indexed from 0).  This
script rewrites the addresses relative to the base address and drops the
"@" records, so the file can be loaded with a plain $readmemh.

The ICCM of the core is 32 bits wide, so the same conversion is done with
--verilog-data-width=4 for that memory.

Usage:
    make_hex.py <elf> <out.hex> [base_addr] [--width=4|8]
"""
import os
import re
import subprocess
import sys

OBJCOPY = os.environ.get("OBJCOPY", "riscv64-unknown-elf-objcopy")
WORD_BYTES = 8          # 64-bit words
ADDR_RE = re.compile(r"@([0-9a-fA-F]+)")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    width_opt = [a for a in sys.argv[1:] if a.startswith("--width=")]

    if len(args) < 2:
        print(__doc__)
        return 1

    elf, out = args[0], args[1]
    base = int(args[2], 0) if len(args) > 2 else 0xEE00_0000
    word_bytes = 8
    if width_opt:
        word_bytes = int(width_opt[0].split("=")[1])

    tmp = out + ".tmp"
    subprocess.run([OBJCOPY, "-O", "verilog",
                    "--verilog-data-width=%d" % word_bytes, elf, tmp],
                   check=True)

    words = []
    addr = 0
    with open(tmp) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("@"):
                addr = int(line[1:], 16)
                continue
            for tok in line.split():
                words.append((addr, int(tok, 16)))
                addr += 1
    os.remove(tmp)

    if not words:
        print("ERROR: no data in %s" % elf)
        return 1

    base_idx = base // word_bytes
    table = {}
    for a, v in words:
        idx = a - base_idx
        if idx < 0:
            continue
        table[idx] = v

    max_idx = max(table)
    with open(out, "w") as f:
        for i in range(max_idx + 1):
            if word_bytes == 8:
                f.write("%016X\n" % table.get(i, 0))
            else:
                f.write("%08X\n" % table.get(i, 0))

    print("wrote %s : %d words (%d bytes) relative to 0x%08X"
          % (out, max_idx + 1, (max_idx + 1) * WORD_BYTES, base))
    return 0


if __name__ == "__main__":
    sys.exit(main())
