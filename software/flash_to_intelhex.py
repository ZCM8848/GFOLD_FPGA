#!/usr/bin/env python3
"""Convert flash_image.hex (one byte per line, little-endian) into standard
Intel HEX (:LLAAAATT...CC) for Quartus Convert Programming Files / CFI Flash
Loader.

flash_image.hex  layout = 303,096 bytes = 37887 x 64-bit words (little-endian),
byte address 0 = first byte of the COO segment (see flash_manifest.json).
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "rtl", "data", "flash", "flash_image.hex")
DST = os.path.join(HERE, "..", "rtl", "data", "flash", "flash_image_intel.hex")


def main():
    with open(SRC) as f:
        data = bytes(int(x, 16) for x in f.read().split())

    lines = []
    for i in range(0, len(data), 16):
        if i % 0x10000 == 0:
            # Extended Linear Address record (type 04): upper 16 bits of address
            base = (i >> 16) & 0xFFFF
            rec = bytes([0x02, 0x00, 0x00, 0x04, (base >> 8) & 0xFF, base & 0xFF])
            lines.append(":" + rec.hex().upper() + ("%02X" % ((-sum(rec)) & 0xFF)))
        chunk = data[i:i + 16]
        n = len(chunk)
        addr = i & 0xFFFF
        record = bytes([n, (addr >> 8) & 0xFF, addr & 0xFF, 0x00]) + chunk
        cksum = (-sum(record)) & 0xFF
        lines.append(":" + record.hex().upper() + ("%02X" % cksum))

    lines.append(":00000001FF")  # EOF record

    with open(DST, "wb") as f:
        f.write(("\n".join(lines) + "\n").encode("ascii"))

    print("wrote %s" % DST)
    print("  %d bytes, %d data records + 1 EOF" % (len(data), len(lines) - 1))


if __name__ == "__main__":
    main()
