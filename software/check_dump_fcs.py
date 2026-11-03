#!/usr/bin/env python3
"""check_dump_fcs.py — verify the Ethernet FCS of the captured dump_tx frame."""
import sys, os, zlib

def crc32_eth(data):
    # standard reflected CRC-32 (same polynomial as crc32_eth_nib)
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if (crc & 1) else 0)
    return crc ^ 0xFFFFFFFF

def main():
    p = sys.argv[1] if len(sys.argv) > 1 else "dump_frame.hex"
    data = [int(x.strip(), 16) for x in open(p) if x.strip()]
    print("frame bytes:", len(data))
    print("preamble:", " ".join(f"{x:02X}" for x in data[:8]))
    print("dst_mac :", " ".join(f"{x:02X}" for x in data[8:14]))
    # FCS covers bytes 8 .. len-5 inclusive (Ethernet header + payload)
    body = bytes(data[8:-4])
    fcs = crc32_eth(body)
    got = data[-4] | (data[-3] << 8) | (data[-2] << 16) | (data[-1] << 24)
    print(f"computed FCS = 0x{fcs:08X}")
    print(f"frame   FCS = 0x{got:08X}")
    ok = (fcs == got)
    print("FCS CHECK:", "PASS" if ok else "FAIL")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
