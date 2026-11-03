#!/usr/bin/env python3
"""gen_dump.py — 从 golden hex 文件生成 check_drs.py 格式的 dump 文件"""
import os, sys

kkt = os.path.join(os.path.dirname(__file__), "..", "rtl", "data", "kkt", "full")

def hex_line(fname):
    with open(os.path.join(kkt, fname)) as f:
        return " ".join(l.strip() for l in f if l.strip())

out = os.path.join(os.path.dirname(__file__), "..", "rtl", "data", "drs_dump.txt")
with open(out, "w") as f:
    for tag, fn in [("V1","v1.hex"), ("V2","v2.hex"),
                    ("UT1","ut1.hex"), ("UT2","ut2.hex"),
                    ("U1","u1.hex"), ("U2","u2.hex"),
                    ("RSK1","rsk1.hex"), ("RSK2","rsk2.hex")]:
        f.write(tag + ":" + hex_line(fn) + "\n")
print("Created " + out)
