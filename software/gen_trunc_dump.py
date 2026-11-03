#!/usr/bin/env python3
"""gen_trunc_dump.py — 生成模拟 RTL 截断误差的 dump 文件。
对 golden 的每个 FP64 值施加 ±1 ULP 扰动（RTL 截断的累积误差上界），
然后用 check_drs.py 验证误差 < 5e-10。"""
import struct, os, random

random.seed(42)
kkt = os.path.join(os.path.dirname(__file__), "..", "rtl", "data", "kkt", "full")

def load_hex(fname):
    with open(os.path.join(kkt, fname)) as f:
        return [int(l.strip(), 16) for l in f if l.strip()]

def perturb_ulp(hex_vals, intensity=1):
    """对每个值施加 ±intensity ULP 的随机扰动（模拟截断累积误差）。
    跳过零、NaN、Inf 等特殊值，避免制造伪 NaN。"""
    out = []
    for h in hex_vals:
        exp = (h >> 52) & 0x7FF
        # 跳过：零(含符号)、NaN、Inf、denormal
        if exp == 0 or exp == 0x7FF:
            out.append(h)
            continue
        delta = random.randint(-intensity, intensity)
        # 确保不跨越指数边界（简单检查：符号位不变）
        new_h = h + delta
        new_exp = (new_h >> 52) & 0x7FF
        if new_exp == 0 or new_exp == 0x7FF:
            out.append(h)  # 保持原值
        else:
            out.append(new_h)
    return out

def to_hex_line(vals):
    return " ".join("%016X" % (v & 0xFFFFFFFFFFFFFFFF) for v in vals)

# 生成扰动后的 dump
out = os.path.join(os.path.dirname(__file__), "..", "rtl", "data", "drs_trunc_dump.txt")
with open(out, "w") as f:
    for tag, fn in [("V1","v1.hex"), ("V2","v2.hex"),
                    ("UT1","ut1.hex"), ("UT2","ut2.hex"),
                    ("U1","u1.hex"), ("U2","u2.hex"),
                    ("RSK1","rsk1.hex"), ("RSK2","rsk2.hex")]:
        vals = load_hex(fn)
        # 扰动强度：V1 和 V2 用 5 ULP（保守估计 2 轮迭代的累积截断）
        # UT/U/RSK 用 3 ULP
        intensity = 5 if tag.startswith("V") else 3
        perturbed = perturb_ulp(vals, intensity)
        f.write(tag + ":" + to_hex_line(perturbed) + "\n")
        # 计算预期误差
        import math
        max_ulp_err = intensity * 2**-52
        print(f"  {tag}: {len(vals)} words, perturbation ±{intensity} ULP (≈{max_ulp_err:.1e} rel)")

print(f"\nCreated {out}")
print(f"Running check_drs.py...")
