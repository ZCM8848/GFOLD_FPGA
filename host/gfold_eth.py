#!/usr/bin/env python3
"""gfold_eth.py �?GFOLD 以太网控制台"""
import socket, struct, sys, time, zlib, os

MAGIC   = 0x4746
FPGA_IP = '192.168.1.200'
PORT    = 4661

def make_header(pkt_type, seq=0, nwords=0):
    return struct.pack('>HBBHH', MAGIC, pkt_type, 0, seq, nwords)

def parse_header(data):
    if len(data) < 8: return None
    return struct.unpack('>HBBHH', data[:8])

def cmd_ping(count=5):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    ok = 0
    for i in range(count):
        seq = i + 1
        sock.sendto(make_header(0x0A, seq=seq), (FPGA_IP, PORT))
        sock.settimeout(2.0)
        try:
            data, _ = sock.recvfrom(2048)
            hdr = parse_header(data)
            if hdr and hdr[1] == 0x02:
                payload = data[8:]
                if len(payload) >= 8:
                    rx_f, link = struct.unpack('<II', payload[:8])
                    print(f"  ACK seq={hdr[3]} rx_frames={rx_f} link=0x{link:08X}")
                ok += 1
        except socket.timeout:
            print(f"  TIMEOUT seq={seq}")
        time.sleep(0.3)
    sock.close()
    print(f"  {ok}/{count} replied")
    return ok == count

def cmd_load(image_path=None):
    if image_path and os.path.exists(image_path):
        with open(image_path, 'rb') as f: image = f.read()
    else:
        image = os.urandom(303096)
        print("(random 303096-byte test image)")
    expected_crc = zlib.crc32(image) & 0xFFFFFFFF
    total_words = len(image) // 8
    print(f"Image: {len(image)}B {total_words}w CRC=0x{expected_crc:08X}")
    wpk = 180; nfull = total_words // wpk
    last = total_words - nfull * wpk
    npkts = nfull + (1 if last > 0 else 0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    print("RESET..."); sock.sendto(make_header(0x09), (FPGA_IP, PORT))
    sock.settimeout(1.0)
    try: sock.recvfrom(2048)
    except: pass
    time.sleep(1.0)
    print(f"LOAD {npkts} pkts...")
    for seq in range(npkts):
        nw = wpk if seq < nfull else last
        off = seq * wpk * 8
        pkt = make_header(0x01, seq=seq, nwords=nw) + image[off:off+nw*8]
        for _ in range(3):
            sock.sendto(pkt, (FPGA_IP, PORT))
            sock.settimeout(1.0)
            try:
                data, _ = sock.recvfrom(2048)
                hdr = parse_header(data)
                if hdr and hdr[1] == 0x02: break
            except: pass
        else:
            print(f"  seq={seq} FAILED"); sock.close(); return False
        # M6: the packet FIFO drains to DDR2 at 25 MHz. Give the writer time
        # before the next 180-word packet arrives.
        time.sleep(0.002)
        if (seq+1) % 50 == 0 or seq == npkts-1: print(f"  {seq+1}/{npkts}")
    print("FINISH...")
    time.sleep(0.01)   # let the final packet drain to DDR2
    fin_pkt = make_header(0x03, nwords=2) + struct.pack('<II', total_words, expected_crc)
    sock.sendto(fin_pkt, (FPGA_IP, PORT))
    sock.settimeout(5.0)
    try:
        data, _ = sock.recvfrom(2048)
        hdr = parse_header(data)
        if hdr and hdr[1] == 0x04:
            p = data[8:]
            if len(p) >= 8:
                acrc, = struct.unpack('<I', p[0:4])
                st = p[4]
                print(f"  FINACK CRC=0x{acrc:08X} status={st}")
                if st == 0 and acrc == expected_crc:
                    print("  PASS"); sock.close(); return True
                print(f"  FAIL status={st}")
    except: print("  FINACK timeout!")
    sock.close(); return False

def cmd_dump(start=0, nwords=10):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    pkt = make_header(0x07, nwords=2) + struct.pack('<II', start, nwords)
    sock.sendto(pkt, (FPGA_IP, PORT))
    sock.settimeout(5.0)
    try:
        data, _ = sock.recvfrom(2048)
        hdr = parse_header(data)
        if hdr and hdr[1] == 0x08:
            p = data[8:]
            words = [struct.unpack('<Q', p[i*8:(i+1)*8])[0] for i in range(len(p)//8)]
            print(f"DUMP {len(words)} words from {start}:")
            for i, w in enumerate(words):
                print(f"  [{start+i}] = 0x{w:016X}")
            sock.close(); return words
        print(f"  Unexpected type=0x{hdr[1]:02X}" if hdr else "  Bad header")
    except: print("  DUMP timeout!")
    sock.close(); return None

def cmd_loadtest():
    # 900 words: word[i] = i, 5 packets of 180
    words = list(range(900))
    image = b''.join(struct.pack('<Q', w) for w in words)
    expected_crc = zlib.crc32(image) & 0xFFFFFFFF
    print(f"Loadtest: 900 words (5 pkts), CRC=0x{expected_crc:08X}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    # RESET
    sock.sendto(make_header(0x09), (FPGA_IP, PORT))
    sock.settimeout(1.0)
    try: sock.recvfrom(2048)
    except: pass
    time.sleep(1.0)
    # LOAD 5 packets
    for seq in range(5):
        pkt = make_header(0x01, seq=seq, nwords=180) + image[seq*1440:(seq+1)*1440]
        sock.sendto(pkt, (FPGA_IP, PORT))
        sock.settimeout(1.0)
        try: sock.recvfrom(2048)
        except: print(f"  seq={seq} timeout")
    # FINISH
    fin_pkt = make_header(0x03, nwords=2) + struct.pack('<II', 900, expected_crc)
    sock.sendto(fin_pkt, (FPGA_IP, PORT))
    sock.settimeout(5.0)
    try:
        data, _ = sock.recvfrom(2048)
        hdr = parse_header(data)
        if hdr and hdr[1] == 0x04:
            p = data[8:]
            acrc, = struct.unpack('<I', p[0:4])
            st = p[4]
            print(f"  FINACK CRC=0x{acrc:08X} status={st}")
            if st != 0 or acrc != expected_crc:
                print("  CRC FAIL"); sock.close(); return False
    except: print("  FINACK timeout!"); sock.close(); return False
    # DUMP 10 words
    print("  Dumping 10 words...")
    dump_pkt = make_header(0x07, nwords=2) + struct.pack('<II', 0, 10)
    sock.sendto(dump_pkt, (FPGA_IP, PORT))
    sock.settimeout(5.0)
    try:
        data, _ = sock.recvfrom(2048)
        hdr = parse_header(data)
        if hdr and hdr[1] == 0x08:
            p = data[8:]
            ok = True
            for i in range(min(10, len(p)//8)):
                w, = struct.unpack('<Q', p[i*8:(i+1)*8])
                exp = i
                match = "OK" if w == exp else "MISMATCH"
                if w != exp: ok = False
                print(f"  [{i}] = 0x{w:016X} (expect 0x{exp:016X}) {match}")
            if ok: print("  DUMP PASS: all words match!")
            else: print("  DUMP FAIL: words mismatch!")
            sock.close(); return ok
    except: print("  DUMP timeout!")
    sock.close(); return False

def cmd_start():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    sock.sendto(make_header(0x05, seq=1), (FPGA_IP, PORT))
    sock.settimeout(2.0)
    try:
        data, _ = sock.recvfrom(2048)
        hdr = parse_header(data)
        print(f"START ACK type=0x{hdr[1]:02X}" if hdr else "START bad ACK")
    except socket.timeout:
        print("START timeout")
    sock.close()

def _dump_words(sock, start, nwords):
    sock.sendto(make_header(0x07, nwords=2) + struct.pack('<II', start, nwords), (FPGA_IP, PORT))
    sock.settimeout(3.0)
    data, _ = sock.recvfrom(2048)
    hdr = parse_header(data)
    if not hdr or hdr[1] != 0x08:
        raise RuntimeError("no DUMP_DATA")
    p = data[8:]
    return [struct.unpack('<Q', p[i*8:(i+1)*8])[0] for i in range(len(p)//8)]

def _d(u):
    return struct.unpack('>d', struct.pack('>Q', u & 0xFFFFFFFFFFFFFFFF))[0]

def _query8(sock, typ, seq=1, timeout=3.0):
    sock.sendto(make_header(typ, seq=seq), (FPGA_IP, PORT))
    sock.settimeout(timeout)
    data, _ = sock.recvfrom(2048)
    hdr = parse_header(data)
    if not hdr or hdr[1] != 0x02 or len(data) < 16:
        raise RuntimeError(f"bad ACK for type 0x{typ:02X}")
    return data[8:16]

def _mass_req(sock):
    p = _query8(sock, 0x0B)
    return struct.unpack('<Q', p)[0]

def _status_req(sock):
    p = _query8(sock, 0x0C)
    w0, w1 = struct.unpack('<II', p)
    return w0 & 0xFFFF, w1 & 0x1

def cmd_check(image_path, wait=0.5):
    if not cmd_load(image_path):
        print("load failed"); return False
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    sock.sendto(make_header(0x05, seq=1), (FPGA_IP, PORT))
    try:
        sock.settimeout(2.0); sock.recvfrom(2048)
    except Exception:
        pass
    print(f"START sent; waiting {wait}s for boot + 2 iterations...")
    time.sleep(wait)
    words = []
    for base in range(0, 120, 10):
        try:
            words += _dump_words(sock, base, 10)
        except Exception as e:
            print(f"  dump @{base} failed: {e}"); sock.close(); return False
    sock.close()
    if len(words) < 120:
        print(f"  short dump: {len(words)} words"); return False
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "data", "kkt", "small2")
    ok = True
    for tag, off, gname in (("VS1", 0, "v1"), ("VS2", 60, "v2")):
        gold = [int(x, 16) for x in open(os.path.join(base, gname + ".hex"))]
        worst = 0.0
        bad = 0
        for i in range(60):
            a = _d(words[off + i]); b = _d(gold[i])
            rel = abs(a - b) / max(abs(b), 1e-12)
            worst = max(worst, rel)
            if rel > 5e-10:
                bad += 1
        print(f"{tag}: worst_rel={worst:.3e} bad={bad}/60 {'PASS' if bad == 0 else 'FAIL'}")
        ok = ok and (bad == 0)
    print("SMALL-PROBLEM CHECK:", "PASS" if ok else "FAIL")
    return ok

def cmd_mass():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    try:
        raw = _mass_req(sock)
        it, _ = _status_req(sock)
    except Exception as e:
        print(f"mass query failed: {e}"); sock.close(); return False
    sock.close()
    val = _d(raw)
    print(f"MASS iter={it} raw=0x{raw:016X} log={val:.12g} exp={pow(2.718281828459045, val):.6f}")
    return True

def cmd_run(image_path, timeout=7200, poll=10):
    if not cmd_load(image_path):
        print("load failed"); return False
    cmd_start()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            it, running = _status_req(sock)
            raw = _mass_req(sock)
        except Exception as e:
            print(f"  status query failed: {e}"); time.sleep(poll); continue
        val = _d(raw)
        print(f"  t={time.time()-t0:7.1f}s iter={it} run={running} "
              f"raw=0x{raw:016X} log={val:.9g}")
        if it >= 49999:
            print(f"RUN COMPLETE iter={it} final_mass_log={val:.12g} "
                  f"final_mass={pow(2.718281828459045, val):.6f}")
            sock.close()
            return True
        time.sleep(poll)
    sock.close()
    print("run timeout")
    return False

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'ping'
    if cmd == 'ping':
        c = int(sys.argv[2]) if len(sys.argv) > 2 else 5
        sys.exit(0 if cmd_ping(c) else 1)
    elif cmd == 'load':
        p = sys.argv[2] if len(sys.argv) > 2 else None
        sys.exit(0 if cmd_load(p) else 1)
    elif cmd == 'dump':
        s = int(sys.argv[2]) if len(sys.argv) > 2 else 0
        n = int(sys.argv[3]) if len(sys.argv) > 3 else 10
        cmd_dump(s, n)
    elif cmd == 'loadtest':
        sys.exit(0 if cmd_loadtest() else 1)
    elif cmd == 'start':
        cmd_start()
    elif cmd == 'mass':
        sys.exit(0 if cmd_mass() else 1)
    elif cmd == 'run50k':
        p = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "data", "flash", "flash_image.bin")
        t = float(sys.argv[3]) if len(sys.argv) > 3 else 7200
        sys.exit(0 if cmd_run(p, t) else 1)
    elif cmd == 'check':
        p = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "data", "kkt", "small2", "flash_image.bin")
        w = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
        sys.exit(0 if cmd_check(p, w) else 1)
    else:
        print("commands: ping [n] | load [path] | dump [start] [n] | loadtest | start | mass | "
              "run50k [image.bin] [timeout_s] | check [image.bin] [wait_s]")
        sys.exit(1)

if __name__ == '__main__':
    main()
