#!/usr/bin/env python3
"""gfold_eth.py — GFOLD 以太网控制台"""
import socket, struct, sys, time, zlib, os

MAGIC   = 0x4746
FPGA_IP = '192.168.1.200'
PORT    = 4661

def make_header(pkt_type, seq=0, nwords=0):
    return struct.pack('>HBBHH', MAGIC, pkt_type, 0, seq, nwords)

def parse_header(data):
    if len(data) < 8:
        return None
    return struct.unpack('>HBBHH', data[:8])

def wait_ack(sock, timeout=2.0):
    """等一个 ACK，返回 header 或 None"""
    sock.settimeout(timeout)
    try:
        data, addr = sock.recvfrom(2048)
        return parse_header(data)
    except socket.timeout:
        return None

def cmd_ping(count=5):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))
    ok = 0
    for i in range(count):
        seq = i + 1
        sock.sendto(make_header(0x0A, seq=seq), (FPGA_IP, PORT))
        hdr = wait_ack(sock)
        if hdr and hdr[0] == MAGIC and hdr[1] == 0x02:
            print(f"  ACK seq={hdr[3]}")
            ok += 1
        else:
            print(f"  TIMEOUT seq={seq}")
        time.sleep(0.3)
    sock.close()
    print(f"  {ok}/{count} replied")
    return ok == count

def cmd_load(image_path=None):
    # 生成或读取镜像
    if image_path and os.path.exists(image_path):
        with open(image_path, 'rb') as f:
            image = f.read()
    else:
        image_size = 303096  # 37887 words × 8 bytes
        image = os.urandom(image_size)
        print(f"(using random {image_size}-byte test image)")

    expected_crc = zlib.crc32(image) & 0xFFFFFFFF
    total_words = len(image) // 8
    print(f"Image: {len(image)} bytes, {total_words} words")
    print(f"Expected CRC: 0x{expected_crc:08X}")

    # 切包（180 字/包，尾包 87 字）
    wpk = 180
    nfull = total_words // wpk
    last = total_words - nfull * wpk
    npkts = nfull + (1 if last > 0 else 0)
    print(f"Packets: {npkts} ({nfull}×180 + {last})")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))

    # RESET
    print("RESET...")
    sock.sendto(make_header(0x09), (FPGA_IP, PORT))
    wait_ack(sock, 1.0)
    time.sleep(0.1)

    # LOAD（停等）
    print("LOAD...")
    for seq in range(npkts):
        if seq < nfull:
            nw = wpk
        else:
            nw = last
        off = seq * wpk * 8
        payload = image[off:off + nw * 8]
        pkt = make_header(0x01, seq=seq, nwords=nw) + payload

        for attempt in range(3):
            sock.sendto(pkt, (FPGA_IP, PORT))
            hdr = wait_ack(sock, 1.0)
            if hdr and hdr[0] == MAGIC and hdr[1] == 0x02:
                break
        else:
            print(f"  seq={seq} FAILED after 3 attempts")
            sock.close()
            return False

        if (seq + 1) % 50 == 0 or seq == npkts - 1:
            print(f"  {seq+1}/{npkts}")

    # FINISH
    fin_payload = struct.pack('<II', total_words, expected_crc)
    fin_pkt = make_header(0x03, seq=0, nwords=2) + fin_payload
    print("FINISH...")
    sock.sendto(fin_pkt, (FPGA_IP, PORT))

    sock.settimeout(5.0)
    try:
        data, addr = sock.recvfrom(2048)
        hdr = parse_header(data)
        if hdr and hdr[0] == MAGIC and hdr[1] == 0x04:
            payload = data[8:]
            if len(payload) >= 8:
                actual_crc, = struct.unpack('<I', payload[0:4])
                status = payload[4]
                missing = payload[5]
                words_lo, = struct.unpack('<H', payload[6:8])
                print(f"  FINACK: CRC=0x{actual_crc:08X} status={status} words_lo={words_lo}")
                if status == 0 and actual_crc == expected_crc:
                    print("  PASS: CRC match!")
                    sock.close()
                    return True
                else:
                    print(f"  FAIL: status={status} CRC_match={actual_crc == expected_crc}")
        else:
            print(f"  Unexpected response type={hdr[1]:#x}" if hdr else "  Bad header")
    except socket.timeout:
        print("  FINACK timeout!")

    sock.close()
    return False

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'ping'
    if cmd == 'ping':
        count = int(sys.argv[2]) if len(sys.argv) > 2 else 5
        sys.exit(0 if cmd_ping(count) else 1)
    elif cmd == 'load':
        path = sys.argv[2] if len(sys.argv) > 2 else None
        sys.exit(0 if cmd_load(path) else 1)
    else:
        print("commands: ping [count] | load [image_path]")
        sys.exit(1)

if __name__ == '__main__':
    main()
