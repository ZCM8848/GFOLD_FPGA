#!/usr/bin/env python3
"""gfold_eth.py — GFOLD 以太网控制台（里程碑 2：ping 子命令）"""
import socket, struct, sys, time

MAGIC   = 0x4746
FPGA_IP = '192.168.1.200'
PORT    = 4661

def make_header(pkt_type, seq=0, nwords=0):
    return struct.pack('>HBBHH', MAGIC, pkt_type, 0, seq, nwords)

def parse_header(data):
    if len(data) < 8:
        return None
    return struct.unpack('>HBBHH', data[:8])

def cmd_ping(count=5):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    sock.bind(('0.0.0.0', PORT))

    ok = 0
    for i in range(count):
        seq = i + 1
        pkt = make_header(0x0A, seq=seq)
        sock.sendto(pkt, (FPGA_IP, PORT))
        try:
            data, addr = sock.recvfrom(2048)
            hdr = parse_header(data)
            if hdr and hdr[0] == MAGIC and hdr[1] == 0x02:
                payload = data[8:]
                if len(payload) >= 8:
                    rx_frames, link = struct.unpack('<II', payload[:8])
                    print(f"  ACK seq={hdr[3]} rx_frames={rx_frames} link=0x{link:08X}")
                else:
                    print(f"  ACK seq={hdr[3]} (payload {len(payload)}B)")
                ok += 1
            else:
                print(f"  UNEXPECTED {data[:16].hex()}")
        except socket.timeout:
            print(f"  TIMEOUT seq={seq}")

        time.sleep(0.3)

    sock.close()
    print(f"  {ok}/{count} replied")
    return ok == count

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'ping'
    if cmd == 'ping':
        count = int(sys.argv[2]) if len(sys.argv) > 2 else 5
        ok = cmd_ping(count)
        sys.exit(0 if ok else 1)
    else:
        print(f"commands: ping [count]")
        sys.exit(1)

if __name__ == '__main__':
    main()
