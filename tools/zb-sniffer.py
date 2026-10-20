#!/usr/bin/env python3
"""zb-sniffer: liest die Sniffer-Firmware (tools/sniffer, zweiter ESP32-H2) und schreibt eine pcap-Datei für Wireshark.

  python3 tools/zb-sniffer.py --port /dev/ttyACM1 --out mitschnitt.pcap [--seconds 120]
  python3 tools/zb-sniffer.py --port /dev/ttyACM1 --fifo /tmp/zb.pcap     (live: wireshark -k -i /tmp/zb.pcap)

pcap: LINKTYPE_IEEE802_15_4_NOFCS (230), Zeit = Unixzeit beim Empfang auf dem Laptop (µs-genau aus der
Sniffer-Uhr, auf den Laptop-Start bezogen). Entschlüsselung: Netzwerkschlüssel in Wireshark unter
Bearbeiten -> Einstellungen -> Protocols -> ZigBee eintragen (nur lokal, nicht weitergeben).
"""
import argparse
import os
import struct
import sys
import time

import serial


def pcap_header():
    return struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 262, 230)


def pcap_record(ts, psdu):
    sec = int(ts)
    usec = int((ts - sec) * 1_000_000)
    return struct.pack("<IIII", sec, usec, len(psdu), len(psdu)) + psdu


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True, help="serielle Schnittstelle des Sniffer-Boards")
    ap.add_argument("--out", help="pcap-Datei")
    ap.add_argument("--fifo", help="benannte Pipe für Live-Wireshark (wird angelegt)")
    ap.add_argument("--seconds", type=float, default=0, help="nach so vielen Sekunden beenden (0 = bis Strg+C)")
    a = ap.parse_args()
    if not a.out and not a.fifo:
        sys.exit("--out oder --fifo angeben")
    if a.fifo:
        if os.path.exists(a.fifo):
            os.remove(a.fifo)
        os.mkfifo(a.fifo)
        print("Pipe", a.fifo, "- Wireshark starten mit: wireshark -k -i", a.fifo, flush=True)
        out = open(a.fifo, "wb", buffering=0)   # blockiert, bis Wireshark liest
    else:
        out = open(a.out, "wb", buffering=0)
    out.write(pcap_header())
    ser = serial.Serial(a.port, 115200, timeout=0.5)
    t_start = time.time()
    t_dev0 = None
    n = 0
    buf = b""
    print("lese", a.port, "...", flush=True)
    try:
        while not a.seconds or time.time() - t_start < a.seconds:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if line.startswith("FR "):
                    p = line.split()
                    if len(p) != 5:
                        continue
                    try:
                        t_us, psdu = int(p[1]), bytes.fromhex(p[4])
                    except ValueError:
                        continue
                    if t_dev0 is None:
                        t_dev0 = t_us - (time.time() - t_start) * 1e6
                    ts = t_start + (t_us - t_dev0) / 1e6
                    out.write(pcap_record(ts, psdu))
                    n += 1
                    if n % 50 == 0:
                        print(n, "Frames", flush=True)
                elif line.startswith(("SNIFFER", "DROP")):
                    print(line, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        ser.close()
        print("fertig:", n, "Frames", "->", a.out or a.fifo, flush=True)


if __name__ == "__main__":
    main()
