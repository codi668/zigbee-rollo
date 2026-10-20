#!/usr/bin/env python3
"""Rollo-Debug: Weboberfläche für die USB-Konsole (rollo>) der Zigbee-Rollo-Firmware.

Start:  python3 tools/rollo-debug.py          (öffnet http://127.0.0.1:8765)
        python3 tools/rollo-debug.py --port /dev/ttyACM0 --no-browser

Braucht nur Python 3 und pyserial. Der Server lauscht ausschließlich auf 127.0.0.1.
Vor dem Flashen (idf.py flash) in der Oberfläche "Port freigeben" drücken oder das
Programm beenden – der Port kann nur von einem Programm gleichzeitig benutzt werden.
Achtung: Das Öffnen des Ports setzt den ESP zurück (USB-Serial/JTAG).
"""
import argparse
import collections
import glob
import json
import os
import queue
import re
import secrets
import shlex
import subprocess
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
except ImportError:
    sys.exit("pyserial fehlt: pip install pyserial")

ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
LOG_LINE = re.compile(r"^([EWIDV]) \((\d+)\) ([\w.\-]+): (.*)$")
MAX_CMD = 80

# ---------------------------------------------------------------- Zustand
state = {
    "connected": False, "port": "", "released": False,
    "st": "?", "error": "", "tilt": None, "tilt_pos": None, "tilt_bottom": None,
    "batt_mv": None, "batt_pct": None, "ntc": None, "net": "?", "charging": None,
    "fw": "", "brownouts": 0, "trace_ok": True, "margin": None,
}
state_lock = threading.Lock()


def parse(line):
    """Zieht Zustandsfelder aus einer Logzeile. @return True, wenn sich etwas änderte."""
    m = LOG_LINE.match(line)
    msg = m.group(4) if m else line
    old = json.dumps(state, sort_keys=True)
    with state_lock:
        t = re.match(r"Zustand (\w+)(?: -> (\w+))?", msg)
        if t:
            state["st"] = t.group(2) or t.group(1)
            if state["st"] != "FEHLER":
                state["error"] = ""
        e = re.match(r"Zustand \w+ -> FEHLER: (.*)", msg)
        if e:
            state["error"] = e.group(1)
        e = re.match(r"Zustand FEHLER: (.*)", msg)
        if e:
            state["st"] = "FEHLER"
            state["error"] = e.group(1)
        w = re.match(r"\s*Wenden\s+(\d+) %\s+Pos (-?\d+)\s+unten (-?\d+)", msg)
        if w:
            state["tilt"], state["tilt_pos"], state["tilt_bottom"] = int(w.group(1)), int(w.group(2)), int(w.group(3))
        w = re.match(r"\s*Wenden\s+unkalibriert\s+Pos (-?\d+)", msg)
        if w:
            state["tilt"], state["tilt_pos"], state["tilt_bottom"] = None, int(w.group(1)), None
        w = re.match(r"Wenden: (\d+) % \(Pos (-?\d+) von (-?\d+)\)", msg)
        if w:
            state["tilt"], state["tilt_pos"], state["tilt_bottom"] = int(w.group(1)), int(w.group(2)), int(w.group(3))
        w = re.match(r"Wenden: unkalibriert \(Pos (-?\d+)\)", msg)
        if w:
            state["tilt"], state["tilt_pos"], state["tilt_bottom"] = None, int(w.group(1)), None
        a = re.search(r"Akku\s+(\d+) mV, (\d+) %(.*)", msg)
        if a:
            state["batt_mv"], state["batt_pct"] = int(a.group(1)), int(a.group(2))
            state["charging"] = "laedt" in a.group(3)
            n = re.search(r"NTC ([\d.]+) C", a.group(3))
            if n:
                state["ntc"] = float(n.group(1))
        r = re.match(r"\s*Rand\s+(an|aus)$", msg)
        if r:
            state["margin"] = r.group(1) == "an"
        n = re.match(r"\s*Netz\s+(\w+)", msg)
        if n:
            state["net"] = n.group(1)
        if msg.startswith("Netz: verbunden"):
            state["net"] = "verbunden"
        elif msg.startswith("Netz: offline"):
            state["net"] = "offline"
    return json.dumps(state, sort_keys=True) != old


# ---------------------------------------------------------------- Telemetrie
FIRMWARE_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "firmware"))


class Telemetry:
    """Messspur (Position/Duty), Fahrten, Akkuverlauf. Alles nur im Speicher."""

    def __init__(self):
        self.lock = threading.Lock()
        self.samples = collections.deque(maxlen=40000)   # (Geräte-ms, Position, Duty)
        self.trips = collections.deque(maxlen=300)
        self.trip_samples = {}                            # Fahrt-ID -> Samples
        self.batt = collections.deque(maxlen=6000)        # (Unix-s, mV, %, °C)
        self.cur = None
        self.next_id = 1
        self.last_dev = 0
        self.trace_unsupported = False

    def add_sample(self, dev, pos, duty):
        with self.lock:
            if dev + 1000 < self.last_dev:       # Neustart des ESP: Uhr beginnt von vorn
                self.samples.clear()
            self.last_dev = dev
            self.samples.append((dev, pos, duty))
        hub.publish("tr", [dev, pos, duty])

    def trip_start(self, dev, src, cmd, val):
        self.trip_end(None)
        with self.lock:
            name = cmd if cmd in ("auf", "ab") else "%s %s" % (cmd, val)
            self.cur = {"id": self.next_id, "t": time.strftime("%H:%M:%S"), "dev0": dev, "cmd": name, "src": src,
                        "goal": None, "final": None, "dev": None, "dev_pct": None, "dur": None, "res": None}
            self.next_id += 1

    def trip_motor(self, res, pos, goal, dur):
        with self.lock:
            if not self.cur or res == "Fortschritt":
                return
            self.cur.update(res=res, goal=goal, dur=dur, evpos=pos)

    def trip_final(self, pos):
        with self.lock:
            c = self.cur
            if not c or c["res"] is None:
                return
            c["final"] = pos
        self.trip_end(pos)

    def trip_end(self, pos):
        with self.lock:
            c, self.cur = self.cur, None
        if not c:
            return
        if c["res"] is None:
            c["res"] = "überschrieben"
        if c["final"] is not None and c["goal"] is not None and c["res"] in ("Ziel erreicht", "Endpunkt erreicht"):
            c["dev"] = c["final"] - c["goal"]
            with state_lock:
                bottom = state["tilt_bottom"]
            if bottom:
                c["dev_pct"] = round(100.0 * c["dev"] / bottom, 2)
        with self.lock:
            lo, hi = c["dev0"] - 150, c["dev0"] + (c["dur"] or 0) + 900
            smp = [x for x in self.samples if lo <= x[0] <= hi]
            self.trip_samples[c["id"]] = smp
            self.trips.append(c)
            while len(self.trip_samples) > 300:
                self.trip_samples.pop(next(iter(self.trip_samples)))
        hub.publish("trip", c)

    def add_batt(self, mv, pct):
        with state_lock:
            ntc = state["ntc"]
        row = (time.time(), mv, pct, ntc)
        with self.lock:
            self.batt.append(row)
        hub.publish("batt", row)

    def snapshot(self):
        with self.lock:
            return {"batt": list(self.batt), "trips": list(self.trips), "samples": list(self.samples)[-1500:]}


tele = Telemetry()

# Build-/Flash-Auftrag
job = {"running": False, "kind": "", "rc": None}
job_lines = collections.deque(maxlen=500)
job_lock = threading.Lock()


def run_job(flash):
    """Baut (und flasht) die Firmware im Unterprozess; Ausgabe läuft live in den Firmware-Reiter."""
    with job_lock:
        if job["running"]:
            return "es läuft schon ein Auftrag"
        with state_lock:
            st, port = state["st"], state["port"]
        if flash and st in ("FAHRT", "KALIBRIERUNG"):
            return "Motor fährt – erst stoppen"
        if flash and not port:
            return "kein Board verbunden"
        if not os.path.isdir(FIRMWARE_DIR):
            return "Firmware-Ordner nicht gefunden: " + FIRMWARE_DIR
        job.update(running=True, kind="flash" if flash else "build", rc=None)
        job_lines.clear()

    def emit(line):
        with job_lock:
            job_lines.append(line)
        hub.publish("job", {"line": line})

    def work():
        hub.publish("job", {"state": dict(job)})
        script = ('export IDF_PATH="${IDF_PATH:-$HOME/.platformio/packages/framework-espidf}"; '
                  'export IDF_TOOLS_PATH="${IDF_TOOLS_PATH:-$HOME/.espressif}"; '
                  '. "$IDF_PATH/export.sh" >/dev/null 2>&1 || { echo "ESP-IDF-Umgebung nicht gefunden"; exit 3; }; '
                  'cd %s && idf.py %s build %s 2>&1' % (shlex.quote(FIRMWARE_DIR),
                                                        ("-p " + shlex.quote(port)) if flash else "",
                                                        "flash" if flash else ""))
        rc = -1
        try:
            if flash:
                emit("== Port wird freigegeben ==")
                link.release(True)
                time.sleep(0.8)
            emit("== %s startet ==" % ("Bauen + Flashen" if flash else "Bauen"))
            p = subprocess.Popen(["bash", "-c", script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, errors="replace", bufsize=1)
            for ln in p.stdout:
                emit(ANSI.sub("", ln.rstrip("\n")))
            rc = p.wait()
        except Exception as e:
            emit("Fehler: %s" % e)
        finally:
            if flash:
                link.release(False)
            emit("== fertig, Rückgabecode %d %s ==" % (rc, "(OK)" if rc == 0 else "(FEHLER)"))
            job.update(running=False, rc=rc)
            hub.publish("job", {"state": dict(job)})

    threading.Thread(target=work, daemon=True).start()
    return ""


# ---------------------------------------------------------------- Verteiler
class Hub:
    def __init__(self):
        self.lock = threading.Lock()
        self.clients = []
        self.history = collections.deque(maxlen=800)

    def publish(self, kind, data):
        item = (kind, data)
        with self.lock:
            if kind == "log":
                self.history.append(item)
            for q in self.clients:
                try:
                    q.put_nowait(item)
                except queue.Full:
                    pass

    def subscribe(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            hist = list(self.history)
            self.clients.append(q)
        return q, hist

    def unsubscribe(self, q):
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)


hub = Hub()


def publish_state():
    with state_lock:
        snap = dict(state)
    hub.publish("state", snap)


# ---------------------------------------------------------------- Seriell
class Link(threading.Thread):
    def __init__(self, port_arg):
        super().__init__(daemon=True)
        self.port_arg = port_arg
        self.ser = None
        self.wlock = threading.Lock()
        self.released = False
        self.stop_flag = False
        self.last_trace_sent = 0.0
        self.silent_echo = {}   # Befehl -> Zeitpunkt, bis zu dem sein Konsolen-Echo ausgeblendet wird

    def find_port(self):
        if self.port_arg:
            return self.port_arg if glob.glob(self.port_arg) else None
        found = sorted(glob.glob("/dev/ttyACM*"))
        return found[0] if found else None

    def set_connected(self, ok, port=""):
        with state_lock:
            changed = state["connected"] != ok or state["port"] != port or state["released"] != self.released
            state["connected"], state["port"], state["released"] = ok, port, self.released
            if not ok:
                state["st"] = "?"
        if changed:
            publish_state()

    def log(self, text, level="T"):
        hub.publish("log", {"t": time.strftime("%H:%M:%S"), "lvl": level, "text": text})

    def release(self, on):
        self.released = on
        if on and self.ser:
            with self.wlock:   # kein send()/reset_chip() mitten im Schließen
                try:
                    self.ser.close()
                except Exception:
                    pass
        self.set_connected(False if on else state["connected"], state["port"])

    def send(self, cmd, silent=False):
        cmd = cmd.strip()
        if cmd.startswith("trace"):
            self.last_trace_sent = time.time()
        if not cmd or len(cmd) > MAX_CMD or not re.fullmatch(r"[\w .\-]+", cmd):
            return "ungültiger Befehl"
        with self.wlock:
            if not self.ser or not self.ser.is_open:
                return "nicht verbunden"
            try:
                self.ser.write((cmd + "\r\n").encode())
            except Exception as e:
                return "Schreibfehler: %s" % e
        if silent:
            self.silent_echo[cmd] = time.time() + 2.0
        else:
            self.log("> " + cmd, "T")
        return ""

    def run(self):
        buf = b""
        while not self.stop_flag:
            if self.released:
                time.sleep(0.2)
                continue
            port = self.find_port()
            if not port:
                self.set_connected(False, "")
                time.sleep(0.7)
                continue
            try:
                # Handshake-Leitungen NICHT anfassen: Explizites DTR/RTS = False beim Öffnen
                # brachte den ESP32-H2 ins ROM-Download ("wait usb download") statt in die Firmware.
                s = serial.Serial(port, 115200, timeout=0.2)
            except Exception as e:
                self.set_connected(False, port)
                time.sleep(1.0)
                continue
            self.ser = s
            self.set_connected(True, port)
            self.log("verbunden mit %s (der Chip kann dabei neu gestartet sein)" % port, "T")
            buf = b""
            try:
                while not self.stop_flag and not self.released:
                    chunk = s.read(512)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        self.handle(raw.decode(errors="replace"))
            except Exception as e:
                self.log("Verbindung verloren: %s" % e, "T")
            finally:
                try:
                    s.close()
                except Exception:
                    pass
                self.ser = None
                self.set_connected(False, "")

    def reset_chip(self):
        """Hardware-Reset über RTS (wie esptool --after hard_reset), holt den Chip aus dem Download-Modus."""
        with self.wlock:
            if not self.ser or not self.ser.is_open:
                return "nicht verbunden"
            try:
                self.ser.dtr = False
                self.ser.rts = True
                time.sleep(0.1)
                self.ser.rts = False
            except Exception as e:
                return "Reset fehlgeschlagen: %s" % e
        self.log("Chip zurückgesetzt (RTS)", "T")
        return ""

    def handle(self, raw):
        line = ANSI.sub("", raw).replace("\r", "").strip()
        if not line or line == "rollo>":
            return
        m = LOG_LINE.match(line)
        msg = m.group(4) if m else line
        # Messspur: nicht ins Log, nur in die Telemetrie
        if m and m.group(3) == "tr":
            p = msg.split()
            if len(p) == 4 and p[0] == "2":
                try:
                    tele.add_sample(int(p[1]), int(p[2]), int(p[3]))
                except ValueError:
                    pass
            return
        if msg in ("trace an", "trace aus", "rollo> trace an", "rollo> trace aus"):
            return
        echo = line[len("rollo> "):] if line.startswith("rollo> ") else line
        if self.silent_echo.get(echo, 0) > time.time():   # Echo eines still gesendeten Befehls (z. B. "trace 60")
            return
        if "Unrecognized command" in line and time.time() - self.last_trace_sent < 3:
            with state_lock:
                state["trace_ok"] = False
            publish_state()
            return
        lvl = m.group(1) if m else "T"
        self.log(line, lvl)
        dev = int(m.group(2)) if m else 0
        # Fahrten
        t = re.match(r"Befehl (\w+): (auf|ab|goto|tippen) Wenden (-?\d+)", msg)
        if t:
            tele.trip_start(dev, t.group(1), t.group(2), t.group(3))
        t = re.match(r"Motor 2: (.+?), Pos (-?\d+), Ziel (-?\d+), (\d+) ms", msg)
        if t:
            tele.trip_motor(t.group(1), int(t.group(2)), int(t.group(3)), int(t.group(4)))
        t = re.match(r"Wenden: (?:\d+ % \(Pos (-?\d+) von|unkalibriert \(Pos (-?\d+)\))", msg)
        if t:
            tele.trip_final(int(t.group(1) if t.group(1) is not None else t.group(2)))
        # Akku, Firmware, Brownout
        a = re.search(r"Akku\s+(\d+) mV, (\d+) %", msg)
        if a:
            if parse(line):
                publish_state()
            tele.add_batt(int(a.group(1)), int(a.group(2)))
            return
        f = re.search(r"Zigbee-Rollosteuerung (\S+)", msg)
        if f:
            with state_lock:
                state["fw"] = f.group(1)
            publish_state()
        if "E BOD" in line or "Reset-Grund: Brownout" in msg:
            with state_lock:
                state["brownouts"] += 1
            publish_state()
            hub.publish("alert", {"t": time.strftime("%H:%M:%S"), "text": "Brownout – Versorgung bricht ein (Kabel/Akku/Netzteil prüfen)"})
        if "wait usb download" in line and time.time() - getattr(self, "_last_auto", 0) > 5:
            self._last_auto = time.time()
            self.log("Chip steht im ROM-Download-Modus -> automatischer Reset", "W")
            threading.Thread(target=self.reset_chip, daemon=True).start()
        if parse(line):
            publish_state()


link = None
TOKEN = secrets.token_urlsafe(16)

# ---------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def host_ok(self):
        host = self.headers.get("Host", "")
        origin = self.headers.get("Origin")
        port = self.server.server_address[1]
        ok_hosts = ("127.0.0.1:%d" % port, "localhost:%d" % port)
        if host not in ok_hosts:
            return False
        if origin is not None and origin not in ("http://" + h for h in ok_hosts):
            return False
        return True

    def reply(self, code, body, ctype="text/plain; charset=utf-8"):
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if not self.host_ok():
            return self.reply(403, "verboten")
        if self.path == "/":
            return self.reply(200, HTML.replace("__TOKEN__", TOKEN), "text/html; charset=utf-8")
        if self.path == "/events":
            return self.events()
        if self.path.startswith("/trip?id="):
            try:
                tid = int(self.path.split("=", 1)[1])
            except ValueError:
                return self.reply(400, "ungültig")
            with tele.lock:
                smp = tele.trip_samples.get(tid, [])
            return self.reply(200, json.dumps(smp), "application/json")
        self.reply(404, "nicht gefunden")

    def do_POST(self):
        if not self.host_ok() or self.headers.get("X-Token") != TOKEN:
            return self.reply(403, "verboten")
        n = int(self.headers.get("Content-Length", "0") or 0)
        try:
            body = json.loads(self.rfile.read(min(n, 4096)) or b"{}")
        except ValueError:
            return self.reply(400, "ungültig")
        if self.path == "/cmd":
            err = link.send(str(body.get("cmd", "")), silent=bool(body.get("silent")))
            return self.reply(200 if not err else 409, json.dumps({"error": err}), "application/json")
        if self.path == "/build":
            err = run_job(bool(body.get("flash")))
            return self.reply(200 if not err else 409, json.dumps({"error": err}), "application/json")
        if self.path == "/reset":
            err = link.reset_chip()
            return self.reply(200 if not err else 409, json.dumps({"error": err}), "application/json")
        if self.path == "/release":
            link.release(bool(body.get("on")))
            return self.reply(200, "{}", "application/json")
        self.reply(404, "nicht gefunden")

    def events(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        q, hist = hub.subscribe()
        try:
            with state_lock:
                snap = dict(state)
            self.sse("state", snap)
            for kind, data in hist:
                self.sse(kind, data)
            self.sse("hist", tele.snapshot())
            with job_lock:
                jl = list(job_lines)
            self.sse("job", {"state": dict(job), "lines": jl})
            while True:
                try:
                    kind, data = q.get(timeout=15)
                    self.sse(kind, data)
                except queue.Empty:
                    self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            hub.unsubscribe(q)

    def sse(self, kind, data):
        self.wfile.write(("event: %s\ndata: %s\n\n" % (kind, json.dumps(data))).encode())
        self.wfile.flush()


HTML = r"""<!doctype html>
<html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rollo-Debug</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--tx:#1b1f24;--mut:#68707a;--bd:#d8dce1;--ac:#1f6feb;--ok:#1a7f37;--warn:#9a6700;--err:#cf222e;--logbg:#0f1419;--logtx:#d6dbe0}
@media (prefers-color-scheme:dark){:root{--bg:#12161b;--card:#1b2128;--tx:#e6e9ed;--mut:#8b949e;--bd:#2e353d;--ac:#4c9aff;--ok:#3fb950;--warn:#d29922;--err:#f85149;--logbg:#0a0d11}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--tx);font:14px/1.4 system-ui,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:10px 16px;background:var(--card);border-bottom:1px solid var(--bd);flex-wrap:wrap}
h1{font-size:16px;margin:0}.dot{width:10px;height:10px;border-radius:50%;background:var(--err);display:inline-block;margin-right:6px}
.dot.on{background:var(--ok)}main{display:grid;grid-template-columns:minmax(300px,400px) 1fr;gap:12px;padding:12px 16px;height:calc(100vh - 54px)}
@media(max-width:900px){main{grid-template-columns:1fr;height:auto}}
.col{display:flex;flex-direction:column;gap:12px;min-height:0;overflow:auto}
.card{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:12px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--mut);margin:0 0 8px}
.big{font-size:22px;font-weight:600}.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:4px 0}
button{font:inherit;padding:6px 10px;border:1px solid var(--bd);border-radius:6px;background:var(--card);color:var(--tx);cursor:pointer}
button:hover{border-color:var(--ac)}button:disabled{opacity:.45;cursor:not-allowed}
button.pri{background:var(--ac);color:#fff;border-color:var(--ac)}button.stop{background:var(--err);color:#fff;border-color:var(--err);font-weight:700;padding:10px 16px}
select{width:auto!important}input[type=number],input[type=text],select{font:inherit;padding:6px 8px;border:1px solid var(--bd);border-radius:6px;background:var(--bg);color:var(--tx);width:90px}
input[type=text]{width:auto;flex:1;min-width:120px}input[type=range]{flex:1;min-width:120px}
.bar{height:14px;background:var(--bg);border:1px solid var(--bd);border-radius:7px;overflow:hidden}.bar>i{display:block;height:100%;background:var(--ac);width:0}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 12px}.kv span:nth-child(odd){color:var(--mut)}
.badge{padding:2px 8px;border-radius:10px;font-size:12px;font-weight:600;background:var(--bd)}
.badge.BEREIT{background:var(--ok);color:#fff}.badge.FAHRT,.badge.KALIBRIERUNG{background:var(--ac);color:#fff}.badge.FEHLER{background:var(--err);color:#fff}.badge.UNKALIBRIERT,.badge.AKKU_LEER{background:var(--warn);color:#fff}
.err{color:var(--err);font-weight:600}.hint{color:var(--mut);font-size:12px}
#alerts{display:none;background:var(--err);color:#fff;padding:8px 12px;border-radius:8px}#alerts button{margin-left:8px;padding:2px 8px}
.tabs{display:flex;gap:4px;border-bottom:1px solid var(--bd);margin-bottom:8px;flex-wrap:wrap}
.tabs button{border:none;border-bottom:2px solid transparent;border-radius:0;background:none;padding:6px 12px}
.tabs button.on{border-bottom-color:var(--ac);color:var(--ac);font-weight:600}
#right{flex:1;display:flex;flex-direction:column;min-height:420px}.panel{display:none;flex:1;min-height:0;flex-direction:column}.panel.on{display:flex}
#log,#joblog{flex:1;overflow:auto;background:var(--logbg);color:var(--logtx);border-radius:6px;padding:8px;font:12px/1.45 ui-monospace,monospace;white-space:pre-wrap;min-height:200px}
#log .E{color:#ff7b72}#log .W{color:#e3b341}#log .T{color:#79c0ff}#log .I{color:#c9d1d9}#log .t{color:#6e7681}
canvas{width:100%;flex:1;min-height:200px;background:var(--card);border:1px solid var(--bd);border-radius:6px}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:4px 8px;border-bottom:1px solid var(--bd);text-align:right;white-space:nowrap}
th:first-child,td:first-child,th:nth-child(2),td:nth-child(2),th:last-child,td:last-child{text-align:left}
tbody tr{cursor:pointer}tbody tr:hover{background:var(--bg)}tbody tr.sel{outline:2px solid var(--ac)}
.tw{overflow:auto;flex:1;min-height:0}.bad{color:var(--err);font-weight:600}.good{color:var(--ok)}
</style></head><body>
<header><h1>Rollo-Debug</h1><span><span id="dot" class="dot"></span><span id="conn">getrennt</span></span><span class="hint" id="fw"></span>
<button id="rel" title="Port freigeben, z. B. zum Flashen von Hand">Port freigeben</button><span class="hint" id="relhint"></span></header>
<main>
<div class="col">
 <div id="alerts"><span id="altext"></span><button id="alx">OK</button></div>
 <div class="card"><h2>Zustand</h2>
  <div class="row"><span id="st" class="badge">?</span><span id="err" class="err"></span></div>
  <div style="margin:8px 0"><div class="big" id="pct">–</div><div class="bar"><i id="barfill"></i></div><div class="hint" id="pos"></div></div>
  <div class="kv"><span>Akku</span><span id="batt">–</span><span>Temperatur</span><span id="ntc">–</span><span>Zigbee</span><span id="net">–</span></div>
  <div class="row"><button data-c="status">Status</button><label class="hint"><input type="checkbox" id="auto"> alle 2 s</label>
  <label class="hint" title="Position/Duty alle 20 ms mitschreiben (Firmware 0.2.6+), für Diagramm und Fahrt-Auswertung"><input type="checkbox" id="trace" checked> Messspur</label></div>
  <div class="hint" id="tracehint"></div>
 </div>
 <div class="card"><h2>Fahren (Kippachse)</h2>
  <div class="row"><button class="stop" data-c="stop">STOPP</button><button data-c="ack">Fehler quittieren</button></div>
  <div class="row"><button data-c="up tilt">Auf (0 %)</button><button data-c="down tilt">Zu (100 %)</button></div>
  <div class="row"><input type="range" id="gp" min="0" max="100" value="50"><b id="gpv">50</b>%<button id="go" class="pri">Fahren</button></div>
  <div class="hint">Prozentfahrt nur mit gesetzten Limits.</div>
 </div>
 <div class="card"><h2>Tippen (Schritte, 4200 = 1 Umdrehung)</h2>
  <div class="row" id="jogrow"></div>
  <div class="row"><input type="number" id="jn" value="300" step="50"><button id="jm">− hoch</button><button id="jp">+ runter</button></div>
 </div>
 <div class="card"><h2>Limits</h2>
  <div class="row"><button data-c="limit top tilt">Oben setzen</button><button data-c="limit bottom tilt">Unten setzen</button></div>
  <div class="row"><input type="number" id="ls" value="2100" min="200"><button id="lsb">Weg setzen (oben = hier)</button></div>
  <div class="row"><button data-c="limit clear tilt" data-confirm="Kalibrierung löschen?">Limits löschen</button></div>
  <div class="row"><span>Endabstand (1,5 %):</span><button data-c="margin on" id="mon">an</button><button data-c="margin off" id="moff">aus</button><span class="hint" id="mstat"></span></div>
  <div class="hint">Aus: 0 % / 100 % liegen genau auf den gesetzten Limits. An: 1,5 % Abstand zu beiden Enden.</div>
  <div class="hint">Erst mit „Tippen“ in die obere Lage, „Oben setzen“, dann nach unten tippen (+), „Unten setzen“.</div>
 </div>
 <div class="card"><h2>Weitere</h2>
  <div class="row"><button data-c="cal tilt">Kalibrieren (Anschlag)</button><button data-c="cal save">cal save</button><button data-c="cal abort">cal abort</button></div>
  <div class="row"><button data-c="batt">Akku messen</button><button data-c="restart" data-confirm="ESP neu starten?">Neustart</button><button id="hw" title="Reset über RTS, auch wenn die Firmware nicht antwortet">Chip-Reset</button><button data-c="reset factory" data-confirm="WERKSEINSTELLUNG: Kalibrierung löschen und Zigbee-Netz verlassen?">Werkseinstellung</button></div>
  <div class="row"><input type="text" id="raw" placeholder="Befehl, z. B. help"><button id="send">Senden</button></div>
 </div>
</div>
<div class="col"><div class="card" id="right">
 <div class="tabs" id="tabs"><button data-t="plog" class="on">Log</button><button data-t="ppos">Position</button><button data-t="ptrips">Fahrten</button><button data-t="pbatt">Akku</button><button data-t="pfw">Firmware</button></div>
 <div class="panel on" id="plog">
  <div class="row"><button id="clr">Leeren</button><button id="sav">Speichern</button><label class="hint"><input type="checkbox" id="scroll" checked> Autoscroll</label>
  <label class="hint"><input type="checkbox" id="onlyw"> nur Warnungen/Fehler</label></div>
  <div id="log"></div></div>
 <div class="panel" id="ppos">
  <div class="row"><select id="pwin"><option value="10">letzte 10 s</option><option value="30" selected>letzte 30 s</option><option value="120">letzte 2 min</option><option value="trip">gewählte Fahrt</option></select>
  <span class="hint" id="posinfo"></span></div><canvas id="cpos"></canvas>
  <div class="hint">Linie: Position (Schritte). Hellere Linie: Duty (%). Gestrichelt: Ziel, Limits. Braucht die Messspur (Firmware 0.2.6).</div></div>
 <div class="panel" id="ptrips">
  <div class="row"><button id="tcsv">CSV speichern</button><button id="tclr">Leeren</button><span class="hint" id="tstat"></span></div>
  <div class="tw"><table><thead><tr><th>Zeit</th><th>Befehl</th><th>Ziel</th><th>Erreicht</th><th>Abw. Schritte</th><th>Abw. %</th><th>Dauer ms</th><th>Ergebnis</th></tr></thead><tbody id="tbody"></tbody></table></div>
  <div class="hint">Zeile anklicken: Verlauf der Fahrt im Reiter „Position“.</div></div>
 <div class="panel" id="pbatt">
  <div class="row"><span id="bstat" class="hint"></span></div>
  <canvas id="cbatt"></canvas><canvas id="ctemp"></canvas></div>
 <div class="panel" id="pfw">
  <div class="row"><button id="jbuild">Nur bauen</button><button id="jflash" class="pri">Bauen + Flashen</button><span class="hint" id="jstat"></span></div>
  <div class="hint">Flashen gibt den Port frei, baut <code>firmware/</code> mit ESP-IDF, flasht und öffnet den Port wieder. Nur bei stehendem Motor.</div>
  <div id="joblog"></div></div>
</div></div>
</main>
<script>
const TOKEN="__TOKEN__";const $=id=>document.getElementById(id);
let S={connected:false};
async function cmd(c,silent){
  let r;try{r=await fetch('/cmd',{method:'POST',headers:{'X-Token':TOKEN,'Content-Type':'application/json'},body:JSON.stringify({cmd:c,silent:!!silent})})}catch(e){return}
  if(!r.ok&&!silent){const j=await r.json().catch(()=>({}));add({t:new Date().toLocaleTimeString(),lvl:'E',text:'Befehl "'+c+'" nicht gesendet: '+(j.error||r.status)});}
}
const post=(p,b)=>fetch(p,{method:'POST',headers:{'X-Token':TOKEN,'Content-Type':'application/json'},body:JSON.stringify(b||{})});
document.querySelectorAll('button[data-c]').forEach(b=>b.onclick=()=>{if(b.dataset.confirm&&!confirm(b.dataset.confirm))return;cmd(b.dataset.c);if(b.dataset.c!=='status')setTimeout(()=>cmd('status'),600)});
const jogs=[-1000,-300,-100,-30,30,100,300,1000];
jogs.forEach(v=>{const b=document.createElement('button');b.textContent=(v>0?'+':'')+v;b.onclick=()=>{cmd('jog tilt '+v);setTimeout(()=>cmd('status'),1500)};$('jogrow').appendChild(b)});
function jog(sign){const n=parseInt($('jn').value,10);if(!n||n<=0||n>100000)return alert('Schritte 1…100000');cmd('jog tilt '+(sign*n));setTimeout(()=>cmd('status'),1500)}
$('jm').onclick=()=>jog(-1);$('jp').onclick=()=>jog(1);
$('gp').oninput=()=>$('gpv').textContent=$('gp').value;
$('go').onclick=()=>{cmd('goto tilt '+$('gp').value);setTimeout(()=>cmd('status'),1500)};
$('lsb').onclick=()=>{const n=parseInt($('ls').value,10);if(!n||n<200)return alert('mindestens 200 Schritte');cmd('limit set tilt '+n);setTimeout(()=>cmd('status'),600)};
$('hw').onclick=()=>post('/reset');
$('send').onclick=()=>{const v=$('raw').value.trim();if(v){cmd(v);$('raw').value=''}};
$('raw').onkeydown=e=>{if(e.key==='Enter')$('send').click()};
$('rel').onclick=()=>post('/release',{on:!S.released});
let timer=null;$('auto').onchange=()=>{clearInterval(timer);if($('auto').checked)timer=setInterval(()=>{if(S.connected)cmd('status',true)},2000)};
// ---- Reiter
document.querySelectorAll('#tabs button').forEach(b=>b.onclick=()=>{document.querySelectorAll('#tabs button').forEach(x=>x.classList.toggle('on',x===b));
  document.querySelectorAll('.panel').forEach(p=>p.classList.toggle('on',p.id===b.dataset.t));redraw()});
// ---- Log
const log=$('log');let lines=[];
function add(m){lines.push(m);if(lines.length>3000)lines.shift();draw(m)}
function el(m){const d=document.createElement('div');d.className=m.lvl;
  const t=document.createElement('span');t.className='t';t.textContent=m.t+' ';d.appendChild(t);d.appendChild(document.createTextNode(m.text));return d}
function draw(m){if($('onlyw').checked&&!'EWT'.includes(m.lvl))return;log.appendChild(el(m));while(log.childElementCount>3000)log.firstChild.remove();if($('scroll').checked)log.scrollTop=log.scrollHeight}
$('onlyw').onchange=()=>{log.textContent='';lines.forEach(draw)};
$('clr').onclick=()=>{lines=[];log.textContent=''};
$('sav').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.map(m=>m.t+' '+m.text).join('\n')],{type:'text/plain'}));a.download='rollo-log-'+Date.now()+'.txt';a.click()};
// ---- Zustand
function render(s){S=s;$('dot').className='dot'+(s.connected?' on':'');
  $('conn').textContent=s.released?'Port freigegeben':s.connected?'verbunden ('+s.port+')':'getrennt – warte auf Board';
  $('fw').textContent=s.fw?'Firmware '+s.fw:'';
  $('rel').textContent=s.released?'Port wieder öffnen':'Port freigeben';$('relhint').textContent=s.released?'zum Flashen frei':'';
  document.querySelectorAll('button[data-c],#go,#jm,#jp,#lsb,#send,#hw,#jogrow button').forEach(b=>b.disabled=!s.connected);
  const st=$('st');st.textContent=s.st;st.className='badge '+s.st;$('err').textContent=s.error;
  $('pct').textContent=s.tilt==null?(s.tilt_pos==null?'–':'unkalibriert'):s.tilt+' %';$('barfill').style.width=(s.tilt==null?0:s.tilt)+'%';
  $('pos').textContent=s.tilt_pos==null?'':'Position '+s.tilt_pos+(s.tilt_bottom!=null?' von '+s.tilt_bottom+' (unten)':'');
  $('batt').textContent=s.batt_mv==null?'–':(s.batt_mv/1000).toFixed(2)+' V, '+s.batt_pct+' %'+(s.charging?' (lädt)':'');
  $('ntc').textContent=s.ntc==null?'–':s.ntc.toFixed(1)+' °C';$('net').textContent=s.net;
  $('mstat').textContent=s.margin==null?'':(s.margin?'ist an':'ist aus');$('mon').classList.toggle('pri',s.margin===true);$('moff').classList.toggle('pri',s.margin===false);
  $('tracehint').textContent=s.trace_ok===false?'Messspur nicht unterstützt (Firmware 0.2.6 oder neuer nötig)':'';
  $('trace').disabled=s.trace_ok===false;
  if(s.brownouts)showAlert('Brownouts seit Programmstart: '+s.brownouts)}
function showAlert(t){$('altext').textContent=t;$('alerts').style.display='block'}
$('alx').onclick=()=>$('alerts').style.display='none';
// ---- Messspur
let samples=[],trips=[],batt=[],selTrip=null,tripSamples=null;
let lastTraceSend=0;
function keepTrace(){if($('trace').checked&&S.connected&&S.trace_ok!==false&&Date.now()-lastTraceSend>25000){lastTraceSend=Date.now();cmd('trace 60',true)}}
setInterval(keepTrace,5000);$('trace').onchange=()=>{lastTraceSend=0;if(!$('trace').checked)cmd('trace off',true);else keepTrace()};
// ---- Diagramme
function css(n){return getComputedStyle(document.documentElement).getPropertyValue(n).trim()}
function chart(cv,series,o){
  const dpr=window.devicePixelRatio||1,w=cv.clientWidth,h=cv.clientHeight;if(!w||!h)return;cv.width=w*dpr;cv.height=h*dpr;
  const c=cv.getContext('2d');c.scale(dpr,dpr);c.clearRect(0,0,w,h);
  const L=52,R=o.y2?44:12,T=14,B=26,pw=w-L-R,ph=h-T-B;
  const all=series.filter(s=>!s.axis).flatMap(s=>s.pts);
  if(!all.length){c.fillStyle=css('--mut');c.font='13px system-ui';c.fillText(o.empty||'keine Daten',L+10,T+30);return}
  let x0=o.xmin??Math.min(...all.map(p=>p[0])),x1=o.xmax??Math.max(...all.map(p=>p[0]));if(x1<=x0)x1=x0+1;
  const ys=all.map(p=>p[1]).concat((o.hl||[]).filter(l=>l.range).map(l=>l.y));let y0=o.ymin??Math.min(...ys),y1=o.ymax??Math.max(...ys);
  if(o.minspan&&y1-y0<o.minspan){const m=(y0+y1)/2;y0=m-o.minspan/2;y1=m+o.minspan/2}
  if(y1<=y0){y1=y0+1}const pad=(y1-y0)*.08;if(o.ymin==null)y0-=pad;if(o.ymax==null)y1+=pad;
  const X=x=>L+(x-x0)/(x1-x0)*pw,Y=y=>T+ph-(y-y0)/(y1-y0)*ph,Y2=y=>T+ph-(y-(o.y2min??0))/((o.y2max??100)-(o.y2min??0))*ph;
  c.strokeStyle=css('--bd');c.fillStyle=css('--mut');c.font='11px system-ui';c.lineWidth=1;
  for(let i=0;i<=5;i++){const y=T+ph*i/5;c.beginPath();c.moveTo(L,y);c.lineTo(L+pw,y);c.stroke();
    c.textAlign='right';c.fillText((y1-(y1-y0)*i/5).toFixed(o.yd??0),L-6,y+4);
    if(o.y2){c.textAlign='left';c.fillText(((o.y2max??100)-((o.y2max??100)-(o.y2min??0))*i/5).toFixed(0),L+pw+6,y+4)}}
  c.textAlign='center';for(let i=0;i<=6;i++){const x=L+pw*i/6;c.fillText(o.xfmt?o.xfmt(x0+(x1-x0)*i/6):(x0+(x1-x0)*i/6).toFixed(1),x,h-8)}
  (o.hl||[]).forEach(l=>{if(l.y<y0||l.y>y1)return;c.strokeStyle=l.color;c.setLineDash([5,4]);c.beginPath();c.moveTo(L,Y(l.y));c.lineTo(L+pw,Y(l.y));c.stroke();c.setLineDash([]);
    c.fillStyle=l.color;c.textAlign='left';c.fillText(l.label,L+4,Y(l.y)-3)});
  c.save();c.beginPath();c.rect(L,T,pw,ph);c.clip();
  series.forEach(s=>{if(!s.pts.length)return;c.strokeStyle=s.color;c.lineWidth=s.w||1.8;c.globalAlpha=s.alpha||1;c.beginPath();
    s.pts.forEach((p,i)=>{const px=X(p[0]),py=s.axis?Y2(p[1]):Y(p[1]);i?c.lineTo(px,py):c.moveTo(px,py)});c.stroke();c.globalAlpha=1;
    if(s.dots)s.pts.forEach(p=>{c.fillStyle=s.color;c.beginPath();c.arc(X(p[0]),Y(p[1]),2.5,0,7);c.fill()})});
  c.restore();
  if(o.title){c.fillStyle=css('--tx');c.textAlign='left';c.font='12px system-ui';c.fillText(o.title,L+4,T+12)}
}
function drawPos(){
  const mode=$('pwin').value;let pts,xmin,xmax,goal=null,info='';
  if(mode==='trip'){
    if(!tripSamples||!tripSamples.length){chart($('cpos'),[],{empty:'Fahrt in „Fahrten“ anklicken (oder Messspur war aus)'});return}
    pts=tripSamples;const t0=pts[0][0];const t=trips.find(x=>x.id===selTrip);goal=t?t.goal:null;
    info=t?('Fahrt '+t.t+' '+t.cmd+(t.dev!=null?', Abweichung '+t.dev+' Schritte':'')):'';
    pts=pts.map(p=>[(p[0]-t0)/1000,p[1],p[2]]);xmin=0;xmax=pts[pts.length-1][0]
  }else{
    if(!samples.length){chart($('cpos'),[],{empty:S.trace_ok===false?'Messspur braucht Firmware 0.2.6':'noch keine Messspur – eine Fahrt auslösen'});return}
    const last=samples[samples.length-1][0],win=parseInt(mode,10)*1000;pts=samples.filter(p=>p[0]>=last-win).map(p=>[(p[0]-last)/1000,p[1],p[2]]);
    xmin=-parseInt(mode,10);xmax=0;const t=trips[trips.length-1];goal=t&&t.goal!=null?t.goal:null;info=t?('letztes Ziel '+t.cmd):''}
  $('posinfo').textContent=info;
  const hl=[];if(goal!=null)hl.push({y:goal,color:css('--warn'),label:'Ziel '+goal,range:true});
  hl.push({y:0,color:css('--mut'),label:'oben 0'});if(S.tilt_bottom)hl.push({y:S.tilt_bottom,color:css('--mut'),label:'unten '+S.tilt_bottom});
  chart($('cpos'),[{pts:pts.map(p=>[p[0],p[2]]),color:css('--ac'),axis:2,alpha:.35,w:1.2},{pts:pts.map(p=>[p[0],p[1]]),color:css('--ac')}],{xmin,xmax,hl,y2:true,y2min:0,y2max:100,xfmt:x=>x.toFixed(1)+' s'})
}
function drawBatt(){
  const f=x=>{const d=new Date(x*1000),p=n=>String(n).padStart(2,'0');return p(d.getHours())+':'+p(d.getMinutes())+':'+p(d.getSeconds())};
  chart($('cbatt'),[{pts:batt.map(b=>[b[0],b[1]/1000]),color:css('--ok'),dots:batt.length<60}],{xfmt:f,yd:3,minspan:0.1,title:'Akkuspannung (V)',empty:'noch keine Akkumessung'});
  chart($('ctemp'),[{pts:batt.filter(b=>b[3]!=null).map(b=>[b[0],b[3]]),color:css('--warn'),dots:batt.length<60}],{xfmt:f,yd:1,minspan:2,title:'Temperatur (°C)',empty:'noch keine Temperaturmessung'});
  if(batt.length){const v=batt.map(b=>b[1]);const l=batt[batt.length-1];$('bstat').textContent='aktuell '+(l[1]/1000).toFixed(2)+' V, '+l[2]+' %   ·   min '+(Math.min(...v)/1000).toFixed(2)+' V, max '+(Math.max(...v)/1000).toFixed(2)+' V   ·   '+batt.length+' Messungen'}
}
function drawTrips(){
  const tb=$('tbody');tb.textContent='';
  [...trips].reverse().forEach(t=>{const tr=document.createElement('tr');if(t.id===selTrip)tr.className='sel';
    const ok=t.res==='Ziel erreicht'||t.res==='Endpunkt erreicht';
    const cells=[t.t,t.cmd+' ('+t.src+')',t.goal??'–',t.final??'–',t.dev==null?'–':(t.dev>0?'+':'')+t.dev,t.dev_pct==null?'–':(t.dev_pct>0?'+':'')+t.dev_pct.toFixed(2),t.dur??'–',t.res];
    cells.forEach((v,i)=>{const td=document.createElement('td');td.textContent=v;if(i===7)td.className=ok?'good':'bad';if(i===4&&t.dev!=null&&Math.abs(t.dev)>80)td.className='bad';tr.appendChild(td)});
    tr.onclick=async()=>{selTrip=t.id;tripSamples=await (await fetch('/trip?id='+t.id)).json();$('pwin').value='trip';
      document.querySelector('#tabs button[data-t=ppos]').click();drawTrips()};tb.appendChild(tr)});
  const d=trips.filter(t=>t.dev!=null).map(t=>Math.abs(t.dev));
  $('tstat').textContent=trips.length+' Fahrten'+(d.length?' · mittlere |Abw.| '+Math.round(d.reduce((a,b)=>a+b,0)/d.length)+' Schritte, max '+Math.max(...d):'');
}
$('tclr').onclick=()=>{trips=[];drawTrips()};
$('tcsv').onclick=()=>{const rows=[['Zeit','Befehl','Quelle','Ziel','Erreicht','Abw_Schritte','Abw_Prozent','Dauer_ms','Ergebnis']].concat(trips.map(t=>[t.t,t.cmd,t.src,t.goal,t.final,t.dev,t.dev_pct,t.dur,t.res]));
  const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([rows.map(r=>r.join(';')).join('\n')],{type:'text/csv'}));a.download='rollo-fahrten-'+Date.now()+'.csv';a.click()};
$('pwin').onchange=drawPos;
let rq=0;function redraw(){if(rq)return;rq=requestAnimationFrame(()=>{rq=0;const on=document.querySelector('.panel.on').id;
  if(on==='ppos')drawPos();if(on==='pbatt')drawBatt();if(on==='ptrips')drawTrips()})}
window.onresize=redraw;
// ---- Firmware-Reiter
const jl=$('joblog');
function jstate(j){$('jstat').textContent=j.running?(j.kind==='flash'?'Bauen + Flashen läuft …':'Bauen läuft …'):(j.rc==null?'':(j.rc===0?'letzter Lauf: OK':'letzter Lauf: FEHLER ('+j.rc+')'));
  $('jbuild').disabled=$('jflash').disabled=j.running}
$('jbuild').onclick=async()=>{jl.textContent='';const r=await post('/build',{flash:false});if(!r.ok)alert((await r.json()).error)};
$('jflash').onclick=async()=>{if(!confirm('Firmware bauen und auf das Board flashen? Der Port wird kurz freigegeben, das Board startet neu.'))return;jl.textContent='';
  const r=await post('/build',{flash:true});if(!r.ok)alert((await r.json()).error)};
function jadd(l){const d=document.createElement('div');d.textContent=l;jl.appendChild(d);while(jl.childElementCount>600)jl.firstChild.remove();jl.scrollTop=jl.scrollHeight}
// ---- Ereignisse
const es=new EventSource('/events');
es.addEventListener('state',e=>{const prev=S.connected;render(JSON.parse(e.data));if(!prev&&S.connected){lastTraceSend=0;setTimeout(()=>{cmd('version',true);cmd('status',true)},1500);setTimeout(keepTrace,2500)}});
es.addEventListener('log',e=>add(JSON.parse(e.data)));
es.addEventListener('hist',e=>{const h=JSON.parse(e.data);samples=h.samples;trips=h.trips;batt=h.batt;redraw()});
es.addEventListener('tr',e=>{samples.push(JSON.parse(e.data));if(samples.length>40000)samples.shift();if(document.getElementById('ppos').classList.contains('on'))redraw()});
es.addEventListener('trip',e=>{trips.push(JSON.parse(e.data));redraw()});
es.addEventListener('batt',e=>{batt.push(JSON.parse(e.data));redraw()});
es.addEventListener('alert',e=>{const a=JSON.parse(e.data);showAlert(a.t+' '+a.text)});
es.addEventListener('job',e=>{const j=JSON.parse(e.data);if(j.lines){jl.textContent='';j.lines.forEach(jadd)}if(j.line!==undefined)jadd(j.line);if(j.state)jstate(j.state)});
es.onerror=()=>{$('conn').textContent='Programm nicht erreichbar';$('dot').className='dot'};
</script></body></html>"""


def main():
    global link
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serielle Schnittstelle (Standard: erste /dev/ttyACM*)")
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    link = Link(a.port)
    link.start()
    srv = ThreadingHTTPServer(("127.0.0.1", a.http_port), Handler)
    srv.daemon_threads = True
    url = "http://127.0.0.1:%d/" % a.http_port
    print("Rollo-Debug läuft auf", url, "(Beenden mit Strg+C)")
    if not a.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop_flag = True


if __name__ == "__main__":
    main()
