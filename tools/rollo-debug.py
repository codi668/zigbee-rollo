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
import queue
import re
import secrets
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
        n = re.match(r"\s*Netz\s+(\w+)", msg)
        if n:
            state["net"] = n.group(1)
        if msg.startswith("Netz: verbunden"):
            state["net"] = "verbunden"
        elif msg.startswith("Netz: offline"):
            state["net"] = "offline"
    return json.dumps(state, sort_keys=True) != old


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
            try:
                self.ser.close()
            except Exception:
                pass
        self.set_connected(False if on else state["connected"], state["port"])

    def send(self, cmd):
        cmd = cmd.strip()
        if not cmd or len(cmd) > MAX_CMD or not re.fullmatch(r"[\w .\-]+", cmd):
            return "ungültiger Befehl"
        with self.wlock:
            if not self.ser or not self.ser.is_open:
                return "nicht verbunden"
            try:
                self.ser.write((cmd + "\r\n").encode())
            except Exception as e:
                return "Schreibfehler: %s" % e
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
        lvl = m.group(1) if m else "T"
        self.log(line, lvl)
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
            err = link.send(str(body.get("cmd", "")))
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
.dot.on{background:var(--ok)}main{display:grid;grid-template-columns:minmax(300px,420px) 1fr;gap:12px;padding:12px 16px;height:calc(100vh - 54px)}
@media(max-width:800px){main{grid-template-columns:1fr;height:auto}}
.col{display:flex;flex-direction:column;gap:12px;min-height:0;overflow:auto}
.card{background:var(--card);border:1px solid var(--bd);border-radius:8px;padding:12px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--mut);margin:0 0 8px}
.big{font-size:22px;font-weight:600}.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:4px 0}
button{font:inherit;padding:6px 10px;border:1px solid var(--bd);border-radius:6px;background:var(--card);color:var(--tx);cursor:pointer}
button:hover{border-color:var(--ac)}button:disabled{opacity:.45;cursor:not-allowed}
button.pri{background:var(--ac);color:#fff;border-color:var(--ac)}button.stop{background:var(--err);color:#fff;border-color:var(--err);font-weight:700;padding:10px 16px}
input[type=number],input[type=text]{font:inherit;padding:6px 8px;border:1px solid var(--bd);border-radius:6px;background:var(--bg);color:var(--tx);width:90px}
input[type=text]{width:auto;flex:1;min-width:120px}input[type=range]{flex:1;min-width:120px}
.bar{height:14px;background:var(--bg);border:1px solid var(--bd);border-radius:7px;overflow:hidden}.bar>i{display:block;height:100%;background:var(--ac);width:0}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 12px}.kv span:nth-child(odd){color:var(--mut)}
.badge{padding:2px 8px;border-radius:10px;font-size:12px;font-weight:600;background:var(--bd)}
.badge.BEREIT{background:var(--ok);color:#fff}.badge.FAHRT,.badge.KALIBRIERUNG{background:var(--ac);color:#fff}.badge.FEHLER{background:var(--err);color:#fff}.badge.UNKALIBRIERT,.badge.AKKU_LEER{background:var(--warn);color:#fff}
.err{color:var(--err);font-weight:600}.hint{color:var(--mut);font-size:12px}
#logcard{flex:1;display:flex;flex-direction:column;min-height:300px}
#log{flex:1;overflow:auto;background:var(--logbg);color:var(--logtx);border-radius:6px;padding:8px;font:12px/1.45 ui-monospace,monospace;white-space:pre-wrap;min-height:0}
#log .E{color:#ff7b72}#log .W{color:#e3b341}#log .T{color:#79c0ff}#log .I{color:#c9d1d9}#log .t{color:#6e7681}
</style></head><body>
<header><h1>Rollo-Debug</h1><span><span id="dot" class="dot"></span><span id="conn">getrennt</span></span>
<button id="rel" title="Port freigeben, z. B. zum Flashen">Port freigeben</button><span class="hint" id="relhint"></span></header>
<main>
<div class="col">
 <div class="card"><h2>Zustand</h2>
  <div class="row"><span id="st" class="badge">?</span><span id="err" class="err"></span></div>
  <div style="margin:8px 0"><div class="big" id="pct">–</div><div class="bar"><i id="barfill"></i></div><div class="hint" id="pos"></div></div>
  <div class="kv"><span>Akku</span><span id="batt">–</span><span>Temperatur</span><span id="ntc">–</span><span>Zigbee</span><span id="net">–</span></div>
  <div class="row"><button data-c="status">Status</button><label class="hint"><input type="checkbox" id="auto"> alle 2 s</label></div>
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
  <div class="hint">Erst mit „Tippen“ in die obere Lage, „Oben setzen“, dann nach unten tippen, „Unten setzen“.</div>
 </div>
 <div class="card"><h2>Weitere</h2>
  <div class="row"><button data-c="cal tilt">Kalibrieren (Anschlag)</button><button data-c="cal save">cal save</button><button data-c="cal abort">cal abort</button></div>
  <div class="row"><button data-c="batt">Akku messen</button><button data-c="restart" data-confirm="ESP neu starten?">Neustart</button><button id="hw" title="Reset über RTS, auch wenn die Firmware nicht antwortet">Chip-Reset</button><button data-c="reset factory" data-confirm="WERKSEINSTELLUNG: Kalibrierung löschen und Zigbee-Netz verlassen?">Werkseinstellung</button></div>
  <div class="row"><input type="text" id="raw" placeholder="Befehl, z. B. help"><button id="send">Senden</button></div>
 </div>
</div>
<div class="col"><div class="card" id="logcard"><h2>Log</h2>
 <div class="row"><button id="clr">Leeren</button><button id="sav">Speichern</button><label class="hint"><input type="checkbox" id="scroll" checked> Autoscroll</label>
 <label class="hint"><input type="checkbox" id="onlyw"> nur Warnungen/Fehler</label></div>
 <div id="log"></div></div></div>
</main>
<script>
const TOKEN="__TOKEN__";const $=id=>document.getElementById(id);
let S={connected:false};
async function cmd(c){
  const r=await fetch('/cmd',{method:'POST',headers:{'X-Token':TOKEN,'Content-Type':'application/json'},body:JSON.stringify({cmd:c})});
  if(!r.ok){const j=await r.json().catch(()=>({}));add({t:new Date().toLocaleTimeString(),lvl:'E',text:'Befehl "'+c+'" nicht gesendet: '+(j.error||r.status)});}
}
document.querySelectorAll('button[data-c]').forEach(b=>b.onclick=()=>{if(b.dataset.confirm&&!confirm(b.dataset.confirm))return;cmd(b.dataset.c);if(b.dataset.c!=='status')setTimeout(()=>cmd('status'),600)});
const jogs=[-1000,-300,-100,-30,30,100,300,1000];
jogs.forEach(v=>{const b=document.createElement('button');b.textContent=(v>0?'+':'')+v;b.onclick=()=>{cmd('jog tilt '+v);setTimeout(()=>cmd('status'),1500)};$('jogrow').appendChild(b)});
function jog(sign){const n=parseInt($('jn').value,10);if(!n||n<=0||n>100000)return alert('Schritte 1…100000');cmd('jog tilt '+(sign*n));setTimeout(()=>cmd('status'),1500)}
$('jm').onclick=()=>jog(-1);$('jp').onclick=()=>jog(1);
$('gp').oninput=()=>$('gpv').textContent=$('gp').value;
$('go').onclick=()=>{cmd('goto tilt '+$('gp').value);setTimeout(()=>cmd('status'),1500)};
$('lsb').onclick=()=>{const n=parseInt($('ls').value,10);if(!n||n<200)return alert('mindestens 200 Schritte');cmd('limit set tilt '+n);setTimeout(()=>cmd('status'),600)};
$('hw').onclick=()=>fetch('/reset',{method:'POST',headers:{'X-Token':TOKEN,'Content-Type':'application/json'},body:'{}'});
$('send').onclick=()=>{const v=$('raw').value.trim();if(v){cmd(v);$('raw').value=''}};
$('raw').onkeydown=e=>{if(e.key==='Enter')$('send').click()};
$('rel').onclick=async()=>{const on=!S.released;await fetch('/release',{method:'POST',headers:{'X-Token':TOKEN,'Content-Type':'application/json'},body:JSON.stringify({on})})};
let timer=null;$('auto').onchange=()=>{clearInterval(timer);if($('auto').checked)timer=setInterval(()=>{if(S.connected)cmd('status')},2000)};
const log=$('log');let lines=[];
function add(m){lines.push(m);if(lines.length>3000)lines.shift();draw(m)}
function el(m){const d=document.createElement('div');d.className=m.lvl;d.dataset.l=m.lvl;
  const t=document.createElement('span');t.className='t';t.textContent=m.t+' ';d.appendChild(t);d.appendChild(document.createTextNode(m.text));return d}
function draw(m){if($('onlyw').checked&&!'EWT'.includes(m.lvl))return;log.appendChild(el(m));while(log.childElementCount>3000)log.firstChild.remove();if($('scroll').checked)log.scrollTop=log.scrollHeight}
$('onlyw').onchange=()=>{log.textContent='';lines.forEach(draw)};
$('clr').onclick=()=>{lines=[];log.textContent=''};
$('sav').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([lines.map(m=>m.t+' '+m.text).join('\n')],{type:'text/plain'}));a.download='rollo-log-'+Date.now()+'.txt';a.click()};
function render(s){S=s;$('dot').className='dot'+(s.connected?' on':'');
  $('conn').textContent=s.released?'Port freigegeben':s.connected?'verbunden ('+s.port+')':'getrennt – warte auf Board';
  $('rel').textContent=s.released?'Port wieder öffnen':'Port freigeben';$('relhint').textContent=s.released?'zum Flashen frei':'';
  document.querySelectorAll('button[data-c],#go,#jm,#jp,#lsb,#send,#hw,#jogrow button').forEach(b=>b.disabled=!s.connected);
  const st=$('st');st.textContent=s.st;st.className='badge '+s.st;$('err').textContent=s.error;
  $('pct').textContent=s.tilt==null?(s.tilt_pos==null?'–':'unkalibriert'):s.tilt+' %';$('barfill').style.width=(s.tilt==null?0:s.tilt)+'%';
  $('pos').textContent=s.tilt_pos==null?'':'Position '+s.tilt_pos+(s.tilt_bottom!=null?' von '+s.tilt_bottom+' (unten)':'');
  $('batt').textContent=s.batt_mv==null?'–':(s.batt_mv/1000).toFixed(2)+' V, '+s.batt_pct+' %'+(s.charging?' (lädt)':'');
  $('ntc').textContent=s.ntc==null?'–':s.ntc.toFixed(1)+' °C';$('net').textContent=s.net}
const es=new EventSource('/events');
es.addEventListener('state',e=>{const prev=S.connected;render(JSON.parse(e.data));if(!prev&&S.connected)setTimeout(()=>cmd('status'),1500)});
es.addEventListener('log',e=>add(JSON.parse(e.data)));
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
