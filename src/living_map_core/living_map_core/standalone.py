"""Run the whole system in ONE process, no ROS, no Gazebo: python3 -m living_map_core.standalone
Opens the explanatory simulation on :8081 and the Command Post live map on :8080."""
import argparse, json, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from .sim_core import World, CH
from .simulation import Simulation
from .viewstate import ViewState
from .sim_html import SIM_HTML
from .dashboard_html import DASH_HTML
from .flow_html import FLOW_HTML

def world_json(w):
    return dict(name=w.name, cell=w.cell, x0=w.x0, y0=w.y0, van=list(w.van), van_size=list(w.van_size), floors=[["".join(CH[v] for v in row) for row in w.g[f]] for f in range(w.nf)],
                gas=w.gas, steam=w.steam, fires=w.fires and [dict(id=f["id"], x=f["x"], y=f["y"], floor=f["floor"]) for f in w.fires])

class Runner:
    def __init__(self, fault="none", auto=True, speed=4.0, map_path=None, cp_port=8080, uplink="immediate", collect=1.0):
        self.uplink, self.collect = uplink, collect; self.map_path, self.auto, self.cp_port = map_path, auto, cp_port
        self.view = ViewState(controls=True, cp_url=f"http://localhost:{cp_port}/"); self.view.speed = speed
        self.lock = threading.Lock(); self._new(fault)
    def _new(self, fault):
        self.sim = Simulation(fault, self.auto, self.map_path, view=self.view, uplink=self.uplink, collect=self.collect); self.view.paused = False
    def loop(self):
        acc, last = 0.0, time.time()
        while True:
            time.sleep(0.05); now = time.time(); dt, last = now - last, now
            if self.view.paused: continue
            acc += dt * self.view.speed
            with self.lock:
                n = 0
                while acc >= 0.5 and n < 40:
                    if not self.sim.final or self.sim.t < 5000: self.sim.step(0.5)
                    acc -= 0.5; n += 1
                acc = min(acc, 2.0)
    def control(self, d):
        with self.lock:
            a = d.get("action")
            if a == "pause": self.view.paused = True
            elif a == "resume": self.view.paused = False
            elif a == "speed": self.view.speed = float(d.get("value", 4.0))
            elif a == "approve": self.sim.approved = True
            elif a == "dispatch": self.sim.cp.force.add(d.get("role", ""))
            elif a == "restart": self._new(d.get("fault", "none"))
    def sim_state(self):
        with self.lock:
            j = self.view.sim_json(); j["instr"] = {r["bid"]: [r["hop_text"], r["tgt_text"]] for r in self.sim.cp.records.values()}; return j
    def cp_state(self):
        with self.lock: return self.sim.cp.dashboard_state(self.sim.t, self.sim.approved)
    def world(self):
        with self.lock: return world_json(self.sim.w)

def serve(port, routes):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _send(self, code, body, ctype):
            self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            r = routes.get(self.path.split("?")[0])
            if r is None: return self._send(404, b"not found", "text/plain")
            v = r(); self._send(200, v.encode() if isinstance(v, str) else json.dumps(v).encode(), "text/html; charset=utf-8" if isinstance(v, str) else "application/json")
        def do_POST(self):
            n = int(self.headers.get("Content-Length", 0) or 0); body = self.rfile.read(n) if n else b"{}"
            r = routes.get("POST " + self.path)
            if r: r(json.loads(body or b"{}"))
            self._send(200, b"{}", "application/json")
    s = ThreadingHTTPServer(("0.0.0.0", port), H); threading.Thread(target=s.serve_forever, daemon=True).start(); return s

def main():
    ap = argparse.ArgumentParser(description="Living Map standalone demo (no ROS)")
    ap.add_argument("--map", default=None, help="map file (JSON), default maps/complex.json"); ap.add_argument("--fault", default="none", choices=["none", "beacon_destroyed", "writer_lost"])
    ap.add_argument("--uplink", default="immediate", choices=["immediate", "batched"], help="immediate = every beacon goes to the Command Post at once; batched = 120 s window"); ap.add_argument("--collect", type=float, default=1.0, help="seconds the Command Post collects related reports before issuing ONE order"); ap.add_argument("--speed", type=float, default=4.0); ap.add_argument("--no-auto-approve", action="store_true"); ap.add_argument("--sim-port", type=int, default=8081); ap.add_argument("--cp-port", type=int, default=8080)
    a = ap.parse_args(); R = Runner(a.fault, not a.no_auto_approve, a.speed, a.map, a.cp_port, a.uplink, a.collect)
    serve(a.sim_port, {"/": lambda: SIM_HTML, "/flow": lambda: FLOW_HTML, "/world": R.world, "/simstate": R.sim_state, "POST /control": R.control})
    serve(a.cp_port, {"/": lambda: DASH_HTML, "/state": R.cp_state, "POST /approve": lambda d: R.control({"action": "approve"}), "POST /dispatch": lambda d: R.control({"action": "dispatch", "role": d.get("role")})})
    threading.Thread(target=R.loop, daemon=True).start()
    print(f"\n  Simulation:    http://localhost:{a.sim_port}\n  Command Post:  http://localhost:{a.cp_port}\n  (Ctrl+C to stop)\n")
    try:
        while True: time.sleep(1)
    except KeyboardInterrupt: pass

if __name__ == "__main__": main()
