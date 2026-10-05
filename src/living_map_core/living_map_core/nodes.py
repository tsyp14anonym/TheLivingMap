"""ROS 2 wrappers (v4: multi-floor, multi-robot, star topology through the van).
Inside the building = ROS_DOMAIN_ID 10, outside (Command Post) = 20. The ONLY crossings between them are the van gateway's UDP uplink
(wireless/satellite stand-in) and the pre-entry briefing files handed to each robot while it is docked in the van.
Robots have NO topic to each other: robot <-> van only (/station/from/<role>, /station/to/<role>).
/world/action is SIMULATOR PLUMBING (it replicates physical effects such as 'the fire lost intensity' between simulator processes); it is not a communication channel."""
import json, os, socket, threading, time, http.server, subprocess, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .sim_core import World, dist, WALL, RUBBLE
from .geo import Anchor
from .agents import WriterAgent, Mesh, GatewayCore
from .command_post import CommandPost, ROLES
from .roles import Responder
from .viewstate import ViewState, grid_rows
from .sim_html import SIM_HTML
from .dashboard_html import DASH_HTML
from .flow_html import FLOW_HTML
from .beacon_protocol import unpack

RUN_DIR = "/tmp/living_map"
UPLINK_ADDR = ("127.0.0.1", 9101)          # van -> Command Post (beacon records, robot reports)
DOWNLINK_ADDR = ("127.0.0.1", 9102)        # Command Post -> van (ORDER_UPDATE for a robot that is already inside); the van relays it on /station/to/<role>
def mission_path(role): return f"{RUN_DIR}/mission_{role}.json"

class Clock:
    def __init__(self, epoch, speed): self.epoch, self.speed = epoch, speed
    def now(self): return (time.time() - self.epoch) * self.speed

def _params(node, **defaults):
    for k, v in defaults.items(): node.declare_parameter(k, v)
    return {k: node.get_parameter(k).value for k in defaults}

def _substeps(last, t, fn):
    total = min(t - last, 2.0); done = 0.0
    while total - done > 1e-6:
        dt = min(0.25, total - done); done += dt; fn(dt, last + done)
    return t

class LogMixin:
    def slog(self, who, msg, t=0.0, level="info"):
        if not hasattr(self, "_pub_log"): self._pub_log = self.create_publisher(String, "/sim/log", 100)
        self._pub_log.publish(String(data=json.dumps(dict(t=round(t, 1), who=who, msg=msg))))
        line, lg = f"[t={t:6.1f}s] {who}: {msg}", self.get_logger()
        # real rclpy remembers ONE severity per call site and raises ValueError if the same line logs with another one,
        # so each severity gets its own line (never getattr(logger, level)(...))
        if level == "warn": lg.warn(line)
        elif level == "error": lg.error(line)
        else: lg.info(line)

class SharedWorld(World):
    """Each simulator process owns a copy of the ground truth. Physical effects (dousing a fire, rescuing a person) are replicated on /world/action."""
    def __init__(self, node):
        super().__init__(); self._node, self._me = node, node.get_name()
        self._pub = node.create_publisher(String, "/world/action", 50); node.create_subscription(String, "/world/action", self._on, 50)
    def douse(self, f, aim, radius, amount):
        hit = super().douse(f, aim, radius, amount)
        if hit: self._pub.publish(String(data=json.dumps(dict(op="douse", src=self._me, f=f, aim=list(aim), radius=radius, amount=amount))))
        return hit
    def secure(self, f, pos, radius, amount):
        hit = super().secure(f, pos, radius, amount)
        if hit: self._pub.publish(String(data=json.dumps(dict(op="secure", src=self._me, f=f, pos=list(pos), radius=radius, amount=amount))))
        return hit
    def rescue_at(self, f, pos, radius=1.6):
        o = super().rescue_at(f, pos, radius)
        if o: self._pub.publish(String(data=json.dumps(dict(op="rescue", src=self._me, id=o["id"]))))
        return o
    def _on(self, m):
        d = json.loads(m.data)
        if d["src"] == self._me: return
        if d["op"] == "douse": World.douse(self, d["f"], tuple(d["aim"]), d["radius"], d["amount"])
        elif d["op"] == "secure": World.secure(self, d["f"], tuple(d["pos"]), d["radius"], d["amount"])
        elif d["op"] == "rescue":
            for o in self.humans + self.animals:
                if o["id"] == d["id"]: o["rescued"] = True

# ------------------------------------------------------------------ inside: Writer (scout)
class WriterNode(LogMixin, Node):
    def __init__(self):
        super().__init__("writer")
        p = _params(self, epoch=0.0, speed=2.0, kill_at=-1.0)
        self.clock = Clock(p["epoch"], p["speed"]); self.world = SharedWorld(self)
        self.agent = WriterAgent(self.world, kill_at=None if p["kill_at"] < 0 else p["kill_at"])
        self.pub_tx = self.create_publisher(String, "/beacon_tx", 50)
        self.pub_st = self.create_publisher(String, "/station/from/writer", 10); self._done_sent = False        # the Writer talks to the van only (it reports "exploration complete")
        self.pub_pose = self.create_publisher(String, "/writer/pose", 10)
        self.pub_known = self.create_publisher(String, "/writer/known", 2)
        self.last, self.logged, self.pose_t, self.known_t = None, 0, 0.0, 0.0
        self.create_timer(0.05, self.tick); self._hbt = 0.0
        self.get_logger().info("Writer ready (inside domain 10). Waiting for the simulation clock...")
    def _step(self, dt, t):
        for tx in self.agent.tick(dt, t): self.pub_tx.publish(String(data=json.dumps(tx)))
    def tick(self):
        t = self.clock.now()
        if t < 0: return
        if time.time() - self._hbt > 10:
            self._hbt = time.time(); r = self.agent.r
            self.get_logger().info(f"HEARTBEAT writer: t={t:.0f}s, pos=({r.pos[0]:.1f},{r.pos[1]:.1f}) floor {r.f}, {len(self.agent.beacons)} beacon(s) dropped, {'DONE' if self.agent.done else 'exploring'}")
        self.last = _substeps(self.last, t, self._step) if self.last is not None else t
        while self.logged < len(self.agent.log):
            tt, m = self.agent.log[self.logged]; self.logged += 1; self.slog("WRITER", m, tt, "warn")
        if self.agent.done and not self._done_sent: self._done_sent = True; self.pub_st.publish(String(data=json.dumps(dict(robot="writer", kind="EXPLORATION_DONE", payload={}, t=round(t, 1)))))
        now = time.time(); r = self.agent.r
        if now - self.pose_t > 0.2:
            self.pose_t = now
            self.pub_pose.publish(String(data=json.dumps(dict(true=r.pos, f=r.f, yaw=r.yaw, done=self.agent.done, dead=self.agent.dead, phase=self.agent.phase, beacons=len(self.agent.beacons), sensors=self.agent.last, t=t))))
        if now - self.known_t > 1.0:
            self.known_t = now; self.pub_known.publish(String(data=json.dumps([grid_rows(r.k.g[f]) for f in range(self.world.nf)])))

# ------------------------------------------------------------------ inside: radio mesh (physics of the beacons)
class MeshNode(LogMixin, Node):
    def __init__(self):
        super().__init__("beacon_mesh")
        p = _params(self, epoch=0.0, speed=2.0, fault="none")
        self.clock, self.fault, self.killed = Clock(p["epoch"], p["speed"]), p["fault"], False
        self.mesh = Mesh(World())
        self.pub_rx = self.create_publisher(String, "/beacon_rx", 50); self.pub_dead = self.create_publisher(String, "/beacon_destroyed", 10)
        self.pub_state = self.create_publisher(String, "/beacon_state", 50)
        self.create_subscription(String, "/beacon_tx", self.on_tx, 50); self.create_subscription(String, "/beacon_write", self.on_write, 50); self.create_timer(0.5, self.tick)
    def on_tx(self, m):
        tx = json.loads(m.data); now = self.clock.now()
        ds = self.mesh.register(tx["hex"], tx["true"], tx["floor"], tx["bid"], now)
        if not ds: self.slog("MESH", f"beacon {tx['bid']} ({tx['type']}) not delivered yet: relays keep retrying, mirrors hold a copy", now, "warn")
        for d in ds: self.pub_rx.publish(String(data=json.dumps(d)))
    def on_write(self, m):
        d = json.loads(m.data); now = self.clock.now()
        ok = self.mesh.write_state(d["bid"], d["state"], d["by"], d["pos"], d["floor"], now)
        self.pub_state.publish(String(data=json.dumps(dict(bid=d["bid"], state=d["state"], by=d["by"], ok=ok, t=now))))
        self.slog("BEACON", f"#{d['bid']} state -> {d['state']} written by {d['by']}" + ("" if ok else "  FAILED: out of radio range"), now, "info" if ok else "warn")
    def tick(self):
        now = self.clock.now()
        if now < 0: return
        if self.fault == "beacon_destroyed" and not self.killed and now >= 150:
            kids = {}
            for b, n in self.mesh.nodes.items(): kids[n["parent"]] = kids.get(n["parent"], 0) + 1
            cand = [b for b, n in self.mesh.nodes.items() if n["alive"] and kids.get(b, 0) >= 1]
            if cand:
                b = max(cand, key=lambda b: kids.get(b, 0)); self.mesh.kill(b, now); self.killed = True
                self.pub_dead.publish(String(data=json.dumps(dict(bid=b)))); self.slog("FAULT", f"beacon {b} DESTROYED", now, "error")
        for d in self.mesh.tick(now):
            self.slog("MESH", "delivery via self-healing route / BEACON_LOST reported by a neighbour (silence is information)", now, "warn"); self.pub_rx.publish(String(data=json.dumps(d)))

# ------------------------------------------------------------------ THE VAN: Outside Network Area gateway (the only station robots talk to)
class GatewayNode(LogMixin, Node):
    def __init__(self):
        super().__init__("outside_network_gateway")
        p = _params(self, epoch=0.0, speed=2.0, interval=120.0, uplink="immediate")
        self.clock, self.world = Clock(p["epoch"], p["speed"]), World(); self.anchor = Anchor(**self.world.anchor)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.core = GatewayCore(self.anchor, self.send, interval=p["interval"], immediate=(p["uplink"] == "immediate")); self.n_in, self.n_up, self._hbt = 0, 0, 0.0
        self.n_down = 0; self.dsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try: self.dsock.bind(DOWNLINK_ADDR); self.dsock.setblocking(False)
        except OSError as e: self.dsock = None; self.get_logger().error(f"VAN CANNOT LISTEN on {DOWNLINK_ADDR}: {e}. Orders cannot be updated while robots are out. Run scripts/run_clean.sh.")
        self.create_subscription(String, "/beacon_rx", self.on_rx, 50)
        self.to = {r: self.create_publisher(String, f"/station/to/{r}", 10) for r in ROLES}
        for r in list(ROLES) + ["writer"]: self.create_subscription(String, f"/station/from/{r}", lambda m, r=r: self.on_station(r, m), 20)   # STAR: only the van listens to robots
        self.create_timer(0.2, self._downlink)
        self.create_timer(0.5, lambda: self.core.tick(self.clock.now()) if self.clock.now() >= 0 else None); self.create_timer(5.0, self._hb)
        self.get_logger().info("VAN READY: RF in -> verify -> rebuild positions -> GPS -> queue -> UDP uplink. Robots talk to the van only (star topology).")
    def on_rx(self, m):
        self.n_in += 1; d = json.loads(m.data); now = self.clock.now(); rej = self.core.rejected
        rec = self.core.on_rx(d["hex"], d["rssi"], d["hops"], now)
        if self.core.rejected > rej: self.slog("GATEWAY", "forged/corrupt packet REJECTED (CRC/MAC)", now, "error")
        elif rec: self.slog("GATEWAY", f"{rec['id']} {rec['type']} P{rec['prio']} floor {rec['floor']} | beacon is {rec['hop_text']} | target: {rec['tgt_text']} | GPS {rec['lat']:.6f}, {rec['lon']:.6f}", now)
    def _hb(self):
        if time.time() - self._hbt < 10: return
        self._hbt = time.time(); now = self.clock.now()
        if now < 0: return
        self.get_logger().info(f"HEARTBEAT van: t={now:.0f}s, {self.n_in} radio packet(s) in, {self.core.received} decoded, {self.n_up} uplink(s) sent, {sum(len(v) for v in self.core.waiting.values())} waiting for a parent beacon")
        if self.n_in == 0 and now > 60: self.get_logger().error("Van has received NO radio packets: the beacon mesh or the Writer is not producing. Check 'HEARTBEAT writer' and 'beacon_mesh'.")
    def _downlink(self):
        """Command Post -> van -> robot. The van is the only relay: it republishes on /station/to/<role> (the robot's own topic)."""
        if self.dsock is None: return
        while True:
            try: data, _ = self.dsock.recvfrom(65535)
            except (BlockingIOError, InterruptedError): return
            d = json.loads(data.decode()); role = d.get("role")
            if role in self.to: self.n_down += 1; self.to[role].publish(String(data=json.dumps(dict(kind=d["kind"], payload=d["payload"], t=d.get("t"))))); self.slog("DOWNLINK", f"Command Post -> van -> {role}: {d['kind']} {sorted(d['payload'])}", self.clock.now(), "warn")
    def on_station(self, role, m):
        d = json.loads(m.data); now = self.clock.now()
        self.sock.sendto(json.dumps(dict(reason="STATUS", sent_t=now, records=[], statuses=[d])).encode(), UPLINK_ADDR)      # forward FIRST: logging must never block the data path
        if role in self.to: self.to[role].publish(String(data=json.dumps(dict(ack=d["kind"], t=round(now, 1)))))          # van -> robot (placeholder for the future protocol)
        self.slog("STATION", f"{role} -> van: {d['kind']} {d.get('payload') or ''}", now)
    def send(self, records, reason):
        now = self.clock.now(); self.n_up += 1
        self.sock.sendto(json.dumps(dict(reason=reason, sent_t=now, records=records)).encode(), UPLINK_ADDR)
        self.slog("UPLINK", f"[{reason}] " + ", ".join(r["id"] for r in records), now, "warn")

# ------------------------------------------------------------------ outside: Command Post + live map
class CommandPostNode(Node):
    def __init__(self):
        super().__init__("command_post")
        p = _params(self, epoch=0.0, speed=2.0, auto_approve=True, port=8080, collect=1.0, cycle=1.0, crit=0.3)
        self.epoch, self.clock = p["epoch"], Clock(p["epoch"], p["speed"]); w = World(); self.anchor = Anchor(**w.anchor)
        self.cp, self.lock, self.approved = CommandPost(self.anchor, collect_window=p["collect"], cycle=p["cycle"], crit=p["crit"]), threading.Lock(), p["auto_approve"]; self.n_rx, self._hbt = 0, 0.0
        self.dsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)          # Command Post -> van (downlink)
        os.makedirs(RUN_DIR, exist_ok=True)
        for r in ROLES:
            if os.path.exists(mission_path(r)): os.remove(mission_path(r))
        self.pub = self.create_publisher(String, "/command_post/state", 10)
        threading.Thread(target=self._udp, daemon=True).start()
        node = self
        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def _send(self, code, body, ctype):
                self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers(); self.wfile.write(body)
            def do_GET(self):
                if self.path == "/state": self._send(200, json.dumps(node.state()).encode(), "application/json")
                else: self._send(200, DASH_HTML.encode(), "text/html; charset=utf-8")
            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0) or 0); body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                if self.path == "/approve": node.approved = True; node.get_logger().warn("OPERATOR approved the dispatch"); self._send(200, b"ok", "text/plain")
                elif self.path == "/dispatch":
                    with node.lock: node.cp.force.add(body.get("role", ""))
                    node.get_logger().warn(f"OPERATOR forced dispatch of {body.get('role')}"); self._send(200, b"ok", "text/plain")
                else: self._send(404, b"", "text/plain")
        self.httpd = http.server.ThreadingHTTPServer(("0.0.0.0", p["port"]), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.create_timer(0.5, self.tick); self.create_timer(5.0, self._hb)
        self.get_logger().info(f"COMMAND POST live map: http://localhost:{p['port']}  (outside domain 20, receives ONLY via the van's uplink)")
    def _udp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)      # no SO_REUSEADDR: a stale Command Post must make this fail loudly, not silently steal packets
        try: s.bind(UPLINK_ADDR)
        except OSError as e:
            self.get_logger().error(f"COMMAND POST CANNOT LISTEN on {UPLINK_ADDR}: {e}. An old run is still alive. Run scripts/run_clean.sh. Nothing will be dispatched until this is fixed.")
            return
        self.get_logger().info(f"Command Post listening for the van's uplink on {UPLINK_ADDR}")
        while True:
            data, _ = s.recvfrom(65535); batch = json.loads(data); self.n_rx += 1
            with self.lock: self.cp.ingest(batch, self.clock.now())
            if batch["reason"] != "STATUS": self.get_logger().info(f"batch [{batch['reason']}] received: {len(batch['records'])} record(s)")
    def _hb(self):
        if time.time() - self._hbt < 10: return
        self._hbt = time.time(); now = self.clock.now()
        if now < 0: return
        self.get_logger().info(f"HEARTBEAT command_post: t={now:.0f}s, {self.n_rx} uplink message(s) received, {len(self.cp.records)} beacon record(s), robots={ {r: v['state'] for r, v in self.cp.state.items()} }")
        if self.n_rx == 0 and now > 60: self.get_logger().error("Command Post has received NOTHING from the van. Check the 'VAN' heartbeat and run scripts/check_run.sh")
    def tick(self):
        now = self.clock.now()
        if now < 0: return
        with self.lock:
            for role, m in self.cp.ready(now, self.epoch, self.approved):
                tmp = mission_path(role) + ".tmp"
                with open(tmp, "w") as f: json.dump(m, f)
                os.replace(tmp, mission_path(role))
                self.get_logger().warn(f"{role.upper()} DISPATCHED (briefing handed over in the van): {len(m['targets'])} target(s), {len(m['hazards'])} hazard(s)")
            for u in self.cp.pop_outbox():
                self.dsock.sendto(json.dumps(u).encode(), DOWNLINK_ADDR); self.get_logger().warn(f"ORDER UPDATE for {u['role'].upper()} sent to the van: {sorted(u['payload'])}")
            self.pub.publish(String(data=json.dumps(dict(n=len(self.cp.records)))))
    def state(self):
        with self.lock: return self.cp.dashboard_state(self.clock.now(), self.approved)

# ------------------------------------------------------------------ inside: responders (ambulance / firetruck / drone) - one node each
class RosLink:
    """Robot side of the station link: publishes ONLY to /station/from/<role> (the van)."""
    def __init__(self, node, role): self.pub, self.role = node.create_publisher(String, f"/station/from/{role}", 10), role
    def send(self, robot, kind, payload, now): self.pub.publish(String(data=json.dumps(dict(robot=robot, kind=kind, payload=payload, t=round(now, 1)))))

class ResponderNode(LogMixin, Node):
    def __init__(self, role):
        super().__init__(role); self.role = role
        self.setup(role)
    def setup(self, role):
        p = _params(self, epoch=0.0, speed=2.0, blind=False, nav="direct")
        self.clock, self.blind, self.nav = Clock(p["epoch"], p["speed"]), p["blind"], p["nav"]
        self.world = SharedWorld(self); self.anchor = Anchor(**self.world.anchor); self.link = RosLink(self, role)
        self.create_subscription(String, f"/station/to/{role}", self._on_station, 10)          # the ONLY thing a robot listens to: the van (acks, ORDER_UPDATE from the Command Post)
        self.pub_pose = self.create_publisher(String, f"/responder/{role}/pose", 10)
        self.pub_bw = self.create_publisher(String, "/beacon_write", 20)          # robot -> beacon (short-range RF write of the STATE byte), not a robot-robot channel
        self.agent, self.last, self.logged, self.pose_t, self.issued, self.results = None, None, 0, 0.0, None, []
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f"{role.upper()} docked in the van. Waiting for a briefing.")
    def _step(self, dt, t): self.agent.tick(dt, t)
    def _on_station(self, m):
        d = json.loads(m.data)
        if d.get("kind") == "ORDER_UPDATE" and self.agent is not None and not self.agent.done:
            t = self.clock.now()
            if self.agent.apply_update(d["payload"], t): self.slog(self.role.upper(), f"order updated by the Command Post (via the van): {sorted(d['payload'])}", t, "warn")
    def beacon_cb(self, bid, state, note, t, pos, floor, role):
        self.pub_bw.publish(String(data=json.dumps(dict(bid=bid, state=state, by=role, pos=pos, floor=floor, t=t)))); return True
    def tick(self):
        t = self.clock.now()
        if t < 0: return
        if t - getattr(self, "_st_t", -99.0) >= 10.0:                      # heartbeat: the Command Post can recover from a lost RETURNED message
            self._st_t = t; self.link.send(self.role, "STATUS", dict(active=bool(self.agent and not self.agent.done), state=(self.agent.state if self.agent else "DOCKED"), issued=self.issued), t)
        if self.agent is None or self.agent.done:
            if self.agent is not None and self.agent.result and self.agent.result not in self.results: self.results.append(self.agent.result)
            mp = mission_path(self.role)
            if self.blind and self.agent is None: self.agent = Responder(self.world, self.role, None, self.anchor, self.link, use_beacons=False, beacon_cb=self.beacon_cb); self.last = t
            elif not self.blind and os.path.exists(mp):
                try:
                    with open(mp) as f: m = json.load(f)
                except ValueError: return
                if m["issued_at"] != self.issued:
                    self.issued = m["issued_at"]; self.agent = Responder(self.world, self.role, m, self.anchor, self.link, beacon_cb=self.beacon_cb, nav=self.nav); self.logged = 0; self.last = t
                    self.slog("CMDPOST", f"{self.role} dispatched: briefing loaded in the van ({len(m['targets'])} target(s))", t, "warn")
            if self.agent is None or self.agent.done: self._pose(t); return
        self.last = _substeps(self.last, t, self._step)
        while self.logged < len(self.agent.log):
            tt, msg = self.agent.log[self.logged]; self.logged += 1; self.slog(self.role.upper(), msg, tt, "warn")
        self._pose(t)
    def _pose(self, t):
        if time.time() - self.pose_t < 0.2: return
        self.pose_t = time.time(); a = self.agent; r = (a.r if a else None)
        from .roles import ROLE_CFG; pos = r.pos if r else list(ROLE_CFG[self.role]["start"]); f = r.f if r else 0
        self.pub_pose.publish(String(data=json.dumps(dict(true=pos, f=f, yaw=(r.yaw if r else 0.0), state=(a.state if a else "DOCKED"), active=bool(a and not a.done), battery=(a.battery if a else None),
                                                          results=self.results + ([a.result] if a and a.done and a.result and a.result not in self.results else []), t=t))))

# ------------------------------------------------------------------ inside: simulator monitor (web god-view + RViz topics)
class SimViewNode(Node):
    def __init__(self):
        super().__init__("sim_view")
        p = _params(self, epoch=0.0, speed=2.0, port=8081, cp_port=8080)
        self.clock = Clock(p["epoch"], p["speed"]); self.lock = threading.Lock(); self.world = SharedWorld(self)
        self.view = ViewState(controls=False, cp_url=f"http://localhost:{p['cp_port']}/"); self.view.speed = p["speed"]
        self.final, self.results, self.idle_since, self.wdone = False, {}, None, False
        self.pub_truth = self.create_publisher(String, "/sim/truth", 2)
        for topic, cb in (("/writer/pose", self.on_w), ("/writer/known", self.on_known), ("/beacon_tx", self.on_tx), ("/beacon_destroyed", self.on_dead), ("/sim/log", self.on_log), ("/beacon_rx", self.on_rx), ("/beacon_state", self.on_state)):
            self.create_subscription(String, topic, cb, 50)
        for r in ROLES: self.create_subscription(String, f"/responder/{r}/pose", lambda m, r=r: self.on_r(r, m), 20)
        self.rviz = False
        try:
            from nav_msgs.msg import OccupancyGrid
            from visualization_msgs.msg import Marker, MarkerArray
            from tf2_ros import StaticTransformBroadcaster
            from geometry_msgs.msg import TransformStamped
            self._M, self._MA, self._OG = Marker, MarkerArray, OccupancyGrid
            self.pub_map = [self.create_publisher(OccupancyGrid, f"/living_map/known_map_f{f}", 1) for f in range(self.world.nf)]
            self.pub_mk = self.create_publisher(MarkerArray, "/living_map/markers", 1)
            tf = TransformStamped(); tf.header.frame_id = "world"; tf.child_frame_id = "map"; tf.transform.rotation.w = 1.0
            self._tfb = StaticTransformBroadcaster(self); self._tfb.sendTransform(tf); self.rviz = True
        except Exception as e: self.get_logger().warn(f"RViz topics disabled ({e})")
        node = self
        class H(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a): pass
            def _s(self, body, ct): self.send_response(200); self.send_header("Content-Type", ct); self.send_header("Cache-Control", "no-store"); self.send_header("Access-Control-Allow-Origin", "*"); self.send_header("Access-Control-Allow-Origin", "*"); self.end_headers(); self.wfile.write(body)
            def do_GET(self):
                if self.path == "/flow": return self._s(FLOW_HTML.encode(), "text/html; charset=utf-8")
                with node.lock:
                    if self.path == "/world": return self._s(json.dumps(node.world_json()).encode(), "application/json")
                    if self.path == "/simstate": node.view.t = node.clock.now(); node.view.set_truth(node.world); return self._s(json.dumps(node.view.sim_json()).encode(), "application/json")
                self._s(SIM_HTML.encode(), "text/html; charset=utf-8")
            def do_POST(self): self._s(b"ok", "text/plain")
        self.httpd = http.server.ThreadingHTTPServer(("0.0.0.0", p["port"]), H)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.create_timer(0.5, self.publish_rviz); self.create_timer(1.0, self.publish_truth)
        self.get_logger().info(f"SIMULATION VIEW (ground truth, explanatory): http://localhost:{p['port']}   |   RViz topics: {self.rviz}")
    def world_json(self):
        from .standalone import world_json; return world_json(self.world)
    def publish_truth(self):
        w = self.world; self.pub_truth.publish(String(data=json.dumps(dict(fires=[f["I"] for f in w.fires], humans=[h["rescued"] for h in w.humans], animals=[a["rescued"] for a in w.animals], gas=[g["I"] for g in w.gas]))))
    def on_w(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.view.t = d.get("t", self.view.t); self.view.set_robot("writer", d["true"][0], d["true"][1], d["f"], "dead" if d["dead"] else "mapped" if d["done"] else "GPS to the door" if d.get("phase") == "gps" else "exploring", d["done"]); self.view.sensors = d.get("sensors"); self.wdone = d["done"] or d["dead"]
    def on_known(self, m):
        with self.lock: self.view.known = json.loads(m.data)
    def on_tx(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.view.add_beacon(d["bid"], d["type"], d["prio"], d["true"][0], d["true"][1], d["floor"], d.get("parent", 0)); self.view.set_state(d["bid"], "DROPPED", "writer")
            self.view.add_flow("DROP", "writer", "beacons", f"#{d['bid']} {d['type']} P{d['prio']}")
    def on_rx(self, m):
        d = json.loads(m.data)
        try: bid = unpack(bytes.fromhex(d["hex"])).bid
        except ValueError: return
        with self.lock: self.view.add_flow("RF", "beacons", "van", f"#{bid} ({d['hops']} hops, {d['rssi']} dBm)")
    def on_state(self, m):
        d = json.loads(m.data)
        with self.lock:
            if d["ok"]: self.view.set_state(d["bid"], d["state"], d["by"])
            self.view.add_flow("WRITE", d["by"], "beacons", f"#{d['bid']} -> {d['state']}" + ("" if d["ok"] else " (out of radio range)"))
    def on_dead(self, m):
        with self.lock: self.view.kill_beacon(json.loads(m.data)["bid"])
    def on_log(self, m):
        d = json.loads(m.data)
        with self.lock:
            self.view.add_log(d["who"], d["msg"], d["t"])
            if d["who"] == "GATEWAY": self.view.flags["gw"] = True
            if d["who"] == "UPLINK": self.view.flags["uplink"] = True
            if d["who"] == "CMDPOST": self.view.flags["dispatched"] = True
    def on_r(self, role, m):
        d = json.loads(m.data)
        with self.lock:
            self.view.set_robot(role, d["true"][0], d["true"][1], d["f"], d["state"], not d["active"], d.get("battery")); self.results[role] = d["results"]
            self.view.board = dict(self.view.board or {}); self.view.board[role] = dict(note=("working" if d["active"] else "docked in the van"))
            active = any(not r["done"] and r["state"] != "DOCKED" for r in self.view.robots.values() if r["role"] != "writer")
            done_any = any(v for v in self.results.values())
            if active:
                self.idle_since = None
                if self.final: self.final, self.view.metrics = False, None          # a robot went out again: the earlier summary is withdrawn
            elif self.wdone and done_any and not self.final:
                # quiet period: the Command Post batches reports for up to 120 s and may hold the ambulance up to 150 s, so wait longer than both
                self.idle_since = self.idle_since or self.clock.now()
                if self.clock.now() - self.idle_since > 200: self.final = True; threading.Thread(target=self._final, daemon=True).start()
    def _final(self):
        from .simulation import blind_baseline
        blind = blind_baseline(); w = self.world; bs = list(self.view.beacons.values())
        hum = [v for rs in self.results.get("ambulance", []) if rs for k, v in rs.get("arrivals", {}).items() if k.endswith(":HUMAN")]
        wb = min(hum) if hum else None
        ah = [rs["t_start"] + v for rs in self.results.get("ambulance", []) if rs and "t_start" in rs for k, v in rs.get("arrivals", {}).items() if k.endswith(":HUMAN")]; abs_b = round(min(ah), 1) if ah else None
        with self.lock:
            self.view.metrics = dict(with_beacons_s=wb, first_person_abs_s=abs_b, blind_s=round(blind.get("time", 0), 1), gain_pct=round(100 * (1 - wb / blind["time"])) if wb and blind.get("success") else None, position_error_m=None, position_error_max_m=None,
                beacons=len(bs), repeaters=sum(1 for b in bs if b["type"] == "REPEATER"), delivery="-", rejected=0, fires_out=sum(1 for f in w.fires if f["I"] < 0.1), fires=len(w.fires),
                rescued=sum(h["rescued"] for h in w.humans + w.animals), people=len(w.humans) + len(w.animals), skipped=[], robots={})
    # ---- RViz ----
    def _mk(self, ns, i, typ, x=0.0, y=0.0, z=0.0, s=(1.0, 1.0, 1.0), rgba=(1, 1, 1, 1), text=None, pts=None):
        M = self._M; mk = M(); mk.header.frame_id = "map"; mk.ns, mk.id, mk.type, mk.action = ns, i, typ, M.ADD
        mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = float(x), float(y), float(z); mk.pose.orientation.w = 1.0
        mk.scale.x, mk.scale.y, mk.scale.z = [float(v) for v in s]; mk.color.r, mk.color.g, mk.color.b, mk.color.a = [float(v) for v in rgba]
        if text is not None: mk.text = text
        if pts is not None:
            from geometry_msgs.msg import Point
            mk.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in pts]
        return mk
    def publish_rviz(self):
        if not self.rviz: return
        M = self._M; w = self.world; PC = {0: (1, .2, .2, 1), 1: (1, .65, .25, 1), 2: (1, .88, .4, 1), 3: (.55, .6, .7, 1)}; SC = {"RESOLVED": (.2, 1, .4, 1), "IN_PROGRESS": (.3, .9, 1, 1), "ON_SITE": (.7, .5, 1, 1), "UNREACHABLE": (.5, .1, .1, 1)}   # beacon colour follows the state an executor wrote into it
        RC = {"writer": (.25, .8, 1, 1), "ambulance": (.5, 1, .6, 1), "firetruck": (1, .55, .25, 1), "drone": (.83, .55, 1, 1)}
        Z = w.z; c = w.cell
        with self.lock:
            v = self.view; mks = []
            if v.known:
                for f in range(w.nf):
                    og = self._OG(); og.header.frame_id = "map"; og.header.stamp = self.get_clock().now().to_msg(); og.info.resolution, og.info.width, og.info.height = c, w.nx, w.ny
                    og.info.origin.position.x, og.info.origin.position.y, og.info.origin.position.z, og.info.origin.orientation.w = w.x0, w.y0, Z[f], 1.0
                    og.data = [{"?": -1, ".": 0, "#": 100, "R": 100, "S": 0, "A": 0}[ch] for row in v.known[f] for ch in row]; self.pub_map[f].publish(og)
            for f in range(w.nf):
                wall = [(w.x0 + (i + .5) * c, w.y0 + (j + .5) * c, Z[f] + .75) for j in range(w.ny) for i in range(w.nx) if w.g[f][j][i] == WALL]
                rub = [(w.x0 + (i + .5) * c, w.y0 + (j + .5) * c, Z[f] + .25) for j in range(w.ny) for i in range(w.nx) if w.g[f][j][i] == RUBBLE]
                mks += [self._mk("walls", f, M.CUBE_LIST, s=(c, c, 1.5), rgba=(.55, .6, .7, .5), pts=wall), self._mk("rubble", f, M.CUBE_LIST, s=(c, c, .5), rgba=(.7, .5, .3, 1), pts=rub)]
            for i, fr in enumerate(w.fires):
                if fr["I"] > .05: mks += [self._mk("fire", i, M.CYLINDER, fr["x"], fr["y"], Z[fr["floor"]] + .15, (4 * fr["I"] + .5, 4 * fr["I"] + .5, .3), (1, .4, .1, .6)), self._mk("fire_t", i, M.TEXT_VIEW_FACING, fr["x"], fr["y"], Z[fr["floor"]] + 1.8, (0, 0, .6), (1, .6, .3, 1), f"FIRE {fr['id']}")]
            for i, g_ in enumerate(w.gas):
                if g_["I"] > 0.05: mks.append(self._mk("gas", i, M.SPHERE, g_["x"], g_["y"], Z[g_["floor"]] + .8, (5, 5, 1.6), (.7, 1, .3, .3)))
            for i, s_ in enumerate(w.steam): mks.append(self._mk("steam", i, M.SPHERE, s_["x"], s_["y"], Z[s_["floor"]] + .8, (2, 2, 1.6), (.85, .85, .9, .3)))
            for i, h in enumerate(w.humans): mks += [self._mk("human", i, M.CYLINDER, h["x"], h["y"], Z[h["floor"]] + .85, (.5, .5, 1.7), (.3, 1, .4, 1) if h["rescued"] else (1, .3, .3, 1)), self._mk("human_t", i, M.TEXT_VIEW_FACING, h["x"], h["y"], Z[h["floor"]] + 2.2, (0, 0, .6), (1, .6, .6, 1), h["id"])]
            for i, a in enumerate(w.animals): mks.append(self._mk("animal", i, M.CUBE, a["x"], a["y"], Z[a["floor"]] + .2, (.6, .3, .4), (.3, 1, .4, 1) if a["rescued"] else (1, .88, .4, 1)))
            mks += [self._mk("van", 0, M.CUBE, w.van[0], w.van[1], w.van_size[2] / 2, tuple(w.van_size), (.3, .6, 1, 1)), self._mk("van_t", 0, M.TEXT_VIEW_FACING, w.van[0], w.van[1], 3.0, (0, 0, .7), (.6, .8, 1, 1), "VAN (Outside Network Area)")]
            bb = v.beacons; links = []
            for b in bb.values():
                z = Z[b["f"]] + .3
                mks += [self._mk("beacon", b["bid"], M.SPHERE, b["x"], b["y"], z, (.5, .5, .5), (.3, .3, .3, 1) if not b["alive"] else SC.get(v.beacon_states.get(b["bid"], ["", ""])[0], PC[b["prio"]]))]
                stt = (v.beacon_states.get(b["bid"]) or ["DROPPED"])[0]
                if stt == "RESOLVED": mks.append(self._mk("beacon_done", b["bid"], M.SPHERE, b["x"], b["y"], z, (.7, .7, .7), (.2, 1, .4, .45)))
                if b["type"] != "REPEATER": mks.append(self._mk("beacon_t", b["bid"], M.TEXT_VIEW_FACING, b["x"], b["y"], z + .6, (0, 0, .35), (1, 1, 1, 1), f"{b['type']} #{b['bid']} P{b['prio']} [{stt}]"))
                par = bb.get(b["parent"]); px, py, pz = (par["x"], par["y"], Z[par["f"]] + .3) if par else (w.van[0], w.van[1], .3)
                links += [(px, py, pz), (b["x"], b["y"], z)]
            if links: mks.append(self._mk("links", 0, M.LINE_LIST, s=(.05, 0, 0), rgba=(.5, .7, 1, .8), pts=links))
            for k, (role, r) in enumerate(v.robots.items()):
                z = Z[r["f"]] + (1.8 if role == "drone" else .25)
                mks += [self._mk("robot", k, M.CUBE, r["x"], r["y"], z, (.7, .55, .35), RC[role]), self._mk("robot_t", k, M.TEXT_VIEW_FACING, r["x"], r["y"], z + .9, (0, 0, .5), RC[role], role.upper())]
            for k, (role, tr) in enumerate(v.trails.items()):
                if len(tr) > 1: mks.append(self._mk("trail", k, M.LINE_STRIP, s=(.08, 0, 0), rgba=(*RC[role][:3], .9), pts=[(a, b_, Z[f] + .05) for a, b_, f in tr]))
            ma = self._MA(); ma.markers = mks
        self.pub_mk.publish(ma)

# ------------------------------------------------------------------ inside: Gazebo mirror (moves models in the Gazebo world)
class GzPoser:
    """Teleports Gazebo models via /world/<w>/set_pose. Uses the gz Python bindings if present, else the `gz service` CLI."""
    def __init__(self, world, logger):
        self.world, self.log, self.latest, self.cv = world, logger, {}, threading.Condition()
        self.mode, self.node = "cli", None
        for tr, ms in (("gz.transport13", "gz.msgs10"), ("gz.transport14", "gz.msgs11"), ("gz.transport12", "gz.msgs9")):
            try:
                import importlib
                T = importlib.import_module(tr); Pm = importlib.import_module(ms + ".pose_pb2"); Bm = importlib.import_module(ms + ".boolean_pb2")
                self.node, self.Pose, self.Boolean, self.mode = T.Node(), Pm.Pose, Bm.Boolean, "py"; break
            except Exception: continue
        self.log.info(f"Gazebo pose sender mode: {self.mode} ({'fast' if self.mode == 'py' else 'CLI fallback: install python3-gz-transport13 for smoother motion'})")
        threading.Thread(target=self._worker, daemon=True).start()
    def set(self, name, x, y, z, yaw):
        with self.cv: self.latest[name] = (x, y, z, yaw); self.cv.notify()
    def _worker(self):
        while True:
            with self.cv:
                while not self.latest: self.cv.wait()
                batch, self.latest = dict(self.latest), {}
            for name, (x, y, z, yaw) in batch.items():
                try: self._send(name, x, y, z, yaw)
                except Exception as e: self.log.warn(f"set_pose failed for {name}: {e}"); time.sleep(1.0)
    def _send(self, name, x, y, z, yaw):
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        if self.mode == "py":
            req = self.Pose(); req.name = name; req.position.x, req.position.y, req.position.z = x, y, z; req.orientation.z, req.orientation.w = qz, qw
            self.node.request(f"/world/{self.world}/set_pose", req, self.Pose, self.Boolean, 1000)
        else:
            req = f'name: "{name}" position: {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} orientation: {{x: 0 y: 0 z: {qz:.4f} w: {qw:.4f}}}'
            subprocess.run(["gz", "service", "-s", f"/world/{self.world}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                            "--timeout", "1000", "--req", req], capture_output=True, timeout=5)

class GazeboBridgeNode(Node):
    def __init__(self):
        super().__init__("gz_bridge")
        p = _params(self, world="living_map_fire")
        self.w = World(); self.poser, self.prev, self.assigned = GzPoser(p["world"], self.get_logger()), {}, {}
        self.pool = {0: 10, 1: 14, 2: 10, 3: 30}; self.used = {k: 0 for k in self.pool}
        self.create_subscription(String, "/writer/pose", lambda m: self.on_robot("writer", m), 10)
        for r in ROLES: self.create_subscription(String, f"/responder/{r}/pose", lambda m, r=r: self.on_robot(r, m), 10)
        self.create_subscription(String, "/beacon_tx", self.on_tx, 50); self.create_subscription(String, "/beacon_destroyed", self.on_dead, 10); self.create_subscription(String, "/sim/truth", self.on_truth, 2)
        self.gone = set()
        self.get_logger().info(f"Gazebo mirror ready for world '{p['world']}'.")
    def on_robot(self, name, m):
        d = json.loads(m.data); x, y = d["true"]; f = d.get("f", 0); z = self.w.z[f] + (1.8 if name == "drone" else 0.25)
        self.poser.set(name, x, y, z, d.get("yaw", 0.0))
    def on_tx(self, m):
        d = json.loads(m.data); k = d["prio"]
        if self.used[k] < self.pool[k]:
            name = f"bcn_p{k}_{self.used[k]}"; self.used[k] += 1; self.assigned[d["bid"]] = name; self.poser.set(name, d["true"][0], d["true"][1], self.w.z[d["floor"]] + 0.18, 0.0)
    def on_dead(self, m):
        name = self.assigned.get(json.loads(m.data)["bid"])
        if name: self.poser.set(name, 0.0, 0.0, -20.0, 0.0)
    def on_truth(self, m):
        d = json.loads(m.data)
        for i, I in enumerate(d["fires"]):
            if I < 0.1 and ("fire", i) not in self.gone: self.gone.add(("fire", i)); self.poser.set(f"fire_{i}", 0.0, 0.0, -30.0, 0.0)
        for i, I in enumerate(d.get("gas", [])):
            if I < 0.05 and ("gas", i) not in self.gone: self.gone.add(("gas", i)); self.poser.set(f"gas_cloud_{i}", 0.0, 0.0, -30.0, 0.0)
        for kind, lst, key in (("person", d["humans"], "humans"), ("animal", d["animals"], "animals")):
            for i, resc in enumerate(lst):
                if resc and (kind, i) not in self.gone: self.gone.add((kind, i)); self.poser.set(f"{kind}_{i}", 0.0, 0.0, -30.0, 0.0)

def _run(node):
    """rclpy ends the whole process when ONE callback raises. Keep the node alive, log the full cause, and carry on (a dead van = no beacon ever reaches the Command Post)."""
    import traceback
    while rclpy.ok():
        try: rclpy.spin(node)
        except KeyboardInterrupt: break
        except Exception: node.get_logger().error("A CALLBACK CRASHED (the node keeps running). Cause:\n" + traceback.format_exc())

def _spin(cls, setup=None):
    rclpy.init(); n = cls()
    if setup: setup(n)
    try: _run(n)
    finally: n.destroy_node(); rclpy.try_shutdown()

def _spin_role(role):
    rclpy.init(); n = ResponderNode(role)
    try: _run(n)
    finally: n.destroy_node(); rclpy.try_shutdown()

def main_writer(): _spin(WriterNode)
def main_mesh(): _spin(MeshNode)
def main_gateway(): _spin(GatewayNode)
def main_command_post(): _spin(CommandPostNode)
def main_ambulance(): _spin_role("ambulance")
def main_firetruck(): _spin_role("firetruck")
def main_drone(): _spin_role("drone")
def main_sim_view(): _spin(SimViewNode)
def main_gz_bridge(): _spin(GazeboBridgeNode)
