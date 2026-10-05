"""Whole system in one process (no ROS): Writer -> beacon mesh -> van gateway -> Command Post -> ambulance / fire truck / drone.
Used by the standalone browser mode, by the metrics runs and by the tests."""
import random
from .sim_core import World, dist, CH
from .geo import Anchor
from .beacon_protocol import make_beacon, pack
from .agents import WriterAgent, Mesh, GatewayCore, StationLink
from .command_post import CommandPost, ROLES
from .roles import Responder
from .viewstate import ViewState, grid_rows
from .beacon_protocol import unpack

def blind_baseline(map_path=None, max_t=1500.0):
    w = World(map_path); link = StationLink(lambda m, n: None)
    ex = Responder(w, "ambulance", None, Anchor(**w.anchor), link, use_beacons=False); t = 0.0
    while not ex.done and t < max_t: ex.tick(0.5, t); link.tick(t); t += 0.5
    return ex.result or dict(success=False, time=max_t, reason="timeout")

class Simulation:
    def __init__(self, fault="none", auto=True, map_path=None, view=None, verbose=False, seed=7, uplink="immediate", collect=1.0, nav="direct"):
        self.map_path, self.fault, self.auto, self.verbose = map_path, fault, auto, verbose
        self.w = World(map_path); self.anchor = Anchor(**self.w.anchor)
        self.seed, self.nav = seed, nav; self.wr = WriterAgent(self.w, seed=seed, kill_at=self.w.fault_t.get("writer_lost", 420.0) if fault == "writer_lost" else None); self.mesh = Mesh(self.w, seed=seed + 4)
        self.cp = CommandPost(self.anchor, collect_window=collect, corr_dist=self.w.thr.get("corr_dist", 8.0)); self.link = StationLink(self._station)
        self.gw = GatewayCore(self.anchor, self._send, immediate=(uplink == "immediate"))
        self.t, self.resp, self.forged, self.wl, self.final, self.timeline, self.killed = 0.0, {}, False, 0, False, [], False
        self.rl = {}; self.approved = auto; self._hb = {}
        self.view = view or ViewState(); self.view.reset(); self.view.fault = fault
        self.say("SYSTEM", f"Living Map started on '{self.w.name}' (fault: {fault})")

    def say(self, who, msg):
        self.view.t = self.t; self.view.add_log(who, msg); self.timeline.append((round(self.t, 1), who, msg))
        if self.verbose: print(f"[{self.t:6.1f}s] {who:9s} {msg}")
    def _send(self, records, reason):
        self.cp.ingest(dict(reason=reason, records=records), self.t); self.view.flags["uplink"] = True
        self.say("UPLINK", f"[{reason}] " + ", ".join(r["id"] for r in records))
    def _station(self, m, now):
        self.cp.ingest(dict(reason="STATUS", records=[], statuses=[m]), now)
        self.say("STATION", f"{m['robot']} -> van: {m['kind']} {m['payload'] if m['payload'] else ''}")

    def _beacon_write(self, bid, state, note, t, pos, floor, role):
        ok = self.mesh.write_state(bid, state, role, pos, floor, t)
        if ok: self.view.set_state(bid, state, role)
        self.view.add_flow("WRITE", role, "beacons", f"#{bid} -> {state}" + ("" if ok else " (out of radio range)"))
        return ok

    def _rx(self, deliveries, t):
        for d in deliveries:
            try: self.view.add_flow("RF", "beacons", "van", f"#{unpack(bytes.fromhex(d['hex'])).bid} ({d['hops']} hops, {d['rssi']} dBm)")
            except ValueError: pass
            rec = self.gw.on_rx(d["hex"], d["rssi"], d["hops"], t)
            if rec:
                self.view.flags["gw"] = True
                self.say("GATEWAY", f"{rec['id']} {rec['type']} P{rec['prio']} floor {rec['floor']} | beacon is {rec['hop_text']} | target: {rec['tgt_text']} | GPS {rec['lat']:.6f}, {rec['lon']:.6f} (+/-{rec['sigma_m']} m)")

    def step(self, dt=0.5):
        t, v = self.t, self.view; v.t = t
        if self.wr.done and not getattr(self, "_wr_reported", False): self._wr_reported = True; self.link.send("writer", "EXPLORATION_DONE", {}, t)      # the Writer tells the Command Post (through the van) that exploration is over
        for tx in self.wr.tick(dt, t):
            v.add_beacon(tx["bid"], tx["type"], tx["prio"], tx["true"][0], tx["true"][1], tx["floor"], tx["parent"]); v.set_state(tx["bid"], "DROPPED", "writer"); v.add_flow("DROP", "writer", "beacons", f"#{tx['bid']} {tx['type']} P{tx['prio']}")
            ds = self.mesh.register(tx["hex"], tx["true"], tx["floor"], tx["bid"], t)
            if not ds: self.say("MESH", f"beacon {tx['bid']} not delivered yet (relays keep retrying, mirrors hold a copy)")
            self._rx(ds, t)
        if self.fault == "beacon_destroyed" and not self.killed and t >= self.w.fault_t.get("beacon_destroyed", 150.0):
            kids = {}
            for bid, n in self.mesh.nodes.items(): kids[n["parent"]] = kids.get(n["parent"], 0) + 1
            cand = [b for b, n in self.mesh.nodes.items() if n["alive"] and kids.get(b, 0) >= 1] or [b for b, n in self.mesh.nodes.items() if n["alive"]]      # a relay if there is one, else the oldest beacon
            if cand: b = max(cand, key=lambda b: (kids.get(b, 0), -b)); self.mesh.kill(b, t); v.kill_beacon(b); self.killed = True; self.say("FAULT", f"beacon {b} DESTROYED")
        self._rx(self.mesh.tick(t), t)
        if not self.forged and t >= 30:
            self.forged = True; b = bytearray(pack(make_beacon(99, 1, 1, t, "HUMAN", 0, 0, 0, 1, 0, (5, 5), (0, 0), 0))); b[3] ^= 0xFF
            self.gw.on_rx(bytes(b).hex(), -60, 1, t); self.say("GATEWAY", "forged packet REJECTED (CRC/MAC)")
        self.gw.tick(t); self.link.tick(t)
        while self.wl < len(self.wr.log): self.say("WRITER", self.wr.log[self.wl][1]); self.wl += 1
        r = self.wr.r; v.set_robot("writer", r.pos[0], r.pos[1], r.f, "dead" if self.wr.dead else "mapped" if self.wr.done else "GPS to the door" if self.wr.phase == "gps" else "exploring", self.wr.done)
        v.sensors = self.wr.last; v.known = [grid_rows(r.k.g[f]) for f in range(self.w.nf)]
        for role, m in self.cp.ready(t, 0, self.approved):
            old = self.resp.get(role)
            if old is not None and old.done and old.result: self._record(role, old)           # a new sortie may start in the same step the previous one ends: never lose its result
            self.resp[role] = Responder(self.w, role, m, self.anchor, self.link, seed=self.seed - 4, beacon_cb=self._beacon_write, nav=self.nav); self.rl[role] = 0; self._hb[role] = t; v.flags["dispatched"] = True
            self.say("CMDPOST", f"{role} dispatched: {len(m['targets'])} target(s), {len(m['hazards'])} hazard(s)")
        for u in self.cp.pop_outbox():                                   # CP -> van -> robot: ORDER_UPDATE (replanning). Robots never message each other.
            ag = self.resp.get(u["role"]); v.add_flow("UPDATE", "cp", u["role"], "ORDER_UPDATE")
            if ag is not None and not ag.done and ag.apply_update(u["payload"], t): self.say("CMDPOST", f"{u['role']} order updated (via the van): {', '.join(sorted(u['payload']))}")
        v.awaiting = self.cp.awaiting(t)
        for role, ag in list(self.resp.items()):
            if t - self._hb.get(role, t) >= 10.0 and not ag.done: self._hb[role] = t; self.link.send(role, "STATUS", dict(active=True, state=ag.state), t)      # heartbeat through the van
            ag.tick(dt, t)
            while self.rl[role] < len(ag.log): self.say(role.upper(), ag.log[self.rl[role]][1]); self.rl[role] += 1
            v.set_robot(role, ag.r.pos[0], ag.r.pos[1], ag.r.f, ag.state, ag.done, ag.battery)
            if ag.done and ag.result and ag.result.get("reason") == "returned" and role in self.resp and getattr(ag, "_reported", False) is False:
                ag._reported = True
        for role, ag in list(self.resp.items()):                         # a robot that returned may be sent again (new sortie)
            if ag.done and self.cp.dispatched[role] is None: self.resp.pop(role); self.rl.pop(role, None); self._record(role, ag)
        v.set_truth(self.w); v.board = self.cp.dashboard_state(t, True)["robots"]
        if not self.final and (self.wr.done or self.wr.dead) and t > 30 and not self.resp and getattr(self, "done_results", None) and all(self.cp.dispatched[r] is None for r in ROLES) and not any(self.cp.targets_for(r) for r in ROLES):
            self.final = True; self._final()
        self.t += dt

    def _record(self, role, ag):
        if getattr(ag, "_recorded", False): return
        ag._recorded = True; self.done_results = getattr(self, "done_results", {}); self.done_results.setdefault(role, []).append(ag.result)
    def _final(self):
        blind = blind_baseline(self.map_path); res = getattr(self, "done_results", {}); w = self.w
        amb = [x for x in res.get("ambulance", []) if x and x.get("first_target_time")]
        errs = []
        for r in self.cp.records.values():
            if r["type"] in ("HUMAN", "ANIMAL", "FIRE"):
                pool = {"HUMAN": w.humans, "ANIMAL": w.animals, "FIRE": w.fires}[r["type"]]
                errs += [min(dist(r["tlocal"], (o["x"], o["y"])) for o in pool if o["floor"] == r["floor"])] if any(o["floor"] == r["floor"] for o in pool) else []
        hum = [v for x in res.get("ambulance", []) if x for k, v in x.get("arrivals", {}).items() if k.endswith(":HUMAN")]
        with_b = min(hum) if hum else min((x["first_target_time"] for x in amb), default=None)
        abs_h = [x["t_start"] + v for x in res.get("ambulance", []) if x for k, v in x.get("arrivals", {}).items() if k.endswith(":HUMAN")]     # end to end: from the start of the mission
        abs_b = round(min(abs_h), 1) if abs_h else None
        self.view.metrics = dict(with_beacons_s=with_b, first_person_abs_s=abs_b, blind_s=round(blind.get("time", 0), 1) if blind else None,
            gain_pct=round(100 * (1 - with_b / blind["time"])) if with_b and blind and blind.get("success") and blind.get("time") else None,
            position_error_m=round(sum(errs) / len(errs), 2) if errs else None, position_error_max_m=round(max(errs), 2) if errs else None,
            beacons=len(self.wr.beacons), repeaters=self.wr.repeaters, delivery=f"{self.mesh.delivered}/{self.mesh.sent}", rejected=self.gw.rejected,
            fires_out=sum(1 for f in w.fires if f["I"] < 0.1), fires=len(w.fires), rescued=sum(h["rescued"] for h in w.humans + w.animals), people=len(w.humans) + len(w.animals),
            skipped=sorted({s for m in self.cp.missions.values() for s in m.get("skipped", [])}) or [r["id"] for r in self.cp.records.values() if r["type"] == "HUMAN" and r["human_needed"]],
            robots={k: [dict(time=x["time"], dist=x["distance"], ok=x["success"], reached=x["reached"], battery=x["battery_left"]) for x in v if x] for k, v in res.items()})
        m = self.view.metrics; self.say("METRICS", f"Executor reaches a person {m['with_beacons_s']} s after leaving the van vs {m['blind_s']} s blind (end to end {m.get('first_person_abs_s')} s); fires out {m['fires_out']}/{m['fires']}; people rescued {m['rescued']}/{m['people']}")

def run(fault="none", map_path=None, max_t=1400.0, verbose=False, dt=0.5, seed=7, nav="direct"):
    s = Simulation(fault, True, map_path, verbose=verbose, seed=seed, nav=nav)
    while s.t < max_t and not s.final: s.step(dt)
    return s
