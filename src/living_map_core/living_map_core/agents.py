"""Writer robot, radio mesh, van gateway, station link. Plain Python; ROS nodes are thin wrappers."""
import math, random
from .sim_core import *
from .beacon_protocol import *
from .geo import Anchor
from .adaptive_queue import AdaptiveQueue

# ----------------------------------------------------------------- event detection
class Detector:
    """2-of-3 sensor corroboration + persistence window (rejects the steam pipe)."""
    def __init__(self, persist=3, thr=None):
        self.persist, self.cnt, self.seen, self.steam_rejected = persist, {}, set(), 0
        t = dict(fire_air=70, fire_thermal=150, fire_thermal_alone=None, gas_vote=50, gas_event=150); t.update(thr or {}); self.t = t      # per-map thresholds: a 140 C lamp in a 7 m arena is not a 400 C building fire
    def _hit(self, key, cond):
        if cond: self.seen.add(key); self.cnt[key] = self.cnt.get(key, 0) + 1
        return cond and self.cnt[key] == self.persist
    def update(self, s, floor=0):
        self.seen = set(); ev = []
        t = self.t; votes = (s["temp"] > t["fire_air"]) + (s["thermal_max"] > t["fire_thermal"]) + (s["gas"] > t["gas_vote"])
        alone = t["fire_thermal_alone"] is not None and s["thermal_max"] > t["fire_thermal_alone"]       # camera sees something much hotter than a person or a hot mug
        if self._hit(("FIRE", floor), (s["temp"] > t["fire_air"] and votes >= 2) or alone): ev.append(dict(type="FIRE", hot=s["hot"][0] if s["hot"] else None))
        if self._hit(("STEAM", floor), s["temp"] > t["fire_air"] and votes < 2 and not alone): self.steam_rejected += 1; ev.append(dict(type="STEAM_REJECTED"))
        if self._hit(("GAS", floor), s["gas"] > t["gas_event"]): ev.append(dict(type="GAS"))
        for b in s["blobs"]:
            if b["conf"] >= 0.6 and self._hit((b["kind"], floor, round(b["x"] / 2), round(b["y"] / 2)), True): ev.append(dict(type=b["kind"], blob=(b["x"], b["y"])))
        for k in list(self.cnt):
            if k not in self.seen: self.cnt[k] = 0
        return ev

ROOT = dict(bid=0, pos_est=(0.0, 0.0), hq=0, floor=0, hops=0, true=None)   # the entrance anchor: heading 0 = into the building

# ----------------------------------------------------------------- Writer robot (tracked: climbs stairs)
class WriterAgent:
    def __init__(self, world, seed=7, budget=44, speed=1.0, drift_rate=0.01, kill_at=None, search_radius=60.0, gps_sigma=0.8, gps_approach=True):
        self.w, self.rng, self.radius, self.gps_sigma = world, random.Random(seed), search_radius, gps_sigma
        start = tuple(getattr(world, "writer_start", (-1.0, 0.0))) if gps_approach else (-1.0, 0.0)
        if world.val(0, start[0], start[1]) != FREE or start == (-1.0, 0.0): start = (-1.0, 0.0); self.phase = "lidar"      # no free start outside the door: begin at the door as before
        else: self.phase = "gps"                                                                                         # OUTSIDE the building: GNSS works here
        self.r = Robot(world, start, 0, self.rng, CAPS_WALK, speed, drift_rate); self._gps_said = False; self.home = start
        if self.phase == "gps": self.r.yaw = math.atan2(-start[1], -start[0])
        self.det, self.beacons, self.budget, self.next_id, self.by = Detector(thr=getattr(world, "thr", None)), [], budget, 1, {0: ROOT}
        self.log, self.done, self.dead, self.kill_at, self.repeaters, self.last = [], False, False, kill_at, 0, None
        self.gas_track = None

    def _say(self, t, msg): self.log.append((t, msg))
    def _gps_approach(self, dt, t):
        """OUTSIDE the building the Writer has GNSS: it drives from the van to the entrance door (the origin of the private frame) steering by noisy GPS fixes
        (sigma ~0.8 m, an assumed value); in the last 2.5 m the LiDAR finds the door gap. At the threshold GPS is lost: from there only LiDAR + odometry (drift)."""
        r = self.r; r.scan(); d = math.hypot(r.pos[0], r.pos[1])
        if not self._gps_said:
            self._gps_said = True; self._say(t, f"GPS approach: leaving the van, driving {d:.1f} m to the entrance door by GNSS (sigma {self.gps_sigma} m)")
        if d <= 0.35:
            a = self.w.anchor; self.phase = "lidar"; r.drift = [0.0, 0.0]
            self._say(t, f"at the entrance door: GPS is LOST inside; the private frame is anchored on the last GNSS fix ({a['lat0']:.5f}, {a['lon0']:.5f}), from here LiDAR + odometry only"); return
        sig = self.gps_sigma if d > 2.5 else 0.0
        fx, fy = r.pos[0] + self.rng.gauss(0, sig), r.pos[1] + self.rng.gauss(0, sig)
        ang = math.atan2(-fy, -fx); step = min(r.speed * dt, d)
        nx, ny = r.pos[0] + step * math.cos(ang), r.pos[1] + step * math.sin(ang)
        if self.w.val(0, nx, ny) != FREE:                                       # a fence or wall in the way: use the true door direction (LiDAR avoidance)
            ang = math.atan2(-r.pos[1], -r.pos[0]); nx, ny = r.pos[0] + step * math.cos(ang), r.pos[1] + step * math.sin(ang)
        r.pos = [nx, ny]; r.yaw = ang; r.travelled += step; r.drift = [0.0, 0.0]          # GPS gives an absolute fix: no odometry drift accumulates outside
    def _head_home(self, t):
        r = self.r; hx, hy = getattr(self, "home", (-3.0, -2.0))
        if self.w.val(0, hx, hy) != FREE: hx, hy = -1.0, 0.0
        if r.f == 0 and math.hypot(r.pos[0] - hx, r.pos[1] - hy) < 0.4: self.parked = True; self._say(t, "back at the van: parked"); return
        p = plan(r.k, r.cell(), goal=(0,) + tuple(self.w.to_cell(hx, hy)), caps=r.caps, unk_v="all", extra=r.extra)
        if p: r.path = p
        else: self.parked = True
    def _near(self, typ, tgt, radius, floor):
        return any(b["type"] == typ and b["floor"] == floor and dist(b["target_est"], tgt) < radius for b in self.beacons)

    def _rubble_on_ray(self, b):
        a = self.r.pos; n = max(1, int(dist(a, b) / 0.25)); g = self.r.k.g[self.r.f]
        for k in range(1, n):
            x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
            i, j = self.w.to_cell(x, y)
            if self.w.inb(i, j) and g[j][i] == RUBBLE: return (x, y)
        return None

    def _parent(self):
        """Best uplink: fewest hops among nodes with a usable link (>= -80 dBm) within 12 m (hop offsets are limited to +-12.7 m)."""
        pos, f, est = tuple(self.r.pos), self.r.f, self.r.est
        cands = [(0, 0, self.w.rssi(pos, f, self.w.van, 0), dist(est, (0.0, 0.0)))] if f == 0 else []      # the van antenna is on floor 0
        # the beacon chain must be a WALKABLE route: same floor, or a stairs beacon (the only place where the floor changes)
        cands += [(b["bid"], b["hops"], self.w.rssi(pos, f, b["true"], b["floor"]), dist(est, b["pos_est"])) for b in self.beacons
                  if b["floor"] == f or (b["type"] == "STAIRS" and dist(est, b["pos_est"]) < 8.0)]
        good = [c for c in cands if c[2] >= -72 and c[3] <= 12.0]              # comfortable link: no relay needed yet
        if good: c = min(good, key=lambda c: (c[1], -c[2])); return c[0], c[1], True
        usable = [c for c in cands if c[2] >= -80 and c[3] <= 12.0]            # still works, but the margin is thin: drop a relay now (keeps a dense, self-healing mesh)
        if usable: c = min(usable, key=lambda c: (c[1], -c[2])); return c[0], c[1], False
        near = [c for c in cands if c[3] <= 12.0] or cands or [(0, 0, -99.0, 0.0)]
        c = max(near, key=lambda c: c[2]); return c[0], c[1], False

    def _emit(self, t, typ, prio, flags, n, target_est, temp, gas):
        bid = self.next_id; self.next_id += 1
        est, f = self.r.est, self.r.f; parent, ph, _ = self._parent(); P = self.by[parent]
        hq = quant_heading(self.r.yaw); hop = to_body(P["pos_est"], P["hq"], est)
        tgt = to_body(est, hq, target_est) if target_est is not None else (0.0, 0.0)
        b = make_beacon(bid, 1, 1, t, typ, prio, flags, f, n, hq, hop, tgt, parent, temp, gas)
        rec = dict(bid=bid, type=typ, prio=prio, floor=f, pos_est=est, true=tuple(self.r.pos), hq=hq, target_est=target_est if target_est is not None else est, hops=ph + 1, parent=parent)
        self.beacons.append(rec); self.by[bid] = rec; self.budget -= 1
        return dict(hex=pack(b).hex(), true=list(self.r.pos), floor=f, bid=bid, type=typ, prio=prio, parent=parent, hq=hq)

    def tick(self, dt, t):
        tx = []
        if self.dead: return tx
        if self.kill_at is not None and t >= self.kill_at:
            self.dead = True; self._say(t, "WRITER LOST (power/structure failure) - memory already lives in the beacons"); return tx
        if self.phase == "gps": self._gps_approach(dt, t); return tx
        r = self.r; r.scan(); s = self.w.sense(r.pos, r.f, self.rng); r.mark_hot(s["temp"]); est, dr, f = r.est, r.drift, r.f
        for ev in self.det.update(s, f):
            typ = ev["type"]
            if typ == "STEAM_REJECTED":
                self._say(t, f"steam-pipe false positive REJECTED (temp {s['temp']:.0f}C, only 1 of 3 sensors agree)"); continue
            if typ == "FIRE":
                tgt = (ev["hot"][0] + dr[0], ev["hot"][1] + dr[1]) if ev["hot"] else est
                if self._near("FIRE", tgt, 4.0, f) or self.budget <= 0: continue
                tx.append(self._emit(t, "FIRE", 1, 0, 1, tgt, s["temp"], s["gas"])); self._say(t, f"FIRE beacon {tx[-1]['bid']} P1 (floor {f})")
            elif typ == "GAS":
                if self._near("GAS", est, 4.0, f) or self.budget <= 0 or self.gas_track: continue
                self.gas_track = dict(t0=t, f=f, best=s["gas"], pos=est)            # follow the plume: the beacon must point at its PEAK, not at where it first crossed the threshold
            else:
                bx, by = ev["blob"]; tgt = (bx + dr[0], by + dr[1])
                if self._near(typ, tgt, 2.5, f) or (self.budget <= 0 and typ != "HUMAN"): continue
                flags, prio = 0, 2
                if typ == "HUMAN":
                    prio = 0 if (any(dist(h, (bx, by)) < 3.0 for h in s["hot"]) or s["gas"] > 500) else 1
                    rub = self._rubble_on_ray(ev["blob"])
                    if rub is not None:
                        flags = FLAG_HUMAN_NEEDED; rt = (rub[0] + dr[0], rub[1] + dr[1])
                        if not self._near("DEBRIS", rt, 3.0, f):
                            tx.append(self._emit(t, "DEBRIS", 3, FLAG_HUMAN_NEEDED, 0, rt, s["temp"], s["gas"])); self._say(t, f"DEBRIS beacon {tx[-1]['bid']} : heavy rescue needed")
                tx.append(self._emit(t, typ, prio, flags, 1, tgt, s["temp"], s["gas"]))
                self._say(t, f"{typ} beacon {tx[-1]['bid']} P{prio} (floor {f})" + (" [human_intervention_required]" if flags else ""))
        gt = self.gas_track
        if gt:
            if s["gas"] > gt["best"]: gt["best"], gt["pos"] = s["gas"], est
            if f != gt["f"] or (s["gas"] < 0.7 * gt["best"] and t - gt["t0"] > 2.0) or t - gt["t0"] > 25.0:
                prio = 0 if gt["best"] > 500 else 1; self.gas_track = None
                tx.append(self._emit(t, "GAS", prio, 0, 1, gt["pos"], s["temp"], gt["best"])); self._say(t, f"GAS beacon {tx[-1]['bid']} P{prio} (peak {gt['best']:.0f} ppm)")
        # landmarks: stairs and atrium shafts are what lets other robots change floor
        ci, cj = self.w.to_cell(*r.pos); k = r.k.g[f]
        for di in range(-8, 9):
            for dj in range(-8, 9):
                a, b = ci + di, cj + dj
                if self.w.inb(a, b) and k[b][a] in (STAIRS, VOID) and self.budget > 0:
                    cx, cy = self.w.center(a, b); kind = 0 if k[b][a] == STAIRS else 2
                    tgt = (cx + dr[0], cy + dr[1])
                    if any(x["type"] == "STAIRS" and x["floor"] == f and dist(x["target_est"], tgt) < 4.0 for x in self.beacons): continue
                    tx.append(self._emit(t, "STAIRS", 3, 0, kind, tgt, s["temp"], s["gas"])); self._say(t, f"STAIRS beacon {tx[-1]['bid']} ({'stairs' if kind == 0 else 'atrium shaft'}, floor {f})")
        if self.budget > 0 and not self._parent()[2]:
            tx.append(self._emit(t, "REPEATER", 3, 0, 0, None, s["temp"], s["gas"])); self.repeaters += 1
            self._say(t, f"link weak -> REPEATER {tx[-1]['bid']} dropped")
        self.last = dict(temp=round(s["temp"], 1), gas=round(s["gas"], 1), thermal=s["thermal_max"], steam=self.det.steam_rejected)
        if not r.path and not r.choose_frontier(self.radius):
            if not self.done: self.done = True; self._say(t, f"exploration complete, {len(self.beacons)} beacons used")
            if not getattr(self, "parked", False): self._head_home(t)      # nothing left to explore: drive back to the van and park there (not against the fence)
        r.follow(r.speed * dt)
        return tx

# ----------------------------------------------------------------- RF mesh (radio physics + gossip mirrors)
class Mesh:
    def __init__(self, world, seed=11, retries=3):
        self.w, self.rng, self.retries = world, random.Random(seed), retries
        self.nodes, self.lost_reported, self.sent, self.delivered, self.undelivered, self.retry_t = {}, set(), 0, 0, set(), 0.0

    def _hop_ok(self, a, fa, b, fb):
        p = self.w.per(self.w.rssi(a, fa, b, fb)); return any(self.rng.random() >= p for _ in range(self.retries))
    def _depth(self, bid):
        d, pid = 0, bid
        while pid and pid in self.nodes and d < 80: pid = self.nodes[pid]["parent"]; d += 1
        return d
    def _next_hop(self, node):
        """Parent if alive; otherwise self-heal: best alive neighbour that is closer (in tree depth) to the van."""
        pid = node["parent"]
        if pid == 0: return (self.w.van, 0, 0)
        if pid in self.nodes and self.nodes[pid]["alive"]: return (self.nodes[pid]["pos"], self.nodes[pid]["floor"], pid)
        my, best = self._depth(node["bid"]), None
        for oid, o in self.nodes.items():
            if not o["alive"] or oid == node["bid"] or self._depth(oid) >= my: continue
            rs = self.w.rssi(node["pos"], node["floor"], o["pos"], o["floor"])
            if rs >= -88 and (best is None or rs > best[0]): best = (rs, oid, o)
        rsv = self.w.rssi(node["pos"], node["floor"], self.w.van, 0) if node["floor"] == 0 else -999.0
        if rsv >= -88 and (best is None or rsv > best[0] + 6): return (self.w.van, 0, 0)
        return (best[2]["pos"], best[2]["floor"], best[1]) if best else (self.w.van, 0, 0)

    def register(self, hexs, true, floor, bid, now):
        """A new beacon was dropped and transmits. Returns a list of deliveries reaching the van antenna
        (may include ancestors whose packets were lost earlier: relays store-and-forward)."""
        pkt = bytes.fromhex(hexs); b = unpack(pkt)
        node = self.nodes.setdefault(bid, dict(bid=bid, pos=tuple(true), floor=floor, parent=b.parent, alive=True, store={}, killed_at=None))
        node["store"][bid] = pkt
        for oid, o in self.nodes.items():                     # gossip: neighbours keep a mirror copy
            if oid != bid and o["alive"] and self.w.rssi(o["pos"], o["floor"], node["pos"], node["floor"]) >= -90:
                o["store"][bid] = pkt
                if oid in o["store"]: node["store"][oid] = o["store"][oid]
        return self._try(bid)

    def _try(self, bid):
        out = []; node = self.nodes[bid]; d = self._deliver(self.nodes[bid]["store"][bid], node)
        if d is None: self.undelivered.add(bid); return out
        self.undelivered.discard(bid)
        chain, pid = [], node["parent"]
        while pid and pid in self.nodes and pid in self.undelivered: chain.append(pid); pid = self.nodes[pid]["parent"]
        for a in reversed(chain):                             # relays forward the stored copies of lost ancestors first
            da = self._deliver(self.nodes[a]["store"][a], self.nodes[a])
            if da: self.undelivered.discard(a); out.append(da)
        return out + [d]

    def _deliver(self, pkt, node):
        self.sent += 1; cur, hops, worst = node, 0, 0.0
        while True:
            npos, nf, pid = self._next_hop(cur); rs = self.w.rssi(cur["pos"], cur["floor"], npos, nf); worst = min(worst, rs) if hops else rs
            if not self._hop_ok(cur["pos"], cur["floor"], npos, nf): return None
            hops += 1
            if pid == 0: break
            cur = self.nodes[pid]
        self.delivered += 1; return dict(hex=pkt.hex(), rssi=round(worst, 1), hops=hops)

    def write_state(self, bid, state, by, pos, floor, now):
        """A robot standing near a beacon rewrites its mutable STATE byte over short-range radio (the signed 32-byte report is never changed).
        Works only if the robot is within radio range of that beacon."""
        n = self.nodes.get(bid)
        if not n or not n["alive"]: return False
        if self.w.rssi(tuple(pos), floor, n["pos"], n["floor"]) < -90: return False
        n["state"], n["state_by"], n["state_t"] = state, by, now
        return True

    def read_state(self, bid, pos, floor):
        """A robot in radio range reads the STATE stored in a beacon (shared memory left by other executors). None if out of range."""
        n = self.nodes.get(bid)
        if not n or not n["alive"] or self.w.rssi(tuple(pos), floor, n["pos"], n["floor"]) < -90: return None
        return n.get("state")

    def kill(self, bid, now):
        if bid in self.nodes and self.nodes[bid]["alive"]: self.nodes[bid]["alive"] = False; self.nodes[bid]["killed_at"] = now

    def tick(self, now):
        """Missing heartbeat -> a neighbour re-broadcasts the mirrored record as BEACON_LOST (silence is data)."""
        out = []
        if now - self.retry_t >= 5.0:                          # ARQ: retry packets that never reached the van
            self.retry_t = now
            for bid in list(self.undelivered):
                n = self.nodes[bid]
                if n["alive"]: out += self._try(bid)
                else:                                              # the beacon is dead: a neighbour holding a mirror forwards it
                    for oid, o in self.nodes.items():
                        if o["alive"] and bid in o["store"] and self.w.rssi(o["pos"], o["floor"], n["pos"], n["floor"]) >= -90:
                            d = self._deliver(o["store"][bid], o)
                            if d: self.undelivered.discard(bid); out.append(d); break
        for bid, n in list(self.nodes.items()):
            if not n["alive"] and bid not in self.lost_reported and now - n["killed_at"] >= 15:
                nb = [(self.w.rssi(o["pos"], o["floor"], n["pos"], n["floor"]), oid, o) for oid, o in self.nodes.items() if o["alive"] and oid != bid]
                nb = [x for x in nb if x[0] >= -90 and bid in x[2]["store"]]
                if not nb: continue
                _, oid, o = max(nb, key=lambda x: x[0]); self.lost_reported.add(bid)
                b = unpack(o["store"][bid]); b.bid, b.type, b.prio, b.flags, b.seq = 1000 + bid, TYPES["BEACON_LOST"], 3, 0, 1
                d = self._deliver(pack(b), o)
                if d: out.append(d)
        return out

    def air(self, pos, floor):
        """Records a robot can read by listening: alive beacons in range + mirrored copies (including of dead beacons)."""
        got = {}
        for oid, o in self.nodes.items():
            if o["alive"] and self.w.rssi(o["pos"], o["floor"], pos, floor) >= -90:
                for rid, pkt in o["store"].items():
                    if pkt: got[rid] = pkt
        return got

# ----------------------------------------------------------------- the Van: gateway (Outside Network Area) + station link
class GatewayCore:
    """Receives beacons at the van antenna, rebuilds positions by chaining the egocentric hops, converts to GPS, queues."""
    def __init__(self, anchor, send, interval=120.0, max_len=8, theta=20.0, immediate=True):
        self.anchor, self.q = anchor, AdaptiveQueue(send, interval, max_len, theta, immediate)
        self.by_bid, self.seen, self.waiting, self.rejected, self.dups, self.received, self.log = {}, set(), {}, 0, 0, 0, []

    def _ancestors(self, rec):
        out, pid = set(), rec["parent"]
        while pid and pid in self.by_bid and pid not in out: out.add(pid); pid = self.by_bid[pid]["parent"]
        return out

    def on_rx(self, hexs, rssi, hops, now):
        try: b = unpack(bytes.fromhex(hexs))
        except ValueError as e: self.rejected += 1; self.log.append((now, f"REJECTED packet: {e}")); return None
        if (b.bid, b.seq) in self.seen: self.dups += 1; return None
        if b.parent == 0: ppos, phq = (0.0, 0.0), 0
        elif b.parent in self.by_bid: ppos, phq = tuple(self.by_bid[b.parent]["local"]), self.by_bid[b.parent]["hq"]
        else: self.waiting.setdefault(b.parent, []).append((hexs, rssi, hops, now)); return None      # parent not received yet
        self.seen.add((b.bid, b.seq)); self.received += 1
        pos = from_body(ppos, phq, b.hop_f / 10.0, b.hop_r / 10.0); tpos = from_body(pos, b.heading, b.tgt_f / 10.0, b.tgt_r / 10.0)
        lat, lon = self.anchor.to_wgs84(*pos); tlat, tlon = self.anchor.to_wgs84(*tpos)
        pname = "the entrance" if b.parent == 0 else f"beacon #{b.parent}"
        rec = dict(bid=b.bid, id=f"BCN_{b.bid:03d}", seq=b.seq, type=TYPE_NAMES[b.type], prio=b.prio, flags=b.flags, human_needed=bool(b.flags & FLAG_HUMAN_NEEDED),
                   floor=b.floor, n=b.n, local=list(pos), tlocal=list(tpos), hq=b.heading, lat=lat, lon=lon, tlat=tlat, tlon=tlon,
                   sigma_m=round(self.anchor.sigma(*pos), 2), parent=b.parent, t0=b.t0, temp=b.temp_dc / 10.0, gas=b.gas, rssi=rssi, hops=hops, rx_t=now,
                   hop_text=f"{describe(b.hop_f / 10.0, b.hop_r / 10.0)} of {pname}", tgt_text=describe(b.tgt_f / 10.0, b.tgt_r / 10.0) + " of this beacon")
        if rec['type'] in ('FIRE', 'GAS') and any(o['type'] == 'HUMAN' and o['prio'] == 0 and o['floor'] == rec['floor'] and dist(o['tlocal'], rec['tlocal']) < 6.0 for o in self.by_bid.values()): rec['escalate'] = True
        self.by_bid[b.bid] = rec; anc = self._ancestors(rec)
        def ctx(r, p): return p["bid"] in anc or (p["type"] in ("FIRE", "GAS") and p["floor"] == r["floor"] and dist(p["tlocal"], r["tlocal"]) < 6.0)
        self.q.push(rec, now, ctx)
        for w in self.waiting.pop(b.bid, []): self.on_rx(*w)
        return rec
    def tick(self, now): self.q.tick(now)

class StationLink:
    """Robot <-> Van link. STAR topology: a robot only ever talks to the van, never to another robot.
    The real radio protocol is TO BE SPECIFIED; for now an ideal channel with a fixed latency."""
    KINDS = ("STATUS", "FIRE_OUT", "RESCUED", "RETURNED", "LOW_BATTERY", "ARRIVED", "UNREACHABLE", "BEACON_UPDATE", "HAZARD_CLEARED", "EXPLORATION_DONE")
    def __init__(self, sink, latency=1.0): self.sink, self.latency, self.q, self.log = sink, latency, [], []
    def send(self, robot, kind, payload, now):
        assert kind in self.KINDS, kind
        self.q.append((now + self.latency, dict(robot=robot, kind=kind, payload=payload, t=round(now, 1))))
    def tick(self, now):
        due = [m for t, m in self.q if t <= now]; self.q = [(t, m) for t, m in self.q if t > now]
        for m in due: self.log.append(m); self.sink(m, now)
