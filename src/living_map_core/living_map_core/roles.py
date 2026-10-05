"""The three responder robots that leave the van: ambulance (rescue), fire truck, drone. They only know what the briefing gave them
(beacon chain, stairs hints, hazards) plus what their own sensors see. They talk ONLY to the van (StationLink), never to each other."""
import math, random, itertools
from .sim_core import *

ROLE_CFG = {
    "ambulance": dict(caps=CAPS_WALK, speed=1.4, start=(-4.0, -2.6), standoff=1.3),
    "firetruck": dict(caps=CAPS_WHEELS, speed=1.2, start=(-4.0, 2.6), standoff=4.5),
    "drone": dict(caps=CAPS_FLY, speed=3.0, start=(-2.2, 2.6), standoff=3.0, battery=420.0, drops=6),
}

def corridor(world, pts, radius=4.0):
    """Cells (floor,i,j) near the inherited beacon trail where unknown space may be crossed (everything else unknown = blocked)."""
    cells, rc = set(), int(radius / world.cell)
    def add(f, x, y):
        ci, cj = world.to_cell(x, y)
        for di in range(-rc, rc + 1):
            for dj in range(-rc, rc + 1):
                if di * di + dj * dj <= rc * rc and world.inb(ci + di, cj + dj): cells.add((f, ci + di, cj + dj))
    for k, (x, y, f) in enumerate(pts):
        add(f, x, y)
        if k and pts[k - 1][2] == f:
            (px, py, _) = pts[k - 1]; n = max(1, int(math.hypot(x - px, y - py) / 1.0))
            for s in range(1, n): add(f, px + (x - px) * s / n, py + (y - py) * s / n)
    return cells

class Responder:
    def __init__(self, world, role, mission, anchor, link, seed=3, use_beacons=True, heat_budget=2500.0, beacon_cb=None, beacon_read=None, nav="direct"):
        self.nav = nav          # "direct": A* straight to the target, trail = cheap safe hint; "trail": legacy (follow every Writer waypoint, unknown only near the trail)
        cfg = ROLE_CFG[role]; self.w, self.role, self.anchor, self.link, self.use = world, role, anchor, link, use_beacons; self.beacon_cb, self._ip = beacon_cb, False; self.beacon_read, self.hold_t0, self.hold_logged, self.released = beacon_read, None, False, False
        self.rng = random.Random(seed + sum(map(ord, role))); self.cfg = cfg
        self.r = Robot(world, cfg["start"], 0, self.rng, cfg["caps"], cfg["speed"], 0.0, scan_r=5.0 if role == "drone" else 6.0)
        self.log, self.t_start, self.done, self.result, self.heat, self.heat_budget = [], None, False, None, 0.0, heat_budget
        self.state, self.ti, self.wp, self.goal, self.first_t, self.timer, self.aim, self.quiet, self.carrying = "ENTER", 0, [], None, None, 0.0, None, 0, False
        self.radius = 4.0; self.arrivals = {}
        self.battery, self.drops, self.done_targets = cfg.get("battery", None), cfg.get("drops", 0), 0
        self.targets, self.skipped, self.haz, self.hints, self.corr = [], [], [], set(), set()
        self.beacon_read = None       # robots NEVER read state written by another robot (it would be a hidden robot-to-robot channel)
        self.cleared = set()          # hazards the COMMAND POST told us are cleared (the only way to learn what another robot did)
        if mission:
            self.targets = mission["targets"]; self.skipped = mission.get("skipped", [])
            self.haz = [(*anchor.to_local(h["lat"], h["lon"]), h["floor"], h["radius"], h.get("bid")) for h in mission["hazards"]]
            for s in mission.get("stairs", []):
                if s["kind"] == 2 and role != "drone": continue
                sx, sy = anchor.to_local(s["lat"], s["lon"]); ci, cj = world.to_cell(sx, sy)
                for di in range(-4, 5):
                    for dj in range(-4, 5): self.hints.add((ci + di, cj + dj))
            if self.nav == "direct": self.targets = self.order_jobs(self.targets, (self.r.pos[0], self.r.pos[1], 0))
    def _say(self, t, m): self.log.append((t, m))
    def order_jobs(self, targets, start, svc=15.0, floor_pen=25.0):
        """Order the jobs of ONE robot so that the urgency-weighted completion time is minimal:  minimise  sum_i  (1/pi_i) * T_i,  T_i = time at which job i is done
        (travel time at the robot's speed + `svc` s of work + a penalty per floor change). Urgent jobs (low pi) go first, UNLESS a cheap job lies on the way
        (a plain sort by pi would drive past it). Exact for <= 7 jobs (all orders), otherwise the Command Post's order is kept."""
        n = len(targets)
        if n < 2 or n > 7: return targets
        pts = [(*self.anchor.to_local(t["lat"], t["lon"]), t["floor"]) for t in targets]
        wgt = [1.0 / max(t.get("pi") if t.get("pi") is not None else 1.0, 0.05) for t in targets]
        def d(a, b): return math.hypot(a[0] - b[0], a[1] - b[1]) + (floor_pen if a[2] != b[2] else 0.0)
        best, bo = 1e18, None
        for perm in itertools.permutations(range(n)):
            cur, tm, cost = start, 0.0, 0.0
            for i in perm: tm += d(cur, pts[i]) / self.r.speed + svc; cost += wgt[i] * tm; cur = pts[i]
            if cost < best - 1e-9: best, bo = cost, perm
        return [targets[i] for i in bo]
    def _hazcost(self, n):
        f, i, j = n; x, y = self.w.center(i, j)
        return sum(4.0 for (hx, hy, hf, rad, hb) in self.haz if hf == f and hb not in self.cleared and dist((x, y), (hx, hy)) < rad + 1.0)
    def _extra(self, n): return self._hazcost(n) + self.r.extra(n)
    def _ok(self, c):
        f, ci, cj = c
        for rad in range(0, 6):
            for di in range(-rad, rad + 1):
                for dj in range(-rad, rad + 1):
                    n = (f, ci + di, cj + dj)
                    if self.w.inb(n[1], n[2]) and self.r.k.g[f][n[2]][n[1]] in (self.r.caps | {UNK}) and (self.r.k.g[f][n[2]][n[1]] != UNK or n in self.corr): return n
        return c
    def _finish(self, t, ok, why=""):
        self.done, self.state = True, "DONE"
        self.result = dict(role=self.role, success=ok, reason=why, time=round(t - self.t_start, 1), t_start=round(self.t_start, 1), first_target_time=self.first_t, distance=round(self.r.travelled, 1),
                           heat_dose=round(self.heat), reached=self.done_targets, arrivals=self.arrivals, skipped=self.skipped, battery_left=None if self.battery is None else round(self.battery))
    def _mark(self, t, tg, state):
        """Executor updates the STATE of the beacon it is working on: (1) writes it into the beacon by radio, (2) reports it to the van."""
        ok = self.beacon_cb(tg["bid"], state, "", t, list(self.r.pos), self.r.f, self.role) if self.beacon_cb else False
        self.link.send(self.role, "BEACON_UPDATE", {"bid": tg["bid"], "id": tg["id"], "state": state, "written": bool(ok)}, t)
        self._say(t, f"beacon {tg['id']} state -> {state}" + ("" if ok or not self.beacon_cb else " (written to the report only; beacon out of radio range)"))
    def apply_update(self, upd, t):
        """ORDER_UPDATE from the Command Post (arrives through the van). Replanning without restarting the robot:
        hazards cleared / new hazards, new danger zones around a victim, and new jobs (queued by urgency; a MUCH more urgent job may replace the one still being approached)."""
        if self.done: return False
        notes = []
        for bid in upd.get("cleared", []):
            if bid not in self.cleared: self.cleared.add(bid); notes.append("cleared")
        for h in upd.get("hazards", []):
            if all(x[4] != h["bid"] for x in self.haz): self.haz.append((*self.anchor.to_local(h["lat"], h["lon"]), h["floor"], h["radius"], h["bid"])); notes.append("new hazard " + h["id"])
        for bid, hz in (upd.get("wait_for") or {}).items():
            for tg in self.targets:
                if tg["bid"] == bid:
                    have = {x["bid"] for x in tg.get("wait_for", [])}; add = [x for x in hz if x["bid"] not in have]
                    if add: tg.setdefault("wait_for", []).extend(add); tg.setdefault("wait_timeout", 150.0); notes.append(f"{tg['id']} now waits for " + ",".join(x["id"] for x in add)); self.released = False
        new = [dict(x) for x in upd.get("targets", []) if x["bid"] not in {y["bid"] for y in self.targets}]
        if new and self.state in ("ENTER", "GO", "WORK"):
            key = lambda x: x.get("pi") if x.get("pi") is not None else 1e9
            cur = self.targets[self.ti] if self.ti < len(self.targets) else None
            if cur is None: self.targets += sorted(new, key=key)
            elif self.state == "GO" and min(key(x) for x in new) < 0.5 * key(cur):        # much more urgent than what I am approaching: switch now, the old job goes back in the queue
                self.targets[self.ti:] = sorted(new + self.targets[self.ti:], key=key); self.wp, self.goal, self.aim, self.released, self.hold_t0 = [], None, None, False, None; self.r.path = []
            else:
                tail = sorted(self.targets[self.ti + 1:] + new, key=key)
                if self.nav == "direct": a = self._local(cur["lat"], cur["lon"], cur["floor"]); tail = self.order_jobs(tail, a)
                self.targets[self.ti + 1:] = tail
            notes.append("new job(s) " + ",".join(x["id"] for x in new))
        if notes: self._say(t, "ORDER UPDATE from the Command Post (via the van): " + "; ".join(notes))
        return bool(notes)
    def _local(self, lat, lon, f): x, y = self.anchor.to_local(lat, lon); return (x, y, f)
    def _pos3(self): return (self.r.pos[0], self.r.pos[1], self.r.f)

    def tick(self, dt, t):
        if self.done: return
        r = self.r
        if self.t_start is None:
            self.t_start = t
            for s in self.skipped: self._say(t, f"SKIP {s}: human_intervention_required -> heavy USAR team, not a robot task")
            self._say(t, f"{self.role} leaves the van" + (f" with {len(self.targets)} target(s)" if self.use else " BLIND (no inherited map)"))
        r.scan(); s = self.w.sense(r.pos, r.f, self.rng)
        if self.role != "firetruck": self.heat += max(0.0, s["temp"] - 60) * dt
        if self.role == "ambulance" and self.heat > self.heat_budget: self._say(t, "HEAT BUDGET exceeded, abort"); return self._finish(t, False, "heat")
        if self.battery is not None:
            self.battery -= dt
            if self.battery < 70 + 0.6 * dist(r.pos, (0, 0)) / self.cfg["speed"] * 1.0 and self.state in ("GO", "WORK"):
                self._say(t, "LOW BATTERY: returning to the van"); self.link.send(self.role, "LOW_BATTERY", {}, t); self.state = "RETURN"; r.path = []
        if not self.use: return self._blind(dt, t, s)
        if self.state == "ENTER": self.state = "GO" if self.targets else "RETURN"
        if self.state == "GO": self._go(dt, t, s)
        elif self.state == "WORK": self._work(dt, t, s)
        elif self.state == "RETURN": self._return(dt, t)

    def _hazard_clear(self, h, s):
        """Is this danger zone safe? (1) the COMMAND POST relayed 'cleared' through the van (robots never talk to each other and never read what another
        robot wrote into a beacon), or (2) my own thermal camera sees it is out."""
        if h.get("bid") in self.cleared: return True
        hp = self._local(h["lat"], h["lon"], h["floor"])[:2]
        if h["type"] == "FIRE" and self.r.f == h["floor"] and dist(self.r.pos, hp) < 10 and self.w.los(self.r.f, self.r.pos, hp) and not any(dist(x, hp) < 5 for x in s["hot"]): return True
        return False

    # ---------------- go to the target using the inherited trail ----------------
    def _prepare(self, t):
        tg = self.targets[self.ti]
        self.wp = [self._local(c["lat"], c["lon"], c["floor"]) for c in tg["chain"]] + [self._local(tg["lat"], tg["lon"], tg["floor"])]
        self.hold_t0, self.hold_logged, self.released = None, False, False; self.radius = 4.0; self._ip = False; self.corr = corridor(self.w, [(self.r.pos[0], self.r.pos[1], 0)] + self.wp + [(0.0, 0.0, 0)], self.radius); self.aim = None
        self._say(t, f"target {tg['id']} ({tg['type']}, P{tg['prio']}, floor {tg['floor']}): following {len(self.wp)-1} inherited beacons")

    def _refine(self, s, tg, exp):
        """terminal guidance: beacons get us close, the robot's own sensors find the exact spot."""
        if self.r.f != tg["floor"]: return
        if tg["type"] in ("HUMAN", "ANIMAL"):
            c = [b for b in s["blobs"] if b["kind"] == tg["type"] and dist((b["x"], b["y"]), exp) < self.w.thr.get("refine_person", 4.5)]
            if c: b = min(c, key=lambda b: dist((b["x"], b["y"]), exp)); self.aim = (b["x"], b["y"])
        else:
            c = [h for h in s["hot"] if dist(h, exp) < self.w.thr.get("refine_fire", 5.5)]
            if c: self.aim = min(c, key=lambda h: dist(h, exp))

    def _go(self, dt, t, s):
        r = self.r
        if self.ti >= len(self.targets): self.state = "RETURN"; return
        tg = self.targets[self.ti]
        if not self.wp: self._prepare(t)
        exp = self.wp[-1][:2]; self._refine(s, tg, exp); end = self.aim or exp
        wf = tg.get("wait_for") or []
        if wf and not self.released and tg["type"] in ("HUMAN", "ANIMAL"):         # victim inside a danger zone: stay OUTSIDE until it is cleared
            opn = [h for h in wf if not self._hazard_clear(h, s)]
            if not opn: self.released = True; self._say(t, f"danger zone around {tg['id']} is CLEAR: entering")
            else:
                if self.hold_t0 is None: self.hold_t0 = t
                near = min([dist(r.pos, self._local(h["lat"], h["lon"], h["floor"])[:2]) for h in opn if h["floor"] == r.f] or [99.0])
                if t - self.hold_t0 > tg.get("wait_timeout", 150.0): self.released = True; self._say(t, f"danger zone still active after {tg.get('wait_timeout', 150.0):.0f} s: entering with hazard avoidance")
                elif near < self.w.thr.get("hold_dist", 9.0):
                    if not self.hold_logged: self.hold_logged = True; self._say(t, f"HOLDING outside the danger zone: waiting for {', '.join(h['id'] for h in opn)} to be cleared (Command Post notice / own sensors)")
                    r.path = []; return
        if r.f == tg["floor"]:
            d = dist(r.pos, end); so = self.cfg["standoff"] * self.w.thr.get("standoff_scale", 1.0)
            if tg["type"] in ("HUMAN", "ANIMAL") and d < so:
                if self.first_t is None: self.first_t = round(t - self.t_start, 1)
                self.link.send(self.role, "ARRIVED", {"note": f"at {tg['id']}"}, t); self._mark(t, tg, "ON_SITE"); self.state, self.timer = "WORK", 0.0; r.path = []; self.arrivals[tg["id"] + ":" + tg["type"]] = round(t - self.t_start, 1); self._say(t, f"REACHED {tg['id']}: loading the casualty"); return
            if tg["type"] == "GAS" and d < 2.0:
                if self.first_t is None: self.first_t = round(t - self.t_start, 1)
                self.link.send(self.role, "ARRIVED", {"note": f"at {tg['id']}", "bid": tg["bid"]}, t); self._mark(t, tg, "ON_SITE"); self.state, self.timer, self.quiet = "WORK", 0.0, 0; r.path = []; self._say(t, f"at gas leak {tg['id']}: sealing the source"); return
            if tg["type"] == "FIRE" and d < so and self.aim is not None and self.w.los(r.f, r.pos, self.aim):
                if self.first_t is None: self.first_t = round(t - self.t_start, 1)
                self.link.send(self.role, "ARRIVED", {"note": f"at {tg['id']}"}, t); self._mark(t, tg, "ON_SITE"); self.state, self.timer, self.quiet = "WORK", 0.0, 0; r.path = []; self._say(t, f"in range of fire {tg['id']}: starting suppression"); return
        while len(self.wp) > 1 and self.wp[0][2] == r.f and dist(r.pos, self.wp[0][:2]) < 1.5: self.wp.pop(0)
        goal = (self.aim[0], self.aim[1], tg["floor"]) if (self.aim and r.f == tg["floor"]) else self.wp[0]
        if self.nav == "direct": goal = self._direct_goal(tg, end, goal)
        if goal != self.goal: self.goal = goal; r.path = []
        if not r.path or any(r.k.g[f][j][i] in (WALL, RUBBLE) for (f, i, j) in r.path[:8] if r.k.g[f][j][i] != UNK):
            gc = (goal[2],) + self.w.to_cell(goal[0], goal[1]); gc = self._ok(gc)
            if self.nav == "direct": p = plan(r.k, r.cell(), goal=gc, caps=r.caps, opt="all", unk_v=self.hints, extra=self._extra, unk_cost=lambda n: 0.0 if n in self.corr else 0.6)
            else: p = plan(r.k, r.cell(), goal=gc, caps=r.caps, opt=self.corr, unk_v=self.hints, extra=self._extra)
            if p is None and self.nav != "direct" and self.radius < 15:                       # beacons are sparse: search a wider band around the trail before giving up
                self.radius *= 2; self.corr = corridor(self.w, [(self.r.pos[0], self.r.pos[1], 0)] + self.wp + [(0.0, 0.0, 0)], self.radius); return
            if p is None:
                self._say(t, f"{tg['id']} unreachable, skipping"); self.link.send(self.role, "UNREACHABLE", {"bid": tg["bid"], "id": tg["id"]}, t); self._mark(t, tg, "UNREACHABLE")
                self.ti += 1; self.wp = []; self.goal = None; return
            r.path = p
        r.follow(r.speed * dt)

    def _direct_goal(self, tg, end, default):
        """Free-space navigation. The Writer's trail only tells us WHERE the target is and WHICH way the stairs are; it is not a route we must walk.
        Same floor as the target and no floor change left in the chain -> plan straight to the target (the trail cells are just cheaper because they are known safe).
        A floor change in the chain -> plan to the stairs entry (last waypoint on this floor), skipping any trail waypoint we do not need to pass."""
        r = self.r
        while len(self.wp) > 1 and self.wp[0][2] != r.f and any(w[2] == r.f for w in self.wp[1:]): self.wp.pop(0)       # we changed floor: forget the old floor's waypoints
        run = []
        for w in self.wp:
            if w[2] != self.wp[0][2]: break
            run.append(w)
        for idx in range(len(run) - 1, -1, -1):                                        # waypoint skipping: if we are already at a later waypoint, drop everything before it
            if idx < len(self.wp) - 1 and run[idx][2] == r.f and dist(r.pos, run[idx][:2]) < 1.5: del self.wp[:idx + 1]; return self._direct_goal(tg, end, self.wp[0])
        if r.f == tg["floor"] and all(w[2] == r.f for w in self.wp): return (end[0], end[1], tg["floor"])
        if self.wp[0][2] == r.f: return run[-1]
        return default

    # ---------------- rescue / extinguish ----------------
    def _work(self, dt, t, s):
        tg = self.targets[self.ti]; r = self.r; self.timer += dt
        if tg["type"] in ("HUMAN", "ANIMAL"):
            if self.timer >= 1.0 and not self._ip: self._ip = True; self._mark(t, tg, "IN_PROGRESS")
            if self.timer >= 3.0:
                o = self.w.rescue_at(r.f, self.aim or r.pos, 2.5)
                self.link.send(self.role, "RESCUED", {"bid": tg["bid"], "id": tg["id"]}, t); self._mark(t, tg, "RESOLVED"); self.carrying = True; self.done_targets += 1
                self._say(t, f"RESCUED {tg['id']}: evacuating to the van"); self.ti += 1; self.wp = []; self.aim = None; self.state = "RETURN" if True else "GO"; r.path = []
            return
        if tg["type"] == "GAS":
            if not self._ip: self._ip = True; self._mark(t, tg, "IN_PROGRESS")
            self.w.secure(r.f, r.pos, 6.0, 0.08 * dt)
            self.quiet = self.quiet + 1 if s["gas"] < 40 else 0
            if self.quiet >= 3 and self.timer > 4.0:
                self.link.send(self.role, "HAZARD_CLEARED", {"bid": tg["bid"], "id": tg["id"]}, t); self._mark(t, tg, "RESOLVED"); self.done_targets += 1
                self._say(t, f"gas leak {tg['id']} is SECURED"); self.ti += 1; self.wp = []; self.aim = None; self.goal = None; self.state = "GO" if self.ti < len(self.targets) else "RETURN"; r.path = []
            elif self.timer > 120:
                self._say(t, f"gas leak {tg['id']}: cannot be sealed"); self.link.send(self.role, "UNREACHABLE", {"bid": tg["bid"], "id": tg["id"]}, t); self._mark(t, tg, "UNREACHABLE")
                self.ti += 1; self.wp = []; self.aim = None; self.goal = None; self.state = "GO" if self.ti < len(self.targets) else "RETURN"; r.path = []
            return
        exp = self.wp[-1][:2] if self.wp else (self.aim or r.pos)
        hot = [h for h in s["hot"] if dist(h, exp) < 6.0]
        if hot:
            self.aim = min(hot, key=lambda h: dist(h, exp)); self.quiet = 0
            if not self._ip: self._ip = True; self._mark(t, tg, "IN_PROGRESS")
            if self.role == "firetruck": self.w.douse(r.f, self.aim, 2.5, 0.04 * dt)
            elif self.timer >= 4.0 and self.drops > 0: self.w.douse(r.f, self.aim, 3.0, 0.45); self.drops -= 1; self.timer = 0.0; self._say(t, f"drone drop ({self.drops} left)")
            elif self.drops <= 0: self._say(t, "payload empty: returning to the van"); self.state = "RETURN"; r.path = []
        else:
            self.quiet += 1
            if self.quiet >= 3:
                self.link.send(self.role, "FIRE_OUT", {"bid": tg["bid"], "id": tg["id"]}, t); self._mark(t, tg, "RESOLVED"); self.done_targets += 1
                self._say(t, f"fire {tg['id']} is OUT"); self.ti += 1; self.wp = []; self.aim = None; self.goal = None; self.state = "GO" if self.ti < len(self.targets) else "RETURN"; r.path = []

    def _return(self, dt, t):
        r = self.r
        if r.f == 0 and dist(r.pos, (0.0, 0.0)) < 1.5:
            self.link.send(self.role, "RETURNED", {"carrying": self.carrying}, t); self._say(t, "back at the van"); return self._finish(t, self.done_targets > 0, "returned")
        if not r.path:
            p = plan(r.k, r.cell(), goal=(0,) + self.w.to_cell(0.0, 0.0), caps=r.caps, opt=self.corr, unk_v=self.hints, extra=self._extra)
            if p is None and self.radius < 15:
                self.radius *= 2; pts = self.wp + [(0.0, 0.0, 0), (r.pos[0], r.pos[1], r.f)]; self.corr = corridor(self.w, pts, self.radius); return
            if p is None: self._say(t, "no way back"); return self._finish(t, False, "trapped")
            r.path = p
        r.follow(r.speed * dt)

    # ---------------- baseline: same robot WITHOUT inherited memory ----------------
    def _blind(self, dt, t, s):
        r = self.r; bad = getattr(self, "bad", []); self.bad = bad
        for b in s["blobs"]:
            if b["kind"] == "HUMAN" and b["conf"] >= 0.6 and self.aim is None and not any(dist((b["x"], b["y"]), q) < 2.0 for q in bad):
                gc = self._ok((r.f,) + self.w.to_cell(b["x"], b["y"])); p = plan(r.k, r.cell(), goal=gc, caps=r.caps)
                if p is not None and (p or dist(r.pos, (b["x"], b["y"])) < 1.3) and dist(self.w.center(gc[1], gc[2]), (b["x"], b["y"])) < 2.0:
                    self.aim = (b["x"], b["y"]); r.path = p; self._say(t, "human found by own sensors")
                else: bad.append((b["x"], b["y"]))
        if self.aim is not None:
            if dist(r.pos, self.aim) < 1.3: self.first_t = round(t - self.t_start, 1); self.arrivals["blind"] = self.first_t; self._say(t, "REACHED victim (blind)"); return self._finish(t, True)
            if not r.path:
                bad.append(self.aim); self.aim = None; self._say(t, "cannot reach that casualty, exploring on")
        elif not r.path and not r.choose_frontier(): return self._finish(t, False, "explored everything")
        r.follow(r.speed * dt)
