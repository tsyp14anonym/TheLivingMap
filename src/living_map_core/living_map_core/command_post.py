"""Command Post: ranks alerts, allocates work to the robots docked in the van, sequences them via station messages."""
from .sim_core import dist
from .beacon_protocol import confidence, priority

ROLES = ("firetruck", "drone", "ambulance")
ROLE_TEXT = {"firetruck": "Fire-fighting robot (ground floor)", "drone": "Fire-fighting drone (upper floors, shafts)", "ambulance": "Rescue robot (ambulance)"}

# Which robot can do what. The Command Post picks an executor by CAPABILITY (not by a hard-coded type -> robot table), so a lost robot can be replaced.
CAPS = {"firetruck": {"extinguish", "seal_gas", "ground"}, "drone": {"extinguish", "seal_gas", "air", "stairs"}, "ambulance": {"rescue", "ground"}}
HAZ, VICTIM = ("FIRE", "GAS"), ("HUMAN", "ANIMAL")

class CommandPost:
    """The ONLY coordinator. Robots never talk to each other: they report to the van, the van uplinks to this class, and this class answers
    (briefing at the van, then ORDER_UPDATE messages back through the van). Decisions are taken once per `cycle` seconds (default 1 s):
    collect -> update priorities -> correlate into incidents -> preempt/aggregate -> one order per robot -> replan running orders."""
    def __init__(self, anchor, brief_delay=8.0, amb_timeout=150.0, collect_window=1.0, max_collect=20.0, danger_collect=20.0,
                 cycle=None, crit=0.3, update_gap=5.0, lost_after=35.0, corr_dist=8.0, corr_time=300.0):
        self.anchor, self.brief_delay, self.amb_timeout = anchor, brief_delay, amb_timeout
        self.window, self.max_collect, self.danger_collect = collect_window, max_collect, danger_collect          # collect related reports for `window` s so ONE order covers them
        self.last_rx, self.pending_since, self.orders, self.order_of, self._op = -99.0, None, [], {}, 0
        self.records, self.batches, self.out, self.rescued, self.messages = {}, [], set(), set(), []
        self.first_t, self.dispatched, self.state, self.missions = {}, {r: None for r in ROLES}, {r: dict(state="docked", note="no target yet") for r in ROLES}, {}
        self.first_human_t, self.failed, self.force = None, {}, set()
        self.bstate, self.flow, self._seq = {}, [], 0
        self.cycle = collect_window if cycle is None else cycle                      # decision cycle (s): the Command Post decides once per cycle, never per event
        self.crit, self.update_gap, self.lost_after, self.corr_dist, self.corr_time = crit, update_gap, lost_after, corr_dist, corr_time
        self._last_cycle, self.outbox, self.last_seen, self.lost = -1e9, [], {}, set()          # outbox: CP -> van -> robot messages (ORDER_UPDATE)
        self._upd_t, self._cleared_sent, self.incident_of, self.incidents = {}, {r: set() for r in ROLES}, {}, {}

    def _ev(self, t, kind, src, dst, label):
        self._seq += 1; self.flow.append(dict(seq=self._seq, t=round(t, 1), kind=kind, src=src, dst=dst, label=label))
        if len(self.flow) > 200: self.flow.pop(0)

    # ---------------- input ----------------
    def ingest(self, batch, now):
        self.batches.append(dict(t=now, reason=batch["reason"], n=len(batch.get("records", []))))
        for r in batch.get("records", []):
            self._ev(now, "UPLINK", "van", "cp", f"{r['id']} {r['type']} P{r['prio']}")
            old = self.records.get(r["bid"])
            if old is None or r["seq"] > old["seq"]: self.records[r["bid"]] = dict(r)
            if old is None and r["type"] in ("FIRE", "GAS", "HUMAN", "ANIMAL"): self.last_rx = now
            if r["type"] == "HUMAN" and self.first_human_t is None: self.first_human_t = now
        for m in batch.get("statuses", []): self.ingest_status(m, now)

    def ingest_status(self, m, now):
        role0 = m.get("robot")
        if role0 in ROLES:
            self.last_seen[role0] = now
            if role0 in self.lost:                                   # it is talking again: back in service
                self.lost.discard(role0); self._ev(now, "STATE", role0, "cp", "robot reconnected"); self.state[role0] = dict(state="dispatched" if self.dispatched[role0] is not None else "docked", note=f"reconnected at {round(now)} s")
        if m["kind"] == "EXPLORATION_DONE":                         # the Writer reports (through the van) that nothing more can be found: waiting for a more urgent job has no value any more
            self.exploring = False; self._ev(now, "STATION", "writer", "cp", "exploration complete"); return
        if m["kind"] == "STATUS":                                  # periodic heartbeat from a robot (through the van): heals a lost RETURNED message
            role, p = m["robot"], m["payload"]; d = self.dispatched.get(role)
            if d is not None and not p.get("active") and now - d > 30:
                self.dispatched[role] = None; self.first_t.pop(role, None); self.missions.pop(role, None)
                self.state[role] = dict(state="docked", note=f"heartbeat: back in the van ({round(now)} s)"); self._ev(now, "STATE", role, "cp", "robot reset by heartbeat")
            return
        self.messages.append(dict(t=round(now, 1), robot=m["robot"], kind=m["kind"], payload=m["payload"]))
        k, p, role = m["kind"], m["payload"], m["robot"]
        self._ev(now, "STATE" if k == "BEACON_UPDATE" else "STATION", role, "cp", (f"{p['id']} -> {p['state']}" if k == "BEACON_UPDATE" else k))
        if k == "BEACON_UPDATE": self.bstate[p["bid"]] = dict(state=p["state"], by=role, t=round(now, 1), written=p.get("written"))
        if k in ("FIRE_OUT", "HAZARD_CLEARED"): self.out.add(p["bid"])
        elif k == "RESCUED": self.rescued.add(p["bid"])
        elif k == "UNREACHABLE": self.failed[(role, p["bid"])] = self.failed.get((role, p["bid"]), 0) + 1
        elif k == "RETURNED":
            self.dispatched[role] = None; self.first_t.pop(role, None); self.missions.pop(role, None); self.state[role] = dict(state="docked", note=f"back in the van at {round(now)} s")
        elif k == "LOW_BATTERY": self.state[role] = dict(state="returning", note="low battery: returning to the van")
        elif k == "ARRIVED": self.state[role] = dict(state="working", note=p.get("note", "at target"))

    def refresh(self, now):
        """pi in [0, +inf): LOWER = MORE URGENT.  pi_base = what the event is worth when it is detected (kind x Writer class / victims).
        pi (effective) = pi_base * exp(-AGING * age): it only DROPS while the event stays unresolved (never negative, never reset: age counts from the
        detection time t0, not from the last report). A hazard in the same incident as a victim inherits the victim's urgency."""
        for r in self.records.values():
            age = now - r["t0"]; r["conf"] = confidence(r["type"], age); r["pi_base"] = round(priority(r["type"], r["prio"], r["n"], 0.0), 3)
            r["pi0"] = priority(r["type"], r["prio"], r["n"], age); r["pi"] = r["pi0"]
            r["reverify"] = r["prio"] <= 1 and r["conf"] < 0.5 and r["type"] in ("FIRE", "GAS", "HUMAN") and r["bid"] not in self.out
            r["out"] = r["bid"] in self.out; r["rescued"] = r["bid"] in self.rescued
        self.correlate()
        victims = [h for h in self.records.values() if h["type"] == "HUMAN" and not h["human_needed"] and h["bid"] not in self.rescued]
        for r in self.records.values():
            if r["type"] in HAZ and r["bid"] not in self.out:
                for h in victims:
                    if self.incident_of.get(h["bid"]) is not None and self.incident_of.get(h["bid"]) == self.incident_of.get(r["bid"]): r["pi"] = min(r["pi"], h["pi0"] * 0.9)
            r["pi"] = round(r["pi"], 3); r["Q"] = r["pi"]

    # ---------------- event correlation ----------------
    def _related(self, a, b):
        """Two events belong together when (1) at least one is a hazard (two victims are not related just because they are close), (2) same floor,
        (3) spatial proximity, (4) temporal proximity of the detections. Events that merely happened at the same time elsewhere stay apart."""
        if a["type"] not in HAZ and b["type"] not in HAZ: return False
        return a["floor"] == b["floor"] and dist(a["tlocal"], b["tlocal"]) <= self.corr_dist and abs(a["t0"] - b["t0"]) <= self.corr_time
    def correlate(self):
        act = [r for r in self.records.values() if r["type"] in HAZ + VICTIM and r["bid"] not in self.out and r["bid"] not in self.rescued]
        par = {r["bid"]: r["bid"] for r in act}
        def find(x):
            while par[x] != x: par[x] = par[par[x]]; x = par[x]
            return x
        for i, a in enumerate(act):
            for b in act[i + 1:]:
                if self._related(a, b):
                    ra, rb = find(a["bid"]), find(b["bid"])
                    if ra != rb: par[max(ra, rb)] = min(ra, rb)         # transitive: fire~gas and gas~person => one incident
        groups = {}
        for r in act: groups.setdefault(find(r["bid"]), []).append(r["bid"])
        self.incident_of = {bid: f"INC-{root}" for root, ms in groups.items() for bid in ms}
        self.incidents = {f"INC-{root}": sorted(ms) for root, ms in groups.items()}
    def _hazards_of(self, r):
        inc = self.incident_of.get(r["bid"])
        return [h for h in self.records.values() if inc and h["type"] in HAZ and h["bid"] not in self.out and h["bid"] != r["bid"] and self.incident_of.get(h["bid"]) == inc]
    @staticmethod
    def _hz(h): return dict(bid=h["bid"], id=h["id"], type=h["type"], lat=h["tlat"], lon=h["tlon"], floor=h["floor"], radius=2.5 if h["type"] == "FIRE" else 3.0)
    @staticmethod
    def _need(r):
        if r["type"] in VICTIM: return {"rescue"}
        if r["type"] == "FIRE": return {"extinguish"} | ({"stairs"} if r["floor"] > 0 else set())
        if r["type"] == "GAS": return {"seal_gas"} | ({"stairs"} if r["floor"] > 0 else set())
        return set()

    # ---------------- allocation ----------------
    def _role_of(self, r):
        """Executor selection by capability: the first CAPABLE robot (preference: fire truck, then drone) that is not lost."""
        if r["type"] in VICTIM: return None if r["human_needed"] else "ambulance"
        if r["type"] in HAZ:
            need = self._need(r); capable = [ro for ro in ("firetruck", "drone") if need <= CAPS[ro]]
            return next((ro for ro in capable if ro not in self.lost), capable[0] if capable else None)
        return None
    exploring, hold_override = True, None          # exploring: the Writer has not yet reported "exploration complete"
    @property
    def hold_above(self):
        """AUTOMATIC threshold (no per-type rule, no fixed delay): a job is LOW-URGENCY when its priority number is worse than that of the LEAST urgent
        life-threatening event (person, fire, gas) at class P1 - read from the priority table. Such a job does not send a robot out while the Writer is still
        exploring (a more urgent event may still turn up); it is released by the aging law pi(t) = pi0 exp(-0.004 t), or at once when exploration is over."""
        return self.hold_override if self.hold_override is not None else max(priority(t, 1, 1, 0.0) for t in ("HUMAN",) + HAZ)
    def targets_for(self, role):
        t = [r for r in self.records.values() if self._role_of(r) == role and r["bid"] not in self.out and r["bid"] not in self.rescued and self.failed.get((role, r["bid"]), 0) < 2]
        return sorted(t, key=lambda r: (r.get("pi", 1e9), r["bid"]))

    def chain(self, bid):
        out, cur, guard = [], self.records.get(bid), 0
        while cur and guard < 60: out.append(dict(lat=cur["lat"], lon=cur["lon"], floor=cur["floor"])); cur = self.records.get(cur["parent"]); guard += 1
        return out[::-1]

    def _target(self, r):
        t = dict(id=r["id"], bid=r["bid"], type=r["type"], prio=r["prio"], floor=r["floor"], lat=r["tlat"], lon=r["tlon"], n=r["n"], pi=r.get("pi"), chain=self.chain(r["bid"]), incident=self.incident_of.get(r["bid"]))
        if r["type"] in VICTIM:          # a victim in a danger zone: the robot enters only after the Command Post says the hazard is cleared (or its own sensors see it is safe)
            t["wait_for"] = [self._hz(h) for h in self._hazards_of(r)]; t["wait_timeout"] = self.amb_timeout
        return t
    def _mission(self, role, now, epoch):
        tg = [self._target(r) for r in self.targets_for(role)]
        hz = [self._hz(r) for r in self.records.values() if r["type"] in HAZ and r["bid"] not in self.out]
        st = [dict(lat=r["tlat"], lon=r["tlon"], floor=r["floor"], kind=r["n"]) for r in self.records.values() if r["type"] == "STAIRS"]
        db = [dict(lat=r["tlat"], lon=r["tlon"], floor=r["floor"]) for r in self.records.values() if r["type"] == "DEBRIS"]
        skipped = [r["id"] for r in self.records.values() if r["type"] == "HUMAN" and r["human_needed"]] if role == "ambulance" else []
        return dict(role=role, issued_at=round(now, 1), run_epoch=epoch, anchor=self.anchor.as_dict(), targets=tg, hazards=hz, stairs=st, debris=db, skipped=skipped)

    # ---------------- robot health + replanning (all robot traffic goes through the van) ----------------
    def _health(self, now):
        for role in ROLES:
            d = self.dispatched[role]
            if d is None or role in self.lost: continue
            ref = max(d, self.last_seen.get(role, d))
            if now - ref > self.lost_after:
                self.lost.add(role); self.state[role] = dict(state="lost", note=f"no message from {role} for {round(now - ref)} s: marked LOST, its jobs can go to another capable robot")
                self._ev(now, "STATE", role, "cp", "robot LOST (no heartbeat)")
    def _push(self, role, upd, now, label):
        self.outbox.append(dict(role=role, kind="ORDER_UPDATE", payload=upd, t=round(now, 1))); self._ev(now, "DISPATCH", "cp", role, f"ORDER_UPDATE {label}")
    def _replan(self, now):
        """A robot that is already out gets an ORDER_UPDATE (through the van) when something changes: a new related hazard around its victim, a hazard that is now
        cleared, or a new job it can do. New jobs are rate-limited (update_gap) so the robot is not flooded."""
        for role in ROLES:
            m = self.missions.get(role)
            if self.dispatched[role] is None or m is None or role in self.lost: continue
            upd, label = {}, []
            have = {t["bid"] for t in m["targets"]}
            new = [r for r in self.targets_for(role) if r["bid"] not in have]
            if new and now - self._upd_t.get(role, -1e9) >= self.update_gap:
                tl = [self._target(r) for r in new]; m["targets"] += tl; upd["targets"] = tl; self._upd_t[role] = now; label.append("+" + ",".join(t["id"] for t in tl))
            known = {h["bid"] for h in m["hazards"]}
            nh = [self._hz(r) for r in self.records.values() if r["type"] in HAZ and r["bid"] not in self.out and r["bid"] not in known]
            if nh: m["hazards"] += nh; upd["hazards"] = nh
            for t in m["targets"]:
                rec = self.records.get(t["bid"])
                if t["type"] in VICTIM and rec is not None:
                    cur = {h["bid"] for h in t.get("wait_for", [])}; add = [self._hz(h) for h in self._hazards_of(rec) if h["bid"] not in cur]
                    if add: t.setdefault("wait_for", []).extend(add); upd.setdefault("wait_for", {})[t["bid"]] = add; label.append(f"{t['id']} waits for " + ",".join(h["id"] for h in add))
            wait_b = {h["bid"] for t in m["targets"] for h in t.get("wait_for", [])}
            cl = sorted((wait_b | {h["bid"] for h in m["hazards"]}) & self.out - self._cleared_sent[role])
            if cl: upd["cleared"] = cl; self._cleared_sent[role] |= set(cl); label.append("cleared " + ",".join(self.records[b]["id"] for b in cl if b in self.records))
            if upd: self._push(role, upd, now, "; ".join(label) or "hazards")

    def pop_outbox(self):
        o, self.outbox = self.outbox, []; return o

    def ready(self, now, epoch=0, approved=True):
        """ONE DECISION CYCLE (every `cycle` s, default 1 s; calls in between do nothing):
          1 refresh priorities (aging)  2 correlate events into incidents  3 robot health  4 replan running orders (ORDER_UPDATE)
          5 for each idle robot: wait until the reports of this window are in (quiet for `window` s) so ONE order covers every related job,
            EXCEPT when a job is critical (effective pi < crit): it preempts the wait (balance: fast for the urgent, aggregated for the rest)
          6 issue one order per robot and record which incidents it covers."""
        if not self.force and now - self._last_cycle < self.cycle - 1e-9: return []
        self._last_cycle = now; self.refresh(now); self._health(now); self._replan(now); go = []
        cand = {ro: self.targets_for(ro) for ro in ROLES}
        waiting = [ro for ro in ROLES if self.dispatched[ro] is None and cand[ro] and ro not in self.lost]
        for ro in ROLES:
            if self.dispatched[ro] is None and not cand[ro] and ro not in self.lost: self.state[ro] = dict(state="docked", note="no target yet")
        if waiting and self.pending_since is None: self.pending_since = now
        if not waiting: self.pending_since = None
        quiet = now - self.last_rx >= self.window or (self.pending_since is not None and now - self.pending_since >= self.max_collect)
        for role in waiting:
            forced = role in self.force; pre = min(c.get("pi", 1e9) for c in cand[role]) < self.crit
            best = min(c.get("pi", 1e9) for c in cand[role])
            if not forced and not pre and self.exploring and best > self.hold_above:
                self.state[role] = dict(state="briefing", note=f"only low-urgency jobs (priority {best:.1f} > {self.hold_above:g}): holding while the Writer explores; they go when a more urgent job appears, when they have aged, or when exploration ends"); continue
            if not forced and not pre and not quiet: self.state[role] = dict(state="briefing", note=f"collecting reports for {self.window:g} s so related events go into ONE order"); continue
            if role == "ambulance" and not forced and not pre:           # a critical victim means a danger zone: wait briefly for the hazard report so ONE order covers both
                first = cand[role][0]
                if first["type"] == "HUMAN" and first["prio"] == 0 and not self._hazards_of(first) and now - first["rx_t"] < self.danger_collect:
                    self.state[role] = dict(state="briefing", note=f"victim {first['id']} is in a danger zone: waiting up to {self.danger_collect:g} s for the hazard report so ONE order covers both"); continue
            if not approved and not forced: self.state[role] = dict(state="awaiting", note="awaiting operator approval"); continue
            if pre and not forced and not quiet: self._ev(now, "DISPATCH", "cp", role, f"PREEMPT: critical pi={min(c.get('pi', 1e9) for c in cand[role]):.2f} < {self.crit:g}, no waiting")
            self.force.discard(role); m = self._mission(role, now, epoch); self.missions[role] = m; self.dispatched[role] = now; self.last_seen[role] = now; self._cleared_sent[role] = set()
            self.state[role] = dict(state="dispatched", note=f"sent at {round(now)} s with {len(m['targets'])} target(s)"); go.append((role, m))
            self._ev(now, "DISPATCH", "cp", role, f"briefing: {', '.join(t['id'] for t in m['targets'])}")
        if go:
            self._op += 1; op = f"OP-{self._op}"; parts = []; incs = set()
            for role, m in go:
                for t in m["targets"]:
                    t["order"] = op; self.order_of[t["bid"]] = op
                    if t.get("incident"): incs.add(t["incident"])
                parts.append(f"{role}: " + ", ".join(t["id"] + (f" (waits for {'/'.join(h['id'] for h in t['wait_for'])})" if t.get("wait_for") else "") for t in m["targets"]))
            self.orders.append(dict(id=op, t=round(now, 1), text=" | ".join(parts), roles={r: [t["id"] for t in m["targets"]] for r, m in go}, incidents=sorted(incs)))
        return go

    def awaiting(self, now):
        return [r for r in ROLES if self.state[r]["state"] == "awaiting"]

    STATE_TEXT = {"KNOWN": "known at the Command Post", "QUEUED": "queued, waiting for a free robot", "HOLDING": "robot holding in the van", "EN_ROUTE": "robot on its way", "ON_SITE": "robot on site",
                  "IN_PROGRESS": "rescue / extinguishing in progress", "RESOLVED": "done", "UNREACHABLE": "no route found", "HEAVY_RESCUE": "needs the human rescue team", "LOST": "beacon went silent", "ACTIVE": "active (infrastructure)"}
    def _planned_role(self, r): return self._role_of(r) or ("ambulance" if r["type"] == "HUMAN" else None)
    def beacon_table(self):
        rows = []
        for r in self.records.values():
            assigned = next((ro for ro, m in self.missions.items() if self.dispatched.get(ro) is not None and any(t["bid"] == r["bid"] for t in m["targets"])), None)
            planned = assigned or self._planned_role(r); bs = self.bstate.get(r["bid"])
            if r["type"] == "BEACON_LOST": st = "LOST"
            elif r["type"] in ("REPEATER", "STAIRS"): st = "ACTIVE"
            elif r["bid"] in self.out or r["bid"] in self.rescued or (bs and bs["state"] == "RESOLVED"): st = "RESOLVED"
            elif r["human_needed"] or r["type"] == "DEBRIS": st = "HEAVY_RESCUE"
            elif planned and self.failed.get((planned, r["bid"]), 0) >= 2 or (bs and bs["state"] == "UNREACHABLE"): st = "UNREACHABLE"
            elif bs and bs["state"] in ("ON_SITE", "IN_PROGRESS"): st = bs["state"]
            elif assigned: st = "EN_ROUTE"
            elif planned and self.state.get(planned, {}).get("state") == "holding": st = "HOLDING"
            elif planned and r["type"] in ("HUMAN", "ANIMAL", "FIRE", "GAS"): st = "QUEUED"
            else: st = "KNOWN"
            why = ""
            if st == "KNOWN": why = ""
            elif st == "QUEUED": why = f"waiting for the {planned}: busy with another job or not briefed yet"
            elif st == "HOLDING": why = "waiting for the danger zone to be cleared"
            elif st == "HEAVY_RESCUE": why = "blocked by rubble: needs the human heavy-rescue team"
            elif st == "UNREACHABLE": why = "no route found (tried twice)"
            elif st == "EN_ROUTE": why = f"{assigned} is on its way" + (" and waits outside the danger zone until it is clear" if any(t.get("wait_for") for ro, m in self.missions.items() for t in m["targets"] if t["bid"] == r["bid"]) else "")
            elif st in ("ON_SITE", "IN_PROGRESS"): why = f"{(bs or {}).get('by') or planned} is working on it"
            rows.append(dict(assigned=assigned, incident=self.incident_of.get(r["bid"]), related=[self.records[b]["id"] for b in self.incidents.get(self.incident_of.get(r["bid"]), []) if b != r["bid"]], caps=sorted(self._need(r)), pi_base=r.get("pi_base"), t_detect=r["t0"], why=why, pi=round(r.get("pi", 0), 2), order=self.order_of.get(r["bid"]), bid=r["bid"], id=r["id"], type=r["type"], prio=r["prio"], floor=r["floor"], state=st, robot=planned, conf=round(r.get("conf", 1), 2), Q=round(r.get("Q", 0), 1),
                             by=(bs["by"] if bs else None), upd=(bs["t"] if bs else None), rx=round(r["rx_t"], 1), where=f"{r['hop_text']}", target=r["tgt_text"]))
        return rows

    def events(self, now, rows=None):
        """The global event store, one dict per event (what the spec asks for): id, type, location, detection time, base + effective priority,
        required capabilities, status, assigned robot, related events (same incident)."""
        out = []
        for x in (rows if rows is not None else self.beacon_table()):
            r = self.records[x["bid"]]
            out.append(dict(event_id=x["id"], type=x["type"], location=dict(floor=r["floor"], lat=r["tlat"], lon=r["tlon"], x=r["tlocal"][0], y=r["tlocal"][1]), detected_at=x["t_detect"],
                            priority_base=x["pi_base"], priority=x["pi"], required_capabilities=x["caps"], status=x["state"], assigned_robot=x["assigned"], related_events=x["related"], incident=x["incident"]))
        return out
    def dashboard_state(self, now, approved=True):
        self.refresh(now)
        recs = []
        for r in self.records.values():
            x, y = r["local"]; tx, ty = r["tlocal"]
            recs.append(dict(id=r["id"], bid=r["bid"], type=r["type"], prio=r["prio"], parent=r["parent"], floor=r["floor"], x=x, y=y, tx=tx, ty=ty, lat=r["lat"], lon=r["lon"],
                             conf=r.get("conf", 1), Q=round(r.get("Q", 0), 1), pi=round(r.get("pi", 0), 2), human_needed=r["human_needed"], reverify=r.get("reverify", False), out=r.get("out", False),
                             rescued=r.get("rescued", False), sigma=r["sigma_m"], rssi=r["rssi"], hops=r["hops"], hop_text=r["hop_text"], tgt_text=r["tgt_text"]))
        robots = {ro: dict(self.state[ro], role_text=ROLE_TEXT[ro], targets=[dict(id=t["id"], type=t["type"], prio=t["prio"], floor=t["floor"]) for t in self.missions.get(ro, {}).get("targets", [])],
                           issued_at=self.missions.get(ro, {}).get("issued_at")) for ro in ROLES}
        return dict(now=round(now, 1), records=recs, batches=self.batches[-8:], robots=robots, messages=self.messages[-14:], skipped=sorted({s for m in self.missions.values() for s in m.get("skipped", [])}),
                    awaiting_approval=bool(self.awaiting(now)), anchor=self.anchor.as_dict(), floors=2, orders=self.orders[-8:], table=(tbl := self.beacon_table()), events=self.events(now, tbl), incidents={k: [self.records[b]["id"] for b in v] for k, v in self.incidents.items()}, lost=sorted(self.lost), cycle=self.cycle, crit=self.crit, exploring=self.exploring, hold_above=round(self.hold_above, 2), flow=self.flow[-120:], state_text=self.STATE_TEXT)
