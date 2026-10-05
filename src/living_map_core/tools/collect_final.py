"""FINAL measurements for the technical report and the technical document (one-floor project).
Single source of truth: every number in the two documents is read from docs/results_final.json or from the code constants.
Usage: python3 tools/collect_final.py            (about 15-20 minutes on one CPU; results are written after every experiment)"""
import sys, os, json, math, time, statistics as st
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(H, ".."))
from living_map_core.simulation import Simulation, blind_baseline
MAPS = os.path.join(H, "..", "maps"); ARENA = os.path.join(MAPS, "proto_arena.json"); BUILD = os.path.join(MAPS, "complex_1floor.json")
OUT = os.path.join(H, "..", "docs", "results_final.json")
R = {}

def run_kpi(mp, seed=7, uplink="immediate", collect=1.0, nav="direct", fault="none", wall_db=None, hold_off=False, max_t=900.0, trace=False):
    w0 = time.time(); s = Simulation(fault, True, mp, seed=seed, uplink=uplink, collect=collect, nav=nav)
    if wall_db is not None: s.w.wall_db = float(wall_db)
    if hold_off: s.cp.hold_override = 1e9                         # nothing counts as low-urgency: ablation of the hold rule
    ups, delays = [], {}
    orig = s.cp.ingest
    def spy(msg, now):
        if msg.get("records"):
            ups.append((now, msg.get("reason"), len(msg["records"])))
            for r in msg["records"]: delays.setdefault(f"P{r['prio']}", []).append(max(0.0, now - r["t0"]))
        return orig(msg, now)
    s.cp.ingest = spy
    ev = dict(drop={}, first_order=None, amb_dep=[], ft_dep=[], arrive_h=None, first_rescue=None, wr_done=None, expl_msg=None, fires_out=None, gps_end=None, killed=None)
    seen = set(); tr = dict(writer=[], ambulance=[], firetruck=[]) if trace else None
    while s.t < max_t and not s.final:
        s.step(0.5); t = s.t
        for b in s.wr.beacons:
            if b["bid"] not in ev["drop"]: ev["drop"][b["bid"]] = (round(t - 0.5, 1), b["type"], [round(v, 2) for v in s.wr.r.pos])
        if ev["first_order"] is None and s.cp.orders: ev["first_order"] = s.cp.orders[0]["t"]
        for role, key in (("ambulance", "amb_dep"), ("firetruck", "ft_dep")):
            ag = s.resp.get(role)
            if ag is not None and (role, id(ag)) not in seen: seen.add((role, id(ag))); ev[key].append((round(t, 1), [x["id"] for x in ag.targets]))
        am = s.resp.get("ambulance")
        if ev["arrive_h"] is None and am is not None and any(k.endswith(":HUMAN") for k in am.arrivals): ev["arrive_h"] = round(t, 1)
        if ev["first_rescue"] is None and any(h["rescued"] for h in s.w.humans): ev["first_rescue"] = round(t, 1)
        if ev["wr_done"] is None and s.wr.done: ev["wr_done"] = round(t, 1)
        if ev["expl_msg"] is None and not s.cp.exploring: ev["expl_msg"] = round(t, 1)
        if ev["fires_out"] is None and all(f["I"] < 0.1 for f in s.w.fires): ev["fires_out"] = round(t, 1)
        if ev["gps_end"] is None and getattr(s.wr, "phase", "lidar") == "lidar": ev["gps_end"] = (round(t, 1), round(math.hypot(*s.wr.r.pos), 2))
        if ev["killed"] is None and s.mesh.lost_reported: ev["killed"] = round(t, 1)
        if tr is not None:
            tr["writer"].append((round(t, 1), round(s.wr.r.pos[0], 2), round(s.wr.r.pos[1], 2), getattr(s.wr, "phase", "lidar")))
            for role in ("ambulance", "firetruck"):
                ag = s.resp.get(role); tr[role].append((round(t, 1), round(ag.r.pos[0], 2), round(ag.r.pos[1], 2), ag.state) if ag is not None else None)
    # ---- detection and localisation against the ground truth (matching radius 3 m)
    truth = [("HUMAN", h["x"], h["y"]) for h in s.w.humans] + [("ANIMAL", a["x"], a["y"]) for a in s.w.animals] + [("FIRE", f["x"], f["y"]) for f in s.w.fires] + [("GAS", g["x"], g["y"]) for g in s.w.gas]
    recs = [r for r in s.cp.records.values() if r["type"] in ("HUMAN", "ANIMAL", "FIRE", "GAS")]; best, fp, dup = {}, 0, 0
    for r in recs:
        x, y = r["tlocal"]; c = [(math.hypot(x - tx, y - ty), i) for i, (k, tx, ty) in enumerate(truth) if k == r["type"]]; d, i = min(c, default=(1e9, None))
        if d <= 3.0:
            if i in best: dup += 1
            best[i] = min(best.get(i, 1e9), d)
        else: fp += 1
    kinds = {k: [i for i, t in enumerate(truth) if t[0] == k] for k in ("HUMAN", "ANIMAL", "FIRE", "GAS")}
    m = s.view.metrics; events = [b for b in s.wr.beacons if b["type"] != "REPEATER"]; lat = [r["rx_t"] - r["t0"] for r in s.cp.records.values() if r["type"] not in ("REPEATER", "BEACON_LOST")]
    upd = sum(1 for e in s.cp.flow if "ORDER_UPDATE" in e["label"]); pre = sum(1 for e in s.cp.flow if "PREEMPT" in e["label"])
    out = dict(seed=seed, final=s.final, end_s=round(s.t, 1), wall_s=round(time.time() - w0, 1), sim_per_wall=round(s.t / max(time.time() - w0, 1e-6), 1),
               fires=len(s.w.fires), fires_out=(m or {}).get("fires_out"), people=(m or {}).get("people"), rescued=(m or {}).get("rescued"), skipped=list((m or {}).get("skipped", [])) if m else [],
               exec_first_person_s=(m or {}).get("with_beacons_s"), blind_s=(m or {}).get("blind_s"), e2e_first_person_s=(m or {}).get("first_person_abs_s"), robots=(m or {}).get("robots", {}),
               beacons=len(s.wr.beacons), event_beacons=len(events), repeaters=s.wr.repeaters, delivered=len(s.cp.records), rx_packets=s.gw.received, rejected=s.gw.rejected,
               mesh_sent=s.mesh.sent, mesh_delivered=s.mesh.delivered, steam_rejected=s.wr.det.steam_rejected, orders=len(s.cp.orders), order_updates=upd, preempts=pre,
               uplink_msgs=len(ups), uplink_reasons={k: sum(1 for u in ups if u[1] == k) for k in {u[1] for u in ups}}, delay_by_class={k: dict(n=len(v), mean=round(st.mean(v), 2), max=round(max(v), 2)) for k, v in delays.items()},
               first_drop_s=min((v[0] for v in ev["drop"].values() if v[1] != "REPEATER"), default=None), first_human_drop_s=min((v[0] for v in ev["drop"].values() if v[1] == "HUMAN"), default=None),
               first_order_s=ev["first_order"], amb_departures=ev["amb_dep"], ft_departures=ev["ft_dep"], arrive_human_s=ev["arrive_h"], first_rescue_s=ev["first_rescue"], writer_done_s=ev["wr_done"],
               exploration_done_at_cp_s=ev["expl_msg"], fires_out_s=ev["fires_out"], gps_end=ev["gps_end"], beacon_lost_s=ev["killed"], writer_dead=s.wr.dead,
               truth_n=len(truth), detected=len(best), recall=round(len(best) / max(len(truth), 1), 3), false_records=fp, duplicate_records=dup,
               pos_err_mean=round(st.mean(best.values()), 3) if best else None, pos_err_max=round(max(best.values()), 3) if best else None, pos_err_by_kind={k: round(st.mean(best[i] for i in ix if i in best), 3) for k, ix in kinds.items() if any(i in best for i in ix)},
               latency_mean_s=round(st.mean(lat), 2) if lat else None, latency_max_s=round(max(lat), 2) if lat else None, event_drop_times=sorted((v[0], v[1]) for v in ev["drop"].values()))
    if trace: out["trace"] = tr; out["drops"] = [(k, *v) for k, v in sorted(ev["drop"].items())]; out["orders_log"] = [(o["t"], o["roles"]) for o in s.cp.orders]
    return out

def save(): json.dump(R, open(OUT, "w"), indent=1, default=str)
def many(label, mp, seeds, **kw):
    out = []
    for sd in seeds: out.append(run_kpi(mp, sd, **kw))
    print(label, "done", flush=True); return out

if __name__ == "__main__":
    t00 = time.time(); S10 = list(range(1, 11)); only = set(sys.argv[1:])            # optional: names of the experiments to run (default: all)
    if os.path.exists(OUT): R.update(json.load(open(OUT)))                           # resumable: whatever is already measured is kept
    R["meta"] = dict(map_arena=os.path.basename(ARENA), map_building=os.path.basename(BUILD), primary="building", seeds=S10, date=time.strftime("%Y-%m-%d"))
    want = lambda k: not only or k in only
    def todo(key, fn):
        if want(key) and key not in R: R[key] = fn(); save()
    def todo_dict(key, items, fn):                                                   # resumable per item (one mode / one radio level)
        if not want(key): return
        d = R.setdefault(key, {})
        for it in items:
            if str(it) not in d: d[str(it)] = fn(it); save()
    def todo_list(key, mp, seeds, **kw):                                             # resumable per seed
        if not want(key): return
        L = R.setdefault(key, []); have = {r["seed"] for r in L}
        for sd in seeds:
            if sd not in have: L.append(run_kpi(mp, sd, **kw)); save(); print(key, "seed", sd, "done", flush=True)
    todo("arena_blind", lambda: blind_baseline(ARENA)); todo("building_blind", lambda: blind_baseline(BUILD))
    todo_list("arena_baseline", ARENA, S10); todo("arena_trace", lambda: run_kpi(ARENA, 7, trace=True))
    todo_dict("arena_faults", ("beacon_destroyed", "writer_lost"), lambda f: many(f, ARENA, range(1, 6), fault=f))
    todo_dict("arena_uplink", ("immediate", "batched"), lambda u: many("uplink " + u, ARENA, range(1, 6), uplink=u))
    todo_dict("arena_nav", ("trail", "direct"), lambda n: many("nav " + n, ARENA, S10, nav=n))
    todo_dict("arena_rf", (1, 6, 12, 18, 24, 30), lambda d: many(f"rf {d} dB", ARENA, range(1, 4), wall_db=d, max_t=300.0))
    todo_dict("arena_hold", ("on", "off"), lambda k: many("hold " + k, ARENA, range(1, 6), hold_off=(k == "off")))
    todo_dict("arena_cycle", (0.5, 1.0, 2.0, 5.0, 10.0), lambda c: many(f"cycle {c}", ARENA, range(1, 4), collect=c))
    todo_list("building_baseline", BUILD, S10); todo("building_trace", lambda: run_kpi(BUILD, 7, trace=True))
    todo_dict("building_faults", ("beacon_destroyed", "writer_lost"), lambda f: many("building " + f, BUILD, range(1, 4), fault=f))
    todo_dict("building_uplink", ("immediate", "batched"), lambda u: many("building uplink " + u, BUILD, range(1, 4), uplink=u))
    todo_dict("building_nav", ("trail", "direct"), lambda n: many("building nav " + n, BUILD, range(1, 5), nav=n))
    todo_dict("building_rf", (12, 24, 30), lambda d: many(f"building rf {d} dB", BUILD, range(1, 3), wall_db=d, max_t=700.0))
    todo_dict("building_hold", ("on", "off"), lambda k: many("building hold " + k, BUILD, range(1, 4), hold_off=(k == "off")))
    R["meta"]["total_wall_s"] = round(time.time() - t00); save(); print("wrote", OUT)
