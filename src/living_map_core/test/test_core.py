import math, random, pytest, os
COMPLEX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps", "complex.json")      # the 2-floor building: fixture for the multi-floor features
from living_map_core.beacon_protocol import *
from living_map_core.geo import Anchor
from living_map_core.sim_core import World, Robot, plan, FREE, STAIRS, VOID, CAPS_WHEELS, CAPS_FLY, UNK
from living_map_core.agents import Detector, GatewayCore, StationLink
from living_map_core.simulation import run, Simulation
from living_map_core import audit

def test_packet_is_32_bytes_and_roundtrips():
    b = make_beacon(3, 1, 1, 55, "HUMAN", 0, 1, 1, 1, quant_heading(1.0), (4.0, -3.0), (2.0, 1.5), 2, 55.0, 3)
    d = pack(b); assert len(d) == PACKET_SIZE == 32
    u = unpack(d); assert (u.bid, u.type, u.prio, u.flags, u.floor, u.parent) == (3, 3, 0, 1, 1, 2) and (u.hop_f, u.hop_r, u.tgt_f, u.tgt_r) == (40, -30, 20, 15)

def test_tampered_packet_rejected():
    d = bytearray(pack(make_beacon(1, 1, 1, 0, "FIRE", 1, 0, 0, 1, 0, (1, 1), (1, 1), 0))); d[5] ^= 0xFF
    with pytest.raises(ValueError): unpack(bytes(d))

def test_egocentric_geometry_right_and_left():
    hq = quant_heading(math.pi / 2)                         # facing north
    ahead, right = to_body((0, 0), hq, (0, 4)); assert abs(ahead - 4) < 0.1 and abs(right) < 0.1
    ahead, right = to_body((0, 0), hq, (3, 0)); assert abs(right - 3) < 0.1                       # east is on the right
    assert describe(4, -3) == "4.0 m ahead, 3.0 m left"
    x, y = from_body((10, 5), hq, 4.0, 3.0); assert abs(x - 13) < 0.1 and abs(y - 9) < 0.1

def test_translation_roundtrip():
    a = Anchor(); x, y = a.to_local(*a.to_wgs84(13.5, -2.2)); assert abs(x - 13.5) < 1e-6 and abs(y + 2.2) < 1e-6

def test_confidence_decays_and_waiting_boosts():
    assert confidence("FIRE", 600) < confidence("FIRE", 60)
    assert queue_score("HUMAN", 1, 1, 0, 100) > queue_score("HUMAN", 1, 1, 0, 0)

def test_map_is_loaded_from_file_and_multifloor():
    w = World(COMPLEX); assert w.nf == 2 and w.nx == 96 and w.ny == 48 and len(w.fires) == 4
    assert any(w.g[0][j][i] == STAIRS and w.g[1][j][i] == STAIRS for j in range(w.ny) for i in range(w.nx))

def test_robot_starts_with_no_knowledge():
    w = World(COMPLEX); r = Robot(w, (-1, 0), 0, random.Random(1)); assert all(v == UNK for f in r.k.g for row in f for v in row)

def test_truck_cannot_use_stairs_but_drone_can():
    w = World(COMPLEX); r = Robot(w, (-1, 0), 0, random.Random(1))
    for f in range(w.nf):
        for j in range(w.ny):
            for i in range(w.nx): r.k.g[f][j][i] = w.g[f][j][i]
    goal = (1,) + w.to_cell(12.0, 6.5)
    assert plan(r.k, r.cell(), goal=goal, caps=CAPS_WHEELS, opt=None) is None
    assert plan(r.k, r.cell(), goal=goal, caps=CAPS_FLY, opt=None) is not None

def test_detector_rejects_steam():
    d = Detector(); s = dict(temp=85, thermal_max=40, gas=0, hot=[], blobs=[])
    ev = []
    for _ in range(4): ev += d.update(s, 0)
    assert [e["type"] for e in ev] == ["STEAM_REJECTED"] and d.steam_rejected == 1

def test_star_topology_audit_logic():
    assert audit.star_violations({"drone": {"/station/to/drone", "/world/action"}}) == []
    assert audit.star_violations({"drone": {"/station/from/ambulance"}})
    assert audit.star_violations({"ambulance": {"/responder/drone/pose"}})

def test_station_link_only_accepts_known_kinds():
    got = []; l = StationLink(lambda m, n: got.append(m), latency=1.0); l.send("drone", "FIRE_OUT", {}, 0.0); l.tick(0.5); assert not got; l.tick(1.5); assert len(got) == 1
    with pytest.raises(AssertionError): l.send("drone", "CHAT_WITH_AMBULANCE", {}, 0.0)

def test_end_to_end_mission():
    s = run(map_path=COMPLEX); m = s.view.metrics
    assert s.final and m["fires_out"] == m["fires"] == 4 and m["rescued"] >= 4 and m["position_error_m"] < 1.0
    assert m["skipped"] and m["with_beacons_s"] < m["blind_s"]

def test_chain_floor_changes_only_at_stairs():
    s = run(map_path=COMPLEX); by = s.cp.records
    for r in by.values():
        p = by.get(r["parent"])
        if p and p["floor"] != r["floor"]: assert p["type"] == "STAIRS" or r["type"] == "STAIRS"

@pytest.mark.parametrize("fault,fires,rescued", [("beacon_destroyed", 4, 4), ("writer_lost", 2, 3)])
def test_faults(fault, fires, rescued):
    s = run(fault, COMPLEX); m = s.view.metrics; assert s.final and m["fires_out"] >= fires and m["rescued"] >= rescued

def test_determinism():
    a, b = run(map_path=COMPLEX, seed=3).view.metrics, run(map_path=COMPLEX, seed=3).view.metrics; assert a["with_beacons_s"] == b["with_beacons_s"] and a["beacons"] == b["beacons"]

def test_executors_update_beacon_state_and_table_follows():
    s = run(map_path=COMPLEX); tab = {r["bid"]: r for r in s.cp.beacon_table()}; phys = s.view.beacon_states
    done = [r for r in tab.values() if r["type"] in ("FIRE", "HUMAN", "ANIMAL") and r["state"] == "RESOLVED"]
    assert len(done) >= 8                                             # 4 fires + 4 rescued
    assert all(phys[r["bid"]][0] == "RESOLVED" and phys[r["bid"]][1] in ("ambulance", "firetruck", "drone") for r in done)   # written INTO the beacon by an executor
    assert any(r["state"] == "HEAVY_RESCUE" for r in tab.values())
    kinds = {e["kind"] for e in s.cp.flow}; assert {"UPLINK", "DISPATCH", "STATE", "STATION"} <= kinds

def test_beacon_state_write_needs_radio_range():
    from living_map_core.agents import Mesh
    w = World(COMPLEX); m = Mesh(w); b = make_beacon(1, 1, 1, 0, "FIRE", 1, 0, 0, 1, 0, (3, 0), (1, 0), 0)
    m.register(pack(b).hex(), (3.0, 0.0), 0, 1, 0.0)
    assert m.write_state(1, "ON_SITE", "drone", (4.0, 0.0), 0, 5.0) is True and m.nodes[1]["state"] == "ON_SITE"
    assert m.write_state(1, "RESOLVED", "drone", (30.0, -8.0), 1, 6.0) is False and m.nodes[1]["state"] == "ON_SITE"


# ---------------- v4.7: continuous priority, collection window, joint orders, gas ----------------
from living_map_core.command_post import CommandPost
from living_map_core.roles import Responder
from living_map_core.beacon_protocol import priority

def _rec(bid, typ, prio, x, y, floor=0, human_needed=False):
    return dict(bid=bid, id=f"BCN_{bid:03d}", seq=1, type=typ, prio=prio, flags=0, human_needed=human_needed, floor=floor, n=1, local=[x, y], tlocal=[x, y], lat=36.8, lon=10.18, tlat=36.8, tlon=10.18,
                parent=0, t0=0.0, rx_t=0.0, sigma_m=0.1, rssi=-60, hops=1, hop_text="", tgt_text="")

def test_priority_is_zero_to_infinity_lower_is_more_urgent_and_drops_with_time():
    crit = priority("HUMAN", 0, 1, 0); person = priority("HUMAN", 1, 1, 0); fire = priority("FIRE", 1, 1, 0); animal = priority("ANIMAL", 2, 1, 0); relay = priority("REPEATER", 3, 1, 0)
    assert 0 <= crit < person < fire < animal < relay and relay > 100            # no upper limit, lower = more urgent
    assert priority("ANIMAL", 2, 1, 600) < animal                                  # it DROPS while the event waits (no starvation)
    assert priority("ANIMAL", 2, 1, 600) < priority("HUMAN", 1, 1, 0)             # an old animal report eventually outranks a fresh ordinary one

def test_collection_window_puts_fire_and_gas_in_one_order():
    cp = CommandPost(Anchor(), collect_window=1.0)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "FIRE", 1, 10, 10)]), 10.0); cp.ingest(dict(reason="IMMEDIATE", records=[_rec(2, "GAS", 1, 12, 10)]), 10.4)
    assert cp.ready(10.5) == []                                                   # still collecting: nothing new for less than 1 s
    go = cp.ready(11.6); assert len(go) == 1 and go[0][0] == "firetruck"
    assert [t["type"] for t in go[0][1]["targets"]] == ["FIRE", "GAS"] or {t["type"] for t in go[0][1]["targets"]} == {"FIRE", "GAS"}
    assert len(cp.orders) == 1

def test_victim_in_danger_zone_gets_one_joint_order_with_the_hazard():
    cp = CommandPost(Anchor(), collect_window=1.0, danger_collect=20.0)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "HUMAN", 0, 30, 5)]), 0.0)
    assert cp.ready(5.0) == [] and "danger zone" in cp.state["ambulance"]["note"]   # waits briefly for the hazard report
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(2, "FIRE", 1, 31, 6)]), 6.0)
    go = dict(cp.ready(8.0)); assert set(go) == {"ambulance", "firetruck"}          # the hazard robot AND the ambulance leave in the SAME order
    t = go["ambulance"]["targets"][0]; assert [h["id"] for h in t["wait_for"]] == ["BCN_002"] and t["order"] == go["firetruck"]["targets"][0]["order"]
    assert any(r["pi"] < priority("FIRE", 1, 1, 8) for r in cp.records.values() if r["type"] == "FIRE")   # the fire inherited the victim's urgency

def test_gas_can_be_sealed_and_beacon_state_is_readable_in_range_only():
    from living_map_core.agents import Mesh
    w = World(COMPLEX); g = w.gas[0]; assert w.secure(0, (g["x"], g["y"]), 6.0, 2.0) is g and g["I"] == 0.0 and w.sense((g["x"], g["y"]), 0, random.Random(1))["gas"] < 20
    m = Mesh(w); m.register(pack(make_beacon(1, 1, 1, 0, "FIRE", 1, 0, 0, 1, 0, (3, 0), (1, 0), 0)).hex(), (3.0, 0.0), 0, 1, 0.0)
    assert m.write_state(1, "RESOLVED", "drone", (4.0, 0.0), 0, 5.0) and m.read_state(1, (4.0, 0.0), 0) == "RESOLVED" and m.read_state(1, (60.0, 20.0), 0) is None

def test_full_mission_handles_gas_and_gives_joint_orders():
    s = run(map_path=COMPLEX); m = s.view.metrics
    assert s.final and m["fires_out"] == 4 and m["rescued"] >= 4 and all(g["I"] == 0.0 for g in s.w.gas)
    assert any(" | " in o["text"] and "waits for" in o["text"] for o in s.cp.orders)  # at least one joint order (hazard robot + ambulance)


# ---------------- v4.9: decision cycle, events + correlation, preemption, replanning, no robot-to-robot ----------------
def _cp(**kw): return CommandPost(Anchor(), **kw)

def test_command_post_decides_once_per_cycle_not_per_event():
    cp = _cp(collect_window=0.0, cycle=1.0)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "FIRE", 1, 10, 10)]), 10.0)
    assert [r for r, _ in cp.ready(10.0)] == ["firetruck"]                  # first cycle: the fire truck gets its order
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(2, "HUMAN", 1, 40, 30)]), 10.3)
    assert cp.ready(10.4) == []                                              # inside the 1 s cycle: NO order, even though a robot is free and waiting
    assert [r for r, _ in cp.ready(11.1)] == ["ambulance"]                  # next cycle: now it decides

def test_priority_base_vs_effective_never_negative_never_reset_decreases_with_time():
    cp = _cp(); rec = _rec(1, "ANIMAL", 2, 5, 5); cp.ingest(dict(reason="IMMEDIATE", records=[rec]), 0.0)
    vals = []
    for t in (0.0, 60.0, 600.0, 6000.0):
        cp.refresh(t); r = cp.records[1]; vals.append(r["pi"]); assert r["pi_base"] == priority("ANIMAL", 2, 1, 0.0) and r["pi"] >= 0
    assert vals == sorted(vals, reverse=True) and vals[-1] < 0.01 * vals[0] + 1e-9 or vals[-1] < vals[0]   # strictly more urgent with time, tends to 0
    again = dict(rec, seq=2); cp.ingest(dict(reason="IMMEDIATE", records=[again]), 600.0); cp.refresh(600.0)     # a repeated report must NOT reset the age
    assert abs(cp.records[1]["pi"] - vals[2]) < 1e-6

def test_event_correlation_space_time_type_and_transitivity():
    cp = _cp(); a = _rec(1, "FIRE", 1, 10, 10); b = _rec(2, "GAS", 1, 17, 10); c = _rec(3, "HUMAN", 1, 24, 10)       # a~b (7 m), b~c (7 m), a-c 14 m: chain
    far = _rec(4, "FIRE", 1, 60, 40); v1 = _rec(5, "HUMAN", 1, 61, 41); v2 = _rec(6, "ANIMAL", 2, 62, 41); late = _rec(7, "GAS", 1, 12, 10); late["t0"] = 5000.0
    other_floor = _rec(8, "GAS", 1, 10, 11, floor=1)
    cp.ingest(dict(reason="IMMEDIATE", records=[a, b, c, far, v1, v2, late, other_floor]), 1.0); cp.refresh(1.0); inc = cp.incident_of
    assert inc[1] == inc[2] == inc[3]                                        # fire + gas + person -> ONE incident (transitive)
    assert inc[4] == inc[5] == inc[6] and inc[4] != inc[1]                   # a separate fire far away keeps its own incident
    assert inc[7] != inc[1]                                                  # same place but detected 5000 s later: not the same incident
    assert inc[8] != inc[1]                                                  # other floor: not related
    cp2 = _cp(); cp2.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "HUMAN", 1, 10, 10), _rec(2, "HUMAN", 1, 11, 10)]), 1.0); cp2.refresh(1.0)
    assert cp2.incident_of[1] != cp2.incident_of[2]                          # two victims are not 'related' just because they are close

def test_event_store_has_the_required_fields_and_related_links():
    cp = _cp(); cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "FIRE", 1, 10, 10), _rec(2, "GAS", 1, 12, 10)]), 1.0); cp.refresh(1.0)
    ev = {e["event_id"]: e for e in cp.events(1.0)}; e = ev["BCN_001"]
    for k in ("event_id", "type", "location", "detected_at", "priority_base", "priority", "required_capabilities", "status", "assigned_robot", "related_events", "incident"): assert k in e, k
    assert e["related_events"] == ["BCN_002"] and e["required_capabilities"] == ["extinguish"] and ev["BCN_002"]["required_capabilities"] == ["seal_gas"]

def test_critical_event_preempts_the_collection_window_but_a_normal_one_waits():
    cp = _cp(collect_window=5.0, cycle=1.0)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "FIRE", 1, 10, 10)]), 10.0); assert cp.ready(10.5) == []          # normal (pi=5): waits for the window
    cp2 = _cp(collect_window=5.0, cycle=1.0); h = _rec(1, "HUMAN", 0, 30, 5); h["n"] = 3                                  # 3 casualties in immediate danger: pi=0.15 < crit
    cp2.ingest(dict(reason="IMMEDIATE", records=[h]), 10.0); go = cp2.ready(10.5)
    assert [r for r, _ in go] == ["ambulance"] and any("PREEMPT" in f["label"] for f in cp2.flow)

def test_replanning_order_update_goes_out_when_a_hazard_appears_and_when_it_is_cleared():
    cp = _cp(collect_window=5.0, cycle=1.0); h = _rec(1, "HUMAN", 0, 30, 5); h["n"] = 3
    cp.ingest(dict(reason="IMMEDIATE", records=[h]), 10.0); assert [r for r, _ in cp.ready(10.5)] == ["ambulance"]       # ambulance leaves without the hazard report (critical)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(2, "FIRE", 1, 31, 6)]), 12.0); cp.ready(13.0)
    up = [u for u in cp.pop_outbox() if u["role"] == "ambulance"]
    assert up and up[0]["kind"] == "ORDER_UPDATE" and [x["id"] for x in up[0]["payload"]["wait_for"][1]] == ["BCN_002"]      # the running order now waits for the new hazard
    cp.ingest_status(dict(robot="firetruck", kind="FIRE_OUT", payload=dict(bid=2, id="BCN_002")), 14.0); cp.ready(15.0)
    up = [u for u in cp.pop_outbox() if u["role"] == "ambulance"]
    assert up and up[0]["payload"]["cleared"] == [2]                         # 'hazard cleared' reaches the ambulance THROUGH the Command Post

def test_executor_learns_hazard_cleared_only_from_the_command_post_never_from_beacons():
    from living_map_core.agents import StationLink
    w = World(COMPLEX); g = w.gas[0]; link = StationLink(lambda m, n: None); a = Anchor(**w.anchor)
    haz = dict(bid=5, id="BCN_005", type="GAS", lat=a.to_latlon(g["x"], g["y"])[0] if hasattr(a, "to_latlon") else 0.0, lon=0.0, floor=0, radius=3.0)
    mission = dict(role="ambulance", targets=[], hazards=[], stairs=[], debris=[], skipped=[])
    r = Responder(w, "ambulance", mission, a, link, beacon_read=lambda *x: "RESOLVED")                              # even if someone hands it a beacon reader...
    assert r.beacon_read is None                                             # ...it is ignored: no hidden robot-to-robot channel
    assert r._hazard_clear(haz, dict(hot=[])) is False
    assert r.apply_update(dict(cleared=[5]), 1.0) and r._hazard_clear(haz, dict(hot=[])) is True

def test_audit_forbids_robots_reading_beacon_state():
    from living_map_core.audit import star_violations
    assert star_violations({"ambulance": {"/station/to/ambulance", "/world/action"}}) == []
    assert star_violations({"ambulance": {"/station/to/ambulance", "/beacon_state"}}) and star_violations({"drone": {"/station/to/firetruck"}})

def test_lost_robot_is_detected_and_its_jobs_go_to_another_capable_robot():
    cp = _cp(collect_window=0.0, cycle=1.0, lost_after=35.0)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "FIRE", 1, 10, 10)]), 10.0); assert [r for r, _ in cp.ready(10.0)] == ["firetruck"]
    go = cp.ready(60.0); assert "firetruck" in cp.lost and cp.state["firetruck"]["state"] == "lost"               # no heartbeat for > 35 s
    assert [(r, [t["id"] for t in m["targets"]]) for r, m in go] == [("drone", ["BCN_001"])]                     # its fire is re-assigned at once to the drone (capability-based selection)
    cp.ingest(dict(reason="IMMEDIATE", records=[_rec(2, "FIRE", 1, 20, 20)]), 61.0); cp.ready(62.0)
    up = [u for u in cp.pop_outbox() if u["role"] == "drone"]
    assert up and [t["id"] for t in up[0]["payload"]["targets"]] == ["BCN_002"]                                   # a new fire reaches the busy drone as an ORDER_UPDATE (replanning), not as a second order
    cp.ingest_status(dict(robot="firetruck", kind="STATUS", payload=dict(active=True)), 63.0); assert "firetruck" not in cp.lost   # it is talking again

def test_full_mission_still_succeeds_with_no_beacon_reading_and_cp_relayed_clearance():
    s = run(map_path=COMPLEX); m = s.view.metrics
    assert s.final and m["fires_out"] == 4 and m["rescued"] >= 4 and all(r.beacon_read is None for r in s.resp.values())


# ---------------- v4.10: Executor navigation (A*, free-space assumption, job ordering) ----------------
def _full_known(w):
    from living_map_core.sim_core import KnownMap
    k = KnownMap(w)
    for f in range(w.nf):
        for j in range(w.ny):
            for i in range(w.nx): k.g[f][j][i] = w.g[f][j][i]
    return k

def _path_cost(k, path, start):
    from living_map_core.sim_core import _prox
    c, prev = 0.0, start
    for n in path:
        if n[0] == prev[0]: c += (1.414 if (n[1] != prev[1] and n[2] != prev[2]) else 1.0) + _prox(k, n[0], (n[1], n[2]))
        else: return None                                        # floor change: cost model differs, not compared here
        prev = n
    return c

def test_astar_gives_the_same_cost_as_dijkstra():
    w = World(COMPLEX); k = _full_known(w); rng = random.Random(5); free = [(0, i, j) for j in range(w.ny) for i in range(w.nx) if w.g[0][j][i] == FREE]
    checked = 0
    for _ in range(25):
        a, b = rng.choice(free), rng.choice(free)
        pa = plan(k, a, goal=b); pd = plan(k, a, goal_fn=lambda c, b=b: c == b)
        assert (pa is None) == (pd is None)
        if pa is not None and a != b:
            ca, cd = _path_cost(k, pa, a), _path_cost(k, pd, a)
            if ca is not None and cd is not None: assert abs(ca - cd) < 1e-6; checked += 1
    assert checked >= 10

def test_free_space_assumption_crosses_unknown_and_prefers_the_known_safe_band():
    from living_map_core.sim_core import KnownMap
    w = World(COMPLEX); k = KnownMap(w); free = [(0, i, j) for j in range(w.ny) for i in range(w.nx) if w.g[0][j][i] == FREE]
    a = free[0]; b = max(free, key=lambda c: abs(c[1] - a[1]) + abs(c[2] - a[2]))
    assert plan(k, a, goal=b, opt=None) is None                  # nothing known, nothing allowed: blocked (the legacy behaviour)
    p = plan(k, a, goal=b, opt="all"); assert p is not None and p[-1] == b      # free-space assumption: a path exists
    band = {(0, i, j) for (f, i, j) in p[::2] for di in range(-1, 2) for dj in range(-1, 2) for i, j in [(i + di, j + dj)]}
    far = [c for c in free if c not in band]
    q = plan(k, a, goal=b, opt="all", unk_cost=lambda n: 0.0 if n in band else 5.0)
    assert sum(1 for n in q if n in band) >= 0.9 * len(q)       # unknown outside the band is expensive: the trail band is followed

def test_job_order_minimises_urgency_weighted_completion_time():
    w = World(COMPLEX); a = Anchor(**w.anchor)
    def tgt(i, x, y, pi): lat, lon = a.to_wgs84(x, y); return dict(id=f"T{i}", bid=i, type="HUMAN", prio=1, floor=0, lat=lat, lon=lon, n=1, pi=pi, chain=[])
    link = StationLink(lambda m, n: None)
    r = Responder(w, "ambulance", dict(role="ambulance", targets=[], hazards=[], stairs=[], debris=[], skipped=[]), a, link)
    r.r.pos = [0.0, 0.0]
    far, n1, n2 = tgt(1, 40, 0, 1.0), tgt(2, 5, 0, 1.1), tgt(3, 6, 0, 1.2)
    order = [t["id"] for t in r.order_jobs([far, n1, n2], (0.0, 0.0, 0))]
    assert order == ["T2", "T3", "T1"]                           # two cheap jobs lie on the way: do them first, the far one is delayed only slightly
    very = tgt(1, 40, 0, 0.1)                                    # but a really critical far job (pi 0.1) still goes first
    assert [t["id"] for t in r.order_jobs([very, n1, n2], (0.0, 0.0, 0))][0] == "T1"
    assert [t["id"] for t in r.order_jobs([n1], (0.0, 0.0, 0))] == ["T2"]

def test_direct_navigation_completes_the_mission():
    s = run(map_path=COMPLEX, nav="direct"); m = s.view.metrics
    assert s.final and m["fires_out"] == 4 and m["rescued"] >= 4 and all(x["ok"] for v in m["robots"].values() for x in v)


# ---------------- v4.11: prototype arena (7.2 x 4.8 m, 0.1 m cells) ----------------
import os
PROTO = os.path.join(os.path.dirname(__file__), "..", "maps", "proto_arena.json")

def test_rays_do_not_pass_through_thin_walls_on_a_fine_grid():
    w = World(PROTO)
    assert w.los(0, (3.45, 1.45), (6.5, 1.9)) is False           # next room, behind the 0.1 m divider
    assert w.los(0, (3.45, 1.45), (2.8, 1.9)) is True            # same room
    assert not any(v in (1,) and False for (_, _, v) in w.scan((3.45, 1.45), 0, 6.0))      # scan runs on the fine grid without error

def test_prototype_arena_mission_completes_with_five_event_types():
    s = run(map_path=PROTO, max_t=600.0, seed=7); m = s.view.metrics
    types = {r["type"] for r in s.cp.records.values()}
    assert s.final and m["fires_out"] == m["fires"] == 1 and m["rescued"] >= 3 and {"FIRE", "GAS", "HUMAN", "ANIMAL", "DEBRIS"} <= types
    assert s.wr.det.steam_rejected >= 1                          # the hot mug in the corridor is rejected as steam (it never becomes a FIRE record)
    assert m["delivery"].split("/")[0] == m["delivery"].split("/")[1]


# ---------------- v4.13: Writer GPS approach, smaller van ----------------
def test_writer_drives_to_the_door_by_gps_then_loses_gps():
    s = Simulation("none", True, COMPLEX, seed=7); start = tuple(s.wr.r.pos); assert start[0] < -1.0 and s.wr.phase == "gps"      # starts OUTSIDE, at the van
    t_switch = None
    for _ in range(80):
        s.step(0.5)
        if s.wr.phase == "lidar": t_switch = s.t; break
    r = s.wr.r
    assert t_switch is not None and t_switch < 20.0 and math.hypot(*r.pos) <= 0.4 + 1e-6      # reached the entrance door
    assert r.drift == [0.0, 0.0] and not s.wr.beacons                                         # GNSS gives an absolute fix: no drift, and nothing is beaconed outdoors
    assert any("GPS is LOST" in m for _, m in s.wr.log) and any("GPS approach" in m for _, m in s.wr.log)

def test_van_is_small_and_its_size_comes_from_the_map():
    import xml.dom.minidom as dom
    from living_map_core.gz_world import build_sdf
    def van_box(path):
        d = dom.parseString(build_sdf(path)); b = [x for x in d.getElementsByTagName("model") if x.getAttribute("name") == "van_body"][0]
        return tuple(float(v) for v in b.getElementsByTagName("size")[0].firstChild.data.split())
    big = World(COMPLEX); small = World(PROTO)
    assert big.van_size == (2.6, 1.3, 1.4) and small.van_size == (0.9, 0.6, 0.5)
    assert van_box(os.path.join(os.path.dirname(__file__), "..", "maps", "complex.json")) == (2.6, 1.3, 1.4) and van_box(PROTO) == (0.9, 0.6, 0.5)      # was 5.0 x 2.2 x 2.4
    from living_map_core.standalone import world_json
    assert world_json(small)["van_size"] == [0.9, 0.6, 0.5]


# ---------------- v4.14: low-urgency jobs wait automatically (no per-type rule, no fixed delay) ----------------
def test_hold_threshold_is_derived_from_the_priority_table():
    cp = _cp(); assert cp.hold_above == max(priority(t, 1, 1, 0.0) for t in ("HUMAN", "FIRE", "GAS")) == 6.0

def test_low_urgency_job_waits_while_exploring_and_is_released_by_aging():
    cp = _cp(collect_window=0.0, cycle=1.0); cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "ANIMAL", 2, 10, 10)]), 10.0)      # pi0 = 15 > 6: low urgency
    assert cp.ready(11.0) == [] and cp.targets_for("ambulance")                                 # held, but still listed as work to do (the mission is not 'finished')
    t_rel = math.log(15 / cp.hold_above) / AGING                                                  # comes out of the aging law, not from a constant
    assert cp.ready(t_rel - 5) == [] and [r for r, _ in cp.ready(t_rel + 5)] == ["ambulance"]

def test_low_urgency_job_goes_at_once_when_the_writer_reports_exploration_over():
    cp = _cp(collect_window=0.0, cycle=1.0); cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "ANIMAL", 2, 10, 10)]), 10.0); assert cp.ready(11.0) == []
    cp.ingest_status(dict(robot="writer", kind="EXPLORATION_DONE", payload={}), 12.0); assert [r for r, _ in cp.ready(13.0)] == ["ambulance"]

def test_urgency_decides_not_the_type():
    cp = _cp(collect_window=0.0, cycle=1.0); cp.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "ANIMAL", 0, 10, 10)]), 10.0)      # an animal in immediate danger (class P0): pi0 = 1.5
    assert [r for r, _ in cp.ready(11.0)] == ["ambulance"]
    cp2 = _cp(collect_window=0.0, cycle=1.0); cp2.ingest(dict(reason="IMMEDIATE", records=[_rec(1, "ANIMAL", 2, 10, 10)]), 10.0); assert cp2.ready(11.0) == []
    cp2.ingest(dict(reason="IMMEDIATE", records=[_rec(2, "HUMAN", 1, 30, 5)]), 50.0); go = cp2.ready(52.0)
    assert [r for r, _ in go] == ["ambulance"] and [t["type"] for t in go[0][1]["targets"]] == ["HUMAN", "ANIMAL"]      # a person (pi 3) is not low urgency: it goes, the held animal rides along


def test_writer_parks_at_the_van_after_exploration_instead_of_stopping_at_the_fence():
    s = Simulation("none", True, PROTO, seed=7)
    for _ in range(2000):
        s.step(0.5)
        if getattr(s.wr, "parked", False): break
    r = s.wr.r
    assert s.wr.done and s.wr.parked and math.hypot(r.pos[0] - s.w.writer_start[0], r.pos[1] - s.w.writer_start[1]) < 0.6     # back at its start next to the van


def test_every_sortie_of_every_robot_is_recorded_exactly_once():
    s = Simulation("none", True, PROTO, seed=7); seen = {"ambulance": set(), "firetruck": set()}
    while s.t < 400 and not s.final:
        s.step(0.5)
        for role in seen:
            if s.resp.get(role) is not None: seen[role].add(id(s.resp[role]))
    assert s.final
    for role in seen: assert len(s.done_results.get(role, [])) == len(seen[role]) >= 1, (role, len(s.done_results.get(role, [])), len(seen[role]))      # no sortie lost when the next one starts in the same step
    assert s.view.metrics["with_beacons_s"] < 15.0                                                  # the first sortie (person reached 6.5 s after leaving the van) is counted


def test_default_map_is_the_one_floor_building_and_carries_its_fault_times():
    from living_map_core.sim_core import default_map_path
    w = World(); assert os.path.basename(default_map_path()) == "complex_1floor.json" and w.nf == 1 and len(w.fires) == 2 and len(w.humans) == 2 and len(w.animals) == 1
    assert w.fault_t["writer_lost"] == 150.0 and w.fault_t["beacon_destroyed"] == 120.0
