"""Regenerates README.md, CHANGELOG.md, docs/FAILURE_CASES.md, docs/VIDEO_SCRIPT.md and the arena table of docs/PROTOTYPE_ARENA.md from docs/results_final.json
(written by tools/collect_final.py), then the repository guide.  The PRIMARY map is the one-floor building (maps/complex_1floor.json); the 7.2 x 4.8 m arena is the
Phase 2 prototype map.  Usage: python3 tools/refresh_final_docs.py"""
import json, os, subprocess, sys, statistics as st
H = os.path.dirname(os.path.abspath(__file__)); DOCS = os.path.join(H, "..", "docs"); WS = os.path.join(H, "..", "..", "..")
R = json.load(open(os.path.join(DOCS, "results_final.json"))); P = "building"
mean = lambda v: st.mean(v) if v else float("nan"); sd = lambda v: st.pstdev(v) if len(v) > 1 else 0.0
ms = lambda v, nd=1: f"{mean(v):.{nd}f} +/- {sd(v):.{nd}f}"
def g(k, rs): return [r[k] for r in rs if r[k] is not None]
B = R[P + "_baseline"]; N = len(B); blind = R[P + "_blind"]["time"]; people = B[0]["people"]
AB = R["arena_baseline"]; NA = len(AB); ablind = R["arena_blind"]["time"]
def w_delay(runs, cls):
    xs = [(r["delay_by_class"][cls]["mean"], r["delay_by_class"][cls]["n"]) for r in runs if cls in r["delay_by_class"]]; mx = [r["delay_by_class"][cls]["max"] for r in runs if cls in r["delay_by_class"]]
    return (sum(m * n for m, n in xs) / max(sum(n for _, n in xs), 1), max(mx) if mx else 0.0)
def totals(runs): return dict(first=mean(g("exec_first_person_s", runs)), dist=mean([sum(x["dist"] for v in r["robots"].values() for x in v) for r in runs]), busy=mean([sum(x["time"] for v in r["robots"].values() for x in v) for r in runs]),
                              missions=mean([sum(len(v) for v in r["robots"].values()) for r in runs]), end=mean(g("end_s", runs)))
dep1 = [r["amb_departures"][0][0] for r in B if r["amb_departures"]]; ft1 = [r["ft_departures"][0][0] for r in B if r["ft_departures"]]
gps = [r["gps_end"] for r in B if r["gps_end"]]; gpsT = mean([x[0] for x in gps])
kinds = {}
for r in B:
    for k, v in r["pos_err_by_kind"].items(): kinds.setdefault(k, []).append(v)
sealed_flag = sum(1 for r in B if r["skipped"]); fires_all = all(r["fires_out"] == r["fires"] for r in B); resc = sorted({r["rescued"] for r in B})
UI, UB = R[P + "_uplink"]["immediate"], R[P + "_uplink"]["batched"]; msgI, msgB = mean(g("uplink_msgs", UI)), mean(g("uplink_msgs", UB))
dI = {c: w_delay(UI, c) for c in ("P0", "P1", "P2", "P3")}; dB = {c: w_delay(UB, c) for c in ("P0", "P1", "P2", "P3")}
FB, FW = R[P + "_faults"]["beacon_destroyed"], R[P + "_faults"]["writer_lost"]; BRF = R[P + "_rf"]; BH = R[P + "_hold"]; BN = R[P + "_nav"]
TN, TD = totals(BN["trail"]), totals(BN["direct"]); RFA = R["arena_rf"]; NAA = R["arena_nav"]
tr = R[P + "_trace"]; dr = {}
for d in tr["drops"]: dr.setdefault(d[2], []).append((d[1], d[0]))
fl_ = lambda typ: dr[typ][0][0] if typ in dr else float("nan")
reach_t = dr["HUMAN"][-1][0] if "HUMAN" in dr else float("nan")                    # the last person found in the traced run is the reachable one
hold_f1 = lambda rs: mean([r["amb_departures"][0][0] for r in rs if r["amb_departures"]])
afl = [r for r in AB if r["skipped"]]; anf = [r for r in AB if not r["skipped"]]; afail = mean([x["time"] for r in anf for x in r["robots"].get("ambulance", []) if not x["ok"]]) if anf else float("nan")
steamA = mean(g("steam_rejected", AB)); wdone = mean(g("writer_done_s", B)); eend = mean(g("end_s", B)); exec_ = mean(g("exec_first_person_s", B)); e2e = mean(g("e2e_first_person_s", B))

# =================================================================================================== FAILURE_CASES.md
def rf_tab(RF, lvl_people):
    rows = []
    for d in sorted(RF, key=int):
        rs = RF[d]; rows.append(f"| {d} dB | {sum(r['fires_out'] or 0 for r in rs)} of {sum(r['fires'] for r in rs)} | {mean([(r['rescued'] or 0) for r in rs]):.1f} of {lvl_people} | {mean(g('repeaters', rs)):.1f} | {mean(g('delivered', rs)):.1f} of {mean(g('beacons', rs)):.1f} | {mean(g('end_s', rs)):.0f} s |")
    return "| loss per wall | fires out (all seeds) | victims rescued (mean) | repeaters | beacons delivered (of dropped) | mission end |\n|---|---|---|---|---|---|\n" + "\n".join(rows)
fc = f"""# Failure cases: analysis and measured behaviour (one-floor building, 48 x 24 m)

All numbers come from `python3 tools/collect_final.py` (simulation of `maps/complex_1floor.json`; {N} seeds for the baseline, 3 per fault, 2 per radio level; raw data in `docs/results_final.json`). Times are simulated seconds. The 7.2 x 4.8 m prototype arena of Phase 2 is measured too and noted where it differs.

| # | Failure case | Mitigation in the system | Measured outcome (building) |
|---|---|---|---|
| 1 | Beacon destroyed (at 120 s) | mirror frames; a neighbour re-broadcasts the lost beacon as `BEACON_LOST` | loss reported {mean(g('beacon_lost_s', FB)) - 120:.1f} s later; mission unchanged: {mean(g('fires_out', FB)):.0f} of {FB[0]['fires']} fires out in every run, {mean(g('rescued', FB)):.1f} of {people} rescued |
| 2 | Writer lost (at 150 s) | the van forwards each record at once; the Command Post keeps the memory; Executors act on it | **{mean(g('fires_out', FW)):.0f} of {FW[0]['fires']} fires out in every run, {mean(g('rescued', FW)):.1f} of {people} rescued** (baseline {mean(g('rescued', B)):.1f}); detection recall {mean(g('recall', FW)):.2f}: the east part was never explored |
| 3 | Priority inversion, steam, batching | 2-of-3 rule rejects steam; class P0 bypasses the queue; critical priority skips the wait | **{sum(r['false_records'] for r in B)} false records** in {N} runs; batched mode: P0 delay {dB['P0'][0]:.2f} s, P1 {dB['P1'][0]:.1f} s, P3 {dB['P3'][0]:.0f} s (max {dB['P3'][1]:.0f} s) |
| 4 | Impassable debris | `debris` beacon with `human_intervention_required`; Executors skip it | sealed victim flagged up-front in {sealed_flag}/{N} runs; no robot stuck |
| 5 | RF attenuation | repeater beacons, ARQ x3, store-and-forward | fine up to 24 dB per wall; **fails at 30 dB in one of two seeds** (see below) |

## 1. Beacon destroyed
Every beacon carries the id and sequence of a neighbour (a *mirror frame*). When a node stays silent, a neighbour re-broadcasts the stored record as `BEACON_LOST` (id 1000 + beacon id, class P3). The busiest relay is destroyed at 120 s; the Command Post learns of it {mean(g('beacon_lost_s', FB)) - 120:.1f} s later (15 s of silence are required) and keeps the last known data; the mission outcome equals the baseline. Reproduce: `bash scripts/run_clean.sh speed:=1 fault:=beacon_destroyed`.

## 2. Writer lost: the single point of failure
The Writer is killed at 150 s (map parameter `fault_times`). Everything it reported before is already at the Command Post, so the Executors finish what is known: the fire truck and the ambulance still act on the gas leak, the animal and what the Writer had found. But **both fires lie in the east part of the building that the Writer had not yet reached**, so none is extinguished and only {mean(g('rescued', FW)):.1f} of {people} victims is rescued (recall {mean(g('recall', FW)):.2f}). The Command Post never receives `EXPLORATION_DONE`, so low-urgency jobs are released by priority aging. This is the honest cost of losing the only scout; a second Writer, or an Executor that also maps, would remove it (Phase 2). Reproduce: `... fault:=writer_lost`.

## 3. Priority inversion, steam and batching
* **Steam.** The detector needs two of three sensor votes for three consecutive readings; a steam source heats the air but has no hot thermal signature, so it is counted as `steam_rejected` and never beaconed. In the building runs the Writer never passes close to the steam source ({mean(g('steam_rejected', B)):.1f} rejections per run), so the rejection is shown by the unit test `test_detector_rejects_steam` and by the prototype arena (rejected {steamA:.1f} time(s) per run, {sum(r['false_records'] for r in AB)} false records in {NA} runs).
* **Batching delay** (uplink messages per run: immediate {msgI:.1f}, batched {msgB:.1f}; 3 seeds each). Mean / max delay at the Command Post by class:

| class | immediate (s) | batched (s) |
|---|---|---|
""" + "\n".join(f"| {c} | {dI[c][0]:.2f} / {dI[c][1]:.2f} | {dB[c][0]:.2f} / {dB[c][1]:.2f} |" for c in ("P0", "P1", "P2", "P3")) + f"""

Class P0 is never held. Mean mission end: immediate {mean(g('end_s', UI)):.0f} s, batched {mean(g('end_s', UB)):.0f} s. Immediate mode is the default; batching is the bandwidth-saving option required by the challenge text. Reproduce: `... uplink:=batched`.
* **Preemption** (an event with priority number below 0.3 skips the collection wait) is covered by unit tests, not exercised by these scenarios.

## 4. Impassable debris
The Writer marks rubble on the line of sight to a person as `debris` and flags the person `human_intervention_required`; the Command Post lists the victim as `HEAVY_RESCUE` and dispatches nobody. In the building this happens in **{sealed_flag} of {N} runs** and the victim is always handed to the human team. On the small arena it happens in only {len(afl)} of {NA} runs: elsewhere the Writer sees the victim from a place with no rubble on the line of sight, the ambulance is sent, finds no route and reports `UNREACHABLE` after about {afail:.0f} s, and the mission lasts {mean(g('end_s', anf)):.0f} s instead of {mean(g('end_s', afl)):.0f} s. Phase 2 improvement: let the Writer check on its own map whether a person can be reached before reporting them.

## 5. RF attenuation (where the design fails)
Radio model: RSSI = -40 - 28 log10(d) - (loss per wall) x walls - 4 dB per rubble pile - 18 dB per floor. The baseline uses 12 dB per wall (assumed). NIST 1997 data at 2.4 GHz (as tabulated by Wi-Fi Vitae): plywood about 1 dB, brick about 6, concrete 102 mm about 15, concrete 203 mm about 29, reinforced concrete 203 mm about 31.

**Building (2 seeds per level, mission capped at 700 s).** The Writer covers long distances behind many walls and every repeater spends part of its budget of 44 beacons.

{rf_tab(BRF, people - 1)}

At 30 dB per wall one seed still completes, but in the other too little reaches the Command Post: the fires stay unknown and no victim is rescued. **The design therefore fails under strong attenuation, as it would in heavy concrete (29 to 31 dB).**

**Prototype arena (3 seeds per level, capped at 300 s).** The arena is small, so the Writer can always drop enough repeaters: every mission completes up to 30 dB per wall.

{rf_tab(RFA, 3)}

Phase 2 mitigations (not implemented): a sub-GHz LoRa link, more relay nodes, and a beacon budget that reserves capacity for events. The prototype arena includes one foil-lined wall to measure this on hardware. Reproduce: set `"wall_db": 24` in a map JSON.

## What these tests do *not* show
* Everything is simulation; the radio parameters are assumed values, not measurements.
* The steam rejection is not exercised on the building, and preemption only by unit tests.
"""
open(os.path.join(DOCS, "FAILURE_CASES.md"), "w").write(fc)

# =================================================================================================== README.md
ar_tr, ar_dr = totals(NAA["trail"]), totals(NAA["direct"])
readme = f"""# The Living Map: spatial memory for emergency robots (IEEE TSYP14, Fire / Hazardous Building, one floor)

A **Writer** robot drives to the entrance of a GPS-denied burning building **by GPS**, then explores the interior without GPS and drops 32-byte radio beacons. A **van** (Outside Network Area) is the only link between inside and outside: it decodes the beacons, converts their private coordinates to GPS and forwards them to the **Command Post**, which ranks the events, groups related ones into single orders and briefs the **Executor** robots (ambulance, fire truck). Robots never talk to each other or to the Command Post directly.

![architecture](src/living_map_core/docs/diagrams/architecture.png)

## Run it (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic)
The default map is the **one-floor building** (`maps/complex_1floor.json`, 48 x 24 m: 2 fires, a gas leak, a steam source, 2 people, an animal, rubble). Put the folder at `~/living_map_ws`.
```bash
cd ~/living_map_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
bash src/living_map_core/scripts/run_clean.sh speed:=1 2>&1 | tee ~/launch.log      # full system: ROS 2 + Gazebo + RViz
bash src/living_map_core/scripts/check_run.sh                                      # after about 60 s
```
A mission lasts about {eend:.0f} simulated seconds; use `speed:=3` to see it in about {eend / 3 / 60:.1f} minutes.
Pages: `http://localhost:8081` simulation, `http://localhost:8080` Command Post, `http://localhost:8081/flow` data flow and beacon table.
Without ROS: `cd src/living_map_core && python3 -m living_map_core.standalone --speed 3`. Gazebo world alone: `gz sim -r -v 2 src/living_map_core/worlds/complex_1floor.sdf`.
The Phase 2 prototype arena (7.2 x 4.8 m, buildable in plywood): add `map:=$HOME/living_map_ws/src/living_map_core/maps/proto_arena.json`.
Tests: `cd src/living_map_core && python3 -m pytest test/test_core.py -q`.

## Results (simulation of the building, {N} seeds; `python3 tools/collect_final.py`)
| measure | value |
|---|---|
| events detected / false records | recall {mean(g('recall', B)) * 100:.0f} % / {sum(r['false_records'] for r in B)} false records in {N} runs |
| target position error | {mean(g('pos_err_mean', B)):.2f} m mean, {max(g('pos_err_max', B)):.2f} m max (by kind: {', '.join(f'{k.lower()} {mean(v):.2f} m' for k, v in sorted(kinds.items()))}) |
| beacons | {mean(g('beacons', B)):.1f} per run ({mean(g('event_beacons', B)):.0f} events + {mean(g('repeaters', B)):.1f} relays), {sum(r['delivered'] for r in B) / sum(r['beacons'] for r in B) * 100:.0f} % delivered, delivery delay below one second |
| orders | {mean(g('orders', B)):.1f} orders and {mean(g('order_updates', B)):.1f} order updates for {mean(g('event_beacons', B)):.0f} event beacons; {msgI:.1f} uplink messages (immediate) or {msgB:.1f} (batched) |
| fires out / victims rescued | {'all runs' if fires_all else 'not all runs'} ({B[0]['fires']} of {B[0]['fires']}) / {', '.join(str(x) for x in resc)} of {people}: the victim sealed behind rubble is flagged up-front in {sealed_flag} of {N} runs and handed to the human team |
| first person: blind Executor / briefed Executor / end to end | {blind:.0f} s / {ms(g('exec_first_person_s', B))} s / {ms(g('e2e_first_person_s', B))} s |
| timeline (mean of {N} runs) | Writer reaches the door by GPS in {gpsT:.1f} s; first order at {mean(g('first_order_s', B)):.0f} s (fire truck, gas leak); ambulance leaves at {mean(dep1):.0f} s; Writer finishes at {wdone:.0f} s; fires out at {mean(g('fires_out_s', B)):.0f} s; mission {ms(g('end_s', B))} s |
| faults (3 seeds) | beacon destroyed: reported {mean(g('beacon_lost_s', FB)) - 120:.0f} s later, mission unchanged; **Writer lost at 150 s: {mean(g('fires_out', FW)):.0f} of {FW[0]['fires']} fires out in every run, {mean(g('rescued', FW)):.1f} of {people} rescued** |
| radio attenuation (2 seeds per level) | fine at 12 and 24 dB per wall; **at 30 dB one seed fails completely** (0 of 2 fires out, nothing rescued) |
| navigation: direct A* vs replaying the trail ({len(BN['direct'])} seeds) | first person {TD['first']:.0f} vs {TN['first']:.0f} s, distance {TD['dist']:.0f} vs {TN['dist']:.0f} m, busy time {TD['busy']:.0f} vs {TN['busy']:.0f} s |
| hold rule (3 seeds) | the ambulance leaves at {hold_f1(BH['on']):.0f} s with the rule and at {hold_f1(BH['off']):.0f} s without it (it no longer leaves for an animal before a person is known); mission {mean(g('end_s', BH['on'])):.0f} s vs {mean(g('end_s', BH['off'])):.0f} s; victims rescued {mean(g('rescued', BH['on'])):.1f} vs {mean(g('rescued', BH['off'])):.1f} |
| simulation speed | {mean(g('sim_per_wall', B)):.0f} times faster than real time |
| Phase 2 prototype arena ({NA} seeds) | recall {mean(g('recall', AB)) * 100:.0f} %, {mean(g('pos_err_mean', AB)):.2f} m error, 3 of 4 rescued, mission {ms(g('end_s', AB))} s; blind {ablind:.0f} s vs briefed {mean(g('exec_first_person_s', AB)):.1f} s; direct A* is longer than the trail there ({ar_dr['dist']:.0f} vs {ar_tr['dist']:.0f} m); every mission completes up to 30 dB per wall |

**Read the timing honestly.** On the building a blind Executor needs {blind:.0f} s to reach a person; a briefed one needs {exec_:.0f} s ({(1 - exec_ / blind) * 100:.0f} % less). But end to end the first person is reached after {e2e:.0f} s, because the Writer must explore first (the first reachable person is found at about {reach_t:.0f} s in the traced run): the chain is **slower than a lucky blind search**. Its value is that no robot searches blind inside a hazard, that one Writer pass serves every Executor, that the gas leak is sealed and the fires are known before a robot enters, that a sealed victim is handed to the human team, and that the memory survives the Writer, except for what the Writer has not yet seen (see the Writer-lost fault).

## Where to read more
| File | Content |
|---|---|
| `REPOSITORY_GUIDE.md` | architecture and the content of every file |
| `src/living_map_core/docs/ARCHITECTURE.md` | modules, ROS graph, message schemas, rules the code enforces |
| `src/living_map_core/docs/FAILURE_CASES.md` | five failure cases with measured results |
| `src/living_map_core/docs/PHASE2_IMPLEMENTATION_PLAN.md` | timeline, bill of materials, physical test protocol |
| `src/living_map_core/docs/PROTOTYPE_ARENA.md` | the plywood arena and the specification check |
| `src/living_map_core/docs/VIDEO_SCRIPT.md`, `SUBMISSION_CHECKLIST.md` | demo video and submission |
| `CHANGELOG.md` | what changed between versions, including fixed measurement errors |
| `TROUBLESHOOTING.md` | when something does not start |

## Limits (read before you trust a number)
All results are from the simulator, which runs the same code as the ROS 2 nodes. Radio loss, GNSS error, sensor noise and thermal behaviour are assumed values to be measured on hardware. The robot-to-van link is an ideal channel (protocol to be defined). The Gazebo world is generated from the same map and has been validated only as well-formed XML. The code also supports several floors (stairs, a drone role), which is outside the evaluated one-floor scope.
"""
open(os.path.join(WS, "README.md"), "w").write(readme)
open(os.path.join(WS, "CHANGELOG.md"), "w").write("""# Changelog

* **v4_16 (this version)**: the one-floor **building** (`maps/complex_1floor.json`) is the default map and Gazebo world; the 7.2 x 4.8 m arena is the Phase 2 prototype map (`map:=.../proto_arena.json`). `tools/make_1floor.py` derives the building from the 2-floor map (geometry unchanged) and adds `fault_times`. All documents are regenerated from `docs/results_final.json`.
* v4_15: fault times are a map parameter; `beacon_destroyed` falls back to the oldest beacon when none has children; **fixed a measurement error**: a sortie that ended in the same simulation step as the next one began was not recorded, which hid the first mission of a robot (the Executor's first-person time was over-reported). Numbers measured before this version are obsolete.
* v4_14: automatic hold of low-urgency jobs while the Writer explores (threshold computed from the priority table), new station message `EXPLORATION_DONE`, the Writer drives back to the van after exploring.
* v4_13: the Writer starts at the van and drives to the door by GPS; smaller, configurable van (`van_size`); the web page follows the loaded map.
* v4_11: prototype arena generated from one definition (map, Gazebo world, plan, material list); detection thresholds and several distances are per-map parameters; rays no longer pass through thin walls on fine grids.
* v4_10: Executors plan directly to the target (A* with a free-space assumption) and order their jobs by urgency-weighted completion time.
* v4_9: Command Post with a configurable decision cycle, incident correlation, preemption, order updates through the van and robot-loss handling; robots no longer read state written by other robots.
* v4_8: fixed a logging call that crashed the van on real ROS 2 (one severity per call site).
""")

# =================================================================================================== VIDEO_SCRIPT.md
ev = lambda typ: f"{fl_(typ):.0f}"
open(os.path.join(DOCS, "VIDEO_SCRIPT.md"), "w").write(f"""# Demo video script (target 3:00; the sprint plan asks for 2 to 3 minutes) - one-floor building

You record; this file says what to show, what to say and when. Times below are **simulated seconds** (seed 7, `docs/results_final.json`); the whole mission lasts about {eend:.0f} s, i.e. {eend / 60:.1f} minutes at `speed:=1`. For a 3-minute video run **`speed:=3`** (about {eend / 3 / 60:.1f} minutes) and cut the start. The story starts about 25 s after the launch when Gazebo opens (8 s without Gazebo).

## Before you record
1. Close other windows, silence notifications. Open three browser tabs: `http://localhost:8081` (simulation), `http://localhost:8080` (Command Post), `http://localhost:8081/flow` (data flow + beacon table).
2. Have these images ready full screen: `docs/diagrams/architecture.png`, `docs/diagrams/fig_trace.png`, `docs/diagrams/fig_results.png`.
3. Screen recorder: press **PrtSc**, choose the video icon, select the whole screen. If it stops after 30 s run once `gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 0` and use **Ctrl+Alt+Shift+R** to start/stop. Files land in `~/Videos`.
4. Start the system: `bash scripts/run_clean.sh speed:=3 2>&1 | tee ~/launch.log`

## What the run shows (simulated time)
| sim time | event | at speed:=3 |
|---|---|---|
| 0-{gpsT:.1f} s | the Writer leaves the van and drives to the door **by GPS**, then loses GPS | 0-{gpsT / 3:.0f} s |
| {ev('GAS')} s | gas leak found (class P0): beacon; first order at {mean(g('first_order_s', B)):.0f} s, the fire truck leaves at {mean(ft1):.0f} s to seal it | {fl_('GAS') / 3:.0f} s |
| {ev('ANIMAL')} s | animal found (class P2): it waits, because a person is not known yet | {fl_('ANIMAL') / 3:.0f} s |
| {ev('DEBRIS')} s | rubble and the person sealed behind it: `debris` beacon, flagged for human rescue | {fl_('DEBRIS') / 3:.0f} s |
| {ev('FIRE')} s | first fire found in the east: the fire truck is sent again | {fl_('FIRE') / 3:.0f} s |
| {reach_t:.0f} s | first reachable person found; the ambulance leaves at {mean(dep1):.0f} s with the person first, then the animal | {reach_t / 3:.0f} s |
| {wdone:.0f} s | the Writer finishes exploring, reports `EXPLORATION_DONE`, drives back to the van | {wdone / 3:.0f} s |
| about {mean(g('fires_out_s', B)):.0f} s | both fires are out | {mean(g('fires_out_s', B)) / 3:.0f} s |
| about {mean(g('arrive_human_s', B)):.0f} s | the ambulance reaches the person | {mean(g('arrive_human_s', B)) / 3:.0f} s |
| about {eend:.0f} s | mission over; results card on the simulation page | {eend / 3:.0f} s |
Exact times shift by a few seconds between runs; follow the screen, not the clock.

## Shot list
| Video time | On screen | Say | Caption |
|---|---|---|---|
| 0:00-0:12 | title slide | "When a robot fails inside a burning building, everything it learned is lost. The Living Map leaves that knowledge in the building." | The Living Map |
| 0:12-0:35 | `architecture.png` | "A Writer robot explores; a van is the only link to the outside; the Command Post ranks events and briefs the Executors. Robots never talk to each other, or directly to the Command Post." | Air-gap: one crossing |
| 0:35-0:55 | terminal, then the simulation page | "One command starts everything. The Writer leaves the van and drives to the door by GPS. Inside, there is no GPS." | GPS to the door, then no GPS |
| 0:55-1:30 | simulation page + flow page | "The Writer drops relay beacons to keep a chain to the van, and event beacons: a gas leak, an animal, a person behind rubble, the fires. Each beacon is 32 bytes with a CRC and an authentication tag; the van rebuilds the position and converts it to GPS." | 32 B beacon, CRC16 + MAC |
| 1:30-2:00 | Command Post page | "Every event has a priority number: zero is the most urgent, and it falls while an event waits. The fire truck leaves for the gas leak right away; the ambulance waits until a reachable person is known, then takes the person first and the animal in the same order." | Priority 0 to infinity |
| 2:00-2:25 | `fig_trace.png` | "Here is the whole run: paths, relay chain, beacons and a timeline. The person behind the rubble goes to the human rescue team." | Memory in the beacons |
| 2:25-2:50 | `fig_results.png` + the failure table | "Once briefed, an Executor reaches a person in {exec_:.0f} seconds instead of {blind:.0f} blind, but end to end the Writer must explore first. We also show where we fail: lose the Writer early and the unexplored fires stay unknown; strong radio attenuation breaks the beacon chain." | Honest limits |
| 2:50-3:00 | closing slide | "Next: a physical prototype in a plywood arena, with two robots, a van and a laptop as Command Post." | Phase 2 |

## If something goes wrong
* A page is empty: reload it, run `bash scripts/check_run.sh`.
* Do **not** claim Gazebo in the video unless the Gazebo window really shows the building in your take.
* To slow the story down for the camera use `speed:=1`.
""")

# =================================================================================================== PROTOTYPE_ARENA table
pa = open(os.path.join(DOCS, "PROTOTYPE_ARENA.md")).read()
if "| seed | fire out |" in pa and "Problems the arena exposed" in pa:
    rows = "\n".join(f"| {r['seed']} | {r['fires_out']}/{r['fires']} | {r['rescued']}/{r['people']} | {r['delivered']}/{r['beacons']} | {r['steam_rejected']} | {r['end_s']:.0f} |" for r in AB)
    a = pa.index("| seed | fire out |"); b = pa.index("Problems the arena exposed")
    pa = pa[:a] + "| seed | fire out | people rescued | beacons delivered | steam rejected | done at (sim s) |\n|---|---|---|---|---|---|\n" + rows + f"\n\nThe sealed person goes to the human rescue team: flagged up-front by the Writer in {len(afl)} of {NA} runs, discovered by a failed ambulance sortie in the others (see FAILURE_CASES.md). The arena is the Phase 2 prototype map; the project's main map is the one-floor building. Source: `docs/results_final.json`.\n\n" + pa[b:]
    open(os.path.join(DOCS, "PROTOTYPE_ARENA.md"), "w").write(pa)

for f in ("docs/Living_Map_Technical_Report.pdf", "docs/results.json", "docs/nav_results.json", "tools/collect_results.py", "tools/refresh_docs.py", "tools/build_report.py", "tools/make_figures.py"):
    p = os.path.join(H, "..", f)
    if os.path.exists(p): os.remove(p)
subprocess.run([sys.executable, os.path.join(H, "make_repo_guide.py")], check=True); print("final docs regenerated")
