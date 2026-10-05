# Failure cases: analysis and measured behaviour (one-floor building, 48 x 24 m)

All numbers come from `python3 tools/collect_final.py` (simulation of `maps/complex_1floor.json`; 10 seeds for the baseline, 3 per fault, 2 per radio level; raw data in `docs/results_final.json`). Times are simulated seconds. The 7.2 x 4.8 m prototype arena of Phase 2 is measured too and noted where it differs.

| # | Failure case | Mitigation in the system | Measured outcome (building) |
|---|---|---|---|
| 1 | Beacon destroyed (at 120 s) | mirror frames; a neighbour re-broadcasts the lost beacon as `BEACON_LOST` | loss reported 15.5 s later; mission unchanged: 2 of 2 fires out in every run, 2.0 of 3 rescued |
| 2 | Writer lost (at 150 s) | the van forwards each record at once; the Command Post keeps the memory; Executors act on it | **0 of 2 fires out in every run, 1.0 of 3 rescued** (baseline 2.0); detection recall 0.50: the east part was never explored |
| 3 | Priority inversion, steam, batching | 2-of-3 rule rejects steam; class P0 bypasses the queue; critical priority skips the wait | **0 false records** in 10 runs; batched mode: P0 delay 0.25 s, P1 19.0 s, P3 53 s (max 108 s) |
| 4 | Impassable debris | `debris` beacon with `human_intervention_required`; Executors skip it | sealed victim flagged up-front in 10/10 runs; no robot stuck |
| 5 | RF attenuation | repeater beacons, ARQ x3, store-and-forward | fine up to 24 dB per wall; **fails at 30 dB in one of two seeds** (see below) |

## 1. Beacon destroyed
Every beacon carries the id and sequence of a neighbour (a *mirror frame*). When a node stays silent, a neighbour re-broadcasts the stored record as `BEACON_LOST` (id 1000 + beacon id, class P3). The busiest relay is destroyed at 120 s; the Command Post learns of it 15.5 s later (15 s of silence are required) and keeps the last known data; the mission outcome equals the baseline. Reproduce: `bash scripts/run_clean.sh speed:=1 fault:=beacon_destroyed`.

## 2. Writer lost: the single point of failure
The Writer is killed at 150 s (map parameter `fault_times`). Everything it reported before is already at the Command Post, so the Executors finish what is known: the fire truck and the ambulance still act on the gas leak, the animal and what the Writer had found. But **both fires lie in the east part of the building that the Writer had not yet reached**, so none is extinguished and only 1.0 of 3 victims is rescued (recall 0.50). The Command Post never receives `EXPLORATION_DONE`, so low-urgency jobs are released by priority aging. This is the honest cost of losing the only scout; a second Writer, or an Executor that also maps, would remove it (Phase 2). Reproduce: `... fault:=writer_lost`.

## 3. Priority inversion, steam and batching
* **Steam.** The detector needs two of three sensor votes for three consecutive readings; a steam source heats the air but has no hot thermal signature, so it is counted as `steam_rejected` and never beaconed. In the building runs the Writer never passes close to the steam source (0.0 rejections per run), so the rejection is shown by the unit test `test_detector_rejects_steam` and by the prototype arena (rejected 1.5 time(s) per run, 0 false records in 10 runs).
* **Batching delay** (uplink messages per run: immediate 20.7, batched 6.0; 3 seeds each). Mean / max delay at the Command Post by class:

| class | immediate (s) | batched (s) |
|---|---|---|
| P0 | 0.25 / 0.50 | 0.25 / 0.50 |
| P1 | 0.06 / 0.50 | 19.00 / 55.50 |
| P2 | 0.17 / 0.50 | 22.67 / 29.50 |
| P3 | 0.10 / 0.50 | 53.37 / 108.50 |

Class P0 is never held. Mean mission end: immediate 384 s, batched 396 s. Immediate mode is the default; batching is the bandwidth-saving option required by the challenge text. Reproduce: `... uplink:=batched`.
* **Preemption** (an event with priority number below 0.3 skips the collection wait) is covered by unit tests, not exercised by these scenarios.

## 4. Impassable debris
The Writer marks rubble on the line of sight to a person as `debris` and flags the person `human_intervention_required`; the Command Post lists the victim as `HEAVY_RESCUE` and dispatches nobody. In the building this happens in **10 of 10 runs** and the victim is always handed to the human team. On the small arena it happens in only 4 of 10 runs: elsewhere the Writer sees the victim from a place with no rubble on the line of sight, the ambulance is sent, finds no route and reports `UNREACHABLE` after about 36 s, and the mission lasts 146 s instead of 72 s. Phase 2 improvement: let the Writer check on its own map whether a person can be reached before reporting them.

## 5. RF attenuation (where the design fails)
Radio model: RSSI = -40 - 28 log10(d) - (loss per wall) x walls - 4 dB per rubble pile - 18 dB per floor. The baseline uses 12 dB per wall (assumed). NIST 1997 data at 2.4 GHz (as tabulated by Wi-Fi Vitae): plywood about 1 dB, brick about 6, concrete 102 mm about 15, concrete 203 mm about 29, reinforced concrete 203 mm about 31.

**Building (2 seeds per level, mission capped at 700 s).** The Writer covers long distances behind many walls and every repeater spends part of its budget of 44 beacons.

| loss per wall | fires out (all seeds) | victims rescued (mean) | repeaters | beacons delivered (of dropped) | mission end |
|---|---|---|---|---|---|
| 12 dB | 4 of 4 | 2.0 of 2 | 13.5 | 20.5 of 20.5 | 387 s |
| 24 dB | 4 of 4 | 2.0 of 2 | 21.5 | 28.5 of 28.5 | 388 s |
| 30 dB | 2 of 4 | 1.0 of 2 | 21.0 | 14.5 of 28.5 | 355 s |

At 30 dB per wall one seed still completes, but in the other too little reaches the Command Post: the fires stay unknown and no victim is rescued. **The design therefore fails under strong attenuation, as it would in heavy concrete (29 to 31 dB).**

**Prototype arena (3 seeds per level, capped at 300 s).** The arena is small, so the Writer can always drop enough repeaters: every mission completes up to 30 dB per wall.

| loss per wall | fires out (all seeds) | victims rescued (mean) | repeaters | beacons delivered (of dropped) | mission end |
|---|---|---|---|---|---|
| 1 dB | 3 of 3 | 3.0 of 3 | 0.0 | 6.7 of 6.7 | 94 s |
| 6 dB | 3 of 3 | 3.0 of 3 | 0.0 | 6.7 of 6.7 | 94 s |
| 12 dB | 3 of 3 | 3.0 of 3 | 0.7 | 7.3 of 7.3 | 94 s |
| 18 dB | 3 of 3 | 3.0 of 3 | 1.3 | 8.0 of 8.0 | 94 s |
| 24 dB | 3 of 3 | 3.0 of 3 | 2.7 | 9.3 of 9.3 | 94 s |
| 30 dB | 3 of 3 | 3.0 of 3 | 4.0 | 10.7 of 10.7 | 94 s |

Phase 2 mitigations (not implemented): a sub-GHz LoRa link, more relay nodes, and a beacon budget that reserves capacity for events. The prototype arena includes one foil-lined wall to measure this on hardware. Reproduce: set `"wall_db": 24` in a map JSON.

## What these tests do *not* show
* Everything is simulation; the radio parameters are assumed values, not measurements.
* The steam rejection is not exercised on the building, and preemption only by unit tests.
