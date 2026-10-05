# Demo video script (target 3:00; the sprint plan asks for 2 to 3 minutes) - one-floor building

You record; this file says what to show, what to say and when. Times below are **simulated seconds** (seed 7, `docs/results_final.json`); the whole mission lasts about 384 s, i.e. 6.4 minutes at `speed:=1`. For a 3-minute video run **`speed:=3`** (about 2.1 minutes) and cut the start. The story starts about 25 s after the launch when Gazebo opens (8 s without Gazebo).

## Before you record
1. Close other windows, silence notifications. Open three browser tabs: `http://localhost:8081` (simulation), `http://localhost:8080` (Command Post), `http://localhost:8081/flow` (data flow + beacon table).
2. Have these images ready full screen: `docs/diagrams/architecture.png`, `docs/diagrams/fig_trace.png`, `docs/diagrams/fig_results.png`.
3. Screen recorder: press **PrtSc**, choose the video icon, select the whole screen. If it stops after 30 s run once `gsettings set org.gnome.settings-daemon.plugins.media-keys max-screencast-length 0` and use **Ctrl+Alt+Shift+R** to start/stop. Files land in `~/Videos`.
4. Start the system: `bash scripts/run_clean.sh speed:=3 2>&1 | tee ~/launch.log`

## What the run shows (simulated time)
| sim time | event | at speed:=3 |
|---|---|---|
| 0-3.5 s | the Writer leaves the van and drives to the door **by GPS**, then loses GPS | 0-1 s |
| 48 s | gas leak found (class P0): beacon; first order at 50 s, the fire truck leaves at 50 s to seal it | 16 s |
| 91 s | animal found (class P2): it waits, because a person is not known yet | 30 s |
| 118 s | rubble and the person sealed behind it: `debris` beacon, flagged for human rescue | 39 s |
| 196 s | first fire found in the east: the fire truck is sent again | 65 s |
| 224 s | first reachable person found; the ambulance leaves at 227 s with the person first, then the animal | 75 s |
| 284 s | the Writer finishes exploring, reports `EXPLORATION_DONE`, drives back to the van | 95 s |
| about 287 s | both fires are out | 96 s |
| about 303 s | the ambulance reaches the person | 101 s |
| about 384 s | mission over; results card on the simulation page | 128 s |
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
| 2:25-2:50 | `fig_results.png` + the failure table | "Once briefed, an Executor reaches a person in 76 seconds instead of 172 blind, but end to end the Writer must explore first. We also show where we fail: lose the Writer early and the unexplored fires stay unknown; strong radio attenuation breaks the beacon chain." | Honest limits |
| 2:50-3:00 | closing slide | "Next: a physical prototype in a plywood arena, with two robots, a van and a laptop as Command Post." | Phase 2 |

## If something goes wrong
* A page is empty: reload it, run `bash scripts/check_run.sh`.
* Do **not** claim Gazebo in the video unless the Gazebo window really shows the building in your take.
* To slow the story down for the camera use `speed:=1`.
