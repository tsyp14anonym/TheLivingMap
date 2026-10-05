# The Living Map: prototype arena (research, specification check, build guide)

Date: 2 Oct 2026. Environment chosen by the team: Fire / Hazardous Building. Final submission deadline (poster): 01/12/2026.

## 1. Recommendation in one paragraph
Build **one small, modular arena of 7.2 x 4.8 m (6 x 4 modules of 1.2 m)** from 12 mm plywood strips, in the style of the **RoboCup Rescue "yellow arena"** (hallways and rooms, 1.2 m hallways, doorways wider than the robot). It is defined once in `tools/make_proto_arena.py`, which writes the **simulator map** (`maps/proto_arena.json`), the **Gazebo world** (`worlds/proto_arena.sdf`), and, when it is run, a **printable plan** and a **material list** in `docs/`, so simulation and plywood cannot drift apart. Walls come to 37.5 m = **8 sheets** of 1.22 x 2.44 m plywood (9 with spare), each sheet cut lengthwise into two 0.61 m strips with no waste.

## 2. What the specification book requires, and how the prototype covers it
Source: the TSYP14 poster ("The Living Map"). Phase 2 (finals) asks for a **complete physical prototype (15 pts)**, pitching 5 min + 2 min Q&A, user manual (4), project architecture (5), GitHub (5).

| Specification requirement | In the simulation | In the prototype (arena) |
|---|---|---|
| At least 2 physical robots: Writer + Executor | Writer + 3 Executors (ambulance, fire truck, drone) | **Writer + 1 Executor**. GAP: the Command Post picks robots by role; a single "universal Executor" role (rescue + suppress + seal as signalled actions) still has to be added |
| Writer explores a GPS-denied space autonomously | Writer explores, drops beacons | Arena interior is GPS-denied; LiDAR SLAM only. Entrance = frame-translation anchor (0,0) |
| Detect at least 2 event types | FIRE, GAS, HUMAN, ANIMAL, DEBRIS, steam rejected | Heat lamp (fire), gas source, 3 heated pads (humans), cooler pad (animal), foam rubble (debris), kettle (steam false alarm). Simulation on the arena detects 5 types |
| Small radio beacons: compact message, RF broadcast, message aging | 32-byte beacon, aging pi(t) | Same 32 B frame on an ESP32 + LoRa node (see 4.3); aging runs in the Command Post |
| Frame translation: private coordinates to GPS | ENU -> WGS84 from the entrance anchor | Same anchor at the entrance door; check with a phone GPS at the real entrance |
| Outside Network Area connects inside to outside; ALL traffic passes through it; no direct robot-to-Command-Post link | Van = gateway; star topology audited | A table in the yard with the gateway + radio (the "van"). Command Post laptop in **another room**, linked ONLY to the van. Robots have no Wi-Fi to the laptop |
| Executor receives a mission and navigates using inherited beacons | Briefing, A* to the target, trail as hint | Executor gets the briefing at the van, reads beacons by radio, drives to the victim |
| Live map at the Command Post | Page on port 8080 | Same page on the laptop |
| Priority 0 immediate relay vs 120 s batching | `uplink:=immediate / batched` | Demo both: P0 = heated victim next to the lamp |

### The five failure cases, as physical tests
| Failure case | Physical test in the arena |
|---|---|
| 1 Beacon destruction | Switch one beacon off after the drop: the neighbour's mirror frame and the missing heartbeat must flag it |
| 2 Writer trapped | Stop the Writer (kill switch) before it returns: the Command Post must still brief the Executor from the memory it holds |
| 3 Priority inversion / steam false positive | Kettle (steam plume) at the corridor start must NOT become a FIRE; a victim beside the lamp must go out at once (P0) |
| 4 Impassable debris | Foam rubble across the south-east door with victim H3 behind it: DEBRIS beacon + `human_intervention_required`, the Executor skips it |
| 5 RF attenuation | **Plywood does not attenuate** (about 1 dB, section 4.2), so add one wall lined with aluminium foil or metal mesh and **measure the signal strength before and after**; chain a repeater beacon if it drops |

## 3. Standards found (why this layout)
* RoboCup Rescue / NIST: the **yellow arena is a maze of hallways and rooms for autonomous robots**; the 2007 overview gives a 10 x 15 m maze with 1.2 m hallways; the **orange arena** uses crossing ramps (15 degrees), the **red arena** stepfields. The 2026 rules set doorways to **robot width + 10 cm**, ramps 15/30 degrees, stairs 35/40/45 degrees.
* RoboCupJunior Rescue Maze (small robots): 30 cm tiles, walls at least 15 cm high, ramps at most 25 degrees, levels 40 to 60 cm reached by ramps, **heated victims at least 10 C above ambient, over 16 cm2, about 7 cm above the floor**, robot height at most 30 cm.
* NIST publishes an arena fabrication guide (RoboCupRescue Robot League Arena Fabrication Guide 2025B). It is a file of more than 30 MB that was not reviewed for this document: download it from the RoboCup rescue site and compare the maze panel details with this plan.

Choices that follow: hallway 1.2 m, doors 0.9 m (for a robot up to about 0.4 m wide), walls 0.61 m (a 2D LiDAR at 0.2 to 0.4 m height sees them, and the sheet is used with no waste), heated pads for victims.
Optional second level (not in the map): a 40 to 60 cm platform reached by a ramp of 15 degrees or less would imitate "floor 1" for ground robots; the stairs/atrium logic for the drone is not needed with 2 ground robots.

## 4. Findings that affect the Phase 2 plan
### 4.1 Gazebo worlds
No maintained Gazebo Harmonic version of the RoboCup arenas was found; the RoboCup Rescue Simulation league still points to Gazebo 11 tutorials (ROS 2 Foxy). `map2sdf` (github.com/atinfinity/map2sdf) converts a nav2 occupancy-grid image into a gz sim world and is tested on Jazzy + Harmonic, a good alternative workflow. This project already has its own map -> SDF generator, which is what `worlds/proto_arena.sdf` uses.

### 4.2 Radio through walls: the arena will not test the attenuation failure by itself
NIST 1997 measurements (as tabulated by Wi-Fi Vitae), loss at 2.4 GHz: plywood 6 mm about 1 dB; drywall about 1 dB; brick about 6 dB; concrete 102 mm about 15 dB; concrete 203 mm about 29 dB; reinforced concrete 203 mm about 31 dB. A plywood arena is nearly transparent, so the "RF attenuation" failure case needs an added shield (foil / metal mesh wall) and an RSSI measurement.

### 4.3 Bill of materials
* **DWM1000 is end of life.** Qorvo announced EOL for the EVK1000 (DW1000) on 24 Mar 2021 (last-time buy 30 Sep 2021) and for the DWM1004C on 20 Apr 2022; replacements for new designs are DWM3000EVB / DWM3001CDK (DW3000). One distributor lists a DWM3000 sample part as end of life too, so check stock before ordering the 6 nodes.
* UWB is for **ranging**, not for carrying the 32-byte message far. For the beacon message, LoRa is simpler. Sources list Tunisia under the EU433 plan (433.05 to 434.79 MHz) and EU863-870; these are third-party summaries, so **confirm with the Tunisian regulator** before transmitting.
* LoRa airtime for the 32-byte frame (BW 125 kHz, CR 4/5, CRC on), computed for this project: **SF7 72 ms** (500 packets per hour at a 1 % duty cycle), SF8 134 ms (269), SF9 247 ms (146), SF10 453 ms (80), SF12 1.81 s (20). In a 7 m arena SF7 is enough.
* **MQ-4**: 200 to 10 000 ppm methane, preheat of at least 24 h (Hanwei) or 48 h (Winsen) for stable readings, and "small sensitivity to alcohol, smoke". So an alcohol-vapour stand-in will not make an MQ-4 react.

### 4.4 Safe stand-ins (engineering suggestions, not from a source: review safety before building)
* Fire: a heat lamp or ceramic heater behind a metal-mesh guard, surface near 140 C, never near flammable material, supervised. The thermal camera (Lepton) is the trigger.
* Gas: keep it **away from the lamp** (it is in the opposite room) and **non-flammable**; match the stand-in to the sensor you will really use.
* Victims: heated pads at least 10 C above ambient, 7 cm above the floor (RoboCupJunior rule). Steam: a kettle.

## 5. What was verified in the simulation (and what was not)
Arena map loaded in the project's own format; Gazebo SDF valid XML (147 models); the mission was run on 5 seeds:

| seed | fire out | people rescued | beacons delivered | steam rejected | done at (sim s) |
|---|---|---|---|---|---|
| 1 | 1/1 | 3/4 | 6/6 | 2 | 141 |
| 2 | 1/1 | 3/4 | 9/9 | 2 | 62 |
| 3 | 1/1 | 3/4 | 7/7 | 2 | 78 |
| 4 | 1/1 | 3/4 | 6/6 | 1 | 145 |
| 5 | 1/1 | 3/4 | 6/6 | 1 | 148 |
| 6 | 1/1 | 3/4 | 6/6 | 1 | 148 |
| 7 | 1/1 | 3/4 | 7/7 | 1 | 74 |
| 8 | 1/1 | 3/4 | 7/7 | 2 | 76 |
| 9 | 1/1 | 3/4 | 6/6 | 1 | 144 |
| 10 | 1/1 | 3/4 | 6/6 | 2 | 148 |

The sealed person goes to the human rescue team: flagged up-front by the Writer in 4 of 10 runs, discovered by a failed ambulance sortie in the others (see FAILURE_CASES.md). The arena is the Phase 2 prototype map; the project's main map is the one-floor building. Source: `docs/results_final.json`.

Problems the arena exposed, now fixed in v4_11: (1) **line of sight and LiDAR rays stepped 0.25 m, so on a 0.1 m grid they passed through walls**: they now step at most one cell (the big building, 0.5 m cells, is unchanged); (2) fire/gas **detection thresholds** were fixed for a 400 C building fire: they are now per map (`thresholds`), and the arena uses the thermal camera as trigger; (3) several **distances tied to the building size** (terminal-guidance radius, hold distance, stand-off, correlation distance) are now per map.
Not verified: Gazebo or RViz on the arena (the SDF was only checked as XML), real DDS, physical robot dynamics (the simulated robots move 1.2 to 3 m/s; real prototypes will be slower), the single-Executor mode, and the RF shield test.

## 6. How to run it
```bash
# no ROS, fastest
cd ~/living_map_ws/src/living_map_core && python3 -m living_map_core.standalone --speed 6 --map maps/proto_arena.json
# Gazebo world alone
gz sim -r -v 2 ~/living_map_ws/src/living_map_core/worlds/proto_arena.sdf
# full ROS system on the arena
bash ~/living_map_ws/src/living_map_core/scripts/run_clean.sh speed:=6 map:=$HOME/living_map_ws/src/living_map_core/maps/proto_arena.json
```
Change the layout: edit `tools/make_proto_arena.py` (walls, doors, objects) and run `python3 tools/make_proto_arena.py .`, then `python3 -c "from living_map_core.gz_world import build_sdf; open('worlds/proto_arena.sdf','w').write(build_sdf('maps/proto_arena.json'))"`.

## 7. Open questions for the organisers
1. Is the physical prototype shown live in the pitch or by video? Is there an arena size or transport limit?
2. Which radio bands and powers are allowed at the venue?
3. May the Command Post and the Outside Network Area be laptops, and is a 4G link acceptable as the "wireless / satellite" link?
4. Does the second robot have to be a single Executor, or is a team of Executors rewarded?

## 8. Sources
RoboCupRescue Rules 2026D and 2025F, Arena Fabrication Guide 2025B (rrl.robocup.org); RoboCup Rescue Robot League overview 2007 (ndia.dtic.mil); RoboCupJunior Rescue Maze rules 2016 to 2026 (junior.robocup.org, robocupjunior.org.au); Wi-Fi Vitae "Wall Attenuation Measurements" (NIST 1997 data) and interline.pl / wifiweave charts; Qorvo EVK1000 and DWM1004C product pages; Qorvo forum; Lansitec / The Things Network / Meshtastic frequency pages; Hanwei and Winsen MQ-4 datasheets; atinfinity/map2sdf; RoboCup Rescue Simulation VRL rules 2022.
Limits: web search is not exhaustive; third-party summaries are marked; nothing here replaces the official rules the organisers send for Phase 2.
