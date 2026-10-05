# Phase 2 implementation plan (6 Oct to 1 Dec 2026)

Final submission deadline (poster): **01/12/2026**. Phase 2 scoring (50 points): physical prototype 15, pitching + Q&A 8, quality of the solution 8, technologies used 5, project architecture 5, user manual 4, GitHub 5.

## 1. What will be physical
| Part | Physical prototype | Simulation (today) |
|---|---|---|
| Writer | 1 ground robot: LiDAR, thermal camera, gas sensor, beacon dropper | 1 tracked Writer |
| Executor | **1 ground robot** (the poster requires at least 2 physical robots: Writer + Executor) | ambulance, fire truck |
| Beacons | 6 to 8 ESP32 + LoRa nodes running the 32-byte frame | simulated mesh |
| Outside Network Area | a table in the "yard" with the gateway radio + computer (the van) | van gateway |
| Command Post | a laptop **in another room**, linked only to the van | Command Post node + web pages |
| Arena | 7.2 x 4.8 m plywood arena, see `PROTOTYPE_ARENA.md` | `maps/proto_arena.json`, `worlds/proto_arena.sdf` |

## 2. Software still to do (gaps found while verifying against the specification)
1. **Universal Executor role.** The Command Post still assumes the roles ambulance / fire truck. One physical Executor needs a role with all capabilities (rescue, suppress, seal) executed as signalled actions.
2. **Robot-to-van radio protocol.** Today an ideal channel ("protocol TBD"). Define framing, acknowledgements and retries.
3. **Firmware for the beacons** (ESP32): the 32-byte pack/unpack, CRC16 + MAC, mirror frames, store-and-forward. Per-deployment key instead of the demo constant.
4. **Outdoor GPS phase on hardware.** The simulator starts the Writer at the van, drives it to the door by GNSS (sigma 0.8 m, assumed) and loses GPS at the threshold. The real robot needs a GNSS receiver and a door-detection step in the LiDAR for the last metres.
5. **Sensor drivers** for the real LiDAR, thermal camera and gas sensor; thresholds in the map JSON (`thresholds`) calibrated on the hardware.
6. **RF characterisation** with a foil-lined wall; a beacon budget that reserves capacity for events (the simulation fails at very high attenuation, see `FAILURE_CASES.md`).
7. **Gazebo check.** The generated world has been validated only as XML.

## 3. Timeline
| Week | Dates | Work |
|---|---|---|
| 1 | 6-12 Oct | decisions and orders (BOM below); confirm radio band |
| 2 | 13-19 Oct | Writer chassis, LiDAR, SLAM, motor control; start the MQ-4 preheat (24 h or more) |
| 3 | 20-26 Oct | thermal + gas drivers; first detections on the bench; beacon firmware v1 |
| 4 | 27 Oct - 2 Nov | LoRa link beacon to van; gateway software; frame translation check against a phone GPS at the entrance |
| 5 | 3-9 Nov | Executor robot, briefing, navigation; arena built and measured |
| 6 | 10-16 Nov | integration of the whole chain; the five failure cases as physical tests; RSSI measurements |
| 7 | 17-23 Nov | user manual, GitHub clean-up, demo video of the prototype; pitch draft |
| 8 | 24 Nov - 1 Dec | rehearsals (5 min pitch + 2 min Q&A), spare parts, submission **before 1 Dec** |

## 4. Bill of materials
Prices are left blank on purpose: quote locally.

| Item | Qty | Role | Notes |
|---|---|---|---|
| NVIDIA Jetson Orin Nano 8 GB | 2 | onboard SLAM, Nav2, vision | check stock and delivery time |
| RPLIDAR A3 (or equivalent 2D LiDAR) | 2 | mapping, obstacle costmap | walls are 0.61 m high |
| ESP32-S3 + LoRa module (433 or 868 MHz) | 6-8 | beacon nodes and the gateway radio | confirm the band with the regulator; 32 B at SF7 is 72 ms on air |
| UWB: Qorvo DWM3001CDK (optional) | 2-4 | ranging only | **DWM1000 is end of life** (Qorvo PCN 21-0072); UWB does not replace the message link |
| Thermal camera: FLIR Lepton 3.5 with breakout | 1 | victims, fire, steam rejection | cheaper alternative: lower-resolution thermal sensor |
| MQ-4 gas sensor (methane) | 1-2 | gas events | preheat at least 24 h; low sensitivity to alcohol |
| IMU | 2 | odometry, heading | any 9-axis IMU |
| GNSS receiver (u-blox class) with antenna | 1 | Writer: GPS outdoors to the entrance door, anchor fix | needs a clear sky view; indoors use a surveyed anchor point instead |
| Chassis, motors, drivers, encoders (robot width at most 0.4 m) | 2 | Writer, Executor | doors are 0.9 m |
| Batteries, DC-DC converters, emergency stop switches | 2 sets | power and safety | |
| Plywood 12 mm, 1.22 x 2.44 m | 9 | arena (8 + spare) | see `docs/proto_arena_materials.md` |
| Brackets, clamps, aluminium foil or metal mesh, foam blocks, cardboard boxes | - | arena, rubble, RF shield | |
| Heat lamp with a metal guard; heated pads; kettle or humidifier | - | fire, victims, steam stand-ins | safety review first |
| Gas stand-in compatible with the chosen sensor | - | gas event | keep away from the heat lamp; non-flammable if possible |
| Laptops | 2 | Command Post, van | existing |

## 5. Physical test protocol
| # | Test | Pass criterion | Scores |
|---|---|---|---|
| T1 | Writer explores the arena autonomously | covers all rooms without collision in 2 of 3 runs | prototype |
| T2 | detection of fire, gas, person, animal, debris | each event beaconed once; kettle rejected | prototype |
| T3 | beacon on air | 32 B frame decoded at the van; a forged packet rejected | architecture |
| T4 | air-gap | with the van switched off, the Command Post receives nothing; robots cannot reach the laptop | architecture |
| T5 | frame translation | target position at the entrance anchor matches a phone GPS within the sensor uncertainty | quality |
| T6 | Executor briefed at the van, reaches the victim | arrives faster than a blind search | prototype |
| T7 | five failure cases | each shown once on video | quality |
| T8 | RF shield wall | RSSI measured with and without; a repeater restores the link | quality |
| T9 | priority and batching | P0 relayed at once, others batched; counts shown | quality |
| T10 | end-to-end run, three times | same chain each time | pitch |

## 6. Risks
| Risk | Effect | Mitigation |
|---|---|---|
| DWM1000 end of life | the originally considered module cannot be bought | LoRa for messages; DWM3001CDK only if ranging is needed |
| MQ-4 needs 24 to 48 h preheat | wasted bench time | power it from week 2 |
| plywood is transparent to radio | the attenuation case would not show | foil or metal-mesh wall + RSSI measurement |
| heat lamp and gas in one room | fire hazard | separate rooms, guard, supervision, extinguisher at hand |
| real robots are slower than the simulated ones | timing differs from the report | scale speeds in the map profile; report real numbers |
| single Executor not supported yet | the Command Post would leave hazards unassigned | build the universal role in week 3 |
| 8 weeks is short | scope creep | freeze the arena layout in week 5 |
