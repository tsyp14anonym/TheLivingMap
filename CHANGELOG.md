# Changelog

* **v4_16 (this version)**: the one-floor **building** (`maps/complex_1floor.json`) is the default map and Gazebo world; the 7.2 x 4.8 m arena is the Phase 2 prototype map (`map:=.../proto_arena.json`). `tools/make_1floor.py` derives the building from the 2-floor map (geometry unchanged) and adds `fault_times`. All documents are regenerated from `docs/results_final.json`.
* v4_15: fault times are a map parameter; `beacon_destroyed` falls back to the oldest beacon when none has children; **fixed a measurement error**: a sortie that ended in the same simulation step as the next one began was not recorded, which hid the first mission of a robot (the Executor's first-person time was over-reported). Numbers measured before this version are obsolete.
* v4_14: automatic hold of low-urgency jobs while the Writer explores (threshold computed from the priority table), new station message `EXPLORATION_DONE`, the Writer drives back to the van after exploring.
* v4_13: the Writer starts at the van and drives to the door by GPS; smaller, configurable van (`van_size`); the web page follows the loaded map.
* v4_11: prototype arena generated from one definition (map, Gazebo world, plan, material list); detection thresholds and several distances are per-map parameters; rays no longer pass through thin walls on fine grids.
* v4_10: Executors plan directly to the target (A* with a free-space assumption) and order their jobs by urgency-weighted completion time.
* v4_9: Command Post with a configurable decision cycle, incident correlation, preemption, order updates through the van and robot-loss handling; robots no longer read state written by other robots.
* v4_8: fixed a logging call that crashed the van on real ROS 2 (one severity per call site).
