# living_map_core - The Living Map (IEEE TSYP14, Fire / Hazardous Building)

Two robots, one memory. The **Writer** explores a GPS-denied fire building, detects hazards and victims and drops
radio beacons. The **Outside Network Area** gateway translates coordinates to GPS and forwards them to the
**Command Post**, which briefs the **Executor** before it enters.

## Run (one command)
    cd ~/living_map_ws && rm -rf build install log
    source /opt/ros/$ROS_DISTRO/setup.bash
    colcon build --symlink-install && source install/setup.bash
    ros2 launch living_map_core living_map_fire_env.launch.py          # speed:=1.0 for a slower demo
    # live map:  http://localhost:8080        (optional: auto_approve:=false to click APPROVE yourself)
    # faults:    fault:=beacon_destroyed | fault:=writer_lost

## Without ROS (pure Python, same logic)
    cd src/living_map_core
    python3 -m living_map_core.offline          # full run + metrics
    python3 -c "from living_map_core.offline import main_faults; main_faults()"
    python3 -m pytest test -q

## Architecture
    Writer (domain 10) -> beacon mesh (RF model) -> Gateway (domain 10, antenna side) --UDP uplink--> Command Post (domain 20)
    Command Post --briefing file at the entry gate--> Executor (domain 10) --reads beacons over RF--> victim
* **Air-gap:** inside = `ROS_DOMAIN_ID=10`, outside = `20`; no node or topic is shared. The only crossings are the gateway UDP
  uplink (wireless/satellite stand-in) and the pre-entry briefing file. `ros2 run living_map_core airgap_auditor` proves it.
* **Beacon packet (32 B):** magic, id, origin, seq, t0, type|prio|flags, n_victims, x/y (cm), target dx/dy (dm), parent, temp, gas, mirror id+seq, CRC16, 3-byte HMAC.
* **Two-clock priority:** confidence C(t)=exp(-lambda*age) per event type; queue score Q = U0 * C * (1 + beta*wait).
* **Adaptive queue:** P0 immediate (with its ancestor chain and nearby hazards); others flush at 120 s, or 8 items, or Q >= 20.
* **Detection:** 2-of-3 sensor rule (temperature, thermal camera, gas) + persistence; a steam pipe is rejected.
* **Debris:** a thermal victim with LiDAR-detected rubble on the line of sight gets `human_intervention_required`; the Executor skips it.
* **Repeaters:** dropped automatically when no beacon or gateway offers a link better than -80 dBm.
* **Gossip mirrors:** neighbours keep copies; a silent beacon becomes a BEACON_LOST event.

## Assumptions (state them in the report)
Radio: log-distance path loss, exponent 2.8, 12 dB per wall, 4 dB per rubble pile. Sensors are simulated with noise.
The world is a Python occupancy grid (fast, deterministic); a Gazebo world can be added later without changing the nodes.

## Sans ROS ni Gazebo (fenetre navigateur)
    cd ~/living_map_ws/src/living_map_core
    python3 -m living_map_core.standalone            # options: --speed 1.5  --fault beacon_destroyed  --manual-approve
Ouvre http://localhost:8080 (simulation) et http://localhost:8080/cp (Command Post).

## Gazebo and RViz (v2.1)
* `worlds/living_map_fire.sdf` is generated from the same grid as the simulation (`python3 -m living_map_core.gz_world`), so Gazebo, RViz and the web view show the same building.
* The `gz_bridge_node` moves the Writer, the Executor and a pool of beacon models in Gazebo through the `/world/living_map_fire/set_pose` service.
  It uses the gz Python bindings when available (smooth) and the `gz service` command otherwise (slower but works).
* `sim_view_node` (inside, port 8081) shows the explanatory ground-truth page and publishes `/living_map/known_map` (OccupancyGrid) and `/living_map/markers` (MarkerArray) for RViz (`rviz/living_map.rviz`).
* Manual start: `gz sim -r $(ros2 pkg prefix living_map_core)/share/living_map_core/worlds/living_map_fire.sdf`
* Diagnosis: `scripts/doctor.sh`; installation: `scripts/install_ros_gz.sh`.
