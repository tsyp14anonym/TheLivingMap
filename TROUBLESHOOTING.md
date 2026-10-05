# Nothing shows up? Read this first

## Why ROS 2 / Gazebo / RViz "did not work"
1. **The first prototype never opened any window.** Its launch file started 6 Python nodes and RViz was OFF by default (`use_rviz:=false`).
   There was no Gazebo in the launch at all. So the terminal was the only output. That is not a crash, it was simply never wired.
2. **A new terminal has no ROS.** Every terminal must run `source /opt/ros/jazzy/setup.bash` (put it in `~/.bashrc` once).
   Otherwise `ros2`, `rviz2` and Python `rclpy` are "not found".
3. **On Ubuntu 24.04 the `gz` command is not installed by `ros-jazzy-ros-gz` alone.** You also need the Gazebo Harmonic package from the
   official Gazebo repository (`gz-harmonic`). The script `scripts/install_ros_gz.sh` does exactly that.
4. **Old copies still running.** If you launch twice, both copies talk on the same ROS domain and logs look haunted
   (duplicated messages, robots jumping). Always `pkill -f living_map; pkill -f "gz sim"; ros2 daemon stop` first.
5. **No display / VM / Wayland.** GUI windows need a desktop session. In a virtual machine use `export LIBGL_ALWAYS_SOFTWARE=1`;
   on Wayland use `export QT_QPA_PLATFORM=xcb`.

## One command that tells you which one it is
    bash ~/living_map_ws/src/living_map_core/scripts/doctor.sh

## Fix everything (Ubuntu 24.04)
    bash ~/living_map_ws/src/living_map_core/scripts/install_ros_gz.sh      # installs ROS 2 Jazzy (if missing), Gazebo Harmonic, RViz
    # open a NEW terminal, then:
    cd ~/living_map_ws && rm -rf build install log
    source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && source install/setup.bash
    ros2 launch living_map_core living_map_fire_env.launch.py

## Windows that open
| Window | What it shows |
|---|---|
| Gazebo | the 3D burning building, the two robots, hazards, victims and the beacons appearing on the floor |
| RViz | top-down view: the Writer's SLAM map growing, robots, beacons, radio links, trails |
| http://localhost:8081 | explanatory simulation page (stages, sensor reasoning, event log with explanations) |
| http://localhost:8080 | Command Post live map (only what came through the gateway) |

## No ROS at all? Still works
    cd ~/living_map_ws/src/living_map_core && python3 -m living_map_core.standalone

## v4 notes
* Robots "unreachable"/stuck: the map file must have walls closing every room; unknown space is crossable only near the beacon trail.
* `map:=` changes the building and the Gazebo world is regenerated into /tmp/living_map/world.sdf.
* Old processes: `pkill -f living_map; pkill -f "gz sim"; ros2 daemon stop` before relaunching.
* Gazebo world was generated but never opened in real Gazebo by me: if a model misbehaves, run with `gazebo:=false` (the web pages and RViz still work) and tell me what you see.

## "A CALLBACK CRASHED ... Logger severity cannot be changed between calls"
Fixed in v4_8. If you still see it, you are running an older build: rebuild (`colcon build --symlink-install`) and check `grep -n "lg.warn" ~/living_map_ws/src/living_map_core/living_map_core/nodes.py` shows a hit.

## "VAN CANNOT LISTEN on ('127.0.0.1', 9102)"
An old run still holds the downlink port. Run `scripts/run_clean.sh` (it frees 8080, 8081, 9101 and 9102). Without the downlink the robots still work, but the Command Post cannot update an order while a robot is out (e.g. "hazard cleared" would only be seen by the robot's own sensors).

## An Executor walks into walls / wanders / never arrives (new navigation)
`nav:=direct` assumes unknown space is free until the LiDAR says otherwise, so a robot may bump into a wall it could not see and replan. If you suspect the new method, run the old one for comparison: `... run_clean.sh speed:=6 nav:=trail` and tell me which robot and which beacon (`grep -E "target|unreachable|REACHED" ~/launch.log`).
