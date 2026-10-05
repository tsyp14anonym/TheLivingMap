#!/usr/bin/env bash
# Kill EVERYTHING from earlier runs, free the ports, then launch. Usage: scripts/run_clean.sh [launch args, e.g. speed:=6 auto_approve:=false]
echo "[1/4] stopping old processes..."
# kill old runs, but never this script itself (its own path contains "living_map")
for pat in "living_map_core" "living_map_fire_env" "gz sim" "rviz2"; do
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$pid" != "$$" ] && [ "$pid" != "$PPID" ] && kill "$pid" 2>/dev/null
  done
done
ros2 daemon stop 2>/dev/null
sleep 2
echo "[2/4] freeing ports 8080 8081 9101 9102 and clearing /tmp/living_map..."
rm -rf /tmp/living_map
if command -v fuser >/dev/null; then for p in 8080 8081; do fuser -k ${p}/tcp 2>/dev/null; done; fuser -k 9101/udp 2>/dev/null; fuser -k 9102/udp 2>/dev/null; fi
sleep 1
left=$(pgrep -f 'lib/living_map_core|living_map_fire_env|gz sim' | grep -vx "$$" | grep -vx "$PPID" | wc -l)
[ "$left" != "0" ] && { echo "WARNING: $left old process(es) still alive: killing hard"; pgrep -f 'lib/living_map_core|living_map_fire_env|gz sim' | grep -vx "$$" | grep -vx "$PPID" | xargs -r kill -9; sleep 1; }
echo "[3/4] loading ROS and the workspace..."
source /opt/ros/jazzy/setup.bash
WS="${LIVING_MAP_WS:-$HOME/living_map_ws}"
[ -f "$WS/install/setup.bash" ] || { echo "ERROR: $WS/install/setup.bash missing. Build first: cd $WS && colcon build --symlink-install"; exit 1; }
source "$WS/install/setup.bash"
ros2 pkg executables living_map_core | grep -q ambulance_node || { echo "ERROR: package not built correctly (no ambulance_node). Rebuild: cd $WS && colcon build --symlink-install"; exit 1; }
echo "[4/4] launching. Open http://localhost:8081 (simulation) and http://localhost:8080 (Command Post). In ANOTHER terminal run scripts/check_run.sh after ~60 s."
exec ros2 launch living_map_core living_map_fire_env.launch.py "$@"
