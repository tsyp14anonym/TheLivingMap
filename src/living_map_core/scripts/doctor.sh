#!/usr/bin/env bash
# Living Map doctor: finds out WHY ROS 2 / Gazebo / RViz do not start on this machine, and tells you the fix.
# usage:  bash doctor.sh
FAILS=0
ok()   { echo "  [ OK ]  $*"; }
bad()  { echo "  [FAIL]  $*"; FAILS=$((FAILS+1)); }
warn() { echo "  [WARN]  $*"; }
hint() { echo "          -> $*"; }
title(){ echo; echo "== $* =="; }

title "1. Operating system"
. /etc/os-release 2>/dev/null
echo "  $PRETTY_NAME  (python: $(python3 --version 2>&1))"
case "$VERSION_ID" in
  24.04) WANT=jazzy ;; 22.04) WANT=humble ;; *) WANT=""; warn "unusual Ubuntu version: ROS 2 Jazzy needs 24.04, Humble needs 22.04" ;;
esac
[ -n "$WANT" ] && ok "this Ubuntu matches ROS 2 '$WANT'"
if grep -qi microsoft /proc/version 2>/dev/null; then warn "WSL detected: GUI windows need WSLg (Windows 11 / recent Windows 10)."; fi
V=$(systemd-detect-virt 2>/dev/null); [ -n "$V" ] && [ "$V" != "none" ] && warn "running inside a virtual machine ($V): Gazebo/RViz need 3D acceleration; if windows are black or crash use: export LIBGL_ALWAYS_SOFTWARE=1"

title "2. ROS 2 installed?"
if ls /opt/ros/*/setup.bash >/dev/null 2>&1; then
  ok "found: $(ls /opt/ros | tr '\n' ' ')"
  [ -n "$WANT" ] && [ ! -d /opt/ros/$WANT ] && bad "ROS '$WANT' (the right one for this Ubuntu) is not installed" && hint "run: bash $(dirname "$0")/install_ros_gz.sh"
else
  bad "no ROS 2 in /opt/ros"; hint "run: bash $(dirname "$0")/install_ros_gz.sh"
fi

title "3. ROS 2 loaded in THIS terminal?"
if command -v ros2 >/dev/null 2>&1; then ok "ros2 command found (ROS_DISTRO=$ROS_DISTRO)"
else
  bad "the 'ros2' command is not in this terminal. ROS is installed but not 'sourced'."
  D=$(ls /opt/ros 2>/dev/null | head -1); hint "run:  source /opt/ros/${WANT:-$D}/setup.bash"
  hint "and make it permanent:  echo 'source /opt/ros/${WANT:-$D}/setup.bash' >> ~/.bashrc"
fi
python3 -c "import rclpy" 2>/dev/null && ok "python can import rclpy" || bad "python cannot import rclpy (ROS not sourced, or another python/conda is active)"
[ -n "$CONDA_PREFIX" ] && warn "a conda environment is active ($CONDA_PREFIX): it breaks ROS python. Run: conda deactivate"
echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0 (default)}"

title "4. Gazebo installed?"
if command -v gz >/dev/null 2>&1; then
  ok "gz command found: $(gz sim --version 2>&1 | head -1)"
else
  bad "the 'gz' command does not exist. This is THE classic problem on Ubuntu 24.04 (Jazzy): 'ros-jazzy-ros-gz' alone does not give you the gz program."
  hint "run: bash $(dirname "$0")/install_ros_gz.sh      (adds the official Gazebo repository and installs gz-harmonic)"
fi
if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 && ok "ros_gz_sim installed" || warn "ros_gz_sim missing (only needed for advanced bridges): sudo apt install ros-${ROS_DISTRO:-jazzy}-ros-gz"
  ros2 pkg prefix rviz2 >/dev/null 2>&1 && ok "rviz2 installed" || bad "rviz2 missing: sudo apt install ros-${ROS_DISTRO:-jazzy}-rviz2"
fi
python3 -c "import gz.transport13" 2>/dev/null && ok "gz python bindings present (smooth Gazebo motion)" || warn "gz python bindings missing: Gazebo motion will use the slower CLI mode. Optional fix: sudo apt install python3-gz-transport13 python3-gz-msgs10"

title "5. Screen / graphics"
if [ -n "$DISPLAY" ] || [ -n "$WAYLAND_DISPLAY" ]; then ok "display found (DISPLAY=$DISPLAY WAYLAND_DISPLAY=$WAYLAND_DISPLAY, session: $XDG_SESSION_TYPE)"
else bad "no display (are you in SSH or a text console?). GUI apps cannot open."; fi
[ "$XDG_SESSION_TYPE" = "wayland" ] && warn "Wayland session: if Gazebo or RViz crash, run 'export QT_QPA_PLATFORM=xcb' first (the launch file already does this for Gazebo)."
if command -v glxinfo >/dev/null 2>&1; then echo "  OpenGL renderer: $(glxinfo -B 2>/dev/null | grep -i 'renderer string' | cut -d: -f2)"; else warn "glxinfo not installed (optional): sudo apt install mesa-utils"; fi

title "6. Leftover processes (the #1 cause of weird logs: two copies running together)"
P=$(pgrep -af "writer_node|mesh_node|gateway_node|executor_node|command_post_node|sim_view_node|gz_bridge|writer_autonomy|outside_network_gateway|executor_mission_briefing|gz sim" | grep -v pgrep | grep -v doctor)
if [ -n "$P" ]; then bad "old processes are still running:"; echo "$P" | sed 's/^/          /'; hint "kill them:  pkill -f living_map; pkill -f 'gz sim'; ros2 daemon stop"; else ok "no leftover processes"; fi
for port in 8080 8081 9101 9102; do ss -ltnu 2>/dev/null | grep -q ":$port " && warn "port $port already in use (an old run?)"; done

title "7. Workspace"
W=${1:-$HOME/living_map_ws}
[ -f "$W/install/setup.bash" ] && ok "workspace built: $W" || { bad "workspace not built: $W/install/setup.bash missing"; hint "cd $W && source /opt/ros/${WANT:-jazzy}/setup.bash && colcon build --symlink-install"; }
[ -f "$W/install/setup.bash" ] && source "$W/install/setup.bash" 2>/dev/null && command -v ros2 >/dev/null && (ros2 pkg prefix living_map_core >/dev/null 2>&1 && ok "package living_map_core is visible to ROS" || bad "living_map_core not visible: run 'source $W/install/setup.bash'")

echo; if [ $FAILS -eq 0 ]; then echo "ALL GOOD. Start with:  ros2 launch living_map_core living_map_fire_env.launch.py"
else echo "$FAILS problem(s) found. Fix them in the order shown above, then run this script again."; echo "No ROS/Gazebo needed for a demo: python3 -m living_map_core.standalone"; fi
