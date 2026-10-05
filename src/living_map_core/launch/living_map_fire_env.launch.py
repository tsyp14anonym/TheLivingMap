"""One command for everything.
 ros2 launch living_map_core living_map_fire_env.launch.py [gazebo:=auto|true|false] [rviz:=auto|true|false] [speed:=2.0]
        [fault:=none|beacon_destroyed|writer_lost] [map:=/path/to/map.json] [roles:=ambulance,firetruck,drone] [auto_approve:=true]"""
import os, shutil, time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _flag(value, available):
    v = value.lower(); return available if v == "auto" else v == "true"

def _setup(context):
    share = get_package_share_directory("living_map_core"); arg = lambda n: LaunchConfiguration(n).perform(context)
    speed, fault, map_path = float(arg("speed")), arg("fault"), arg("map")
    auto = arg("auto_approve").lower() == "true"; roles = [r for r in arg("roles").split(",") if r]
    has_gz, has_display = shutil.which("gz") is not None, bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    use_gz = _flag(arg("gazebo"), has_gz and has_display); use_rviz = _flag(arg("rviz"), shutil.which("rviz2") is not None and has_display)
    epoch = time.time() + (25.0 if use_gz else 8.0)            # Gazebo needs time to open before the story starts
    env = {"LIVING_MAP_FILE": map_path} if map_path else {}
    def N(exe, name, domain, **params):
        return Node(package="living_map_core", executable=exe, name=name, output="screen",
                    additional_env=dict(env, ROS_DOMAIN_ID=str(domain)), parameters=[dict(epoch=epoch, speed=speed, **params)])
    acts = [LogInfo(msg=f"=== THE LIVING MAP | inside=domain 10, outside=domain 20 | map={map_path or 'maps/complex.json'} | fault={fault} | robots={roles} | gazebo={use_gz} rviz={use_rviz} ==="),
            LogInfo(msg="Simulation view (explanatory): http://localhost:8081     Command Post live map: http://localhost:8080")]
    if not use_gz: acts.append(LogInfo(msg="Gazebo is OFF (gz command or display not found). Run scripts/doctor.sh to see why. Everything else works."))
    acts += [N("writer_node", "writer", 10, kill_at=420.0 if fault == "writer_lost" else -1.0), N("mesh_node", "beacon_mesh", 10, fault=fault),
             N("gateway_node", "outside_network_gateway", 10, uplink=arg("uplink")), N("sim_view_node", "sim_view", 10), N("command_post_node", "command_post", 20, auto_approve=auto, collect=float(arg("collect")), cycle=float(arg("cycle")), crit=float(arg("crit")))]
    for r in roles: acts.append(N(f"{r}_node", r, 10, nav=arg("nav")))
    if use_gz:
        os.makedirs("/tmp/living_map", exist_ok=True)
        world = os.path.join(share, "worlds", "complex_1floor.sdf")      # default map = the one-floor building
        if map_path:                                            # a different map needs a different Gazebo world: generate it now
            from living_map_core.gz_world import build_sdf
            world = "/tmp/living_map/world.sdf"; open(world, "w").write(build_sdf(map_path))
        acts += [ExecuteProcess(cmd=["gz", "sim", "-r", "-v", "2", world], output="screen", additional_env={"ROS_DOMAIN_ID": "10", "QT_QPA_PLATFORM": "xcb"}),
                 Node(package="living_map_core", executable="gz_bridge_node", name="gz_bridge", output="screen", additional_env=dict(env, ROS_DOMAIN_ID="10"))]
    if use_rviz: acts.append(ExecuteProcess(cmd=["rviz2", "-d", os.path.join(share, "rviz", "living_map.rviz")], output="screen", additional_env={"ROS_DOMAIN_ID": "10"}))
    acts.append(TimerAction(period=epoch - time.time() + 30.0, actions=[ExecuteProcess(cmd=["ros2", "run", "living_map_core", "airgap_auditor"], output="screen")]))
    return acts

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("speed", default_value="3.0", description="simulation speed factor"),
        DeclareLaunchArgument("auto_approve", default_value="true", description="false = operator clicks APPROVE DISPATCH"),
        DeclareLaunchArgument("fault", default_value="none", description="none | beacon_destroyed | writer_lost"),
        DeclareLaunchArgument("gazebo", default_value="auto", description="auto | true | false"),
        DeclareLaunchArgument("rviz", default_value="auto", description="auto | true | false"),
        DeclareLaunchArgument("map", default_value="", description="path to a map JSON file (default: maps/complex_1floor.json, the one-floor building; maps/proto_arena.json is the Phase 2 prototype arena)"),
        DeclareLaunchArgument("uplink", default_value="immediate", description="immediate (every beacon goes to the Command Post at once) | batched (120 s window)"),
        DeclareLaunchArgument("collect", default_value="1.0", description="seconds the Command Post collects related reports before issuing ONE order"),
        DeclareLaunchArgument("nav", default_value="direct", description="Executor navigation: direct = A* straight to the target (the Writer trail is only a safe-route hint); trail = follow every Writer waypoint (legacy)"),
        DeclareLaunchArgument("cycle", default_value="1.0", description="Command Post decision cycle in seconds: it decides ONCE per cycle, never per event"),
        DeclareLaunchArgument("crit", default_value="0.3", description="effective priority below which an event is critical and skips the collection wait (0 = most urgent, no upper limit)"),
        DeclareLaunchArgument("roles", default_value="ambulance,firetruck", description="which responder robots leave the van (add drone for a multi-floor map)"),
        OpaqueFunction(function=_setup)])
