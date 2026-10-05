#!/usr/bin/env bash
# Run in a SECOND terminal while the system is running. Shows what works, what does not, and WHY.
source /opt/ros/jazzy/setup.bash; source "${LIVING_MAP_WS:-$HOME/living_map_ws}/install/setup.bash"
ok(){ echo "  [OK]   $1"; }; bad(){ echo "  [FAIL] $1"; }
echo "== 1. duplicate / stale processes (each name must appear ONCE) =="
for n in writer_node mesh_node gateway_node command_post_node ambulance_node firetruck_node sim_view_node; do
  c=$(pgrep -fc "lib/living_map_core/$n"); [ "$c" = "1" ] && ok "$n x1" || bad "$n x$c  (0 = crashed or not started, 2+ = old copy still alive: run run_clean.sh)"
done
cd=$(pgrep -fc "lib/living_map_core/drone_node"); [ "$cd" = "0" ] || { [ "$cd" = "1" ] && ok "drone_node x1 (multi-floor role, started with roles:=ambulance,firetruck,drone)" || bad "drone_node x$cd"; }
echo "== 2. who owns UDP 9101 (Command Post, uplink) and UDP 9102 (van, downlink): exactly one owner each =="
ss -lunp 2>/dev/null | grep -E "9101|9102" | sed 's/^/    /' || echo "    nobody (Command Post is not listening)"
echo "== 3. is the Writer alive and dropping beacons? =="
ROS_DOMAIN_ID=10 timeout 8 ros2 topic echo /writer/pose --once --field data 2>/dev/null | python3 -c "
import sys,json
txt=[l.strip().strip(\"'\") for l in sys.stdin if l.strip().startswith(('{',\"'{\"))]
try:
    d=json.loads(txt[0]); print('    writer t=%.0fs beacons=%d done=%s dead=%s'%(d['t'],d['beacons'],d['done'],d['dead']))
except Exception:
    print('  [??]   could not read /writer/pose (echo may be slow or parse failed): falling back to the heartbeat below')"
grep "HEARTBEAT writer" ~/launch.log 2>/dev/null | tail -1 | sed 's/^/    last heartbeat: /'
echo "== 4. are beacons flowing van -> Command Post? =="
python3 - << 'PY'
import json, urllib.request
try:
    s = json.loads(urllib.request.urlopen("http://localhost:8080/state", timeout=4).read())
    print(f"    Command Post: sim time {s['now']} s, {len(s['records'])} beacon record(s)")
    for r, v in s["robots"].items(): print(f"    {r:10s} {v['state']:11s} {v['note']}")
    n = len(s["records"]); print("  [OK]   beacons are arriving" if n > 3 else "  [FAIL] only %d record(s): beacons are NOT reaching the Command Post (or it is too early)" % n)
except Exception as e: print("  [FAIL] cannot read the Command Post:", e)
PY
echo "== 5. last heartbeats in the launch log =="; grep -h HEARTBEAT ~/launch.log 2>/dev/null | tail -4 | cut -c1-220 | sed 's/^/    /'
echo "== 6. errors in the launch log =="; grep -hiE "error|traceback|exception|process has died" ~/launch.log 2>/dev/null | tail -8 | cut -c1-220 | sed 's/^/    /'
