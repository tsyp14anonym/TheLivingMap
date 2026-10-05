"""Integration test of the ROS layer WITHOUT ROS: a tiny fake rclpy routes topics in-process; UDP and HTTP are real.
Run:  python3 test/fake_ros_integration.py        (this exercises nodes.py logic, not real DDS / Gazebo / RViz)"""
import sys, types, time, json, urllib.request, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
BUS, TIMERS, PUBLISHED, LOG = {}, [], {}, []
class String:
    def __init__(self, data=""): self.data = data
class Auto:
    def __init__(self, **k):
        for a, b in k.items(): setattr(self, a, b)
    def __getattr__(self, k):
        if k.startswith("__"): raise AttributeError(k)
        v = Auto(); object.__setattr__(self, k, v); return v
class Marker(Auto): ADD = 0; CUBE = 1; SPHERE = 2; CYLINDER = 3; LINE_STRIP = 4; LINE_LIST = 5; CUBE_LIST = 6; TEXT_VIEW_FACING = 9
_SITES = {}
class _Log:
    """Mimics real rclpy: ONE call site (file, line, bytecode offset) may only ever log with ONE severity, otherwise
    ValueError('Logger severity cannot be changed between calls.')  -- this is what crashed the van on a real machine."""
    def __init__(self, n): self.n = n
    def _log(self, m, sev):
        f = sys._getframe(2); key = (self.n, f.f_code.co_filename, f.f_lineno, f.f_lasti)
        if _SITES.setdefault(key, sev) != sev: raise ValueError("Logger severity cannot be changed between calls.")
        LOG.append((self.n, m))
    def info(self, m): self._log(m, "info")
    def warn(self, m): self._log(m, "warn")
    def error(self, m): self._log(m, "error")
class Node:
    def __init__(self, name): self.name = name; self.params = {}; self.subs = []
    def get_name(self): return self.name
    def declare_parameter(self, k, v): self.params.setdefault(k, v)
    def get_parameter(self, k): return types.SimpleNamespace(value=self.params[k])
    def get_logger(self): return _Log(self.name)
    def get_clock(self): return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: Auto()))
    def create_publisher(self, t, topic, q):
        def pub(m, topic=topic): PUBLISHED[topic] = PUBLISHED.get(topic, 0) + 1; [cb(m) for cb in list(BUS.get(topic, []))]
        return types.SimpleNamespace(publish=pub)
    def create_subscription(self, t, topic, cb, q): self.subs.append(topic); BUS.setdefault(topic, []).append(cb)
    def create_timer(self, p, cb): TIMERS.append(cb)
    def destroy_node(self): pass
def mod(name, **attrs):
    m = types.ModuleType(name); [setattr(m, k, v) for k, v in attrs.items()]; sys.modules[name] = m; return m
mod("rclpy", node=mod("rclpy.node", Node=Node)); mod("std_msgs", msg=mod("std_msgs.msg", String=String))
mod("nav_msgs", msg=mod("nav_msgs.msg", OccupancyGrid=Auto)); mod("visualization_msgs", msg=mod("visualization_msgs.msg", Marker=Marker, MarkerArray=Auto))
mod("geometry_msgs", msg=mod("geometry_msgs.msg", Point=Auto, TransformStamped=Auto)); mod("tf2_ros", StaticTransformBroadcaster=lambda n: types.SimpleNamespace(sendTransform=lambda t: None))
from living_map_core import nodes, audit
epoch, speed, orig = time.time(), float(os.environ.get("SPEED", "60")), nodes._params
def P(node, **d):
    d = dict(d); d["epoch"] = epoch; d["speed"] = speed
    if "port" in d: d["port"] = 8098 if node.name == "sim_view" else 8099
    return orig(node, **d)
nodes._params = P
POSES = []
nodes.GzPoser = lambda world, log: types.SimpleNamespace(set=lambda n, x, y, z, yaw: POSES.append((n, round(x, 1), round(y, 1), round(z, 1))))
cp = nodes.CommandPostNode(); gw = nodes.GatewayNode(); w = nodes.WriterNode(); m = nodes.MeshNode()
rs = {r: getattr(nodes.ResponderNode, "__call__", None) and nodes.ResponderNode(r) for r in ("ambulance", "firetruck", "drone")}
sv = nodes.SimViewNode(); gz = nodes.GazeboBridgeNode(); allnodes = [w, *rs.values(), gw, m, sv, gz]
t_end = time.time() + float(os.environ.get("WALL", "240"))
while time.time() < t_end and not sv.final:
    for cb in TIMERS: cb()
    time.sleep(0.002)
time.sleep(3.0)
for cb in TIMERS: cb()
get = lambda port, path: json.loads(urllib.request.urlopen(f"http://localhost:{port}{path}").read())
st = get(8098, "/simstate"); wd = get(8098, "/world"); cps = get(8099, "/state")
print("sim_view: t", st["t"], "| stages", st["stages"], "| beacons", len(st["beacons"]), "| robots", sorted(r["role"] for r in st["robots"]), "| known floors", len(st["known"] or []), "| metrics", bool(st["metrics"]))
print("world floors", len(wd["floors"]), "| command post records", len(cps["records"]), "| robots", {k: v["state"] for k, v in cps["robots"].items()})
if st["metrics"]: print({k: v for k, v in st["metrics"].items() if k in ("with_beacons_s", "blind_s", "fires_out", "fires", "rescued", "people")})
print("rviz publishes: markers", PUBLISHED.get("/living_map/markers"), "| maps f0/f1", PUBLISHED.get("/living_map/known_map_f0"), PUBLISHED.get("/living_map/known_map_f1"))
pn = {p[0] for p in POSES}; print("gazebo: poses sent", len(POSES), "| movers", sorted(x for x in pn if not x.startswith("bcn")), "| beacons placed", sum(1 for x in pn if x.startswith("bcn")), "| drone max z", max([p[3] for p in POSES if p[0] == "drone"] or [0]))
print("files:", sorted(f for f in os.listdir("/tmp/living_map") if f.startswith("mission")))
# star topology: nobody but the van subscribes to /station/from/*, robots subscribe only to their own /station/to/<role>
subs = {n.name: set(n.subs) for n in allnodes}
van_only = all(not any(t.startswith("/station/from/") for t in subs[r]) for r in ("writer", "ambulance", "firetruck", "drone"))
viol = audit.star_violations({r: subs[r] for r in ("writer", "ambulance", "firetruck", "drone")})
print("STAR topology: robots listen to", {r: sorted(subs[r]) for r in ("writer", "ambulance", "firetruck", "drone")}); print("violations:", viol, "| only van hears robots:", van_only and any(t.startswith("/station/from/") for t in subs["outside_network_gateway"]))
print("DEBUG gateway received", gw.core.received, "waiting", {k: len(v) for k, v in gw.core.waiting.items()}, "queue", len(gw.core.q.pending) if hasattr(gw.core.q, "pending") else "?", "| mesh sent/delivered", m.mesh.sent, m.mesh.delivered, "undelivered", sorted(m.mesh.undelivered), "| writer beacons", len(w.agent.beacons), "| cp records", len(cp.cp.records))
print("DEBUG2 fires I", [(f["id"], round(f["I"], 2)) for f in sv.world.fires], "cp.out", sorted(cp.cp.out), "failed", cp.cp.failed, "fire records", [(r["bid"], r["floor"], r["id"]) for r in cp.cp.records.values() if r["type"] == "FIRE"])
print("DEBUG2 logs", [m[:110] for n, m in LOG if ("unreachable" in m or "no way" in m or "LOW BATT" in m or "payload" in m or "HEAT" in m)])
print("BEACON STATES physical:", {b: v[0] for b, v in sv.view.beacon_states.items() if v[0] not in ("DROPPED",)})
print("flow events seen by sim_view:", sorted({e["kind"] for e in sv.view.flow}), "| table rows at CP:", len(cps["table"]), "| CP states:", sorted({r["state"] for r in cps["table"]}))
flow_ok = {"DROP", "RF", "WRITE"} <= {e["kind"] for e in sv.view.flow} and {"UPLINK", "DISPATCH", "STATE"} <= {e["kind"] for e in cps["flow"]} and any(r["state"] == "RESOLVED" for r in cps["table"])
ok = st["metrics"] and st["metrics"]["fires_out"] == st["metrics"]["fires"] and st["metrics"]["rescued"] >= st["metrics"]["people"] - 1 and not viol and flow_ok
if os.environ.get("DEBUG"): [print("  LOG", n, m[:150]) for n, m in LOG if any(k in m for k in ("FIRETRUCK","DRONE","unreachable","OUT","dispatched","DISPATCHED","fire","FIRE")) and n in ("firetruck","drone","sim_view","command_post","outside_network_gateway") or "firetruck" in m.lower()]
dl = [m for n, m in LOG if "DOWNLINK" in m or "order updated" in m or "ORDER UPDATE" in m]
print("ORDER_UPDATE relayed by the van:", gw.n_down, "| examples:", [m[:120] for m in dl[:3]])
beacon_readers = [r for r in ("writer", "ambulance", "firetruck", "drone") if "/beacon_state" in subs[r]]
print("robots reading /beacon_state:", beacon_readers)
ok = ok and not beacon_readers
print("INTEGRATION:", "PASS" if ok else "FAIL"); sys.exit(0 if ok else 1)
