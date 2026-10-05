"""Writes REPOSITORY_GUIDE.md at the workspace root: architecture, annotated tree, and a file-by-file reference.
The 'Defines' and 'Imports' columns are read from the code itself (ast), so they cannot go out of date.  Usage: python3 tools/make_repo_guide.py"""
import ast, os, glob, re
H = os.path.dirname(os.path.abspath(__file__)); PKG = os.path.join(H, ".."); WS = os.path.join(PKG, "..", ".."); P = os.path.join(PKG, "living_map_core")
D = {  # one-line purpose of every file (the part only a human can write)
 "beacon_protocol.py": "the 32-byte beacon v2 (pack / unpack, CRC16 + 3-byte MAC), egocentric hop geometry, and the priority / confidence / queue-score maths",
 "geo.py": "the entrance anchor: local ENU <-> WGS84 conversion (frame translation) with a position-uncertainty model",
 "sim_core.py": "the simulated world: map loading, sensors, line of sight, radio model, the robot's private map, and the multi-floor A* planner",
 "agents.py": "the Writer (GPS approach, exploration, beacon drops), the event Detector (2-of-3 rule), the beacon Mesh, the van GatewayCore and the StationLink",
 "adaptive_queue.py": "the van's uplink queue: priority 0 at once, the rest on 120 s / size / score",
 "command_post.py": "the Command Post: event store, priority aging, correlation into incidents, 1 s decision cycle, orders, ORDER_UPDATE replanning, robot health",
 "roles.py": "the Executors (ambulance, fire truck, drone): briefing, A* navigation with free-space assumption, job ordering, hazard handling, order updates",
 "simulation.py": "the one-process orchestrator that wires Writer, mesh, van, Command Post and Executors; also the metrics and the blind baseline",
 "standalone.py": "runs the whole system without ROS and serves the web pages (ports 8081 and 8080)",
 "viewstate.py": "the state object that the web pages read (robots, beacons, events, flows)",
 "sim_html.py": "the simulation web page (HTML + JavaScript string): map, robots, beacons, results card",
 "dashboard_html.py": "the Command Post web page: live map, ranked orders, incidents",
 "flow_html.py": "the data-flow web page: beacon table and the robot -> van -> Command Post flow",
 "nodes.py": "the ROS 2 nodes (thin wrappers around the code above): writer, beacon_mesh, van gateway, command_post, three Executors, sim_view, gz_bridge",
 "gz_world.py": "generates the Gazebo (Harmonic) world from the same map JSON, including the van, fires and victims",
 "audit.py": "checks the star topology: no robot may listen to anything but its own van topic; air-gap between the two ROS domains",
 "order.py": "command-line tool to approve / dispatch orders (`ros2 run living_map_core order ...`)",
 "offline.py": "offline demo and fault tests (three scenarios) without ROS",
 "__init__.py": "package marker (empty)",
}
OTHER = {
 "README.md": "front page: what the project is, architecture diagram, how to run, results (generated), limits",
 "CHANGELOG.md": "what changed between versions, including fixed measurement errors",
 "TROUBLESHOOTING.md": "what to do when something does not start or an Executor stays idle",
 "REPOSITORY_GUIDE.md": "this file",
 "LICENSE": "licence text (placeholder holder: change it)", ".gitignore": "keeps build/, install/, log/ and caches out of git",
 "src/living_map_core/README.md": "package-level notes: modules, beacon packet, detection, version notes",
 "src/living_map_core/package.xml": "ROS 2 package manifest", "src/living_map_core/setup.py": "Python package setup and installed data files (launch, maps, worlds, scripts)", "src/living_map_core/setup.cfg": "install paths for the ROS 2 entry points",
 "src/living_map_core/resource/living_map_core": "ament resource marker (empty)",
 "src/living_map_core/launch/living_map_fire_env.launch.py": "launches everything: arguments speed, auto_approve, fault, gazebo, rviz, map, roles, uplink, collect, cycle, crit, nav",
 "src/living_map_core/scripts/run_clean.sh": "one-command start: stops old runs, frees ports 8080 8081 9101 9102, sources ROS, launches",
 "src/living_map_core/scripts/check_run.sh": "after a minute of running: duplicate processes, port owners, writer alive, beacons flowing, last heartbeats, errors",
 "src/living_map_core/scripts/doctor.sh": "checks the machine: ROS, Gazebo, Python, ports",
 "src/living_map_core/scripts/install_ros_gz.sh": "installs the ROS 2 <-> Gazebo bridge packages",
 "src/living_map_core/maps/complex.json": "the 2-floor Industrial Complex (48 x 24 m, 0.5 m cells): grid rows, fires, gas, steam, people, animals, van, anchor",
 "src/living_map_core/maps/complex_1floor.json": "the same building, ground floor only (stairs and atrium become floor; floor-1 events removed)",
 "src/living_map_core/maps/proto_arena.json": "the 7.2 x 4.8 m one-floor prototype arena (0.1 m cells), with per-map sensor thresholds and a small van",
 "src/living_map_core/worlds/living_map_fire.sdf": "Gazebo world generated from complex.json",
 "src/living_map_core/worlds/proto_arena.sdf": "Gazebo world generated from proto_arena.json",
 "src/living_map_core/rviz/living_map.rviz": "RViz configuration (two floors, robots, beacons, hazards)",
 "src/living_map_core/test/test_core.py": "pytest tests: protocol, geometry, detector, mesh, gateway, Command Post rules, navigation, arena, GPS approach",
 "src/living_map_core/test/fake_ros_integration.py": "integration test of the ROS layer with a fake rclpy (real UDP and HTTP); checks the star topology and the order updates",
 "src/living_map_core/tools/make_map.py": "draws the 2-floor building and writes maps/complex.json",
 "src/living_map_core/tools/make_proto_arena.py": "defines the prototype arena once and writes its map, SVG plan and material list",
 "src/living_map_core/tools/collect_final.py": "ALL measurements of the one-floor project (baseline, faults, uplink modes, navigation, radio stress, hold rule, larger building); resumable; writes docs/results_final.json",
 "src/living_map_core/tools/bench_nav.py": "quick comparison of the two Executor navigation modes (trail / direct)",
 "src/living_map_core/tools/refresh_final_docs.py": "regenerates README, FAILURE_CASES, VIDEO_SCRIPT and the arena table from docs/results_final.json",
 "src/living_map_core/tools/make_final_figures.py": "draws the priority curves, the results figure and the mission trace + timeline from docs/results_final.json",
 "src/living_map_core/tools/make_diagrams.py": "draws architecture, sequence and beacon-frame diagrams (SVG + PNG)",
 "src/living_map_core/tools/make_repo_guide.py": "generates this guide",
 "src/living_map_core/docs/ARCHITECTURE.md": "modules, ROS graph, message schemas, rules the code enforces",
 "src/living_map_core/docs/FAILURE_CASES.md": "five failure cases with measured results",
 "src/living_map_core/docs/PHASE2_IMPLEMENTATION_PLAN.md": "timeline to Dec 1, bill of materials, physical test protocol, risks",
 "src/living_map_core/docs/PROTOTYPE_ARENA.md": "the plywood arena, the specification check, research and sources",
 "src/living_map_core/docs/SUBMISSION_CHECKLIST.md": "what is left to submit and where the documents disagree",
 "src/living_map_core/docs/VIDEO_SCRIPT.md": "shot list and narration for the demo video",
 "src/living_map_core/docs/results_final.json": "measured results of the one-floor project (generated by tools/collect_final.py)",
 "src/living_map_core/docs/proto_arena_materials.md": "wall length, plywood sheets and cut list for the arena", "src/living_map_core/docs/proto_arena_plan.svg": "printable top view of the arena", "src/living_map_core/docs/proto_arena_plan.png": "the same plan as an image",
 "src/living_map_core/docs/diagrams/": "architecture, sequence, state machines, data flow, beacon frame (SVG + PNG), fig_priority, fig_results, fig_trace (PNG)",
}
def info(path):
    t = ast.parse(open(path).read()); cls = [n.name for n in t.body if isinstance(n, ast.ClassDef)]; fn = [n.name for n in t.body if isinstance(n, ast.FunctionDef) and not n.name.startswith("_")]
    imp = sorted({(n.module or "").split(".")[-1] if (n.level or (n.module or "").startswith("living_map_core")) else None for n in ast.walk(t) if isinstance(n, ast.ImportFrom)} - {None, ""} - {"living_map_core"})
    return cls, fn, imp
rows, deps = [], {}
for f in sorted(glob.glob(os.path.join(P, "*.py"))):
    n = os.path.basename(f); cls, fn, imp = info(f); deps[n[:-3]] = [i for i in imp if os.path.exists(os.path.join(P, i + ".py"))]
    d = ", ".join(f"`{c}`" for c in cls[:8]) + ((", " if cls and fn else "") + ", ".join(f"`{x}()`" for x in fn[:8]) if fn else "")
    rows.append(f"| `{n}` | {sum(1 for _ in open(f))} | {D.get(n, '')} | {d or '-'} | {', '.join(deps[n[:-3]]) or '-'} |")
tools_t = []
tree = []
for root, dirs, files in os.walk(WS):
    dirs[:] = sorted(d for d in dirs if d not in ("__pycache__", ".pytest_cache", "build", "install", "log", ".git", "legacy_antigravity"))
    for fl in sorted(files):
        if fl.endswith((".pyc", ".bak")): continue
        rel = os.path.relpath(os.path.join(root, fl), WS).replace(os.sep, "/")
        if rel.startswith("src/living_map_core/docs/diagrams/"): continue
        tree.append(rel)
tree = sorted(set(tree + ["src/living_map_core/docs/diagrams/"]))
def desc(p):
    if p in OTHER: return OTHER[p]
    b = os.path.basename(p)
    if p.startswith("src/living_map_core/living_map_core/") and b in D: return D[b]
    return ""
trows = "\n".join(f"| `{p}` | {desc(p)} |" for p in tree if not p.startswith("src/living_map_core/living_map_core/"))
dep_lines = "\n".join(f"* `{k}` imports: " + (", ".join(f"`{x}`" for x in v) if v else "nothing from this package") for k, v in sorted(deps.items()) if k != "__init__")
guide = f"""# Repository guide: architecture and content of every file

Repository root = a ROS 2 workspace (`living_map_ws`). Clone it as `~/living_map_ws`, then `colcon build --symlink-install`.

## 1. Architecture in one page
![architecture](src/living_map_core/docs/diagrams/architecture.png)

The code has four layers. **Plain Python never imports ROS**; the ROS layer is a thin wrapper, so everything is testable without ROS.

| layer | files | role |
|---|---|---|
| 1. Core logic | `beacon_protocol`, `geo`, `sim_core`, `adaptive_queue` | beacon format, frame translation, the world and its sensors / radio / planner, the uplink queue |
| 2. Actors | `agents` (Writer, Detector, Mesh, GatewayCore, StationLink), `command_post`, `roles` (Executors) | the robots, the van and the Command Post |
| 3. Orchestration | `simulation` (one process), `standalone` (+ web pages `sim_html`, `dashboard_html`, `flow_html`, `viewstate`), `nodes` (ROS 2), `gz_world` (Gazebo) | wires the actors together in three ways: pure Python, web, ROS 2 + Gazebo |
| 4. Tooling | `tools/`, `test/`, `scripts/`, `launch/` | measurements, diagrams, report, tests, one-command start |

Data path: **Writer** (`agents.WriterAgent`) -> **beacon mesh** (`agents.Mesh`) -> **van** (`agents.GatewayCore` + `adaptive_queue`) -> UDP uplink -> **Command Post** (`command_post.CommandPost`) -> briefing and `ORDER_UPDATE` back through the van -> **Executors** (`roles.Responder`). Robots only talk to the van (`audit.py` checks it).

### Which module imports which (read from the code)
{dep_lines}

## 2. Every file
| file | purpose |
|---|---|
{trows}

## 3. The Python package `living_map_core/` in detail
"Defines" lists the public classes and functions found in the file; "Imports" lists the modules of this package it uses.

| file | lines | purpose | defines | imports |
|---|---|---|---|---|
""" + "\n".join(rows) + """

## 4. How to run, test and regenerate
```bash
colcon build --symlink-install && source install/setup.bash
bash src/living_map_core/scripts/run_clean.sh speed:=1                      # full system, three web pages (default map: the one-floor arena)
python3 -m living_map_core.standalone --speed 6                              # no ROS
python3 -m pytest src/living_map_core/test/test_core.py -q                   # all tests, several minutes (run from src/living_map_core)
python3 src/living_map_core/tools/collect_final.py                           # measurements -> docs/results_final.json
python3 src/living_map_core/tools/make_diagrams.py && python3 src/living_map_core/tools/make_final_figures.py && python3 src/living_map_core/tools/refresh_final_docs.py
```
Pass `map:=<path>` to use another map (for example `maps/proto_arena.json` or `maps/complex_1floor.json`).

## 5. What is deliberately not in the repository
`build/`, `install/`, `log/`, caches, the old downloaded zips, the challenge PDFs (organisers' material) and the old sprint-plan PDF (outdated). The earlier `legacy_antigravity/` prototype was removed because the current code replaces it and nothing imports it.
"""
open(os.path.join(WS, "REPOSITORY_GUIDE.md"), "w").write(guide); print("guide written:", len(guide.splitlines()), "lines;", len(tree), "files listed")
