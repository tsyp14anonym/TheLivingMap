"""Derives maps/complex_1floor.json (the one-floor building) from maps/complex.json: ground floor only, the stairs and the atrium shaft become floor,
floor-1 events are removed.  The only addition is `fault_times` (when the simulated faults strike, in seconds).  Usage: python3 tools/make_1floor.py"""
import json, os
H = os.path.dirname(os.path.abspath(__file__)); maps = os.path.join(H, "..", "maps")
d = json.load(open(os.path.join(maps, "complex.json"))); f0 = d["floors"][0]
f0["rows"] = [r.replace("S", ".").replace("A", ".") for r in f0["rows"]]
d["floors"] = [f0]
for k in ("fires", "gas", "steam", "humans", "animals"): d[k] = [o for o in d[k] if o.get("floor", 0) == 0]
d["name"] = "Industrial Complex (1 floor)"; d["legend"] = {k: v for k, v in d["legend"].items() if k not in ("S", "A")}
d["fault_times"] = {"beacon_destroyed": 120.0, "writer_lost": 150.0}          # the mission lasts about 380 s: strike while the Writer is still exploring
json.dump(d, open(os.path.join(maps, "complex_1floor.json"), "w"), indent=0); print("wrote maps/complex_1floor.json")
