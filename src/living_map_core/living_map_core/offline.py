"""Whole system without ROS: python3 -m living_map_core.offline  (demo)   /  fault_tests (three scenarios with assertions)."""
import sys, json, argparse
from .simulation import Simulation, run

def summary(s):
    m = s.view.metrics or {}
    return {k: m.get(k) for k in ("with_beacons_s", "blind_s", "gain_pct", "position_error_m", "position_error_max_m", "fires_out", "fires", "rescued", "people", "beacons", "repeaters", "delivery", "rejected", "skipped")}

def main_demo():
    ap = argparse.ArgumentParser(); ap.add_argument("--fault", default="none"); ap.add_argument("--map", default=None); ap.add_argument("--quiet", action="store_true"); a = ap.parse_args()
    s = run(a.fault, a.map, verbose=not a.quiet); print("\n=== RESULTS ==="); print(json.dumps(summary(s), indent=1)); return s

def main_faults():
    rows, ok = [], True
    for name, need in (("none", dict(fires_out=4, rescued=4)), ("beacon_destroyed", dict(fires_out=4, rescued=4)), ("writer_lost", dict(fires_out=2, rescued=3))):
        s = run(name); m = summary(s); good = s.final and all((m[k] or 0) >= v for k, v in need.items()); ok &= good
        rows.append((name, m["fires_out"], m["fires"], m["rescued"], m["people"], m["with_beacons_s"], m["blind_s"], "PASS" if good else "FAIL"))
    print(f"{'fault':18s} fires  rescued  first-person(beacons/blind)  result")
    for r in rows: print(f"{r[0]:18s} {r[1]}/{r[2]}    {r[3]}/{r[4]}      {r[5]} s / {r[6]} s            {r[7]}")
    sys.exit(0 if ok else 1)

if __name__ == "__main__": main_demo()
