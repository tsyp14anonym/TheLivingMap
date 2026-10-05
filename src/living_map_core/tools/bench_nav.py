"""Compare Executor navigation: legacy 'trail' (follow every Writer waypoint) vs 'direct' (A* straight to the target) on the same maps/seeds.
Usage: python3 tools/bench_nav.py [seed ...]      (about 35 s of CPU per run)"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from living_map_core.simulation import run

def summarize(s):
    m = s.view.metrics; rows = [x for v in m["robots"].values() for x in v]
    return dict(first_rescue=m["with_beacons_s"], fires_out=f'{m["fires_out"]}/{m["fires"]}', rescued=f'{m["rescued"]}/{m["people"]}',
                dist=round(sum(x["dist"] for x in rows)), busy=round(sum(x["time"] for x in rows)), missions=len(rows), unreachable=sum(1 for x in rows if not x["ok"]), t_end=round(s.t))

if __name__ == "__main__":
    seeds = [int(a) for a in sys.argv[1:]] or [7]
    print(f'{"seed":>4} {"nav":>6} | {"1st rescue s":>12} {"fires out":>9} {"rescued":>7} {"distance m":>10} {"busy s":>7} {"missions":>8} {"failed":>6} {"end s":>6}')
    for seed in seeds:
        for nav in ("trail", "direct"):
            t0 = time.time(); s = run(seed=seed, nav=nav); r = summarize(s)
            print(f'{seed:>4} {nav:>6} | {str(r["first_rescue"]):>12} {r["fires_out"]:>9} {r["rescued"]:>7} {r["dist"]:>10} {r["busy"]:>7} {r["missions"]:>8} {r["unreachable"]:>6} {r["t_end"]:>6}   ({time.time()-t0:.0f}s wall)', flush=True)
