"""Give orders to the Command Post from the terminal.
  python3 -m living_map_core.order status
  python3 -m living_map_core.order approve              # approve every robot that is ready (when auto_approve is false)
  python3 -m living_map_core.order dispatch ambulance   # send that robot NOW (skips the briefing wait and the 'fire next to victim' hold)
Roles: ambulance, firetruck, drone.  A robot is only sent if the Command Post already has a target for it."""
import sys, json, urllib.request
def _call(port, path, body=None):
    req = urllib.request.Request(f"http://localhost:{port}{path}", data=None if body is None else json.dumps(body).encode(), method="GET" if body is None else "POST")
    return urllib.request.urlopen(req, timeout=5).read().decode()
def main():
    a = sys.argv[1:]; port = 8080
    if "--port" in a: i = a.index("--port"); port = int(a[i + 1]); del a[i:i + 2]
    if not a or a[0] not in ("status", "approve", "dispatch"): print(__doc__); return 1
    try:
        if a[0] == "status":
            s = json.loads(_call(port, "/state")); print(f"t={s['now']} s, {len(s['records'])} records at the Command Post")
            for r, v in s["robots"].items(): print(f"  {r:10s} {v['state']:10s} {v['note']}")
        elif a[0] == "approve": _call(port, "/approve", {}); print("approved")
        else:
            if len(a) < 2 or a[1] not in ("ambulance", "firetruck", "drone"): print("usage: dispatch ambulance|firetruck|drone"); return 1
            _call(port, "/dispatch", {"role": a[1]}); print(f"dispatch order sent for {a[1]}")
    except OSError as e: print("cannot reach the Command Post on port", port, "- is the system running?", e); return 1
if __name__ == "__main__": sys.exit(main())
