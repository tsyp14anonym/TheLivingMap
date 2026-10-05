"""Auditor. (1) Air-gap: inside (domain 10) and outside (domain 20) share no node and no topic.
(2) Star topology: robots have no topic to each other; only the van (outside_network_gateway) listens to /station/from/*."""
import os, re, subprocess, sys

ROBOTS = ("writer", "ambulance", "firetruck", "drone")
ALLOWED_EXTRA = {"/world/action", "/parameter_events", "/rosout"}     # /world/action = simulator physics plumbing (the "world"), not a communication channel. /beacon_state is NOT allowed: a robot may not read what another robot wrote

def _ros(args, domain):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    try: return subprocess.run(["ros2"] + args, env=env, capture_output=True, text=True, timeout=25).stdout
    except Exception as e: print("ros2 CLI failed:", e); return ""

def _set(args, domain): return {l.strip() for l in _ros(args, domain).splitlines() if l.strip()}

def subscriptions(node_info_text):
    m = re.search(r"Subscribers:(.*?)(?:Publishers:|Service Servers:|$)", node_info_text, re.S)
    return {l.strip().split(":")[0] for l in (m.group(1).splitlines() if m else []) if l.strip().startswith("/")}

def star_violations(subs_by_robot):
    """subs_by_robot: {robot: set(topics it subscribes to)} -> list of problems."""
    bad = []
    for r, subs in subs_by_robot.items():
        for t in subs:
            if t in ALLOWED_EXTRA or t == f"/station/to/{r}": continue
            bad.append(f"{r} subscribes to {t} (robots may only listen to the van on /station/to/{r}; everything between robots goes through the Command Post)")
    return bad

def main():
    ni, no = _set(["node", "list"], 10), _set(["node", "list"], 20)
    ti = _set(["topic", "list"], 10) - {"/rosout", "/parameter_events"}; to = _set(["topic", "list"], 20) - {"/rosout", "/parameter_events"}
    print("INSIDE  nodes :", sorted(ni)); print("OUTSIDE nodes :", sorted(no)); problems = []
    if ni & no: problems.append(f"nodes visible in both domains: {sorted(ni & no)}")
    if ti & to: problems.append(f"topics shared across domains: {sorted(ti & to)}")
    if "/command_post" in ni: problems.append("command_post is reachable from inside")
    if set("/" + r for r in ROBOTS) & no: problems.append("a robot is reachable from outside")
    if not ni or not no: problems.append("one domain is empty (is the system running?)")
    subs = {r: subscriptions(_ros(["node", "info", "/" + r], 10)) for r in ROBOTS if "/" + r in ni}
    problems += star_violations(subs)
    if problems:
        print("AUDIT: FAIL"); [print(" -", p) for p in problems]; sys.exit(1)
    print("AUDIT: PASS  air-gap intact (only the van's UDP uplink and the pre-entry briefing files cross) and STAR topology holds (robots listen only to the van)")

if __name__ == "__main__": main()
