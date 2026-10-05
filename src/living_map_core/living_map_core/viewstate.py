"""Everything the explanatory simulation page needs, filled by the standalone runner or by ROS topics."""
import math
from .sim_core import CH
STAGE_NAMES = ["Explore", "Detect", "Drop beacons", "Van gateway", "Command Post", "Dispatch robots", "Response"]

def grid_rows(g): return ["".join(CH[v] for v in row) for row in g]

class ViewState:
    def __init__(self, controls=False, cp_url="/cp"):
        self.controls, self.cp_url = controls, cp_url
        self.paused, self.speed, self.fault = False, 4.0, "none"
        self.reset()
    def reset(self):
        self.t = 0.0; self.robots, self.known, self.sensors, self.metrics = {}, None, None, None
        self.beacons, self.log, self.trails, self.truth, self.board, self.awaiting, self.wp = {}, [], {}, dict(fires=[], humans=[], animals=[]), None, [], {}
        self.flags = dict(gw=False, uplink=False, dispatched=False, response=False); self.flow, self.flow_seq, self.beacon_states = [], 0, {}
    def set_robot(self, role, x, y, f, state="", done=False, battery=None):
        self.robots[role] = dict(role=role, x=round(x, 2), y=round(y, 2), f=f, state=state, done=done, battery=battery)
        tr = self.trails.setdefault(role, [])
        if not tr or math.hypot(tr[-1][0] - x, tr[-1][1] - y) > 0.4 or tr[-1][2] != f:
            tr.append([round(x, 2), round(y, 2), f])
            if len(tr) > 600: tr.pop(0)
    def add_beacon(self, bid, typ, prio, x, y, f, parent=0): self.beacons[bid] = dict(bid=bid, type=typ, prio=prio, x=x, y=y, f=f, parent=parent, alive=True)
    def kill_beacon(self, bid):
        if bid in self.beacons: self.beacons[bid]["alive"] = False; self.beacon_states[bid] = ["DESTROYED", "fire/collapse", round(self.t, 1)]
    def add_flow(self, kind, src, dst, label):
        self.flow_seq += 1; self.flow.append(dict(seq=self.flow_seq, t=round(self.t, 1), kind=kind, src=src, dst=dst, label=label))
        if len(self.flow) > 200: self.flow.pop(0)
    def set_state(self, bid, state, by): self.beacon_states[bid] = [state, by, round(self.t, 1)]
    def add_log(self, who, msg, t=None):
        self.log.append(dict(t=round(self.t if t is None else t, 1), who=who, msg=msg))
        if len(self.log) > 400: self.log.pop(0)
    def set_truth(self, w):
        self.truth = dict(fires=[dict(id=f["id"], x=f["x"], y=f["y"], f=f["floor"], I=round(f["I"], 2)) for f in w.fires],
                          gas=[dict(id=g["id"], x=g["x"], y=g["y"], f=g["floor"], I=round(g["I"], 2)) for g in w.gas],
                          humans=[dict(id=h["id"], x=h["x"], y=h["y"], f=h["floor"], rescued=h["rescued"]) for h in w.humans],
                          animals=[dict(id=h["id"], x=h["x"], y=h["y"], f=h["floor"], rescued=h["rescued"]) for h in w.animals])
    def stages(self):
        rb = self.robots; done = bool(self.metrics)
        return [self.t > 0, len(self.beacons) > 0, len(self.beacons) > 0, self.flags["gw"], self.flags["uplink"], self.flags["dispatched"], done]
    def sim_json(self):
        return dict(t=round(self.t, 1), robots=list(self.robots.values()), known=self.known, beacons=list(self.beacons.values()), trails=self.trails, truth=self.truth,
                    sensors=self.sensors, log=self.log[-90:], metrics=self.metrics, board=self.board, awaiting=self.awaiting, stages=self.stages(),
                    controls=self.controls, paused=self.paused, speed=self.speed, fault=self.fault, cp_url=self.cp_url, flow=self.flow[-120:], beacon_states=self.beacon_states)
