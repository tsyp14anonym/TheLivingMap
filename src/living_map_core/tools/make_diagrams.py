"""Draws the report/README diagrams into docs/diagrams (SVG + PNG). Usage: python3 tools/make_diagrams.py"""
import os, math
from xml.sax.saxutils import escape as esc
import cairosvg
OUT = os.path.join(os.path.dirname(__file__), "..", "docs", "diagrams"); os.makedirs(OUT, exist_ok=True)
F = 'font-family="DejaVu Sans, Arial, sans-serif"'
C = dict(inside="#fff3e0", ona="#e3f2fd", out="#e8f5e9", box="#ffffff", line="#37474f", air="#c62828", acc="#1565c0", ok="#2e7d32", warn="#ef6c00")

def box(x, y, w, h, title, lines=(), fill="#fff", stroke=C["line"], bold=True, fs=13, r=8):
    s = f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{r}" fill="{fill}" stroke="{stroke}" stroke-width="1.6"/>'
    s += f'<text x="{x + w/2}" y="{y + 20}" text-anchor="middle" font-size="{fs}" font-weight="{"bold" if bold else "normal"}" {F}>{esc(title)}</text>'
    for i, l in enumerate(lines): s += f'<text x="{x + w/2}" y="{y + 38 + i*14}" text-anchor="middle" font-size="10.5" fill="#37474f" {F}>{esc(l)}</text>'
    return s
def arrow(x1, y1, x2, y2, label="", color=C["acc"], dash=False, lx=0, ly=-6, fs=10.5, both=False):
    d = ' stroke-dasharray="6 4"' if dash else ""
    s = f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="2"{d} marker-end="url(#a{color[1:]})"' + (f' marker-start="url(#s{color[1:]})"' if both else "") + '/>'
    if label: s += f'<text x="{(x1+x2)/2 + lx}" y="{(y1+y2)/2 + ly}" text-anchor="middle" font-size="{fs}" fill="{color}" {F}>{esc(label)}</text>'
    return s
def defs(colors):
    d = "<defs>"
    for c in colors:
        d += f'<marker id="a{c[1:]}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
        d += f'<marker id="s{c[1:]}" viewBox="0 0 10 10" refX="1" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M10,0 L0,5 L10,10 z" fill="{c}"/></marker>'
    return d + "</defs>"
def svg(w, h, body, colors):
    return f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">{defs(colors)}<rect width="100%" height="100%" fill="white"/>{body}</svg>'
def save(name, s, scale=2.0):
    p = os.path.join(OUT, name); open(p + ".svg", "w").write(s); cairosvg.svg2png(bytestring=s.encode(), write_to=p + ".png", scale=scale); print("wrote", name)

# ---------------------------------------------------------------- 1. architecture
def num(x, y, n, col):
    return f'<circle cx="{x}" cy="{y}" r="10" fill="white" stroke="{col}" stroke-width="2"/><text x="{x}" y="{y+4}" text-anchor="middle" font-size="11" font-weight="bold" fill="{col}" {F}>{n}</text>'
def architecture():
    b = []
    b.append(f'<text x="550" y="26" text-anchor="middle" font-size="17" font-weight="bold" {F}>The Living Map: system architecture (star topology, one crossing)</text>')
    b.append(f'<rect x="10" y="44" width="500" height="520" rx="12" fill="{C["inside"]}" stroke="#bf360c" stroke-width="1.5"/><text x="26" y="68" font-size="13" font-weight="bold" fill="#bf360c" {F}>INSIDE: GPS-denied, no external link (ROS domain 10)</text>')
    b.append(f'<rect x="530" y="44" width="170" height="520" rx="12" fill="{C["ona"]}" stroke="{C["acc"]}" stroke-width="1.5"/><text x="615" y="66" text-anchor="middle" font-size="11.5" font-weight="bold" fill="{C["acc"]}" {F}>OUTSIDE NETWORK AREA</text><text x="615" y="82" text-anchor="middle" font-size="10.5" fill="{C["acc"]}" {F}>the van: the only crossing</text>')
    b.append(f'<rect x="730" y="44" width="360" height="520" rx="12" fill="{C["out"]}" stroke="{C["ok"]}" stroke-width="1.5"/><text x="910" y="68" text-anchor="middle" font-size="13" font-weight="bold" fill="{C["ok"]}" {F}>OUTSIDE: Command Post (domain 20)</text>')
    # inside: writer + mesh on top, executors in a row, rule box
    b.append(box(26, 100, 215, 100, "Writer robot", ["GPS to the door, then NO GPS:", "LiDAR exploration + odometry;", "temp + thermal + gas, 2-of-3 rule;", "decides what to record, drops beacons"], C["box"]))
    b.append(box(266, 100, 230, 100, "Beacon mesh (32 B frames)", ["egocentric hops, mirror frames", "store-and-forward, ARQ x3,", "repeaters where the link is weak", "CRC16 + 3-byte MAC"], C["box"]))
    b.append(box(26, 262, 225, 74, "Ambulance", ["rescues people and animals;", "waits for 'hazard cleared'"], C["box"]))
    b.append(box(271, 262, 225, 74, "Fire truck", ["extinguishes fires,", "seals gas leaks"], C["box"]))
    b.append(f'<text x="26" y="250" font-size="12" font-weight="bold" fill="#bf360c" {F}>Executors (robots that leave the van)</text>')
    b.append(box(26, 392, 470, 156, "No robot-to-robot channel, not even a hidden one", ["Robots publish only to the van and listen only to the van.", "They never read beacon state written by another robot.", "What one robot must know about another arrives as a", "message from the Command Post, relayed by the van", "(briefing before entry, ORDER_UPDATE while inside).", "audit.py and a test fail if a robot listens to anything else."], "#fff8e1", "#bf360c", fs=12))
    # ona
    b.append(box(540, 100, 150, 170, "Van gateway", ["decode, CRC16 + MAC", "chain hops from the", "entrance anchor", "ENU -> WGS84 (GPS)", "adaptive queue:", "P0 goes out at once,", "others 120 s / size / score"], C["box"], C["acc"]))
    b.append(box(540, 292, 150, 110, "Station link", ["robot <-> van:", "status, orders,", "exploration complete", "(protocol TBD)"], C["box"], C["acc"]))
    # outside
    b.append(box(742, 100, 160, 64, "Event store", ["id, type, location, t0,", "base + effective pi"], C["box"], C["ok"]))
    b.append(box(916, 100, 160, 64, "Correlator", ["fire + gas, victim + hazard", "-> one incident"], C["box"], C["ok"]))
    b.append(box(742, 178, 160, 64, "Priority manager", ["pi(t) = pi0 e^(-0.004 t)", "lower number = more urgent"], C["box"], C["ok"]))
    b.append(box(916, 178, 160, 64, "Planner, 1 s cycle", ["aggregate, preempt, hold", "low-urgency jobs"], C["box"], C["ok"]))
    b.append(box(742, 256, 160, 64, "Executor selection", ["by capability,", "lost-robot fallback"], C["box"], C["ok"]))
    b.append(box(916, 256, 160, 64, "Live map + table", ["web pages :8080 :8081,", "ranked orders, data flow"], C["box"], C["ok"]))
    b.append(box(742, 334, 334, 76, "Replanning of running orders", ["ORDER_UPDATE through the van: new hazard,", "hazard cleared, new job. A robot silent for 35 s", "is LOST and its jobs are re-assigned."], C["box"], C["ok"], fs=12))
    b.append(box(742, 424, 334, 124, "Frame translation (entrance anchor)", ["private robot coordinates -> real GPS:", "X = x cos t - y sin t,   Y = x sin t + y cos t", "lat = lat0 + Y / R", "lon = lon0 + X / (R cos lat0)", "anchor (lat0, lon0, t) is fixed at the entrance door"], C["box"], C["ok"], fs=12))
    # arrows (numbered)
    b.append(arrow(241, 150, 266, 150, "", C["warn"])); b.append(num(253, 138, 1, C["warn"]))
    b.append(arrow(496, 150, 540, 150, "", C["warn"])); b.append(num(518, 138, 2, C["warn"]))
    for x in (138, 383): b.append(f'<line x1="{x}" y1="336" x2="{x}" y2="372" stroke="{C["acc"]}" stroke-width="2"/>')
    b.append(f'<line x1="138" y1="372" x2="383" y2="372" stroke="{C["acc"]}" stroke-width="2"/><line x1="383" y1="372" x2="510" y2="372" stroke="{C["acc"]}" stroke-width="2"/>')
    b.append(arrow(510, 372, 540, 350, "", C["acc"], both=True)); b.append(num(500, 360, 3, C["acc"]))
    b.append(arrow(690, 160, 742, 132, "", C["acc"])); b.append(num(716, 142, 4, C["acc"]))
    b.append(arrow(742, 296, 690, 350, "", C["ok"])); b.append(num(716, 330, 5, C["ok"]))
    b.append(f'<line x1="520" y1="44" x2="520" y2="564" stroke="{C["air"]}" stroke-width="3" stroke-dasharray="10 6"/><line x1="710" y1="44" x2="710" y2="564" stroke="{C["air"]}" stroke-width="3" stroke-dasharray="10 6"/>')
    b.append(f'<text x="615" y="470" text-anchor="middle" font-size="12" font-weight="bold" fill="{C["air"]}" {F}>AIR-GAP</text><text x="615" y="486" text-anchor="middle" font-size="10" fill="{C["air"]}" {F}>no direct link between</text><text x="615" y="499" text-anchor="middle" font-size="10" fill="{C["air"]}" {F}>robots and the</text><text x="615" y="512" text-anchor="middle" font-size="10" fill="{C["air"]}" {F}>Command Post</text>')
    leg = "1 the Writer drops a beacon  |  2 RF packets, multi-hop to the van  |  3 robot status and orders (relay)  |  4 uplink UDP 9101  |  5 downlink UDP 9102 (ORDER_UPDATE)"
    b.append(f'<text x="550" y="590" text-anchor="middle" font-size="11" fill="#37474f" {F}>{esc(leg)}</text>')
    save("architecture", svg(1100, 604, "".join(b), [C["line"], C["warn"], C["acc"], C["ok"]]))

# ---------------------------------------------------------------- 2. sequence
def sequence(gap=42, name="sequence"):
    names = ["Writer", "Beacon mesh", "Van (ONA)", "Command Post", "Executor"]; xs = [100, 320, 540, 780, 990]; top = 96; bot = top + 18 * gap + 28
    b = [f'<text x="550" y="26" text-anchor="middle" font-size="17" font-weight="bold" {F}>End-to-end sequence: from detection to a finished rescue</text>']
    for n, x in zip(names, xs):
        b.append(box(x - 62, top - 34, 124, 30, n, [], "#eceff1", fs=12, r=6)); b.append(f'<line x1="{x}" y1="{top}" x2="{x}" y2="{bot}" stroke="#90a4ae" stroke-width="1.5" stroke-dasharray="5 4"/>')
    b.append(f'<rect x="{xs[2]-8}" y="{top}" width="16" height="{bot-top}" fill="#bbdefb" opacity="0.5"/>')
    steps = [(0, 0, "1 GPS to the door; inside no GPS: explore, sense (2-of-3 + persistence)", None),
             (0, 1, "2 drop beacon: 32 B, egocentric hop + target, class P0..P3", C["warn"]),
             (1, 2, "3 RF multi-hop, store-and-forward, ARQ x3 (repeater if link weak)", C["warn"]),
             (2, 2, "4 CRC16 + MAC check, chain hops, ENU -> WGS84", None),
             (2, 3, "5 uplink :9101  (P0 at once; others 120 s / size / score)", C["acc"]),
             (3, 3, "6 every 1 s: update pi, correlate; low-urgency jobs wait while exploring", None),
             (3, 2, "7 briefing + first order (critical pi < crit skips the wait)", C["ok"]),
             (2, 4, "8 briefing handed over at the van, robot enters", C["ok"]),
             (4, 4, "9 A* to the target, LiDAR replanning, hazard check", None),
             (4, 2, "10 status: ARRIVED / FIRE_OUT / RESCUED / heartbeat", C["acc"]),
             (2, 3, "11 forward status (the van is the only relay)", C["acc"]),
             (3, 2, "12 ORDER_UPDATE: hazard cleared / new job", C["ok"]),
             (2, 4, "13 relayed to the robot's own topic /station/to/<role>", C["ok"]),
             (0, 2, "14 Writer: exploration complete (to the van only)", C["acc"]),
             (2, 3, "15 forwarded: held low-urgency jobs are released", C["acc"]),
             (4, 4, "16 enter, rescue, return to the van", None),
             (4, 2, "17 RETURNED -> role free for the next order", C["acc"]),
             (2, 3, "18 live map: RESOLVED", C["acc"])]
    y = top + 28
    for a, bb, text, col in steps:
        if a == bb:
            w = len(text) * 6.0 + 20; x0 = min(max(xs[a] - w / 2, 8), 1092 - w)
            b.append(f'<rect x="{x0}" y="{y-12}" width="{w}" height="22" rx="5" fill="#fffde7" stroke="#bdbdbd"/><text x="{x0+10}" y="{y+3}" font-size="10.5" {F}>{esc(text)}</text>')
        else:
            x1, x2 = xs[a], xs[bb]; b.append(arrow(x1, y, x2, y, "", col or C["line"])); b.append(f'<text x="{(x1+x2)/2}" y="{y-5}" text-anchor="middle" font-size="10.5" fill="{col or C["line"]}" {F}>{esc(text)}</text>')
        y += gap
    b.append(f'<text x="{xs[2]+14}" y="{bot+18}" font-size="10.5" fill="{C["acc"]}" {F}>the van is the only point where inside and outside meet</text>')
    save(name, svg(1100, bot + 44, "".join(b), [C["line"], C["warn"], C["acc"], C["ok"]]))

# ---------------------------------------------------------------- 3. beacon frame
def beacon_frame():
    fields = [("magic", 1, "0xB6"), ("id", 2, "beacon"), ("org", 1, "origin"), ("seq", 2, "version"), ("t0", 4, "written at (s)"), ("T|P|F", 1, "type 4b, prio 2b, flags 2b"), ("floor", 1, ""), ("n", 1, "victims"),
              ("hdg", 1, "heading /256"), ("hopF", 1, "ahead dm"), ("hopR", 1, "right dm"), ("tgtF", 1, "ahead dm"), ("tgtR", 1, "right dm"), ("parent", 2, "chain"), ("temp", 2, "0.1 C"), ("gas", 2, "ppm"),
              ("mirror id", 2, "neighbour"), ("m.seq", 1, ""), ("CRC16", 2, "integrity"), ("MAC", 3, "HMAC-SHA256/24")]
    assert sum(w for _, w, _ in fields) == 32, sum(w for _, w, _ in fields)
    cols = ["#e3f2fd", "#e3f2fd", "#e3f2fd", "#e3f2fd", "#fff3e0", "#fff3e0", "#fff3e0", "#fff3e0", "#e8f5e9", "#e8f5e9", "#e8f5e9", "#e8f5e9", "#e8f5e9", "#f3e5f5", "#fce4ec", "#fce4ec", "#f3e5f5", "#f3e5f5", "#eeeeee", "#eeeeee"]
    b = [f'<text x="500" y="26" text-anchor="middle" font-size="17" font-weight="bold" {F}>Beacon v2: 32 bytes, egocentric (no absolute coordinates inside the building)</text>']
    unit, x, y0 = 30.0, 20.0, 60.0; row_bytes = 16
    pos = 0
    for (name, w, note), col in zip(fields, cols):
        for k in range(w):
            r, c = divmod(pos + k, row_bytes)
        # draw as spans across rows
        left = w; p = pos
        while left > 0:
            r, c = divmod(p, row_bytes); take = min(left, row_bytes - c)
            xx, yy = x + c * unit * 1.9, y0 + r * 80
            b.append(f'<rect x="{xx}" y="{yy}" width="{take*unit*1.9}" height="56" fill="{col}" stroke="#455a64"/>')
            if left == w or take == w: b.append(f'<text x="{xx + take*unit*0.95}" y="{yy+24}" text-anchor="middle" font-size="12" font-weight="bold" {F}>{esc(name)}</text><text x="{xx + take*unit*0.95}" y="{yy+40}" text-anchor="middle" font-size="9.5" fill="#455a64" {F}>{w} B{(" " + esc(note)) if w >= 2 and len(note) <= 14 else ""}</text>')
            p += take; left -= take
        pos += w
    for r in range(2): b.append(f'<text x="{x}" y="{y0 + r*80 - 6}" font-size="10" fill="#607d8b" {F}>bytes {r*row_bytes}-{r*row_bytes + 15}</text>')
    lines = ["T|P|F = packet type (4 bits: REPEATER, FIRE, GAS, HUMAN, ANIMAL, DEBRIS, BEACON_LOST, STAIRS), priority class P0..P3 (2 bits), flags (2 bits:",
             "human_intervention_required, verified).  hdg = heading in 1/256 turns.  hopF/hopR/tgtF/tgtR = metres ahead / to the right in decimetres (+-12.7 m).",
             "temp = 0.1 C units, gas = ppm, n = number of victims, t0 = time the beacon was written (used for aging), mirror = id + seq of a neighbour.",
             "",
             "How positions are rebuilt: each beacon says 'I am hopF m ahead and hopR m to the right of my PARENT (in the parent's heading); what I found",
             "is tgtF / tgtR from ME'. The van chains the hops from the entrance anchor, so Writer drift does not accumulate in the stored frames.",
             "Mirror frames: each beacon also carries a neighbour's id + seq, so a destroyed beacon is detected (missing heartbeat) and its data recovered."]
    for i, l in enumerate(lines): b.append(f'<text x="20" y="{250 + i*20}" font-size="11.5" {F}>{esc(l)}</text>')
    save("beacon_frame", svg(1000, 400, "".join(b), [C["line"]]))

# ---------------------------------------------------------------- 4. state machines
def states():
    b = [f'<text x="550" y="24" text-anchor="middle" font-size="16" font-weight="bold" {F}>State machines of the three actors (names as in the code)</text>']
    def col(x0, title, items, notes=()):
        b.append(f'<text x="{x0 + 160}" y="52" text-anchor="middle" font-size="13" font-weight="bold" {F}>{esc(title)}</text>')
        ys = []
        for i, (name, sub) in enumerate(items):
            y = 70 + i * 62; ys.append(y); b.append(box(x0 + 40, y, 240, 44, name, [sub] if sub else [], C["box"], fs=11.5, r=6))
        for i in range(len(items) - 1): b.append(arrow(x0 + 160, ys[i] + 44, x0 + 160, ys[i + 1], "", C["line"]))
        return ys
    ya = col(0, "Writer", [("At the van (parked)", "start position next to the van"), ("GPS approach (phase gps)", "GNSS fixes, no drift, 0.8 m noise"), ("Exploring (phase lidar)", "LiDAR + odometry, beacons"), ("Exploration complete", "reports EXPLORATION_DONE"), ("Returning, then parked", "A* back to the van")])
    b.append(box(10, 392, 140, 40, "Lost (dead)", ["fault: Writer killed"], "#ffebee", C["air"], fs=11, r=6)); b.append(f'<path d="M 40 412 L 28 412 L 28 216 L 40 216" fill="none" stroke="{C["air"]}" stroke-width="2" stroke-dasharray="6 4" marker-end="url(#a{C["air"][1:]})"/>')
    yb = col(370, "Executor (ambulance, fire truck)", [("DOCKED", "in the van"), ("Briefed", "mission file handed over"), ("ENTER", "leaves the van"), ("GO", "A* to the target; may HOLD"), ("WORK", "rescue / extinguish / seal"), ("RETURN", "back to the van")])
    b.append(f'<path d="M 650 {yb[4] + 22} L 700 {yb[4] + 22} L 700 {yb[3] + 22} L 650 {yb[3] + 22}" fill="none" stroke="{C["acc"]}" stroke-width="2" marker-end="url(#a{C["acc"][1:]})"/><text x="704" y="{(yb[3] + yb[4]) / 2 + 26}" font-size="10" fill="{C["acc"]}" {F}>next target</text>')
    yc = col(740, "Command Post: its view of one robot", [("docked", "no job, or jobs on hold"), ("briefing", "collecting reports / holding"), ("dispatched", "order sent"), ("working", "ARRIVED received"), ("returning", "RETURNED expected")])
    b.append(box(1040 - 0, 70, 1, 1, "", [], "none", "none")) if False else None
    b.append(box(960, 392, 130, 40, "lost", ["silent > 35 s"], "#ffebee", C["air"], fs=11, r=6)); b.append(f'<path d="M 1060 392 L 1060 216 L 1020 216" fill="none" stroke="{C["air"]}" stroke-width="2" stroke-dasharray="6 4" marker-end="url(#a{C["air"][1:]})"/>')
    b.append(box(740 + 280 + 20, 150, 1, 1, "", [], "none", "none")) if False else None
    save("state_machines", svg(1100, 450, "".join(b), [C["line"], C["warn"], C["acc"], C["ok"], C["air"]]))

# ---------------------------------------------------------------- 5. data-flow strip (the "system chain" of the poster, with what is carried)
def data_flow():
    b = [f'<text x="550" y="22" text-anchor="middle" font-size="15" font-weight="bold" {F}>Data flow: from detection to a finished rescue</text>']
    steps = [("Writer", "senses, decides what to record", C["inside"], "#bf360c"), ("Beacon", "32 B, egocentric, CRC + MAC", C["inside"], "#bf360c"), ("Van (ONA)", "verify, chain, GPS, queue", C["ona"], C["acc"]),
             ("Command Post", "rank, group, one order", C["out"], C["ok"]), ("Briefing", "mission file at the van", C["ona"], C["acc"]), ("Executor", "A*, rescue / extinguish", C["inside"], "#bf360c")]
    w, gap, x0, y0 = 150, 30, 12, 50
    for i, (n, sub, fill, st) in enumerate(steps):
        x = x0 + i * (w + gap); b.append(box(x, y0, w, 58, n, [sub], fill, st, fs=13))
        if i < len(steps) - 1: b.append(arrow(x + w, y0 + 29, x + w + gap, y0 + 29, "", C["line"]))
    lab = ["RF, multi-hop, ARQ x3", "uplink UDP :9101", "orders 1 s cycle", "handed over before entry", "status, ORDER_UPDATE via the van"]
    for i, t in enumerate(lab): b.append(f'<text x="{x0 + (i + 1) * (w + gap) - gap / 2}" y="{y0 + 80}" text-anchor="middle" font-size="9.5" fill="#37474f" {F}>{esc(t)}</text>')
    xe = x0 + 5 * (w + gap) + w / 2; xv = x0 + 2 * (w + gap) + w / 2
    b.append(f'<path d="M {xe} {y0 + 58} L {xe} {y0 + 112} L {xv} {y0 + 112} L {xv} {y0 + 58}" fill="none" stroke="{C["acc"]}" stroke-width="2" stroke-dasharray="6 4" marker-end="url(#a{C["acc"][1:]})"/>')
    b.append(f'<text x="{(xe + xv) / 2}" y="{y0 + 128}" text-anchor="middle" font-size="10" fill="{C["acc"]}" {F}>the Executor reports only to the van; the van forwards to the Command Post (no robot-to-robot link)</text>')
    save("data_flow", svg(1100, 205, "".join(b), [C["line"], C["acc"]]))

if __name__ == "__main__":
    architecture(); sequence(); sequence(24, "sequence_compact"); beacon_frame(); states(); data_flow()
