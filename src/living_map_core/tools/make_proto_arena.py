"""PROTOTYPE ARENA (7.2 m x 4.8 m) for the physical Phase-2 test of The Living Map, defined ONCE and used for everything:
  maps/proto_arena.json  -> the same map format the simulator loads (also builds the Gazebo world: worlds/proto_arena.sdf)
  docs/proto_arena_plan.svg -> printable top view with dimensions
  docs/proto_arena_materials.md -> wall length, plywood sheets, cut list
Design rules (from the RoboCup Rescue / NIST arenas): 1.2 m hallways, doorways wider than the robot + 10 cm, walls cut from standard 1.22 x 2.44 m plywood
(two 0.61 m strips per sheet, no waste), everything modular and re-arrangeable.  Coordinates: metres, entrance anchor (0,0) on the west wall, x east, y north.
Usage: python3 tools/make_proto_arena.py [outdir]"""
import json, math, os, sys

CELL, X0, Y0, NX, NY = 0.1, -6.0, -3.2, 140, 64        # 14.0 m x 6.4 m window: yard (van + parked robots) + the arena
T, H = 0.10, 0.61                                       # wall thickness in the grid (the real plywood is 12 mm), wall height (half a 1.22 m sheet)
W, L = 4.8, 7.2                                         # arena: 6 x 4 modules of 1.2 m
DOOR_ENTRANCE, DOOR_ROOM = 0.9, 0.9                     # clear opening (robot width + 10 cm for a robot up to ~0.4 m wide, as RoboCup Rescue sets its doorways)

# ---- walls as centre lines (x0,y0,x1,y1) and doors (x0,y0,x1,y1) that are carved out again ----
walls = [(0, -2.4, 0, 2.4), (L, -2.4, L, 2.4), (0, 2.4, L, 2.4), (0, -2.4, L, -2.4),                   # outer
         (0, 0.6, L, 0.6), (0, -0.6, L, -0.6),                                                         # corridor walls (corridor 1.2 m wide)
         (3.6, 0.6, 3.6, 2.4), (3.6, -2.4, 3.6, -0.6)]                                                 # room dividers: 4 rooms of 3.6 x 1.8 m
doors = [(0, -DOOR_ENTRANCE / 2, 0, DOOR_ENTRANCE / 2),
         (1.8, 0.6, 1.8 + DOOR_ROOM, 0.6), (5.0, 0.6, 5.0 + DOOR_ROOM, 0.6), (1.8, -0.6, 1.8 + DOOR_ROOM, -0.6), (5.0, -0.6, 5.0 + DOOR_ROOM, -0.6)]
boxes = [(2.0, 1.95, 2.4, 2.3), (4.2, -2.0, 4.6, -1.6), (6.5, 1.0, 6.9, 1.4), (2.9, -2.0, 3.3, -1.6)]    # obstacles / furniture (0.4 m boxes)
rubble = (5.0, -0.65, 5.9, -0.55)                                                                        # collapsed debris sealing the south-east door (impassable: priority 3)

def build():
    g = [["#"] * NX for _ in range(NY)]
    def fill(x0, y0, x1, y1, ch):
        for j in range(NY):
            for i in range(NX):
                cx, cy = X0 + (i + .5) * CELL, Y0 + (j + .5) * CELL
                if x0 <= cx < x1 and y0 <= cy < y1: g[j][i] = ch
    fill(-5.5, -3.0, 0, 3.0, ".")                                                      # fenced yard outside the entrance (Outside Network Area / van)
    fill(0, -2.4, L, 2.4, ".")                                                         # arena interior
    for (a, b, c, d) in walls:
        if a == c: fill(a - T / 2, min(b, d) - T / 2, a + T / 2, max(b, d) + T / 2, "#")
        else: fill(min(a, c) - T / 2, b - T / 2, max(a, c) + T / 2, b + T / 2, "#")
    for (a, b, c, d) in doors:
        if a == c: fill(a - T / 2, min(b, d), a + T / 2, max(b, d), ".")
        else: fill(min(a, c), b - T / 2, max(a, c), b + T / 2, ".")
    for (a, b, c, d) in boxes: fill(a, b, c, d, "#")
    fill(*rubble, "R")
    rows = ["".join(r) for r in reversed(g)]
    objects = dict(
        fires=[dict(id="F1", x=5.6, y=1.7, floor=0, peak=140, sigma=0.35)],                         # heat lamp behind a guard (peak = surface temperature, deg C)
        gas=[dict(id="G1", x=2.4, y=-1.8, floor=0, ppm=900, sigma=0.5)],                           # gas source (see the safety note: use a NON-flammable stand-in)
        steam=[dict(id="ST1", x=1.4, y=0.2, floor=0, peak=90, sigma=0.45)],                         # kettle or steam source in the corridor: the 'false alarm' that the rate-of-rise filter must reject
        humans=[dict(id="H1", x=2.8, y=1.9, floor=0), dict(id="H2", x=6.5, y=1.9, floor=0), dict(id="H3", x=6.4, y=-1.8, floor=0)],   # heated pads; H3 sits behind the rubble (heavy rescue)
        animals=[dict(id="A1", x=0.9, y=1.9, floor=0)])
    return dict(name="Prototype Arena 7.2 x 4.8 m (1 floor)", cell=CELL, origin=[X0, Y0], van=[-1.5, 0.0], van_size=[0.9, 0.6, 0.5], fault_times=dict(beacon_destroyed=20.0, writer_lost=25.0), wall_height=H,
                anchor=dict(lat0=36.8065, lon0=10.1815, yaw_deg=45.0, alt=15.0), floor_height=3.5,
                legend={".": "floor", "#": "wall / furniture", "R": "rubble"}, floors=[dict(z=0.0, rows=rows)],
                # prototype sensor thresholds: the thermal camera is the trigger for the heat lamp (140 C seen from across the room; the air barely warms),
                # a kettle (steam plume) stays below it and is rejected as steam. Defaults of the big building (70 / 150 C) are unchanged.
                thresholds=dict(fire_air=40, fire_thermal=110, fire_thermal_alone=110, gas_vote=50, gas_event=150,
                                # distances scaled to a 7 m arena (defaults 4.5 / 5.5 / 9 / 1.0 / 8 are for the 48 m building)
                                refine_person=1.0, refine_fire=1.5, hold_dist=1.5, standoff_scale=0.25, corr_dist=1.5), **objects)

def segs_len(segs):
    return sum(math.hypot(c - a, d - b) for (a, b, c, d) in segs)

def svg(d):
    s, ox, oy = 90.0, 250.0, 40.0                                  # px per metre, margins
    px = lambda x: ox + (x + 0.0) * s; py = lambda y: oy + (2.4 - y) * s
    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{int(L * s + 420)}" height="{int(W * s + 150)}" font-family="sans-serif" font-size="12">',
         f'<rect width="100%" height="100%" fill="white"/><text x="{ox}" y="22" font-size="15" font-weight="bold">The Living Map - prototype arena {L} x {W} m (wall height {H} m, plywood 12 mm)</text>']
    for i in range(0, 7):                                         # 1.2 m module grid
        o.append(f'<line x1="{px(i * 1.2)}" y1="{py(2.4)}" x2="{px(i * 1.2)}" y2="{py(-2.4)}" stroke="#ddd"/>')
    for j in range(0, 5): o.append(f'<line x1="{px(0)}" y1="{py(-2.4 + j * 1.2)}" x2="{px(L)}" y2="{py(-2.4 + j * 1.2)}" stroke="#ddd"/>')
    for (a, b, c, e) in walls: o.append(f'<line x1="{px(a)}" y1="{py(b)}" x2="{px(c)}" y2="{py(e)}" stroke="#333" stroke-width="7" stroke-linecap="square"/>')
    for (a, b, c, e) in doors: o.append(f'<line x1="{px(a)}" y1="{py(b)}" x2="{px(c)}" y2="{py(e)}" stroke="white" stroke-width="9"/><line x1="{px(a)}" y1="{py(b)}" x2="{px(c)}" y2="{py(e)}" stroke="#4a4" stroke-width="2" stroke-dasharray="4 3"/>')
    for (a, b, c, e) in boxes: o.append(f'<rect x="{px(a)}" y="{py(e)}" width="{(c - a) * s}" height="{(e - b) * s}" fill="#bbb" stroke="#555"/>')
    a, b, c, e = rubble; o.append(f'<rect x="{px(a)}" y="{py(e) - 4}" width="{(c - a) * s}" height="10" fill="#a73" stroke="#531"/><text x="{px(a)}" y="{py(e) + 24}" fill="#531">rubble (impassable)</text>')
    def dot(x, y, col, label, r=9, dy=4): o.append(f'<circle cx="{px(x)}" cy="{py(y)}" r="{r}" fill="{col}" stroke="#222"/><text x="{px(x) + 12}" y="{py(y) + dy}">{label}</text>')
    for f in d["fires"]: dot(f["x"], f["y"], "#e44", f'fire {f["id"]} (heat lamp)', dy=34)
    for gg in d["gas"]: dot(gg["x"], gg["y"], "#9c4", f'gas {gg["id"]}')
    for st in d["steam"]: dot(st["x"], st["y"], "#8bd", f'steam {st["id"]} (kettle)')
    for h in d["humans"]: dot(h["x"], h["y"], "#f90", f'victim {h["id"]} (heated pad)')
    for an in d["animals"]: dot(an["x"], an["y"], "#fc6", f'animal {an["id"]} (cooler pad)', 7)
    o.append(f'<rect x="{px(-1.9)}" y="{py(1.0)}" width="{1.9 * s - 8}" height="{2.0 * s}" fill="#eef" stroke="#44a" stroke-dasharray="5 3"/><text x="{px(-1.85)}" y="{py(0.2)}">Outside Network Area</text><text x="{px(-1.85)}" y="{py(0.2) + 16}">(van: gateway + radio)</text>')
    o.append(f'<text x="{px(-1.85)}" y="{py(-1.4)}">Command Post: laptop in another</text><text x="{px(-1.85)}" y="{py(-1.4) + 16}">room, linked ONLY to the van</text>')
    o.append(f'<text x="{px(0.1)}" y="{py(-0.9)}" fill="#4a4">entrance 0.9 m  (anchor 0,0)</text>')
    for k in range(0, 7): o.append(f'<text x="{px(k * 1.2) - 8}" y="{py(-2.4) + 22}">{k * 1.2:.1f}</text>')
    o.append(f'<text x="{ox}" y="{py(-2.4) + 46}">dimensions in metres - corridor 1.2 m - rooms 3.6 x 1.8 m - doors 0.9 m - grid = one 1.2 m module</text></svg>')
    return "\n".join(o)

def materials():
    outer_len = segs_len(walls)
    gaps = segs_len(doors)
    net = outer_len - gaps
    sheets = math.ceil(net / 2.44 / 2)
    return net, sheets

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(os.path.join(out, "maps"), exist_ok=True); os.makedirs(os.path.join(out, "docs"), exist_ok=True)
    d = build(); json.dump(d, open(os.path.join(out, "maps", "proto_arena.json"), "w"), indent=0)
    open(os.path.join(out, "docs", "proto_arena_plan.svg"), "w").write(svg(d))
    net, sheets = materials()
    open(os.path.join(out, "docs", "proto_arena_materials.md"), "w").write(
        f"# Prototype arena materials\n\n- Wall length to build (after the door openings): **{net:.1f} m**, height {H} m\n- Plywood 12 mm, standard sheet 1.22 x 2.44 m cut lengthwise into two 0.61 x 2.44 m strips: **{sheets} sheets** ({sheets * 2} strips) + 10 % spare = **{math.ceil(sheets * 1.1)} sheets**\n"
        f"- Each strip = one 2.44 m wall (or two 1.22 m modules); strips are joined with 2 right-angle brackets + clamps so the layout can be changed in minutes\n- Floor: leave the bare floor (smooth) or lay 6 mm MDF/hardboard; the yard outside the entrance is just tape on the floor\n"
        f"- Rubble: 6-10 pieces of foam/cardboard blocks, 10-15 cm high, stacked across the south-east door (impassable for the ground robot)\n- Obstacles: 4 cardboard boxes 0.4 x 0.4 m\n")
    print(f"wrote maps/proto_arena.json, docs/proto_arena_plan.svg, docs/proto_arena_materials.md  | walls {net:.1f} m -> {sheets} sheets")
