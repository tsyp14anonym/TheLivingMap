"""Draws the 2-floor 'Industrial Complex' scenario and writes maps/complex.json.
The runtime NEVER contains this layout: it just loads the JSON. Edit this file (or the JSON) to change the building.
Coordinates: metres, entrance anchor (0,0) on the west wall. Grid cell = 0.5 m."""
import json, sys
CELL, X0, Y0, NX, NY = 0.5, -6.0, -12.0, 96, 48

def new_floor(): return [["#"] * NX for _ in range(NY)]           # everything solid, then we carve
def ij(x, y): return int((x - X0) // CELL), int((y - Y0) // CELL)
def fill(g, x0, y0, x1, y1, ch):
    for j in range(NY):
        for i in range(NX):
            cx, cy = X0 + (i + .5) * CELL, Y0 + (j + .5) * CELL
            if x0 <= cx < x1 and y0 <= cy < y1: g[j][i] = ch

def build():
    f0, f1 = new_floor(), new_floor()
    # ---------------- ground floor ----------------
    fill(f0, -5.5, -5, 0, 5, "."); fill(f0, 0, -1, 0.5, 1, ".")                      # fenced yard + entrance door
    fill(f0, 0.5, -1.5, 39.5, 1.5, ".")                                                # main corridor
    north = [(0.5, 10, 4, 6), (10.5, 20, 14, 16), (20.5, 30, 24, 26), (30.5, 39.5, 34, 36)]   # rooms north (x0,x1, door x0,x1)
    south = [(0.5, 12, 5, 7), (12.5, 26, 18, 20), (26.5, 33, 29, 31), (33.5, 39.5, 36, 38)]
    for (a, b, d0, d1) in north: fill(f0, a, 2, b, 9.5, "."); fill(f0, d0, 1.5, d1, 2, ".")
    for (a, b, d0, d1) in south: fill(f0, a, -9.5, b, -2, "."); fill(f0, d0, -2, d1, -1.5, ".")
    # details: pillars, furniture, rubble
    for x in (15, 18, 21, 24): fill(f0, x, -6, x + .5, -5.5, "#"); fill(f0, x, -9, x + .5, -8.5, "#")   # machinery hall pillars
    fill(f0, 12.5, -9.5, 13, -6, "#")                                                                   # partition stub
    fill(f0, 12.5, -6, 26, -5.5, "R")                                                                     # ceiling collapse across the hall (seals the south part)
    fill(f0, 11.5, 3, 13.5, 4.5, "#"); fill(f0, 15, 7, 19, 8, "#")                                      # desks in Office A
    fill(f0, 21.5, 3, 23.5, 4.5, "#"); fill(f0, 27, 3, 29, 4.5, "#"); fill(f0, 22, 8, 29, 8.5, "#")     # cafeteria counter/tables
    fill(f0, 1, 7.5, 9, 8.5, "#"); fill(f0, 1, 3, 2, 6, "#"); fill(f0, 8, 3, 9, 6, "#")                 # server racks
    fill(f0, 1, -8.5, 3, -7.5, "#"); fill(f0, 9, -8.5, 11, -7.5, "#")                                   # storage shelves
    fill(f0, 31, 3, 33, 5, "#"); fill(f0, 37, 3, 39, 4, "#")                                            # workshop benches
    # stairs (S) + atrium shaft (A) in the stair hall
    fill(f0, 28, -8, 30, -5, "S"); fill(f0, 30.5, -8, 32.5, -6, "A")
    # ---------------- first floor ----------------
    fill(f1, 0.5, -1.5, 39.5, 1.5, ".")
    north1 = [(0.5, 13, 5, 7), (13.5, 26, 19, 21), (26.5, 39.5, 32, 34)]
    south1 = [(0.5, 14, 6, 8), (14.5, 27, 20, 22), (27.5, 39.5, 31, 33)]
    for (a, b, d0, d1) in north1: fill(f1, a, 2, b, 9.5, "."); fill(f1, d0, 1.5, d1, 2, ".")
    for (a, b, d0, d1) in south1: fill(f1, a, -9.5, b, -2, "."); fill(f1, d0, -2, d1, -1.5, ".")
    fill(f1, 2, 4, 4, 5, "#"); fill(f1, 8, 4, 10, 5, "#"); fill(f1, 16, 5, 24, 6, "#"); fill(f1, 16, 8, 24, 8.5, "#")       # lab benches, library shelves
    fill(f1, 28, 3, 30, 4, "#"); fill(f1, 36, 3, 38, 4, "#"); fill(f1, 16, -8, 18, -7, "#"); fill(f1, 22, -5, 24, -4, "#")
    fill(f1, 28, -8, 30, -5, "S"); fill(f1, 30.5, -8, 32.5, -6, "A")
    fill(f1, 30.5, -6, 32.5, -5.5, "."); fill(f1, 28, -9.5, 32.5, -8.5, ".")
    rows = lambda g: ["".join(r) for r in reversed(g)]                                                  # top row = max y (easier to read)
    objects = dict(
        fires=[dict(id="F1", x=35.0, y=6.0, floor=0, peak=420, sigma=1.6), dict(id="F2", x=36.0, y=-6.5, floor=0, peak=380, sigma=1.5),
               dict(id="F3", x=6.0, y=6.5, floor=1, peak=420, sigma=1.6), dict(id="F4", x=34.0, y=6.5, floor=1, peak=400, sigma=1.6)],
        gas=[dict(id="G1", x=4.0, y=-6.0, floor=0, ppm=900, sigma=2.6)],
        steam=[dict(id="ST1", x=25.0, y=6.0, floor=0, peak=80, sigma=1.0)],
        humans=[dict(id="H1", x=37.5, y=-7.5, floor=0), dict(id="H2", x=23.0, y=-8.0, floor=0), dict(id="H3", x=35.5, y=8.0, floor=1)],
        animals=[dict(id="A1", x=15.0, y=6.0, floor=0), dict(id="A2", x=19.0, y=-5.5, floor=1)])
    return dict(name="Industrial Complex (2 floors)", cell=CELL, origin=[X0, Y0], van=[-4.0, 0.0],
                anchor=dict(lat0=36.8065, lon0=10.1815, yaw_deg=45.0, alt=15.0), floor_height=3.5,
                legend={".": "floor", "#": "wall / furniture", "R": "rubble", "S": "stairs", "A": "atrium shaft (drone only)"},
                floors=[dict(z=0.0, rows=rows(f0)), dict(z=3.5, rows=rows(f1))], **objects)

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "maps/complex.json"
    json.dump(build(), open(out, "w"), indent=0); print("wrote", out)
