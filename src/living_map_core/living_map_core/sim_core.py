"""Ground truth world (loaded from a map file), sensors, RF model and multi-floor planning. Pure Python, no ROS.
Robots never read the map: they only get sensor data (LiDAR scan, thermal/temperature/gas) from this module."""
import math, random, heapq, json, os

UNK, FREE, WALL, RUBBLE, STAIRS, VOID = -1, 0, 1, 2, 3, 4
CHARS = {".": FREE, "#": WALL, "R": RUBBLE, "S": STAIRS, "A": VOID}
CH = {UNK: "?", FREE: ".", WALL: "#", RUBBLE: "R", STAIRS: "S", VOID: "A"}
NB = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
CAPS_WALK = frozenset({FREE, STAIRS})            # ambulance robot, writer (tracked, climbs stairs)
CAPS_WHEELS = frozenset({FREE})                  # fire truck robot (no stairs)
CAPS_FLY = frozenset({FREE, STAIRS, VOID})       # drone (flies through shafts and stairwells)

def dist(a, b): return math.hypot(a[0] - b[0], a[1] - b[1])

def default_map_path():
    env = os.environ.get("LIVING_MAP_FILE")
    if env: return env
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (os.path.join(here, "..", "maps", "complex_1floor.json"),):
        if os.path.exists(cand): return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("living_map_core"), "maps", "complex_1floor.json")
    except Exception: return os.path.join(here, "..", "maps", "complex_1floor.json")

class World:
    def __init__(self, path=None):
        d = json.load(open(path or default_map_path()))
        self.name, self.cell, (self.x0, self.y0) = d["name"], d["cell"], d["origin"]
        self.van = tuple(d["van"]); self.van_size = tuple(d.get("van_size", (2.6, 1.3, 1.4)))      # length, width, height in metres (a small gateway vehicle)
        self.writer_start = tuple(d.get("writer_start", (-3.0, -1.2)))      # the Writer starts at the van, OUTSIDE the building, and drives to the door by GPS
        self.anchor = d["anchor"]; self.floor_h = d.get("floor_height", 3.5); self.wall_h = d.get("wall_height", 2.6); self.thr = d.get("thresholds", {}); self.wall_db = d.get("wall_db", 12.0); self.fault_t = d.get("fault_times", {})      # radio loss per wall (dB); a foil-lined prototype wall is higher
        self.nf = len(d["floors"]); self.z = [f["z"] for f in d["floors"]]
        self.ny, self.nx = len(d["floors"][0]["rows"]), len(d["floors"][0]["rows"][0])
        self.g = [[[CHARS[c] for c in row] for row in reversed(f["rows"])] for f in d["floors"]]   # g[f][j][i], j=0 is min y
        self.fires = [dict(f, I=1.0) for f in d["fires"]]
        self.gas, self.steam = [dict(g, I=1.0) for g in d["gas"]], d["steam"]
        self.humans = [dict(h, rescued=False) for h in d["humans"]]
        self.animals = [dict(a, rescued=False) for a in d["animals"]]

    # ---- grid helpers ----
    def to_cell(self, x, y): return int(math.floor((x - self.x0) / self.cell)), int(math.floor((y - self.y0) / self.cell))
    def center(self, i, j): return self.x0 + (i + .5) * self.cell, self.y0 + (j + .5) * self.cell
    def inb(self, i, j): return 0 <= i < self.nx and 0 <= j < self.ny
    def val(self, f, x, y):
        i, j = self.to_cell(x, y)
        return self.g[f][j][i] if self.inb(i, j) else WALL
    def los(self, f, a, b, kinds=(WALL,)):
        n = max(1, int(dist(a, b) / min(0.25, self.cell)))
        for k in range(1, n):
            if self.val(f, a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) in kinds: return False
        return True
    def crossings(self, f, a, b):
        n = max(1, int(dist(a, b) / min(0.25, self.cell))); prev = FREE; walls = rub = 0
        for k in range(1, n):
            v = self.val(f, a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            if v == WALL and prev != WALL: walls += 1
            elif v == RUBBLE and prev != RUBBLE: rub += 1
            prev = v
        return walls, rub

    # ---- radio ----
    def rssi(self, a, fa, b, fb):
        """Log-distance path loss (exponent 2.8) + 12 dB per wall, 4 dB per rubble pile, 18 dB per floor slab (assumed values)."""
        walls, rub = self.crossings(fa, a, b)
        d3 = math.hypot(dist(a, b), abs(fa - fb) * self.floor_h)
        return -40.0 - 28.0 * math.log10(max(d3, 1.0)) - self.wall_db * walls - 4.0 * rub - 18.0 * abs(fa - fb)
    @staticmethod
    def per(rssi):
        if rssi >= -85: return 0.0
        if rssi >= -95: return (-85 - rssi) / 10.0 * 0.9
        return 1.0

    # ---- sensors ----
    def sense(self, pos, f, rng):
        temp, gas, hot, tmax = 25.0, 0.0, [], 30.0
        for fr in self.fires:
            if fr["floor"] != f or fr["I"] < 0.02: continue
            d2 = (pos[0] - fr["x"]) ** 2 + (pos[1] - fr["y"]) ** 2
            temp += fr["peak"] * fr["I"] * math.exp(-d2 / (2 * fr["sigma"] ** 2))
            if d2 <= 100 and self.los(f, pos, (fr["x"], fr["y"])) and fr["peak"] * fr["I"] > 40:
                hot.append((fr["x"] + rng.gauss(0, .2), fr["y"] + rng.gauss(0, .2))); tmax = max(tmax, 25 + fr["peak"] * fr["I"])
        for s in self.steam:
            if s["floor"] == f: temp += s["peak"] * math.exp(-((pos[0] - s["x"]) ** 2 + (pos[1] - s["y"]) ** 2) / (2 * s["sigma"] ** 2))
        for gs in self.gas:
            if gs["floor"] == f: gas += gs["ppm"] * gs["I"] * math.exp(-((pos[0] - gs["x"]) ** 2 + (pos[1] - gs["y"]) ** 2) / (2 * gs["sigma"] ** 2))
        blobs = []
        for kind, lst in (("HUMAN", self.humans), ("ANIMAL", self.animals)):
            for o in lst:
                if o["floor"] != f or o["rescued"]: continue
                d = dist(pos, (o["x"], o["y"]))
                if d <= 7 and self.los(f, pos, (o["x"], o["y"])):
                    conf = max(0.0, min(0.99, 0.97 - 0.05 * d + rng.gauss(0, 0.03)))
                    blobs.append(dict(kind=kind, x=o["x"] + rng.gauss(0, .15), y=o["y"] + rng.gauss(0, .15), conf=conf))
        return dict(temp=temp + rng.gauss(0, .5), gas=max(0.0, gas + rng.gauss(0, 2)), thermal_max=tmax, hot=hot, blobs=blobs)

    def scan(self, pos, f, R=6.0):
        out = {}; g = self.g[f]
        for k in range(180):
            a = math.radians(k * 2); step = min(0.25, self.cell); r = step
            while r <= R:
                i, j = self.to_cell(pos[0] + r * math.cos(a), pos[1] + r * math.sin(a))
                if not self.inb(i, j): break
                v = g[j][i]; out[(i, j)] = v
                if v in (WALL, RUBBLE): break
                r += step
        return [(i, j, v) for (i, j), v in out.items()]

    # ---- actuators (what robots do to the world) ----
    def douse(self, f, aim, radius, amount):
        hit = None
        for fr in self.fires:
            if fr["floor"] == f and fr["I"] > 0 and dist(aim, (fr["x"], fr["y"])) <= radius:
                fr["I"] = max(0.0, fr["I"] - amount); hit = fr
                if fr["peak"] * fr["I"] <= 40: fr["I"] = 0.0          # below what any sensor can see: the fire is out
        return hit
    def secure(self, f, pos, radius, amount):
        """A robot closes / seals gas sources within `radius` of itself (reduces the leak). Returns the source it acted on."""
        hit = None
        for g in self.gas:
            if g["floor"] == f and g["I"] > 0 and dist(pos, (g["x"], g["y"])) <= radius:
                g["I"] = max(0.0, g["I"] - amount); hit = g
                if g["ppm"] * g["I"] <= 20: g["I"] = 0.0
        return hit
    def rescue_at(self, f, pos, radius=1.6):
        for lst in (self.humans, self.animals):
            for o in lst:
                if o["floor"] == f and not o["rescued"] and dist(pos, (o["x"], o["y"])) <= radius: o["rescued"] = True; return o
        return None

class KnownMap:
    def __init__(self, w): self.w = w; self.g = [[[UNK] * w.nx for _ in range(w.ny)] for _ in range(w.nf)]
    def update(self, f, cells):
        g = self.g[f]
        for i, j, v in cells: g[j][i] = v

def _prox(k, f, n):
    i, j = n; g = k.g[f]
    for di, dj in NB[:4]:
        a, b = i + di, j + dj
        if 0 <= a < k.w.nx and 0 <= b < k.w.ny and g[b][a] in (WALL, RUBBLE): return 0.5
    return 0.0

def plan(k, start, goal=None, goal_fn=None, caps=CAPS_WALK, opt=None, unk_v=None, extra=None, unk_cost=None):
    """A* (goal given) / Dijkstra (goal_fn) over (floor,i,j). opt = set of (f,i,j) where unknown cells may be crossed (None = never; "all" = anywhere, the
    free-space assumption: unknown is passable until the robot sees otherwise). unk_cost(n) = extra cost of crossing the unknown cell n (so the known-safe trail is preferred).
    unk_v = None | 'all' | set of (i,j): cells where the unknown other side of stairs/shaft may be used."""
    w = k.w; best = {start: 0.0}; prev = {}
    def h(n):                                                    # octile distance in the plane: admissible (every step costs >= 1, diagonal >= 1.414)
        if goal is None: return 0.0
        dx, dy = abs(n[1] - goal[1]), abs(n[2] - goal[2]); return max(dx, dy) + 0.414 * min(dx, dy)
    heap = [(h(start), 0.0, start)]
    while heap:
        _, c, cur = heapq.heappop(heap)
        if c > best[cur]: continue
        if (goal is not None and cur == goal) or (goal_fn and cur != start and goal_fn(cur)):
            path = [cur]
            while cur in prev: cur = prev[cur]; path.append(cur)
            path.pop(); return path[::-1]
        f, i, j = cur; g = k.g[f]
        for di, dj in NB:
            n = (f, i + di, j + dj)
            if not w.inb(n[1], n[2]): continue
            v = g[n[2]][n[1]]
            if v == UNK:
                if opt is None or (not isinstance(opt, str) and n not in opt): continue
            elif v not in caps: continue
            if di and dj and (g[j][i + di] in (WALL, RUBBLE) or g[j + dj][i] in (WALL, RUBBLE)): continue
            nc = c + (1.414 if di and dj else 1.0) + (extra(n) if extra else 0.0) + _prox(k, f, (n[1], n[2])) + (unk_cost(n) if (v == UNK and unk_cost) else 0.0)
            if nc < best.get(n, 1e18): best[n] = nc; prev[n] = cur; heapq.heappush(heap, (nc + h(n), nc, n))
        v0 = g[j][i]; hint = isinstance(unk_v, (set, frozenset)) and (i, j) in unk_v
        if (v0 in (STAIRS, VOID) and v0 in caps) or (v0 == UNK and hint and (isinstance(opt, str) or cur in (opt or ()))):     # vertical moves (stairs / shaft)
            for f2 in (f - 1, f + 1):
                if not 0 <= f2 < w.nf: continue
                v2 = k.g[f2][j][i]
                if v2 == UNK:
                    if not (unk_v == "all" or hint): continue
                elif not (v2 in (STAIRS, VOID) and v2 in caps): continue
                n = (f2, i, j); nc = c + (3.0 if VOID in caps else 8.0)
                if nc < best.get(n, 1e18): best[n] = nc; prev[n] = cur; heapq.heappush(heap, (nc + h(n), nc, n))
    return None

class Robot:
    """Physical body + private map. `est` is the drifting SLAM pose, `pos`/`f` are the truth. Knows nothing about the world file."""
    def __init__(self, world, start, floor, rng, caps=CAPS_WALK, speed=1.0, drift_rate=0.0, scan_r=6.0):
        self.w, self.rng, self.speed, self.drift_rate, self.caps, self.scan_r = world, rng, speed, drift_rate, caps, scan_r
        self.pos, self.f, self.yaw, self.drift = [start[0], start[1]], floor, 0.0, [0.0, 0.0]
        self.k, self.path, self.dead, self.hot, self.travelled = KnownMap(world), [], set(), set(), 0.0
    @property
    def est(self): return (self.pos[0] + self.drift[0], self.pos[1] + self.drift[1])
    def cell(self): i, j = self.w.to_cell(*self.pos); return (self.f, i, j)
    def scan(self):
        self.k.update(self.f, self.w.scan(self.pos, self.f, self.scan_r)); f, i, j = self.cell()
        if self.k.g[f][j][i] == UNK: self.k.g[f][j][i] = self.w.g[f][j][i]
    def extra(self, n): return 40.0 if n in self.hot else 0.0
    def mark_hot(self, temp):
        if temp > 120:
            f, ci, cj = self.cell()
            for di in range(-3, 4):
                for dj in range(-3, 4): self.hot.add((f, ci + di, cj + dj))
    def is_frontier(self, c, radius=60.0):
        f, i, j = c; v = self.k.g[f][j][i]
        if v not in self.caps or c in self.dead: return False
        x, y = self.w.center(i, j)
        if math.hypot(x, y) > radius: return False
        for di, dj in NB[:4]:
            a, b = i + di, j + dj
            if self.w.inb(a, b) and self.k.g[f][b][a] == UNK: return True
        if v in (STAIRS, VOID):                                                # unexplored floor above/below
            for f2 in (f - 1, f + 1):
                if 0 <= f2 < self.w.nf and self.k.g[f2][j][i] == UNK: return True
        return False
    def stair_target(self, c):
        f, i, j = c; v = self.k.g[f][j][i]
        if v in (STAIRS, VOID) and v in self.caps and c not in self.dead:
            for f2 in (f + 1, f - 1):
                if 0 <= f2 < self.w.nf and self.k.g[f2][j][i] == UNK: return f2
        return None
    def choose_frontier(self, radius=60.0):
        st = self.stair_target(self.cell())
        if st is not None: self.path = [(st,) + self.cell()[1:]]; return True          # climb to the unexplored floor
        if self.is_frontier(self.cell(), radius): self.dead.add(self.cell())
        p = plan(self.k, self.cell(), goal_fn=lambda c: self.is_frontier(c, radius), caps=self.caps, unk_v="all", extra=self.extra)
        self.path = p or []
        return p is not None
    def follow(self, budget):
        moved = 0.0
        while self.path and budget > 1e-9:
            nf, ni, nj = self.path[0]; tx, ty = self.w.center(ni, nj)
            if nf != self.f:                                                   # take stairs / fly up the shaft
                here = self.w.g[self.f][self.w.to_cell(*self.pos)[1]][self.w.to_cell(*self.pos)[0]]
                there = self.w.g[nf][nj][ni]
                if here in (STAIRS, VOID) and there in (STAIRS, VOID) and here in self.caps and there in self.caps:
                    self.f = nf; self.path.pop(0); cost = 3.0 if here == STAIRS else 1.5; budget -= cost; moved += cost; continue
                self.path = []; self.dead.add((self.f,) + self.w.to_cell(*self.pos)); break
            if self.w.g[nf][nj][ni] in (WALL, RUBBLE) or self.w.g[nf][nj][ni] not in self.caps: self.path = []; break
            d = math.hypot(tx - self.pos[0], ty - self.pos[1])
            if d > 1e-6: self.yaw = math.atan2(ty - self.pos[1], tx - self.pos[0])
            if d <= budget: self.pos = [tx, ty]; self.path.pop(0); budget -= d; moved += d
            else:
                self.pos[0] += (tx - self.pos[0]) / d * budget; self.pos[1] += (ty - self.pos[1]) / d * budget; moved += budget; budget = 0
        self.travelled += moved
        if self.drift_rate and moved:
            self.drift[0] += self.rng.gauss(0, self.drift_rate * moved); self.drift[1] += self.rng.gauss(0, self.drift_rate * moved)
        return moved
