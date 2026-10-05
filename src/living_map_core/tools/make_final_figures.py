"""Figures of the one-floor project, all drawn from docs/results_final.json and the code constants.
Writes docs/diagrams/fig_priority.png, fig_results.png, fig_trace.png.   Usage: python3 tools/make_final_figures.py"""
import json, os, sys, statistics as st
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
H = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(H, ".."))
from living_map_core.beacon_protocol import priority, confidence
DOCS = os.path.join(H, "..", "docs"); OUT = os.path.join(DOCS, "diagrams"); os.makedirs(OUT, exist_ok=True)
R = json.load(open(os.path.join(DOCS, "results_final.json"))); P = R.get("meta", {}).get("primary", "building"); MAP = json.load(open(os.path.join(H, "..", "maps", "complex_1floor.json" if P == "building" else "proto_arena.json")))
plt.rcParams.update({"font.family": "DejaVu Serif", "font.size": 7.5, "axes.titlesize": 8, "axes.labelsize": 7.5, "legend.fontsize": 6.5, "axes.spines.top": False, "axes.spines.right": False})
mean = lambda v: st.mean(v) if v else float("nan")
sd = lambda v: st.pstdev(v) if len(v) > 1 else 0.0

# ------------------------------------------------------------------ priority curves (math only) with both thresholds
hold = max(priority(t, 1, 1, 0.0) for t in ("HUMAN", "FIRE", "GAS"))
fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.35)); t = [i * 5 for i in range(181)]
for name, typ, cls, col in (("person in danger (P0)", "HUMAN", 0, "#c62828"), ("fire (P1)", "FIRE", 1, "#ef6c00"), ("gas (P1)", "GAS", 1, "#6a1b9a"), ("animal (P2)", "ANIMAL", 2, "#1565c0")):
    ax[0].plot(t, [priority(typ, cls, 1, x) for x in t], label=name, color=col, lw=1.4)
ax[0].axhline(0.3, color="k", ls="--", lw=0.8, label="critical 0.3: skips the wait")
ax[0].axhline(hold, color="gray", ls=":", lw=1.2, label=f"low-urgency {hold:g}: held while exploring")
ax[0].set_yscale("log"); ax[0].set_ylim(0.05, 40); ax[0].set_xlabel("time since detection (s)"); ax[0].set_ylabel("priority number pi")
ax[0].set_title("(a) pi(t) = pi0 exp(-0.004 t)", loc="left"); ax[0].legend(frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.24), ncol=2, fontsize=6)
tm = [i * 0.5 for i in range(61)]
for typ, col, lab in (("FIRE", "#ef6c00", "fire / gas (0.10 per min)"), ("HUMAN", "#c62828", "person / animal (0.01 per min)")): ax[1].plot(tm, [confidence(typ, x * 60) for x in tm], color=col, lw=1.4, label=lab)
ax[1].set_xlabel("age of the observation (min)"); ax[1].set_ylabel("confidence in stored data"); ax[1].set_ylim(0, 1.05); ax[1].set_title("(b) the other clock: observations get stale", loc="left"); ax[1].legend(frameon=False, loc="lower left")
fig.tight_layout(pad=0.6); fig.savefig(os.path.join(OUT, "fig_priority.png"), dpi=300, bbox_inches="tight"); plt.close(fig)

# ------------------------------------------------------------------ results: timing, uplink delay, RF stress
base = R[P + "_baseline"]; ex = [r["exec_first_person_s"] for r in base if r["exec_first_person_s"] is not None]; e2e = [r["e2e_first_person_s"] for r in base if r["e2e_first_person_s"] is not None]
fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.4)); vals = [R[P + "_blind"]["time"], mean(ex), mean(e2e)]; errs = [0, sd(ex), sd(e2e)]
b = ax[0].bar(["blind\nrobot", "Executor\nbriefed", "end to\nend"], vals, yerr=errs, color=["#b0bec5", "#2e7d32", "#ef6c00"], width=0.6, capsize=2)
for r_, v, e_ in zip(b, vals, errs): ax[0].text(r_.get_x() + r_.get_width() / 2, v + e_ + max(vals) * 0.03, f"{v:.1f} s", ha="center", fontsize=7)
ax[0].set_ylabel("time to reach the first person (s)"); ax[0].set_title(f"(a) {P}, {len(base)} seeds", loc="left"); ax[0].set_ylim(0, (max(vals) + max(errs)) * 1.25)
def delay(mode, cls):
    xs = [(r["delay_by_class"][cls]["mean"], r["delay_by_class"][cls]["n"]) for r in R[P + "_uplink"][mode] if cls in r["delay_by_class"]]
    return sum(m * n for m, n in xs) / max(sum(n for _, n in xs), 1)
ps = ["P0", "P1", "P2", "P3"]; mi = {m: sum(r["uplink_msgs"] for r in R[P + "_uplink"][m]) / len(R[P + "_uplink"][m]) for m in ("immediate", "batched")}
ax[1].bar([i - 0.2 for i in range(4)], [delay("immediate", p) for p in ps], 0.4, label=f'immediate ({mi["immediate"]:.1f} msgs/run)', color="#1565c0")
ax[1].bar([i + 0.2 for i in range(4)], [delay("batched", p) for p in ps], 0.4, label=f'batched ({mi["batched"]:.1f} msgs/run)', color="#ef6c00")
ax[1].set_xticks(range(4)); ax[1].set_xticklabels(ps); ax[1].set_ylabel("mean delay (s)"); ax[1].set_title("(b) uplink delay by class", loc="left"); ax[1].legend(frameon=False, loc="upper left")
if P + "_rf" in R:
    ds = sorted(int(k) for k in R[P + "_rf"]); fo = [mean([ (r["fires_out"] or 0) / max(r["fires"], 1) for r in R[P + "_rf"][str(d)]]) for d in ds]; rs = [mean([ (r["rescued"] or 0) / max(r["people"] - 1, 1) for r in R[P + "_rf"][str(d)]]) for d in ds]; rp = [mean([r["repeaters"] for r in R[P + "_rf"][str(d)]]) for d in ds]
    ax[2].plot(ds, fo, "o-", color="#c62828", label="fires out (fraction)"); ax[2].plot(ds, [min(x, 1.0) for x in rs], "^-", color="#2e7d32", label="reachable victims rescued (fraction)")
    ax[2].set_ylim(-0.05, 1.15); ax[2].set_xlabel("radio loss per wall (dB)"); ax[2].set_ylabel("fraction done"); a2 = ax[2].twinx(); a2.plot(ds, rp, "s--", color="#455a64", label="repeaters"); a2.set_ylabel("repeater beacons"); a2.spines["right"].set_visible(True)
    h1, l1 = ax[2].get_legend_handles_labels(); h2, l2 = a2.get_legend_handles_labels(); ax[2].legend(h1 + h2, l1 + l2, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.27), ncol=1, fontsize=6)
ax[2].set_title("(c) RF attenuation stress", loc="left"); fig.tight_layout(pad=0.6); fig.savefig(os.path.join(OUT, "fig_results.png"), dpi=300, bbox_inches="tight"); plt.close(fig)

# ------------------------------------------------------------------ one run on the arena: paths + timeline
tr = R[P + "_trace"]; x0, y0 = MAP["origin"]; c = MAP["cell"]; rows = MAP["floors"][0]["rows"][::-1]; ny, nx = len(rows), len(rows[0])
img = np.zeros((ny, nx, 3)); col = {".": (1, 1, 1), "#": (0.2, 0.2, 0.2), "R": (0.65, 0.45, 0.2)}
for j, row in enumerate(rows):
    for i, ch in enumerate(row): img[j, i] = col.get(ch, (1, 1, 1))
fig = plt.figure(figsize=(7.6, 3.3)); gs = fig.add_gridspec(1, 2, width_ratios=[1.7, 1.0], wspace=0.35); a = fig.add_subplot(gs[0]); g = fig.add_subplot(gs[1])
a.imshow(img, origin="lower", extent=[x0, x0 + nx * c, y0, y0 + ny * c], interpolation="nearest"); a.set_xlim(x0, x0 + nx * c); a.set_ylim(y0, y0 + ny * c); a.set_aspect("equal")
for k, lst, mk, cl, lab in (("fires", MAP["fires"], "*", "#e53935", "fire"), ("gas", MAP["gas"], "o", "#7cb342", "gas"), ("steam", MAP["steam"], "s", "#4fc3f7", "steam"), ("humans", MAP["humans"], "P", "#ff9800", "person"), ("animals", MAP["animals"], "d", "#ffd54f", "animal")):
    a.scatter([o["x"] for o in lst], [o["y"] for o in lst], marker=mk, s=30, color=cl, edgecolor="k", linewidth=0.5, label=lab, zorder=5)
w = tr["trace"]["writer"]; xs = [p[1] for p in w]; ys = [p[2] for p in w]; ph = [p[3] for p in w]
gp = [i for i, p in enumerate(ph) if p == "gps"]; li = [i for i, p in enumerate(ph) if p != "gps"]
if gp: a.plot([xs[i] for i in gp] + [xs[li[0]]], [ys[i] for i in gp] + [ys[li[0]]], color="#00897b", lw=2.2, label="Writer, GPS approach", zorder=4)
a.plot([xs[i] for i in li], [ys[i] for i in li], color="#5c6bc0", lw=1.0, label="Writer, no GPS", zorder=3)
for role, cl in (("ambulance", "#1e88e5"), ("firetruck", "#e53935")):
    seg = [p for p in tr["trace"][role] if p is not None]
    if seg: a.plot([p[1] for p in seg], [p[2] for p in seg], color=cl, lw=1.0, ls="--", label=role, zorder=3)
for bid, t_, typ, pos in [(d[0], d[1], d[2], d[3]) for d in tr["drops"]]:
    if typ == "REPEATER": a.scatter([pos[0]], [pos[1]], marker="^", s=9, color="#9e9e9e", zorder=5)
    else: a.scatter([pos[0]], [pos[1]], marker="^", s=30, color="k", zorder=6); a.annotate(f"{typ[:3]} {bid}", (pos[0], pos[1]), xytext=(2, 3), textcoords="offset points", fontsize=5)
a.scatter([MAP["van"][0]], [MAP["van"][1]], marker="s", s=60, color="#1565c0", zorder=6, label="van"); a.set_title("(a) paths (black triangles = event beacons, gray = repeaters)", loc="left")
a.legend(frameon=False, fontsize=5.5, ncol=4, loc="upper center", bbox_to_anchor=(0.5, -0.1)); a.tick_params(labelsize=6)
def spans(seq, pred):
    out, s0 = [], None
    for i, p in enumerate(seq):
        on = p is not None and pred(p)
        if on and s0 is None: s0 = i
        if (not on) and s0 is not None: out.append((seq[s0][0], seq[i - 1][0])); s0 = None
    if s0 is not None: out.append((seq[s0][0], seq[-1][0]))
    return out
rowsy = {"Writer": 3, "Command Post": 2, "Ambulance": 1, "Fire truck": 0}
for s_, e_ in spans(w, lambda p: p[3] == "gps"): g.barh(3, e_ - s_ + 0.5, left=s_, color="#00897b", height=0.5)
for s_, e_ in spans(w, lambda p: p[3] != "gps"): g.barh(3, e_ - s_ + 0.5, left=s_, color="#5c6bc0", height=0.5)
for t_, roles in tr["orders_log"]: g.plot([t_, t_], [1.6, 2.4], color="#2e7d32", lw=1.6)
for role, y_, cl in (("ambulance", 1, "#1e88e5"), ("firetruck", 0, "#e53935")):
    for s_, e_ in spans(tr["trace"][role], lambda p: True): g.barh(y_, e_ - s_ + 0.5, left=s_, color=cl, height=0.5)
for d in tr["drops"]: g.plot([d[1], d[1]], [2.62, 3.38], color=("#9e9e9e" if d[2] == "REPEATER" else "k"), lw=0.8)
for key, lab, ha_, dx_ in (("exploration_done_at_cp_s", "exploration\ncomplete", "right", -0.8), ("arrive_human_s", "first person\nreached", "left", 0.8)):
    if tr.get(key): g.axvline(tr[key], color="gray", ls=":", lw=0.9); g.text(tr[key] + dx_, -0.62, lab, fontsize=5.5, color="gray", ha=ha_)
g.set_yticks(list(rowsy.values())); g.set_yticklabels(list(rowsy.keys())); g.set_xlabel("simulated time (s)"); g.set_ylim(-0.8, 3.7); g.set_title("(b) timeline: event beacons (black), orders (green)", loc="left"); g.tick_params(labelsize=6)
fig.savefig(os.path.join(OUT, "fig_trace.png"), dpi=300, bbox_inches="tight"); plt.close(fig); print("figures written")
