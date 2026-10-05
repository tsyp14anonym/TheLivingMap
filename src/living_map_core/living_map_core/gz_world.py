"""Generates the Gazebo (Harmonic) world from the SAME map file the simulation uses (multi-floor).
   python3 -m living_map_core.gz_world [map.json] > worlds/living_map_fire.sdf"""
import sys, math
from .sim_core import World, WALL, RUBBLE, STAIRS, VOID, FREE

WORLD = "living_map_fire"

def _shell(w, f):
    """Wall cells that touch open space (the solid fill outside the building is not drawn)."""
    out = set()
    for j in range(w.ny):
        for i in range(w.nx):
            if w.g[f][j][i] != WALL: continue
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    a, b = i + di, j + dj
                    if w.inb(a, b) and w.g[f][b][a] != WALL: out.add((i, j))
    return out

def _runs(w, cells):
    """Merge cells (set of (i,j)) into few boxes: horizontal runs, stacked vertically when identical."""
    rows = []
    for j in range(w.ny):
        i = 0
        while i < w.nx:
            if (i, j) in cells:
                k = i
                while (k + 1, j) in cells: k += 1
                rows.append((i, k, j)); i = k + 1
            else: i += 1
    rs, boxes, used = set(rows), [], set()
    for (a, b, j) in rows:
        if (a, b, j) in used: continue
        j2 = j
        while (a, b, j2 + 1) in rs: j2 += 1; used.add((a, b, j2))
        boxes.append((a, b, j, j2))
    return boxes

def _mat(rgb, a=1.0, emis=None):
    r, g, b = rgb; e = f"<emissive>{emis[0]} {emis[1]} {emis[2]} 1</emissive>" if emis else ""
    return f"<material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse>{e}</material>"

def _static_box(name, x, y, z, sx, sy, sz, rgb, a=1.0):
    return (f'<model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name="l">'
            f'<collision name="c"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>'
            f'<visual name="v"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>{_mat(rgb, a)}</visual></link></model>')

def _static(name, x, y, z, geom, rgb, a=1.0, emis=None, transp=0.0, extra=""):
    t = f"<transparency>{transp}</transparency>" if transp else ""
    return (f'<model name="{name}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose><link name="l">'
            f'<visual name="v"><geometry>{geom}</geometry>{_mat(rgb, a, emis)}{t}</visual>{extra}</link></model>')

def _mover(name, x, y, z, visuals):
    return (f'<model name="{name}"><static>false</static><pose>{x} {y} {z} 0 0 0</pose><link name="l"><gravity>false</gravity>'
            f'<inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>{visuals}</link></model>')

def _vis(n, geom, rgb, pose="0 0 0 0 0 0", emis=None):
    return f'<visual name="{n}"><pose>{pose}</pose><geometry>{geom}</geometry>{_mat(rgb, 1.0, emis)}</visual>'


def build_sdf(map_path=None):
    w = World(map_path); o = []; c = w.cell; H0 = w.z[1] - 0.2 if w.nf > 1 else 3.0
    o.append('<?xml version="1.0" ?>\n<sdf version="1.9">\n<world name="%s">' % WORLD)
    o.append('<physics name="fast" type="ignored"><max_step_size>0.01</max_step_size><real_time_factor>1.0</real_time_factor></physics>')
    for f, n in (("gz-sim-physics-system", "gz::sim::systems::Physics"), ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
                 ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster")):
        o.append(f'<plugin filename="{f}" name="{n}"/>')
    o.append('<scene><ambient>0.45 0.45 0.5 1</ambient><background>0.05 0.07 0.12 1</background><shadows>true</shadows></scene>')
    o.append('<gui fullscreen="0"><plugin filename="MinimalScene" name="3D View"><gz-gui><title>3D View</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>'
             '<engine>ogre2</engine><scene>scene</scene><ambient_light>0.4 0.4 0.4</ambient_light><background_color>0.05 0.07 0.12</background_color><camera_pose>18 -26 28 0 0.85 1.5708</camera_pose></plugin>'
             '<plugin filename="GzSceneManager" name="Scene Manager"><gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>'
             '<plugin filename="InteractiveViewControl" name="Interactive view control"><gz-gui><property key="resizable" type="bool">false</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="state" type="string">floating</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>'
             '<plugin filename="WorldControl" name="World control"><gz-gui><title>World control</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">72</property><property type="double" key="width">121</property><property type="double" key="z">1</property><property type="string" key="state">floating</property><anchors target="3D View"><line own="left" target="left"/><line own="bottom" target="bottom"/></anchors></gz-gui><play_pause>true</play_pause><step>true</step><start_paused>false</start_paused></plugin>'
             '<plugin filename="WorldStats" name="World stats"><gz-gui><title>World stats</title><property type="bool" key="showTitleBar">false</property><property type="bool" key="resizable">false</property><property type="double" key="height">110</property><property type="double" key="width">290</property><property type="double" key="z">1</property><property type="string" key="state">floating</property><anchors target="3D View"><line own="right" target="right"/><line own="bottom" target="bottom"/></anchors></gz-gui><sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor><iterations>true</iterations></plugin></gui>')
    o.append('<light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>')
    o.append('<model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>120 80</size></plane></geometry></collision>'
             '<visual name="v"><geometry><plane><normal>0 0 1</normal><size>120 80</size></plane></geometry>' + _mat((0.16, 0.17, 0.2)) + '</visual></link></model>')
    for f in range(w.nf):
        z0 = w.z[f]
        floorcells = {(i, j) for j in range(w.ny) for i in range(w.nx) if w.g[f][j][i] in (FREE, RUBBLE)}
        if f == 0:                                                        # ground slab (yard + building); stairs/atrium stay open only above
            for n, (a, b, j1, j2) in enumerate(_runs(w, floorcells)):
                sx, sy = (b - a + 1) * c, (j2 - j1 + 1) * c
                o.append(_static_box(f"slab0_{n}", w.x0 + a * c + sx / 2, w.y0 + j1 * c + sy / 2, 0.01, sx, sy, 0.02, (0.28, 0.29, 0.32)))
        else:                                                             # upper slab: open at stairs and atrium shaft
            cells = floorcells | _shell(w, f)
            for n, (a, b, j1, j2) in enumerate(_runs(w, cells)):
                sx, sy = (b - a + 1) * c, (j2 - j1 + 1) * c
                o.append(_static_box(f"slab{f}_{n}", w.x0 + a * c + sx / 2, w.y0 + j1 * c + sy / 2, z0 - 0.1, sx, sy, 0.2, (0.36, 0.37, 0.42), 0.6))
        hgt = (H0 - z0 - 0.1) if f + 1 < w.nf else w.wall_h
        for kind, nm, hh, col in ((WALL, "wall", hgt, (0.62, 0.65, 0.72)), (RUBBLE, "rubble", 0.9, (0.55, 0.38, 0.22))):
            cells = _shell(w, f) if kind == WALL else {(i, j) for j in range(w.ny) for i in range(w.nx) if w.g[f][j][i] == RUBBLE}
            for n, (a, b, j1, j2) in enumerate(_runs(w, cells)):
                sx, sy = (b - a + 1) * c, (j2 - j1 + 1) * c
                o.append(_static_box(f"{nm}{f}_{n}", w.x0 + a * c + sx / 2, w.y0 + j1 * c + sy / 2, z0 + hh / 2, sx, sy, hh, col, 0.75 if kind == WALL else 1.0))
        if f + 1 < w.nf:                                                  # stairs: one sloped ramp per connected stair block
            sc = [(i, j) for j in range(w.ny) for i in range(w.nx) if w.g[f][j][i] == STAIRS]
            if sc:
                ia, ib, ja, jb = min(i for i, j in sc), max(i for i, j in sc), min(j for i, j in sc), max(j for i, j in sc)
                sx, sy = (ib - ia + 1) * c, (jb - ja + 1) * c; cx, cy = w.x0 + ia * c + sx / 2, w.y0 + ja * c + sy / 2; rise = w.z[f + 1] - z0
                if sy >= sx: ang = math.atan2(rise, sy); ln = math.hypot(sy, rise); size = f"{sx:.2f} {ln:.2f} 0.15"; pose = f"{cx:.2f} {cy:.2f} {z0 + rise / 2:.2f} {ang:.4f} 0 0"
                else: ang = math.atan2(rise, sx); ln = math.hypot(sx, rise); size = f"{ln:.2f} {sy:.2f} 0.15"; pose = f"{cx:.2f} {cy:.2f} {z0 + rise / 2:.2f} 0 {-ang:.4f} 0"
                o.append(f'<model name="stairs{f}"><static>true</static><pose>{pose}</pose><link name="l"><visual name="v"><geometry><box><size>{size}</size></box></geometry>{_mat((0.5, 0.75, 0.35))}</visual></link></model>')
            vc = [(i, j) for j in range(w.ny) for i in range(w.nx) if w.g[f][j][i] == VOID]
            if vc:
                ia, ib, ja, jb = min(i for i, j in vc), max(i for i, j in vc), min(j for i, j in vc), max(j for i, j in vc)
                o.append(_static(f"atrium{f}", w.x0 + (ia + ib + 1) * c / 2, w.y0 + (ja + jb + 1) * c / 2, z0 + 1.75, f"<box><size>{(ib - ia + 1) * c:.2f} {(jb - ja + 1) * c:.2f} {w.z[f + 1] - z0:.2f}</size></box>", (0.4, 0.6, 0.9), 0.25, None, 0.75))
    for i, fr in enumerate(w.fires):
        z = w.z[fr["floor"]]
        light = f'<light type="point" name="fl{i}"><pose>0 0 1.2 0 0 0</pose><diffuse>1 0.45 0.1 1</diffuse><specular>1 0.4 0.1 1</specular><attenuation><range>9</range><constant>0.3</constant><linear>0.15</linear><quadratic>0.03</quadratic></attenuation><cast_shadows>false</cast_shadows></light>'
        o.append(f'<model name="fire_{i}"><static>false</static><pose>{fr["x"]} {fr["y"]} {z} 0 0 0</pose><link name="l"><gravity>false</gravity><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial>'
                 + _vis("a", "<cylinder><radius>0.9</radius><length>0.5</length></cylinder>", (1, .3, .05), "0 0 0.25 0 0 0", (1, .35, .05))
                 + _vis("b", "<cylinder><radius>0.6</radius><length>0.9</length></cylinder>", (1, .55, .1), "0 0 0.7 0 0 0", (1, .6, .1))
                 + _vis("c", "<cylinder><radius>0.3</radius><length>1.0</length></cylinder>", (1, .85, .3), "0 0 1.2 0 0 0", (1, .9, .4)) + light + '</link></model>')
    for i, g_ in enumerate(w.gas):
        z = w.z[g_["floor"]]
        o.append(_static(f"gas_tank_{i}", g_["x"], g_["y"], z + 0.5, "<cylinder><radius>0.35</radius><length>1.0</length></cylinder>", (.85, .85, .2)))
        o.append(_static(f"gas_cloud_{i}", g_["x"], g_["y"], z + 1.0, "<sphere><radius>2.3</radius></sphere>", (.6, 1, .2), 0.35, (.2, .4, .05), 0.65))
    for i, s_ in enumerate(w.steam):
        z = w.z[s_["floor"]]
        o.append(_static(f"steam_pipe_{i}", s_["x"], s_["y"], z + 0.4, "<cylinder><radius>0.15</radius><length>0.8</length></cylinder>", (.7, .7, .75)))
        o.append(_static(f"steam_cloud_{i}", s_["x"], s_["y"], z + 1.1, "<sphere><radius>1.0</radius></sphere>", (.95, .95, 1), 0.4, None, 0.7))
    for i, h in enumerate(w.humans):
        o.append(_mover(f"person_{i}", h["x"], h["y"], w.z[h["floor"]], _vis("body", "<cylinder><radius>0.22</radius><length>1.4</length></cylinder>", (1, .25, .25), "0 0 0.7 0 0 0")
                 + _vis("head", "<sphere><radius>0.18</radius></sphere>", (1, .8, .7), "0 0 1.6 0 0 0") + _vis("ring", "<cylinder><radius>0.8</radius><length>0.03</length></cylinder>", (1, .1, .1), "0 0 0.02 0 0 0", (.6, 0, 0))))
    for i, a in enumerate(w.animals): o.append(_mover(f"animal_{i}", a["x"], a["y"], w.z[a["floor"]] + 0.2, _vis("b", "<box><size>0.6 0.3 0.4</size></box>", (1, .88, .4))))
    vx, vy = w.van
    vl, vw, vh = w.van_size; k = vh / 2.4                                     # van size comes from the map (small gateway vehicle by default)
    o.append(_static_box("van_body", vx, vy, vh / 2, vl, vw, vh, (0.2, 0.45, 0.85)))
    o.append(_static("van_antenna", vx, vy, 3.1 * k, f"<cylinder><radius>{0.1 * k:.3f}</radius><length>{1.6 * k:.3f}</length></cylinder>", (.3, .6, 1), 1, (.1, .3, .7)))
    o.append(_static("van_dish", vx, vy, 4.0 * k, f"<sphere><radius>{0.3 * k:.3f}</radius></sphere>", (.3, .6, 1), 1, (.2, .5, 1)))
    o.append(_static("entrance_anchor", 0.0, 0.0, 0.03, "<cylinder><radius>0.5</radius><length>0.06</length></cylinder>", (.2, .5, 1), 1, (.1, .3, .8)))
    o.append(_mover("writer", -1.0, 0.0, 0.25, _vis("body", "<box><size>0.6 0.5 0.3</size></box>", (.25, .8, 1)) + _vis("lidar", "<cylinder><radius>0.1</radius><length>0.15</length></cylinder>", (.1, .1, .1), "0 0 0.22 0 0 0") + _vis("nose", "<box><size>0.2 0.2 0.1</size></box>", (1, 1, 1), "0.3 0 0.05 0 0 0")))
    o.append(_mover("ambulance", -4.0, -2.6, 0.25, _vis("body", "<box><size>0.9 0.55 0.4</size></box>", (.9, 1, .9)) + _vis("cross1", "<box><size>0.3 0.08 0.02</size></box>", (1, .1, .1), "0 0 0.21 0 0 0", (.8, 0, 0)) + _vis("cross2", "<box><size>0.08 0.3 0.02</size></box>", (1, .1, .1), "0 0 0.21 0 0 0", (.8, 0, 0)) + _vis("nose", "<box><size>0.2 0.2 0.1</size></box>", (1, 1, 1), "0.45 0 0.05 0 0 0")))
    o.append(_mover("firetruck", -4.0, 2.6, 0.3, _vis("body", "<box><size>1.1 0.6 0.45</size></box>", (1, .4, .1)) + _vis("tank", "<cylinder><radius>0.15</radius><length>0.7</length></cylinder>", (.8, .2, .05), "-0.1 0 0.35 0 1.5708 0") + _vis("light", "<sphere><radius>0.1</radius></sphere>", (1, .1, .1), "0.4 0 0.3 0 0 0", (1, 0, 0))))
    rot = "".join(_vis(f"rotor{k}", "<cylinder><radius>0.22</radius><length>0.02</length></cylinder>", (.85, .6, 1), f"{sx} {sy} 0.08 0 0 0", (.4, .2, .6)) for k, (sx, sy) in enumerate(((0.3, 0.3), (-0.3, 0.3), (0.3, -0.3), (-0.3, -0.3))))
    o.append(_mover("drone", -2.2, 2.6, 1.8, _vis("body", "<box><size>0.35 0.35 0.12</size></box>", (.83, .55, 1)) + rot + _vis("nose", "<box><size>0.12 0.1 0.06</size></box>", (1, 1, 1), "0.2 0 0 0 0 0")))
    cols = {0: (1, .15, .15), 1: (1, .65, .2), 2: (1, .88, .3), 3: (.6, .66, .78)}; pool = {0: 10, 1: 14, 2: 10, 3: 30}
    for pr, col in cols.items():
        for k in range(pool[pr]):
            o.append(_mover(f"bcn_p{pr}_{k}", 0, 0, -20, _vis("b", "<cylinder><radius>0.14</radius><length>0.36</length></cylinder>", col, "0 0 0 0 0 0", col) + _vis("t", "<sphere><radius>0.11</radius></sphere>", (1, 1, 1), "0 0 0.24 0 0 0", (1, 1, 1))))
    o.append("</world>\n</sdf>")
    return "\n".join(o)

if __name__ == "__main__": sys.stdout.write(build_sdf(sys.argv[1] if len(sys.argv) > 1 else None))
