"""Miniskybot's meshes, made at build time from the STL files its repository ships (nothing of theirs in git) and from
its OpenSCAD model's dimensions, merged per body and colour, written as OBJ files with their normals (smooth across
the faces of its rounded edges, its wheels' rims and its caster's tower, sharp at its edges: a crease angle).

    build()             its meshes in MESHES, made again when this file or config.py is newer

Its printed parts as show_robot() (parts/Openscad/main.scad) puts them together: its chassis (exported upside down,
ready to print: turned back) and battery holder, its two Futaba S3003; its wheels about their own axle. What its STL
files leave out, from its OpenSCAD model's dimensions and its photo: its wheels' O-rings in the V of their rims
(Servo-wheel.scad's raw_wheel()), its black 4-arm horns in their wheels' recesses (horn4()), their screws; its
Skymega's metal hex spacers, under it and over it; its servos' cables (black, red, white, Futaba's) and its battery's wires (red and black)
up to its board, their plugs' places from its board's pads (skymega.py: its Skymega, from Obijuan's KiCad and
Gerber files, made at build time with its meshes).

Frames: the body's meshes in the robot's (mm: x right, y forward, z up, origin on the floor under the wheels' axle);
the wheels' in their own (their axle's middle, their axis x, their outer face towards +x for the right wheel, -x for
the left).
"""
import json
import os
import struct

import numpy as np

from . import config as S
from . import skymega

MESHES = os.path.join(S.BUILD, 'meshes')
CREASE_DEG = 35                 # faces meeting at less than this are shaded as one smooth surface

# ---- reading and writing -------------------------------------------------------------------------------------------


def read_stl(path):
    """An STL file's triangles (n, 3, 3): OpenSCAD's ASCII, or binary; its degenerate triangles dropped."""
    with open(path, 'rb') as f:
        data = f.read()
    if not data.lstrip().startswith(b'solid') or b'facet' not in data[:1000]:
        n = struct.unpack('<I', data[80:84])[0]
        rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
        t = rec['v'].reshape(-1, 3, 3).astype(np.float64)
    else:
        v = [ln.split()[1:4] for ln in data.decode(errors='replace').splitlines() if ln.strip().startswith('vertex')]
        t = np.array(v, np.float64).reshape(-1, 3, 3)
    return t[np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1) > 1e-6]


def write_obj(path, tris, crease=CREASE_DEG):
    """Triangles (n, 3, 3) as an OBJ file, each corner's normal the mean of its vertex's faces within the crease
    angle of its own face (weighted by their areas); its degenerate triangles dropped."""
    tris = np.asarray(tris, np.float64)
    tris = tris[np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1) > 1e-6]
    v, idx = np.unique(tris.reshape(-1, 3).round(4), axis=0, return_inverse=True)
    idx = idx.reshape(-1, 3)
    fn = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])      # its length twice the area
    unit = fn / np.linalg.norm(fn, axis=1, keepdims=True)
    cos = np.cos(np.radians(crease))
    corners = idx.ravel()
    face = np.repeat(np.arange(len(idx)), 3)
    order = np.argsort(corners, kind='stable')
    starts = np.searchsorted(corners[order], np.arange(len(v) + 1))
    normals = np.empty((len(corners), 3))
    for k in range(len(v)):
        cs = order[starts[k]:starts[k + 1]]
        fs = face[cs]
        near = unit[fs] @ unit[fs].T >= cos                             # (its corners, its faces)
        n = near @ fn[fs]
        normals[cs] = n / np.linalg.norm(n, axis=1, keepdims=True)
    lines = [f'v {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in v]
    lines += [f'vn {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in normals]
    lines += [f'f {a + 1}//{3 * i + 1} {b + 1}//{3 * i + 2} {c + 1}//{3 * i + 3}\n' for i, (a, b, c) in enumerate(idx)]
    with open(path + '.part', 'w') as f:
        f.writelines(lines)
    os.replace(path + '.part', path)


# ---- frames ------------------------------------------------------------------------------------------------------


def _rx(deg):
    c, s = (round(v) for v in (np.cos(np.radians(deg)), np.sin(np.radians(deg))))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def _to_robot(p):
    """Points of main.scad's frame in the robot's."""
    x, y, z = np.moveaxis(p, -1, 0)
    return np.stack(S.scad_to_robot(x, y, z), axis=-1)


def _wheel_to_own(p, deg):
    """Points of Servo-wheel.scad's frame (its axis z, its horn's recess towards +z) in the wheel's own: show_robot()'s
    rotate([deg, 0, 0]), then the robot's axes."""
    w = p @ _rx(deg).T
    return np.stack([-w[..., 1], w[..., 0], w[..., 2]], axis=-1)


# ---- shapes ------------------------------------------------------------------------------------------------------


def _hull2(pts):
    """The convex hull of 2D points, counterclockwise."""
    pts = sorted(set(map(tuple, np.round(pts, 6))))
    def half(ps):
        out = []
        for p in ps:
            while len(out) >= 2 and np.cross(np.subtract(out[-1], out[-2]), np.subtract(p, out[-2])) <= 0:
                out.pop()
            out.append(p)
        return out
    lo, hi = half(pts), half(pts[::-1])
    return np.array(lo[:-1] + hi[:-1])


def prism(poly, z0, z1):
    """A convex polygon (counterclockwise, xy) extruded from z0 to z1: its triangles, outward."""
    n = len(poly)
    b = np.column_stack([poly, np.full(n, z0)])
    t = np.column_stack([poly, np.full(n, z1)])
    tris = [(t[0], t[i], t[i + 1]) for i in range(1, n - 1)] + [(b[0], b[i + 1], b[i]) for i in range(1, n - 1)]
    for i in range(n):
        j = (i + 1) % n
        tris += [(b[i], b[j], t[j]), (b[i], t[j], t[i])]
    return np.array(tris)


def _circle(r, n, c=(0, 0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)])


def cylinder(r, z0, z1, n=24, c=(0, 0)):
    return prism(_circle(r, n, c), z0, z1)


def torus(big, small, n=96, m=16):
    """A torus about z, in the xy plane."""
    u = np.linspace(0, 2 * np.pi, n + 1)[:, None]
    w = np.linspace(0, 2 * np.pi, m + 1)[None, :]
    p = np.stack([(big + small * np.cos(w)) * np.cos(u), (big + small * np.cos(w)) * np.sin(u),
                  small * np.sin(w) + 0 * u], axis=-1)
    a, b, c, d = p[:-1, :-1], p[1:, :-1], p[1:, 1:], p[:-1, 1:]
    return np.concatenate([np.stack([a, b, c], 2).reshape(-1, 3, 3), np.stack([a, c, d], 2).reshape(-1, 3, 3)])


def tube(path, r, n=8):
    """A round tube along a polyline (its ends closed)."""
    path = np.asarray(path, float)
    t = np.gradient(path, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    ref = np.array([0, 0, 1.0]) if abs(t[0][2]) < 0.9 else np.array([1.0, 0, 0])
    u = np.cross(t[0], ref)
    u /= np.linalg.norm(u)
    rings = []
    for k in range(len(path)):                    # parallel transport of u along the path
        u = u - (u @ t[k]) * t[k]
        u /= np.linalg.norm(u)
        w = np.cross(t[k], u)
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)[:, None]
        rings.append(path[k] + r * (np.cos(a) * u + np.sin(a) * w))
    rings = np.array(rings)
    a, b = rings[:-1], rings[1:]
    c, d = np.roll(b, -1, axis=1), np.roll(a, -1, axis=1)
    tris = [np.stack([a, b, c], 2).reshape(-1, 3, 3), np.stack([a, c, d], 2).reshape(-1, 3, 3)]
    for ring, end, sign in ((rings[0], path[0], -1), (rings[-1], path[-1], 1)):
        e = np.repeat(end[None], n, 0)
        nxt = np.roll(ring, -1, axis=0)
        tris.append(np.stack([e, ring, nxt] if sign < 0 else [e, nxt, ring], 1))
    return np.concatenate(tris)


def rounded(points, radius, n=8):
    """A polyline through its points, its corners rounded (quadratic Bezier curves)."""
    p = np.asarray(points, float)
    out = [p[0]]
    for a, v, c in zip(p[:-2], p[1:-1], p[2:]):
        p1 = v + (a - v) * min(radius / np.linalg.norm(a - v), 0.5)
        p2 = v + (c - v) * min(radius / np.linalg.norm(c - v), 0.5)
        t = np.linspace(0, 1, n)[:, None]
        out.extend((1 - t) ** 2 * p1 + 2 * t * (1 - t) * v + t ** 2 * p2)
    out.append(p[-1])
    out = np.array(out)
    keep = np.r_[True, np.linalg.norm(np.diff(out, axis=0), axis=1) > 1e-6]   # corners meeting on a short leg
    return out[keep]


def bezier(points, n=40):
    """A Bezier curve through its control points."""
    p = np.asarray(points, float)
    t = np.linspace(0, 1, n)[:, None]
    while len(p) > 1:
        p = np.stack([(1 - t) * a + t * b for a, b in zip(p[:-1], p[1:])]) if p.ndim == 2 else \
            (1 - t) * p[:-1] + t * p[1:]
    return p[0]


# ---- its parts ---------------------------------------------------------------------------------------------------
# Servo-wheel.scad: its 4-arm horn's centre a 10.2 mm square, its arms from it, 15 mm long, 5 mm round at their ends,
# its plate 2 mm thick in its wheel's recess (horn_height - horn_plate_height up from its middle: flush with its outer
# face), its hub through the wheel's middle hole to the servo's spline; its two screws on opposite arms (its photo),
# 13.3 mm out.
HORN_CENTRE, HORN_ARM, HORN_END, HORN_SCREW_MM = 10.0, 15.0, 5.0, 13.3
HORN_PLATE_Z = (S.WHEEL_HEIGHT / 2 - 2.0, S.WHEEL_HEIGHT / 2)


def horn():
    """Its 4-arm horn in Servo-wheel.scad's frame."""
    plate = [prism(np.array([[-5.1, -5.1], [5.1, -5.1], [5.1, 5.1], [-5.1, 5.1]]), *HORN_PLATE_Z)]
    end = HORN_CENTRE / 2 + HORN_ARM - HORN_END / 2                                  # 17.5 mm out
    arm = _hull2(np.vstack([_circle(HORN_END / 2, 20, (0, end)),
                            [[-HORN_CENTRE / 2, HORN_CENTRE / 2], [HORN_CENTRE / 2, HORN_CENTRE / 2],
                             [HORN_CENTRE / 2, HORN_CENTRE / 2 + 2], [-HORN_CENTRE / 2, HORN_CENTRE / 2 + 2]]]))
    for k in range(4):
        c, s = np.cos(k * np.pi / 2), np.sin(k * np.pi / 2)
        plate.append(prism(_hull2(arm @ np.array([[c, s], [-s, c]])), *HORN_PLATE_Z))
    hub = cylinder(HORN_CENTRE / 2 - 0.3, -S.WHEEL_HEIGHT / 2 - S.WHEEL_GAP, HORN_PLATE_Z[0] + 0.01, 24)
    return np.concatenate(plate + [hub])


def horn_screws():
    """The two screws on its arms (pan heads; its hub's middle open, as in its photo), in Servo-wheel.scad's frame."""
    top = HORN_PLATE_Z[1]
    return np.concatenate([cylinder(1.7, top, top + 1.0, 16, c) for c in ((0, HORN_SCREW_MM), (0, -HORN_SCREW_MM))])


def o_ring():
    """Its 3 mm O-ring in the V of its rim (raw_wheel(): the V's bottom at the rim's radius less 1.5 mm, its sides at
    45 deg): its section's middle 1.5 mm x sqrt(2) out of it, in Servo-wheel.scad's frame."""
    big = S.RIM_RADIUS_MM - S.WHEEL_OR_DIAM / 2 + S.WHEEL_OR_DIAM / 2 * np.sqrt(2)
    return torus(big, S.WHEEL_OR_DIAM / 2)


UPPER_SPACER_MM = 8.0


def spacers():
    """The Skymega's four M3 hex spacers (5.5 mm across flats) from the top plate to the board and the four over it
    (its photo), robot frame."""
    cx, cy, _ = S.scad_to_robot(S.SKYMEGA_X_SCAD, S.MID_Y_SCAD, 0)
    plate = S.scad_to_robot(0, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)[2]
    hexagon = _circle(5.5 / np.sqrt(3), 6) @ np.array([[np.cos(np.pi / 6), np.sin(np.pi / 6)],
                                                        [-np.sin(np.pi / 6), np.cos(np.pi / 6)]])
    top = board_top()[2]
    return np.concatenate([prism(hexagon + (cx + dx * S.SKYMEGA_DRILL, cy + dy * S.SKYMEGA_DRILL), z0, z1)
                           for dx in (-1, 1) for dy in (-1, 1)
                           for z0, z1 in ((plate, plate + S.SKYMEGA_SPACER_MM), (top, top + UPPER_SPACER_MM))])


# Its board's servo headers (skymega.py: Servo1..8, three pads each, 2.54 mm pitch), the two its servos are plugged
# into (Obijuan's examples: SERVO2 and SERVO4), its power connector M1: where model.py draws their plugs.
SERVO_PLUGGED = {'r': 2, 'l': 4}                    # SERVO2, SERVO4 (pins 8 and 9)
RISE_Y = -9.5                                       # where their cables come up, from the board's middle
PLUG_H = 14.0                                       # a servo plug's housing over the header's base
POWER_H = 7.1                                       # its connector's housing's height over the board (its model's)
BAT_PLUG_H = 8.0                                    # its battery's plug (cream housing) on it
PLUG_BASE = 2.5                                     # its headers' black bodies over the board


def power():
    """Its power connector's middle on the board: (x, y) from the board's middle."""
    (x0, y0), (x1, y1) = skymega.power_connector()
    return (x0 + x1) / 2, (y0 + y1) / 2


def board_top():
    cx, cy, _ = S.scad_to_robot(S.SKYMEGA_X_SCAD, S.MID_Y_SCAD, 0)
    return cx, cy, S.scad_to_robot(0, 0, S.SKYMEGA_BOTTOM_SCAD)[2] + S.PCB_MM


def plug(side):
    """A servo plug's place on its board: its middle pin's (x, y), its top's z."""
    cx, cy, top = board_top()
    x, y = skymega.servo_header(SERVO_PLUGGED[side])[1]                # its middle pad
    return cx + x, cy + y, top + PLUG_BASE + PLUG_H


def wires():
    """Its servos' cables (three strands side by side: black, red, white) from under its board up through the top
    plate's cutout, out of the board's left side and over into their plugs; its battery's red and black wires from
    its holder up through the top plate's front cutout to the power connector. {colour: triangles}."""
    cx, cy, top = board_top()
    plate = S.scad_to_robot(0, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)[2]
    out = {'black': [], 'red': [], 'white': []}
    r = 0.55
    for side, x0, loop in (('r', 6.0, 0.0), ('l', -6.0, 3.5)):     # the second round the first
        px, py, pz = plug(side)
        yr = cy + RISE_Y                                            # up through the top plate's cutout
        path = rounded([(x0, yr, plate - 8), (x0, yr, plate + 3 + loop), (cx - 27 - loop, yr, plate + 3 + loop),
                        (cx - 31 - loop, yr, top + 8), (cx - 28 - loop, py, pz + 4 + loop), (px, py, pz + 4 + loop),
                        (px, py, pz)], 6.0)
        for k, colour in enumerate(('black', 'red', 'white')):        # -, +, signal: across the plug
            out[colour].append(tube(path + (0, (k - 1) * 1.1, 0), r))
    # the battery's wires: from the holder's front, up through the top plate's front cutout, over to the connector
    bx, by = cx + power()[0], cy + power()[1]
    y = S.scad_to_robot(S.BATTERY_C1 - 2, 0, 0)[1]
    for k, colour in enumerate(('red', 'black')):
        dx = (k - 0.5) * 1.4
        path = bezier([(8 + dx, y, plate - 18), (8 + dx, y, plate + 10), (4 + dx, y + 2, top + 16),
                       (bx + dx, by + 12, top + POWER_H + BAT_PLUG_H + 11), (bx + dx, by, top + POWER_H + BAT_PLUG_H + 7), (bx + dx, by, top + POWER_H + BAT_PLUG_H)], 50)
        out[colour].append(tube(path, 0.6))
    return {k: np.concatenate(v) for k, v in out.items()}


# ---- build -------------------------------------------------------------------------------------------------------
BODY = ('printed', 'servos', 'spacers', 'wire_black', 'wire_red', 'wire_white')
WHEEL = ('wheel', 'horn', 'horn_screws', 'o_ring')


BOARD = ('skymega_top', 'skymega_bottom', 'skymega_side')      # its board's faces and edge (skymega.py)
PARTS_JSON = os.path.join(MESHES, 'skymega_parts.json')       # its components' meshes' colours


def names():
    return list(BODY) + [f'{k}_{s}' for s in 'rl' for k in WHEEL] + list(BOARD)


def skymega_parts():
    """Its components' meshes (one per colour): [(name, path, (r, g, b, a))], from build()."""
    with open(PARTS_JSON) as f:
        return [(n, p, tuple(c)) for n, p, c in json.load(f)]


def build(out=MESHES):
    """Its meshes in out: {name: path}."""
    path = {k: os.path.join(out, f'{k}.obj') for k in names()}
    sources = [__file__, S.__file__, skymega.__file__]
    if all(os.path.exists(p) for p in [*path.values(), PARTS_JSON]) and \
            min(map(os.path.getmtime, [*path.values(), PARTS_JSON])) > max(map(os.path.getmtime, sources)):
        return path
    os.makedirs(out, exist_ok=True)
    cx, cy, top = board_top()
    colours = skymega.build(out, (cx, cy, top), write_obj)
    with open(PARTS_JSON, 'w') as f:
        json.dump(colours, f)
    stl = {k: read_stl(os.path.join(S.MINISKYBOT_DIR, p)) for k, p in S.STL.items()}
    # its chassis as printed: rotate([0, 180, 0]) translate([0, 0, -servo_c1 - top_plate_thickness]), undone
    ch = stl['chassis'] * (-1, 1, -1) + (0, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)
    # its battery holder: battery() under the frame, its top battery_top_gap under the plate's bottom
    b = stl['battery_holder'] + (S.BATTERY_C1 / 2, S.MID_Y_SCAD, -S.BATTERY_C3 - S.BATTERY_TOP_GAP)
    write_obj(path['printed'], _to_robot(np.concatenate([ch, b])))
    # its servos: translate([servo_c4 / 2, servo_c3, servo_c1 / 2]) rotate([+-90, 0, 0])
    sv = [stl['servo'] @ _rx(deg).T + (S.SERVO_C4 / 2, S.SERVO_C3 + dy, S.SERVO_C1 / 2) for deg, dy in ((90, 0), (-90, 0.1))]
    write_obj(path['servos'], _to_robot(np.concatenate(sv)), crease=25)
    write_obj(path['spacers'], spacers(), crease=40)
    for colour, tris in wires().items():
        write_obj(path[f'wire_{colour}'], tris, crease=60)
    # its wheels, about their own axle
    for side, deg in (('r', 90), ('l', -90)):
        for k, tris in (('wheel', stl['wheel']), ('horn', horn()), ('horn_screws', horn_screws()), ('o_ring', o_ring())):
            write_obj(path[f'{k}_{side}'], _wheel_to_own(tris, deg), crease=60 if k == 'o_ring' else CREASE_DEG)
    return path


if __name__ == '__main__':
    for k, p in build().items():
        print(k, '->', p)
