"""KOGUMA's meshes from its STEP (hardware/KOGUMA.Mk3.step in its repository: its Fusion 360 assembly, every part
where it is), tessellated with gmsh (as ZoroBot3's: zorobot/cad.py) and merged per moving body and colour, in the
robot frame (mm: x right, y forward, z up, origin on the axle between its tyres). Its colours are its photo's (its
maker's: KOGUMA-CHAN. Mk3), not its STEP's: its carbon fibre bottom plate (its two faces textured with a 2/2 twill
weave, drawn here), its board's faces and parts from its KiCad board (board(): pcb.py), its printed parts black
(its sensors' holders, its motors' mounts, its fan motor's holder), its wheels white with black tyres, its spur
gears white, its motors' pinions brass, its silver motors, its orange axle screws and silver bearing collars, its
purple nylon nuts. Drawn here, as its photos show them and its STEP does not: its rims (Kyosho's Multi Wheel II:
rim()), its IR emitters and photodiodes with their legs (sensors()), its battery's four cells as one block in black
tape with rounded edges (battery()), the screw on its fan motor holder's nose (nose_screw()). The beams its STEP
draws from its four sensors (3.2 mm rods 125 mm long) are left out.

    python -m ratasim.koguma.cad [STEP OUT_DIR]

The simulator calls ensure(), which needs gmsh only when the meshes are missing or older than the STEP, the KiCad
board or the code.

Writes OUT_DIR/<body>_<n>.stl, OUT_DIR/plate_{top,bottom}.obj and carbon.png (its plate's faces, UVs on the weave),
pcb_{top,bottom}.obj and .png (its board's faces), mcu_top.obj and .png (its STM32's marking) and
OUT_DIR/manifest.json: [{"file", "body", "rgba", "parts"[, "texture"]}], body in chassis, wheel_l, wheel_r.
"""
import json
import math
import os
import re
import sys

import numpy as np

from . import config as K

TYRE = ('コンポーネント1', 'Component1')
WHEEL_PARTS = ('ホイール＆タイヤ', 'T71')       # turn with the wheel: its tyre and rim, its spur gear
PLATE = '底板'
BOARD = '基板2'                                  # its board: its faces from its KiCad board (board())
RIM = 'ホイール＆タイヤ'                         # its STEP's wheel (a plain disc in its tyre), drawn again: rim()
RIM_X_MM = (26.3, 34.3)                         # the right rim's across (its STEP's, less its spur gear's 2 mm)
RIM_SPOKES = 16
BATTERY_EDGE_MM = 1.0                           # its taped battery's edges' radius
NOSE_HOLE_XY = (-6.84, 34.22)                   # its fan motor holder's nose hole, over its stacked spacers (STEP)
RIM_PAIR_MM = (0.74, 1.12, 1.5)                 # each spoke's middle off its pair's: at the hub, half way, at the lip
# Its colours (its photo), by what a part's name holds; the first that matches
PART_RGB = (
    (('センサホルダ', 'モーターマウント', 'エンコーダギアマウント', '吸引モーター抑え'), (26, 26, 28)),  # printed: black
    (('IR emitter',), (240, 238, 222)),                          # its emitters (OPV332 laser diodes), clear
    (('IR photodiode',), (34, 33, 40)),                     # its photodiodes' dark epoxy (SFH 229FA: daylight filter)
    (('IR legs',), (150, 152, 158)),                             # their tinned legs
    (('コンポーネント1', 'Component1'), (24, 24, 25)),           # its tyres
    (('ホイール＆タイヤ',), (236, 236, 230)),                    # its rims: white
    (('T71', 'T41', '71 teeth', '41 teeth'), (238, 236, 226)),   # its spur gears and its encoders' gears: white
    (('T20', '20 teeth'), (196, 156, 66)),                       # its motors' pinions: brass
    (('NFP-D0816S', '吸引モータ'), (118, 121, 128)),             # its motors' cans: silver (lighter ones wash
                                                                 # out to white under the studio's lights)
    (('アルミテーパー',), (176, 179, 186)),                      # its wheels' tapered bearing collars: silver
    (('ナイロンナット',), (112, 58, 150)),                       # purple nylon nuts
    (('ボタンビス', 'Screw'), (214, 104, 38)),                   # its axles' screws: orange, as its STEP and photo
    (('スペーサー',), (34, 34, 36)),                             # its board's posts: black in its photo
    (('ベアリング', 'LF-630', '磁石', '平行ピン'), (128, 130, 136)),
    (('エンコーダ基板', 'TSSOP14'), (22, 22, 24)),
    (('Lipo',), (17, 17, 18)),                                   # its cells, one block in black tape (battery())
    (('ナベ小ネジ',), (38, 38, 41)),                              # the screw on its fan motor holder's nose
    (('吸引ファン',), (178, 184, 190)),                          # its acrylic impeller
    (('一次スカート',), (40, 40, 42)),                           # its skirt
)
FAN_CAP_RGB = (24, 24, 26)              # its fan motor's black end cap (its brushes', its wires' end), on top
FAN_CAP_Z_MM = 86.3 - K.AXLE_IN_STEP_MM[2]      # from here up (its can from 64.3 to 89.3 in its STEP)
LED_RGB = (226, 226, 214)               # what its STEP leaves unnamed in its sensors' groups
PIN_RGB = (90, 90, 94)                  # the encoders' chips' pins (unnamed)
PLATE_EDGE_RGB = (34, 34, 36)
OTHER_RGB = (120, 120, 124)

# Its plate's carbon fibre: a 2/2 twill of 2.5 mm tows, a 10 mm tile (4 tows each way)
TOW_MM = 2.5
PX_MM = 32


def step_name(name):
    """A STEP name as text: its \\X2\\...\\X0\\ escapes (UTF-16 hex) decoded."""
    return re.sub(r'\\X2\\([0-9A-F]+)\\X0\\',
                  lambda m: ''.join(chr(int(m.group(1)[i:i + 4], 16)) for i in range(0, len(m.group(1)), 4)), name)


def colour(name, tris):
    for keys, rgb in PART_RGB:
        if any(k in name for k in keys):
            return rgb
    if not name.split('/')[-1] or name.endswith('SOLID'):
        c = tris.reshape(-1, 3).mean(0)
        return LED_RGB if c[1] > 15 else PIN_RGB
    return OTHER_RGB


def body(name, tris):
    if any(k in name for k in WHEEL_PARTS):
        return 'wheel_r' if tris[..., 0].mean() > 0 else 'wheel_l'
    return 'chassis'


def is_beam(name, tris):
    """The rods its STEP draws along its sensors' beams: unnamed, 125 mm long."""
    v = tris.reshape(-1, 3)
    return not step_name(name).split('/')[-1] and np.ptp(v, axis=0).max() > 60


def carbon_texture(path):
    """A tile of 2/2 twill (4 x 4 tows): each tow floats over two crossing tows and dives under the next two, a step
    along at each row (its diagonal staircase); each float a bundle of fibres, lit along its crown, darker where it
    dives; the floats along y (warp) catch more light than those along x (weft), as a carbon plate's do from one
    side under its resin's gloss."""
    from PIL import Image
    n = int(TOW_MM * PX_MM)
    size = 4 * n
    rng = np.random.default_rng(7)
    yy, xx = np.mgrid[0:size, 0:size]
    i, j = xx // n, yy // n
    u, w = (xx % n + 0.5) / n, (yy % n + 0.5) / n
    k = (i + j) % 4
    warp = k < 2                                        # the tow on top runs along y here
    across = np.where(warp, u, w)                       # across its fibres
    along = np.where(warp, (k + w) / 2, (k - 2 + u) / 2)      # along its float of two cells, 0 to 1
    crown = 1 - (2 * across - 1) ** 2                   # its round section
    dive = 0.6 + 0.4 * np.sin(np.pi * along) ** 0.7     # darker at its ends, under the crossing tows
    streaks = rng.normal(0, 1, (2, size))
    fibres = np.where(warp, streaks[0][xx], streaks[1][yy])
    gap = 0.75 + 0.25 * np.clip(np.minimum(across, 1 - across) * n / 2.0, 0, 1)    # its tows side by side
    sheen = np.where(warp, 1.0, 0.62)
    v = 18 + (14 + 48 * crown * sheen) * dive * gap + 3 * fibres
    rgb = np.stack([v, v, v * 1.05 + 1], axis=-1)
    Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(path)


def plate_faces(tris, out, prefix='plate', uv=None):
    """A plate's top and bottom faces as OBJ files (OUT/PREFIX_top.obj, PREFIX_bottom.obj), their UVs by uv (from
    their vertices, mm: by default on the weave's tile, which MuJoCo repeats); the rest of it (its edges) returned."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n = n[:, 2] / np.linalg.norm(n, axis=1)
    z0, z1 = tris[..., 2].min(), tris[..., 2].max()
    faces = {'top': (n > 0.99) & (np.abs(tris[..., 2] - z1).max(1) < 0.01),
             'bottom': (n < -0.99) & (np.abs(tris[..., 2] - z0).max(1) < 0.01)}
    for face, m in faces.items():
        v, idx = np.unique(tris[m].reshape(-1, 3).round(4), axis=0, return_inverse=True)
        idx = idx.reshape(-1, 3) + 1
        uvs = v[:, :2] / (4 * TOW_MM) if uv is None else uv(v)
        with open(os.path.join(out, f'{prefix}_{face}.obj'), 'w') as f:
            f.write(''.join(f'v {a:.4f} {b:.4f} {c:.4f}\n' for a, b, c in v))
            f.write(''.join(f'vt {a:.5f} {b:.5f}\n' for a, b in uvs))
            f.write(''.join(f'f {a}/{a} {b}/{b} {c}/{c}\n' for a, b, c in idx))
    return tris[~(faces['top'] | faces['bottom'])]


def board(tris, out):
    """Its board from its KiCad board (pcb.py) on its STEP's (its triangles, robot frame): its top and bottom faces
    textured (pcb_top.obj, pcb_bottom.obj), its parts, its MCU's marked top (mcu_top.obj). KiCad's main board
    centred on the STEP's (their outlines the same to 0.1 mm). Returns its manifest's entries and {rgb: triangles}
    (its edges, its parts)."""
    from . import pcb
    fx0, fy0, fx1, fy1 = pcb.textures(out)
    kx0, ky0, kx1, ky1 = pcb.main_frame()
    v = tris.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    ox, oy, top = (lo[0] + hi[0] - kx0 - kx1) / 2, (lo[1] + hi[1] + ky0 + ky1) / 2, hi[2]

    def uv(p):
        return np.stack([(p[:, 0] - ox - fx0) / (fx1 - fx0), 1 - (oy - p[:, 1] - fy0) / (fy1 - fy0)], axis=1)

    def robot(t):
        r = np.stack([t[..., 0] + ox, oy - t[..., 1], top + t[..., 2]], axis=-1)
        return r[:, ::-1]                       # KiCad's y down: mirrored, its winding turned back outward

    edges = plate_faces(tris, out, 'pcb', uv)
    groups = {pcb.EDGE_RGB: [edges]}
    made, mcu = pcb.parts()
    for rgb, t in made.items():
        groups.setdefault(rgb, []).append(robot(t))
    entries = [{'file': f'pcb_{face}.obj', 'body': 'chassis', 'rgba': [1, 1, 1, 1], 'parts': [BOARD],
                'texture': f'pcb_{face}.png'} for face in ('top', 'bottom')]
    if mcu is not None:
        pcb.mcu_texture(os.path.join(out, 'mcu_top.png'))
        c = robot(mcu[None])[0][::-1]           # its corners from its pin 1's, in order
        uvs = ((0, 1), (1, 1), (1, 0), (0, 0))
        n = np.cross(c[1] - c[0], c[2] - c[0])[2]
        faces = ((1, 2, 3), (1, 3, 4)) if n > 0 else ((1, 3, 2), (1, 4, 3))
        with open(os.path.join(out, 'mcu_top.obj'), 'w') as f:
            f.write(''.join(f'v {a:.4f} {b:.4f} {z:.4f}\n' for a, b, z in c))
            f.write(''.join(f'vt {a} {b}\n' for a, b in uvs))
            f.write(''.join(f'f {a}/{a} {b}/{b} {c_}/{c_}\n' for a, b, c_ in faces))
        entries.append({'file': 'mcu_top.obj', 'body': 'chassis', 'rgba': [1, 1, 1, 1], 'parts': ['STM32F446RET6'],
                        'texture': 'mcu_top.png'})
    return entries, groups


def _occ_triangles(volumes, size=(0.1, 1.0), curvature=24):
    """The surface triangles (mm) of these OCC volumes of the current gmsh model, outward: their elements from
    size[0] to size[1] mm, curvature of them a circle."""
    import gmsh
    from ..zorobot.cad import outward
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin', size[0])
    gmsh.option.setNumber('Mesh.MeshSizeMax', size[1])
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', curvature)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    index = np.zeros(int(tags.max()) + 1, int)
    index[tags.astype(int)] = np.arange(len(tags))
    xyz = xyz.reshape(-1, 3)
    out = []
    for d, v in volumes:
        faces = []
        for _, s in gmsh.model.getBoundary([(d, v)], oriented=False):
            types, _, nodes = gmsh.model.mesh.getElements(2, abs(s))
            faces += [index[nd.astype(int).reshape(-1, 3)] for ty, nd in zip(types, nodes) if ty == 2]
        out += outward(faces, xyz)
    return np.concatenate(out)


def rim():
    """Its rims, Kyosho's Mini-Z Multi Wheel II, narrow, white, offset 2.0 (MZH131W-N2B; its STEP has a plain disc;
    Kyosho's photos of them, its maker's of the robot): a barrel under its tyre, a thick front lip rounded at its
    edge, a back wall against its spur gear, and a deep dish: a flat hub disc 3.4 mm in, with a raised ring round
    its middle (its axle's orange screw in it), and sixteen spokes from the disc's edge up to the lip in eight
    pairs, the two of a pair close and nearly parallel (side by side at the disc, a little apart at the lip); each
    spoke curved (the dish's concave profile), narrower at the hub, its top edges chamfered. The right one in the
    robot frame (mm, from the axle's middle); the left one is its mirror."""
    import gmsh
    x_in, x_out = RIM_X_MM
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    occ = gmsh.model.occ

    def tube(x0, x1, r0, r1):
        return occ.cut([(3, occ.addCylinder(x0, 0, 0, x1 - x0, 0, 0, r1))],
                       [(3, occ.addCylinder(x0 - 1, 0, 0, x1 - x0 + 2, 0, 0, r0))])[0]

    def section(x, r, a, half_w, depth):
        """A spoke's section at radius r, angle a: half_w across it, depth along the axle (its top at x), its top
        edges chamfered."""
        c, s = math.cos(a), math.sin(a)
        ch = min(0.3, half_w * 0.6)
        local = ((x - depth, -half_w), (x - depth, half_w), (x - ch, half_w), (x, half_w - ch),
                 (x, -half_w + ch), (x - ch, -half_w))
        pts = [occ.addPoint(px, r * c - w * s, r * s + w * c) for px, w in local]
        return occ.addWire([occ.addLine(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))])

    lip = x_out - 0.1
    solids = tube(x_in, lip - 1.0, 9.7, 10.9)                       # its barrel, under its tyre
    solids += tube(lip - 1.4, lip - 0.55, 9.5, 11.3)                # its lip
    solids += [(3, occ.addTorus(lip - 0.55, 0, 0, 10.4, 0.55, zAxis=[1, 0, 0]))]   # its lip's rounded edge
    solids += tube(x_in, x_in + 0.6, 3.3, 10.9)                     # its back wall
    hub_x = x_out - 3.4                                             # its hub disc's face
    solids += tube(hub_x - 0.9, hub_x, 3.3, 5.5)                    # its hub disc
    solids += tube(hub_x - 0.9, hub_x + 0.8, 3.3, 4.3)              # its raised ring
    pitch = 2 * math.pi / RIM_SPOKES
    for k in range(RIM_SPOKES):
        pair, side = divmod(k, 2)
        sign = 1 if side else -1
        centre = 2 * pitch * pair
        # (radius, its top's x, its angle off the pair's middle: its middle line PAIR_MM[i] off the pair's, half
        # its width)
        stations = ((5.0, hub_x + 0.05, sign * RIM_PAIR_MM[0] / 5.0, 0.52),
                    (7.6, hub_x + 1.55, sign * RIM_PAIR_MM[1] / 7.6, 0.55),
                    (9.9, lip - 0.75, sign * RIM_PAIR_MM[2] / 9.9, 0.6))
        wires = [section(x, r, centre + a, w, 1.25) for r, x, a, w in stations]
        solids += [e for e in occ.addThruSections(wires, makeSolid=True, makeRuled=False) if e[0] == 3]
    whole, _ = occ.fuse(solids[:1], solids[1:])
    t = _occ_triangles(whole)
    gmsh.finalize()
    return t


def _rounded_box(lo, hi, radius):
    """A box from lo to hi (mm) with every edge rounded to radius: its triangles."""
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    occ = gmsh.model.occ
    box = occ.addBox(*lo, *(np.asarray(hi) - lo))
    occ.synchronize()
    edges = [abs(e[1]) for e in gmsh.model.getBoundary([(2, s) for _, s in gmsh.model.getBoundary([(3, box)])],
                                                         combined=False, oriented=False)]
    rounded = occ.fillet([box], sorted(set(edges)), [radius])
    t = _occ_triangles(rounded, size=(0.5, 8.0), curvature=8)
    gmsh.finalize()
    return t


def battery(cells):
    """Its battery as its photo shows it: its four 180 mAh cells side by side, wrapped together in black tape, one
    block with its edges rounded (1 mm)."""
    v = np.concatenate([t.reshape(-1, 3) for _, t in cells])
    return [('全体v2/Lipo 180mAh x 4 (taped)', _rounded_box(v.min(0), v.max(0), BATTERY_EDGE_MM))]


def nose_screw(holder):
    """The screw its photo shows on its fan motor holder's nose (none in its STEP): an M3 pan head with a cross
    recess (5.5 mm across, 2 mm high), on the ring round the hole over its two stacked spacers. [(name, triangles)]
    in the STEP frame."""
    import gmsh
    v = holder.reshape(-1, 3)
    centre = np.array(NOSE_HOLE_XY)
    top = v[np.hypot(v[:, 0] - centre[0], v[:, 1] - centre[1]) < 2.6, 2].max()      # the ring's top face
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    occ = gmsh.model.occ
    head = occ.addCylinder(0, 0, 0, 0, 0, 2.0, 2.75)
    occ.synchronize()
    faces = gmsh.model.getBoundary([(3, head)], oriented=False)
    rim = {abs(c[1]) for c in gmsh.model.getBoundary(faces, combined=False, oriented=False)}
    ring_top = [c for c in rim if gmsh.model.getBoundingBox(1, c)[2] > 1.99]
    head = occ.fillet([head], ring_top, [0.8])
    slots = [(3, occ.addBox(-1.6, -0.35, 1.0, 3.2, 0.7, 1.5)), (3, occ.addBox(-0.35, -1.6, 1.0, 0.7, 3.2, 1.5))]
    head, _ = occ.cut(head, slots)
    t = _occ_triangles(head)
    gmsh.finalize()
    return [('全体v2/ナベ小ネジ M3 (its photo)', t + (centre[0], centre[1], top))]


def bullet(tip=5.3):
    """A 3 mm through-hole LED's (or photodiode's) epoxy body, its axis +x, its flange's back at 0: a 0.8 mm
    flange 3.6 mm across, a 3 mm body, a round dome at its tip (5.3 mm long in all)."""
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    occ = gmsh.model.occ
    r = 1.5
    parts = [(3, occ.addCylinder(0, 0, 0, 0.8, 0, 0, 1.8)), (3, occ.addCylinder(0.8, 0, 0, tip - r - 0.8, 0, 0, r)),
             (3, occ.addSphere(tip - r, 0, 0, r))]
    whole, _ = occ.fuse(parts[:1], parts[1:])
    t = _occ_triangles(whole)
    gmsh.finalize()
    return t


def holder_holes(tris, origin, axis):
    """The holes in a sensor holder's front face, the upper first: [(centre (3,), radius, its face's normal, the
    holder's depth along it)], from
    its triangles (mm) and its beam's axis. Its front face (the triangles facing along the axis, foremost) drawn on a
    0.05 mm grid: the empty regions inside it are its holes."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1)[:, None]
    facing = n @ (np.asarray(axis, float) / np.linalg.norm(axis)) > 0.95       # its face's own normal, not the beam's
    axis = n[facing].mean(0)
    axis /= np.linalg.norm(axis)
    side = np.cross([0, 0, 1.0], axis)
    side /= np.linalg.norm(side)
    up = np.cross(axis, side)
    along = ((tris - origin) @ axis).mean(1)
    front = along[facing].max()
    face = tris[facing & (np.abs(along - front) < 0.05)]
    uv = np.stack([(face - origin) @ side, (face - origin) @ up], axis=-1)
    lo, hi = uv.reshape(-1, 2).min(0), uv.reshape(-1, 2).max(0)
    step = 0.05
    gu, gv = np.meshgrid(np.arange(lo[0], hi[0], step), np.arange(lo[1], hi[1], step), indexing='ij')
    p = np.stack([gu.ravel(), gv.ravel()], axis=-1)
    covered = np.zeros(len(p), bool)
    for a, b, c in uv:
        v0, v1, v2 = c - a, b - a, p - a
        d00, d01, d11 = v0 @ v0, v0 @ v1, v1 @ v1
        d20, d21 = v2 @ v0, v2 @ v1
        den = d00 * d11 - d01 * d01
        if abs(den) < 1e-12:
            continue
        s1 = (d11 * d20 - d01 * d21) / den
        s2 = (d00 * d21 - d01 * d20) / den
        covered |= (s1 >= -1e-6) & (s2 >= -1e-6) & (s1 + s2 <= 1 + 1e-6)
    empty = ~covered.reshape(gu.shape)
    label = np.zeros(empty.shape, int)
    holes = []
    for start in zip(*np.nonzero(empty)):
        if label[start]:
            continue
        label[start] = len(holes) + 1
        todo, cells, border = [start], [], False
        while todo:
            i, j = todo.pop()
            cells.append((i, j))
            border |= i in (0, empty.shape[0] - 1) or j in (0, empty.shape[1] - 1)
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < empty.shape[0] and 0 <= b < empty.shape[1] and empty[a, b] and not label[a, b]:
                    label[a, b] = len(holes) + 1
                    todo.append((a, b))
        holes.append((cells, border))
    out = []
    for cells, border in holes:
        if border or len(cells) < 200:
            continue
        ij = np.array(cells)
        cu, cv = gu[ij[:, 0], ij[:, 1]].mean(), gv[ij[:, 0], ij[:, 1]].mean()
        radius = math.sqrt(len(cells) * step * step / math.pi)
        out.append((origin + axis * front + side * cu + up * cv, radius, axis, front - along[n @ axis < -0.95].min()))
    return sorted(out, key=lambda h: -h[0][2])


def bar(p0, p1, w, up=(0, 0, 1.0)):
    """A square bar w across from p0 to p1: its triangles, outward."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    a = (p1 - p0) / np.linalg.norm(p1 - p0)
    u = np.cross(a, up) if abs(np.dot(a, up)) < 0.9 else np.cross(a, [1.0, 0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(a, u)
    h = w / 2
    c = [p + h * (su * u + sv * v) for p in (p0, p1) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    quads = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    tris = [(c[q[0]], c[q[i]], c[q[i + 1]]) for q in quads for i in (1, 2)]
    t = np.array(tris)
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    centre = (p0 + p1) / 2
    flip = np.einsum('ij,ij->i', n, t.mean(1) - centre) < 0
    t[flip] = t[flip][:, ::-1]
    return t


def sensors(holders):
    """Its IR sensors' emitters and photodiodes (in none of its STEP's parts but as the beams it draws), as its
    maker's photo shows them: a 3 mm one in each hole of each holder, its dome out of the holder's face (1.3 mm), the
    emitter in the hole its beam goes through (the upper one), the photodiode in the other; each one's two legs
    (0.45 mm, 2.54 mm apart, tinned) straight back out of the holder, then bent down into the board, the emitter's
    farther back. [(name, triangles)] in the STEP frame."""
    body = bullet()
    length = body[..., 0].max()
    board_top = 68.8                            # its board's top in its STEP (z)
    out = []
    for name, (_, (x, y), deg) in K.IR_SENSORS.items():
        origin = np.array([x, y, K.IR_Z_MM]) + K.AXLE_IN_STEP_MM
        axis = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])
        n, t = min(holders, key=lambda h: np.linalg.norm(h[1].reshape(-1, 3).mean(0)[:2] - origin[:2]))
        holes = holder_holes(t, origin, axis)
        if len(holes) != 2:
            raise SystemExit(f'KOGUMA: its {name} sensor\'s holder has {len(holes)} holes, not 2')
        for (centre, _, normal, depth), what, bend in zip(holes, ('IR emitter', 'IR photodiode'), (2.4, 1.0)):
            side = np.cross([0, 0, 1.0], normal)
            side /= np.linalg.norm(side)
            rot = np.stack([normal, side, np.cross(normal, side)], axis=1)
            back = centre + normal * (1.3 - length)
            out.append((f'センサまとめ/{what} ({name})', body @ rot.T + back))
            knee = centre - normal * (depth + bend)
            for s_ in (-1.27, 1.27):
                p0, p1 = back + side * s_, knee + side * s_
                p2 = p1.copy()
                p2[2] = board_top - 0.8
                legs = np.concatenate([bar(p0, p1 + normal * 0.22, 0.45), bar(p1, p2, 0.45)])
                out.append((f'センサまとめ/IR legs ({name})', legs))
    return out


def build(step=K.CAD_STEP, out=K.CAD_MESHES):
    from ..zorobot.cad import tessellate, write_stl
    parts = [(step_name(n), t) for n, _, t in tessellate(step)]
    parts += sensors([(n, t) for n, t in parts if 'センサホルダ' in n])
    cells = [(n, t) for n, t in parts if 'Lipo' in n]
    parts = [(n, t) for n, t in parts if 'Lipo' not in n] + battery(cells)
    parts += nose_screw(next(t for n, t in parts if n.endswith('吸引モーター抑え')))
    right = rim()
    parts = [p for p in parts if not (RIM in p[0] and p[0].endswith('SOLID'))]
    parts += [(f'{RIM}/Kyosho multi-spoke rim', right + K.AXLE_IN_STEP_MM),
              (f'{RIM}/Kyosho multi-spoke rim (mirror)', right[:, ::-1] * (-1, 1, 1) + K.AXLE_IN_STEP_MM)]
    beams = [n for n, t in parts if is_beam(n, t)]
    if len(beams) != 4:
        raise SystemExit(f'KOGUMA: expected the four beams of its sensors in its STEP, found {len(beams)}')
    parts = [(n, t) for n, t in parts if not is_beam(n, t)]
    tyres = [t.reshape(-1, 3) for n, t in parts if any(k in n for k in TYRE)]
    if len(tyres) != 2:
        raise SystemExit(f'KOGUMA: expected two tyres in its STEP, found {len(tyres)}')
    axle = np.mean([(v.min(0) + v.max(0)) / 2 for v in tyres], axis=0)
    if np.abs(axle - K.AXLE_IN_STEP_MM).max() > 0.5:
        raise SystemExit(f'KOGUMA: its tyres centred at {axle.round(2)}, not at {K.AXLE_IN_STEP_MM} (config.py)')
    os.makedirs(out, exist_ok=True)
    carbon_texture(os.path.join(out, 'carbon.png'))
    manifest = [{'file': f'plate_{face}.obj', 'body': 'chassis', 'rgba': [1, 1, 1, 1], 'parts': [PLATE],
                 'texture': 'carbon.png'} for face in ('top', 'bottom')]
    groups = {}
    for name, t in parts:
        t = t - axle
        if BOARD in name:
            entries, made = board(t, out)
            manifest += entries
            for rgb, tris in made.items():
                groups.setdefault(('chassis', rgb), []).extend((f'{BOARD}/KiCad', x) for x in tris)
            continue
        if PLATE in name:
            groups.setdefault(('chassis', PLATE_EDGE_RGB), []).append((name, plate_faces(t, out)))
            continue
        if name.endswith('/吸引モータ'):
            cap = t[..., 2].min(1) >= FAN_CAP_Z_MM
            groups.setdefault(('chassis', FAN_CAP_RGB), []).append((name, t[cap]))
            t = t[~cap]
        groups.setdefault((body(name, t), colour(name, t)), []).append((name, t))
    for i, ((b, rgb), items) in enumerate(sorted(groups.items())):
        f = f'{b}_{i:02d}.stl'
        write_stl(os.path.join(out, f), np.concatenate([t for _, t in items]))
        manifest.append({'file': f, 'body': b, 'rgba': [round(c / 255, 3) for c in rgb] + [1],
                         'parts': sorted({n.split('/')[-1] or '(unnamed)' for n, _ in items})})
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1, ensure_ascii=False)
    print(f'{len(parts)} parts, {sum(len(t) for _, t in parts)} triangles, {len(manifest)} meshes in {out}')
    return out


def ensure(step=K.CAD_STEP, out=K.CAD_MESHES):
    """The meshes' directory, built first if they are missing or older than the STEP, the KiCad board or the code."""
    from . import pcb
    m = os.path.join(out, 'manifest.json')
    sources = [__file__, pcb.__file__] + [f for f in (step, pcb.PCB) if os.path.exists(f)]
    if os.path.exists(m) and all(os.path.getmtime(m) >= os.path.getmtime(f) for f in sources):
        return out
    if not os.path.exists(step):
        raise SystemExit(f'KOGUMA: its STEP is missing ({step}): git submodule update --init external/KOGUMA.MkIIl')
    try:
        import gmsh     # noqa: F401
    except ImportError:
        raise SystemExit('KOGUMA: gmsh is needed to make its meshes from its STEP (pip install -e .[cad]).')
    print('KOGUMA: its meshes from its STEP (once, about a minute)', flush=True)
    build(step, out)
    return out


if __name__ == '__main__':
    build(*sys.argv[1:3])
