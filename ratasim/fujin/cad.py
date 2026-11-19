"""Fujin's meshes from its STEP (boards/Fujin_V1/Fujin_Body_Full.step in its repository: its board with every
component and its mechanics): every part where the STEP puts it, tessellated with gmsh (as ZoroBot3's: zorobot/cad.py)
and merged per moving body and colour, in the robot frame (mm: x right, y forward, z up, origin on the axle between
the wheels, found from the two tyres). Its board's two faces textured with its team's photos of them (board.py: its
art, printed in colour, is in none of its files); the black printed caps its IR sensors wear in its photos (not in its
STEP) made here, round each emitter and phototransistor pair.

    python -m ratasim.fujin.cad [STEP OUT_DIR]

The simulator calls ensure(), which needs gmsh only when the meshes are missing or older than the STEP, this file or
board.py.

Writes OUT_DIR/<body>_<n>.stl, OUT_DIR/board_{top,bottom}.obj and .png (its faces, UVs on their textures) and
OUT_DIR/manifest.json: [{"file", "body", "rgba", "parts"[, "texture"]}], body in chassis, wheel_l, wheel_r.
"""
import json
import math
import os
import sys

import numpy as np

from . import board as B
from . import config as F

WHEEL_PARTS = ('pneu_miniz', 'MM_wheel', 'spur_gear', 'MM_Encoder_hub')     # turn with the wheel
TYRE = 'pneu_miniz'

# Its colours, as in its team's photos (raiju.team/fujin: the robot on a table, its board from below and from above,
# its printed frame, its fan), not the STEP's (a pink frame, light grey tyres, every uncoloured part blue).
BOARD_RGB = (105, 178, 212)         # its board, printed sky blue (its art left out; darker than in its photos,
                                    # which the studio's lights wash out)
PRINTED_RGB = (34, 34, 36)          # its frame, wheels, encoders' hubs and fan: black PLA
TYRE_RGB = (22, 22, 22)
METAL_RGB = (190, 192, 198)
PART_RGB = (                        # name contains: colour
    (('MM_frame', 'MM_wheel', 'MM_Encoder_hub', 'MM_fan'), PRINTED_RGB),
    ((TYRE,), TYRE_RGB),
    (('spur_gear',), (236, 236, 230)),               # white
    (('MM_8523_motor',), (40, 40, 44)),               # its coreless motors' black cans
    (('SKF_', 'Cylinder'), METAL_RGB),               # the wheels' bearings, its pin header
    (('cap0603',), (160, 125, 85)),
    (('SW3dPS-0603', 'CircuitWorks-SOT-223', 'b3u-1000p'), (25, 25, 25)),
    (('ws2812B',), (240, 240, 232)),
    (('9.6.1.1', '9.63.1.1'), (235, 228, 210)),      # its JST connectors
)
UNCOLOURED = (0, 0, 255)            # what gmsh gives a part with no colour in the STEP
# Its IR sensors' caps (its photos: black PLA, as its frame): each a block round its emitter and its phototransistor,
# both lying along the sensor's axis, its section two round lobes over them down to the board, a hole for each, its
# front flush with theirs.
CAP_LEN_MM = 6.0
CAP_RADIUS_MM = 3.7                 # its lobes' (the emitters are 5 mm LEDs, 5.9 at their flange)
CAP_HOLES_MM = (2.6, 2.75)          # its holes' radii: the emitter's, the phototransistor's can's


def colour(name, rgb, tris):
    """Its colour (as its photos show), by its name or, for the STEP's unnamed parts, by its size and place (tris in
    the robot frame)."""
    v = tris.reshape(-1, 3)
    size = sorted(v.max(0) - v.min(0))
    if size[0] < 2 and size[2] > 70:
        return BOARD_RGB
    for keys, c in PART_RGB:
        if any(k in name for k in keys):
            return c
    if tuple(rgb) != UNCOLOURED:
        return tuple(rgb)
    if np.abs((v.min(0) + v.max(0))[1:] / 2).max() < 0.5:
        return METAL_RGB            # on the axle: its bearings' balls and rings
    if size[2] > 20:
        return PRINTED_RGB          # its 3S battery's cells, wrapped in black tape
    if len(tris) > 5000 or max(size) > 12:
        return METAL_RGB            # its phototransistors' cans, the header's pins
    if size[0] > 5 and len(tris) < 400:
        return (225, 225, 235)      # its emitters: water-clear 5 mm LEDs
    if 5.5 < size[2] < 6:
        return (200, 165, 90)       # the sensors' bent leads
    return (22, 22, 24)             # small components' bodies


def body(name, tris):
    if any(k in name for k in WHEEL_PARTS):
        return 'wheel_l' if tris[..., 0].mean() < 0 else 'wheel_r'
    return 'chassis'


def caps(parts):
    """The caps' triangles (robot frame, mm): for each sensor (config.IR_SENSORS: its emitter's centre and axis), the
    parts lying by its emitter (their centres within 8 mm of it, 2 to 5 mm over the board), its phototransistor the
    biggest of them besides the emitter."""
    import gmsh
    from ..zorobot.cad import outward
    top = F.PCB_BOTTOM_MM + 1.5
    out = []
    for _, (x, y), deg in F.IR_SENSORS.values():
        a = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))])
        side = np.array([a[1], -a[0]])            # its local x (right of its axis)
        near = []
        for _, _, t in parts:
            v = t.reshape(-1, 3)
            c = (v.min(0) + v.max(0)) / 2
            if np.hypot(c[0] - x, c[1] - y) < 8 and top + 2 < c[2] < top + 5:
                near.append((len(t), c, v))
        emitter = min(near, key=lambda n: np.hypot(n[1][0] - x, n[1][1] - y))
        pt = max((n for n in near if n is not emitter), key=lambda n: n[0])
        front = max(float(((v[:, :2] - (x, y)) @ a).max()) for _, _, v in near)
        z = emitter[1][2]
        lobes = ((0.0, CAP_HOLES_MM[0]), (float((pt[1][:2] - (x, y)) @ side), CAP_HOLES_MM[1]))
        gmsh.initialize()
        gmsh.option.setNumber('General.Terminal', 0)
        occ = gmsh.model.occ
        y0 = front - CAP_LEN_MM                     # local: x right, y along its axis from the emitter, z up
        solid = [(3, occ.addCylinder(u, y0, z, 0, CAP_LEN_MM, 0, CAP_RADIUS_MM)) for u, _ in lobes]
        lo, hi = min(u for u, _ in lobes), max(u for u, _ in lobes)
        solid += [(3, occ.addBox(lo, y0, top, hi - lo, CAP_LEN_MM, z + CAP_RADIUS_MM - top)),
                  (3, occ.addBox(lo - CAP_RADIUS_MM, y0, top, hi - lo + 2 * CAP_RADIUS_MM, CAP_LEN_MM, z - top))]
        body, _ = occ.fuse(solid[:1], solid[1:])
        cut = [(3, occ.addCylinder(u, y0 - 1, z, 0, CAP_LEN_MM + 2, 0, r)) for u, r in lobes]
        cut.append((3, occ.addBox(lo - 10, y0 - 1, top - 10, hi - lo + 20, CAP_LEN_MM + 2, 10)))
        body, _ = occ.cut(body, cut)
        occ.synchronize()
        gmsh.option.setNumber('Mesh.MeshSizeMax', 1.0)
        gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 16)
        gmsh.model.mesh.generate(2)
        tags, xyz, _ = gmsh.model.mesh.getNodes()
        index = np.zeros(int(tags.max()) + 1, int)
        index[tags.astype(int)] = np.arange(len(tags))
        xyz = xyz.reshape(-1, 3)
        faces = []
        for d, v in body:
            for _, s in gmsh.model.getBoundary([(d, v)], oriented=False):
                types, _, nodes = gmsh.model.mesh.getElements(2, abs(s))
                faces += [index[nd.astype(int).reshape(-1, 3)] for ty, nd in zip(types, nodes) if ty == 2]
        gmsh.finalize()
        t = np.concatenate(outward(faces, xyz))
        xy = (x, y) + t[..., :1] * side + t[..., 1:2] * a
        out.append(np.concatenate([xy, t[..., 2:]], axis=-1))
    return np.concatenate(out)


def board_faces(tris, out):
    """The board's triangles: its top and bottom faces written as board_top.obj and board_bottom.obj, UVs on their
    textures (board.py); the rest (its edges, its holes' walls) returned."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n = n[:, 2] / np.linalg.norm(n, axis=1)
    z0, z1 = tris[..., 2].min(), tris[..., 2].max()
    faces = {'top': (n > 0.99) & (np.abs(tris[..., 2] - z1).max(1) < 0.01),
             'bottom': (n < -0.99) & (np.abs(tris[..., 2] - z0).max(1) < 0.01)}
    for face, m in faces.items():
        v, idx = np.unique(tris[m].reshape(-1, 3).round(4), axis=0, return_inverse=True)
        idx = idx.reshape(-1, 3) + 1
        with open(os.path.join(out, f'board_{face}.obj'), 'w') as f:
            f.write(''.join(f'v {a:.4f} {b:.4f} {c:.4f}\n' for a, b, c in v))
            f.write(''.join(f'vt {u:.5f} {w:.5f}\n' for u, w in B.uv(v[:, :2])))
            f.write(''.join(f'f {a}/{a} {b}/{b} {c}/{c}\n' for a, b, c in idx))
    return tris[~(faces['top'] | faces['bottom'])]


def build(step=F.CAD_STEP, out=F.CAD_MESHES):
    from ..zorobot.cad import tessellate, write_stl
    B.download()
    parts = tessellate(step)
    tyres = [t.reshape(-1, 3) for n, _, t in parts if TYRE in n]
    if len(tyres) != 2:
        raise SystemExit(f'Fujin: expected two tyres ({TYRE}) in its STEP, found {len(tyres)}')
    axle = np.mean([(v.min(0) + v.max(0)) / 2 for v in tyres], axis=0)
    if np.abs(axle - F.AXLE_IN_STEP_MM).max() > 0.5:
        raise SystemExit(f'Fujin: its tyres centred at {axle.round(2)}, not at {F.AXLE_IN_STEP_MM} (config.py)')
    os.makedirs(out, exist_ok=True)
    B.textures(out, BOARD_RGB)
    manifest = [{'file': f'board_{face}.obj', 'body': 'chassis', 'rgba': [1, 1, 1, 1], 'parts': ['Board'],
                 'texture': f'board_{face}.png'} for face in ('top', 'bottom')]
    groups = {}
    for name, rgb, t in parts:
        t = t - axle
        rgb = colour(name, rgb, t)
        if rgb == BOARD_RGB:
            t = board_faces(t, out)
        groups.setdefault((body(name, t), rgb), []).append((name, t))
    parts = [(n, rgb, t - axle) for n, rgb, t in parts]
    groups.setdefault(('chassis', PRINTED_RGB), []).append(('IR sensors\' caps', caps(parts)))
    for i, ((b, rgb), items) in enumerate(sorted(groups.items())):
        f = f'{b}_{i:02d}.stl'
        write_stl(os.path.join(out, f), np.concatenate([t for _, t in items]))
        manifest.append({'file': f, 'body': b, 'rgba': [round(c / 255, 3) for c in rgb] + [1],
                         'parts': sorted({n.split('/')[-1] for n, _ in items})})
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print(f'{len(parts)} parts, {sum(len(t) for *_, t in parts)} triangles, {len(manifest)} meshes in {out}')
    return out


def ensure(step=F.CAD_STEP, out=F.CAD_MESHES):
    """The meshes' directory, built first if they are missing or older than the STEP, this file (its colours, its
    caps) or board.py (its textures)."""
    m = os.path.join(out, 'manifest.json')
    if os.path.exists(m) and os.path.getmtime(m) >= max(os.path.getmtime(__file__), os.path.getmtime(B.__file__)) and \
            (not os.path.exists(step) or os.path.getmtime(m) >= os.path.getmtime(step)):
        return out
    if not os.path.exists(step):
        raise SystemExit(f'Fujin: its STEP is missing ({step}): git submodule update --init external/fujin')
    try:
        import gmsh     # noqa: F401
    except ImportError:
        raise SystemExit('Fujin: gmsh is needed to make its meshes from its STEP (pip install -e .[cad]).')
    print('Fujin: its meshes from its STEP (once, about two minutes)', flush=True)
    build(step, out)
    return out


if __name__ == '__main__':
    build(*sys.argv[1:3])
