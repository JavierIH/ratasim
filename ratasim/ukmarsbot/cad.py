"""UKMARSBOT's parts as its KiCad boards place them: every footprint's 3D model (its (model ...): KiCad's library
for the stock parts, kicad-packages3D's STEP files, CC BY-SA 4.0 with an exception for the designs that use them,
downloaded at build time as Bulebule's (bulebule/parts.py); the ukmarsbot repository's own for its gearmotors, its
TB6612FNG module and its switches; its Arduino Nano's X3D), tessellated with gmsh, moved as KiCad moves it (its
offset, rotation and scale, its footprint's place and orientation, flipped under the board for a footprint on the
bottom) and merged per colour, in the robot frame (mm).

KiCad's 3D frame: x as the board's, y up the screen (the board's -y), z up from the board's top (from its bottom for
a footprint on the bottom); a model in mm (STEP) or in tenths of an inch (VRML, X3D). A model's points p go to

    T(footprint) Rz(orientation) [Rx(180): on the bottom] T(offset) Rz(-rz) Ry(-ry) Rx(-rx) S(scale) p

(KiCad's 3d-viewer, create_scene.cpp), then to the robot frame as kicad.py's: x_robot = -y3d - 105, y_robot = x3d - 105.
A stock model named .wrl is taken as its .step (kicad-cli's --subst-models). The motors' footprints place the
gearmotors with their brackets; the wheels, the battery and the skids are not on the boards (model.py).

    python -m ratasim.ukmarsbot.cad        # makes them again

Writes MESHES/<n>.stl and MESHES/manifest.json: [{"file", "rgba", "parts"}], and the boards' faces (pcb.py).
"""
import json
import math
import os
import re
import sys
import urllib.request
import xml.etree.ElementTree as ET

import numpy as np

from . import config as U
from . import pcb
from .kicad import ORIGIN, Board, find, one

KICAD = 'https://gitlab.com/kicad/libraries/kicad-packages3D/-/raw/master/'
PARTS = os.path.join(U.BUILD, 'parts')          # the models downloaded, their tessellations
MESHES = os.path.join(U.BUILD, 'meshes')
WRL_MM = 2.54                                   # VRML and X3D models: a unit is 0.1"

# Colours, where a model's are not the robot's (its photos: renders/ukmarsbot-600x526.png)
UNCOLOURED = (0, 0, 255)                        # what gmsh gives a volume or face with no colour
BRACKET_RGB = (28, 28, 30)                      # its gearmotors' brackets, black
GEARBOX_RGB = (200, 200, 196)
CAN_RGB = (190, 192, 198)
ENCODER_PCB_RGB = (20, 110, 50)
MODULE_RGB = (190, 30, 35)                      # the TB6612FNG module: a red board
SWITCH_RGB = (200, 30, 30)
METAL_RGB = (200, 200, 205)
BRASS_RGB = (211, 170, 33)
BLACK_RGB = (25, 25, 27)
CLEAR_RGB, CLEAR_ALPHA = (225, 230, 240), 0.7


def _rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    m = {'x': [[1, 0, 0], [0, c, -s], [0, s, c]], 'y': [[c, 0, s], [0, 1, 0], [-s, 0, c]],
         'z': [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
    out = np.eye(4)
    out[:3, :3] = m
    return out


def _move(x, y, z):
    out = np.eye(4)
    out[:3, 3] = x, y, z
    return out


def _scale(sx, sy, sz):
    return np.diag([sx, sy, sz, 1.0])


TO_ROBOT = _move(-ORIGIN[1], -ORIGIN[0], 0) @ _rot('z', 90)     # (x3d, y3d) -> (-y3d - 105, x3d - 105)


def _xyz(node, key):
    return [float(v) for v in one(one(node, key), 'xyz')[1:]]


def placements(board, top_z):
    """[(footprint's reference, model's path as written, 4x4 matrix from the model's units to the robot frame)] for
    each shown model of each footprint; top_z: the board's top in the robot frame."""
    out = []
    for f in board.footprints:
        ref = f.ref or next((t[2] for t in find(f.node, 'fp_text') if t[1] == 'reference'), '')
        place = _move(f.x, -f.y, top_z - (board.thickness if f.bottom else 0.0)) @ _rot('z', f.rot)
        if f.bottom:
            place = place @ _rot('y', 180) @ _rot('z', 180)
        for m in find(f.node, 'model'):
            if 'hide' in m:
                continue
            unit = WRL_MM if m[1].lower().endswith(('.wrl', '.x3d')) and not _stock(m[1]) else 1.0
            rx, ry, rz = _xyz(m, 'rotate')
            mtx = (place @ _move(*_xyz(m, 'offset')) @ _rot('z', -rz) @ _rot('y', -ry) @ _rot('x', -rx)
                   @ _scale(*_xyz(m, 'scale')) @ _scale(unit, unit, unit))
            out.append((ref, m[1], TO_ROBOT @ mtx))
    return out


def _stock(path):
    return '3DMODEL_DIR}' in path


def source(path, board_path):
    """Where a model's file is (a KiCad library model downloaded first, as STEP)."""
    if _stock(path):
        rel = re.sub(r'^\$\{KICAD\d*_3DMODEL_DIR\}/', '', path)
        rel = re.sub(r'\.wrl$', '.step', rel)
        local = os.path.join(PARTS, rel.replace('/', '__'))
        if not os.path.exists(local):
            os.makedirs(PARTS, exist_ok=True)
            urllib.request.urlretrieve(KICAD + rel, local + '.part')
            os.replace(local + '.part', local)
        return local
    here = os.path.dirname(board_path)
    return os.path.normpath(os.path.join(here, path.replace('${KIPRJMOD}/', '')))


# ---- tessellation ------------------------------------------------------------------------------------------------
def _x3d(path):
    """[(rgb 0-255, triangles)] of an X3D file: its IndexedFaceSets, their Transforms (translation, rotation, scale)
    and their materials' diffuse colours."""
    def tf(e):
        m = np.eye(4)
        if e.get('translation'):
            m = m @ _move(*map(float, e.get('translation').split()))
        if e.get('rotation'):
            x, y, z, a = map(float, e.get('rotation').split())
            n = math.sqrt(x * x + y * y + z * z) or 1.0
            x, y, z = x / n, y / n, z / n
            c, s, t = math.cos(a), math.sin(a), 1 - math.cos(a)
            r = np.eye(4)
            r[:3, :3] = [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                         [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                         [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]
            m = m @ r
        if e.get('scale'):
            m = m @ _scale(*map(float, e.get('scale').split()))
        return m
    root = ET.parse(path).getroot()
    defs = {e.get('DEF'): e for e in root.iter() if e.get('DEF')}

    def use(e):
        return defs[e.get('USE')] if e is not None and e.get('USE') else e
    out = []

    def walk(e, m):
        if e.tag == 'Transform':
            m = m @ tf(e)
        if e.tag == 'Shape':
            mat = use(e.find('.//Material'))
            rgb = tuple(int(round(float(v) * 255)) for v in mat.get('diffuseColor').split()) if mat is not None \
                else (128, 128, 128)
            ifs = e.find('IndexedFaceSet')
            if ifs is None:
                return
            pts = np.array(use(ifs.find('Coordinate')).get('point').replace(',', ' ').split(), float).reshape(-1, 3)
            pts = (np.c_[pts, np.ones(len(pts))] @ m.T)[:, :3]
            idx = [int(v) for v in ifs.get('coordIndex').replace(',', ' ').split()]
            tris, poly = [], []
            for i in idx + [-1]:
                if i >= 0:
                    poly.append(i)
                    continue
                tris += [(poly[0], poly[k], poly[k + 1]) for k in range(1, len(poly) - 1)]
                poly = []
            if tris:
                out.append((rgb, pts[np.array(tris)]))
            return
        for c in e:
            walk(c, m)
    walk(root, np.eye(4))
    return out


def _step(path):
    """[(rgb 0-255, triangles)] of a STEP file: each face in its own colour, else its volume's, else as uncoloured()
    tells (gmsh reads no colour from the ukmarsbot repository's models: theirs are on sub-shapes)."""
    import gmsh
    from ..zorobot.cad import outward
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    gmsh.option.setString('Geometry.OCCTargetUnit', 'MM')
    gmsh.option.setNumber('Geometry.OCCImportLabels', 1)
    gmsh.model.occ.importShapes(path)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin', 0.05)
    gmsh.option.setNumber('Mesh.MeshSizeMax', 1.5)
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 10)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    index = np.zeros(int(tags.max()) + 1, int)
    index[tags.astype(int)] = np.arange(len(tags))
    xyz = xyz.reshape(-1, 3)
    by_colour = {}
    model = os.path.basename(path)
    for d, v in gmsh.model.getEntities(3):
        name = gmsh.model.getEntityName(d, v)
        bbox = np.array(gmsh.model.getBoundingBox(d, v)).reshape(2, 3)
        own = tuple(gmsh.model.getColor(d, v)[:3])
        faces, colours = [], []
        for _, sface in gmsh.model.getBoundary([(d, v)], oriented=False):
            types, _, nodes = gmsh.model.mesh.getElements(2, abs(sface))
            for ty, nd in zip(types, nodes):
                if ty == 2:
                    faces.append(index[nd.astype(int).reshape(-1, 3)])
                    c = tuple(gmsh.model.getColor(2, abs(sface))[:3])
                    if c == UNCOLOURED:
                        c = own if own != UNCOLOURED else uncoloured(model, name, bbox)
                    colours.append(c)
        for tris, rgb in zip(outward(faces, xyz), colours):
            by_colour.setdefault(rgb, []).append(tris)
    gmsh.finalize()
    return [(rgb, np.concatenate(t)) for rgb, t in by_colour.items()]


def tessellation(path):
    """[(rgb 0-255, triangles (n, 3, 3) in the model's units)] of a model, cached (made again when this file is
    newer: its colours)."""
    npz = os.path.join(PARTS, os.path.basename(path) + '.npz')
    if not (os.path.exists(npz) and os.path.getmtime(npz) >= max(os.path.getmtime(path), os.path.getmtime(__file__))):
        os.makedirs(PARTS, exist_ok=True)
        groups = _x3d(path) if path.lower().endswith('.x3d') else _step(path)
        np.savez_compressed(npz, rgb=np.array([c for c, _ in groups]),
                            **{f't{i}': t for i, (_, t) in enumerate(groups)})
    z = np.load(npz)
    return [(tuple(int(v) for v in z['rgb'][i]), z[f't{i}']) for i in range(len(z['rgb']))]


def _apply(m, tris):
    v = tris.reshape(-1, 3)
    return ((np.c_[v, np.ones(len(v))] @ m.T)[:, :3]).reshape(-1, 3, 3)


def uncoloured(model, name, bbox):
    """The colour of a volume its STEP leaves uncoloured, by its model, its name, its size and place (the model's
    frame, mm), as its photos and its boards' renders show it."""
    lo, hi = bbox
    size = hi - lo
    if model.startswith('motor_encoder_covered'):        # its axis z: the encoder z < 2.3, the can, the gearbox
        if 'bracket' in name:
            return BRACKET_RGB
        if size[2] > 15:
            return GEARBOX_RGB
        if size[2] > 10:
            return CAN_RGB
        if size[0] > 10:
            return ENCODER_PCB_RGB
        if max(size[0], size[1]) < 1.1:
            return METAL_RGB                             # its back shaft
        if size[0] < 0.5:
            return BRASS_RGB                             # its terminals
        return BLACK_RGB                                 # its Hall sensors
    if sorted(size)[1] < 1.0:
        return METAL_RGB                                 # pins and leads
    if model.startswith('TB6612') and sorted(size)[0] < 1.8 and sorted(size)[2] > 15:
        return MODULE_RGB
    if model.startswith('MINI-SPDT') and lo[1] < 4:
        return SWITCH_RGB
    return BLACK_RGB


def recolour(path, rgb):
    """Its IR emitters (SFH4545) water-clear, as in its photos: KiCad's model of them is green."""
    if 'LED_D5.0mm_Horizontal_O6.35mm' in path and rgb == (102, 178, 38):
        return CLEAR_RGB
    return rgb


def _on_axle(pieces):
    """A gearmotor lifted onto its wheels' axle (config.AXLE_Z, its gearbox centred on it): its footprint's offset
    (z 1.6, the board's thickness, as if from its bottom) leaves it 0.5 mm into the board, 0.45 mm under the axle."""
    box = np.concatenate([t for rgb, t in pieces if rgb == GEARBOX_RGB]).reshape(-1, 3)
    dz = U.AXLE_Z - (box[:, 2].min() + box[:, 2].max()) / 2
    return [(rgb, t + (0, 0, dz)) for rgb, t in pieces]


def parts():
    """[(reference, model's file, rgb, triangles in the robot frame mm)] of both boards."""
    out = []
    for board_path, top in ((U.MAINBOARD, U.MAIN_TOP), (U.SENSOR_BOARD, U.SENSOR_TOP)):
        for ref, path, m in placements(Board(board_path), top):
            src = source(path, board_path)
            pieces = [(recolour(src, rgb), _apply(m, t)) for rgb, t in tessellation(src)]
            if os.path.basename(src).startswith('motor_encoder_covered'):
                pieces = _on_axle(pieces)
            out += [(ref, os.path.basename(src), rgb, t) for rgb, t in pieces]
    return out


def build(out=MESHES):
    from ..zorobot.cad import write_stl
    groups = {}
    for ref, name, rgb, t in parts():
        groups.setdefault(rgb, []).append((f'{ref} {name}', t))
    os.makedirs(out, exist_ok=True)
    manifest = []
    for i, (rgb, items) in enumerate(sorted(groups.items())):
        f = f'part_{i:02d}.stl'
        write_stl(os.path.join(out, f), np.concatenate([t for _, t in items]))
        manifest.append({'file': f, 'rgba': [round(c / 255, 3) for c in rgb] + [CLEAR_ALPHA if rgb == CLEAR_RGB else 1],
                         'parts': sorted({n for n, _ in items})})
    pcb.build(out)
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print(f'UKMARSBOT: {sum(len(t) for _, t in groups.items())} parts\' pieces, {len(manifest)} meshes in {out}')
    return out


def ensure(out=MESHES):
    """The meshes' directory, made first if they are missing or older than this file, pcb.py, config.py or the
    boards."""
    m = os.path.join(out, 'manifest.json')
    sources = [__file__, pcb.__file__, U.__file__, U.MAINBOARD, U.SENSOR_BOARD]
    if os.path.exists(m) and os.path.getmtime(m) >= max(map(os.path.getmtime, sources)):
        return out
    try:
        import gmsh     # noqa: F401
    except ImportError:
        raise SystemExit('UKMARSBOT: gmsh is needed to make its parts from their 3D models (pip install -e .[cad]).')
    print('UKMARSBOT: its parts from their 3D models (once)', flush=True)
    return build(out)


if __name__ == '__main__':
    build(*sys.argv[1:2])
