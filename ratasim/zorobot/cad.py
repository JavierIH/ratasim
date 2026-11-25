"""ZoroBot3's meshes from its Onshape assembly (the "Full Assembly" exported as one STEP file): every part where the
designers placed it, tessellated with gmsh and merged per moving body and colour, in the robot frame (mm: x right,
y forward, z up, origin on the axle between the wheels, found from the two wheels).

    python cad.py STEP OUT_DIR      (any Python with gmsh and numpy; it imports nothing else)

The simulator calls ensure(), which needs gmsh only when the meshes are missing or older than the STEP.

Writes OUT_DIR/<body>_<n>.obj (with each CAD face's own normals) and OUT_DIR/manifest.json: [{"file", "body", "rgba", "parts"}], body in
chassis, wheel_l, wheel_r.
"""
import json
import os
import sys

import numpy as np

WHEEL_PARTS = ('/Rueda/', '/Goma/', '/Spur gear (40 teeth)/')     # turn with the wheel
KICAD_GREY = (210, 209, 199)        # KiCad's STEP gives every component this colour
COMPONENT_RGB = (                   # name contains: colour
    (('C_0', 'C_1'), (160, 125, 85)),
    (('R_0', 'R_2512'), (25, 25, 25)),
    (('LED',), (240, 240, 232)),
    (('USB', 'SW_SPST'), (200, 200, 205)),
    (('JST', 'DF13'), (235, 228, 210)),
)
FAN, RIM = 'Ventilador_v2', 'Rueda'    # its fan the colour of its wheels' rims
METAL = ('ball bearing', '/Ball', 'MR63ZZ', ' shell', 'screw', 'nut', 'Varilla')


def colour(name, rgba):
    rgb = tuple(rgba[:3])
    if any(k in name for k in METAL):
        return (190, 192, 198)
    if rgb == KICAD_GREY:
        for keys, c in COMPONENT_RGB:
            if any(k in name for k in keys):
                return c
        return (22, 22, 24)         # ICs and the rest
    return rgb


def tessellate(step):
    """[(name, rgb, triangles (n, 3, 3) mm in the STEP frame)] for every volume."""
    import gmsh
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    gmsh.option.setString('Geometry.OCCTargetUnit', 'MM')
    gmsh.option.setNumber('Geometry.OCCImportLabels', 1)
    gmsh.model.occ.importShapes(step)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin', 0.15)
    gmsh.option.setNumber('Mesh.MeshSizeMax', 3.0)
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 24)     # points per turn; the minimum keeps vias coarse
    gmsh.option.setNumber('Mesh.Algorithm', 6)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    index = np.zeros(int(tags.max()) + 1, int)
    index[tags.astype(int)] = np.arange(len(tags))
    xyz = xyz.reshape(-1, 3)
    parts = []
    for d, v in gmsh.model.getEntities(3):
        name = gmsh.model.getEntityName(d, v).replace('Shapes/Full Assembly/', '')
        faces = []
        for _, s in gmsh.model.getBoundary([(d, v)], oriented=False):
            types, _, nodes = gmsh.model.mesh.getElements(2, abs(s))
            faces += [index[nd.astype(int).reshape(-1, 3)] for ty, nd in zip(types, nodes) if ty == 2]  # triangles
        if faces:
            faces = outward(faces, xyz)
            parts.append((name, colour(name, gmsh.model.getColor(d, v)), np.concatenate([xyz[f] for f in faces]),
                          np.concatenate([normals(f, xyz) for f in faces])))
    gmsh.finalize()
    return parts


def normals(f, xyz):
    """A face's normals at its triangles' corners (n, 3, 3): at each node the mean of its triangles' normals in that
    face only (weighted by their areas), so that a curved face shades smooth and the edges between faces stay sharp
    (the renderer's own normals average across them: the parts looked moulded in clay)."""
    t = xyz[f]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])         # twice the area along the normal
    acc = np.zeros((len(xyz), 3))
    for k in range(3):
        np.add.at(acc, f[:, k], n)
    out = acc[f]
    length = np.linalg.norm(out, axis=-1, keepdims=True)
    flat = np.repeat((n / np.maximum(np.linalg.norm(n, axis=-1, keepdims=True), 1e-12))[:, None], 3, axis=1)
    return np.where(length > 1e-12, out / np.maximum(length, 1e-12), flat)


def outward(faces, xyz):
    """Each face's triangles (node indices; consistent within a face, not across faces), flipped so that
    neighbouring faces run their shared edges in opposite directions and each connected piece encloses a positive
    volume: the renderer hides triangles seen from behind."""
    links = {}      # face: [(neighbour, same direction on their shared edge)]
    edges = {}      # undirected border edge: [(face, direction)]
    for i, f in enumerate(faces):
        e = np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]])
        key = np.sort(e, axis=1)
        _, inv, count = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        for k in np.flatnonzero(count[inv.ravel()] == 1):     # the face's border
            edges.setdefault(tuple(key[k]), []).append((i, bool(e[k, 0] < e[k, 1])))
    for pairs in edges.values():
        for (a, da), (b, db) in zip(pairs, pairs[1:]):
            links.setdefault(a, []).append((b, da == db))
            links.setdefault(b, []).append((a, da == db))
    flip, out = [None] * len(faces), [None] * len(faces)
    for start in range(len(faces)):
        if flip[start] is not None:
            continue
        flip[start], todo, piece = False, [start], [start]
        while todo:
            a = todo.pop()
            for b, same in links.get(a, ()):
                if flip[b] is None:
                    flip[b] = flip[a] ^ same
                    todo.append(b)
                    piece.append(b)
        tris = {i: faces[i][:, ::-1] if flip[i] else faces[i] for i in piece}
        vol = sum(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() for t in (xyz[f] for f in tris.values()))
        for i, f in tris.items():
            out[i] = f[:, ::-1] if vol < 0 else f
    return out


def robot_frame(parts):
    """STEP (x forward, y left, z up) to the robot frame, with the origin midway between the wheels' centres: a
    function of the points and one of the normals (the turn alone)."""
    centres = [(t.reshape(-1, 3).min(0) + t.reshape(-1, 3).max(0)) / 2 for n, _, t, _ in parts if n.endswith('/Rueda/Rueda')]
    if len(centres) != 2:
        raise SystemExit(f'expected two wheels (Rueda), found {len(centres)}')
    c = np.mean(centres, axis=0)
    turn = lambda t: np.stack([-t[..., 1], t[..., 0], t[..., 2]], axis=-1)    # noqa: E731
    return lambda t: turn(t - c), turn


def body(name):
    if any(k in name for k in WHEEL_PARTS):
        return 'wheel_l' if name.startswith('Gearmotor L') else 'wheel_r'
    return 'chassis'


def write_obj(path, tris, norms):
    """Triangles with their corners' normals (an STL has none: MuJoCo would make its own, across sharp edges)."""
    n = len(tris)
    lines = (['v %.4f %.4f %.4f' % tuple(p) for p in np.asarray(tris).reshape(-1, 3)] +
             ['vn %.4f %.4f %.4f' % tuple(p) for p in np.asarray(norms).reshape(-1, 3)] +
             ['f %d//%d %d//%d %d//%d' % (a, a, a + 1, a + 1, a + 2, a + 2) for a in range(1, 3 * n, 3)])
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def build(step, out):
    parts = tessellate(step)
    rim = next(rgb for n, rgb, *_ in parts if n.split('/')[-1] == RIM)
    parts = [(n, rim if n.split('/')[-1] == FAN else rgb, t, nn) for n, rgb, t, nn in parts]
    to_robot, turn = robot_frame(parts)
    groups = {}
    for name, rgb, t, nn in parts:
        groups.setdefault((body(name), rgb), []).append((name, to_robot(t), turn(nn)))
    os.makedirs(out, exist_ok=True)
    for old in os.listdir(out):
        if old.endswith(('.stl', '.obj')):
            os.remove(os.path.join(out, old))
    manifest = []
    for i, ((b, rgb), items) in enumerate(sorted(groups.items())):
        f = f'{b}_{i:02d}.obj'
        write_obj(os.path.join(out, f), np.concatenate([t for _, t, _ in items]), np.concatenate([nn for *_, nn in items]))
        manifest.append({'file': f, 'body': b, 'rgba': [round(c / 255, 3) for c in rgb] + [1],
                         'parts': sorted({n.split('/')[-1] for n, *_ in items})})
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print(f'{len(parts)} parts, {sum(len(t) for _, _, t, _ in parts)} triangles, {len(manifest)} meshes in {out}')


def ensure(step, out):
    """The meshes' directory, built first if they are missing or older than the STEP or this file."""
    m = os.path.join(out, 'manifest.json')
    if os.path.exists(m) and (not os.path.exists(step) or
                              os.path.getmtime(m) >= max(os.path.getmtime(step), os.path.getmtime(__file__))):
        return out
    if not os.path.exists(step):
        raise SystemExit(f'ZoroBot3: its STEP is missing. In Onshape, export its "Full Assembly" as one STEP file '
                         f'to {step} (README: ZoroBot3 in a maze).')
    try:
        import gmsh     # noqa: F401
    except ImportError:
        raise SystemExit('ZoroBot3: gmsh is needed to make its meshes from its STEP (pip install -e .[cad]).')
    build(step, out)
    return out


if __name__ == '__main__':
    build(*sys.argv[1:3])
