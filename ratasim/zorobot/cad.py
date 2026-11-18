"""ZoroBot3's meshes from its Onshape assembly (the "Full Assembly" exported as one STEP file): every part where the
designers placed it, tessellated with gmsh and merged per moving body and colour, in the robot frame (mm: x right,
y forward, z up, origin on the axle between the wheels, found from the two wheels).

    python cad.py STEP OUT_DIR      (any Python with gmsh and numpy; it imports nothing else)

The simulator calls ensure(), which needs gmsh only when the meshes are missing or older than the STEP.

Writes OUT_DIR/<body>_<n>.stl and OUT_DIR/manifest.json: [{"file", "body", "rgba", "parts"}], body in
chassis, wheel_l, wheel_r.
"""
import json
import os
import struct
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
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 10)
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
            parts.append((name, colour(name, gmsh.model.getColor(d, v)), np.concatenate(outward(faces, xyz))))
    gmsh.finalize()
    return parts


def outward(faces, xyz):
    """Each face's triangles (node indices; consistent within a face, not across faces) as coordinates, flipped so
    that neighbouring faces run their shared edges in opposite directions and each connected piece encloses a
    positive volume: the renderer hides triangles seen from behind."""
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
        tris = {i: xyz[faces[i][:, ::-1] if flip[i] else faces[i]] for i in piece}
        vol = sum(np.einsum('ij,ij->i', t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() for t in tris.values())
        for i, t in tris.items():
            out[i] = t[:, ::-1] if vol < 0 else t
    return out


def robot_frame(parts):
    """STEP (x forward, y left, z up) to the robot frame, with the origin midway between the wheels' centres."""
    centres = [(t.reshape(-1, 3).min(0) + t.reshape(-1, 3).max(0)) / 2 for n, _, t in parts if n.endswith('/Rueda/Rueda')]
    if len(centres) != 2:
        raise SystemExit(f'expected two wheels (Rueda), found {len(centres)}')
    c = np.mean(centres, axis=0)
    return lambda t: np.stack([-(t[..., 1] - c[1]), t[..., 0] - c[0], t[..., 2] - c[2]], axis=-1)


def body(name):
    if any(k in name for k in WHEEL_PARTS):
        return 'wheel_l' if name.startswith('Gearmotor L') else 'wheel_r'
    return 'chassis'


def write_stl(path, tris):
    tris = np.asarray(tris, np.float32).reshape(-1, 3, 3)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)) + rec.tobytes())


def build(step, out):
    parts = tessellate(step)
    to_robot = robot_frame(parts)
    groups = {}
    for name, rgb, t in parts:
        groups.setdefault((body(name), rgb), []).append((name, to_robot(t)))
    os.makedirs(out, exist_ok=True)
    manifest = []
    for i, ((b, rgb), items) in enumerate(sorted(groups.items())):
        f = f'{b}_{i:02d}.stl'
        write_stl(os.path.join(out, f), np.concatenate([t for _, t in items]))
        manifest.append({'file': f, 'body': b, 'rgba': [round(c / 255, 3) for c in rgb] + [1],
                         'parts': sorted({n.split('/')[-1] for n, _ in items})})
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print(f'{len(parts)} parts, {sum(len(t) for *_, t in parts)} triangles, {len(manifest)} meshes in {out}')


def ensure(step, out):
    """The meshes' directory, built first if they are missing or older than the STEP."""
    m = os.path.join(out, 'manifest.json')
    if os.path.exists(m) and (not os.path.exists(step) or os.path.getmtime(m) >= os.path.getmtime(step)):
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
