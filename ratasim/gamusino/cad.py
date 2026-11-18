"""Gamusino's meshes from its 3D model (gamusino.glb, exported from Blender: mm, y up, z forward): every part where
the model puts it, merged per moving body and material, in the robot frame (mm: x right, y forward, z up, origin on
the axle between the wheels, found from the two tyres), the PCB's silkscreen as a texture. Also what the physics
needs (geometry.json): wheels, casters, the hull's box.

    python -m ratasim.gamusino.cad
"""
import json
import os
import struct

import numpy as np

from . import config as G

_DT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_N = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}
WHEEL_PARTS = ('Tyre', 'Hub', 'Gear', 'Axle')       # turn with the wheel


def _load(path):
    d = open(path, 'rb').read()
    off, g, b = 12, None, b''
    while off < len(d):
        ln, kind = struct.unpack('<I4s', d[off:off + 8])
        if kind == b'JSON':
            g = json.loads(d[off + 8:off + 8 + ln])
        elif kind == b'BIN\x00':
            b = d[off + 8:off + 8 + ln]
        off += 8 + ln
    return g, b


def _acc(g, b, i):
    a = g['accessors'][i]
    v = g['bufferViews'][a['bufferView']]
    n, dt = _N[a['type']], np.dtype(_DT[a['componentType']])
    start = v.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = v.get('byteStride', 0) or n * dt.itemsize
    raw = np.frombuffer(b, np.uint8, count=stride * (a['count'] - 1) + n * dt.itemsize, offset=start)
    rows = np.lib.stride_tricks.as_strided(raw, (a['count'], n * dt.itemsize), (stride, 1))
    return rows.copy().view(dt).reshape(a['count'], n).astype(float)


def _local(node):
    m = np.eye(4)
    if 'matrix' in node:
        return np.array(node['matrix'], float).reshape(4, 4).T
    x, y, z, w = node.get('rotation', (0, 0, 0, 1))
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, :3] *= np.array(node.get('scale', (1, 1, 1)))[None, :]
    m[:3, 3] = node.get('translation', (0, 0, 0))
    return m


def read(path):
    """(gltf json, binary, [(node name, positions, normals, uvs or None, faces, material index)]) in the file's frame."""
    g, b = _load(path)
    out = []

    def walk(i, parent):
        node = g['nodes'][i]
        m = parent @ _local(node)
        if 'mesh' in node:
            rot = m[:3, :3] / np.linalg.norm(m[:3, :3], axis=0)
            for p in g['meshes'][node['mesh']]['primitives']:
                at = p['attributes']
                pos = _acc(g, b, at['POSITION']) @ m[:3, :3].T + m[:3, 3]
                nrm = _acc(g, b, at['NORMAL']) @ rot.T if 'NORMAL' in at else None
                uv = _acc(g, b, at['TEXCOORD_0']) if 'TEXCOORD_0' in at else None
                idx = _acc(g, b, p['indices']).ravel().astype(int) if 'indices' in p else np.arange(len(pos))
                out.append((node.get('name', ''), pos, nrm, uv, idx.reshape(-1, 3), p.get('material', -1)))
        for c in node.get('children', ()):
            walk(c, m)
    for i in g['scenes'][g.get('scene', 0)]['nodes']:
        walk(i, np.eye(4))
    return g, b, out


def _bbox(prims, name):
    v = np.concatenate([p[1] for p in prims if p[0] == name])
    return v.min(0), v.max(0)


# The PCB's texture is its fabrication panel: beside the main board's middle column it carries the encoder boards
# (AS5040, left) and two IR boards (right), and the model's PCB keeps them as wings. These are cut away (u, v of
# both textures): on the robot they are broken off the panel.
PANELS_UV = ((0.040, 0.390, 0.312, 0.560), (0.684, 0.385, 0.965, 0.565))


def _cut_panels(g, prims):
    """The PCB without the panel's wings, the edges they shared with the main board closed by new side walls."""
    def key(p):
        return tuple(np.round(p, 3))
    out, boxes, kept, cut, top_y = [], [[] for _ in PANELS_UV], {}, set(), None
    for name, pos, nrm, uv, faces, mat in prims:
        face = g['materials'][mat]['name'] if mat >= 0 else ''
        if name != 'PCB' or face not in ('pcbtop', 'pcbbot'):
            out.append((name, pos, nrm, uv, faces, mat))
            continue
        c_uv = uv[faces].mean(axis=1)
        which = np.full(len(faces), -1)
        for i, (u0, v0, u1, v1) in enumerate(PANELS_UV):
            which[(c_uv[:, 0] >= u0) & (c_uv[:, 0] <= u1) & (c_uv[:, 1] >= v0) & (c_uv[:, 1] <= v1)] = i
        for i in range(len(PANELS_UV)):
            boxes[i].append(pos[faces[which == i]][:, :, [0, 2]].reshape(-1, 2))
        if face == 'pcbtop':
            top_y = float(pos[:, 1].max())
            for tri, w in zip(faces, which):
                for a, b in ((0, 1), (1, 2), (2, 0)):
                    e = frozenset((key(pos[tri[a]]), key(pos[tri[b]])))
                    if w >= 0:
                        cut.add(e)
                    else:
                        kept[e] = pos[tri].mean(axis=0)
        out.append((name, pos, nrm, uv, faces[which < 0], mat))
    boxes = [(np.concatenate(bx).min(axis=0) - 0.3, np.concatenate(bx).max(axis=0) + 0.3) for bx in boxes]
    walls, normals = [], []
    for name, pos, nrm, uv, faces, mat in out:
        if name == 'PCB' and g['materials'][mat]['name'] == 'sideb':
            bottom_y = float(pos[:, 1].min())
    for e in set(kept) & cut:                       # the edges the main board shared with a wing: now its border
        p1, p2 = (np.array(p) for p in e)
        n = np.cross(p2 - p1, [0, 1, 0])
        n = n / (np.linalg.norm(n) or 1)
        if np.dot(n, (p1 + p2) / 2 - kept[e]) < 0:   # outward, away from the board
            n, p1, p2 = -n, p2, p1
        q1, q2 = p1.copy(), p2.copy()
        p1[1] = p2[1] = top_y
        q1[1] = q2[1] = bottom_y
        walls += [p1, q1, q2, p1, q2, p2]
        normals += [n] * 6
    result = []
    for name, pos, nrm, uv, faces, mat in out:
        if name == 'PCB' and g['materials'][mat]['name'] == 'sideb':
            c = pos[faces].mean(axis=1)[:, [0, 2]]
            in_wing = np.any([np.all((c >= lo) & (c <= hi), axis=1) for lo, hi in boxes], axis=0)
            faces = faces[~in_wing]
            base = len(pos)
            pos = np.vstack([pos, np.array(walls)])
            nrm = np.vstack([nrm, np.array(normals)]) if nrm is not None else None
            faces = np.vstack([faces, base + np.arange(len(walls)).reshape(-1, 3)])
        result.append((name, pos, nrm, uv, faces, mat))
    return result


def build(glb=G.CAD_GLB, out=G.CAD_MESHES):
    g, b, prims = read(glb)
    prims = _cut_panels(g, prims)
    tyres = sorted((_bbox(prims, n) for n in ('Tyre', 'Tyre.001')), key=lambda bb: bb[0][0])
    c = np.mean([(lo + hi) / 2 for lo, hi in tyres], axis=0)          # the axle's middle (file frame)

    def to_robot(v):        # file: x left, y up, z forward
        return np.stack([-(v[:, 0] - c[0]), v[:, 2] - c[2], v[:, 1] - c[1]], axis=1)

    def body(name, pos):
        if name.split('.')[0] in WHEEL_PARTS:
            return 'wheel_r' if pos[:, 0].mean() < c[0] else 'wheel_l'
        return 'chassis'

    os.makedirs(out, exist_ok=True)
    textures = {}
    for i, im in enumerate(g.get('images', [])):
        v = g['bufferViews'][im['bufferView']]
        f = f'tex_{i}.png'
        open(os.path.join(out, f), 'wb').write(b[v.get('byteOffset', 0):v.get('byteOffset', 0) + v['byteLength']])
        textures[i] = f
    groups = {}
    for name, pos, nrm, uv, faces, mat in prims:
        groups.setdefault((body(name, pos), mat), []).append((to_robot(pos),
                                                              to_robot(nrm + c) if nrm is not None else None, uv, faces))
    manifest = []
    for k, ((bd, mat), items) in enumerate(sorted(groups.items())):
        m = g['materials'][mat] if mat >= 0 else {}
        pbr = m.get('pbrMetallicRoughness', {})
        tex = pbr.get('baseColorTexture', {}).get('index')
        f = f'{bd}_{k:02d}.obj'
        with open(os.path.join(out, f), 'w') as fh:
            base = 1
            for pos, nrm, uv, faces in items:
                fh.writelines(f'v {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in pos)
                if nrm is not None:
                    fh.writelines(f'vn {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in nrm)
                if tex is not None:
                    fh.writelines(f'vt {u:.5f} {1 - w:.5f}\n' for u, w in uv)    # glTF's v runs down the image
                for a, bb, cc in faces + base:
                    if tex is not None:
                        fh.write(f'f {a}/{a}/{a} {bb}/{bb}/{bb} {cc}/{cc}/{cc}\n')
                    else:
                        fh.write(f'f {a}//{a} {bb}//{bb} {cc}//{cc}\n')
                base += len(pos)
        manifest.append({'file': f, 'body': bd, 'material': m.get('name', ''),
                         'rgba': [round(x, 3) for x in pbr.get('baseColorFactor', (1, 1, 1, 1))],
                         'texture': textures[g['textures'][tex]['source']] if tex is not None else None})
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)

    # What the physics needs, mm in the robot frame
    def rb(name):
        lo, hi = (to_robot(np.array(v)[None])[0] for v in _bbox(prims, name))
        return np.minimum(lo, hi), np.maximum(lo, hi)
    (tl, th), (tl2, th2) = rb('Tyre'), rb('Tyre.001')
    pcb_lo, pcb_hi = rb('PCB')
    every = to_robot(np.concatenate([p[1] for p in prims]))
    casters = [((lo + hi) / 2, (hi[2] - lo[2]) / 2) for lo, hi in (rb('CasterF'), rb('CasterB'))]
    geo = {
        'wheel_radius': float((th[2] - tl[2]) / 2),
        'tire_width': float(th[0] - tl[0]),
        'wheels_separation': float(abs((tl[0] + th[0]) / 2 - (tl2[0] + th2[0]) / 2)),
        'axle_height': float(-tl[2]),
        'pcb': [pcb_lo.round(2).tolist(), pcb_hi.round(2).tolist()],
        'robot': [every.min(0).round(2).tolist(), every.max(0).round(2).tolist()],
        'casters': [{'pos': p.round(2).tolist(), 'radius': round(float(r), 2)} for p, r in casters],
    }
    json.dump(geo, open(os.path.join(out, 'geometry.json'), 'w'), indent=1)
    print(f'{len(prims)} primitives, {len(manifest)} meshes in {out}')
    print(json.dumps(geo))
    return out


def ensure():
    m = os.path.join(G.CAD_MESHES, 'manifest.json')
    if not os.path.exists(m) or os.path.getmtime(m) < os.path.getmtime(G.CAD_GLB):
        build()
    return G.CAD_MESHES


if __name__ == '__main__':
    build()
