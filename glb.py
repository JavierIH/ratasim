"""A minimal GLB (binary glTF 2.0) reader: every triangle of the default scene in world coordinates, with its
material's base colour. Enough for kicad-cli's exports; no textures, no skins, no sparse accessors."""
import json
import struct

import numpy as np

_DTYPES = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_SIZES = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def _accessor(g, binary, i):
    a = g['accessors'][i]
    v = g['bufferViews'][a['bufferView']]
    n, dt = _SIZES[a['type']], np.dtype(_DTYPES[a['componentType']])
    start = v.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = v.get('byteStride', 0) or n * dt.itemsize
    raw = np.frombuffer(binary, np.uint8, count=stride * (a['count'] - 1) + n * dt.itemsize, offset=start)
    rows = np.lib.stride_tricks.as_strided(raw, (a['count'], n * dt.itemsize), (stride, 1))
    return rows.copy().view(dt).reshape(a['count'], n)


def _local(node):
    if 'matrix' in node:
        return np.array(node['matrix'], float).reshape(4, 4).T
    m = np.eye(4)
    x, y, z, w = node.get('rotation', (0, 0, 0, 1))
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, :3] *= np.array(node.get('scale', (1, 1, 1)))[None, :]
    m[:3, 3] = node.get('translation', (0, 0, 0))
    return m


def read(path):
    """[(triangles (n, 3, 3) float, rgba (4,), mesh name)] in the file's units and axes."""
    data = open(path, 'rb').read()
    magic, _, _ = struct.unpack('<4sII', data[:12])
    assert magic == b'glTF', 'not a GLB file'
    off, g, binary = 12, None, b''
    while off < len(data):
        length, kind = struct.unpack('<I4s', data[off:off + 8])
        chunk = data[off + 8:off + 8 + length]
        if kind == b'JSON':
            g = json.loads(chunk)
        elif kind == b'BIN\x00':
            binary = chunk
        off += 8 + length
    out, placed = [], {}

    def walk(i, parent):
        node = g['nodes'][i]
        m = parent @ _local(node)
        if 'mesh' in node:
            placed[node['mesh']] = m
        for c in node.get('children', ()):
            walk(c, m)
    for i in g['scenes'][g.get('scene', 0)]['nodes']:
        walk(i, np.eye(4))
    for k, mesh in enumerate(g['meshes']):        # every mesh (kicad-cli 9 can point a node past the list)
        m = placed.get(k, np.eye(4))
        for p in mesh['primitives']:
            if p.get('mode', 4) != 4:
                continue
            pos = _accessor(g, binary, p['attributes']['POSITION']).astype(float)
            pos = pos @ m[:3, :3].T + m[:3, 3]
            idx = (_accessor(g, binary, p['indices']).ravel() if 'indices' in p else np.arange(len(pos)))
            mat = g.get('materials', [])[p['material']] if 'material' in p and p['material'] < len(g.get('materials', [])) else {}
            rgba = np.array(mat.get('pbrMetallicRoughness', {}).get('baseColorFactor', (0.8, 0.8, 0.8, 1)))
            out.append((pos[idx.reshape(-1, 3)], rgba, mesh.get('name', '')))
    return out
