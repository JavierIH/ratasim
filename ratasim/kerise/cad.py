"""KERISE v2's meshes from its own files (the submodule external/kerise, read at build time): its board from its KiCad
board (kicad/KERISEv2.kicad_pcb: its outline, its two faces drawn as textures from its tracks, zones, pads, vias and
silkscreen, every footprint's part at its package's size) and its motors' mounts from its STL (3dcad/-4digree.stl).
Its Gerber files (gerber/KERISEv2.zip, 2016-12-03) are of an earlier board than its KiCad file's (2016-12-26, the
one in its photos: a round nose): the KiCad file is drawn.

    meshes()        [{'name', 'file', 'body', 'rgba' or 'texture'}], made in build/cad when missing or older than
                    this file, config.py or its sources

Frames: KiCad's in mm, y down; the robot's (config.py) x right, y forward, z up from the floor, with KiCad's
PCB_ORIGIN on the axle. The STL's in inches: two plates side by side (x along, y up from their foot, z through).
"""
import json
import math
import os
import re
import struct

import numpy as np

from ..futura.pcb import _triangulate
from ..miniskybot.cad import write_obj
from . import config as K

PCB = os.path.join(K.KERISE_DIR, 'kicad', 'KERISEv2.kicad_pcb')
STL = os.path.join(K.KERISE_DIR, '3dcad', '-4digree.stl')
OUT = os.path.join(K.BUILD, 'cad')
PX_MM = 20                      # the textures' resolution
TAB_Y = 158.5                   # KiCad y: its board ends at 158; past it, joined by two bridges, a USB tab for its
#                                 charger, cut off (not on the robot in its photos)
# Its board's colours as its photos show it: a white solder mask, black silkscreen
MASK, COPPER, TIN, SILK, HOLE = (236, 236, 230), (222, 220, 205), (200, 198, 186), (25, 25, 25), (10, 10, 10)
EDGE = (0.85, 0.83, 0.70, 1)
PLASTIC = (0.94, 0.94, 0.92, 1)         # its printed mounts, white


# ---- KiCad's s-expressions ---------------------------------------------------------------------------------------
def _sexp(text):
    tokens = re.findall(r'"(?:[^"\\]|\\.)*"|\(|\)|[^\s()"]+', text)
    stack, cur = [], []
    for t in tokens:
        if t == '(':
            stack.append(cur)
            cur = []
        elif t == ')':
            done, cur = cur, stack.pop()
            cur.append(done)
        else:
            cur.append(t.strip('"'))
    return cur[0]


def _kids(node, name):
    return [k for k in node[1:] if isinstance(k, list) and k and k[0] == name]


def _kid(node, name):
    k = _kids(node, name)
    return k[0] if k else None


def _xy(node):
    return float(node[1]), float(node[2])


_BOARD = None


def board():
    global _BOARD
    if _BOARD is None:
        with open(PCB, encoding='utf-8') as f:
            _BOARD = _sexp(f.read())
    return _BOARD


def to_robot(x, y):
    """A KiCad point (mm) in the robot's frame (x, y)."""
    return x - K.PCB_ORIGIN[0], K.PCB_ORIGIN[1] - y


def _rot(x, y, deg):
    """A footprint's point rotated as KiCad does (its angles counter-clockwise on screen, y down)."""
    a = math.radians(deg)
    return x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)


def _arc(c, start, deg, step=5.0):
    """Points of an arc from start around c (KiCad: positive angles clockwise on screen)."""
    n = max(2, int(abs(deg) / step) + 1)
    out = []
    for i in range(n + 1):
        a = math.radians(deg * i / n)
        dx, dy = start[0] - c[0], start[1] - c[1]
        out.append((c[0] + dx * math.cos(a) - dy * math.sin(a), c[1] + dx * math.sin(a) + dy * math.cos(a)))
    return out


# ---- its outline -----------------------------------------------------------------------------------------------
def outline():
    """Its board's outline (KiCad mm), counter-clockwise in the robot's frame: its Edge.Cuts' lines and arcs chained,
    the bridges to its tab closed straight."""
    pieces = []
    for g in board()[1:]:
        if not isinstance(g, list) or g[0] not in ('gr_line', 'gr_arc') or _kid(g, 'layer')[1] != 'Edge.Cuts':
            continue
        if g[0] == 'gr_line':
            pts = [_xy(_kid(g, 'start')), _xy(_kid(g, 'end'))]
        else:
            pts = _arc(_xy(_kid(g, 'start')), _xy(_kid(g, 'end')), float(_kid(g, 'angle')[1]))
        if max(p[1] for p in pts) <= TAB_Y - 0.4:
            pieces.append(pts)
    chain = pieces.pop(0)
    while pieces:
        end = chain[-1]
        best = min(range(len(pieces)), key=lambda i: min(math.dist(end, pieces[i][0]), math.dist(end, pieces[i][-1])))
        p = pieces.pop(best)
        if math.dist(end, p[-1]) < math.dist(end, p[0]):
            p = p[::-1]
        assert math.dist(end, p[0]) < 2.5, f'its outline has a gap at {end}'
        chain += p[1:] if math.dist(end, p[0]) < 1e-6 else p
    poly = []
    for p in chain:
        if not poly or math.dist(p, poly[-1]) > 1e-4:
            poly.append(p)
    if math.dist(poly[0], poly[-1]) < 1e-4:
        poly.pop()
    rob = [to_robot(*p) for p in poly]
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(rob, rob[1:] + rob[:1]))
    return poly if area > 0 else poly[::-1]


# ---- its parts -------------------------------------------------------------------------------------------------
def modules():
    """Its footprints: (library name, reference, (x, y) KiCad mm, rotation deg, pads), each pad
    (shape, (x, y) KiCad mm, rotation deg, (w, h), drill or 0, layers)."""
    out = []
    for m in _kids(board(), 'module'):
        at = _kid(m, 'at')
        x, y = _xy(at)
        rot = float(at[3]) if len(at) > 3 else 0.0
        ref = next((t[2] for t in _kids(m, 'fp_text') if t[1] == 'reference'), '')
        pads = []
        for p in _kids(m, 'pad'):
            pa = _kid(p, 'at')
            px, py = _rot(*_xy(pa), rot)
            drill = _kid(p, 'drill')
            d = float(drill[1]) if drill and len(drill) > 1 and drill[1] != 'oval' else 0.0
            pads.append((p[3], (x + px, y + py), float(pa[3]) if len(pa) > 3 else rot,
                         (float(_kid(p, 'size')[1]), float(_kid(p, 'size')[2])), d, _kid(p, 'layers')[1:]))
        out.append((m[1], ref, (x, y), rot, pads))
    return out


# Packages: (width along the footprint's x, depth along its y, height mm, colour); the IR emitters and receivers
# and the motors' pads are drawn apart, the rest are on the tab or are its logo.
BLACK, BEIGE, CHIP = (0.06, 0.06, 0.07, 1), (0.92, 0.88, 0.78, 1), (0.10, 0.10, 0.11, 1)
CERAMIC, METAL, LED = (0.78, 0.66, 0.48, 1), (0.75, 0.75, 0.76, 1), (0.95, 0.95, 0.92, 1)
PACKAGES = {
    'C_0402': (1.0, 0.5, 0.5, CERAMIC), 'C_0603': (1.6, 0.8, 0.8, CERAMIC), 'C_0805': (2.0, 1.25, 1.0, CERAMIC),
    'R_0402': (1.0, 0.5, 0.35, BLACK), 'R_0603': (1.6, 0.8, 0.45, BLACK), 'R_0805': (2.0, 1.25, 0.5, BLACK),
    'LED_0603': (1.6, 0.8, 0.6, LED),
    'SOT-23': (2.9, 1.3, 1.0, CHIP), 'SOT-23-5': (2.9, 1.6, 1.1, CHIP),
    'SOIC-8': (3.9, 4.9, 1.5, CHIP), 'TSSOP-14': (4.4, 5.0, 1.1, CHIP),
    'LQFP-64': (10.0, 10.0, 1.4, CHIP), 'QFN-24': (3.0, 3.0, 0.9, CHIP), 'DRV8835': (2.0, 2.0, 0.8, CHIP),
    'LXDC55': (2.5, 2.0, 1.0, CHIP), 'SMT-0440': (4.0, 4.0, 2.5, BLACK),
    'SW_PUSH': (4.2, 3.2, 1.6, METAL), 'Slide_SW': (3.0, 7.5, 1.5, METAL),
    'JST_SH_BM04B': (6.0, 4.25, 2.9, BEIGE), 'JST_SH_BM05B': (7.0, 4.25, 2.9, BEIGE),
    'Hirose_DF13-02P': (5.5, 3.4, 5.4, BEIGE),
}
IR_PARTS = ('LED-5MM_Angle', 'Photo-5MM_Angle')


def _package(name):
    for key, v in PACKAGES.items():
        if key in name:
            return v
    return None


def _box(c, half, rot):
    """A box's triangles (KiCad x, y; z up from the board's top): centre, half sizes (footprint frame), rotation."""
    corners = np.array([[sx * half[0], sy * half[1]] for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
    pts = np.array([[c[0] + p[0], c[1] + p[1]] for p in (_rot(*q, rot) for q in corners)])
    lo, hi = c[2] - half[2], c[2] + half[2]
    v = np.array([[*p, lo] for p in pts] + [[*p, hi] for p in pts])
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7)]
    for i in range(4):
        j = (i + 1) % 4
        faces += [(i, j, 4 + j), (i, 4 + j, 4 + i)]
    return v[np.array(faces)]


def _cylinder(c, axis_deg, r, length, n=20):
    """A cylinder's triangles lying on the board along a footprint's direction (KiCad frame, z up)."""
    a = math.radians(axis_deg)
    ax = np.array([math.cos(a), math.sin(a), 0.0])
    u, w = np.array([-ax[1], ax[0], 0.0]), np.array([0.0, 0.0, 1.0])
    c = np.array(c, float)
    ring = [r * (math.cos(t) * u + math.sin(t) * w) for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    a0, a1 = c - ax * length / 2, c + ax * length / 2
    tris = []
    for i in range(n):
        p, q = ring[i], ring[(i + 1) % n]
        tris += [(a0 + p, a0 + q, a1 + q), (a0 + p, a1 + q, a1 + p), (a0, a0 + q, a0 + p), (a1, a1 + p, a1 + q)]
    return np.array(tris)


def ir_parts():
    """Its IR emitters (VSLY5850, clear, in black sleeves) over its photodiodes (SFH213FA, black), each 5 mm lying on
    the board, from its pins 8.6 mm towards its footprint's -y: {colour: triangles} (KiCad frame)."""
    out = {}
    for name, _ref, (x, y), rot, _pads in modules():
        if not any(p in name for p in IR_PARTS):
            continue
        dx, dy = _rot(0.0, -1.0, rot)
        ang = math.degrees(math.atan2(dy, dx))
        z = 2.5 if 'Photo' in name else 7.5
        mid = (x + dx * 4.3, y + dy * 4.3, z)
        out.setdefault(BLACK, []).append(_cylinder(mid, ang, 2.6, 7.0 if 'Photo' not in name else 8.6))
        if 'Photo' not in name:
            tip = (x + dx * 8.4, y + dy * 8.4, z)
            out.setdefault((0.85, 0.9, 0.95, 1), []).append(_cylinder(tip, ang, 2.5, 1.6))
    return out


def parts():
    """Every footprint's part on the board: {colour: triangles} (KiCad frame, z up from the board's top)."""
    groups = ir_parts()
    for name, _ref, (x, y), rot, pads in modules():
        pkg = _package(name)
        if not pkg or y > TAB_Y:
            continue
        w, d, h, colour = pkg
        groups.setdefault(colour, []).append(_box((x, y, h / 2), (w / 2, d / 2, h / 2), rot))
    return {k: np.concatenate(v) for k, v in groups.items()}


# ---- its faces -------------------------------------------------------------------------------------------------
def frame():
    xs, ys = zip(*outline())
    return min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5


def _layer_items(layer):
    """What its board draws on a layer: ('seg', a, b, width), ('poly', points), ('pad', ...), ('circle', c, r, w)."""
    out = []
    b = board()
    for s in _kids(b, 'segment'):
        if _kid(s, 'layer')[1] == layer:
            out.append(('seg', _xy(_kid(s, 'start')), _xy(_kid(s, 'end')), float(_kid(s, 'width')[1])))
    for z in _kids(b, 'zone'):
        if _kid(z, 'layer')[1] == layer:
            for fp in _kids(z, 'filled_polygon'):
                out.append(('poly', [_xy(p) for p in _kids(_kid(fp, 'pts'), 'xy')]))
    if layer.endswith('Cu'):
        for v in _kids(b, 'via'):
            out.append(('circle', _xy(_kid(v, 'at')), float(_kid(v, 'size')[1]) / 2, 0))
    if layer.endswith('SilkS'):
        for g in b[1:]:
            if isinstance(g, list) and g[0] in ('gr_line', 'gr_circle') and _kid(g, 'layer')[1] == layer:
                a, e = _xy(_kid(g, 'center' if g[0] == 'gr_circle' else 'start')), _xy(_kid(g, 'end'))
                w = float(_kid(g, 'width')[1])
                out.append(('seg', a, e, w) if g[0] == 'gr_line' else ('circle', a, math.dist(a, e), w))
        for m in _kids(b, 'module'):
            at = _kid(m, 'at')
            mx, my = _xy(at)
            rot = float(at[3]) if len(at) > 3 else 0.0
            for g in m[1:]:
                if not isinstance(g, list) or g[0] not in ('fp_line', 'fp_circle', 'fp_arc') or _kid(g, 'layer')[1] != layer:
                    continue
                a, e = (_rot(*_xy(_kid(g, k)), rot) for k in ('center' if g[0] == 'fp_circle' else 'start', 'end'))
                a, e = (mx + a[0], my + a[1]), (mx + e[0], my + e[1])
                w = float(_kid(g, 'width')[1])
                if g[0] == 'fp_line':
                    out.append(('seg', a, e, w))
                elif g[0] == 'fp_circle':
                    out.append(('circle', a, math.dist(a, e), w))
                else:
                    pts = _arc(a, e, float(_kid(g, 'angle')[1]))
                    out += [('seg', p, q, w) for p, q in zip(pts, pts[1:])]
    return out


def _pads(side):
    """Its pads on a side ('F' or 'B'): (shape, centre, rotation, size)."""
    out = []
    for _name, _ref, _at, _rot_, pads in modules():
        for shape, c, rot, size, _d, layers in pads:
            if any(lay in (f'{side}.Cu', '*.Cu') for lay in layers):
                out.append((shape, c, rot, size))
    return out


def _drills():
    out = [(_xy(_kid(v, 'at')), 0.3) for v in _kids(board(), 'via')]
    for _name, _ref, _at, _rot_, pads in modules():
        out += [(c, d) for _s, c, _r, _size, d, _l in pads if d]
    return out


def textures(out):
    """kerise_top.png and kerise_bottom.png: each face (both seen from above) as its layers make it: copper under
    its white mask, its pads tinned, its silkscreen, its holes."""
    from PIL import Image, ImageDraw
    x0, y0, x1, y1 = frame()
    size = (int(round((x1 - x0) * PX_MM)), int(round((y1 - y0) * PX_MM)))

    def px(p):
        return (p[0] - x0) * PX_MM, (p[1] - y0) * PX_MM
    for face, side in (('top', 'F'), ('bottom', 'B')):
        img = Image.new('RGB', size, MASK)
        d = ImageDraw.Draw(img)

        def draw(items, colour):
            for it in items:
                if it[0] == 'seg':
                    r = max(it[3] * PX_MM / 2, 0.6)
                    q = [px(it[1]), px(it[2])]
                    d.line(q, fill=colour, width=max(1, int(round(2 * r))))
                    for cx, cy in q:
                        d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=colour)
                elif it[0] == 'poly':
                    d.polygon([px(p) for p in it[1]], fill=colour)
                elif it[0] == 'circle':
                    (cx, cy), r = px(it[1]), it[2] * PX_MM
                    if it[3]:
                        d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=colour, width=max(1, int(it[3] * PX_MM)))
                    else:
                        d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=colour)
        draw(_layer_items(f'{side}.Cu'), COPPER)
        for shape, c, rot, (w, h) in _pads(side):
            cx, cy = px(c)
            if shape == 'circle':
                r = w * PX_MM / 2
                d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=TIN)
                continue
            if shape == 'oval':
                r = min(w, h) / 2
                a, b = _rot((w / 2 - r) if w > h else 0, (h / 2 - r) if h >= w else 0, rot)
                draw([('seg', (c[0] - a, c[1] - b), (c[0] + a, c[1] + b), 2 * r)], TIN)
                continue
            corners = [_rot(sx * w / 2, sy * h / 2, rot) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
            d.polygon([px((c[0] + a, c[1] + b)) for a, b in corners], fill=TIN)
        draw(_layer_items(f'{side}.SilkS'), SILK)
        for c, dia in _drills():
            (cx, cy), r = px(c), dia * PX_MM / 2
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=HOLE)
        if face == 'bottom':
            img = img.transpose(Image.FLIP_LEFT_RIGHT)      # seen from below: its texture's u runs the other way
        img.save(os.path.join(out, f'kerise_{face}.png'))


def board_meshes(out):
    """kerise_top.obj, kerise_bottom.obj (UVs on the textures) and kerise_side.obj, robot frame (mm)."""
    poly = [to_robot(*p) for p in outline()]
    tris = _triangulate(poly)
    x0, y0, x1, y1 = frame()
    top, bottom = K.BOARD_Z_MM, K.BOARD_Z_MM - K.PCB_MM
    for face, z, flip in (('top', top, False), ('bottom', bottom, True)):
        with open(os.path.join(out, f'kerise_{face}.obj'), 'w') as f:
            for x, y in poly:
                f.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
            for x, y in poly:
                kx, ky = x + K.PCB_ORIGIN[0], K.PCB_ORIGIN[1] - y
                u = (kx - x0) / (x1 - x0)
                f.write(f'vt {1 - u if flip else u:.5f} {1 - (ky - y0) / (y1 - y0):.5f}\n')
            for t in tris:
                a, b, c = (i + 1 for i in (t[::-1] if flip else t))
                f.write(f'f {a}/{a} {b}/{b} {c}/{c}\n')
    with open(os.path.join(out, 'kerise_side.obj'), 'w') as f:
        n = len(poly)
        for x, y in poly:
            f.write(f'v {x:.4f} {y:.4f} {top:.4f}\nv {x:.4f} {y:.4f} {bottom:.4f}\n')
        for i in range(n):
            j = (i + 1) % n
            t0, b0, t1, b1 = 2 * i + 1, 2 * i + 2, 2 * j + 1, 2 * j + 2
            f.write(f'f {t0} {b0} {b1}\nf {t0} {b1} {t1}\n')


# ---- its motors' mounts ----------------------------------------------------------------------------------------
def _stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack('<I', data[80:84])[0]
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return rec['v'].astype(np.float64) * 25.4


def _hole(verts, guess, r_max):
    """A hole's centre (x, y) and radius: the plate's vertices around it, on its wall."""
    d = np.hypot(verts[:, 0] - guess[0], verts[:, 1] - guess[1])
    ring = verts[d < r_max]
    c = ring[:, :2].mean(axis=0)
    return c, float(np.hypot(*(ring[:, :2] - c).T).mean())


def mounts():
    """Its two motors' mounts, standing on its board, each where its wheel's axle and its motor go through it:
    {colour: triangles} (robot frame). Each plate of the STL: its foot (its lowest y) on the board, its wheel's hole
    on the axle, its motor's 12 mm ahead (the right one) or behind (the left one), between 8 and 12 mm from the
    middle. Both are the STL's plate placed mirrored: their triangles' order reversed."""
    tris = _stl(STL)
    plate = tris[tris[:, :, 1].mean(axis=1) > 0]            # the STL's two plates are the same: one of them
    v = plate.reshape(-1, 3)
    foot = v[:, 1].min()
    axle, ra = _hole(v, (21.5, foot + 10.5), 2.2)
    motor, rm = _hole(v, (9.5, foot + 11.0), 3.3)
    assert abs(axle[1] - foot - K.MOUNT_AXLE_MM) < 0.3 and abs(motor[1] - foot - K.MOUNT_MOTOR_MM) < 0.3, (axle, motor, foot)
    assert abs(axle[0] - motor[0] - K.MOTOR_OFFSET_MM) < 0.3, (axle, motor)
    out = []
    for s in (-1, 1):                                       # left, right
        p = plate.copy()
        x = s * (8.0 + p[:, :, 2] - v[:, 2].min())
        y = -s * (p[:, :, 0] - axle[0])
        z = K.BOARD_Z_MM + p[:, :, 1] - foot
        out.append(np.stack([x, y, z], axis=2)[:, ::-1])
    return {PLASTIC: np.concatenate(out)}


# ---- all ---------------------------------------------------------------------------------------------------------
def _sources():
    return [os.path.abspath(__file__), os.path.join(K.HERE, 'config.py'), PCB, STL]


def build():
    os.makedirs(OUT, exist_ok=True)
    textures(OUT)
    board_meshes(OUT)
    entries = [{'name': f'board_{f}', 'file': os.path.join(OUT, f'kerise_{f}.obj'), 'body': 'chassis',
                'texture': os.path.join(OUT, f'kerise_{f}.png')} for f in ('top', 'bottom')]
    entries.append({'name': 'board_side', 'file': os.path.join(OUT, 'kerise_side.obj'), 'body': 'chassis',
                    'rgba': EDGE})
    for i, (colour, t) in enumerate(sorted(parts().items())):
        t = t.copy()
        t[:, :, 0] = t[:, :, 0] - K.PCB_ORIGIN[0]
        t[:, :, 1] = K.PCB_ORIGIN[1] - t[:, :, 1]          # KiCad's y down: a mirror, its triangles reversed
        t[:, :, 2] += K.BOARD_Z_MM
        path = os.path.join(OUT, f'kerise_part_{i}.obj')
        write_obj(path, t[:, ::-1], 40)
        entries.append({'name': f'part_{i}', 'file': path, 'body': 'chassis', 'rgba': colour})
    for i, (colour, t) in enumerate(mounts().items()):
        path = os.path.join(OUT, f'kerise_mount_{i}.obj')
        write_obj(path, t, 40)
        entries.append({'name': f'mount_{i}', 'file': path, 'body': 'chassis', 'rgba': colour})
    with open(os.path.join(OUT, 'manifest.json'), 'w') as f:
        json.dump(entries, f, indent=1)
    return entries


def meshes():
    manifest = os.path.join(OUT, 'manifest.json')
    if not os.path.exists(manifest) or os.path.getmtime(manifest) < max(os.path.getmtime(p) for p in _sources()):
        return build()
    with open(manifest) as f:
        return json.load(f)
