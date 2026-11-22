"""KERISE v1's meshes from its own files (the submodule external/kerise-v1, read at build time): its board from its
KiCad board (kicad/Mouse-01.kicad_pcb: its outline, its two faces drawn as textures from its tracks, pads, vias and
silkscreen, every footprint's part at its package's size), its motors' mounts and its wheels (a spur gear and a rim
printed in one piece) from its STL files (3dprint/mounter-4set.stl, 3dprint/wheel-2set.stl); its tyres, its ribbon
cables and its battery's wires drawn here.

    meshes()        [{'name', 'file', 'body', 'rgba' or 'texture'}], made in build/cad when missing or older than
                    this file, config.py or its sources

Frames: KiCad's in mm, y down; the robot's (config.py) x right, y forward, z up from the floor, with KiCad's
PCB_ORIGIN on the axle. The STL files' in mm: their parts side by side, each lying on its side.
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

PCB = os.path.join(K.KERISE_DIR, 'kicad', 'Mouse-01.kicad_pcb')     # its board as at its contest (2016-11-02)
STL = os.path.join(K.KERISE_DIR, '3dprint', 'mounter-4set.stl')
WHEEL_STL = os.path.join(K.KERISE_DIR, '3dprint', 'wheel-2set.stl')
OUT = os.path.join(K.BUILD, 'cad')
PX_MM = 20                      # the textures' resolution
TAB_Y = 1e9                     # KiCad y past which a board's tab would be cut off (its board has none)
# Its board's colours as its photo shows it: a black solder mask, white silkscreen
MASK, COPPER, TIN, SILK, HOLE = (22, 22, 24), (38, 38, 40), (200, 198, 186), (232, 232, 228), (6, 6, 6)
EDGE = (0.14, 0.14, 0.13, 1)          # its edges: dark
CORNER_MM = 0.4                         # its outline's corners rounded (its router's)
PLASTIC = (0.07, 0.07, 0.075, 1)        # its printed mounts, black


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
LENS = (0.88, 0.9, 0.92, 0.4)          # its emitters' clear epoxy
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
    'Hirose_DF13-02P': (5.5, 3.4, 5.4, BEIGE), 'JST_PH_B2B': (6.0, 4.5, 6.0, (0.93, 0.92, 0.88, 1)),
    'TO-252': (6.5, 6.1, 2.3, CHIP), 'SKRPACE010': (4.2, 3.2, 1.8, METAL),
    'Pin_Header_Straight_1x03': (7.6, 2.5, 2.5, BLACK),
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


def _cylinder(c, axis_deg, r, length, n=40):
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
    """Its IR emitters (VSLY5850, clear) and photodiodes (SFH213FA, in black sleeves) side by side, each 5 mm lying on
    the board, from its pins towards its footprint's -y: {colour: triangles} (KiCad frame, z up from the board)."""
    out = {}
    for name, _ref, (x, y), rot, _pads in modules():
        if not any(p in name for p in IR_PARTS):
            continue
        dx, dy = _rot(0.0, -1.0, rot)
        ang = math.degrees(math.atan2(dy, dx))
        if 'Photo' in name:
            out.setdefault(BLACK, []).append(_cylinder((x + dx * 4.6, y + dy * 4.6, 2.6), ang, 2.6, 8.6))
        else:                                       # its clear body and its dome
            out.setdefault(LENS, []).append(_cylinder((x + dx * 4.3, y + dy * 4.3, 2.5), ang, 2.5, 7.0))
            out.setdefault(LENS, []).append(_dome((x + dx * 7.8, y + dy * 7.8, 2.5), ang, 2.5))
            out.setdefault(LENS, []).append(_cylinder((x + dx * 0.9, y + dy * 0.9, 2.5), ang, 2.9, 1.0))   # its rim
    return out


def _dome(c, axis_deg, r, n=40, m=10):
    """A half sphere's triangles (KiCad frame, z up), its flat face at c, bulging along a footprint's direction."""
    a = math.radians(axis_deg)
    ax = np.array([math.cos(a), math.sin(a), 0.0])
    u, w = np.array([-ax[1], ax[0], 0.0]), np.array([0.0, 0.0, 1.0])
    c = np.array(c, float)

    def p(i, j):
        phi, th = math.pi / 2 * j / m, 2 * math.pi * i / n
        return c + r * (math.cos(phi) * (math.cos(th) * u + math.sin(th) * w) + math.sin(phi) * ax)
    tris = []
    for i in range(n):
        for j in range(m):
            tris += [(p(i, j), p(i + 1, j), p(i + 1, j + 1)), (p(i, j), p(i + 1, j + 1), p(i, j + 1))]
    return np.array(tris)


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
    poly = rounded([to_robot(*p) for p in outline()], CORNER_MM)
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


def rounded(poly, r, min_turn=20.0, n=6):
    """A polygon with its corners (turning over min_turn degrees) rounded: from r before each to r after it."""
    out = []
    k = len(poly)
    for i in range(k):
        a, b, c = np.array(poly[i - 1]), np.array(poly[i]), np.array(poly[(i + 1) % k])
        u, v = b - a, c - b
        la, lb = np.linalg.norm(u), np.linalg.norm(v)
        turn = math.degrees(math.acos(np.clip(u @ v / max(la * lb, 1e-12), -1, 1)))
        if turn < min_turn or la < 2 * r or lb < 2 * r:
            out.append(tuple(b))
            continue
        p, q = b - u / la * r, b + v / lb * r
        for j in range(n + 1):                      # a quadratic curve with the corner as its control point
            t = j / n
            out.append(tuple((1 - t) ** 2 * p + 2 * (1 - t) * t * b + t ** 2 * q))
    return out


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
    {colour: triangles} (robot frame). A plate of the STL (four, 28 by 17.5 by 4 mm, lying): its foot (its lowest y)
    on the board, along y, its axle's hole on the axle, its motor's 11.5 mm ahead, 4 mm thick across x with its
    middle MOUNT_X_MM from the robot's, its motor's recess facing in; the right one the left one mirrored."""
    tris = _stl(STL) / 25.4                                 # _stl reads inches: this one is in mm
    c = tris.mean(axis=1)
    plate = tris[(c[:, 0] < 28.5) & (c[:, 1] > 0)]          # one of its four
    v = plate.reshape(-1, 3)
    foot = v[:, 1].min()
    axle, ra = _hole(v, (3.0, foot + 9.1), 1.8)
    motor, rm = _hole(v, (14.5, foot + 8.5), 3.4)
    assert abs(axle[1] - foot - K.MOUNT_AXLE_MM) < 0.2 and abs(motor[1] - foot - K.MOUNT_MOTOR_MM) < 0.2, (axle, motor, foot)
    assert abs(motor[0] - axle[0] - K.MOTOR_OFFSET_MM) < 0.2, (axle, motor)
    out = []
    for s in (-1, 1):                                       # left, right
        x = s * (K.MOUNT_X_MM + 2.0 - plate[:, :, 2])      # its z 4 (the recess's face) inwards
        y = plate[:, :, 0] - axle[0]
        z = K.BOARD_Z_MM + plate[:, :, 1] - foot
        t = np.stack([x, y, z], axis=2)
        out.append(t[:, ::-1] if s > 0 else t)              # the right one a mirror: its triangles reversed
    return {PLASTIC: np.concatenate(out)}


# ---- its wheels, tyres and cables ---------------------------------------------------------------------------------
GEAR_TEETH = 36                 # its wheels' spur gears (KERISE v1's STL); its motors' pinions 9 (4:1)
RIM_R_MM, TYRE_Z_MM = 10.0, (1.0, 8.5)         # the STL's rim under its tyre and the tyre's place along its axis
WHEEL_PLASTIC = (0.93, 0.93, 0.91, 1)
TYRE = (0.07, 0.07, 0.075, 1)
RIBBON, RIBBON_STRIPE, IDC = (0.72, 0.72, 0.69, 1), (0.75, 0.12, 0.12, 1), (0.07, 0.07, 0.075, 1)
RED, BLACK_WIRE = (0.78, 0.08, 0.06, 1), (0.05, 0.05, 0.05, 1)


def _stl_mm(path):
    return _stl(path) / 25.4            # _stl reads inches


def wheel(side):
    """One of the STL's four wheels in its wheel's frame (mm): its axle along x, x 0 its tyre's middle, its gear
    inwards (-x for the right one); the left one the right one turned half a turn about z."""
    t = _stl_mm(WHEEL_STL)
    c = t.mean(axis=1)
    one = t[(np.abs(c[:, 0]) < 11.0) & (np.abs(c[:, 1]) < 11.0)]
    z = one[:, :, 2] - sum(TYRE_Z_MM) / 2
    w = np.stack([z, one[:, :, 0], one[:, :, 1]], axis=2)
    return w if side == 'r' else w * (-1, -1, 1)


def _orient(tris, centres):
    """Triangles turned to face away from their centres (each its own point)."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    flip = np.einsum('ij,ij->i', n, tris.mean(axis=1) - centres) < 0
    tris[flip] = tris[flip][:, ::-1]
    return tris


def tyre(r_in, r_out, width, edge=1.0, n=120):
    """A tyre (wheel frame, its axle along x): its section a rectangle with its outer corners rounded, revolved."""
    h = width / 2
    prof = [(-h, r_in), (h, r_in)]
    for x0, sgn in ((h - edge, 1), (-h + edge, -1)):
        prof += [(x0 + sgn * edge * math.sin(a), r_out - edge + edge * math.cos(a))
                 for a in (np.linspace(math.pi / 2, 0, 7) if sgn > 0 else np.linspace(0, math.pi / 2, 7))]
    prof = np.array(prof)
    mid = prof.mean(axis=0)
    th = np.linspace(0, 2 * math.pi, n + 1)
    tris, cen = [], []
    for i in range(n):
        for j in range(len(prof)):
            (xa, ra), (xb, rb) = prof[j], prof[(j + 1) % len(prof)]
            q = [np.array([x, r * math.cos(t), r * math.sin(t)]) for x, r, t in
                 ((xa, ra, th[i]), (xb, rb, th[i]), (xb, rb, th[i + 1]), (xa, ra, th[i + 1]))]
            c = np.array([mid[0], mid[1] * math.cos((th[i] + th[i + 1]) / 2), mid[1] * math.sin((th[i] + th[i + 1]) / 2)])
            tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
            cen += [c, c]
    return _orient(np.array(tris), np.array(cen))


def _bezier(p0, p1, p2, p3, n=40):
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 + 3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3


def _sweep(path, section, across0, across1):
    """A section ([(u, v)] around its centre, u across, v the other way) swept along a path (robot frame); its
    across direction turning from across0 at its start to across1 at its end."""
    path = np.asarray(path, float)
    tang = np.gradient(path, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    rings = []
    for k, (p, t) in enumerate(zip(path, tang)):
        f = k / (len(path) - 1)
        a = (1 - f) * np.asarray(across0, float) + f * np.asarray(across1, float)
        a -= (a @ t) * t
        a /= np.linalg.norm(a)
        b = np.cross(t, a)
        rings.append([p + u * a + v * b for u, v in section])
    tris, cen = [], []
    m = len(section)
    for k in range(len(path) - 1):
        for j in range(m):
            q = (rings[k][j], rings[k][(j + 1) % m], rings[k + 1][(j + 1) % m], rings[k + 1][j])
            c = (path[k] + path[k + 1]) / 2
            tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
            cen += [c, c]
    return _orient(np.array(tris), np.array(cen))


def _box_at(c, half):
    """An axis-aligned box's triangles (robot frame)."""
    return _box(tuple(c), half, 0.0)


def cables():
    """Its encoders' ribbon cables (six wires, 1.27 mm apart, the first one red) from each encoder's front, in its
    middle, folded down to its header's socket ahead (P1 the left motor's, P2 the right one's), the sockets, and its
    battery's wires, a red and a black from each cell's left end to its connector behind on the left (P3, P4):
    {colour: triangles} (robot frame)."""
    out = {}
    zb, zm, ym = K.BOARD_Z_MM, K.MOTOR_Z_MM, K.MOTOR_OFFSET_MM
    heads = {}
    for _n, ref, _at, _r, pads in modules():
        if ref in ('P1', 'P2', 'P3', 'P4'):
            heads[ref] = to_robot(*np.mean([c for _s, c, *_ in pads], axis=0))     # its pins' middle
    us = np.linspace(-3.8, 3.8, 49)                 # its six wires' ridges on both faces
    bump = 0.3 + 0.15 * np.abs(np.cos(math.pi * (us + 3.8) / 1.27 - math.pi / 2))
    ribbon = [(u, -h) for u, h in zip(us, bump)] + [(u, h) for u, h in zip(us[::-1], bump[::-1])]
    stripe = [(u, v) for u, v in ((-3.8, -0.46), (-2.55, -0.46), (-2.55, 0.46), (-3.8, 0.46))]
    enc_mid = K.MOTOR_FACE_X_MM - K.MOTOR_LEN_MM - K.ENCODER_LEN_MM / 2
    for ref, s in (('P1', -1), ('P2', 1)):
        hx, hy = heads[ref]
        top = zb + 9.0
        start = np.array([s * enc_mid, ym + K.MOTOR_R_MM * 0.6, zm + K.MOTOR_R_MM * 0.75])
        path = _bezier(start, start + (0, 6.0, 3.0), np.array([hx, hy - 4.0, top + 9.0]), np.array([hx, hy, top]), 120)
        t = np.linspace(0, 1, len(path))[:, None]
        path = path + np.hstack([np.zeros_like(t), 1.2 * np.sin(4 * math.pi * t) * np.sin(math.pi * t),
                                 np.zeros_like(t)])        # its folds, as in its photo
        for shape, colour in ((ribbon, RIBBON), (stripe, RIBBON_STRIPE)):
            out.setdefault(colour, []).append(_sweep(path, shape, (1, 0, 0), (0, 1, 0)))
        out.setdefault(IDC, []).append(_box_at((hx, hy, zb + 4.5), (3.3, 4.6, 4.5)))
    cx, cy, cz = K.CELL_HALF_MM
    tube = [(0.35 * math.cos(a), 0.35 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    for i, ref in enumerate(('P3', 'P4')):          # its rear cell to P3, its front one to P4
        hx, hy = heads[ref]
        yc = CELL_Y[i]
        z = K.BATTERY_Z_MM + cz
        for dy, dx, colour in ((-1.0, -1.0, RED), (1.0, 1.0, BLACK_WIRE)):
            start = np.array([-cx + 0.3, yc + dy, z])
            end = np.array([hx + dx, hy, zb + 6.0])
            path = _bezier(start, start + (-8.0, 0, 1.0), end + (0, 6.0, 10.0), end)
            out.setdefault(colour, []).append(_sweep(path, tube, (0, 0, 1), (1, 0, 0)))
    return {k: np.concatenate(v) for k, v in out.items()}


CELL_Y = (K.MOTOR_OFFSET_MM - 6.3, K.MOTOR_OFFSET_MM + 6.3)    # its two cells' middles, side by side on its motors


# ---- all ---------------------------------------------------------------------------------------------------------
def _sources():
    return [os.path.abspath(__file__), os.path.join(K.HERE, 'config.py'), PCB, STL, WHEEL_STL]


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
    for i, (colour, t) in enumerate(sorted(cables().items())):
        path = os.path.join(OUT, f'kerise_cable_{i}.obj')
        write_obj(path, t, 40)
        entries.append({'name': f'cable_{i}', 'file': path, 'body': 'chassis', 'rgba': colour})
    tyre_mesh = tyre(RIM_R_MM, K.WHEEL_RADIUS_MM, TYRE_Z_MM[1] - TYRE_Z_MM[0])
    for side in ('l', 'r'):
        for part, t, colour in (('wheel', wheel(side), WHEEL_PLASTIC), ('tyre', tyre_mesh, TYRE)):
            path = os.path.join(OUT, f'kerise_{part}_{side}.obj')
            write_obj(path, t, 40)
            entries.append({'name': f'{part}_{side}', 'file': path, 'body': f'wheel_{side}', 'rgba': colour})
    with open(os.path.join(OUT, 'manifest.json'), 'w') as f:
        json.dump(entries, f, indent=1)
    return entries


def meshes():
    manifest = os.path.join(OUT, 'manifest.json')
    if not os.path.exists(manifest) or os.path.getmtime(manifest) < max(os.path.getmtime(p) for p in _sources()):
        return build()
    with open(manifest) as f:
        return json.load(f)
