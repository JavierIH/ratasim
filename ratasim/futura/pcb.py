"""Futura's board from its Gerber files (Futura_Gerber.zip, V1.1, Eagle's RS-274X and Excellon; downloaded at build
time): its outline (V1.1.GML), the textures of its two faces drawn from its layers (copper under the solder mask,
tinned pads where the mask opens, silkscreen, holes), its faces as meshes with those textures, and its pads, where
model.py puts its parts. Everything in mm, in the robot's frame (config.GERBER_ORIGIN_IN at the origin).
"""
import math
import os
import re

import numpy as np

from . import config as F

IN = 25.4
PX_MM = 20                                      # the textures' resolution
OX, OY = F.GERBER_ORIGIN_IN[0] * IN, F.GERBER_ORIGIN_IN[1] * IN
MASK, COPPER, TIN, SILK, HOLE = (14, 92, 38), (28, 118, 48), (205, 205, 200), (236, 236, 228), (20, 20, 20)


class Board:
    """A set of Gerber files: where its layers are (a layer's extension to its path) and the point (mm, in the
    Gerbers' own frame) that is the origin of what this module returns."""

    def __init__(self, path, origin_mm):
        self.path, self.ox, self.oy = path, origin_mm[0], origin_mm[1]


MAIN = Board(lambda layer: os.path.join(F.GERBERS, f'V1.1.{layer}'), (OX, OY))
# Its encoder board (encoder_gerber.zip): origin at the middle of its bottom edge, where its legs stand.
ENCODER = Board(lambda layer: os.path.join(F.ENC_GERBERS, f'encoder V1.1.{layer}'),
                (F.ENC_ORIGIN_IN[0] * IN, F.ENC_ORIGIN_IN[1] * IN))


def parse(layer, b=MAIN):
    """A Gerber layer's shapes (mm, robot frame): ('line', aperture, points), ('flash', aperture, (x, y)) and
    ('region', None, points); an aperture is ('C', [diameter]) or ('R', [width, height])."""
    s = open(b.path(layer)).read()
    assert 'G70*' in s or '%MOIN' in s, f'{layer}: not in inches'
    scale = IN / 10 ** int(re.search(r'%FSLAX(\d)(\d)Y', s).group(2))
    aps = {int(m.group(1)): (m.group(2), [float(v) * IN for v in m.group(3).split('X')])
           for m in re.finditer(r'%ADD(\d+)([A-Z0-9]+),([^*]*)\*%', s)}
    body = re.sub(r'%[^%]*%', '', s).replace('\n', '').replace('\r', '')
    out, x, y, ap, mode, region = [], b.ox, b.oy, None, 1, None
    for cmd in body.split('*'):
        for g in map(int, re.findall(r'G(\d+)', cmd)):
            if g in (1, 2, 3):
                mode = g
            elif g == 36:
                region = []
            elif g == 37:
                if region:
                    out.append(('region', None, [(u - b.ox, v - b.oy) for u, v in region]))
                region = None
        d = re.search(r'D(\d+)$', cmd)
        d = int(d.group(1)) if d else None
        if d is not None and d >= 10:
            ap = aps.get(d)
            continue
        if not re.search(r'[XYIJ]', cmd):
            continue

        def num(c, default):
            m = re.search(c + r'(-?\d+)', cmd)
            return int(m.group(1)) * scale if m else default
        nx, ny = num('X', x), num('Y', y)
        if d in (None, 1):
            if mode == 1:
                seg = [(x, y), (nx, ny)]
            else:                                   # G75: multi-quadrant arcs, I and J from the start
                cx, cy = x + num('I', 0), y + num('J', 0)
                r = math.hypot(x - cx, y - cy)
                a0, a1 = math.atan2(y - cy, x - cx), math.atan2(ny - cy, nx - cx)
                if mode == 3:
                    a1 += 2 * math.pi if a1 <= a0 else 0
                else:
                    a1 -= 2 * math.pi if a1 >= a0 else 0
                n = max(4, int(abs(a1 - a0) * 16))
                seg = [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n))
                       for i in range(n + 1)]
            if region is not None:
                region += seg if not region else seg[1:]
            else:
                out.append(('line', ap, [(u - b.ox, v - b.oy) for u, v in seg]))
        elif d == 2 and region:
            out.append(('region', None, [(u - b.ox, v - b.oy) for u, v in region]))
            region = []
        elif d == 3:
            out.append(('flash', ap, (nx - b.ox, ny - b.oy)))
        x, y = nx, ny
    return out


def holes(b=MAIN):
    """Its drilled holes (V1.1.TXT, Excellon, inches): (x, y, diameter) mm."""
    tools, t, out = {}, None, []
    for line in open(b.path('TXT')).read().splitlines():
        m = re.match(r'T(\d+)C([\d.]+)', line)
        if m:
            tools[int(m.group(1))] = float(m.group(2)) * IN
            continue
        m = re.match(r'T(\d+)$', line)
        if m:
            t = int(m.group(1))
            continue
        m = re.match(r'X(\d+)Y(\d+)', line)
        if m:
            out.append((int(m.group(1)) / 1e4 * IN - b.ox, int(m.group(2)) / 1e4 * IN - b.oy, tools[t]))
    return out


def pads(layer='GTP', b=MAIN):
    """Its pads on a layer (the top's solder paste by default): (x, y, width, height) mm."""
    out = []
    for kind, ap, g in parse(layer, b):
        if kind == 'flash' and ap:
            w = ap[1][0]
            out.append((g[0], g[1], w, ap[1][1] if ap[0] == 'R' and len(ap[1]) > 1 else w))
        elif kind == 'region':
            xs, ys = zip(*g)
            out.append(((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, max(xs) - min(xs), max(ys) - min(ys)))
    return out


# ---- its outline -------------------------------------------------------------------------------------------------
def outline(b=MAIN):
    """Its board's outline (V1.1.GML: its edges drawn there and back, in pieces), one closed loop counter-clockwise,
    without collinear points."""
    pts, edges = [], set()

    def key(p):                                 # its pieces meet within 0.0001" (2.5 um)
        for i, q in enumerate(pts):
            if abs(p[0] - q[0]) < 0.01 and abs(p[1] - q[1]) < 0.01:
                return i
        pts.append(p)
        return len(pts) - 1
    for kind, ap, g in parse('GML', b):
        if kind != 'line' or not ap or ap[1][0] == 0:    # its width-0 circles: the motor mounts' holes
            continue
        for a, b in zip(g, g[1:]):
            i, j = key(a), key(b)
            if i != j:
                edges.add((min(i, j), max(i, j)))
    nbr = {}
    for i, j in edges:
        nbr.setdefault(i, []).append(j)
        nbr.setdefault(j, []).append(i)
    start = min(nbr, key=lambda i: (pts[i][1], pts[i][0]))
    loop, prev = [start], None
    while True:
        nxt = [j for j in nbr[loop[-1]] if j != prev]
        prev = loop[-1]
        if nxt[0] == start:
            break
        loop.append(nxt[0])
    poly = [pts[i] for i in loop]
    if sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1])) < 0:
        poly.reverse()
    out = list(poly)
    k = 0
    while k < len(out):
        a, b, c = out[k - 1], out[k], out[(k + 1) % len(out)]
        if abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 1e-6:
            out.pop(k)
        else:
            k += 1
    return out


def _triangulate(poly):
    """Ear clipping of a simple counter-clockwise polygon: index triples."""
    p = np.array(poly)

    def area(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    idx, tris = list(range(len(poly))), []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            if area(p[i0], p[i1], p[i2]) <= 1e-9:
                continue
            if any(area(p[i0], p[i1], p[j]) >= 0 and area(p[i1], p[i2], p[j]) >= 0 and area(p[i2], p[i0], p[j]) >= 0
                   for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise ValueError('no ear: the outline is not simple')
    return tris + [tuple(idx)]


# ---- its textures ------------------------------------------------------------------------------------------------
def _frame(poly):
    xs, ys = zip(*poly)
    return min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5


def _draw(img, layer, colour, frame, grow=0.0, b=MAIN):
    from PIL import ImageDraw
    x0, y0, x1, y1 = frame
    d = ImageDraw.Draw(img)

    def px(p):
        return ((p[0] - x0) * PX_MM, (y1 - p[1]) * PX_MM)
    for kind, ap, g in parse(layer, b):
        if kind == 'region':
            d.polygon([px(p) for p in g], fill=colour)
        elif kind == 'flash' and ap:
            (cx, cy), w = px(g), (ap[1][0] + grow) * PX_MM / 2
            h = (ap[1][1] + grow) * PX_MM / 2 if ap[0] == 'R' and len(ap[1]) > 1 else w
            if ap[0] == 'R':
                d.rectangle((cx - w, cy - h, cx + w, cy + h), fill=colour)
            else:
                d.ellipse((cx - w, cy - w, cx + w, cy + w), fill=colour)
        elif kind == 'line' and ap:
            r = max(ap[1][0] * PX_MM / 2, 0.6)
            q = [px(p) for p in g]
            d.line(q, fill=colour, width=max(1, int(round(2 * r))))
            for cx, cy in q:
                d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=colour)


def textures(out, b=MAIN, prefix=''):
    """top.png and bottom.png (with a prefix before the name): each face as its layers make it (both seen from above,
    as its Gerbers draw them)."""
    from PIL import Image, ImageDraw
    frame = _frame(outline(b))
    size = (int(round((frame[2] - frame[0]) * PX_MM)), int(round((frame[3] - frame[1]) * PX_MM)))
    for name, (copper, mask, silk) in ((prefix + 'top.png', ('GTL', 'GTS', 'GTO')), (prefix + 'bottom.png', ('GBL', 'GBS', 'GBO'))):
        img = Image.new('RGB', size, MASK)
        _draw(img, copper, COPPER, frame, 0.0, b)
        _draw(img, mask, TIN, frame, 0.0, b)
        _draw(img, silk, SILK, frame, 0.0, b)
        d = ImageDraw.Draw(img)
        for x, y, dia in holes(b):
            cx, cy, r = (x - frame[0]) * PX_MM, (frame[3] - y) * PX_MM, dia * PX_MM / 2
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=HOLE)
        img.save(os.path.join(out, name))


# ---- its meshes --------------------------------------------------------------------------------------------------
def meshes(out, b=MAIN, prefix='pcb_', z_top=None, z_bottom=None):
    """pcb_top.obj and pcb_bottom.obj (UVs on the textures) and pcb_side.obj, mm (the prefix instead of pcb_; the
    faces at z_top and z_bottom: the board's)."""
    z_top = F.BOARD_TOP_MM if z_top is None else z_top
    z_bottom = F.BOARD_BOTTOM_MM if z_bottom is None else z_bottom
    poly = outline(b)
    tris = _triangulate(poly)
    x0, y0, x1, y1 = _frame(poly)
    for name, z, flip in ((prefix + 'top.obj', z_top, False), (prefix + 'bottom.obj', z_bottom, True)):
        with open(os.path.join(out, name), 'w') as f:
            for x, y in poly:
                f.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
            for x, y in poly:
                f.write(f'vt {(x - x0) / (x1 - x0):.5f} {(y - y0) / (y1 - y0):.5f}\n')
            for t in tris:
                a, b, c = (i + 1 for i in (t[::-1] if flip else t))
                f.write(f'f {a}/{a} {b}/{b} {c}/{c}\n')
    with open(os.path.join(out, prefix + 'side.obj'), 'w') as f:
        n = len(poly)
        for x, y in poly:
            f.write(f'v {x:.4f} {y:.4f} {z_top:.4f}\nv {x:.4f} {y:.4f} {z_bottom:.4f}\n')
        for i in range(n):
            j = (i + 1) % n
            t0, b0, t1, b1 = 2 * i + 1, 2 * i + 2, 2 * j + 1, 2 * j + 2
            f.write(f'f {t0} {b0} {b1}\nf {t0} {b1} {t1}\n')


def build(out):
    os.makedirs(out, exist_ok=True)
    textures(out)
    meshes(out)
    textures(out, ENCODER, 'enc_')                  # its encoder board, standing: the model turns it (z: its thickness)
    meshes(out, ENCODER, 'enc_', F.ENC_THICKNESS_MM / 2, -F.ENC_THICKNESS_MM / 2)
