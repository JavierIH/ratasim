"""UKMARSBOT's boards' faces from their Gerber files, as its repository ships them (hardware/: the main board's
V1.3-alpha, the advanced wall sensor's V1.2; KiCad 6's RS-274X in mm and Excellon in inches, read from their zips):
the textures of each board's two faces (copper under the solder mask, tinned pads where the mask opens,
silkscreen, holes and Edge.Cuts' inner cutouts) and their meshes (the outline kicad.py reads from the board's
.kicad_pcb, UVs on the textures), in the robot's frame (mm).

A Gerber's coordinates are KiCad's with y up: x_robot = -y_gerber - 105, y_robot = x_gerber - 105 (kicad.py).
"""
import math
import os
import re
import zipfile

from . import config as U
from .kicad import ORIGIN, Board

IN = 25.4
PX_MM = 20                                      # the textures' resolution
MASK, COPPER, TIN, SILK, HOLE = (16, 90, 40), (30, 116, 52), (200, 200, 196), (240, 240, 234), (12, 12, 12)
BOARDS = {      # name: its .kicad_pcb, its Gerbers' zip, their names' prefix, its top and bottom (mm, robot frame)
    'mainboard': (U.MAINBOARD, os.path.join('hardware', 'mainboard', 'version-1.3',
                                            'ukmarsbot-mainboard-V1.3-alpha-gerbers.zip'), 'mainboard-',
                  U.MAIN_TOP, U.MAIN_BOTTOM),
    'sensorboard': (U.SENSOR_BOARD, os.path.join('hardware', 'wall-sensors', 'wall-sensor-advanced',
                                                 'ukmarsbot-wall-sensor-advanced-V1.2-gerbers.zip'),
                    'ukmarsbot-wall-sensor-advanced-', U.SENSOR_TOP, U.SENSOR_BOTTOM),
}


def _robot(gx, gy):
    return -gy - ORIGIN[1], gx - ORIGIN[0]


def _read(board, layer):
    _, zpath, prefix, *_ = BOARDS[board]
    with zipfile.ZipFile(os.path.join(U.UKMARSBOT_DIR, zpath)) as z:
        return z.read(prefix + layer).decode()


def parse(board, layer):
    """A Gerber layer's shapes (mm, robot frame): ('line', aperture, points), ('flash', aperture, (x, y)) and
    ('region', None, points). An aperture: ('C', [d]), ('R', [w, h]), ('O', [w, h]) or ('P', corners, radius) (KiCad's
    RoundRect macro), w and h along the robot's x and y."""
    s = _read(board, layer)
    assert '%MOMM*%' in s and '%FSLAX46Y46*%' in s, f'{layer}: not KiCad\'s mm, 4.6'
    aps = {}
    for m in re.finditer(r'%ADD(\d+)(\w+),([^*]*)\*%', s):
        kind, p = m.group(2), [float(v) for v in m.group(3).split('X')]
        if kind in ('R', 'O'):
            aps[int(m.group(1))] = (kind, [p[1], p[0]])           # turned to the robot's frame
        elif kind == 'C':
            aps[int(m.group(1))] = ('C', p[:1])
        elif kind == 'RoundRect':
            aps[int(m.group(1))] = ('P', [(-p[k + 1], p[k]) for k in (1, 3, 5, 7)], p[0])     # from its centre
    for m in re.finditer(r'%ADD(\d+)C,([\d.]+)\*%', s):
        aps[int(m.group(1))] = ('C', [float(m.group(2))])
    body = re.sub(r'%[^%]*%', '', s).replace('\n', '').replace('\r', '')
    out, x, y, ap, mode, region = [], 0.0, 0.0, None, 1, None

    def close():
        if region and len(region) > 2:
            out.append(('region', None, [_robot(u, v) for u, v in region]))
    for cmd in body.split('*'):
        if cmd.startswith('G04'):
            continue
        for g in map(int, re.findall(r'G(\d+)', cmd)):
            if g in (1, 2, 3):
                mode = g
            elif g == 36:
                region = []
            elif g == 37:
                close()
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
            return int(m.group(1)) / 1e6 if m else default
        nx, ny = num('X', x), num('Y', y)
        if d in (None, 1):
            if mode == 1:
                seg = [(x, y), (nx, ny)]
            else:                                   # G75: multi-quadrant arcs, I and J from the start
                cx, cy = x + num('I', 0), y + num('J', 0)
                r = math.hypot(x - cx, y - cy)
                a0, a1 = math.atan2(y - cy, x - cx), math.atan2(ny - cy, nx - cx)
                if mode == 3:
                    a1 += 2 * math.pi if a1 <= a0 + 1e-9 else 0
                else:
                    a1 -= 2 * math.pi if a1 >= a0 - 1e-9 else 0
                n = max(4, int(abs(a1 - a0) * 16))
                seg = [(cx + r * math.cos(a0 + (a1 - a0) * i / n), cy + r * math.sin(a0 + (a1 - a0) * i / n))
                       for i in range(n + 1)]
            if region is not None:
                region += seg if not region else seg[1:]
            else:
                out.append(('line', ap, [_robot(u, v) for u, v in seg]))
        elif d == 2 and region is not None:
            close()
            region = []
        elif d == 3:
            out.append(('flash', ap, _robot(nx, ny)))
        x, y = nx, ny
    return out


def holes(board):
    """Its drilled holes, plated and not (Excellon, inches, decimal): (x, y, diameter) mm, robot frame."""
    out = []
    for f in ('PTH.drl', 'NPTH.drl'):
        tools, t = {}, None
        for line in _read(board, f).splitlines():
            m = re.match(r'T(\d+)C([\d.]+)', line)
            if m:
                tools[int(m.group(1))] = float(m.group(2)) * IN
                continue
            m = re.match(r'T(\d+)$', line)
            if m:
                t = int(m.group(1))
                continue
            m = re.match(r'X(-?[\d.]+)Y(-?[\d.]+)$', line)
            if m:
                out.append((*_robot(float(m.group(1)) * IN, float(m.group(2)) * IN), tools[t]))
    return out


def outlines(board):
    """Its Edge.Cuts' loops (kicad.py), the outer one first."""
    return Board(BOARDS[board][0]).loops()


# ---- its textures ------------------------------------------------------------------------------------------------
def frame(board):
    xs, ys = zip(*outlines(board)[0])
    return min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5


def _draw(img, shapes, colour, fr):
    from PIL import ImageDraw
    x0, y0, x1, y1 = fr
    d = ImageDraw.Draw(img)

    def px(p):
        return ((p[0] - x0) * PX_MM, (y1 - p[1]) * PX_MM)

    def dot(c, r):
        d.ellipse((c[0] - r, c[1] - r, c[0] + r, c[1] + r), fill=colour)

    def stroke(q, r):
        d.line(q, fill=colour, width=max(1, int(round(2 * r))))
        for c in q:
            dot(c, r)
    for kind, ap, g in shapes:
        if kind == 'region':
            d.polygon([px(p) for p in g], fill=colour)
        elif kind == 'flash' and ap:
            cx, cy = px(g)
            if ap[0] == 'C':
                dot((cx, cy), ap[1][0] * PX_MM / 2)
            elif ap[0] == 'R':
                w, h = ap[1][0] * PX_MM / 2, ap[1][1] * PX_MM / 2
                d.rectangle((cx - w, cy - h, cx + w, cy + h), fill=colour)
            elif ap[0] == 'O':
                w, h = ap[1][0] * PX_MM / 2, ap[1][1] * PX_MM / 2
                r = min(w, h)
                stroke([(cx - w + r, cy - h + r), (cx + w - r, cy + h - r)], r)
            else:                                   # a rounded rectangle: its corners' polygon, grown by its radius
                corners = [(cx + u * PX_MM, cy - v * PX_MM) for u, v in ap[1]]
                d.polygon(corners, fill=colour)
                stroke(corners + corners[:1], ap[2] * PX_MM)
        elif kind == 'line' and ap:
            stroke([px(p) for p in g], max(ap[1][0] * PX_MM / 2, 0.6))


def textures(board, out):
    """<board>_top.png and <board>_bottom.png: each face as its layers make it (both seen from above)."""
    from PIL import Image, ImageDraw
    fr = frame(board)
    size = (int(round((fr[2] - fr[0]) * PX_MM)), int(round((fr[3] - fr[1]) * PX_MM)))
    for face, side in (('top', 'Top'), ('bottom', 'Bottom')):
        img = Image.new('RGB', size, MASK)
        _draw(img, parse(board, f'Cu{side}.gbr'), COPPER, fr)
        _draw(img, parse(board, f'Mask{side}.gbr'), TIN, fr)
        _draw(img, parse(board, f'Silk{side}.gbr'), SILK, fr)
        d = ImageDraw.Draw(img)

        def px(p):
            return ((p[0] - fr[0]) * PX_MM, (fr[3] - p[1]) * PX_MM)
        for x, y, dia in holes(board):
            (cx, cy), r = px((x, y)), dia * PX_MM / 2
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=HOLE)
        for loop in outlines(board)[1:]:
            d.polygon([px(p) for p in loop], fill=HOLE)
        img.save(os.path.join(out, f'{board}_{face}.png'))


# ---- its meshes --------------------------------------------------------------------------------------------------
def meshes(board, out):
    """<board>_top.obj and <board>_bottom.obj (UVs on the textures) and <board>_side.obj, mm."""
    from .model import _triangulate
    poly = outlines(board)[0]
    tris = _triangulate(poly)
    x0, y0, x1, y1 = frame(board)
    top, bottom = BOARDS[board][3:5]
    for face, z, flip in (('top', top, False), ('bottom', bottom, True)):
        with open(os.path.join(out, f'{board}_{face}.obj'), 'w') as f:
            for x, y in poly:
                f.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
            for x, y in poly:
                f.write(f'vt {(x - x0) / (x1 - x0):.5f} {(y - y0) / (y1 - y0):.5f}\n')
            for t in tris:
                a, b, c = (i + 1 for i in (t[::-1] if flip else t))
                f.write(f'f {a}/{a} {b}/{b} {c}/{c}\n')
    with open(os.path.join(out, f'{board}_side.obj'), 'w') as f:
        n = len(poly)
        for x, y in poly:
            f.write(f'v {x:.4f} {y:.4f} {top:.4f}\nv {x:.4f} {y:.4f} {bottom:.4f}\n')
        for i in range(n):
            j = (i + 1) % n
            t0, b0, t1, b1 = 2 * i + 1, 2 * i + 2, 2 * j + 1, 2 * j + 2
            f.write(f'f {t0} {b0} {b1}\nf {t0} {b1} {t1}\n')


def build(out):
    os.makedirs(out, exist_ok=True)
    for board in BOARDS:
        textures(board, out)
        meshes(board, out)
