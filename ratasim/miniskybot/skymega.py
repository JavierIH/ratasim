"""Miniskybot's Skymega board as Obijuan published it (github.com/Obijuan/obijuan-designs, Skymega-board: v1.0, KiCad
and Gerber files, CC BY-SA 3.0, its own README; downloaded at build time, nothing of it in git, pinned by SHA-256):
its outline, its mounting holes and every component its KiCad board (skymega.brd, KiCad's legacy format) places, each
with its 3D model (the VRML files its zip ships, in packages3d/), and the textures of its two faces drawn from its
Gerber files (copper under the solder mask, tinned pads where the mask opens, silkscreen, holes).

    build(out)          its meshes and textures in out; the dict of what it made (see build())

Frames: the board's, in mm: x to its right, y towards its top edge (the expansion port's), z up from its top face,
the middle of its four mounting holes at the origin; KiCad's legacy .brd has y down and 0.1 mil units, a Gerber's
y up and 0.1 mil. Put on the robot, its top edge is the robot's front (cad.py).
"""
import hashlib
import math
import os
import re
import urllib.request
import zipfile

import numpy as np

from . import config as S

URL = 'https://raw.githubusercontent.com/Obijuan/obijuan-designs/master/Skymega-board/'
FILES = {       # name: its SHA-256
    'skymega-1.0-kicad-src.zip': 'da69f3d44c50454bd22d155def2a45ca5ae133fd4d0a0c7f61b22d03d9e1bf83',
    'skymega-v1.0-gerber.zip': '5c1d9d3642307bd402ff2a4aa36d0a4baed5b9241f633c2450971133ff8e1e36',
}
DIR = os.path.join(S.BUILD, 'skymega')
SRC = os.path.join(DIR, 'skymega-1.0-src')
GERBER = os.path.join(DIR, 'skymega-v1.0-gerber')
MIL = 0.00254                   # one legacy unit (0.1 mil) in mm
UNIT3D = 2.54                   # a VRML unit in mm (0.1 in)
PCB_MM = S.PCB_MM
PX_MM = 20                      # the textures' resolution
# The board's colours as its photo shows it: a blue solder mask, its copper under it a little lighter, tinned pads
MASK, COPPER, TIN, SILK, HOLE = (18, 52, 128), (28, 74, 160), (205, 205, 200), (238, 238, 232), (12, 12, 12)
SKIP = ('PLACA',)               # a green square standing for the board in its 3D view; the board's own mesh is here
EDGE = (0.07, 0.20, 0.50, 1)    # its board's edge


def download():
    """Its two zips in DIR (once; SKYMEGA_DOWNLOADS: a directory where they already are), checked, unzipped."""
    os.makedirs(DIR, exist_ok=True)
    local = os.environ.get('SKYMEGA_DOWNLOADS')
    for name, sha in FILES.items():
        path = os.path.join(DIR, name)
        if not os.path.exists(path):
            if local and os.path.exists(os.path.join(local, name)):
                with open(os.path.join(local, name), 'rb') as f:
                    data = f.read()
            else:
                req = urllib.request.Request(URL + name, headers={'User-Agent': 'ratasim'})
                data = urllib.request.urlopen(req, timeout=60).read()
            with open(path + '.part', 'wb') as f:
                f.write(data)
            os.replace(path + '.part', path)
        with open(path, 'rb') as f:
            if hashlib.sha256(f.read()).hexdigest() != sha:
                os.unlink(path)
                raise RuntimeError(f'{name}: not the file pinned in ratasim/miniskybot/skymega.py (SHA-256 {sha})')
    for name, target in (('skymega-1.0-kicad-src.zip', SRC), ('skymega-v1.0-gerber.zip', GERBER)):
        if not os.path.isdir(target):
            with zipfile.ZipFile(os.path.join(DIR, name)) as z:
                z.extractall(DIR)


# ---- its KiCad board ---------------------------------------------------------------------------------------------


def _read(name):
    with open(os.path.join(SRC, name), encoding='latin-1') as f:
        return f.read()


def holes_centre():
    """Its four mounting holes' middle, in the .brd's units (its 'Taladro' modules)."""
    pts = [(int(m.group(1)), int(m.group(2))) for m in
           re.finditer(r'\$MODULE Taladro\nPo (-?\d+) (-?\d+) ', _read('skymega.brd'))]
    assert len(pts) == 4, 'its four mounting holes'
    return sum(p[0] for p in pts) / 4, sum(p[1] for p in pts) / 4


def to_board(x, y):
    """A .brd point (y down) in the board's frame (mm)."""
    hx, hy = holes_centre()
    return (x - hx) * MIL, -(y - hy) * MIL


def outline():
    """Its Edge.Cuts (layer 28): four lines and four 90 deg arcs (centre, start; the end is the start turned a
    quarter the way a line's end is), as a counter-clockwise polygon (mm, board frame)."""
    lines, arcs = [], []
    for m in re.finditer(r'\$DRAWSEGMENT\nPo (\d) (-?\d+) (-?\d+) (-?\d+) (-?\d+) \d+\nDe 28 ', _read('skymega.brd')):
        kind, x0, y0, x1, y1 = int(m.group(1)), *(int(m.group(k)) for k in range(2, 6))
        (lines if kind == 0 else arcs).append(((x0, y0), (x1, y1)))
    ends = [p for ln in lines for p in ln]
    segs = [list(ln) for ln in lines]
    for (cx, cy), (sx, sy) in arcs:
        a0, r = math.atan2(sy - cy, sx - cx), math.hypot(sx - cx, sy - cy)
        for sign in (1, -1):
            end = (cx + r * math.cos(a0 + sign * math.pi / 2), cy + r * math.sin(a0 + sign * math.pi / 2))
            if min(math.dist(end, p) for p in ends) < 2:
                segs.append([(cx + r * math.cos(a0 + sign * t), cy + r * math.sin(a0 + sign * t))
                             for t in np.linspace(0, math.pi / 2, 9)])
                break
        else:
            raise ValueError('Edge.Cuts: an arc that meets no line')
    loop = segs.pop(0)
    while segs:
        for i, sg in enumerate(segs):
            for t in (sg, sg[::-1]):
                if math.dist(t[0], loop[-1]) < 2:
                    loop += t[1:]
                    segs.pop(i)
                    break
            else:
                continue
            break
        else:
            raise ValueError('Edge.Cuts: an open outline')
    poly = [to_board(x, y) for x, y in loop[:-1]]
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))
    return poly if area > 0 else poly[::-1]


def modules():
    """Its components: (name, x, y, angle deg, its 3D model's file, scale, offset in, rotation deg), the position in
    the board's frame. THT parts all stand on its top face, as its 3D view draws them whatever layer they are filed
    under."""
    out = []
    for m in re.finditer(r'\$MODULE (\S+)\n(.*?)\$EndMODULE', _read('skymega.brd'), re.S):
        name, body = m.groups()
        shape = re.search(r'\$SHAPE3D\n(.*?)\$EndSHAPE3D', body, re.S)
        if name in SKIP or not shape:
            continue
        po = re.search(r'^Po (-?\d+) (-?\d+) (-?\d+) ', body, re.M)
        x, y = to_board(int(po.group(1)), int(po.group(2)))
        f = os.path.basename(re.search(r'^Na "(.*)"', shape.group(1), re.M).group(1))
        vec = {k: tuple(float(v) for v in re.search(rf'^{k} (.*)$', shape.group(1), re.M).group(1).split())
               for k in ('Sc', 'Of', 'Ro')}
        out.append((name, x, y, int(po.group(3)) / 10, f, vec['Sc'], vec['Of'], vec['Ro']))
    return out


# ---- their 3D models (VRML 2, Wings 3D's) -------------------------------------------------------------------------


def read_wrl(path):
    """A VRML file's shapes: [(diffuse RGB, opacity, triangles (n, 3, 3))], in its own units."""
    with open(path, encoding='latin-1') as f:
        text = f.read()
    mats, out = {}, []
    for sh in re.split(r'\bShape\s*\{', text)[1:]:
        m = re.search(r'material\s+DEF\s+(\S+)\s+Material\s*\{(.*?)\}', sh, re.S)
        if m:
            d = re.search(r'diffuseColor\s+(\S+)\s+(\S+)\s+(\S+)', m.group(2))
            t = re.search(r'transparency\s+(\S+)', m.group(2))
            mats[m.group(1)] = (tuple(float(v) for v in d.groups()), 1 - (float(t.group(1)) if t else 0.0))
            mat = mats[m.group(1)]
        else:
            mat = mats[re.search(r'material\s+USE\s+(\S+)', sh).group(1)]
        pts = np.array(re.findall(r'-?\d[\d.]*(?:e[+-]?\d+)?', re.search(r'point\s*\[(.*?)\]', sh, re.S).group(1)),
                       float).reshape(-1, 3)
        tris, poly = [], []
        for i in re.findall(r'-?\d+', re.search(r'coordIndex\s*\[(.*?)\]', sh, re.S).group(1)):
            if int(i) < 0:
                tris += [(poly[0], poly[k], poly[k + 1]) for k in range(1, len(poly) - 1)]
                poly = []
            else:
                poly.append(int(i))
        out.append((mat[0], mat[1], pts[np.array(tris)]))
    return out


def _rot(deg, axis):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array({'x': [[1, 0, 0], [0, c, -s], [0, s, c]], 'y': [[c, 0, s], [0, 1, 0], [-s, 0, c]],
                     'z': [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis])


def place(tris, mod):
    """A component's model's triangles (VRML units) on the board (mm, board frame): KiCad's legacy order, the model's
    scale, its rotation about x, y then z, its offset (inches), the module's rotation (counter-clockwise on the
    screen) and its place."""
    _, x, y, ang, _, sc, off, ro = mod
    p = tris * sc
    for deg, axis in zip(ro, 'xyz'):
        if deg:
            p = p @ _rot(deg, axis).T
    p = (p + np.array(off) * 10) * UNIT3D
    return p @ _rot(ang, 'z').T + (x, y, 0)


# Its parts the robot's wiring goes to (cad.py): the servo headers' columns (Servo1..8 in its silkscreen), the power
# connector M1 (straight, its pins along x).
def _pads(name, value):
    """A module's pads' (x, y) in the board's frame (mm), their rotation and the module's place applied."""
    for m in re.finditer(r'\$MODULE (\S+)\n(.*?)\$EndMODULE', _read('skymega.brd'), re.S):
        if m.group(1) != name or not re.search(rf'^T0 .*N"{value}"', m.group(2), re.M):
            continue
        po = re.search(r'^Po (-?\d+) (-?\d+) (-?\d+) ', m.group(2), re.M)
        x0, y0, a = int(po.group(1)), int(po.group(2)), math.radians(int(po.group(3)) / 10)
        pads = []
        for pad in re.finditer(r'\$PAD\n.*?Po (-?\d+) (-?\d+)\n', m.group(2), re.S):
            lx, ly = int(pad.group(1)), int(pad.group(2))
            pads.append(to_board(x0 + lx * round(math.cos(a)) + ly * round(math.sin(a)),
                                 y0 - lx * round(math.sin(a)) + ly * round(math.cos(a))))
        return pads
    raise KeyError(value)


def servo_header(n):
    """Servo n's three pads, GND, VCC and signal (the board's frame)."""
    return _pads('CON1X3', f'Servo{n}')


def power_connector():
    """M1's two pads."""
    return _pads('Molex2P-R', 'M1')


# ---- the board's faces from its Gerber files ----------------------------------------------------------------------


def _gerber(name):
    with open(os.path.join(GERBER, name), encoding='latin-1') as f:
        return f.read()


def _g(x, y):
    """A Gerber point (0.1 mil, y up) in the board's frame."""
    hx, hy = holes_centre()
    return (x - hx) * MIL, (y + hy) * MIL


def parse(name):
    """A Gerber layer's shapes (mm, board frame): ('line', aperture, points), ('flash', aperture, (x, y)) and
    ('region', None, points); an aperture is ('C', [d]), ('R', [w, h]) or ('O', [w, h]) (inches to mm)."""
    s = _gerber(name)
    assert '%MOIN*%' in s and '%FSLAX34Y34*%' in s, f'{name}: not inches, 3.4'
    aps = {int(m.group(1)): (m.group(2), [float(v) * 25.4 for v in m.group(3).split('X')])
           for m in re.finditer(r'%ADD(\d+)([CRO]),([^*]*)\*%', s)}
    out, x, y, ap, region = [], 0, 0, None, None
    for cmd in re.sub(r'%[^%]*%', '', s).replace('\n', '').replace('\r', '').split('*'):
        if cmd == 'G36':
            region = []
        elif cmd == 'G37':
            if region and len(region) > 2:
                out.append(('region', None, [_g(*p) for p in region]))
            region = None
        m = re.match(r'G54D(\d+)$|D(\d+)$', cmd)
        if m and int(m.group(1) or m.group(2)) >= 10:
            ap = aps.get(int(m.group(1) or m.group(2)))
            continue
        m = re.match(r'X(-?\d+)Y(-?\d+)D0([123])$', cmd)
        if not m:
            continue
        nx, ny, d = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if d == 1:
            if region is not None:
                region += [(x, y)] if not region else []
                region.append((nx, ny))
            else:
                out.append(('line', ap, [_g(x, y), _g(nx, ny)]))
        elif d == 2 and region is not None:
            if region and len(region) > 2:
                out.append(('region', None, [_g(*p) for p in region]))
            region = []
        elif d == 3:
            out.append(('flash', ap, _g(nx, ny)))
        x, y = nx, ny
    return out


def drills():
    """Its holes (Excellon, mm): (x, y, diameter), the board's frame."""
    tools, t, out = {}, None, []
    hx, hy = holes_centre()
    for line in _gerber('skymega.drl').splitlines():
        m = re.match(r'T(\d+)C([\d.]+)', line)
        if m:
            tools[int(m.group(1))] = float(m.group(2))
        elif re.match(r'T\d+$', line):
            t = int(line[1:])
        else:
            m = re.match(r'X(-?[\d.]+)Y(-?[\d.]+)$', line)
            if m:
                out.append(((float(m.group(1)) / 25.4 * 10000 - hx) * MIL,
                            (float(m.group(2)) / 25.4 * 10000 + hy) * MIL, tools[t]))
    return out


def frame():
    xs, ys = zip(*outline())
    return min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5


def _draw(img, shapes, colour, fr):
    from PIL import ImageDraw
    x0, y0, x1, y1 = fr
    d = ImageDraw.Draw(img)

    def px(p):
        return (p[0] - x0) * PX_MM, (y1 - p[1]) * PX_MM

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
            else:                                   # an oval: a stroke along its long side
                w, h = ap[1][0] * PX_MM / 2, ap[1][1] * PX_MM / 2
                r = min(w, h)
                stroke([(cx - w + r, cy - h + r), (cx + w - r, cy + h - r)], r)
        elif kind == 'line' and ap:
            stroke([px(p) for p in g], max(ap[1][0] * PX_MM / 2, 0.6))


def textures(out):
    """skymega_top.png and skymega_bottom.png: each face as its layers make it (both seen from above)."""
    from PIL import Image, ImageDraw
    fr = frame()
    size = (int(round((fr[2] - fr[0]) * PX_MM)), int(round((fr[3] - fr[1]) * PX_MM)))
    for face, layers in (('top', ('Front.gtl', 'Mask_Front.gts', 'SilkS_Front.gto')),
                         ('bottom', ('Back.gbl', 'Mask_Back.gbs', 'SilkS_Back.gbo'))):
        img = Image.new('RGB', size, MASK)
        for layer, colour in zip(layers, (COPPER, TIN, SILK)):
            _draw(img, parse(f'skymega-{layer}'), colour, fr)
        d = ImageDraw.Draw(img)
        for x, y, dia in drills():
            cx, cy, r = (x - fr[0]) * PX_MM, (fr[3] - y) * PX_MM, dia * PX_MM / 2
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=HOLE)
        img.save(os.path.join(out, f'skymega_{face}.png'))


# ---- its meshes --------------------------------------------------------------------------------------------------
CHIP = ('DIL28-3', 'dil_28-w300.wrl', 0.95)     # its ATmega in its socket: the socket's own model has none; the chip's
#                                                 model (the same library) lifted over the socket's frame


def parts():
    """Every component's triangles on the board (mm, board frame), merged by colour: {(r, g, b, a): triangles}."""
    groups = {}
    mods = modules()
    socket = next(m for m in mods if m[0] == CHIP[0])
    mods.append((CHIP[0], *socket[1:4], CHIP[1], socket[5], (socket[6][0], socket[6][1], socket[6][2] + CHIP[2] / 10),
                 socket[7]))
    cache = {}
    for m in mods:
        shapes = cache.setdefault(m[4], read_wrl(os.path.join(SRC, 'packages3d', m[4])))
        for rgb, alpha, tris in shapes:
            key = (*(round(c, 2) for c in rgb), round(alpha, 2))
            groups.setdefault(key, []).append(place(tris, m))
    return {k: np.concatenate(v) for k, v in groups.items()}


def board_meshes(out, origin):
    """skymega_top.obj and skymega_bottom.obj (UVs on the textures) and skymega_side.obj, in the robot's frame (the
    board's, its middle at origin = (x, y, z of its top face))."""
    poly = outline()
    n = len(poly)
    x0, y0, x1, y1 = frame()
    ox, oy, oz = origin
    for face, z, flip in (('top', oz, False), ('bottom', oz - PCB_MM, True)):
        with open(os.path.join(out, f'skymega_{face}.obj'), 'w') as f:
            for x, y in poly:
                f.write(f'v {x + ox:.4f} {y + oy:.4f} {z:.4f}\n')
            for x, y in poly:
                f.write(f'vt {(x - x0) / (x1 - x0):.5f} {(y - y0) / (y1 - y0):.5f}\n')
            for k in range(1, n - 1):       # its outline is convex: a fan
                a, b, c = (i + 1 for i in ((0, k + 1, k) if flip else (0, k, k + 1)))
                f.write(f'f {a}/{a} {b}/{b} {c}/{c}\n')
    with open(os.path.join(out, 'skymega_side.obj'), 'w') as f:
        for x, y in poly:
            f.write(f'v {x + ox:.4f} {y + oy:.4f} {oz:.4f}\nv {x + ox:.4f} {y + oy:.4f} {oz - PCB_MM:.4f}\n')
        for i in range(n):
            j = (i + 1) % n
            t0, b0, t1, b1 = 2 * i + 1, 2 * i + 2, 2 * j + 1, 2 * j + 2
            f.write(f'f {t0} {b0} {b1}\nf {t0} {b1} {t1}\n')


def build(out, origin, write_obj):
    """Everything in out (once downloaded): the faces' textures and meshes, its components' meshes (one per colour,
    write_obj(path, triangles, crease) from cad.py), in the robot's frame with the board's middle at origin.
    Returns {'colours': [(name, path, rgba)], 'files': [every path]}."""
    download()
    os.makedirs(out, exist_ok=True)
    textures(out)
    board_meshes(out, origin)
    colours = []
    for i, (key, tris) in enumerate(sorted(parts().items())):
        path = os.path.join(out, f'skymega_part_{i}.obj')
        write_obj(path, tris + origin, 50)
        colours.append((f'skymega_part_{i}', path, key))
    return colours
