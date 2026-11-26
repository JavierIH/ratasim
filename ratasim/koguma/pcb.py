"""KOGUMA's board from its KiCad board (hardware/ecad/KOGUMA-CHAN.MkII_v2.kicad_pcb in its repository, its Mk3's,
read at build time): its two faces drawn as textures (its tracks and its ground pour under its black solder mask, its
pads gold, its silkscreen white: lines, arcs, circles and its parts' references), and its bigger parts at their
packages' sizes where its footprints put them: its STM32F446 (LQFP-64, its leads, its marking), its drivers, its
regulators, its IMU, its op-amps, its transistors and diodes, its resonator, its buzzer, its electrolytic capacitor,
its two switches, its JST-PH connectors and its programming header. Its small passives and its LEDs are left to the
textures; its sensors' 3 mm emitters and photodiodes are cad.sensors()'s; its two encoder boards (panelled below it
in the same file) are its STEP's.

Its outline and its holes are its STEP's (its board, 基板2): cad.py cuts its top and bottom faces out of it and
gives them UVs on these textures, KiCad's board centred on the STEP's.

Frames: KiCad's in mm, y down; the robot's (cad.py, mm) x right, y forward, z up, its origin on the axle. A texture
is drawn as KiCad's top view (its rows along KiCad's y), the bottom face's too (seen through the board).
"""
import math
import os

import numpy as np

from ..kerise.cad import _kid, _kids, _sexp, _xy
from . import config as K

PCB = os.path.join(K.KOGUMA_DIR, 'hardware', 'ecad', 'KOGUMA-CHAN.MkII_v2.kicad_pcb')
PX_MM = 24                              # the textures' resolution
MAIN_MAX_Y = 134.0                      # KiCad y past which its panel holds its encoder boards, not its main board
# Its colours: a black solder mask, its copper a shade lighter under it, gold pads (ENIG), white silkscreen
MASK, COPPER, GOLD, SILK, DRILL = (16, 16, 18), (34, 34, 37), (206, 168, 92), (236, 236, 232), (8, 8, 8)
EDGE_RGB = (40, 40, 38)                 # its edges: the FR-4 under its mask
PCB_THICK_MM = 1.0

_BOARD = None


def board():
    global _BOARD
    if _BOARD is None:
        with open(PCB, encoding='utf-8') as f:
            _BOARD = _sexp(f.read())
    return _BOARD


def _rot(x, y, deg):
    """A footprint's point rotated as KiCad does (its angles counter-clockwise on screen, y down)."""
    a = math.radians(deg)
    return x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)


def _at(node):
    at = _kid(node, 'at')
    return float(at[1]), float(at[2]), (float(at[3]) if len(at) > 3 else 0.0)


def _width(g):
    s = _kid(g, 'stroke')
    w = _kid(s, 'width') if s else _kid(g, 'width')
    return float(w[1]) if w else 0.12


def _arc3(a, m, b, step=6.0):
    """Points along the arc from a through m to b."""
    (ax, ay), (mx, my), (bx, by) = a, m, b
    d = 2 * (ax * (my - by) + mx * (by - ay) + bx * (ay - my))
    if abs(d) < 1e-9:
        return [a, b]
    ux = ((ax * ax + ay * ay) * (my - by) + (mx * mx + my * my) * (by - ay) + (bx * bx + by * by) * (ay - my)) / d
    uy = ((ax * ax + ay * ay) * (bx - mx) + (mx * mx + my * my) * (ax - bx) + (bx * bx + by * by) * (mx - ax)) / d
    r = math.hypot(ax - ux, ay - uy)
    t0, tm, t1 = (math.atan2(p[1] - uy, p[0] - ux) for p in (a, m, b))
    span = (t1 - t0) % (2 * math.pi)
    if (tm - t0) % (2 * math.pi) > span:          # through m the other way round
        span -= 2 * math.pi
    n = max(2, int(abs(math.degrees(span)) / step))
    return [(ux + r * math.cos(t0 + span * i / n), uy + r * math.sin(t0 + span * i / n)) for i in range(n + 1)]


def main_frame():
    """Its main board's extent in KiCad (x0, y0, x1, y1) mm: its Edge.Cuts above its encoder boards."""
    pts = []
    for g in board()[1:]:
        if isinstance(g, list) and g[0] in ('gr_line', 'gr_arc') and _kid(g, 'layer')[1] == 'Edge.Cuts':
            p = [_xy(_kid(g, k)) for k in ('start', 'mid', 'end') if _kid(g, k)]
            if max(y for _, y in p) < MAIN_MAX_Y:
                pts += _arc3(*p) if len(p) == 3 else p
    xs, ys = zip(*pts)
    return min(xs), min(ys), max(xs), max(ys)


def footprints():
    """Its footprints on its main board: (library name, reference, side 'F' or 'B', (x, y, rotation), node)."""
    out = []
    for f in _kids(board(), 'footprint'):
        x, y, rot = _at(f)
        if y > MAIN_MAX_Y:
            continue
        ref = next((p[2] for p in _kids(f, 'property') if p[1] == 'Reference'), '')
        out.append((f[1], ref, _kid(f, 'layer')[1][0], (x, y, rot), f))
    return out


def pads(f, at):
    """A footprint's pads: (shape, (x, y) KiCad, rotation, (w, h), drill, layers)."""
    x, y, rot = at
    out = []
    for p in _kids(f, 'pad'):
        px, py, prot = _at(p)
        dx, dy = _rot(px, py, rot)
        drill = _kid(p, 'drill')
        d = float(drill[1]) if drill and len(drill) > 1 and drill[1] != 'oval' else 0.0
        size = _kid(p, 'size')
        out.append((p[3], (x + dx, y + dy), prot if len(_kid(p, 'at')) > 3 else rot,
                    (float(size[1]), float(size[2])), d, _kid(p, 'layers')[1:]))
    return out


# ---- its faces ---------------------------------------------------------------------------------------------------
def _silk(side):
    """What its silkscreen draws on a side: ('seg', points, width), ('circle', c, r, width), ('poly', points),
    ('text', text, (x, y), angle, height, mirrored)."""
    layer = f'{side}.SilkS'
    out = []

    def shape(g, place):
        kind = g[0].split('_')[1]
        w = _width(g)
        fill = _kid(g, 'fill')
        filled = fill is not None and fill[1] in ('yes', 'solid')
        if kind == 'line':
            out.append(('seg', [place(_xy(_kid(g, 'start'))), place(_xy(_kid(g, 'end')))], w))
        elif kind == 'arc':
            out.append(('seg', [place(p) for p in _arc3(*(_xy(_kid(g, k)) for k in ('start', 'mid', 'end')))], w))
        elif kind == 'circle':
            c, e = place(_xy(_kid(g, 'center'))), place(_xy(_kid(g, 'end')))
            out.append(('circle', c, math.dist(c, e), 0 if filled else w))
        elif kind == 'rect':
            (ax, ay), (bx, by) = _xy(_kid(g, 'start')), _xy(_kid(g, 'end'))
            pts = [place(p) for p in ((ax, ay), (bx, ay), (bx, by), (ax, by), (ax, ay))]
            out.append(('poly', pts) if filled else ('seg', pts, w))
        elif kind == 'poly':
            pts = [place(_xy(p)) for p in _kids(_kid(g, 'pts'), 'xy')]
            out.append(('poly', pts) if filled else ('seg', pts + pts[:1], w))

    def text(t, s, place):
        if _kid(t, 'hide') or 'hide' in t or not s or s.startswith('${'):
            return
        tx, ty, angle = _at(t)
        eff = _kid(t, 'effects')
        h = float(_kid(_kid(eff, 'font'), 'size')[1])
        j = _kid(eff, 'justify')
        out.append(('text', s, place((tx, ty)), angle, h, bool(j and 'mirror' in j)))

    for g in board()[1:]:
        if isinstance(g, list) and g[0].startswith('gr_') and _kid(g, 'layer') and _kid(g, 'layer')[1] == layer:
            shape(g, lambda p: p)
    for _name, _ref, _side, (x, y, rot), f in footprints():
        def place(p, x=x, y=y, rot=rot):
            dx, dy = _rot(p[0], p[1], rot)
            return x + dx, y + dy
        for g in f[1:]:
            if not isinstance(g, list) or not _kid(g, 'layer') or _kid(g, 'layer')[1] != layer:
                continue
            if g[0] in ('fp_line', 'fp_arc', 'fp_circle', 'fp_rect', 'fp_poly'):
                shape(g, place)
            elif g[0] in ('property', 'fp_text'):
                text(g, g[2], place)
    return out


def textures(out):
    """pcb_top.png and pcb_bottom.png: each face as KiCad's top view draws it; returns its frame (KiCad mm)."""
    from PIL import Image, ImageDraw, ImageFont
    x0, y0, x1, y1 = main_frame()
    x0, y0, x1, y1 = x0 - 0.5, y0 - 0.5, x1 + 0.5, y1 + 0.5
    size = (int(round((x1 - x0) * PX_MM)), int(round((y1 - y0) * PX_MM)))
    b = board()

    def px(p):
        return (p[0] - x0) * PX_MM, (p[1] - y0) * PX_MM

    vias = [(_xy(_kid(v, 'at')), float(_kid(v, 'size')[1]), float(_kid(v, 'drill')[1])) for v in _kids(b, 'via')]
    for face, side in (('top', 'F'), ('bottom', 'B')):
        img = Image.new('RGB', size, MASK)
        d = ImageDraw.Draw(img)

        def line(pts, w, colour):
            r = max(w * PX_MM / 2, 0.6)
            q = [px(p) for p in pts]
            d.line(q, fill=colour, width=max(1, int(round(2 * r))))
            for cx, cy in q:
                d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=colour)

        def disc(c, r, colour):
            (cx, cy), r = px(c), r * PX_MM
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=colour)

        # its copper, under its mask
        for z in _kids(b, 'zone'):
            for fp in _kids(z, 'filled_polygon'):
                if _kid(fp, 'layer')[1] == f'{side}.Cu':
                    d.polygon([px(_xy(p)) for p in _kids(_kid(fp, 'pts'), 'xy')], fill=COPPER)
        for s in _kids(b, 'segment'):
            if _kid(s, 'layer')[1] == f'{side}.Cu':
                line([_xy(_kid(s, 'start')), _xy(_kid(s, 'end'))], float(_kid(s, 'width')[1]), COPPER)
        for c, dia, _drill in vias:
            disc(c, dia / 2, COPPER)
        # its pads, where its mask opens
        for _name, _ref, _side, at, f in footprints():
            for shape, c, rot, (w, h), drill, layers in pads(f, at):
                if not any(lay in (f'{side}.Cu', '*.Cu') for lay in layers):
                    continue
                if shape == 'circle':
                    disc(c, w / 2, GOLD)
                elif shape == 'oval' or (shape == 'roundrect' and min(w, h) < 0.6):
                    r = min(w, h) / 2
                    a, e = _rot((w / 2 - r) if w > h else 0, (h / 2 - r) if h >= w else 0, rot)
                    line([(c[0] - a, c[1] - e), (c[0] + a, c[1] + e)], 2 * r, GOLD)
                else:
                    corners = [_rot(sx * w / 2, sy * h / 2, rot) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                    d.polygon([px((c[0] + a, c[1] + e)) for a, e in corners], fill=GOLD)
                if drill:
                    disc(c, drill / 2, DRILL)
        for c, _dia, drill in vias:
            disc(c, drill / 2, DRILL)
        # its silkscreen
        for it in _silk(side):
            if it[0] == 'seg':
                line(it[1], it[2], SILK)
            elif it[0] == 'poly':
                d.polygon([px(p) for p in it[1]], fill=SILK)
            elif it[0] == 'circle':
                (cx, cy), r = px(it[1]), it[2] * PX_MM
                if it[3]:
                    d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=SILK, width=max(1, int(round(it[3] * PX_MM))))
                else:
                    d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=SILK)
            else:
                _kind, s, c, angle, h, mirror = it
                font = ImageFont.load_default(size=max(6, int(round(h * PX_MM / 0.72))))
                box = font.getbbox(s)
                glyphs = Image.new('L', (box[2] + 4, box[3] + 4), 0)
                ImageDraw.Draw(glyphs).text((2, 2), s, font=font, fill=255)
                glyphs = glyphs.crop(glyphs.getbbox() or (0, 0, 1, 1))
                if mirror:
                    glyphs = glyphs.transpose(Image.FLIP_LEFT_RIGHT)
                glyphs = glyphs.rotate(-angle if mirror else angle, expand=True, resample=Image.BICUBIC)
                cx, cy = px(c)
                img.paste(SILK, (int(round(cx - glyphs.width / 2)), int(round(cy - glyphs.height / 2))), glyphs)
        img.save(os.path.join(out, f'pcb_{face}.png'))
    return x0, y0, x1, y1


# ---- its parts ---------------------------------------------------------------------------------------------------
CHIP, SILVER, IVORY, PIN, BLACK = (24, 24, 27), (176, 178, 182), (236, 232, 218), (214, 190, 128), (12, 12, 13)
BEIGE = (204, 190, 160)
# Its parts by footprint: (its body across its footprint's x and y, its height, its colour[, its body's centre off
# the footprint's origin]); 'leads' draws its leads from its body's edge to its pads.
PARTS = {
    'QFP50P1200X1200X160-64N': dict(body=(10.0, 10.0), h=1.6, rgb=CHIP, leads=True),           # STM32F446RET6
    'MP6550GGP': dict(body=(2.0, 3.0), h=0.9, rgb=CHIP),                        # its motors' drivers (QFN, 2 x 3)
    'MYLSM00502ERPL': dict(body=(3.0, 3.6), h=2.3, rgb=CHIP),                   # its 5 V step-up module (Murata)
    'LGA-14L_STM': dict(body=(2.5, 3.0), h=0.86, rgb=CHIP),                     # its IMU (LSM6DSR, LGA-14 2.5 x 3)
    'MSOP-8_3x3mm': dict(body=(3.0, 3.0), h=1.1, rgb=CHIP, leads=True),         # its sensors' op-amps
    'SOT95P280X145-5N': dict(body=(1.6, 2.9), h=1.45, rgb=CHIP, leads=True),    # MIC5219 3.3 V regulator
    'SOT95P237X112-3N': dict(body=(1.3, 2.9), h=1.12, rgb=CHIP, leads=True),    # IRLML6344
    'SOT-23-6': dict(body=(1.6, 2.9), h=1.1, rgb=CHIP, leads=True),
    'SOT-23': dict(body=(1.3, 2.9), h=1.0, rgb=CHIP, leads=True),
    'D_SOD-123': dict(body=(2.7, 1.6), h=1.1, rgb=CHIP, leads=True),
    'CSTNE10M0G52Z000R0': dict(body=(3.2, 1.3), h=0.8, rgb=BEIGE, centre=(-1.2, 0)),   # its 10 MHz resonator
    'SMT0540SR': dict(body=(5.0, 5.0), h=2.0, rgb=CHIP, centre=(0, 0)),         # its buzzer
    'TVAF06A020BR': dict(body=(3.5, 2.9), h=1.6, rgb=SILVER),                   # its push switch
    'SSSS810701': dict(body=(6.7, 2.6), h=1.4, rgb=SILVER, centre=(0, 0)),      # its slide switch (bottom)
}


def _quad_box(lo, hi):
    """A box's 12 triangles, outward, from its corners lo and hi."""
    (x0, y0, z0), (x1, y1, z1) = lo, hi
    v = np.array([(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
                  (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def _frustum(hx, hy, z0, z1, inset):
    """A box from z0 to z1 whose top is inset on every side (a moulded package's draft): its triangles, outward."""
    lo = np.array([(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)])
    hi = lo - np.sign(lo) * inset
    v = np.array([(*p, z0) for p in lo] + [(*p, z1) for p in hi], float)
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5),
         (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def _cylinder(r, z0, z1, n=28, a0=0.0, a1=2 * math.pi, caps=True):
    """A cylinder (or a sector of its side) along z: its triangles, outward."""
    t = np.linspace(a0, a1, n + 1)
    ring = np.stack([r * np.cos(t), r * np.sin(t)], axis=1)
    tris = []
    for i in range(n):
        p, q = ring[i], ring[i + 1]
        tris += [((*p, z0), (*q, z0), (*q, z1)), ((*p, z0), (*q, z1), (*p, z1))]
        if caps:
            tris += [((0, 0, z1), (*p, z1), (*q, z1)), ((0, 0, z0), (*q, z0), (*p, z0))]
    return np.array(tris, float)


def _place(tris, at, side, board_z):
    """Triangles drawn in a footprint's frame (mm: x and y as KiCad's, y down; z up from the board's face) to KiCad's
    frame with z up from the board's top face (a part on its bottom upside down under it)."""
    x, y, rot = at
    a = math.radians(rot)
    c, s = math.cos(a), math.sin(a)
    t = tris.copy()
    px, py = t[..., 0], t[..., 1]
    t[..., 0], t[..., 1] = x + px * c + py * s, y - px * s + py * c
    if side == 'B':
        t[..., 2] = -board_z - t[..., 2]
        t = t[:, ::-1]                  # mirrored: its winding turned back outward
    return t


def _leads(pads_, body, z, w_frac=0.6):
    """Gull-wing leads, flattened: from its body's edge out to the outer part of each pad outside it (footprint
    frame)."""
    hx, hy = body[0] / 2, body[1] / 2
    out = []
    for (px, py), (w, h) in pads_:
        if abs(px) <= hx and abs(py) <= hy:
            continue
        if abs(px) - hx > abs(py) - hy:         # off its left or right side
            x0, x1 = sorted((math.copysign(hx, px), px + math.copysign(0.2 * max(w, h), px)))
            half = min(w, h) * w_frac / 2
            out.append(_quad_box((x0, py - half, 0.0), (x1, py + half, z)))
        else:
            y0, y1 = sorted((math.copysign(hy, py), py + math.copysign(0.2 * max(w, h), py)))
            half = min(w, h) * w_frac / 2
            out.append(_quad_box((px - half, y0, 0.0), (px + half, y1, z)))
    return out


def _local_pads(f):
    """A footprint's pads in its own frame: ((x, y), (w, h) along its x and y)."""
    _x, _y, rot = _at(f)
    out = []
    for p in _kids(f, 'pad'):
        px, py, prot = _at(p)
        size = _kid(p, 'size')
        w, h = float(size[1]), float(size[2])
        if round((prot - rot) / 90) % 2:
            w, h = h, w
        out.append(((px, py), (w, h)))
    return out


def _jst(pads_):
    """A JST PH vertical header (B2B-PH-K): its ivory housing, open on top round its two pins (gold)."""
    (x0, y0), x1 = pads_[0][0], pads_[-1][0][0]
    cx = (x0 + x1) / 2
    lo, hi = (cx - 3.0, y0 - 1.7, 0.0), (cx + 3.0, y0 + 2.8, 6.0)      # 6.0 x 4.5 x 6.0
    wall = 0.6
    ivory = [_quad_box(lo, (hi[0], hi[1], 1.6))]                         # its floor
    ivory += [_quad_box((lo[0], lo[1], 1.6), (lo[0] + wall, hi[1], hi[2])),
              _quad_box((hi[0] - wall, lo[1], 1.6), (hi[0], hi[1], hi[2])),
              _quad_box((lo[0] + wall, lo[1], 1.6), (hi[0] - wall, lo[1] + wall, hi[2])),
              _quad_box((lo[0] + wall, hi[1] - wall, 1.6), (hi[0] - wall, hi[1], hi[2]))]
    pins = [_quad_box((px - 0.32, py - 0.32, 1.6), (px + 0.32, py + 0.32, 5.2)) for (px, py), _ in pads_]
    return {IVORY: ivory, PIN: pins}


def _header(pads_):
    """A 2 x 3 1.27 mm pin header: its black base, its six gold pins."""
    xs, ys = zip(*(p for p, _ in pads_))
    base = [_quad_box((min(xs) - 0.635, min(ys) - 0.635, 0.0), (max(xs) + 0.635, max(ys) + 0.635, 1.0))]
    pins = [_quad_box((x - 0.2, y - 0.2, 1.0), (x + 0.2, y + 0.2, 4.0)) for x, y in zip(xs, ys)]
    return {BLACK: base, PIN: pins}


def _electrolytic():
    """A 6.3 x 7.7 mm SMD electrolytic capacitor: its black plastic base (its + corners cut), its silver can, its
    black stripe on its - side."""
    s = 3.3
    base = [_quad_box((-s, -s, 0.0), (s, s, 1.0))]
    can = [_cylinder(3.15, 1.0, 7.7, n=32, a0=-0.65 * math.pi, a1=0.65 * math.pi, caps=False),
           _cylinder(3.15, 7.69, 7.7, n=32)]
    stripe = [_cylinder(3.16, 1.0, 7.7, n=12, a0=0.65 * math.pi, a1=1.35 * math.pi, caps=False)]
    return {BLACK: base + stripe, SILVER: can}


def parts():
    """Its parts: {rgb: [triangles]} in KiCad's frame (mm, z up from its top face), and its MCU's top (four corners,
    KiCad frame, for its marking)."""
    groups, mcu = {}, None
    thick = PCB_THICK_MM
    for name, ref, side, at, f in footprints():
        lib = name.split(':')[-1]
        pads_ = _local_pads(f)
        made = {}
        if lib.startswith('JST_PH'):
            made = _jst(pads_)
        elif lib.startswith('PinHeader_2x03'):
            made = _header(pads_)
        elif lib.startswith('CP_Elec'):
            made = _electrolytic()
        else:
            spec = next((v for k, v in PARTS.items() if lib.startswith(k)), None)
            if spec is None:
                continue
            bx, by = spec['body']
            cx, cy = spec.get('centre', (0.0, 0.0))
            h = spec['h']
            body = _frustum(bx / 2, by / 2, 0.1, h, min(0.15, h * 0.1)) + (cx, cy, 0)
            made = {spec['rgb']: [body]}
            if spec.get('leads'):
                made.setdefault(SILVER, []).extend(_leads(pads_, (bx, by), 0.2))
            if lib.startswith('QFP'):
                hx = bx / 2 - min(0.15, h * 0.1)
                mcu = _place(np.array([[(-hx, -hx, h + 0.005), (hx, -hx, h + 0.005), (hx, hx, h + 0.005),
                                        (-hx, hx, h + 0.005)]]), at, side, thick)[0]
        for rgb, tris in made.items():
            groups.setdefault(rgb, []).append(_place(np.concatenate(tris), at, side, thick))
    return {k: np.concatenate(v) for k, v in groups.items()}, mcu


def mcu_texture(path):
    """Its STM32F446's top: its marking, white on black, its pin 1's dimple."""
    from PIL import Image, ImageDraw, ImageFont
    n = 256
    img = Image.new('RGB', (n, n), CHIP)
    d = ImageDraw.Draw(img)
    font = ImageFont.load_default(size=26)
    small = ImageFont.load_default(size=20)
    for i, (s, ft) in enumerate((('STM32F446', font), ('RET6', font), ('GH24P 93', small))):
        w = d.textlength(s, font=ft)
        d.text(((n - w) / 2, 80 + i * 34), s, font=ft, fill=(150, 150, 148))
    d.ellipse((22, 22, 46, 46), fill=(54, 54, 58))                      # its pin 1's dimple (its pads' -x -y corner)
    img.save(path)
