"""Futura's gearmotors and their brackets, drawn here from their makers' dimensions (nothing of Pololu's files is used
or kept): the Pololu 10:1 Micro Metal Gearmotor HP (#999, the "N20" size; Pololu's dimension diagram "Micro Metal
Gearmotors with precious metal brushes and no encoder", 2024) and the Pololu Micro Metal Gearmotor Bracket Extended
(#1089, white; its sizes measured on Pololu's 3D model of it, micro-metal-gearmotor-with-1089-extended-bracket.step).

Their frame: x along the shaft (out of the gearbox), its origin on the axis at the gearbox's front plate; y across
(12 mm), z up (10 mm). Every mesh is a closed triangle soup in mm, its faces outward.
"""
import math

import numpy as np

from .pcb import _triangulate

# The motor (the diagram)
WIDTH, HEIGHT = 12.0, 10.0                  # its cross-section: a 12 mm can with two flats 10 mm apart
GEARBOX = 9.0                               # its gearbox's length (all ratios but 1000:1)
CAN = 15.0                                  # its motor's can
CAP = 1.2                                   # its black end cap
SHAFT = (3.0, 2.5, 10.0)                    # its output shaft: diameter, across its D's flat, out of the front plate
BUSHING = (4.0, 0.7)                        # its front bushing: diameter, height
PLATE = 0.8                                 # its gearbox's brass plates (assumed)
TERMINALS = (7.0, 1.5, 0.25, 1.9)           # its terminals: apart, wide, thick, out of the can (7 mm: the HP's, not HPCB's)
# The bracket (Pololu's model): a sleeve 15 x 11.5 mm and 17.5 mm long over the gearbox and the can's front half, open
# underneath (the motor's bottom on the mount), 1.5 mm walls, a 0.5 mm lip over the front plate with a 6 mm hole; an
# ear each side, 3 mm thick and flush with the bottom, round (r 4.25) about its screw's hole 9 mm off the axis and
# 12.5 mm behind the front plate, the hole 2.3 mm with a hex pocket on top that holds the 2-56 nut.
SLEEVE = (15.0, 6.5, 17.0, 0.5)             # width, top over the axis, length behind the front plate, front lip
EAR = (9.0, -12.5, 4.25, 3.0)               # its hole's y, its x, its radius, its thickness
EAR_HOLE, NUT_POCKET = 1.15, (4.9, 1.5)     # its hole's radius; its hex pocket: across flats, depth
NUT = (4.76, 1.6)                           # a 2-56 hex nut: across flats, thick


def _ccw(poly):
    p = np.asarray(poly, float)
    area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    return p if area > 0 else p[::-1]


def prism(poly, x0, x1):
    """A simple polygon in (y, z) between x0 and x1."""
    p = _ccw(poly)
    a, b = np.c_[np.full(len(p), x0), p], np.c_[np.full(len(p), x1), p]
    tris = [(b[i], b[j], b[k]) for i, j, k in _triangulate([tuple(v) for v in p])]
    tris += [(a[i], a[k], a[j]) for i, j, k in _triangulate([tuple(v) for v in p])]
    for i in range(len(p)):
        j = (i + 1) % len(p)
        tris += [(a[i], a[j], b[j]), (a[i], b[j], b[i])]
    return np.array(tris)


def _ray(poly, c, ang):
    """Where a ray from c at ang leaves a convex polygon."""
    u = np.array([math.cos(ang), math.sin(ang)])
    best = math.inf
    for i in range(len(poly)):
        p, q = poly[i], poly[(i + 1) % len(poly)]
        e = q - p
        den = u[0] * e[1] - u[1] * e[0]
        if abs(den) < 1e-12:
            continue
        t = ((p[0] - c[0]) * e[1] - (p[1] - c[1]) * e[0]) / den
        s = ((p[0] - c[0]) * u[1] - (p[1] - c[1]) * u[0]) / den
        if t > 1e-9 and -1e-9 <= s <= 1 + 1e-9:
            best = min(best, t)
    return c + best * u


def holed_prism(outer, inner, x0, x1, n=48):
    """A convex polygon in (y, z) with a convex hole (both star-shaped about the hole's centre), between x0 and x1."""
    outer, inner = _ccw(outer), _ccw(inner)
    c = inner.mean(0)
    angs = {round(math.atan2(*(v - c)[::-1]) % (2 * math.pi), 9) for v in np.r_[outer, inner]}
    angs = sorted(angs | {round(2 * math.pi * k / n, 9) for k in range(n)})
    o = np.array([_ray(outer, c, a) for a in angs])
    i = np.array([_ray(inner, c, a) for a in angs])
    tris = []
    for k in range(len(angs)):
        m = (k + 1) % len(angs)
        O0, O1, I0, I1 = ([(x, *v) for x in (x0, x1)] for v in (o[k], o[m], i[k], i[m]))
        tris += [(O0[1], O1[1], I1[1]), (O0[1], I1[1], I0[1])]      # its x1 face
        tris += [(O0[0], I1[0], O1[0]), (O0[0], I0[0], I1[0])]      # its x0 face
        tris += [(O0[0], O1[0], O1[1]), (O0[0], O1[1], O0[1])]      # its outer wall
        tris += [(I0[0], I1[1], I1[0]), (I0[0], I0[1], I1[1])]      # its hole's wall
    return np.array(tris)


def _circle(r, n=40, c=(0.0, 0.0)):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


def _flats(r, half, n=48):
    """A circle of radius r cut by two flats at +-half (z): the can's section."""
    a0 = math.asin(half / r)
    pts = []
    for lo, hi in ((-a0, a0), (math.pi - a0, math.pi + a0)):
        pts += [(r * math.cos(a), r * math.sin(a)) for a in np.linspace(lo, hi, n // 2)]
    return np.array(pts)


def _rounded_rect(w, h, r, n=6):
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180),
                       (w / 2 - r, -h / 2 + r, 270)):
        pts += [(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)))
                for a in np.linspace(a0, a0 + 90, n)]
    return np.array(pts)


def _hexagon(across, c=(0.0, 0.0)):
    r = across / math.sqrt(3)
    return np.array([(c[0] + r * math.cos(math.radians(30 + 60 * k)), c[1] + r * math.sin(math.radians(30 + 60 * k)))
                     for k in range(6)])


def _yzx(tris):
    """A prism made along x with its profile in (y, z), turned so it is made along z with its profile in (x, y)."""
    return tris[..., [1, 2, 0]]


def can():
    """Its can (from the gearbox's back to the end cap), steel."""
    return prism(_flats(WIDTH / 2, HEIGHT / 2), -GEARBOX - CAN, -GEARBOX)


def end_cap():
    """Its black end cap and the brush holder's block on it."""
    x = -GEARBOX - CAN
    return np.r_[prism(_flats(WIDTH / 2 - 0.15, HEIGHT / 2 - 0.15), x - CAP, x),
                 prism(_rounded_rect(7.5, 5.0, 0.8), x - CAP - 0.35, x - CAP + 0.01)]


def plate():
    """A brass plate of its gearbox (its front at x = 0)."""
    return prism(_rounded_rect(WIDTH, HEIGHT, 1.0), -PLATE, 0.0)


def shaft():
    """Its D shaft, out of its bushing."""
    r, flat = SHAFT[0] / 2, SHAFT[1] - SHAFT[0] / 2
    a0 = math.asin(flat / r)
    a = np.linspace(math.pi - a0, 2 * math.pi + a0, 40)                # all round but the flat (its top)
    d = np.c_[r * np.cos(a), r * np.sin(a)]
    return prism(d, BUSHING[1] - 0.05, SHAFT[2])


def bracket():
    """Its extended bracket: sleeve, front lip, two ears with their holes and nut pockets."""
    w, top, length, lip = SLEEVE
    t = 1.5
    bottom = -HEIGHT / 2
    u = [(-w / 2, bottom), (-w / 2 + t, bottom), (-w / 2 + t, HEIGHT / 2), (w / 2 - t, HEIGHT / 2),
         (w / 2 - t, bottom), (w / 2, bottom), (w / 2, top), (-w / 2, top)]
    out = [prism(u, -length, 0.0)]
    out.append(holed_prism([(-w / 2, bottom), (w / 2, bottom), (w / 2, top), (-w / 2, top)], _circle(3.0), 0.0, lip))
    hy, hx, r, thick = EAR
    for s in (-1, 1):
        # its outline in (x, y): from the sleeve's wall out to a half circle round its hole
        half = [(hx + r * math.cos(a), s * hy + s * r * math.sin(a)) for a in np.linspace(0, math.pi, 25)]
        ear = np.array([(hx + r, s * (w / 2 - 0.01))] + half + [(hx - r, s * (w / 2 - 0.01))])
        c = (hx, s * hy)
        z_pocket = bottom + thick - NUT_POCKET[1]
        out.append(_yzx(holed_prism(ear, _circle(EAR_HOLE, 24, c), bottom, z_pocket)))
        out.append(_yzx(holed_prism(ear, _hexagon(NUT_POCKET[0], c), z_pocket, bottom + thick)))
    return np.concatenate(out)


def nut():
    """A 2-56 hex nut, centred, its axis z."""
    return _yzx(holed_prism(_hexagon(NUT[0]), _circle(1.05, 24), -NUT[1] / 2, NUT[1] / 2))


def _clean(tris):
    """Without its degenerate triangles (two angles of holed_prism on one vertex)."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    return tris[np.linalg.norm(n, axis=1) > 1e-9]


def meshes():
    return {k: _clean(f()) for k, f in (('n20_can', can), ('n20_cap', end_cap), ('n20_plate', plate),
                                        ('n20_shaft', shaft), ('n20_bracket', bracket), ('nut', nut))}
