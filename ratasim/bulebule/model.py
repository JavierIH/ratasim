"""Bulebule's model (its 2018 board): its board (pcb.py), its components (KiCad's 3D models, parts.py) where its layout
and photos put them, its modules (Blue Pill, GY-6500, TB6612FNG, DC-DC converters) built from those parts on
boards of their sizes, its printed and mechanical parts (mounts, motors, gears, wheels, battery) from its CadQuery dimensions.
Collisions: the board's outline as a convex hull up to the battery, its four tires; each side's wheels coupled.

    python -m ratasim.bulebule.model        (builds its meshes into build/cad)
"""
import json
import math
import os
import struct

import numpy as np

from .. import config as RC
from ..model import MM, ROBOT_GROUP, _f
from . import config as B, parts, pcb

OUT = os.path.join(B.BUILD, 'cad')
PT = B.PCB_TOP
BLUE_PILL = (0.0, 31.83)            # its centre: midway between its pin rows (layout)
WHEELS = [(s, f) for s in (-1, 1) for f in (1, -1)]     # side (-1 left), front (+1) / rear (-1)
WHEEL_X = B.WHEELS_SEPARATION / 2
# The IR LEDs, soldered lying on the board (its build guide), each bent across its two pads (layout): left side
# (the right one mirrored), the pads' centre (mm), where it looks (deg, 90 straight ahead), emitter or receiver,
# its body's base from its pads along where it looks (mm). The side pairs, by the centre, look 25 deg ahead of
# sideways; the front pairs, on the shoulders, ahead. None past the board's edge (photos): the front receivers,
# where the board's arc closes in, with their bodies over their pads.
IR_LEDS = [(-4.00, 49.85, 155.0, 'rx', 0.6), (-14.02, 46.85, 153.8, 'tx', 0.6),
           (-30.03, 38.82, 100.9, 'rx', -0.6), (-34.03, 29.84, 100.0, 'tx', 0.6)]
IR_AXIS_Z = 3.0                     # their axis over the board: their heat-shrink sleeves (r 3) lie on it
IR_BODY = 6.0                       # their bodies, flange to lens tip (KiCad's 8.6: photos show them shorter)
IR_SLEEVE = 4.8                     # their heat-shrink sleeves, the lens out of them (photos)
LEGS = B.PCB_THICK + 1.0            # through-hole legs under a board's surface, trimmed


# ---- geometry helpers: triangles (n, 3, 3) mm --------------------------------------------------------------------
def rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return {'x': np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), 'y': np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            'z': np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def box(sx, sy, sz):
    """A box from its corner at the origin's centre in x and y, standing on z = 0."""
    x, y = sx / 2, sy / 2
    v = np.array([[-x, -y, 0], [x, -y, 0], [x, y, 0], [-x, y, 0], [-x, -y, sz], [x, -y, sz], [x, y, sz], [-x, y, sz]])
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6),
         (3, 0, 4), (3, 4, 7)]
    return v[np.array(f)]


def prism(profile, length):
    """A profile (counter-clockwise in the y-z plane) extruded along x from 0 to length."""
    p = np.array(profile)
    n = len(p)
    c = p.mean(axis=0)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        a0, b0 = np.r_[0, p[i]], np.r_[0, p[j]]
        a1, b1 = np.r_[length, p[i]], np.r_[length, p[j]]
        tris += [[a0, b0, b1], [a0, b1, a1]]
        tris += [[np.r_[0, c], b0, a0], [np.r_[length, c], a1, b1]]
    return np.array(tris)


def cylinder(r, length, n=40):
    """Along x from 0 to length, its axis on y = z = 0."""
    return prism([(r * math.cos(t), r * math.sin(t)) for t in np.linspace(0, 2 * math.pi, n, endpoint=False)], length)


def tube(r_out, r_in, length, n=40):
    """An open tube along x from 0 to length, its axis on y = z = 0."""
    o = [(math.cos(t), math.sin(t)) for t in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    tris = []
    for i in range(n):
        j = (i + 1) % n
        for r, inner in ((r_out, False), (r_in, True)):
            a0, b0 = np.r_[0, r * o[i][0], r * o[i][1]], np.r_[0, r * o[j][0], r * o[j][1]]
            a1, b1 = a0 + [length, 0, 0], b0 + [length, 0, 0]
            q = [[a0, b0, b1], [a0, b1, a1]]
            tris += [t[::-1] for t in q] if inner else q
        for x, start in ((0.0, True), (length, False)):
            oi, oj = np.r_[x, r_out * o[i][0], r_out * o[i][1]], np.r_[x, r_out * o[j][0], r_out * o[j][1]]
            ii, ij = np.r_[x, r_in * o[i][0], r_in * o[i][1]], np.r_[x, r_in * o[j][0], r_in * o[j][1]]
            q = [[oi, oj, ij], [oi, ij, ii]]
            tris += [t[::-1] for t in q] if start else q
    return np.array(tris)


def gear(teeth, module, width):
    """A spur gear along x (its axis on y = z = 0): trapezoid teeth on its root circle."""
    rp, ra, rf = module * teeth / 2, module * (teeth / 2 + 1), module * (teeth / 2 - 1.25)
    prof = []
    for k in range(teeth):
        t = 2 * math.pi * k / teeth
        w = math.pi / teeth
        for a, r in ((-0.55 * w, rf), (-0.30 * w, ra), (0.30 * w, ra), (0.55 * w, rf)):
            prof.append((r * math.cos(t + a), r * math.sin(t + a)))
    return prism(prof, width)


def holed_disc(r_in, r_out, n, r_at, r_hole, m=24):
    """A flat ring in the y-z plane (its face to +x) with n round holes of r_hole on the circle r_at: one sector
    per hole, its boundary seen from the hole's centre, joined to the hole's edge at the same angles."""
    tris, a = [], math.pi / n
    for k in range(n):
        phi = 2 * math.pi * k / n
        c = r_at * np.array([math.cos(phi), math.sin(phi)])
        corners = [r * np.array([math.cos(phi + e * a), math.sin(phi + e * a)]) for r in (r_in, r_out) for e in (-1, 1)]
        th = sorted(set(np.linspace(0, 2 * math.pi, m, endpoint=False)) |
                    {math.atan2(*(q - c)[::-1]) % (2 * math.pi) for q in corners})

        def edge(t):
            d = np.array([math.cos(t), math.sin(t)])
            cd, cc = c @ d, c @ c
            hits = [-cd + math.sqrt(cd * cd - cc + r_out * r_out)]
            disc = cd * cd - cc + r_in * r_in
            if disc > 0 and -cd - math.sqrt(disc) > 0:
                hits.append(-cd - math.sqrt(disc))
            for e in (-1, 1):
                nrm = np.array([-math.sin(phi + e * a), math.cos(phi + e * a)])
                if abs(nrm @ d) > 1e-12 and -(nrm @ c) / (nrm @ d) > 1e-9:
                    hits.append(-(nrm @ c) / (nrm @ d))
            return c + min(hits) * d

        b = [edge(t) for t in th]
        h = [c + r_hole * np.array([math.cos(t), math.sin(t)]) for t in th]
        for i in range(len(th)):
            j = (i + 1) % len(th)
            q = [np.r_[0, p] for p in (h[i], b[i], b[j], h[j])]
            tris += [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]
    return np.array(tris)


def place(tris, at=(0, 0, 0), *rotations):
    for axis, deg in rotations:
        tris = tris @ rot(axis, deg).T
    return tris + np.array(at)


# ---- the assembly ---------------------------------------------------------------------------------------------
GREY = {(210, 209, 199): (205, 205, 208), (228, 227, 207): (230, 228, 215)}     # KiCad's metal grey: tin


def ir_led(add, x, y, deg, kind, bend):
    """A 5 mm LED lying on the board: its legs up from its pads, bent towards where it looks; in black heat-shrink."""
    d = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])
    t = np.array([-d[1], d[0], 0.0])
    za = PT + IR_AXIS_Z
    base = np.array([x, y, za]) + bend * d
    lens = (200, 215, 235) if kind == 'tx' else (30, 30, 45)                # water clear / black
    for rgb, tris in parts.load('led5'):
        if rgb == (178, 25, 13):                                            # its body (KiCad's: on the board, 3 mm up)
            z = tris[..., 2] - 3.0                                          # shortened: its cylinder, not its
            cut = 8.6 - IR_BODY                                             # flange or its dome
            z = np.where(z > 6.0, z - cut, np.where(z > 1.0, 1.0 + (z - 1.0) * (5.0 - cut) / 5.0, z))
            body = np.concatenate([tris[..., :2], z[..., None]], axis=-1)
            add(place(body, (0, 0, 0), ('x', -90), ('z', deg - 90)) + base, lens)
    add(place(tube(3.0, 2.75, IR_SLEEVE), (0, 0, 0), ('z', deg)) + base - 0.5 * d, (18, 18, 20))
    for s in (-1, 1):
        p = np.array([x, y, 0.0]) + s * 1.27 * t
        add(place(cylinder(0.25, za - PT + LEGS), (0, 0, 0), ('y', -90)) + p + [0, 0, PT - LEGS], (205, 205, 208))
        add(place(cylinder(0.25, abs(bend) + 0.5), (0, 0, 0), ('z', deg)) + p + (min(bend, 0) - 0.25) * d + [0, 0, za],
            (205, 205, 208))
        add(place(cylinder(0.8, 0.5), (0, 0, 0), ('y', -90)) + p + [0, 0, PT], (205, 205, 210))     # its solder


def build():
    groups = {}

    def add(tris, rgb, body='chassis'):
        groups.setdefault((body, tuple(rgb)), []).append(np.asarray(tris, float))

    def part(name, at, *rotations, recolour=None, body='chassis', flip=False):
        """A part on a board's surface at z; flip: a pin header upside down (its short pins up through a module);
        legs trimmed 1 mm under the board."""
        for rgb, tris in parts.load(name):
            t = tris * [1, -1, -1] + [0, 0, 2.54] if flip else tris.copy()
            t[..., 2] = np.maximum(t[..., 2], -LEGS)
            add(place(t, at, *rotations), (recolour or {}).get(rgb, GREY.get(rgb, rgb)), body)

    os.makedirs(OUT, exist_ok=True)
    pcb.build(OUT)
    bx, by = BLUE_PILL
    # The Blue Pill on its pin headers (their plastic under it), the GY-6500 flat under it; its USB on the left end,
    # its BOOT jumpers and reset button by it, its crystal, regulator, LEDs and SWD header (vertical) to the right
    zb = PT + 2.54
    add(place(box(53.34, 22.86, 1.6), (bx, by, zb)), (22, 72, 170))
    top = zb + 1.6
    for s in (-1, 1):
        part('hdr20', (bx, by + s * 7.62, PT), ('z', 90), flip=True)
    part('lqfp48', (bx, by, top), ('z', 45))
    part('hc49', (bx + 10.0, by, top), ('z', 90))
    part('usb', (bx - 24.6, by, top), ('z', 90))
    part('sot223', (bx + 16.0, by - 1.0, top), ('z', 90))
    part('hdr4', (bx + 24.13, by, top))
    part('hdr2x3', (bx - 14.0, by + 4.5, top), ('z', 90))
    for dx in (-2.54, 2.54):                                                # its BOOT jumpers' caps
        add(place(box(2.5, 5.0, 6.0), (bx - 14.0 + dx, by + 4.5, top + 2.6)), (230, 200, 30))
    part('tl3342', (bx - 15.0, by - 5.5, top))
    for dy in (-5.0, 5.0):
        part('led0805', (bx + 20.5, by + dy, top), ('z', 90))
    for dx in (5.0, -6.0, -9.0):
        part('c0805', (bx + dx, by + 7.5, top), ('z', 90))
    add(place(box(15, 25, 1.0), (bx - 14, by, PT)), (110, 45, 150))                 # GY-6500
    part('qfn24', (bx - 14, by, PT + 1.0))
    # The IR LEDs in black heat-shrink, their legs bent up from their pads; their MOSFETs and resistors
    for x, y, deg, kind, bend in IR_LEDS:
        for s in (-1, 1):
            ir_led(add, s * abs(x), y, deg if s < 0 else 180 - deg, kind, bend)
    for s in (-1, 1):
        part('sot23', (s * 9.0, 44.9, PT), ('z', -90))                        # the side emitters' MOSFETs
        part('r0805', (s * 9.0, 48.35, PT))
        part('r1206', (s * 3.0, 44.85, PT))
        for xr in (19.0, 26.0):
            part('r0805', (s * xr, 44.33, PT))
        part('sot23', (s * 29.6, 34.35, PT), ('z', 180 if s < 0 else 0))     # the front emitters' MOSFETs
        part('r1206', (s * 27.2, 29.4, PT), ('z', 90))
    # Mounts, motors (Faulhaber 15 mm, IE2 encoders inside), pinions, gears, wheels, axles
    mx = B.WAIST - B.MOUNT[0] / 2
    for s in (-1, 1):
        add(place(box(*B.MOUNT), (s * mx, 0, PT)), (24, 24, 26))
        inner = s * (mx - B.MOUNT[0] / 2)
        add(place(cylinder(B.MOTOR_D / 2, 16.5), (0, 0, 0), ('z', 180 if s > 0 else 0)) + [inner, 0, B.MOTOR_Z], (16, 16, 18))
        add(place(cylinder(B.MOTOR_D / 2 - 0.2, 4.0), (0, 0, 0), ('z', 180 if s > 0 else 0)) + [inner - s * 16.5, 0, B.MOTOR_Z], (70, 70, 74))
        add(place(cylinder(B.MOTOR_D / 2 + 0.15, 0.8), (0, 0, 0), ('z', 180 if s > 0 else 0)) + [inner - s * 2, 0, B.MOTOR_Z], (175, 175, 180))
        outer = s * B.WAIST
        add(place(gear(B.Z_PINION, B.MODULE, 1.6), (0, 0, 0), ('z', 0 if s > 0 else 180)) + [outer + s * 0.2, 0, B.MOTOR_Z], (200, 160, 60))
    for s, f in WHEELS:
        body = f'wheel_{"l" if s < 0 else "r"}{"" if f > 0 else "2"}'
        y = f * B.AXIS_SEPARATION / 2
        flip = ('z', 0 if s > 0 else 180)
        add(place(cylinder(1.5, 11.0), (0, 0, 0), flip) + [s * (B.WAIST - 3), y, B.AXLE_Z], (190, 190, 195))       # its axle, fixed
        local = np.array([-s * WHEEL_X, -y, -B.AXLE_Z])          # wheel meshes in its body's frame
        g = place(gear(B.Z_GEAR, B.MODULE, 1.2), (0, 0, 0), flip) + [s * (B.WAIST + 0.2), y, B.AXLE_Z]
        add(g + local, (236, 236, 232), body)
        x0 = s * B.TIRE_X
        at = np.array([x0, y, B.AXLE_Z]) + local
        face = x0 + s * (B.TIRE_W - B.RIM_RECESS)                           # the rim's face, in the tire
        rim = np.array([face, y, B.AXLE_Z]) + local
        add(place(tube(B.WHEEL_RADIUS, B.RIM_D / 2 - 0.05, B.TIRE_W), (0, 0, 0), flip) + at, (14, 14, 16), body)
        add(place(cylinder(B.RIM_D / 2, 0.4), (0, 0, 0), flip) + at, (34, 34, 36), body)        # its back
        add(place(holed_disc(B.BEARING_D / 2, B.RIM_D / 2, B.RIM_HOLES, B.RIM_HOLE_R, B.RIM_HOLE_D / 2), (0, 0, 0), flip) + rim,
            (34, 34, 36), body)
        for k in range(B.RIM_HOLES):                                        # its holes: walls, dark inside
            t = 2 * math.pi * k / B.RIM_HOLES
            c = np.array([0, B.RIM_HOLE_R * math.cos(t), B.RIM_HOLE_R * math.sin(t)])
            add(place(tube(B.RIM_HOLE_D / 2 + 0.3, B.RIM_HOLE_D / 2, 2.0, 16) + c - [2.0, 0, 0], (0, 0, 0), flip) + rim, (30, 30, 32), body)
            add(place(cylinder(B.RIM_HOLE_D / 2, 0.1, 16) + c - [2.0, 0, 0], (0, 0, 0), flip) + rim, (6, 6, 7), body)
        add(place(tube(B.BEARING_D / 2, 1.6, 0.8 + B.RIM_RECESS - 0.3), (0, 0, 0), flip) + rim - [s * 0.8, 0, 0], (200, 200, 205), body)
        add(place(tube(B.WASHER_D / 2, 1.6, 0.5), (0, 0, 0), flip) + rim + [s * 0.2, 0, 0], (190, 190, 195), body)    # the bolt's washer
        add(place(cylinder(B.BOLT_HEAD[0] / math.sqrt(3), B.BOLT_HEAD[1], 6), (0, 0, 0), flip) + rim + [s * 0.7, 0, 0],
            (200, 200, 206), body)                                          # its hex head
    bx_, by_, bz_ = B.BATTERY
    add(place(box(bx_, by_, bz_), (0, 0, B.MOTOR_Z + B.MOTOR_D / 2)), (26, 26, 30))
    add(place(box(bx_ - 6, by_ - 10, 0.2), (0, 0, B.MOTOR_Z + B.MOTOR_D / 2 + bz_)), (200, 200, 205))       # its label
    # The motors' encoder headers, in front of them, with their ribbon cables' IDC sockets
    for s in (-1, 1):
        part('hdr2x3', (s * 16.01, 14.04, PT))
        add(place(box(6.2, 9.4, 8.6), (s * 16.01, 14.04, PT + 2.54)), (214, 206, 188))
    # Behind (layout, the 2018 board): the TB6612FNG carrier on two rows of 8 pins, three DC-DC converters on rows of
    # 3, the HC-05's 6-pin socket (empty), the battery's connector, the speaker; red LEDs and buttons on the tabs
    for yy in (-11.56, -26.92):
        part('hdr8', (0, yy, PT), ('z', 90), flip=True)
    zd = PT + 2.54
    add(place(box(20.3, 20.3, 1.0), (0, -19.24, zd)), (205, 40, 32))
    part('ssop24', (0, -19.24, zd + 1.0), ('z', 90))
    for dx in (-6.5, 6.5):
        part('c0805', (dx, -14.5, zd + 1.0))
    for xc, yc in ((-16.0, -10.18), (16.0, -10.18), (-16.0, -24.18)):
        part('hdr3', (xc, yc, PT), ('z', 90), flip=True)
        add(place(box(10.2, 12.7, 1.0), (xc, yc + 1.27 - 6.35, PT + 2.54)), (25, 105, 60))
        part('sot23_6', (xc - 2.5, yc - 7.5, PT + 3.54))
        part('srn4018', (xc + 2.0, yc - 7.0, PT + 3.54))
    part('sock6', (0, -32.18, PT), ('z', 90))                                        # the HC-05's, empty
    part('jst2', (24.0, -29.25, PT), ('z', 90))
    add(place(box(5.0, 4.0, 2.5), (-24.6, -31.9, PT)), (22, 22, 24))                # the speaker
    add(place(cylinder(0.8, 0.1), (0, 0, 0), ('y', -90)) + [-24.6, -31.9, PT + 2.5], (60, 60, 64))
    for s in (-1, 1):
        part('led1206', (s * 30.4, -32.2, PT), ('z', 90), recolour={(87, 173, 113): (220, 30, 30)})
        part('pts810', (s * 36.3, -31.6, PT), ('z', 90))
    # meshes and manifest
    manifest = []
    for k, ((body, rgb), items) in enumerate(sorted(groups.items())):
        f = f'{body}_{k:02d}.stl'
        _write_stl(os.path.join(OUT, f), np.concatenate(items))
        manifest.append({'file': f, 'body': body, 'rgba': [round(c / 255, 3) for c in rgb] + [1]})
    for name, tex in (('pcb_top', 'top.png'), ('pcb_bottom', 'bottom.png')):
        manifest.append({'file': name + '.obj', 'body': 'chassis', 'rgba': [1, 1, 1, 1], 'texture': tex})
    manifest.append({'file': 'pcb_side.obj', 'body': 'chassis', 'rgba': [0.10, 0.36, 0.75, 1]})
    json.dump(manifest, open(os.path.join(OUT, 'manifest.json'), 'w'), indent=1)
    print(f'{len(manifest)} meshes in {OUT}')
    return OUT


def _write_stl(path, tris):
    tris = np.asarray(tris, np.float32).reshape(-1, 3, 3)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as fh:
        fh.write(b'\0' * 80 + struct.pack('<I', len(tris)) + rec.tobytes())


def _manifest():
    m = os.path.join(OUT, 'manifest.json')
    if not os.path.exists(m):
        build()
    return json.load(open(m))


# ---- MuJoCo -----------------------------------------------------------------------------------------------------
def defaults_xml():
    return (f'<default class="bvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="bhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            f'<default class="btire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="{B.TIRE_FRICTION} 0.005 0.0001" condim="3" solref="0.004 1"/></default>')


def assets_xml():
    out = ''.join(f'<texture name="b_{t}" type="2d" file="{OUT}/{t}.png"/><material name="b_{t}" texture="b_{t}" specular="0.3" shininess="0.4"/>'
                  for t in ('top', 'bottom'))
    poly = pcb.outline()
    top = B.MOTOR_Z + B.MOTOR_D / 2 + B.BATTERY[2]
    hull = ' '.join(f'{x * MM:.5f} {y * MM:.5f} {z * MM:.5f}' for x, y in poly for z in (B.PCB_BOTTOM, top))
    out += f'<mesh name="b_hull" vertex="{hull}"/>'
    out += ''.join(f'<mesh name="b_{e["file"][:-4]}" file="{OUT}/{e["file"]}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
                   for e in _manifest())
    return out


def _visual(e, at=(0, 0, 0)):
    look = f'material="b_{e["texture"][:-4]}"' if e.get('texture') else f'rgba="{_f(*e["rgba"])}"'
    return f'<geom class="bvisual" mesh="b_{e["file"][:-4]}" {look} pos="{_f(*(v * MM for v in at))}"/>'


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    p, man = prefix, _manifest()
    chassis = ''.join(_visual(e) for e in man if e['body'] == 'chassis')
    wheels = ''
    for s, f in WHEELS:
        name = f'wheel_{"l" if s < 0 else "r"}{"" if f > 0 else "2"}'
        x, y = s * WHEEL_X, f * B.AXIS_SEPARATION / 2
        vis = ''.join(_visual(e) for e in man if e['body'] == name)
        wheels += f'''
      <body name="{p}{name}" pos="{_f(x * MM, y * MM, B.AXLE_Z * MM)}">
        <joint name="{p}{name}" type="hinge" axis="-1 0 0" armature="1e-6"/>
        <geom name="{p}tire_{name[6:]}" class="btire" type="cylinder" size="{_f(B.WHEEL_RADIUS * MM, B.CONTACT_W / 2 * MM)}" euler="0 90 0" mass="0.004"/>
        {vis}
      </body>'''
    m = B.MASS_KG - 0.016
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * RC.CELL_MM * MM, start_xy[1] * RC.CELL_MM * MM, 0)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(0, 0.004, 0.012)}" mass="{m:.4f}" diaginertia="{_f(9e-5, 6e-5, B.INERTIA_Z)}"/>
      <geom name="{p}hull" class="bhull" type="mesh" mesh="b_hull"/>
      {chassis}{wheels}
    </body>'''


def equality_xml(prefix=''):
    """Each side's rear wheel geared to its front one."""
    return ''.join(f'<joint joint1="{prefix}wheel_{s}2" joint2="{prefix}wheel_{s}"/>' for s in 'lr')


if __name__ == '__main__':
    build()
