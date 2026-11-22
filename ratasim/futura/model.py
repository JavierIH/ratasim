"""Futura assembled: its board from its Gerber files (pcb.py: its outline, its faces textured with its copper, pads,
silkscreen and holes), its parts where its pads and holes are (their sizes their packages'): its STM32F405, its
HCMS-2903 display, its buzzer, its gyro, regulators and H-bridges, its capacitors and resistors (every pair of
0603 and 0805 pads), its IR emitters and phototransistors lying along the axes their holes give, its buttons, header,
switch and battery connectors; its two Pololu micro metal gearmotors end to end in their extended brackets (both drawn
in n20.py from Pololu's dimensions) on its white printed mount (its "Futura enhanced spacer.stl"), screwed through its
four holes, its encoder boards (from their own
Gerber files) straddling the brackets with their ICs facing the magnetic rings on its wheels, its printed wheels and skate
(their own STL files, downloaded at build time) and its battery behind the motors, where the photo of its kit shows them. Its frame: origin between the
wheels at the axle's height. Collisions: its board's outline as a convex hull up to 13 mm over it, its skate, its
tyres.
"""
import math
import os
import struct

import numpy as np

from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from .. import config as C
from . import config as F
from . import n20, pcb

MESHES = os.path.join(F.BUILD, 'meshes')
WHITE = (0.93, 0.93, 0.9, 1)
BLACK = (0.04, 0.04, 0.04, 1)
DARK = (0.12, 0.12, 0.13, 1)
GREY = (0.55, 0.55, 0.57, 1)
SILVER = (0.75, 0.76, 0.78, 1)
TANTALUM = (0.88, 0.62, 0.10, 1)
CERAMIC = (0.62, 0.52, 0.36, 1)
GREEN = (0.07, 0.41, 0.17, 1)
KAPTON = (0.85, 0.5, 0.08, 1)
TOP = F.BOARD_TOP_MM
MOTOR_Z = F.MOTOR_AXIS_MM                       # its motors' axis: on its mount's blocks (config)
MOUNT_TOP = TOP + F.MOTOR_MOUNT_Z_MM            # its printed mount's blocks' top
HULL_TOP = TOP + 13.0                           # the collision hull's top


def _read_stl(path):
    data = open(path, 'rb').read()
    n = struct.unpack('<I', data[80:84])[0]
    if 84 + 50 * n == len(data):
        rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
        return rec['v'].reshape(-1, 3, 3).astype(np.float64)
    v = [list(map(float, line.split()[1:4])) for line in data.decode(errors='replace').splitlines()
         if line.strip().startswith('vertex')]
    return np.array(v).reshape(-1, 3, 3)


def _write_stl(path, tris):
    tris = np.asarray(tris, np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    rec['v'] = tris.reshape(-1, 9)
    with open(path + '.part', 'wb') as f:
        f.write(b'ratasim futura'.ljust(80, b' ') + struct.pack('<I', len(tris)) + rec.tobytes())
    os.replace(path + '.part', path)


def meshes():
    """Its meshes in MESHES (made again when pcb.py or config.py is newer): its board's faces and their textures
    (pcb.py), its wheel (centred, its axis x) and its skate (centred, on z = 0)."""
    parts = n20.meshes()
    parts['battery'] = _rounded_box(BATTERY[2], 1.0)
    path = {k: os.path.join(MESHES, f'{k}.stl') for k in ('wheel', 'skate', 'mount', 'ring', *parts)}
    path.update({k: os.path.join(MESHES, f'{k}.obj')
                 for k in ('pcb_top', 'pcb_bottom', 'pcb_side', 'enc_top', 'enc_bottom', 'enc_side')})
    made = [os.path.join(MESHES, n) for n in ('top.png', 'bottom.png', 'enc_top.png', 'enc_bottom.png')] + list(path.values())
    sources = [pcb.__file__, F.__file__, n20.__file__]
    if all(os.path.exists(p) for p in made) and \
            min(map(os.path.getmtime, made)) > max(map(os.path.getmtime, sources)):
        return path
    F.download()
    os.makedirs(MESHES, exist_ok=True)
    pcb.build(MESHES)
    w = _read_stl(os.path.join(F.PRINTS, 'solid wheel wide.stl'))
    w -= (F.WHEEL_STL_CENTRE[0], F.WHEEL_STL_CENTRE[1], F.WHEEL_WIDTH_MM / 2)
    _write_stl(path['wheel'], w[..., [2, 0, 1]])          # its axis (z) to x
    s = _read_stl(os.path.join(F.PRINTS, 'skate.stl'))
    s -= (F.SKATE_STL_CENTRE[0], F.SKATE_STL_CENTRE[1], 0.0)
    _write_stl(path['skate'], s)
    m = _read_stl(os.path.join(F.PRINTS, 'Futura enhanced spacer.stl'))
    m -= (F.MOUNT_STL_CENTRE[0], F.MOUNT_STL_CENTRE[1], 0.0)
    m = m[..., [1, 0, 2]]                               # its length (STL y) along the axle (x)
    m[..., 0] *= F.MOTOR_MOUNT_HOLES[-1][0] / F.MOUNT_STL_HOLE_X     # its holes 31 mm apart, to the board's 29.7
    m[..., 2] += F.BOARD_TOP_MM
    _write_stl(path['mount'], m[:, ::-1])               # (the swap of two axes mirrors it: its faces turned back)
    _write_stl(path['ring'], _annulus(*F.RING_MM))
    for k, tris in parts.items():
        _write_stl(path[k], tris)
    return path


def _annulus(d_in, d_out, thickness, n=96):
    """A flat ring (mm), its axis x, centred: outward-facing triangles."""
    a = np.linspace(0, 2 * np.pi, n + 1)
    c, s = np.cos(a), np.sin(a)
    t = thickness / 2
    tris = []
    for i in range(n):
        for r, flip in ((d_out / 2, False), (d_in / 2, True)):               # the two walls
            p = [(-t, r * c[i], r * s[i]), (-t, r * c[i + 1], r * s[i + 1]), (t, r * c[i + 1], r * s[i + 1]), (t, r * c[i], r * s[i])]
            tris += [(p[0], p[2], p[1]), (p[0], p[3], p[2])] if flip else [(p[0], p[1], p[2]), (p[0], p[2], p[3])]
        ri, ro = d_in / 2, d_out / 2
        for x, up in ((t, True), (-t, False)):                                # the two faces
            q = [(x, ri * c[i], ri * s[i]), (x, ro * c[i], ro * s[i]), (x, ro * c[i + 1], ro * s[i + 1]), (x, ri * c[i + 1], ri * s[i + 1])]
            tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])] if up else [(q[0], q[2], q[1]), (q[0], q[3], q[2])]
    return np.array(tris)


def _rounded_box(size, r, n=8):
    """A box (mm, centred) with its edges and corners rounded to r: the hull of eight spheres."""
    from scipy.spatial import ConvexHull
    u, v = np.meshgrid(np.linspace(0, np.pi, n + 1), np.linspace(0, 2 * np.pi, 4 * n, endpoint=False))
    ball = r * np.c_[(np.sin(u) * np.cos(v)).ravel(), (np.sin(u) * np.sin(v)).ravel(), np.cos(u).ravel()]
    half = np.array(size) / 2 - r
    pts = np.unique(np.round(np.concatenate([ball + half * np.array(c) for c in np.ndindex(2, 2, 2) for c in [np.array(c) * 2 - 1]]), 9), axis=0)
    tris = pts[ConvexHull(pts).simplices]
    n_ = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    inward = np.einsum('ij,ij->i', n_, tris.mean(1)) < 0          # convex and centred: outward is away from 0
    tris[inward] = tris[inward][:, ::-1]
    return tris


def defaults_xml():
    return (f'<default class="uvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="uhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="uskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>'
            '<default class="utire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{F.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    path = meshes()
    out = ''.join(f'<texture name="u_{t}" type="2d" file="{MESHES}/{t}.png"/>'
                  f'<material name="u_{t}" texture="u_{t}" specular="0.3" shininess="0.4"/>' for t in ('top', 'bottom', 'enc_top', 'enc_bottom'))
    out += ''.join(f'<mesh name="u_{k}" file="{p}" scale="{_f(MM, MM, MM)}"{shell}/>'
                   for k, p in path.items() for shell in [' inertia="shell"' if p.endswith('.obj') else ''])
    hull = ' '.join(f'{x * MM:.5f} {y * MM:.5f} {z * MM:.5f}' for x, y in pcb.outline()
                    for z in (F.BOARD_BOTTOM_MM, HULL_TOP))
    return out + f'<mesh name="u_hull" vertex="{hull}"/>'


# ---- its parts ---------------------------------------------------------------------------------------------------
def _box(c, half, rgba, z_deg=0.0):
    """A box centred at c (mm), its half sizes, turned z_deg about z."""
    return (f'<geom class="uvisual" type="box" pos="{_f(*(v * MM for v in c))}" size="{_f(*(v * MM for v in half))}" '
            f'euler="0 0 {z_deg:g}" rgba="{_f(*rgba)}"/>')


def _on_board(x, y, size, rgba, z0=0.0, z_deg=0.0):
    """A part on the board's top at (x, y): its size (mm), its bottom z0 over the board."""
    return _box((x, y, TOP + z0 + size[2] / 2), [v / 2 for v in size], rgba, z_deg)


def _rod(p0, p1, r, rgba, kind='cylinder'):
    a = ' '.join(f'{v * MM:.5f}' for v in (*p0, *p1))
    return f'<geom class="uvisual" type="{kind}" fromto="{a}" size="{r * MM:.5f}" rgba="{_f(*rgba)}"/>'


# Where its pads and holes put them (pcb.pads(), pcb.holes(); mm) and their packages' sizes: x, y, size (x, y, z).
CHIPS = [
    ((0.0, 18.16), (10.0, 10.0, 1.4), BLACK),      # STM32F405RG, LQFP64
    ((0.0, 48.05), (4.0, 5.0, 1.5), DARK),         # LY3200ALH, LGA-16
    ((18.43, 25.13), (3.9, 4.9, 1.5), BLACK),      # TPS76850, SO-8
    ((14.3, 25.8), (1.6, 2.9, 1.1), BLACK),        # TPS73633, SOT-23-5
    ((-6.6, -7.69), (4.9, 3.9, 1.5), BLACK),       # ZXMHC3F381N8, SO-8: its H-bridges
    ((6.6, -7.69), (4.9, 3.9, 1.5), BLACK),
    ((-5.58, -14.3), (3.0, 3.0, 1.0), BLACK),      # TC4427, MSOP-8: their drivers
    ((5.58, -14.3), (3.0, 3.0, 1.0), BLACK),
    ((6.85, 27.58), (1.3, 2.5, 0.9), BLACK),       # D1
    ((31.79, 26.67), (3.9, 2.5, 1.8), DARK),       # 10 uH
    ((0.01, -8.97), (4.3, 7.3, 2.8), TANTALUM),    # 220 uF
    ((11.59, 18.76), (2.8, 3.5, 1.9), TANTALUM),   # 100 uF
    ((22.38, 28.66), (1.6, 3.2, 1.6), TANTALUM),   # 100 uF
]
DISPLAY = (-8.99, 35.0)                         # HCMS-2903: its 12 pins' holes, 2.54 mm apart, 7.62 mm across
BUZZER = (11.24, 36.05)                         # its two pins' holes, 5 mm apart
BUTTONS = [((-31.80, 22.75), 0.0), ((32.0, -22.0), 0.0), ((26.80, -30.66), 45.0)]   # Reset, SW0, SW1: 6 x 6 mm
HEADER = (-22.08, 22.24)                        # JTAG and serial: 2 x 4 pins, 2.54 mm
POWER_SWITCH = (-33.0, -24.0)                   # its slide switch, 3 pins
BATTERY = (0.0, -22.5, (30.0, 12.0, 7.0))       # its battery: where, its size (behind the motors, the photo)
BATTERY_SOCKETS = [(-26.3, -24.21), (-26.3, -30.68)]   # two 2 mm sockets (JST PH): its two cells


def _passives():
    """A part on every pair of like pads a 0603 or 0805 part apart (its paste layer): black resistors and LEDs (0603),
    brown capacitors (0805)."""
    pads = [p for p in pcb.pads() if {round(p[2], 2), round(p[3], 2)} in ({0.7, 0.9}, {1.0, 1.1})]
    used, out = set(), ''
    pairs = sorted((math.hypot(a[0] - b[0], a[1] - b[1]), i, j) for i, a in enumerate(pads)
                   for j, b in enumerate(pads) if i < j and a[2:] == b[2:]
                   and (abs(a[0] - b[0]) < 0.05 or abs(a[1] - b[1]) < 0.05))
    for d, i, j in pairs:
        if i in used or j in used or not 1.2 < d < 2.0:
            continue
        used |= {i, j}
        (x0, y0), (x1, y1) = pads[i][:2], pads[j][:2]
        small = pads[i][2] < 0.95
        size = (1.6, 0.8, 0.45) if small else (2.0, 1.25, 0.6)
        out += _on_board((x0 + x1) / 2, (y0 + y1) / 2, size, BLACK if small else CERAMIC, 0.0,
                         math.degrees(math.atan2(y1 - y0, x1 - x0)))
    return out


def _ir_pairs():
    """Its emitters (5 mm) and phototransistors (3 mm) lying on the board, from their holes along their axes, in
    their black sleeves (the photo), their lenses' tips clear."""
    out = ''
    for em, rx in F.IR_LEADS.values():
        for holes, length, r in ((em, F.EMITTER_LEN_MM, 2.5), (rx, F.RECEIVER_LEN_MM, 1.5)):
            a = math.radians(F.ir_axis(holes))
            u = (math.cos(a), math.sin(a))
            mx, my = (holes[0][0] + holes[1][0]) / 2, (holes[0][1] + holes[1][1]) / 2
            z = TOP + r

            def at(t):
                return (mx + t * u[0], my + t * u[1], z)
            out += _rod(at(1.0), at(length - r), r, BLACK, 'capsule')
            out += _rod(at(length - r - 0.2), at(length - r), r * 0.98, (0.6, 0.6, 0.65, 1))
            out += _rod(at(1.0), at(1.8), r + 0.4, BLACK)                  # its flange
            for x, y in holes:                                              # its leads, bent down into the board
                out += _rod((x, y, TOP), (x, y, z), 0.25, SILVER)
                out += _rod((x, y, z), (x + 1.0 * u[0], y + 1.0 * u[1], z), 0.25, SILVER)
    return out


GOLD = (0.85, 0.7, 0.3, 1)
STEEL = (0.38, 0.38, 0.41, 1)
CAN = (0.47, 0.48, 0.51, 1)
BRACKET = (0.9, 0.9, 0.86, 1)
ENC_BOTTOM_Z = TOP + F.ENC_RISE_MM              # its encoder boards' bottom edge


def _enc_pt(s, lx, ly, lz):
    """A point on an encoder board (its Gerbers' frame, mm from the middle of its bottom edge: lx along the board,
    ly up it, lz out of its top face, towards its wheel) in the robot's frame; s = -1 left, 1 right."""
    return (s * (F.ENC_X_MM + F.ENC_THICKNESS_MM / 2 + lz), s * lx, ENC_BOTTOM_Z + ly)


def _enc_part(s, lx, ly, lz0, size, rgba, deg=0.0):
    """A box on an encoder board: its size (along the board, up it, out of it), its bottom at lz0 out of the middle
    of its thickness (negative: on the inner face)."""
    c = _enc_pt(s, lx, ly, lz0 + size[2] / 2)
    return _box(c, (size[2] / 2, size[0] / 2, size[1] / 2), rgba)


def _encoders(s):
    """Its encoder board on one side: its Gerbers' faces standing at the notch's edge, its AS5304A (SSOP-20) on the
    wheel's side facing the ring, its four 0603/0805 parts on the inner side, its two right-angle headers on its
    legs, plunged into the main board's pins, and the magnetic ring on the wheel."""
    out = ''
    xy = '0 1 0 0 0 1' if s > 0 else '0 -1 0 0 0 1'                  # its x along +-y, its y up, its z out
    pos = _f(*(v * MM for v in (s * (F.ENC_X_MM + F.ENC_THICKNESS_MM / 2), 0, ENC_BOTTOM_Z)))
    for k in ('top', 'bottom'):
        out += f'<geom class="uvisual" type="mesh" mesh="u_enc_{k}" material="u_enc_{k}" pos="{pos}" xyaxes="{xy}"/>'
    out += f'<geom class="uvisual" type="mesh" mesh="u_enc_side" rgba="{_f(*GREEN)}" pos="{pos}" xyaxes="{xy}"/>'
    t = F.ENC_THICKNESS_MM / 2
    iy = (1.081 - F.ENC_ORIGIN_IN[1]) * 25.4                           # its IC's middle (its pads, GTP)
    out += _enc_part(s, 0.0, iy, t, (7.2, 5.3, F.ENC_IC_HEIGHT_MM), BLACK)                       # AS5304A, SSOP-20
    out += _enc_part(s, -2.6, iy - 1.8, t + F.ENC_IC_HEIGHT_MM, (0.9, 0.9, 0.04), (0.7, 0.7, 0.72, 1))   # its pin 1's dot
    for (x0, y0, x1, y1), big in (((0.4392, 1.1131, 0.4392, 1.18), True), ((0.525, 1.18, 0.525, 1.1131), True),
                                  ((0.296, 1.0869, 0.2448, 1.0869), False), ((0.2448, 1.0101, 0.296, 1.0101), False)):
        cx = ((x0 + x1) / 2 - F.ENC_ORIGIN_IN[0]) * 25.4
        cy = ((y0 + y1) / 2 - F.ENC_ORIGIN_IN[1]) * 25.4
        vertical = abs(x0 - x1) < 1e-6
        size = (1.0, 2.0, 0.7) if big and vertical else (2.0, 1.0, 0.6) if not vertical else (1.0, 1.6, 0.5)
        out += _enc_part(s, cx, cy, -t - size[2], size, CERAMIC if big else BLACK)
    for cx in (-10.54, 10.54):                                           # its two right-angle headers, 2 pins at 2.54 mm
        h0, h1 = F.ENC_PINS_X_MM - 1.27, F.ENC_X_MM                       # from the pins' x to the board's inner face
        out += _box((s * (h0 + h1) / 2, s * cx, ENC_BOTTOM_Z + 1.27), ((h1 - h0) / 2, 2.54, 1.27), BLACK)
        for dx in (-1.27, 1.27):
            out += _rod((s * F.ENC_PINS_X_MM, s * (cx + dx), ENC_BOTTOM_Z + 1.27), (s * F.ENC_PINS_X_MM, s * (cx + dx), TOP - 1.0), 0.3, GOLD)
    # the ring: on the wheel's open face
    rx = s * (F.WHEEL_FACE_MM - F.RING_MM[2] / 2)
    out += f'<geom class="uvisual" type="mesh" mesh="u_ring" pos="{_f(rx * MM, 0, 0)}" rgba="0.10 0.10 0.11 1"/>'
    return out


BRASS = (0.78, 0.6, 0.32, 1)
CAN_STEEL = (0.6, 0.61, 0.64, 1)
WIRE = (0.12, 0.45, 0.2, 1)
MOTOR_PADS = (2.0, 1.27)                        # its motors' pads (holes, x +-2.0, y +-1.27): R_motor, L_motor


def _motor_geoms(s):
    """One gearmotor and its bracket (n20.py: their frame, x along the shaft), its shaft out to its wheel; s = -1 left,
    1 right (the left one turned half round about z)."""
    pos = (s * F.MOTOR_FRONT_X_MM * MM, 0, MOTOR_Z * MM)
    turn = 0 if s > 0 else 180

    def mesh(name, rgba, dx=0.0):
        return (f'<geom class="uvisual" type="mesh" mesh="u_{name}" pos="{_f(pos[0] + s * dx * MM, 0, pos[2])}" '
                f'euler="0 0 {turn}" rgba="{_f(*rgba)}"/>')

    def at(lx, ly, lz):                                  # a point of its frame in the robot's
        return (s * (F.MOTOR_FRONT_X_MM + lx), s * ly, MOTOR_Z + lz)

    out = mesh('n20_can', CAN_STEEL) + mesh('n20_cap', BLACK) + mesh('n20_shaft', SILVER) + mesh('n20_bracket', BRACKET)
    for dx in (0.0, -4.2, -n20.GEARBOX + n20.PLATE):     # its gearbox's three brass plates
        out += mesh('n20_plate', BRASS, dx)
    for (ly, lz), x0, x1 in (((4.9, 3.9), -8.2, -0.8), ((-4.9, -3.9), -8.2, -0.8)):   # their two pillars
        out += _rod(at(x0, ly, lz), at(x1, ly, lz), 0.55, BRASS)
    for x0, (ly, lz, r) in ((-7.9, (-1.4, 0.8, 3.6)), (-7.4, (1.9, -0.6, 3.0)), (-3.9, (1.4, 0.6, 3.6)),
                            (-3.4, (-1.9, -0.4, 3.0))):                                  # its gears
        out += _rod(at(x0, ly, lz), at(x0 + 0.5, ly, lz), r, BRASS if r > 3.2 else STEEL)
        out += _rod(at(x0 + 0.5, ly, lz), at(x0 + 2.4, ly, lz), 1.1, STEEL)
    out += _rod(at(0.0, 0, 0), at(n20.BUSHING[1], 0, 0), n20.BUSHING[0] / 2, BRASS)      # its bushing
    apart, wide, thick, length = n20.TERMINALS
    x_can = -n20.GEARBOX - n20.CAN
    for ly in (-apart / 2, apart / 2):                   # its two terminals, green wires down to its pads (the photo)
        c = at(x_can - length / 2 + 0.3, ly, 0)
        out += _box(c, (length / 2, thick / 2, wide / 2), GOLD)
        tip = at(x_can - length + 0.6, ly, 0)
        pad = (s * MOTOR_PADS[0], math.copysign(MOTOR_PADS[1], tip[1]))
        bend = (*pad, TOP + 1.5)
        out += _rod(tip, bend, 0.3, WIRE, 'capsule') + _rod(bend, (*pad, TOP), 0.3, WIRE)
    for hx, hy in F.MOTOR_MOUNT_HOLES:                   # its screws (2-56 x 20 mm) up through board, mount and ear
        if hx * s < 0:
            continue
        ear_top = MOTOR_Z - n20.HEIGHT / 2 + n20.EAR[3]
        nut_z = ear_top - n20.NUT_POCKET[1] + n20.NUT[1] / 2
        out += (f'<geom class="uvisual" type="mesh" mesh="u_nut" pos="{_f(hx * MM, hy * MM, nut_z * MM)}" '
                f'rgba="{_f(*SILVER)}"/>')
        out += _rod((hx, hy, ear_top - 1.0), (hx, hy, F.BOARD_BOTTOM_MM + 20.0), 1.09, SILVER)    # its thread
        out += _rod((hx, hy, F.BOARD_BOTTOM_MM - 1.4), (hx, hy, F.BOARD_BOTTOM_MM), 1.9, SILVER)  # its pan head
        for deg in (0, 90):
            out += _box((hx, hy, F.BOARD_BOTTOM_MM - 1.4), (1.2, 0.25, 0.3), DARK, deg)        # its cross
    return out


def _motors():
    """Its two Pololu micro metal gearmotors HP end to end in their extended brackets (n20.py), their rear caps at the
    middle, each shaft out to its wheel's hub; the brackets' ears on its white printed mount, screwed through the
    board; its two encoder boards and magnetic rings."""
    out = ''.join(_motor_geoms(s) + _encoders(s) for s in (-1, 1))
    out += f'<geom class="uvisual" type="mesh" mesh="u_mount" rgba="{_f(*WHITE)}"/>'         # its printed mount
    return out


def _parts():
    """Its visible parts but its board, wheels and skate."""
    out = ''.join(_on_board(x, y, size, rgba) for (x, y), size, rgba in CHIPS)
    out += _passives() + _ir_pairs() + _motors()
    x, y = DISPLAY
    out += _on_board(x, y, (17.8, 10.2, 4.3), (0.55, 0.6, 0.62, 1), 0.6)                     # HCMS-2903
    for k in range(4):
        out += _on_board(x + (k - 1.5) * 4.0, y, (2.6, 3.8, 0.05), (0.95, 0.35, 0.1, 1), 4.9)  # its dots, lit
    x, y = BUZZER
    out += _rod((x, y, TOP), (x, y, TOP + 7.5), 6.0, BLACK)
    out += _rod((x, y, TOP + 7.5), (x, y, TOP + 7.55), 1.0, (0.7, 0.6, 0.3, 1))
    for (x, y), deg in BUTTONS:
        out += _on_board(x, y, (6.0, 6.0, 3.4), SILVER, 0.0, deg)
        out += _rod((x, y, TOP + 3.4), (x, y, TOP + 5.0), 1.7, BLACK)
    x, y = HEADER
    out += _on_board(x, y, (5.08, 10.16, 2.5), BLACK)
    for i in range(2):
        for j in range(4):
            out += _rod((x + (i - 0.5) * 2.54, y + (j - 1.5) * 2.54, TOP + 2.5),
                        (x + (i - 0.5) * 2.54, y + (j - 1.5) * 2.54, TOP + 8.5), 0.35, (0.85, 0.7, 0.3, 1))
    out += _on_board(*POWER_SWITCH, (3.5, 8.6, 3.5), BLACK)
    for x, y in BATTERY_SOCKETS:
        out += _on_board(x, y, (4.5, 6.0, 6.0), WHITE)
    x, y, size = BATTERY                                                                      # 2S 120 mAh, its edges rounded
    out += (f'<geom class="uvisual" type="mesh" mesh="u_battery" pos="{_f(x * MM, y * MM, (TOP + size[2] / 2) * MM)}" '
            f'rgba="{_f(*KAPTON)}"/>')
    return out


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    p = prefix
    r = F.WHEEL_RADIUS_MM * MM
    wheels = ''
    for side, sign in (('l', -1), ('r', 1)):
        at = (sign * F.TRACK_MM / 2, 0, 0)
        wheels += f'''
      <body name="{p}wheel_{side}" pos="{_f(*(v * MM for v in at))}">
        <joint name="{p}wheel_{side}" type="hinge" axis="-1 0 0" armature="{F.MOTOR_ROTOR_INERTIA * F.GEAR ** 2:.3g}"/>
        <geom name="{p}tire_{side}" class="utire" type="cylinder" size="{_f(r, (F.WHEEL_WIDTH_MM - 1) / 2 * MM)}" euler="0 90 0" mass="0.004"/>
        <geom class="uvisual" type="cylinder" size="{_f(r, (F.WHEEL_WIDTH_MM - 1) / 2 * MM)}" euler="0 90 0" rgba="{_f(*BLACK)}"/>
        <geom class="uvisual" type="mesh" mesh="u_wheel" euler="0 0 {0 if sign > 0 else 180}" rgba="{_f(*GREY)}"/>
      </body>'''
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(x * MM, y * MM, F.IR_Z_MM * MM)}" size="0.0015" group="4" rgba="1 0 0 0.6"/>'
                    for n, ((x, y), _) in F.IR_SENSORS.items())
    board = ''.join(f'<geom class="uvisual" type="mesh" mesh="u_pcb_{k}" material="u_{k}"/>' for k in ('top', 'bottom'))
    board += f'<geom class="uvisual" type="mesh" mesh="u_pcb_side" rgba="{_f(*GREEN)}"/>'
    skate_z = F.GROUND_Z_MM
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, -F.GROUND_Z_MM * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in F.COM_MM))}" mass="{F.MASS_KG - 0.008:.4f}" diaginertia="4e-5 3e-5 6e-5"/>
      <geom name="{p}hull" class="uhull" type="mesh" mesh="u_hull"/>
      <geom name="{p}skid" class="uskid" type="box" pos="{_f(0, F.SKATE_Y_MM * MM, (skate_z + 1.5) * MM)}" size="{_f(5 * MM, 5 * MM, 1.5 * MM)}"/>
      <geom class="uvisual" type="mesh" mesh="u_skate" pos="{_f(0, F.SKATE_Y_MM * MM, skate_z * MM)}" rgba="{_f(*WHITE)}"/>
      {board}{_parts()}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    """The maze (Ratatron's, sim/model.py) with Futura alone, for its firmware's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="futura_maze">
  <compiler angle="degree" inertiafromgeom="auto"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic" impratio="10" noslip_iterations="4"/>
  <visual>
    <global offwidth="1920" offheight="1080" azimuth="90" elevation="-55"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
  </visual>
  <default>
    <default class="wall"><geom type="box" group="{WALL_GROUP}" rgba="0.95 0.95 0.95 1" contype="1" conaffinity="2"/></default>
    <default class="wall_top"><geom type="box" group="0" rgba="0.85 0.05 0.05 1" contype="0" conaffinity="0"/></default>
    <default class="post"><geom type="box" group="{WALL_GROUP}" rgba="0.72 0.72 0.75 1" contype="1" conaffinity="2"/></default>
    <default class="post_top"><geom type="box" group="0" rgba="0.45 0.02 0.02 1" contype="0" conaffinity="0"/></default>
    {defaults_xml()}
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.25 0.27 0.3" rgb2="0.05 0.05 0.06" width="256" height="256"/>
    <material name="floor" rgba="0.02 0.02 0.02 1" specular="0.1" shininess="0.1"/>
    {assets_xml()}
  </asset>
  <worldbody>
    <light pos="{_f(centre, centre, 3)}" dir="0 0 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>
    <light pos="{_f(centre - 1, centre - 2, 2)}" dir="0.3 0.6 -0.7" diffuse="0.35 0.35 0.35" castshadow="false"/>
    {maze_xml(maze)}
    {robot_xml()}
    {hand_xml()}
  </worldbody>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
