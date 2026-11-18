"""Bulebule's model: its board (pcb.py), its components (KiCad's 3D models, parts.py) where its layout and photos put
them, its modules (Blue Pill, GY-6500, DRV8833, HC-05, DC-DC converters) built from those parts on boards of their
sizes, its printed and mechanical parts (mounts, motors, gears, wheels, battery) from its CadQuery dimensions.
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
BLUE_PILL = (0.0, 31.8)             # its centre: midway between its pin rows (layout)
WHEELS = [(s, f) for s in (-1, 1) for f in (1, -1)]     # side (-1 left), front (+1) / rear (-1)
WHEEL_X = B.WHEELS_SEPARATION / 2
# The IR LEDs' pads on the dome (layout, mm); each pair an emitter (outer) and a receiver
LEDS = [(3.85, 50.4, 'rx'), (13.6, 47.2, 'tx'), (30.3, 39.0, 'rx'), (34.15, 29.7, 'tx')]


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


def place(tris, at=(0, 0, 0), *rotations):
    for axis, deg in rotations:
        tris = tris @ rot(axis, deg).T
    return tris + np.array(at)


# ---- the assembly ---------------------------------------------------------------------------------------------
GREY = {(210, 209, 199): (205, 205, 208), (228, 227, 207): (230, 228, 215)}     # KiCad's metal grey: tin


def build():
    groups = {}

    def add(tris, rgb, body='chassis'):
        groups.setdefault((body, tuple(rgb)), []).append(np.asarray(tris, float))

    def part(name, at, *rotations, recolour=None, body='chassis'):
        for rgb, tris in parts.load(name):
            add(place(tris, at, *rotations), (recolour or {}).get(rgb, GREY.get(rgb, rgb)), body)

    os.makedirs(OUT, exist_ok=True)
    pcb.build(OUT)
    bx, by = BLUE_PILL
    # The Blue Pill on its headers, the GY-6500 flat under it
    zb = PT + 2.5
    add(place(box(53.34, 22.86, 1.6), (bx, by, zb)), (22, 72, 170))
    top = zb + 1.6
    for s in (-1, 1):
        part('hdr20', (bx, by + s * 7.62, PT - 3.0), ('z', 90))
    part('lqfp48', (bx + 2, by, top), ('z', 45))
    part('hc49', (bx - 9, by - 6.5, top))
    part('usb', (bx - 24.6, by, top), ('z', 90))
    part('sot223', (bx - 15, by + 6, top), ('z', 180))
    part('hdr2x3', (bx + 15, by + 4, top), ('z', 90))
    for k in (0, 1):                                                       # its BOOT jumpers' caps
        add(place(box(2.5, 5.0, 6.0), (bx + 15 - 2.54 + 2.54 * 2 * k, by + 4, top + 2.6)), (230, 200, 30))
    part('tl3342', (bx + 13, by - 6.5, top))
    for dx in (-19, -16):
        part('led0805', (bx + dx, by - 8, top))
    for dx in (-5, 6, 9):
        part('c0805', (bx + dx, by + 7.5, top), ('z', 90))
    add(place(box(15, 25, 1.0), (bx - 14, by, PT)), (110, 45, 150))                 # GY-6500
    part('qfn24', (bx - 14, by, PT + 1.0))
    # The IR LEDs lying on the dome's edge, in black heat-shrink with their tips out
    for x, y, kind in LEDS:
        for s in (-1, 1):
            px, py = s * x, y
            heading = math.degrees(math.atan2(py - pcb.DOME_CY, px))            # outward, from the dome's centre
            colour = (200, 215, 235) if kind == 'tx' else (30, 30, 45)
            for rgb, tris in parts.load('led5'):
                t = place(tris, (0, 0, 2.9), ('x', -90))          # its axis along +y, its legs back, on the board
                add(place(t, (px, py, PT), ('z', heading - 90)), colour if rgb == (178, 25, 13) else (205, 205, 208))
            sleeve = place(cylinder(3.0, 6.0), (0, 5.4, 2.9), ('z', 90))
            add(place(sleeve, (px, py, PT), ('z', heading - 90)), (18, 18, 20))
            part('sot23', (s * (x - 3), y - 5.5, PT), ('z', heading))
            part('r0805', (s * (x - 4.5), y - 8.5, PT), ('z', heading))
    for s in (-1, 1):                                                         # its red LEDs on the shoulders
        part('led0805', (s * 33, 27.5, PT), recolour={(87, 173, 113): (220, 30, 30)})
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
        x0 = s * (B.WAIST + 1.4)
        add(place(cylinder(B.WHEEL_RADIUS, B.TIRE_W), (0, 0, 0), flip) + [x0, y, B.AXLE_Z] + local, (14, 14, 16), body)
        add(place(cylinder(B.RIM_D / 2, 0.4), (0, 0, 0), flip) + [x0 + s * B.TIRE_W, y, B.AXLE_Z] + local, (34, 34, 36), body)
        add(place(cylinder(3.1, 0.6), (0, 0, 0), flip) + [x0 + s * B.TIRE_W, y, B.AXLE_Z] + local, (200, 200, 205), body)
    bx_, by_, bz_ = B.BATTERY
    add(place(box(bx_, by_, bz_), (0, 0, B.MOTOR_Z + B.MOTOR_D / 2)), (26, 26, 30))
    add(place(box(bx_ - 6, by_ - 10, 0.2), (0, 0, B.MOTOR_Z + B.MOTOR_D / 2 + bz_)), (200, 200, 205))       # its label
    # Behind: the DRV8833 module on two header rows, the HC-05 standing, two DC-DC converters, buttons, switch, JST
    for yy in (-10.2, -26.7):
        part('hdr8', (0, yy, PT - 3.0), ('z', 90))
    zd = PT + 2.5
    add(place(box(20.3, 19.0, 1.2), (0, -18.45, zd)), (120, 40, 140))
    part('htssop16', (0, -18.45, zd + 1.2))
    for dx in (-6, 6):
        part('c0805', (dx, -14, zd + 1.2))
    hc = place(box(37.0, 15.5, 1.6), (0, 0, 0), ('x', 90)) + [0, -33.0, PT + 0.5]
    add(hc, (20, 80, 165))
    add(place(box(27.0, 13.0, 2.2), (0, 0, 0), ('x', 90)) + [0, -34.6, PT + 1.8], (25, 120, 150))
    add(place(box(20.0, 9.0, 0.2), (0, 0, 0), ('x', 90)) + [0, -36.85, PT + 3.8], (232, 232, 228))
    for s in (-1, 1):
        add(place(box(17.0, 11.0, 1.0), (s * 19, -31.0, PT)), (20, 100, 60))
        part('sot23_6', (s * 15, -31.0, PT + 1.0))
        part('srn4018', (s * 22, -31.0, PT + 1.0))
        part('push6', (s * 34.0, -31.0, PT))
    part('slide', (34.5, -24.5, PT), ('z', 180))
    part('jst2', (-21.0, -22.5, PT))
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
        <geom name="{p}tire_{name[6:]}" class="btire" type="cylinder" size="{_f(B.WHEEL_RADIUS * MM, B.TIRE_W / 2 * MM)}" euler="0 90 0" mass="0.004"/>
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
