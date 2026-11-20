"""Futura assembled: its board from its Gerber files (pcb.py: its outline, its faces textured with its copper, pads,
silkscreen and holes), its parts where its pads and holes are (their sizes their packages'): its STM32F405, its
HCMS-2903 display, its buzzer, its gyro, regulators and H-bridges, its capacitors and resistors (every pair of
0603 and 0805 pads), its IR emitters and phototransistors lying along the axes their holes give, its buttons, header,
switch and battery connectors; its two motors end to end in white printed mounts screwed through its four holes,
its encoder boards standing at its wheels' notches, its printed wheels and skate (their own STL files, downloaded at
build time) and its battery behind the motors, where the photo of its kit shows them. Its frame: origin between the
wheels at the axle's height. Collisions: its board's outline as a convex hull up to its mounts' top, its skate, its
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
from . import pcb

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
MOTOR_Z = TOP + 6.5                             # its motors' axis (the photo: their cans just over the board)
MOUNT_TOP = TOP + 13.0                          # its printed mounts' top


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
    path = {k: os.path.join(MESHES, f'{k}.stl') for k in ('wheel', 'skate')}
    path.update({k: os.path.join(MESHES, f'{k}.obj') for k in ('pcb_top', 'pcb_bottom', 'pcb_side')})
    made = [os.path.join(MESHES, n) for n in ('top.png', 'bottom.png')] + list(path.values())
    sources = [pcb.__file__, F.__file__]
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
    return path


def defaults_xml():
    return (f'<default class="uvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="uhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="uskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>'
            '<default class="utire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{F.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    path = meshes()
    out = ''.join(f'<texture name="u_{t}" type="2d" file="{MESHES}/{t}.png"/>'
                  f'<material name="u_{t}" texture="u_{t}" specular="0.3" shininess="0.4"/>' for t in ('top', 'bottom'))
    out += ''.join(f'<mesh name="u_{k}" file="{p}" scale="{_f(MM, MM, MM)}"{shell}/>'
                   for k, p in path.items() for shell in [' inertia="shell"' if p.endswith('.obj') else ''])
    hull = ' '.join(f'{x * MM:.5f} {y * MM:.5f} {z * MM:.5f}' for x, y in pcb.outline()
                    for z in (F.BOARD_BOTTOM_MM, MOUNT_TOP))
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


def _motors():
    """Its two motors end to end over its H-bridges, their cans (12 mm, its photo) from the middle out into its
    printed mounts, screwed through the board's four holes; its encoder boards upright at the notches."""
    out = ''
    for s in (-1, 1):
        out += _rod((s * 1.5, 0, MOTOR_Z), (s * 13.0, 0, MOTOR_Z), 6.0, SILVER)
        out += _rod((s * 0.8, 0, MOTOR_Z), (s * 1.5, 0, MOTOR_Z), 4.0, BLACK)                # its brushes' cap
        out += _box((s * 20.0, 0, (TOP + MOUNT_TOP) / 2), (7.5, 6.5, (MOUNT_TOP - TOP) / 2), WHITE)
        for hx, hy in F.MOTOR_MOUNT_HOLES:
            if hx * s > 0:
                out += _rod((hx, hy, TOP), (hx, hy, TOP + 5.0), 3.0, WHITE)
                out += _rod((hx, hy, TOP + 5.0), (hx, hy, TOP + 5.8), 1.9, SILVER)            # its screw
        x0 = s * (F.NOTCH_X_MM + 0.8)
        out += _box((x0, 0, -1.5), (0.8, 10.0, 10.5), GREEN)                                 # its encoder board
        out += _box((x0 + s * 1.2, 0, 0), (0.4, 2.5, 3.0), BLACK)                            # AS5304, at its ring
    out += _box((0, 9.0, TOP + 2.0), (14.85, 1.5, 2.0), WHITE)                                # its mounts' bridge
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
    out += _on_board(0.0, -22.5, (30.0, 12.0, 7.0), KAPTON)                                   # 2S 120 mAh
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
        <geom class="uvisual" type="mesh" mesh="u_wheel" rgba="{_f(*GREY)}"/>
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
