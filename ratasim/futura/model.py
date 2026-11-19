"""Futura assembled: its board (made here from the photo of its kit: a rectangle between the wheels and a half disc
for its nose), its printed wheels and skate (its own STL files, downloaded at build time), and simple shapes for the
rest as the photo shows them: its STM32F405, its HCMS-2903 display, its buzzer, its IR pairs on the arc, two motors
end to end in white printed mounts, its encoder boards, its battery. Its frame: origin between the wheels at the
axle's height. Collisions: its board (a box and a disc), its skate, its tyres.
"""
import math
import os
import struct

import numpy as np

from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from .. import config as C
from . import config as F

MESHES = os.path.join(F.BUILD, 'meshes')
GREEN = (0.05, 0.42, 0.18, 1)
WHITE = (0.93, 0.93, 0.9, 1)
BLACK = (0.04, 0.04, 0.04, 1)
GREY = (0.55, 0.55, 0.57, 1)
SILVER = (0.75, 0.76, 0.78, 1)


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


def board_outline(n=48):
    """Its board's outline (x, y) mm, counter-clockwise: the half disc from its right side round to its left, then
    the back."""
    b = F.BOARD
    arc = [(b['half_width'] * math.cos(math.pi * k / n), b['centre_y'] + b['half_width'] * math.sin(math.pi * k / n))
           for k in range(n + 1)]
    return arc + [(-b['half_width'], b['back']), (b['half_width'], b['back'])]


def _board_tris():
    b = F.BOARD
    pts = board_outline()
    z0, z1 = b['bottom'], b['bottom'] + b['thickness']
    c = (0.0, (b['back'] + b['centre_y']) / 2)
    tris = []
    for i in range(len(pts)):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % len(pts)]
        tris += [[(c[0], c[1], z1), (x0, y0, z1), (x1, y1, z1)], [(c[0], c[1], z0), (x1, y1, z0), (x0, y0, z0)],
                 [(x0, y0, z0), (x1, y1, z0), (x1, y1, z1)], [(x0, y0, z0), (x1, y1, z1), (x0, y0, z1)]]
    return tris


def meshes():
    """Its meshes in MESHES (once): its board, its wheel (centred, its axis x) and its skate (centred, on z = 0)."""
    path = {k: os.path.join(MESHES, f'{k}.stl') for k in ('board', 'wheel', 'skate')}
    if all(os.path.exists(p) for p in path.values()):
        return path
    F.download()
    os.makedirs(MESHES, exist_ok=True)
    _write_stl(path['board'], _board_tris())
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
    return ''.join(f'<mesh name="u_{k}" file="{p}" scale="{_f(MM, MM, MM)}"/>' for k, p in meshes().items())


def _box(c, half, rgba, extra=''):
    return (f'<geom class="uvisual" type="box" pos="{_f(*(v * MM for v in c))}" size="{_f(*(v * MM for v in half))}" '
            f'rgba="{_f(*rgba)}" {extra}/>')


def _cyl(c, r, half_len, axis, rgba):
    """A cylinder along axis (x, y, z)."""
    return (f'<geom class="uvisual" type="cylinder" pos="{_f(*(v * MM for v in c))}" size="{_f(r * MM, half_len * MM)}" '
            f'zaxis="{_f(*axis)}" rgba="{_f(*rgba)}"/>')


def _parts():
    """Its visible parts but the wheels: board, chips, display, buzzer, IR pairs, motors and mounts, battery."""
    b = F.BOARD
    top = b['bottom'] + b['thickness']
    out = f'<geom class="uvisual" type="mesh" mesh="u_board" rgba="{_f(*GREEN)}"/>'
    out += _box((0, 15, top + 0.7), (5, 5, 0.7), BLACK)                        # STM32F405 (LQFP64)
    out += _box((2, 27, top + 1.6), (10, 3.5, 1.6), (0.15, 0.15, 0.15, 1))     # HCMS-2903
    out += _box((2, 27, top + 3.25), (8.5, 2.2, 0.05), (0.95, 0.35, 0.1, 1))   # its dots (lit)
    out += _cyl((-16, 17, top + 3.5), 6, 3.5, (0, 0, 1), BLACK)               # buzzer
    out += _box((13, 15, top + 0.6), (2.5, 2.5, 0.6), BLACK)                  # LY3200ALH
    for (x, y), deg in F.IR_SENSORS.values():                                 # SFH4545 and TEFT4300 side by side
        a = math.radians(deg)
        for side in (-1, 1):
            cx, cy = x + side * 3.2 * -math.sin(a), y + side * 3.2 * math.cos(a)
            out += _cyl((cx, cy, F.IR_Z_MM), 2.6, 3.5, (math.cos(a), math.sin(a), 0), BLACK)
    for s in (-1, 1):                                                         # motors, end to end, and mounts
        out += _cyl((s * 16, 2, top + 6.5), 6.0, 9.0, (1, 0, 0), SILVER)
        out += _box((s * 27, 2, top + 6.5), (3.0, 7.5, 6.5), WHITE)
        out += _box((s * 5.5, 2, top + 6.5), (2.5, 7.5, 6.5), WHITE)
        out += _box((s * 31.5, 0, -4.0), (0.8, 9, 9), GREEN)                  # encoder boards
    out += _box((0, -15, top + 4.2), (15, 6, 4.2), (0.85, 0.5, 0.08, 1))      # 2S 120 mAh, its Kapton
    return out


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    p = prefix
    b = F.BOARD
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
    hull_z0, hull_z1 = b['bottom'], 8.0
    rect = ((-b['half_width'], b['back'], hull_z0), (b['half_width'], b['centre_y'], hull_z1))
    rc = [(u + v) / 2 * MM for u, v in zip(*rect)]
    rh = [(v - u) / 2 * MM for u, v in zip(*rect)]
    skate_z = F.GROUND_Z_MM
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, -F.GROUND_Z_MM * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in F.COM_MM))}" mass="{F.MASS_KG - 0.008:.4f}" diaginertia="4e-5 3e-5 6e-5"/>
      <geom name="{p}hull" class="uhull" type="box" pos="{_f(*rc)}" size="{_f(*rh)}"/>
      <geom name="{p}hull_front" class="uhull" type="cylinder" pos="{_f(0, b['centre_y'] * MM, (hull_z0 + hull_z1) / 2 * MM)}" size="{_f(b['half_width'] * MM, (hull_z1 - hull_z0) / 2 * MM)}"/>
      <geom name="{p}skid" class="uskid" type="box" pos="{_f(0, F.SKATE_Y_MM * MM, (skate_z + 1.5) * MM)}" size="{_f(5 * MM, 5 * MM, 1.5 * MM)}"/>
      <geom class="uvisual" type="mesh" mesh="u_skate" pos="{_f(0, F.SKATE_Y_MM * MM, skate_z * MM)}" rgba="{_f(*WHITE)}"/>
      {_parts()}{sites}{wheels}
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
