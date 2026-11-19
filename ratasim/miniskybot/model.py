"""Miniskybot 2 assembled as its OpenSCAD model's show_robot() puts it together (parts/Openscad/main.scad), from the
STL files its repository ships: its printed chassis (exported upside down, ready to print: turned back), its battery
holder under it, its two Futaba S3003 and its two wheels on their horns; with what its STL files leave out, as its
photo and its parts list show them: the wheels' black O-rings and black 4-arm horns, the Ø16 marble of its ball
caster, its SRF02 ranger in the front's hole (fake_ultrasound.scad's board and transducer, black as in its photo) and
its Skymega board on four spacers over its top plate (its blue board, its ATmega328 in its socket, its headers, its
power connector, LED and button; drawn plainly, not from its files). Its frame: origin on the floor under its
wheels' axle. Collisions: a box round its chassis and battery holder, its ranger, its board, its marble, its tyres.
"""
import os
import struct

import numpy as np

from .. import config as C
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import config as S

MESHES = os.path.join(S.BUILD, 'meshes')
WHITE = (0.94, 0.94, 0.91, 1)                   # its printed parts (white ABS, its photo)
BLACK = (0.05, 0.05, 0.05, 1)
SERVO = (0.10, 0.10, 0.11, 1)
HOLDER = (0.16, 0.16, 0.18, 1)
BLUE = (0.10, 0.25, 0.62, 1)                    # the Skymega's solder mask
GREEN = (0.10, 0.42, 0.20, 1)
SILVER = (0.75, 0.76, 0.78, 1)
GLASS = (0.78, 0.86, 0.92, 0.55)
CREAM = (0.93, 0.91, 0.84, 1)


# ---- its meshes from its STL files ---------------------------------------------------------------------------------
def _read_stl(path):
    """An STL file's triangles (n, 3, 3): OpenSCAD's ASCII, or binary."""
    data = open(path, 'rb').read()
    if not data.lstrip().startswith(b'solid') or b'facet' not in data[:1000]:
        n = struct.unpack('<I', data[80:84])[0]
        rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
        return rec['v'].reshape(-1, 3, 3).astype(np.float64)
    v = [line.split()[1:4] for line in data.decode(errors='replace').splitlines() if line.strip().startswith('vertex')]
    return np.array(v, np.float64).reshape(-1, 3, 3)


def _write_stl(path, tris):
    tris = np.asarray(tris, np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', 9), ('a', '<u2')]))
    rec['v'] = tris.reshape(-1, 9)
    with open(path + '.part', 'wb') as f:
        f.write(b'ratasim miniskybot'.ljust(80, b' ') + struct.pack('<I', len(tris)) + rec.tobytes())
    os.replace(path + '.part', path)


def _rx(deg):
    c, s = (round(v) for v in (np.cos(np.radians(deg)), np.sin(np.radians(deg))))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def _to_robot(p):
    """Points of main.scad's frame in the robot's."""
    x, y, z = np.moveaxis(p, -1, 0)
    return np.stack(S.scad_to_robot(x, y, z), axis=-1)


def meshes():
    """Its meshes in MESHES (made again when model.py or config.py is newer): its parts in the robot's frame, its
    wheels in their own (their axle's middle, their axis x)."""
    names = ('chassis', 'battery_holder', 'servo_r', 'servo_l', 'wheel_r', 'wheel_l')
    path = {k: os.path.join(MESHES, f'{k}.stl') for k in names}
    sources = [__file__, S.__file__]
    if all(os.path.exists(p) for p in path.values()) and \
            min(map(os.path.getmtime, path.values())) > max(map(os.path.getmtime, sources)):
        return path
    os.makedirs(MESHES, exist_ok=True)
    stl = {k: _read_stl(os.path.join(S.MINISKYBOT_DIR, p)) for k, p in S.STL.items()}
    # its chassis as printed: rotate([0, 180, 0]) translate([0, 0, -servo_c1 - top_plate_thickness]), undone
    ch = stl['chassis'] * (-1, 1, -1) + (0, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)
    _write_stl(path['chassis'], _to_robot(ch))
    # its battery holder: battery() under the frame, its top battery_top_gap under the plate's bottom
    b = stl['battery_holder'] - (0, 0, S.BATTERY_C3 / 2)
    b = b + (S.BATTERY_C1 / 2, S.MID_Y_SCAD, -S.BATTERY_C3 / 2 - S.BATTERY_TOP_GAP)
    _write_stl(path['battery_holder'], _to_robot(b))
    # its servos: translate([servo_c4 / 2, servo_c3, servo_c1 / 2]) rotate([+-90, 0, 0])
    for side, deg, dy in (('r', 90, 0.0), ('l', -90, 0.1)):
        sv = stl['servo'] @ _rx(deg).T + (S.SERVO_C4 / 2, S.SERVO_C3 + dy, S.SERVO_C1 / 2)
        _write_stl(path[f'servo_{side}'], _to_robot(sv))
    # its wheels, about their own axle: rotate([+-90, 0, 0]) and the robot's axes (its axis along x)
    for side, deg in (('r', 90), ('l', -90)):
        w = stl['wheel'] @ _rx(deg).T                  # main.scad's frame, about the wheel's middle
        w = np.stack([-w[..., 1], w[..., 0], w[..., 2]], axis=-1)
        _write_stl(path[f'wheel_{side}'], w)
    return path


# ---- its parts drawn -----------------------------------------------------------------------------------------------
def _box(c, half, rgba, cls='mvisual'):
    return (f'<geom class="{cls}" type="box" pos="{_f(*(v * MM for v in c))}" size="{_f(*(v * MM for v in half))}" '
            f'rgba="{_f(*rgba)}"/>')


def _rod(p0, p1, r, rgba, cls='mvisual'):
    a = ' '.join(f'{v * MM:.5f}' for v in (*p0, *p1))
    return f'<geom class="{cls}" type="cylinder" fromto="{a}" size="{r * MM:.5f}" rgba="{_f(*rgba)}"/>'


def _srf02():
    """Its SRF02 in the front's hole: its board behind the front part's face, its transducer through it."""
    x, y, z = S.SRF02_POS
    base = y - S.ULTRASOUND_LENGTH
    face = S.scad_to_robot(S.BATTERY_C1 + S.FRONT_C1, 0, 0)[1]
    bx, by, bz = S.ULTRASOUND_BASE
    return (_box((x, base - bz / 2, z), (by / 2, bz / 2, bx / 2), GREEN)
            + _rod((x, base, z), (x, y - 0.4, z), S.ULTRASOUND_HOLE / 2 - 0.2, BLACK)
            + _rod((x, y - 0.4, z), (x, y, z), S.ULTRASOUND_HOLE / 2 - 1.5, (0.22, 0.22, 0.23, 1))
            + ''.join(_rod((x - dx, face, z + dz), (x - dx, face + 1.0, z + dz), 2.6, SILVER)    # its two screws
                      for dx, dz in ((-8, -9), (7, 9))))


def _skymega():
    """Its Skymega on four spacers: its board, its ATmega328 in its socket, its expansion port and ICSP headers,
    its eight servo connectors, its power connector, test LED and button (the board's photo and pin table)."""
    cx, cy, _ = S.scad_to_robot(S.SKYMEGA_X_SCAD, S.MID_Y_SCAD, 0)
    z0 = S.scad_to_robot(0, 0, S.SKYMEGA_BOTTOM_SCAD)[2]
    top = z0 + S.PCB_MM
    h = S.SKYMEGA_MM / 2
    g = [_box((cx, cy, z0 + S.PCB_MM / 2), (h, h, S.PCB_MM / 2), BLUE)]
    plate = S.scad_to_robot(0, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)[2]
    for dx in (-1, 1):
        for dy in (-1, 1):
            p = (cx + dx * S.SKYMEGA_DRILL, cy + dy * S.SKYMEGA_DRILL)
            g.append(_rod((*p, plate), (*p, z0), 2.5, WHITE))              # its spacers
            g.append(_rod((*p, top), (*p, top + 2.0), 2.7, SILVER))        # their screws' heads
    g.append(_box((cx + 4, cy - 2, top + 1.5), (3.9, 18.0, 1.5), BLACK))   # DIP-28 socket and its ATmega328
    g.append(_box((cx + 4, cy - 2, top + 4.0), (3.6, 17.5, 1.0), (0.13, 0.13, 0.14, 1)))
    g.append(_box((cx - 6, cy + 17, top + 4.5), (10.0, 4.5, 4.5), BLACK))  # its expansion port (2 x 5, boxed)
    g.append(_box((cx + 17, cy + 13, top + 4.5), (4.5, 6.5, 4.5), BLACK))  # ICSP (2 x 3, boxed)
    for k in range(4):                                                     # its servo connectors: 2 rows of 4 x 3 pins
        for row in (-1, 1):
            g.append(_box((cx - 17 + row * 2.6, cy - 12 + k * 4.2, top + 1.25), (1.27, 1.9, 1.25), BLACK))
            for pin in range(3):
                py = cy - 12 + k * 4.2 + (pin - 1) * 1.27
                g.append(_box((cx - 17 + row * 2.6, py, top + 4.5), (0.32, 0.32, 3.2), (0.85, 0.75, 0.30, 1)))
    g.append(_box((cx - 20, cy + 10, top + 3.5), (2.5, 4.0, 3.5), CREAM))  # its 2-pin power connector
    g.append(_box((cx + 18, cy - 16, top + 1.75), (3.0, 3.0, 1.75), BLACK))   # test button
    g.append(_rod((cx + 18, cy - 16, top + 3.5), (cx + 18, cy - 16, top + 4.5), 1.75, (0.25, 0.25, 0.27, 1)))
    g.append(f'<geom class="mvisual" type="sphere" pos="{_f(*(v * MM for v in (cx + 21, cy - 21, top + 3)))}" '
             f'size="{1.6 * MM:.5f}" rgba="0.2 0.8 0.3 0.9"/>')                 # its green test LED
    g.append(_box((cx + 12, cy - 22, top + 1.8), (5.5, 2.3, 1.8), SILVER))  # its crystal (low profile)
    return ''.join(g)


def _caster():
    y = S.CASTER_Y_MM
    return (f'<geom class="mvisual" type="sphere" pos="{_f(0, y * MM, S.CASTER_R_MM * MM)}" '
            f'size="{S.CASTER_R_MM * MM:.5f}" rgba="{_f(*GLASS)}"/>')


def _wheel_visuals(s):
    sign = 1 if s == 'r' else -1
    w = S.WHEEL_WIDTH_MM / 2
    g = [f'<geom class="mvisual" type="mesh" mesh="m_wheel_{s}" rgba="{_f(*WHITE)}"/>',
         _rod((-S.WHEEL_OR_DIAM / 2, 0, 0), (S.WHEEL_OR_DIAM / 2, 0, 0), S.WHEEL_RADIUS_MM, BLACK),   # its O-ring
         _rod((-S.WHEEL_OR_DIAM / 2 - 0.05, 0, 0), (S.WHEEL_OR_DIAM / 2 + 0.05, 0, 0), S.WHEEL_RADIUS_MM - 3.0, WHITE)]
    # its 4-arm horn on its outer face, its screw
    x = sign * (w + 0.6)
    for a in (0, 90):
        g.append(f'<geom class="mvisual" type="box" pos="{_f(x * MM, 0, 0)}" size="{_f(0.6 * MM, 16 * MM, 2.6 * MM)}" '
                 f'euler="{a} 0 0" rgba="{_f(*BLACK)}"/>')
    g.append(_rod((x - 0.6 * sign, 0, 0), (x + 1.8 * sign, 0, 0), 5.0, BLACK))
    g.append(_rod((x + 1.8 * sign, 0, 0), (x + 2.3 * sign, 0, 0), 2.6, SILVER))
    return ''.join(g)


# ---- MuJoCo -----------------------------------------------------------------------------------------------------
def defaults_xml():
    return (f'<default class="mvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="mhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="mcaster"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" condim="1" '
            'friction="0.02"/></default>'
            '<default class="mtire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" priority="1" '
            f'friction="{S.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    return ''.join(f'<mesh name="m_{k}" file="{p}" scale="{_f(MM, MM, MM)}"/>' for k, p in meshes().items())


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    """Its body on the floor of a cell: its meshes and parts, its collisions, its wheels (bodies at the axle)."""
    p = prefix
    r = S.WHEEL_RADIUS_MM * MM
    wheels = ''
    for s, sign in (('l', -1), ('r', 1)):
        wheels += f'''
      <body name="{p}wheel_{s}" pos="{_f(sign * S.TRACK_MM / 2 * MM, 0, S.AXLE_Z_MM * MM)}">
        <joint name="{p}wheel_{s}" type="hinge" axis="-1 0 0" armature="{S.SERVO_ARMATURE:.3g}"/>
        <geom name="{p}tire_{s}" class="mtire" type="cylinder" size="{_f(r, S.WHEEL_OR_DIAM / 2 * MM)}" euler="0 90 0" mass="0.012"/>
        {_wheel_visuals(s)}
      </body>'''
    body = ''.join(f'<geom class="mvisual" type="mesh" mesh="m_{k}" rgba="{_f(*rgba)}"/>'
                   for k, rgba in (('chassis', WHITE), ('battery_holder', HOLDER), ('servo_r', SERVO),
                                   ('servo_l', SERVO)))
    lo = S.scad_to_robot(-S.REAR_C1 - S.BALL_SIZE / 2 - S.BALL_WALL - S.BALL_AIRGAP, S.REAR_C2, -S.BATTERY_C3 - S.BATTERY_TOP_GAP)
    hi = S.scad_to_robot(S.BATTERY_C1 + S.FRONT_C1, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)
    hull = _box([(a + b) / 2 for a, b in zip(lo, hi)], [abs(b - a) / 2 for a, b in zip(lo, hi)], (0, 0, 1, 0.1), 'mhull')
    hull = hull.replace('<geom ', f'<geom name="{p}hull" ', 1)
    x, y, z = S.SRF02_POS
    hull += _rod((x, y - S.ULTRASOUND_LENGTH, z), (x, y, z), S.ULTRASOUND_HOLE / 2, (0, 0, 1, 0.1), 'mhull')
    cx, cy, _ = S.scad_to_robot(S.SKYMEGA_X_SCAD, S.MID_Y_SCAD, 0)
    z0 = S.scad_to_robot(0, 0, S.SKYMEGA_BOTTOM_SCAD)[2]
    hull += _box((cx, cy, z0 + 4), (S.SKYMEGA_MM / 2, S.SKYMEGA_MM / 2, 4), (0, 0, 1, 0.1), 'mhull')
    caster = (f'<geom name="{p}caster" class="mcaster" type="sphere" pos="{_f(0, S.CASTER_Y_MM * MM, S.CASTER_R_MM * MM)}" '
              f'size="{S.CASTER_R_MM * MM:.5f}"/>')
    site = f'<site name="{p}srf02" pos="{_f(*(v * MM for v in S.SRF02_POS))}" size="0.002" group="4" rgba="1 0 0 0.6"/>'
    visuals = body + _srf02() + _skymega() + _caster()
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, 0.0005)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in S.COM_MM))}" mass="{S.MASS_KG - 0.024:.4f}" diaginertia="2.4e-4 2.0e-4 3.2e-4"/>
      {hull}{caster}{visuals}{site}{wheels}
    </body>'''


def scene_xml(maze):
    """The maze with Miniskybot alone, for its program's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="miniskybot_maze">
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
