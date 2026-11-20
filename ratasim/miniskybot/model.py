"""Miniskybot 2 assembled as its OpenSCAD model's show_robot() puts it together (parts/Openscad/main.scad), its meshes
made by cad.py from the STL files its repository ships and from its OpenSCAD model's dimensions: its printed chassis,
battery holder and wheels (white, as in its photo), its two Futaba S3003, its wheels' black O-rings and 4-arm
horns, its Skymega's hex spacers, its servos' cables and its battery's wires; drawn here, as its photo, its parts list
and their data sheets show them: the Ø16 marble of its ball caster and its SRF02 ranger in the front's hole (its board
behind the front's face, its transducer out of it); its Skymega board over its top plate as Obijuan published it
(skymega.py: its outline, its holes, its components and their 3D models from its KiCad files, its faces from its
Gerber files), with the plugs of its two servos and of its battery. Its frame: origin on the floor under its wheels' axle. Collisions: a box round its chassis and
battery holder, its ranger, its board, its marble, its tyres.
"""
import os

from .. import config as C
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as S

# Its printed parts (chassis, battery holder, wheels) in white printed plastic, a little warm and matt, not pure
# white: RGB (235, 232, 222); the rest as in its photo
PRINTED = (0.92, 0.91, 0.87, 1)
BLACK = (0.05, 0.05, 0.05, 1)
SERVO = (0.09, 0.09, 0.10, 1)                   # the Futaba S3003's black case
RUBBER = (0.03, 0.03, 0.03, 1)                  # its O-rings
EDGE = (0.07, 0.20, 0.50, 1)                    # its board's edge
STEEL = (0.55, 0.56, 0.59, 1)
WIRE = {'black': (0.06, 0.06, 0.06, 1), 'red': (0.75, 0.08, 0.06, 1), 'white': (0.92, 0.92, 0.88, 1)}
GREEN = (0.10, 0.42, 0.20, 1)
GLASS = (0.78, 0.86, 0.92, 0.55)
CREAM = (0.93, 0.90, 0.80, 1)                   # its power connector


# ---- its parts drawn -----------------------------------------------------------------------------------------------
def _box(c, half, rgba, cls='mvisual'):
    return (f'<geom class="{cls}" type="box" pos="{_f(*(v * MM for v in c))}" size="{_f(*(v * MM for v in half))}" '
            f'rgba="{_f(*rgba)}"/>')


def _rod(p0, p1, r, rgba, cls='mvisual'):
    a = ' '.join(f'{v * MM:.5f}' for v in (*p0, *p1))
    return f'<geom class="{cls}" type="cylinder" fromto="{a}" size="{r * MM:.5f}" rgba="{_f(*rgba)}"/>'


def _srf02():
    """Its SRF02 in the front's hole: its board (24 x 20 mm, as its data sheet gives it) right behind the front
    part's face, its transducer out of the hole to its front (17 mm over the board's back, as its data sheet gives
    it), its two screws' heads on the face, the metal clamp under one."""
    x, y, z = S.SRF02_POS
    face = S.scad_to_robot(S.BATTERY_C1 + S.FRONT_C1, 0, 0)[1]
    inner = face - S.FRONT_THICKNESS
    return (_box((x, inner - 0.8 - 0.05, z), (10.0, 0.8, 12.0), GREEN)
            + _rod((x, inner - 0.05, z), (x, y - 0.4, z), S.ULTRASOUND_HOLE / 2 - 0.2, BLACK)
            + _rod((x, y - 0.4, z), (x, y, z), S.ULTRASOUND_HOLE / 2 - 1.5, (0.20, 0.20, 0.21, 1))
            + ''.join(_rod((x - dx, face, z + dz), (x - dx, face + 1.2, z + dz), 2.6, STEEL)    # its two screws
                      for dx, dz in ((-8, -9), (7, 9)))
            + _box((x + 8, face + 0.3, z - 7.5), (3.5, 0.3, 4.5), STEEL))      # its metal clamp under one (its photo)


def _skymega():
    """Its Skymega on four hex spacers (meshes, under it and over it): its board's faces textured from its Gerber
    files, every component its KiCad board places with its 3D model (meshes, one per colour: skymega.py), its
    servos' plugs in two of its headers and its battery's plug on its power connector (their wires: cad.py)."""
    g = [f'<geom class="mvisual" type="mesh" mesh="m_skymega_{f}" material="skymega_{f}"/>' for f in ('top', 'bottom')]
    g.append(f'<geom class="mvisual" type="mesh" mesh="m_skymega_side" rgba="{_f(*EDGE)}"/>')
    g += [f'<geom class="mvisual" type="mesh" mesh="m_{name}" rgba="{_f(*c)}"/>' for name, _, c in cad.skymega_parts()]
    cx, cy, top = cad.board_top()
    for side in cad.SERVO_PLUGGED:                                           # a servo's plug: three pins wide
        px, py, pz = cad.plug(side)
        g.append(_box((px, py, pz - cad.PLUG_H / 2), (1.25, 3.8, cad.PLUG_H / 2), BLACK))
    bx, by = cad.power()
    g.append(_box((cx + bx, cy + by, top + cad.POWER_H + cad.BAT_PLUG_H / 2), (4.0, 2.6, cad.BAT_PLUG_H / 2), CREAM))
    return ''.join(g)


def _caster():
    y = S.CASTER_Y_MM
    return (f'<geom class="mvisual" type="sphere" pos="{_f(0, y * MM, S.CASTER_R_MM * MM)}" '
            f'size="{S.CASTER_R_MM * MM:.5f}" rgba="{_f(*GLASS)}"/>')


def _wheel_visuals(s):
    """Its wheel's meshes: the printed wheel, its O-ring, its horn and its screws."""
    return ''.join(f'<geom class="mvisual" type="mesh" mesh="m_{k}_{s}" rgba="{_f(*rgba)}"/>'
                   for k, rgba in (('wheel', PRINTED), ('o_ring', RUBBER), ('horn', BLACK), ('horn_screws', STEEL)))


# ---- MuJoCo -----------------------------------------------------------------------------------------------------
def defaults_xml():
    return (f'<default class="mvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="mhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="mcaster"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" condim="1" '
            'friction="0.02"/></default>'
            '<default class="mtire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" priority="1" '
            f'friction="{S.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    meshes = {**cad.build(), **{k: p for k, p, _ in cad.skymega_parts()}}
    tex = ''.join(f'<texture name="skymega_{f}" type="2d" file="{os.path.join(cad.MESHES, f"skymega_{f}.png")}"/>'
                  f'<material name="skymega_{f}" texture="skymega_{f}" specular="0.3" shininess="0.4"/>'
                  for f in ('top', 'bottom'))
    return tex + ''.join(f'<mesh name="m_{k}" file="{p}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
                         for k, p in meshes.items())


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
                   for k, rgba in (('printed', PRINTED), ('servos', SERVO), ('spacers', STEEL),
                                   *((f'wire_{c}', WIRE[c]) for c in WIRE)))
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
