"""The MuJoCo scene with Perenquen: the maze (ratasim/model.py) and the robot from its FreeCAD assembly (cad.py: its
meshes per body and colour, in perenquen/assets). Its four wheels: each side's two geared to its motor (an equality).
Collisions: its board (a box behind, a half disc in front, from the board up to its motor blocks), its tyres."""
import json
import os

from .. import config as C
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import config as P


def parts():
    return json.load(open(os.path.join(P.CAD_MESHES, 'manifest.json')))


def defaults_xml():
    return (f'<default class="pvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="phull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="ptire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{P.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    return ''.join(f'<mesh name="p_{k:02d}" file="{os.path.join(P.CAD_MESHES, e["file"])}" scale="{_f(MM, MM, MM)}" '
                   f'inertia="shell"/>' for k, e in enumerate(parts()))


def _cad(at=(0.0, 0.0, 0.0)):
    """The CAD frame (x forward, y left) in the robot's (x right, y forward), seen from a body at `at` (mm)."""
    ox, oz = P.CAD_TO_ROBOT
    return f'pos="{_f((0 - at[0]) * MM, (-ox - at[1]) * MM, (-oz - at[2]) * MM)}" euler="0 0 90"'


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    p, man = prefix, parts()

    def visual(k, e, at=(0.0, 0.0, 0.0)):
        return f'<geom class="pvisual" mesh="p_{k:02d}" rgba="{_f(*e["rgba"])}" {_cad(at)}/>'
    chassis = ''.join(visual(k, e) for k, e in enumerate(man) if e['body'] == 'chassis')
    r = P.WHEEL_RADIUS_MM * MM
    wheels = ''
    for side, sign in (('l', -1), ('r', 1)):
        for axle, cad, suffix in ((0.0, 'f', ''), (P.REAR_AXLE_Y, 'r', '2')):
            at = (sign * P.TIRE_Y_MM, axle, 0.0)
            vis = ''.join(visual(k, e, at) for k, e in enumerate(man) if e['body'] == f'wheel_{cad}{side}')
            wheels += f'''
      <body name="{p}wheel_{side}{suffix}" pos="{_f(*(v * MM for v in at))}">
        <joint name="{p}wheel_{side}{suffix}" type="hinge" axis="-1 0 0" armature="{P.MOTOR_ROTOR_INERTIA * P.GEAR ** 2 / 2:.3g}"/>
        <geom name="{p}tire_{side}{suffix}" class="ptire" type="cylinder" size="{_f(r, P.TIRE_WIDTH_MM / 2 * MM)}" euler="0 90 0" mass="0.003"/>
        {vis}
      </body>'''
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(x * MM, y * MM, P.IR_Z_MM * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, _, (x, y), _) in P.IR_SENSORS.items())
    b = P.BOARD
    cy = b['front'] - b['half_width']           # the half disc's centre
    z0, z1 = b['bottom'], P.TOP_MM
    hull = (f'<geom name="{p}hull" class="phull" type="box" pos="{_f(0, (b["back"] + cy) / 2 * MM, (z0 + z1) / 2 * MM)}" '
            f'size="{_f(b["half_width"] * MM, (cy - b["back"]) / 2 * MM, (z1 - z0) / 2 * MM)}"/>'
            f'<geom name="{p}hull_front" class="phull" type="cylinder" pos="{_f(0, cy * MM, (z0 + z1) / 2 * MM)}" '
            f'size="{_f(b["half_width"] * MM, (z1 - z0) / 2 * MM)}"/>')
    mass = P.MASS_KG - 4 * 0.003
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, (P.WHEEL_RADIUS_MM + 0.2) * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in P.COM_MM))}" mass="{mass:.4f}" diaginertia="{_f(7.0e-5, 4.2e-5, 1.0e-4)}"/>
      {hull}
      {chassis}{sites}{wheels}
    </body>'''


def equality_xml(prefix=''):
    """Each side's rear wheel geared to its front one (the same pinion between their gears)."""
    return ''.join(f'<joint joint1="{prefix}wheel_{s}2" joint2="{prefix}wheel_{s}"/>' for s in 'lr')


def scene_xml(maze):
    """The maze with Perenquen alone, for its firmware's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="perenquen_maze">
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
  <equality>{equality_xml()}</equality>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
