"""NezumiBot (OPRobots, github.com/OPRobots/NezumiBot) assembled (assets/, from nezumi_3d): its printed parts in
black, its white cap with MARINE in blue, its electronics, its N20 wheels with Ø30 tyres, its two foam pads. Its frame:
origin between the wheels at the axle's height. Collisions: a box around its base and lid, its foam pads, its tyres.
"""
import json
import os

from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from .. import config as C
from . import config as N


def parts():
    """Its meshes: file, body (robot, wheel_l, wheel_r) and rgba."""
    return [dict(p, file=os.path.join(N.ASSETS, p['mesh'] + '.stl'))
            for p in json.load(open(os.path.join(N.ASSETS, 'parts.json')))]


def defaults_xml():
    return (f'<default class="nvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="nhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="nskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>'
            '<default class="ntire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{N.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    return ''.join(f'<mesh name="n_{k:02d}" file="{e["file"]}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
                   for k, e in enumerate(parts()))


def _box(lo, hi):
    c = [(a + b) / 2 * MM for a, b in zip(lo, hi)]
    h = [(b - a) / 2 * MM for a, b in zip(lo, hi)]
    return f'pos="{_f(*c)}" size="{_f(*h)}"'


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    """Its body on the floor of a cell: its meshes, its collision box and foam pads, its wheels (bodies at the axle,
    their meshes in the robot's frame)."""
    p, man = prefix, parts()

    def visual(k, e, at=(0, 0, 0)):
        return (f'<geom class="nvisual" mesh="n_{k:02d}" rgba="{_f(*e["rgba"])}" '
                f'pos="{_f(*(-v * MM for v in at))}"/>')
    chassis = ''.join(visual(k, e) for k, e in enumerate(man) if e['body'] == 'robot')
    r = N.WHEEL_RADIUS_MM * MM
    wheels = ''
    for side, sign in (('l', -1), ('r', 1)):
        at = (sign * N.CONTACT_X_MM, 0, N.AXLE_Z_MM)
        vis = ''.join(visual(k, e, at) for k, e in enumerate(man) if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="{p}wheel_{side}" pos="{_f(*(v * MM for v in at))}">
        <joint name="{p}wheel_{side}" type="hinge" axis="-1 0 0" armature="{N.MOTOR_ROTOR_INERTIA * N.GEAR ** 2:.3g}"/>
        <geom name="{p}tire_{side}" class="ntire" type="cylinder" size="{_f(r, N.WHEEL_WIDTH_MM / 2 * MM)}" euler="0 90 0" mass="0.004"/>
        {vis}
      </body>'''
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(*(v * MM for v in xyz))}" size="0.0015" group="4" rgba="1 0 0 0.6"/>'
                    for n, (xyz, _) in N.IR_SENSORS.items())
    skids = ''.join(f'<geom name="{p}skid_{n.split("_")[-1]}" class="nskid" type="box" {_box(s["min"], s["max"])}/>'
                    for n, s in N.SKIDS.items())
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, -N.GROUND_Z_MM * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in N.COM_MM))}" mass="{N.MASS_KG - 0.008:.4f}" diaginertia="6e-5 5e-5 8e-5"/>
      <geom name="{p}hull" class="nhull" type="box" {_box(N.HULL_BOX_MM["min"], N.HULL_BOX_MM["max"])}/>
      {skids}
      {chassis}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    """The maze (Ratatron's, sim/model.py) with NezumiBot alone, for its firmware's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="nezumibot_maze">
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

