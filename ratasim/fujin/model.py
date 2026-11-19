"""The MuJoCo scene with Fujin: the same maze as Ratatron's (ratasim/model.py) and the robot from its STEP
(fujin/cad.py: every part's mesh, merged per body and colour, its board's faces textured with its photos, its
sensors' caps); simple shapes for its collisions: boxes over its board's outline (walls only: its board rides 2 mm
over the floor), its two tyres, two skids under its nose and tail."""
import json
import os
import re

from .. import config as RC
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as F

TIRE_SCALE = F.WHEEL_RADIUS_MM / F.TIRE_CAD_RADIUS_MM


def _meshes():
    return json.load(open(os.path.join(cad.ensure(), 'manifest.json')))


def _visual(entry, pos=(0, 0, 0)):
    look = f'material="f_{entry["file"][:-4]}"' if 'texture' in entry else f'rgba="{_f(*entry["rgba"])}"'
    return f'<geom class="fvisual" mesh="f_{entry["file"][:-4]}" {look} pos="{_f(*(v * MM for v in pos))}"/>'


def defaults_xml():
    return (f'<default class="fvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="fhull"><geom group="3" contype="2" conaffinity="0" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="fskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>'
            f'<default class="ftire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="{F.TIRE_FRICTION} 0.005 0.0001" condim="3" solref="0.004 1"/></default>')


def assets_xml():
    d = F.CAD_MESHES
    out = ''
    for e in _meshes():
        tyre = any(cad.TYRE in p for p in e['parts']) and e['body'] != 'chassis'
        scale = (MM, MM * TIRE_SCALE, MM * TIRE_SCALE) if tyre else (MM, MM, MM)
        out += f'<mesh name="f_{e["file"][:-4]}" file="{d}/{e["file"]}" scale="{_f(*scale)}" inertia="shell"/>'
        if 'texture' in e:          # its board's faces (board.py)
            out += (f'<texture name="f_{e["file"][:-4]}" type="2d" file="{d}/{e["texture"]}"/>'
                    f'<material name="f_{e["file"][:-4]}" texture="f_{e["file"][:-4]}" specular="0.3" shininess="0.4"/>')
    return out


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    """The robot; with a prefix on its bodies', joints', geoms' and sites' names (for scenes with others)."""
    r = F.WHEEL_RADIUS_MM
    meshes = _meshes()
    chassis = ''.join(_visual(e) for e in meshes if e['body'] == 'chassis')
    sites = ''.join(f'<site name="ir_{n}" pos="{_f(x * MM, y * MM, F.IR_Z_MM * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, (x, y), _d) in F.IR_SENSORS.items())
    wheels = ''
    for side, s in (('l', -1), ('r', 1)):
        x = s * 35.2            # the tyres' middle (the STEP: 30.9 to 39.4)
        visual = ''.join(_visual(e, (-x, 0, 0)) for e in meshes if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="wheel_{side}" pos="{_f(x * MM, 0, 0)}">
        <joint name="wheel_{side}" type="hinge" axis="-1 0 0" armature="{F.WHEEL_ARMATURE:.3g}" frictionloss="{F.WHEEL_FRICTION_NM:.3g}"/>
        <geom name="tire_{side}" class="ftire" type="cylinder" size="{_f(r * MM, 1.5 * MM)}" euler="0 90 0" mass="0.003"/>
        {visual}
      </body>'''
    skids = ''.join(f'<geom name="skid_{n}" class="fskid" type="sphere" size="{1.5 * MM:.6g}" '
                    f'pos="{_f(0, y * MM, (-r + 0.4 + 1.5) * MM)}"/>' for n, y in (('front', 46), ('rear', -66)))
    z0, z1 = F.PCB_BOTTOM_MM + 0.5, F.TOP_MM
    hull = ''.join(f'<geom name="hull{"_" + str(i) if i else ""}" class="fhull" type="box" '
                   f'pos="{_f(0, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM)}" size="{_f(hx * MM, (y1 - y0) / 2 * MM, (z1 - z0) / 2 * MM)}"/>'
                   for i, (hx, y0, y1) in enumerate(F.OUTLINE))
    m = F.MASS_KG - 0.006
    xml = f'''
    <body name="robot" pos="{_f(start_xy[0] * RC.CELL_MM * MM, start_xy[1] * RC.CELL_MM * MM, r * MM)}">
      <freejoint name="robot"/>
      <inertial pos="{_f(*(v * MM for v in F.COM_MM))}" mass="{m:.4f}" diaginertia="{_f(1.0e-4, 5e-5, F.YAW_INERTIA)}"/>
      {hull}{skids}
      {chassis}{sites}{wheels}
    </body>'''
    return re.sub(r'( name| joint)="', rf'\1="{prefix}', xml) if prefix else xml


def scene_xml(maze):
    centre = maze.size * RC.CELL_MM * MM / 2
    return f'''<mujoco model="fujin_maze">
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
