"""The MuJoCo scene with ZoroBot3: the same maze as Ratatron's (sim/model.py) and the robot from its Onshape assembly
(zorobot/cad.py: every part's mesh, merged per body and colour); simple shapes for its collisions."""
import json
import os

from .. import config as RC
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as Z

PCB_BOT = -Z.AXLE_OVER_PCB_MM - Z.PCB_THICK_MM
TIRE_SCALE = Z.WHEEL_RADIUS_MM / Z.TIRE_CAD_RADIUS_MM


def _meshes():
    return json.load(open(os.path.join(cad.ensure(Z.CAD_STEP, Z.CAD_MESHES), 'manifest.json')))


def _visual(entry, pos=(0, 0, 0)):
    return (f'<geom class="zvisual" mesh="z_{entry["file"][:-4]}" rgba="{_f(*entry["rgba"])}" '
            f'pos="{_f(*(v * MM for v in pos))}"/>')


def robot_xml(start_xy=(0.5, 0.5)):
    r = Z.WHEEL_RADIUS_MM
    meshes = _meshes()
    chassis = ''.join(_visual(e) for e in meshes if e['body'] == 'chassis')
    sites = ''.join(f'<site name="ir_{n}" pos="{_f(x * MM, y * MM, Z.IR_Z_MM * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, (x, y), _d) in Z.IR_SENSORS.items())
    wheels = ''
    for side, s in (('l', -1), ('r', 1)):
        x = s * Z.WHEELS_SEPARATION / 2
        visual = ''.join(_visual(e, (-x, 0, 0)) for e in meshes if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="wheel_{side}" pos="{_f(x * MM, 0, 0)}">
        <joint name="wheel_{side}" type="hinge" axis="-1 0 0" armature="{Z.MOTOR_ROTOR_INERTIA * Z.GEAR ** 2:.3g}"/>
        <geom name="tire_{side}" class="ztire" type="cylinder" size="{_f(r * MM, 1.0 * MM)}" euler="0 90 0" mass="0.004"/>
        {visual}
      </body>'''
    skids = ''.join(f'<geom name="skid_{n}" class="zskid" type="sphere" size="{1.5 * MM:.6g}" '
                    f'pos="{_f(0, y * MM, (-Z.AXLE_HEIGHT_MM + 0.3 + 1.5) * MM)}"/>' for n, y in (('front', 40), ('rear', -38)))
    # Collisions with the walls: the board's outline (a semicircle in front, 40.7 behind the axle) up to the fan's motor.
    cy, back, top = Z.BOARD_FRONT_MM - Z.ROBOT_WIDTH / 2, -Z.ROBOT_BACK_LENGTH, 18.0
    hull = (f'<geom name="hull" class="zhull" type="box" pos="{_f(0, (back + cy) / 2 * MM, (PCB_BOT + top) / 2 * MM)}" '
            f'size="{_f(Z.ROBOT_WIDTH / 2 * MM, (cy - back) / 2 * MM, (top - PCB_BOT) / 2 * MM)}"/>'
            f'<geom name="hull_front" class="zhull" type="cylinder" pos="{_f(0, cy * MM, (PCB_BOT + top) / 2 * MM)}" '
            f'size="{_f(Z.ROBOT_WIDTH / 2 * MM, (top - PCB_BOT) / 2 * MM)}"/>')
    i = Z.MASS_KG - 0.008
    return f'''
    <body name="robot" pos="{_f(start_xy[0] * RC.CELL_MM * MM, start_xy[1] * RC.CELL_MM * MM, Z.AXLE_HEIGHT_MM * MM)}">
      <freejoint name="robot"/>
      <inertial pos="{_f(*(v * MM for v in Z.COM_MM))}" mass="{i:.4f}" diaginertia="{_f(6.4e-5 * i / 0.077, 4.1e-5 * i / 0.077, 9.2e-5 * i / 0.077)}"/>
      {hull}{skids}
      {chassis}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    a = Z.CAD_MESHES
    mesh_assets = ''.join(
        f'<mesh name="z_{e["file"][:-4]}" file="{e["file"]}" inertia="shell" '
        f'scale="{_f(MM, *((MM * TIRE_SCALE,) * 2 if e["parts"] == ["Goma"] else (MM, MM)))}"/>' for e in _meshes())
    centre = maze.size * RC.CELL_MM * MM / 2
    return f'''<mujoco model="zorobot3_maze">
  <compiler meshdir="{a}" angle="degree" inertiafromgeom="auto"/>
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
    <default class="zvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>
    <default class="zhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>
    <default class="zskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>
    <default class="ztire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="{Z.TIRE_FRICTION} 0.005 0.0001" condim="3" solref="0.004 1"/></default>
  </default>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.25 0.27 0.3" rgb2="0.05 0.05 0.06" width="256" height="256"/>
    <material name="floor" rgba="0.02 0.02 0.02 1" specular="0.1" shininess="0.1"/>
    {mesh_assets}
  </asset>
  <worldbody>
    <light pos="{_f(centre, centre, 3)}" dir="0 0 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>
    <light pos="{_f(centre - 1, centre - 2, 2)}" dir="0.3 0.6 -0.7" diffuse="0.35 0.35 0.35" castshadow="false"/>
    {maze_xml(maze)}
    {robot_xml()}
  </worldbody>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
