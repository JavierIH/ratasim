"""The MuJoCo scene with ZoroBot3: the same maze as Ratatron's (sim/model.py) and the robot from its PCB and printed
parts (zorobot/assets.py), the rest (tires, gears, motors, battery, sensor boards) as primitives."""
import math

from .. import config as RC
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import assets
from . import config as Z

PCB_TOP = -Z.AXLE_OVER_PCB_MM                  # robot frame z of the board's top face
PCB_BOT = PCB_TOP - Z.PCB_THICK_MM
STL = (0.0, -Z.STL_AXLE_Y, PCB_TOP)             # the printed parts' frame in the robot's
FAN_XY = (Z.FAN_HOLE_KICAD[0] - Z.PCB_CENTRE_X, Z.AXLE_Y_KICAD - Z.FAN_HOLE_KICAD[1])
FAN_BRACKET_HOLE_X = 0.54                       # its motor hole in the brackets' frame (0.55 right of the board's)
WHEEL_X = Z.ROBOT_WIDTH / 2 - Z.TIRE_WIDTH_MM / 2
RIM_OUT_Y = -7.2                                # the wheel STL along its axis: rim face -7.2, hub end +4.3


def _mesh(name, rgba, pos=(0, 0, 0), euler=(0, 0, 0)):
    return (f'<geom class="zvisual" mesh="z_{name}" rgba="{_f(*rgba)}" pos="{_f(*(v * MM for v in pos))}" '
            f'euler="{_f(*euler)}"/>')


def _box(rgba, pos, half, zrot=0.0, name=None):
    n = f'name="{name}" ' if name else ''
    return (f'<geom {n}class="zvisual" type="box" rgba="{_f(*rgba)}" pos="{_f(*(v * MM for v in pos))}" '
            f'size="{_f(*(v * MM for v in half))}" euler="0 0 {zrot:.6g}"/>')


def _cyl(rgba, pos, r, half, euler=(0, 0, 0)):
    return (f'<geom class="zvisual" type="cylinder" rgba="{_f(*rgba)}" pos="{_f(*(v * MM for v in pos))}" '
            f'size="{_f(r * MM, half * MM)}" euler="{_f(*euler)}"/>')


BLACK, PURPLE, WHITE = (0.06, 0.06, 0.07, 1), (0.45, 0.25, 0.85, 1), (0.92, 0.92, 0.9, 1)
SILVER, YELLOW, BLUE = (0.75, 0.76, 0.78, 1), (0.95, 0.7, 0.15, 1), (0.6, 0.72, 0.95, 1)


def robot_xml(start_xy=(0.5, 0.5)):
    r = Z.WHEEL_RADIUS_MM
    my, mz = Z.MOTOR_AXIS
    ey, ez = Z.ENCODER_GEAR
    top = PCB_TOP + Z.FAN_BRACKET_Z
    v = [_mesh('board', (0.04, 0.04, 0.045, 1)), _mesh('silk', WHITE, pos=(0, 0, 0.01)),
         _mesh('bracket_l', BLACK, pos=STL), _mesh('bracket_r', BLACK, pos=STL),
         _mesh('fan_bracket', BLACK, pos=(FAN_XY[0] - FAN_BRACKET_HOLE_X, STL[1], top)),
         _mesh('fan', PURPLE, pos=(FAN_XY[0], FAN_XY[1], top - 1.84)),                       # under the bracket's boss
         _cyl(SILVER, (FAN_XY[0], FAN_XY[1], top + 8), 4.25, 10),                           # the fan's motor
         _box(YELLOW, (0, -10, PCB_TOP + 14.2 + 6.2), (19, 9, 6)),                           # 2S 260 mAh LiPo on top
         _box((0.85, 0.85, 0.8, 1), (0, -21, PCB_TOP + 14.2 + 4), (5, 2.5, 3)),                # its connector
         _box((0.9, 0.9, 0.9, 1), (0, Z.AXLE_Y_KICAD - 60.62, PCB_TOP + 0.8), (2.5, 2.5, 0.8), name='rgb')]
    for s in (-1, 1):
        v.append(_cyl(SILVER, (s * 12.8, my, mz), 4.25, 10, (0, 90, 0)))                    # 8520, through the bracket
        v.append(_cyl((0.85, 0.7, 0.3, 1), (s * Z.GEAR_X, my, mz), 2.4, 1.4, (0, 90, 0)))   # its pinion (8T)
        v.append(_mesh('enc_gear', BLUE, pos=(s * Z.GEAR_X, ey, ez), euler=(0, 0, 90)))      # encoder gear (22T)
        v.append(_box((0.1, 0.25, 0.12, 1), (s * (Z.GEAR_X + 4.0), ey, ez - 1), (0.8, 6, 7.5)))   # AS5145B board
    for name, (_, (x, y, z), deg) in Z.IR_SENSORS.items():     # sensor boards: emitter over receiver
        a = math.radians(deg)
        v.append(_box(BLACK, (x, y, PCB_TOP + 7.7), (3.5, 3.0, 7.7), zrot=deg - 90))
        for dz, col in ((4, (0.85, 0.85, 0.9, 1)), (-2, (0.05, 0.05, 0.05, 1))):
            v.append(f'<geom class="zvisual" type="sphere" rgba="{_f(*col)}" size="{2.5 * MM:.6g}" '
                     f'pos="{_f((x + 3.2 * math.cos(a)) * MM, (y + 3.2 * math.sin(a)) * MM, (PCB_TOP + 7.7 + dz) * MM)}"/>')
    sites = ''.join(f'<site name="ir_{n}" pos="{_f(x * MM, y * MM, (PCB_TOP + 7.7) * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, (x, y, z), _d) in Z.IR_SENSORS.items())
    wheels = ''
    for side, s in (('l', -1), ('r', 1)):
        wheels += f'''
      <body name="wheel_{side}" pos="{_f(s * Z.WHEELS_SEPARATION / 2 * MM, 0, 0)}">
        <joint name="wheel_{side}" type="hinge" axis="-1 0 0" armature="{Z.MOTOR_ROTOR_INERTIA * Z.GEAR ** 2:.3g}"/>
        <geom name="tire_{side}" class="ztire" type="cylinder" size="{_f(r * MM, 1.0 * MM)}" euler="0 90 0" mass="0.004"/>
        {_cyl(BLACK, (s * (WHEEL_X - Z.WHEELS_SEPARATION / 2), 0, 0), r, Z.TIRE_WIDTH_MM / 2, (0, 90, 0))}
        {_mesh('wheel', PURPLE, pos=(s * (Z.ROBOT_WIDTH / 2 + RIM_OUT_Y + 0.2 - Z.WHEELS_SEPARATION / 2), 0, 0), euler=(0, 0, 90 * s))}
        {_cyl(WHITE, (s * (Z.GEAR_X - Z.WHEELS_SEPARATION / 2), 0, 0), 10.5, 1.4, (0, 90, 0))}
      </body>'''
    skids = ''.join(f'<geom name="skid_{n}" class="zskid" type="sphere" size="{1.5 * MM:.6g}" '
                    f'pos="{_f(0, y * MM, (-Z.AXLE_HEIGHT_MM + 0.3 + 1.5) * MM)}"/>' for n, y in (('front', 40), ('rear', -38)))
    top = 26.0
    hull = (f'<geom name="hull" class="zhull" type="box" pos="{_f(0, (-43.9 + 13) / 2 * MM, (PCB_BOT + top) / 2 * MM)}" '
            f'size="{_f(35.1 * MM, (13 + 43.9) / 2 * MM, (top - PCB_BOT) / 2 * MM)}"/>'
            f'<geom name="hull_front" class="zhull" type="cylinder" pos="{_f(0, 13 * MM, (PCB_BOT + top) / 2 * MM)}" '
            f'size="{_f(35.0 * MM, (top - PCB_BOT) / 2 * MM)}"/>')
    i = Z.MASS_KG - 0.008
    return f'''
    <body name="robot" pos="{_f(start_xy[0] * RC.CELL_MM * MM, start_xy[1] * RC.CELL_MM * MM, Z.AXLE_HEIGHT_MM * MM)}">
      <freejoint name="robot"/>
      <inertial pos="{_f(*(v * MM for v in Z.COM_MM))}" mass="{i:.4f}" diaginertia="{_f(6.4e-5 * i / 0.077, 4.1e-5 * i / 0.077, 9.2e-5 * i / 0.077)}"/>
      {hull}{skids}
      {''.join(v)}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    a = assets.ensure()
    names = ('board', 'silk', 'bracket_l', 'bracket_r', 'fan_bracket', 'fan', 'wheel', 'enc_gear')
    mesh_assets = ''.join(f'<mesh name="z_{n}" file="{n}.stl" scale="0.001 0.001 0.001" inertia="shell"/>' for n in names)
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
