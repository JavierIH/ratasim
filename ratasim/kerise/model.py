"""The MuJoCo scene with KERISE v2: the maze at half size (its firmware's: 90 mm cells, 6 mm walls 25 mm high) and
the robot: its board from its KiCad and Gerber files and its motors' mounts from its STL (cad.py), its motors,
gears, wheels and battery as simple shapes; for its collisions, boxes over its board's outline (walls only), its two
tyres and two skids under its board's front and back."""
import re

from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f
from . import cad
from . import config as K

BLACK, WHITE, MOTOR = '0.04 0.04 0.045 1', '0.92 0.92 0.9 1', '0.08 0.08 0.09 1'
BATTERY = '0.10 0.28 0.85 1'
# Its board's outline for collisions: boxes (x half-width, y from, y to), from its Edge.Cuts
OUTLINE = ((25.0, 10.0, 22.0), (19.0, 22.0, 30.0), (10.0, 30.0, 34.0),     # its nose (an arc of 25 mm)
           (12.0, -10.0, 10.0),                                            # between its wheels' cut-outs
           (25.0, -14.0, -10.0), (20.0, -24.0, -14.0))                     # its tail


def maze_xml(maze):
    """The maze at half size: posts and walls as boxes (6 mm), a red top on each."""
    cell, t, h, cap = K.CELL_MM * MM, K.WALL_MM * MM, K.WALL_H_MM * MM, 1.0 * MM
    geoms = []

    def block(name, cx, cy, sx, sy, kind):
        geoms.append(f'<geom name="{name}" class="{kind}" pos="{_f(cx, cy, (h - cap) / 2)}" '
                     f'size="{_f(sx / 2, sy / 2, (h - cap) / 2)}"/>')
        geoms.append(f'<geom class="{kind}_top" pos="{_f(cx, cy, h - cap / 2)}" size="{_f(sx / 2, sy / 2, cap / 2)}"/>')
    n = maze.size
    for px in range(n + 1):
        for py in range(n + 1):
            block(f'post_{px}_{py}', px * cell, py * cell, t, t, 'post')
    for x, y, kind in maze.segments():
        if kind == 'h':
            block(f'wall_h_{x}_{y}', (x + 0.5) * cell, y * cell, cell - t, t, 'wall')
        else:
            block(f'wall_v_{x}_{y}', x * cell, (y + 0.5) * cell, t, cell - t, 'wall')
    side = n * cell
    floor = (f'<geom name="floor" type="plane" pos="{_f(side / 2, side / 2, 0)}" size="{_f(side / 2 + 0.3, side / 2 + 0.3, 0.05)}" '
             f'material="floor" condim="3" friction="0.05 0.001 0.0001"/>')
    return floor + '\n'.join(geoms)


def defaults_xml():
    return (f'<default class="evisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="ehull"><geom group="3" contype="2" conaffinity="0" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="eskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.1 0.001 0.0001"/></default>'
            f'<default class="etire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="{K.TIRE_FRICTION} 0.005 0.0001" condim="3" solref="0.004 1"/></default>')


def assets_xml():
    out = ''
    for e in cad.meshes():
        out += f'<mesh name="e_{e["name"]}" file="{e["file"]}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
        if 'texture' in e:
            out += (f'<texture name="e_{e["name"]}" type="2d" file="{e["texture"]}"/>'
                    f'<material name="e_{e["name"]}" texture="e_{e["name"]}" specular="0.3" shininess="0.4"/>')
    return out


def _mesh(e, pos=(0, 0, 0)):
    look = f'material="e_{e["name"]}"' if 'texture' in e else f'rgba="{_f(*e["rgba"])}"'
    return f'<geom class="evisual" type="mesh" mesh="e_{e["name"]}" {look} pos="{_f(*(v * MM for v in pos))}"/>'


def _box(pos, half, rgba):
    return f'<geom class="evisual" type="box" pos="{_f(*(v * MM for v in pos))}" size="{_f(*(v * MM for v in half))}" rgba="{rgba}"/>'


def _rounded_box(pos, half, r, rgba):
    """A box with its edges rounded (radius r): three slabs, twelve edge cylinders, eight corner spheres."""
    (x, y, z), (a, b, c) = pos, half
    g = _box(pos, (a, b - r, c - r), rgba) + _box(pos, (a - r, b, c - r), rgba) + _box(pos, (a - r, b - r, c), rgba)
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                at = (x + sx * (a - r), y + sy * (b - r), z + sz * (c - r))
                g += f'<geom class="evisual" type="sphere" pos="{_f(*(v * MM for v in at))}" size="{r * MM:.5f}" rgba="{rgba}"/>'
    for sy in (-1, 1):
        for sz in (-1, 1):                                                       # along x
            g += (f'<geom class="evisual" type="cylinder" pos="{_f(x * MM, (y + sy * (b - r)) * MM, (z + sz * (c - r)) * MM)}" '
                  f'size="{_f(r * MM, (a - r) * MM)}" euler="0 90 0" rgba="{rgba}"/>')
    for sx in (-1, 1):
        for sz in (-1, 1):                                                       # along y
            g += (f'<geom class="evisual" type="cylinder" pos="{_f((x + sx * (a - r)) * MM, y * MM, (z + sz * (c - r)) * MM)}" '
                  f'size="{_f(r * MM, (b - r) * MM)}" euler="90 0 0" rgba="{rgba}"/>')
    for sx in (-1, 1):
        for sy in (-1, 1):                                                       # along z
            g += (f'<geom class="evisual" type="cylinder" pos="{_f((x + sx * (a - r)) * MM, (y + sy * (b - r)) * MM, z * MM)}" '
                  f'size="{_f(r * MM, (c - r) * MM)}" rgba="{rgba}"/>')
    return g


def _cyl_x(pos, r, half_len, rgba):
    return (f'<geom class="evisual" type="cylinder" pos="{_f(*(v * MM for v in pos))}" size="{_f(r * MM, half_len * MM)}" '
            f'euler="0 90 0" rgba="{rgba}"/>')


def robot_xml(start_xy=(0.5, 0.5), prefix='', cell_mm=K.CELL_MM):
    """The robot; with a prefix on its bodies', joints', geoms' and sites' names (for scenes with others)."""
    r, zb = K.WHEEL_RADIUS_MM, K.BOARD_Z_MM
    meshes = cad.meshes()
    body = ''.join(_mesh(e) for e in meshes if e['body'] == 'chassis')
    # its motors across it (1717: 17 mm, 17 mm long, its encoder 6 mm behind), from its mounts inwards
    zm = zb + K.MOUNT_MOTOR_MM
    for s, y in ((-1, -K.MOTOR_OFFSET_MM), (1, K.MOTOR_OFFSET_MM)):
        x0 = s * 8.5
        body += _cyl_x((x0 - s * 8.5, y, zm), 8.5, 8.5, MOTOR)
        body += _cyl_x((x0 - s * 20.0, y, zm), 8.0, 3.0, BLACK)
        body += _cyl_x((s * 13.5, y, zm), 1.6, 1.0, '0.75 0.72 0.6 1')          # its pinion, by the gear
    # its battery: two cells (LG325-0120) on its motors
    body += _rounded_box((0, 0, zm + 8.5 + 3.4), (12.5, 17.0, 3.4), 1.5, BATTERY)
    sites = ''.join(f'<site name="ir_{n}" pos="{_f(x * MM, y * MM, K.IR_Z_MM * MM)}" size="0.0012" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, ((x, y), _d) in K.IR_SENSORS.items())
    wheels = ''
    for side, s in (('l', -1), ('r', 1)):
        x = s * K.TRACK_X_MM
        visual = (_cyl_x((0, 0, 0), r, K.TIRE_WIDTH_MM / 2, BLACK) + _cyl_x((s * 0.2, 0, 0), r - 2.2, K.TIRE_WIDTH_MM / 2 + 0.2, WHITE)
                  + _cyl_x((-s * 5.5, 0, 0), 9.8, 1.5, WHITE))                # its spur gear inside
        wheels += f'''
      <body name="wheel_{side}" pos="{_f(x * MM, 0, r * MM)}">
        <joint name="wheel_{side}" type="hinge" axis="-1 0 0" armature="{K.WHEEL_ARMATURE:.3g}" frictionloss="{K.WHEEL_FRICTION_NM:.3g}"/>
        <geom name="tire_{side}" class="etire" type="cylinder" size="{_f(r * MM, K.TIRE_WIDTH_MM / 2 * MM)}" euler="0 90 0" mass="0.002"/>
        {visual}
      </body>'''
    skids = ''.join(f'<geom name="skid_{n}" class="eskid" type="sphere" size="{1.0 * MM:.6g}" '
                    f'pos="{_f(0, y * MM, (zb - K.PCB_MM - 0.4 + 1.0) * MM)}"/>' for n, y in (('front', 30), ('rear', -21)))
    z0, z1 = zb - K.PCB_MM, zm + 8.5 + 6.8
    hull = ''.join(f'<geom name="hull{"_" + str(i) if i else ""}" class="ehull" type="box" '
                   f'pos="{_f(0, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM)}" size="{_f(hx * MM, (y1 - y0) / 2 * MM, (z1 - z0) / 2 * MM)}"/>'
                   for i, (hx, y0, y1) in enumerate(OUTLINE))
    m = K.MASS_KG - 0.004
    xml = f'''
    <body name="robot" pos="{_f(start_xy[0] * cell_mm * MM, start_xy[1] * cell_mm * MM, 0)}">
      <freejoint name="robot"/>
      <inertial pos="{_f(*(v * MM for v in K.COM_MM))}" mass="{m:.4f}" diaginertia="{_f(1.2e-5, 1.0e-5, K.YAW_INERTIA)}"/>
      {hull}{skids}
      {body}{sites}{wheels}
    </body>'''
    return re.sub(r'( name| joint)="', rf'\1="{prefix}', xml) if prefix else xml


def scene_xml(maze):
    centre = maze.size * K.CELL_MM * MM / 2
    return f'''<mujoco model="kerise_maze">
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
    <light pos="{_f(centre, centre, 2)}" dir="0 0 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>
    <light pos="{_f(centre - 0.5, centre - 1, 1)}" dir="0.3 0.6 -0.7" diffuse="0.35 0.35 0.35" castshadow="false"/>
    {maze_xml(maze)}
    {robot_xml()}
    {hand_xml()}
  </worldbody>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
