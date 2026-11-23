"""The MuJoCo scene with Ratatron II: the maze (ratasim/model.py) and the robot from its FreeCAD file (cad.py: its
meshes per body and colour, in ratatron2/assets). Its four wheels: each side's two geared to its motor's pinion (an
equality). Its LEDs: D2-D7 along its tail (led1-led6, its firmware's LED1-LED6), yellow, off or lit (set_leds), and
D1, its pilot, blue, lit. Collisions: boxes over its board's outline (Edge.Cuts), from under the board to its
battery's top, and its tyres."""
import json
import os

from .. import config as C
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import config as R

# Its LEDs (0603, water clear): the clear lens over a white body, or lit: D2-D7 yellow, D1 (its pilot) blue
LED_OFF = 'rgba="0.86 0.89 0.93 1" specular="0.9" shininess="0.95"'
LED_ON = 'rgba="1 0.72 0.02 1" emission="0.9" specular="0.3" shininess="0.8"'
PILOT_ON = 'rgba="0.03 0.22 1 1" emission="0.9" specular="0.3" shininess="0.8"'
# Its board's outline for collisions, boxes (x half-width, y from, y to), from its Edge.Cuts: 68 mm wide at its
# shoulders and tail, 44 between its wheels' cut-outs, its nose narrowing to 32 mm at its front edge
OUTLINE = ((22.0, -24.0, 24.0), (34.0, 24.0, 34.4), (32.0, 34.4, 40.0), (28.3, 40.0, 46.0), (24.3, 46.0, 52.0),
           (19.5, 52.0, 60.0), (34.0, -30.5, -24.0), (28.0, -36.0, -30.5))
TOP_MM = 18.5           # its battery's and its wheel supports' top over the board (the CAD's z)
WHEELS = (('l', '', 'fl'), ('r', '', 'fr'), ('l', '2', 'rl'), ('r', '2', 'rr'))   # joint side, suffix, CAD body


def parts():
    return json.load(open(os.path.join(R.ASSETS, 'manifest.json')))


def defaults_xml():
    return (f'<default class="r2visual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="r2hull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="r2tire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{R.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    return (f'<material name="r2_led_off" {LED_OFF}/><material name="r2_led_on" {LED_ON}/>'
            f'<material name="r2_pilot_on" {PILOT_ON}/>' +
            ''.join(f'<mesh name="r2_{e["file"][:-4]}" file="{os.path.join(R.ASSETS, e["file"])}" '
                    f'scale="{_f(MM, MM, MM)}" inertia="shell"/>' for e in parts()))


def robot_xml(start_xy=(0.5, 0.5), prefix='', leds=0):
    """The robot in the start cell (or at start_xy cells); with a prefix on its names (for scenes with others). Its
    LEDs lit as the bits of leds (bit 0 LED1)."""
    p, man, lift = prefix, parts(), R.FLOOR_TO_CAD_MM

    def visual(e, at=(0.0, 0.0, 0.0)):
        name = e['file'][:-4]
        body = e['body']
        pos = _f(-at[0] * MM, -at[1] * MM, (lift - at[2]) * MM)
        if body.startswith('led_'):
            n = body[4:]
            look = 'r2_pilot_on' if n == 'pilot' else 'r2_led_on' if leds >> (int(n) - 1) & 1 else 'r2_led_off'
            return f'<geom name="{p}led{n}" class="r2visual" mesh="r2_{name}" pos="{pos}" material="{look}"/>'
        return f'<geom class="r2visual" mesh="r2_{name}" rgba="{_f(*e["rgba"])}" pos="{pos}"/>'
    chassis = ''.join(visual(e) for e in man if e['body'] == 'robot' or e['body'].startswith('led_'))
    r = R.TIRE_RADIUS_MM * MM
    wheels = ''
    for side, suffix, cad in WHEELS:
        at = ((-1 if side == 'l' else 1) * R.TIRE_X_MM, (1 if suffix == '' else -1) * R.AXLE_Y_MM, R.TIRE_RADIUS_MM)
        vis = ''.join(visual(e, at) for e in man if e['body'] == f'wheel_{cad}')
        wheels += f'''
      <body name="{p}wheel_{side}{suffix}" pos="{_f(*(v * MM for v in at))}">
        <joint name="{p}wheel_{side}{suffix}" type="hinge" axis="-1 0 0" armature="{R.WHEEL_ARMATURE:.3g}" frictionloss="{R.WHEEL_FRICTION_NM:.3g}"/>
        <geom name="{p}tire_{side}{suffix}" class="r2tire" type="cylinder" size="{_f(r, R.TIRE_WIDTH_MM / 2 * MM)}" euler="0 90 0" mass="{R.WHEEL_KG}"/>
        {vis}
      </body>'''
    z0, z1 = lift - R.PCB_MM, lift + TOP_MM
    hull = ''.join(f'<geom name="{p}hull{"_" + str(i) if i else ""}" class="r2hull" type="box" '
                   f'pos="{_f(0, (y0 + y1) / 2 * MM, (z0 + z1) / 2 * MM)}" '
                   f'size="{_f(hx * MM, (y1 - y0) / 2 * MM, (z1 - z0) / 2 * MM)}"/>'
                   for i, (hx, y0, y1) in enumerate(OUTLINE))
    mass = R.MASS_KG - 4 * R.WHEEL_KG
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, 0.0002)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in R.COM_MM))}" mass="{mass:.4f}" diaginertia="{_f(*R.INERTIA)}"/>
      {hull}
      {chassis}{wheels}
    </body>'''


def equality_xml(prefix=''):
    """Each side's rear wheel geared to its front one (the same pinion between their gears)."""
    return ''.join(f'<joint joint1="{prefix}wheel_{s}2" joint2="{prefix}wheel_{s}"/>' for s in 'lr')


def scene_xml(maze, leds=0):
    """The maze with Ratatron II alone."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="ratatron2_maze">
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
    {robot_xml(leds=leds)}
  </worldbody>
  <equality>{equality_xml()}</equality>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''


def set_leds(m, mask, prefix=''):
    """Its LEDs D2-D7 lit as the bits of mask (bit 0 LED1), on a compiled model."""
    on, off = m.material('r2_led_on').id, m.material('r2_led_off').id
    for i in range(R.NUM_LEDS):
        m.geom_matid[m.geom(f'{prefix}led{i + 1}').id] = on if mask >> i & 1 else off
