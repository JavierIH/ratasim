"""The MuJoCo scene with KOGUMA: the same maze as Ratatron's (ratasim/model.py) and the robot from its STEP
(koguma/cad.py: every part's mesh, merged per body and colour, its carbon fibre plate's faces textured); simple
shapes for its collisions: its plate's outline from it up to its top (walls only), its two tyres, two skids under
its plate's nose and tail (its plate rides 2.2 mm over the floor; its nose's skid a felt pad in its photo: their
contacts overdamped, so that its nose dipping as it brakes does not bounce its wheels off the floor)."""
import json
import os
import re

from .. import config as RC
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as K

TIRE_SCALE = K.WHEEL_RADIUS_MM / K.TIRE_CAD_RADIUS_MM     # its tyres drawn at its firmware's radius


def _meshes():
    return json.load(open(os.path.join(cad.ensure(), 'manifest.json')))


def _name(entry):
    return 'kg_' + entry['file'][:-4]


def _visual(entry, pos=(0, 0, 0)):
    look = f'material="{_name(entry)}"' if 'texture' in entry else f'rgba="{_f(*entry["rgba"])}"'
    return f'<geom class="kgvisual" mesh="{_name(entry)}" {look} pos="{_f(*(v * MM for v in pos))}"/>'


def defaults_xml():
    return (f'<default class="kgvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="kghull"><geom group="3" contype="2" conaffinity="0" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="kgskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001" solref="0.01 3"/></default>'
            f'<default class="kgtire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="{K.TIRE_FRICTION} 0.005 0.0001" condim="3" solref="0.004 1"/></default>')


def hull_mesh_xml():
    """Its collision hull against the walls: its plate's outline (K.PLATE_OUTLINE) from its plate up to its top."""
    z0, z1 = K.PLATE_Z_MM[0], K.TOP_MM
    pts = [(x, y, z) for z in (z0, z1) for x, y in K.PLATE_OUTLINE]
    return f'<mesh name="kg_hull" vertex="{" ".join(_f(*(c * MM for c in p)) for p in pts)}"/>'


def assets_xml():
    d = K.CAD_MESHES
    out = hull_mesh_xml()
    for e in _meshes():
        tyre = any(k in p for p in e['parts'] for k in cad.TYRE) and e['body'] != 'chassis'
        scale = (MM, MM * TIRE_SCALE, MM * TIRE_SCALE) if tyre else (MM, MM, MM)
        out += f'<mesh name="{_name(e)}" file="{d}/{e["file"]}" scale="{_f(*scale)}" inertia="shell"/>'
        if 'texture' in e:          # its carbon fibre plate (cad.py: a tile of its weave, repeated)
            out += (f'<texture name="{_name(e)}" type="2d" file="{d}/{e["texture"]}"/>'
                    f'<material name="{_name(e)}" texture="{_name(e)}" specular="0.6" shininess="0.7" reflectance="0.04"/>')
    return out


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    """The robot; with a prefix on its bodies', joints', geoms' and sites' names (for scenes with others)."""
    r = K.WHEEL_RADIUS_MM
    meshes = _meshes()
    chassis = ''.join(_visual(e) for e in meshes if e['body'] == 'chassis')
    sites = ''.join(f'<site name="ir_{n}" pos="{_f(x * MM, y * MM, K.IR_Z_MM * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, (x, y), _d) in K.IR_SENSORS.items())
    wheels = ''
    for side, s in (('l', -1), ('r', 1)):
        x = s * K.TRACK_MM / 2
        visual = ''.join(_visual(e, (-x, 0, 0)) for e in meshes if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="wheel_{side}" pos="{_f(x * MM, 0, 0)}">
        <joint name="wheel_{side}" type="hinge" axis="-1 0 0" armature="{K.WHEEL_ARMATURE:.3g}" damping="{K.WHEEL_VISCOUS:.4g}" frictionloss="{K.WHEEL_FRICTION_NM:.4g}"/>
        <geom name="tire_{side}" class="kgtire" type="ellipsoid" size="{_f(3.0 * MM, r * MM, r * MM)}" mass="0.002"/>
        {visual}
      </body>'''
    z_skid = K.PLATE_Z_MM[0]
    skids = ''.join(f'<geom name="skid_{n}" class="kgskid" type="sphere" size="{1.5 * MM:.6g}" '
                    f'pos="{_f(0, y * MM, (z_skid + 1.5) * MM)}"/>' for n, y in (('front', 54), ('rear', -54)))
    hull = '<geom name="hull" class="kghull" type="mesh" mesh="kg_hull"/>'
    m = K.MASS_KG - 0.004
    xml = f'''
    <body name="robot" pos="{_f(start_xy[0] * RC.CELL_MM * MM, start_xy[1] * RC.CELL_MM * MM, r * MM)}">
      <freejoint name="robot"/>
      <inertial pos="{_f(*(v * MM for v in K.COM_MM))}" mass="{m:.4f}" diaginertia="{_f(5.5e-5, 2.5e-5, K.YAW_INERTIA - 4e-6)}"/>
      {hull}{skids}
      {chassis}{sites}{wheels}
    </body>'''
    return re.sub(r'( name| joint)="', rf'\1="{prefix}', xml) if prefix else xml


def tyre_pairs_xml(prefix=''):
    """Its tyres on the floor as explicit contacts whose friction gives a little (K.TYRE_CREEP: MuJoCo's
    solreffriction): a rubber tyre's slip under its lateral force in a turn, and under its drive."""
    if not K.TYRE_CREEP:
        return ''
    mu = K.TIRE_FRICTION
    return '<contact>' + ''.join(
        f'<pair geom1="{prefix}tire_{s}" geom2="floor" condim="3" friction="{mu} {mu} 0.005 0.0001 0.0001" '
        f'solref="0.004 1" solreffriction="{_f(*K.TYRE_CREEP)}"/>' for s in 'lr') + '</contact>'


def scene_xml(maze):
    centre = maze.size * RC.CELL_MM * MM / 2
    return f'''<mujoco model="koguma_maze">
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
  {tyre_pairs_xml()}
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
