"""The MuJoCo scene with Gamusino: the same maze as Ratatron's (sim/model.py) and the robot from its 3D model
(gamusino/cad.py: the meshes per body and material, the PCB's silkscreen as a texture), a site at each wall sensor's
lens; simple shapes for its collisions: a box hull over the PCB, the two tyres, the two ball casters."""
import json
import os

from .. import config as RC
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as G


def _cad():
    d = cad.ensure()
    return json.load(open(os.path.join(d, 'manifest.json'))), json.load(open(os.path.join(d, 'geometry.json')))


def _visual(e, pos=(0, 0, 0)):
    look = f'material="g_{e["material"]}"' if e['texture'] else f'rgba="{_f(*e["rgba"])}"'
    return f'<geom class="gvisual" mesh="g_{e["file"][:-4]}" {look} pos="{_f(*(v * MM for v in pos))}"/>'


def defaults_xml():
    return (f'<default class="gvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="ghull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="gskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>'
            f'<default class="gtire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="{G.TIRE_FRICTION} 0.005 0.0001" condim="3" solref="0.004 1"/></default>')


def assets_xml():
    manifest, _ = _cad()
    d = G.CAD_MESHES
    out = ''.join(f'<texture name="g_{t[:-4]}" type="2d" file="{d}/{t}"/>' for t in sorted({e['texture'] for e in manifest if e['texture']}))
    out += ''.join(f'<material name="g_{e["material"]}" texture="g_{e["texture"][:-4]}" specular="0.3" shininess="0.3"/>'
                   for e in {e['material']: e for e in manifest if e['texture']}.values())
    out += ''.join(f'<mesh name="g_{e["file"][:-4]}" file="{d}/{e["file"]}" scale="{_f(MM, MM, MM)}" inertia="shell"/>' for e in manifest)
    return out


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    manifest, geo = _cad()
    p = prefix
    r, sep = geo['wheel_radius'], geo['wheels_separation']
    chassis = ''.join(_visual(e) for e in manifest if e['body'] == 'chassis')
    wheels = ''
    for side, s in (('l', -1), ('r', 1)):
        x = s * sep / 2
        vis = ''.join(_visual(e, (-x, 0, 0)) for e in manifest if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="{p}wheel_{side}" pos="{_f(x * MM, 0, 0)}">
        <joint name="{p}wheel_{side}" type="hinge" axis="-1 0 0" armature="1e-6"/>
        <geom name="{p}tire_{side}" class="gtire" type="cylinder" size="{_f(r * MM, geo['tire_width'] / 2 * MM)}" euler="0 90 0" mass="0.003"/>
        {vis}
      </body>'''
    casters = ''.join(f'<geom name="{p}caster_{n}" class="gskid" type="sphere" size="{c["radius"] * MM:.6g}" '
                      f'pos="{_f(*(v * MM for v in c["pos"]))}"/>' for n, c in zip(('front', 'rear'), geo['casters']))
    (x0, y0, z0), (x1, y1, _) = geo['pcb']
    top = geo['robot'][1][2]
    hull = (f'<geom name="{p}hull" class="ghull" type="box" pos="{_f((x0 + x1) / 2 * MM, (y0 + y1) / 2 * MM, (z0 + top) / 2 * MM)}" '
            f'size="{_f((x1 - x0) / 2 * MM, (y1 - y0) / 2 * MM, (top - z0) / 2 * MM)}"/>')
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(x * MM, y * MM, G.IR_Z_MM * MM)}" size="0.001" group="4"/>'
                    for n, (_, _, (x, y), _) in G.IR_SENSORS.items())
    m = G.MASS_KG - 0.006
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * RC.CELL_MM * MM, start_xy[1] * RC.CELL_MM * MM, geo['axle_height'] * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in G.COM_MM))}" mass="{m:.4f}" diaginertia="{_f(4.5e-5 * m / 0.09, 3.0e-5 * m / 0.09, 7.0e-5 * m / 0.09)}"/>
      {hull}{casters}{sites}
      {chassis}{wheels}
    </body>'''


def scene_xml(maze):
    centre = maze.size * RC.CELL_MM * MM / 2
    return f'''<mujoco model="gamusino_maze">
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
  </worldbody>
  <actuator>
    <motor name="motor_l" joint="wheel_l"/>
    <motor name="motor_r" joint="wheel_r"/>
  </actuator>
</mujoco>'''
