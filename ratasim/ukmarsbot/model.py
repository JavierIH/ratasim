"""The MuJoCo scene with UKMARSBOT: the maze (ratasim/model.py) and the robot as its KiCad boards lay it out: its two
boards, their outlines (Edge.Cuts, kicad.py) and their faces textured from their Gerber files (pcb.py), and every
part their footprints place, their 3D models where KiCad puts them (cad.py): the Arduino Nano, the TB6612FNG module,
the gearmotors in their brackets with their encoders, the IR emitters and phototransistors, the capacitors,
resistors, diodes, headers, switches and button; the PP3 battery standing behind the motors and the 32 mm wheels,
which its boards do not place, where its photos and renders put them.
Collisions: the main board's outline as a convex hull up to the battery's top, its two tyres (2 mm wide, at its
firmware's MOUSE_RADIUS), its two skids.
"""
import json
import math
import os

from .. import config as C
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as U
from . import pcb
from .kicad import Board

EDGE = (0.55, 0.60, 0.42, 1)                # the boards' edges: FR4
BLACK = (0.06, 0.06, 0.07, 1)
SILVER = (0.72, 0.72, 0.74, 1)
SKID = (0.92, 0.92, 0.90, 1)
WHEEL_W = 7.0                          # Pololu 32 x 7 mm wheels
CONTACT_W = 2.0                        # the tyres' collision: their contact centred where its firmware's turns
                                       # put it (MOUSE_RADIUS), not at their edges
_cache = {}


def boards():
    if 'b' not in _cache:
        _cache['b'] = Board(U.MAINBOARD), Board(U.SENSOR_BOARD)
    return _cache['b']


# ---- geometry ----------------------------------------------------------------------------------------------------
def _triangulate(poly):
    """Ear clipping of a simple counter-clockwise polygon: index triangles."""
    idx = list(range(len(poly)))
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0
    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cross(a, b, c) <= 1e-9:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            break
    if len(idx) == 3:
        tris.append(tuple(idx))
    return tris


def _box(lo, hi, rgba):
    c = [(a + b) / 2 * MM for a, b in zip(lo, hi)]
    s = [abs(b - a) / 2 * MM for a, b in zip(lo, hi)]
    return f'<geom class="kvisual" type="box" pos="{_f(*c)}" size="{_f(*s)}" rgba="{_f(*rgba)}"/>'


def _rod(p0, p1, r, rgba, kind='cylinder'):
    a = ' '.join(f'{v * MM:.5f}' for v in (*p0, *p1))
    return f'<geom class="kvisual" type="{kind}" fromto="{a}" size="{r * MM:.5f}" rgba="{_f(*rgba)}"/>'


# ---- its parts ---------------------------------------------------------------------------------------------------
def _meshes():
    return json.load(open(os.path.join(cad.ensure(), 'manifest.json')))


def _wheel(s):
    """Its wheel's visuals in its body (the axle's centre): a black tyre, a white rim, its hub."""
    r = U.WHEEL_RADIUS_MM[s]
    sign = -1 if s == 'l' else 1
    w = WHEEL_W / 2
    g = [_rod((-w, 0, 0), (w, 0, 0), r, BLACK),
         _rod((-w - 0.2, 0, 0), (w + 0.2, 0, 0), r - 3.2, (0.93, 0.93, 0.92, 1)),
         _rod((sign * (w + 0.2), 0, 0), (sign * (w + 0.6), 0, 0), 4.0, (0.85, 0.85, 0.84, 1))]
    for k in range(3):                       # three spokes' openings, dark (photos)
        a = math.radians(90 + k * 120)
        y, z = 7.0 * math.cos(a), 7.0 * math.sin(a)
        g.append(_rod((sign * (w + 0.1), y, z), (sign * (w + 0.3), y, z), 3.2, (0.25, 0.25, 0.26, 1)))
    return ''.join(g)


def _chassis():
    t = U.MAIN_TOP
    g = [f'<geom class="kvisual" type="mesh" mesh="k_{b}_{face}" material="k_{b}_{face}"/>'
         for b in pcb.BOARDS for face in ('top', 'bottom')]
    g += [f'<geom class="kvisual" type="mesh" mesh="k_{b}_side" rgba="{_f(*EDGE)}"/>' for b in pcb.BOARDS]
    g += [f'<geom class="kvisual" type="mesh" mesh="k_{e["file"][:-4]}" rgba="{_f(*e["rgba"])}"/>' for e in _meshes()]
    # the battery standing behind the motors (a rechargeable PP3: 48.5 x 26.5 x 17.5 mm), its leads' plug
    g.append(_box((-24.75, -33.75, t), (23.75, -16.25, t + 26.5), (0.94, 0.94, 0.92, 1)))
    g.append(_box((-24.80, -33.80, t + 8), (23.80, -16.20, t + 18), (0.95, 0.55, 0.10, 1)))
    for x in (-6.0, 6.0):
        g.append(_rod((x, -25.0, t + 26.5), (x, -25.0, t + 29.0), 3.0, SILVER))
    # its skids
    for y in (U.SKID_Y[0], U.SKID_Y[1]):
        g.append(_rod((0, y, U.MAIN_BOTTOM), (0, y, U.SKID_R + U.SKID_CLEARANCE_MM), U.SKID_R, SKID))
    return ''.join(g)


# ---- MuJoCo -----------------------------------------------------------------------------------------------------
def defaults_xml():
    return (f'<default class="kvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="khull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="ktire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{U.TIRE_FRICTION} 0.005 0.0001"/></default>'
            '<default class="kskid"><geom group="3" contype="2" conaffinity="1" rgba="1 1 1 0.3" condim="1" '
            'friction="0.05" mass="0"/></default>')


def assets_xml():
    main, _ = boards()
    d = cad.ensure()
    out = ''.join(f'<texture name="k_{b}_{face}" type="2d" file="{d}/{b}_{face}.png"/>'
                  f'<material name="k_{b}_{face}" texture="k_{b}_{face}" specular="0.3" shininess="0.4"/>'
                  for b in pcb.BOARDS for face in ('top', 'bottom'))
    out += ''.join(f'<mesh name="k_{b}_{part}" file="{d}/{b}_{part}.obj" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
                   for b in pcb.BOARDS for part in ('top', 'bottom', 'side'))
    out += ''.join(f'<mesh name="k_{e["file"][:-4]}" file="{d}/{e["file"]}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
                   for e in _meshes())
    hull = ' '.join(f'{x * MM:.5f} {y * MM:.5f} {z * MM:.5f}' for x, y in main.loops()[0]
                    for z in (U.MAIN_BOTTOM, U.MAIN_TOP + 26.5))
    return out + f'<mesh name="k_hull" vertex="{hull}"/>'


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    p = prefix
    wheels = ''
    for s, sign in (('l', -1), ('r', 1)):
        r = U.WHEEL_RADIUS_MM[s]
        wheels += f'''
      <body name="{p}wheel_{s}" pos="{_f(sign * U.MOUSE_RADIUS_MM * MM, 0, r * MM)}">
        <joint name="{p}wheel_{s}" type="hinge" axis="-1 0 0" armature="2e-6"/>
        <geom name="{p}tire_{s}" class="ktire" type="cylinder" size="{_f(r * MM, CONTACT_W / 2 * MM)}" euler="0 90 0" mass="0.004"/>
        {_wheel(s)}
      </body>'''
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(x * MM, y * MM, U.IR_Z_MM * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, _, (x, y), _) in U.IR_SENSORS.items())
    skids = ''.join(f'<geom name="{p}skid_{k}" class="kskid" type="sphere" size="{U.SKID_R * MM:.5f}" '
                    f'pos="{_f(0, y * MM, (U.SKID_R + U.SKID_CLEARANCE_MM) * MM)}"/>'
                    for k, y in enumerate(U.SKID_Y))
    mass = U.MASS_KG - 2 * 0.004
    izz = U.INERTIA_Z - 2 * 0.004 * (U.MOUSE_RADIUS_MM * MM) ** 2
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, 0.0005)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in U.COM_MM))}" mass="{mass:.4f}" diaginertia="{_f(1.25e-4, 1.0e-4, izz)}"/>
      <geom name="{p}hull" class="khull" type="mesh" mesh="k_hull"/>
      {skids}{_chassis()}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    """The maze with UKMARSBOT alone, for its firmware's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="ukmarsbot_maze">
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
