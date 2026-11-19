"""The MuJoCo scene with UKMARSBOT: the maze (ratasim/model.py) and the robot as its KiCad boards lay it out
(kicad.py): its two boards' outlines (Edge.Cuts) as plates, and on them, where their footprints are, its parts as
simple solids of their sizes: the Arduino Nano on its sockets, the TB6612FNG module, the gearmotors in their printed
brackets (their footprint's outline: encoder, motor, gearbox, shaft), the IR emitters and phototransistors along
their axes, the capacitors, resistors, switches and button; the PP3 battery standing behind the motors and the 32 mm
wheels where its photos and renders put them.
Collisions: the main board's outline as a convex hull up to the battery's top, its two tyres (2 mm wide, at its
firmware's MOUSE_RADIUS), its two skids.
"""
import math

from .. import config as C
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import config as U
from .kicad import Board

GREEN = (0.05, 0.42, 0.22, 1)
BLACK = (0.06, 0.06, 0.07, 1)
SILVER = (0.72, 0.72, 0.74, 1)
BRASS = (0.78, 0.66, 0.38, 1)
PRINTED = (0.10, 0.14, 0.34, 1)        # its brackets, dark blue (photos)
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


def _plate(poly, z0, z1):
    """A board: its outline extruded from z0 to z1 (mm), as a mesh's vertices and faces (m)."""
    n = len(poly)
    verts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    faces = []
    for a, b, c in _triangulate(poly):
        faces += [(a, c, b), (a + n, b + n, c + n)]
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, j + n), (i, j + n, i + n)]
    v = ' '.join(f'{x * MM:.5f} {y * MM:.5f} {z * MM:.5f}' for x, y, z in verts)
    f = ' '.join(f'{a} {b} {c}' for a, b, c in faces)
    return v, f


def _box(lo, hi, rgba):
    c = [(a + b) / 2 * MM for a, b in zip(lo, hi)]
    s = [abs(b - a) / 2 * MM for a, b in zip(lo, hi)]
    return f'<geom class="uvisual" type="box" pos="{_f(*c)}" size="{_f(*s)}" rgba="{_f(*rgba)}"/>'


def _rod(p0, p1, r, rgba, kind='cylinder'):
    a = ' '.join(f'{v * MM:.5f}' for v in (*p0, *p1))
    return f'<geom class="uvisual" type="{kind}" fromto="{a}" size="{r * MM:.5f}" rgba="{_f(*rgba)}"/>'


def _along(p, deg, d, z):
    a = math.radians(deg)
    return p[0] + d * math.cos(a), p[1] + d * math.sin(a), z


# ---- its parts ---------------------------------------------------------------------------------------------------
def _motor(sign):
    """A gearmotor in its bracket (the footprint's outline, the right one; sign -1 mirrors it to the left): its
    encoder board x 5.8-7.3, its motor can x 7.3-20 (12 wide), its gearbox x 20-31.5, its shaft to the wheel; the
    printed bracket over the gearbox x 23-31.5, its tabs to y +-13.2 with their screws."""
    z = U.MAIN_TOP + U.MOTOR_AXIS_OVER_BOARD
    t = U.MAIN_TOP

    def x(a, b):
        return sorted((sign * a, sign * b))
    g = []
    (a, b) = x(5.8, 7.3)
    g.append(_rod((a, 0, z), (b, 0, z), 6.0, GREEN))
    (a, b) = x(6.6, 7.3)
    g.append(_rod((a, 0, z), (b, 0, z), 2.0, BLACK))                    # its magnetic disc
    (a, b) = x(7.3, 20.0)
    g.append(_box((a, -6.0, z - 5.0), (b, 6.0, z + 5.0), SILVER))
    (a, b) = x(20.0, 23.0)
    g.append(_box((a, -6.0, z - 5.0), (b, 6.0, z + 5.0), BRASS))
    (a, b) = x(23.0, 31.5)
    g.append(_box((a, -7.5, t), (b, 7.5, z + 6.5), PRINTED))
    g.append(_box((a, -13.2, t), (b, 13.2, t + 2.5), PRINTED))
    for y in (-9.0, 9.0):
        g.append(_rod(((a + b) / 2, y, t + 2.5), ((a + b) / 2, y, t + 4.5), 2.6, SILVER))      # its M3 nuts
    (a, b) = x(31.5, U.MOUSE_RADIUS_MM - WHEEL_W / 2)
    g.append(_rod((a, 0, z), (b, 0, z), 1.5, SILVER))
    return ''.join(g)


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
    main, sensor = boards()
    t, st, sb = U.MAIN_TOP, U.SENSOR_TOP, U.SENSOR_BOTTOM
    g = ['<geom class="uvisual" type="mesh" mesh="u_mainboard" rgba="{}"/>'.format(_f(*GREEN)),
         '<geom class="uvisual" type="mesh" mesh="u_sensorboard" rgba="{}"/>'.format(_f(*GREEN))]
    # the Arduino Nano on its two sockets, its USB end at its pin 1 (x -33)
    nano = main.footprint(lib='Arduino_Nano')[0]
    x0, y0, x1, y1 = nano.bbox(('F.Fab',))
    nb = sb
    g.append(_box((x0, y0, nb), (x1, y1, nb + U.PCB_THICK), (0.08, 0.25, 0.62, 1)))
    for y in (y0 + 1.2, y1 - 1.2):
        g.append(_box((x0 + 1.5, y - 1.25, t), (x1 - 4.5, y + 1.25, nb), BLACK))
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    top = nb + U.PCB_THICK
    g.append(_box((cx - 3.5, cy - 3.5, top), (cx + 3.5, cy + 3.5, top + 1.0), BLACK))           # its ATmega328P
    g.append(_box((x0 - 1.5, cy - 3.8, top), (x0 + 7.5, cy + 3.8, top + 4.0), SILVER))          # its mini USB
    g.append(_box((cx + 7, cy - 2.2, top), (cx + 11, cy + 2.2, top + 1.5), SILVER))           # its crystal
    g.append(_box((x1 - 6.5, cy - 1.5, top), (x1 - 3.5, cy + 1.5, top + 1.5), (0.85, 0.85, 0.85, 1)))   # reset
    # the TB6612FNG module on its pins
    tb = main.footprint(lib='TB6612FNG-module')[0]
    x0, y0, x1, y1 = tb.bbox()
    g.append(_box((x0 + 0.5, y0 + 0.5, t + 2.5), (x1 - 0.5, y1 - 0.5, t + 2.5 + U.PCB_THICK), (0.78, 0.10, 0.12, 1)))
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    g.append(_box((cx - 4.0, cy - 2.6, t + 4.1), (cx + 4.0, cy + 2.6, t + 5.3), BLACK))
    for y in (y0 + 1.5, y1 - 1.5):
        g.append(_box((x0 + 1.0, y - 1.25, t), (x1 - 1.0, y + 1.25, t + 2.5), BLACK))
    # the sensor board's connector: a socket on the main board, its header
    j = main.footprint(lib='1X14_LOCK')[0]
    x0, y0, x1, y1 = j.bbox()
    g.append(_box((x0, y0 + 0.4, t), (x1, y1 - 0.4, sb), BLACK))
    # the gearmotors, the battery standing behind them (a rechargeable PP3: 48.5 x 26.5 x 17.5 mm), its leads' plug
    g += [_motor(1), _motor(-1)]
    g.append(_box((-24.75, -33.75, t), (23.75, -16.25, t + 26.5), (0.94, 0.94, 0.92, 1)))
    g.append(_box((-24.80, -33.80, t + 8), (23.80, -16.20, t + 18), (0.95, 0.55, 0.10, 1)))
    for x in (-6.0, 6.0):
        g.append(_rod((x, -25.0, t + 26.5), (x, -25.0, t + 29.0), 3.0, SILVER))
    # its switches, button, capacitors, resistors (main board)
    for f in main.footprints:
        if 'SW_DIP' in f.lib:
            x0, y0, x1, y1 = f.bbox()
            g.append(_box((x0 + 1.5, y0 + 1.5, t), (x1 - 1.5, y1 - 1.5, t + 3.5), (0.80, 0.10, 0.10, 1)))
            for k in range(4):
                y = y0 + 1.5 + (k + 0.5) * (y1 - y0 - 3) / 4
                g.append(_box(((x0 + x1) / 2 - 1.5, y - 0.6, t + 3.5), ((x0 + x1) / 2, y + 0.6, t + 4.3), SKID))
        elif 'TACTILE' in f.lib:
            x0, y0, x1, y1 = f.bbox(('F.Fab',))
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            g.append(_box((cx - 3, cy - 3, t), (cx + 3, cy + 3, t + 3.5), BLACK))
            g.append(_rod((cx, cy, t + 3.5), (cx, cy, t + 6.5), 1.8, (0.75, 0.08, 0.08, 1)))
        elif 'SW_MINI-SPDT' in f.lib:
            x0, y0, x1, y1 = f.bbox()
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            g.append(_box((cx - 3.5, cy - 4.0, t), (cx + 3.5, cy + 4.0, t + 9.0), (0.85, 0.15, 0.15, 1)))
            g.append(_rod((cx, cy, t + 9.0), (cx, cy - 2.0, t + 18.0), 1.2, SILVER, 'capsule'))
        elif 'CP_Radial_D8' in f.lib:
            p = f.point(1.25, 0)
            g.append(_rod((p[0], p[1], t), (p[0], p[1], t + 11.5), 4.0, (0.10, 0.12, 0.30, 1)))
        elif 'R_Axial' in f.lib:
            g.append(_resistor(f, t))
        elif 'Molex_KK' in f.lib:
            x0, y0, x1, y1 = f.bbox()
            g.append(_box((x0 + 0.5, y0 + 0.5, t), (x1 - 0.5, y1 - 0.5, t + 9.0), (0.92, 0.90, 0.85, 1)))
    # the sensor board: its emitters over it, its phototransistors under it, its capacitors, LED, resistors
    for f in sensor.footprints:
        if f.value == 'SFH4545':          # 5 mm, flange 5.45 mm from its pads, lens tip at 14.05
            z = st + 3.0
            g.append(_rod(f.point(1.27, 5.45) + (z,), f.point(1.27, 11.55) + (z,), 2.5, (0.80, 0.85, 0.92, 0.75)))
            g.append(_rod(f.point(1.27, 11.55) + (z,), f.point(1.27, 11.56) + (z,), 2.5, (0.80, 0.85, 0.92, 0.75),
                          'capsule'))
        elif f.value == 'TEFT4300':       # 5 mm, its body under the board
            z = sb - 3.0
            g.append(_rod(f.point(1.27, -1.27) + (z,), f.point(1.27, -7.37) + (z,), 2.5, (0.20, 0.20, 0.22, 1)))
            g.append(_rod(f.point(1.27, -7.37) + (z,), f.point(1.27, -7.38) + (z,), 2.5, (0.20, 0.20, 0.22, 1),
                          'capsule'))
        elif 'CP_Radial_D10' in f.lib:
            p = f.point(1.25, 0)
            g.append(_rod((p[0], p[1], st), (p[0], p[1], st + 12.5), 5.0, (0.08, 0.08, 0.10, 1)))
        elif f.value == 'LED':
            p = f.point(1.27, 0)
            g.append(_rod((p[0], p[1], st), (p[0], p[1], st + 4.0), 1.5, (0.90, 0.15, 0.10, 0.85), 'capsule'))
        elif 'TO-92' in f.lib:
            p = f.point(2.54, 0)
            g.append(_box((p[0] - 2.3, p[1] - 1.0, st), (p[0] + 2.3, p[1] + 1.0, st + 4.5), BLACK))
        elif 'R_Axial' in f.lib:
            g.append(_resistor(f, st))
    # its skids
    for y in (U.SKID_Y[0], U.SKID_Y[1]):
        g.append(_rod((0, y, U.MAIN_BOTTOM), (0, y, U.SKID_R + U.SKID_CLEARANCE_MM), U.SKID_R, SKID))
    return ''.join(g)


def _resistor(f, z):
    a, b = f.point(0, 0), f.point(7.62, 0)
    m = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    d = math.dist(a, b)
    ux, uy = (b[0] - a[0]) / d, (b[1] - a[1]) / d
    return _rod((m[0] - ux * 3.15, m[1] - uy * 3.15, z + 1.25), (m[0] + ux * 3.15, m[1] + uy * 3.15, z + 1.25), 1.25,
                (0.55, 0.70, 0.85, 1))


# ---- MuJoCo -----------------------------------------------------------------------------------------------------
def defaults_xml():
    return (f'<default class="uvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="uhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="utire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{U.TIRE_FRICTION} 0.005 0.0001"/></default>'
            '<default class="uskid"><geom group="3" contype="2" conaffinity="1" rgba="1 1 1 0.3" condim="1" '
            'friction="0.05" mass="0"/></default>')


def assets_xml():
    main, sensor = boards()
    out = ''
    for name, board, z0, z1 in (('mainboard', main, U.MAIN_BOTTOM, U.MAIN_TOP),
                                ('sensorboard', sensor, U.SENSOR_BOTTOM, U.SENSOR_TOP)):
        v, f = _plate(board.loops()[0], z0, z1)
        out += f'<mesh name="u_{name}" vertex="{v}" face="{f}" inertia="shell"/>'
    hull = ' '.join(f'{x * MM:.5f} {y * MM:.5f} {z * MM:.5f}' for x, y in main.loops()[0]
                    for z in (U.MAIN_BOTTOM, U.MAIN_TOP + 26.5))
    return out + f'<mesh name="u_hull" vertex="{hull}"/>'


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    p = prefix
    wheels = ''
    for s, sign in (('l', -1), ('r', 1)):
        r = U.WHEEL_RADIUS_MM[s]
        wheels += f'''
      <body name="{p}wheel_{s}" pos="{_f(sign * U.MOUSE_RADIUS_MM * MM, 0, r * MM)}">
        <joint name="{p}wheel_{s}" type="hinge" axis="-1 0 0" armature="2e-6"/>
        <geom name="{p}tire_{s}" class="utire" type="cylinder" size="{_f(r * MM, CONTACT_W / 2 * MM)}" euler="0 90 0" mass="0.004"/>
        {_wheel(s)}
      </body>'''
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(x * MM, y * MM, U.IR_Z_MM * MM)}" size="0.0015" group="4" '
                    f'rgba="1 0 0 0.6"/>' for n, (_, _, (x, y), _) in U.IR_SENSORS.items())
    skids = ''.join(f'<geom name="{p}skid_{k}" class="uskid" type="sphere" size="{U.SKID_R * MM:.5f}" '
                    f'pos="{_f(0, y * MM, (U.SKID_R + U.SKID_CLEARANCE_MM) * MM)}"/>'
                    for k, y in enumerate(U.SKID_Y))
    mass = U.MASS_KG - 2 * 0.004
    izz = U.INERTIA_Z - 2 * 0.004 * (U.MOUSE_RADIUS_MM * MM) ** 2
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, 0.0005)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in U.COM_MM))}" mass="{mass:.4f}" diaginertia="{_f(1.25e-4, 1.0e-4, izz)}"/>
      <geom name="{p}hull" class="uhull" type="mesh" mesh="u_hull"/>
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
