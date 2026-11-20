"""Miniskybot 2 assembled as its OpenSCAD model's show_robot() puts it together (parts/Openscad/main.scad), its meshes
made by cad.py from the STL files its repository ships and from its OpenSCAD model's dimensions: its printed chassis,
battery holder and wheels (blue PLA; white in its photo), its two Futaba S3003, its wheels' black O-rings and 4-arm
horns, its Skymega's hex spacers, its servos' cables and its battery's wires; drawn here, as its photo, its parts list
and their data sheets show them: the Ø16 marble of its ball caster, its SRF02 ranger in the front's hole (its board
behind the front's face, its transducer out of it) and its Skymega board over its top plate (its blue board, its
ATmega328 in its socket, its headers and its servos' plugs, its power connector, LED and button; drawn plainly, not
from its files). Its frame: origin on the floor under its wheels' axle. Collisions: a box round its chassis and
battery holder, its ranger, its board, its marble, its tyres.
"""
from .. import config as C
from ..hand import hand_xml
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import cad
from . import config as S

# Its printed parts (chassis, battery holder, wheels) in blue PLA, RGB (25, 95, 200); the rest as in its photo
PRINTED = (0.10, 0.37, 0.78, 1)
BLACK = (0.05, 0.05, 0.05, 1)
SERVO = (0.09, 0.09, 0.10, 1)                   # the Futaba S3003's black case
RUBBER = (0.03, 0.03, 0.03, 1)                  # its O-rings
BLUE = (0.05, 0.13, 0.36, 1)                    # the Skymega's solder mask, darker than its printed parts
STEEL = (0.55, 0.56, 0.59, 1)
WIRE = {'black': (0.06, 0.06, 0.06, 1), 'red': (0.75, 0.08, 0.06, 1), 'white': (0.92, 0.92, 0.88, 1)}
GREEN = (0.10, 0.42, 0.20, 1)
GLASS = (0.78, 0.86, 0.92, 0.55)
CREAM = (0.93, 0.91, 0.84, 1)


# ---- its parts drawn -----------------------------------------------------------------------------------------------
def _box(c, half, rgba, cls='mvisual'):
    return (f'<geom class="{cls}" type="box" pos="{_f(*(v * MM for v in c))}" size="{_f(*(v * MM for v in half))}" '
            f'rgba="{_f(*rgba)}"/>')


def _rod(p0, p1, r, rgba, cls='mvisual'):
    a = ' '.join(f'{v * MM:.5f}' for v in (*p0, *p1))
    return f'<geom class="{cls}" type="cylinder" fromto="{a}" size="{r * MM:.5f}" rgba="{_f(*rgba)}"/>'


def _srf02():
    """Its SRF02 in the front's hole: its board (24 x 20 mm, as its data sheet gives it) right behind the front
    part's face, its transducer out of the hole to its front (17 mm over the board's back, as its data sheet gives
    it), its two screws' heads on the face."""
    x, y, z = S.SRF02_POS
    face = S.scad_to_robot(S.BATTERY_C1 + S.FRONT_C1, 0, 0)[1]
    inner = face - S.FRONT_THICKNESS
    return (_box((x, inner - 0.8 - 0.05, z), (10.0, 0.8, 12.0), GREEN)
            + _rod((x, inner - 0.05, z), (x, y - 0.4, z), S.ULTRASOUND_HOLE / 2 - 0.2, BLACK)
            + _rod((x, y - 0.4, z), (x, y, z), S.ULTRASOUND_HOLE / 2 - 1.5, (0.20, 0.20, 0.21, 1))
            + ''.join(_rod((x - dx, face, z + dz), (x - dx, face + 1.2, z + dz), 2.6, STEEL)    # its two screws
                      for dx, dz in ((-8, -9), (7, 9))))


def _skymega():
    """Its Skymega on four hex spacers (meshes): its board, its ATmega328 in its socket, its expansion port and ICSP
    headers, its eight servo headers with its servos' plugs in two of them, its power connector, test LED and button
    (the board's photo and pin table)."""
    cx, cy, top = cad.board_top()
    z0 = top - S.PCB_MM
    h = S.SKYMEGA_MM / 2
    g = [_box((cx, cy, z0 + S.PCB_MM / 2), (h, h, S.PCB_MM / 2), BLUE)]
    for dx in (-1, 1):
        for dy in (-1, 1):
            p = (cx + dx * S.SKYMEGA_DRILL, cy + dy * S.SKYMEGA_DRILL)
            g.append(_rod((*p, top), (*p, top + 2.0), 2.7, STEEL))        # its spacers' screws' heads
    g.append(_box((cx + 4, cy - 2, top + 1.5), (3.9, 18.0, 1.5), BLACK))   # DIP-28 socket and its ATmega328
    g.append(_box((cx + 4, cy - 2, top + 4.0), (3.6, 17.5, 1.0), (0.13, 0.13, 0.14, 1)))
    g.append(_box((cx - 6, cy + 17, top + 4.5), (10.0, 4.5, 4.5), BLACK))  # its expansion port (2 x 5, boxed)
    g.append(_box((cx + 17, cy + 13, top + 4.5), (4.5, 6.5, 4.5), BLACK))  # ICSP (2 x 3, boxed)
    hx, hy, n = cad.SERVO_HEADER['x0'], cad.SERVO_HEADER['y'], cad.SERVO_HEADER['n']
    g.append(_box((cx + hx + 1.27 * (n - 1), cy + hy, top + 1.25), (1.27 * n, 3.81, 1.25), BLACK))
    plugged = set(cad.SERVO_PLUGGED.values())
    for k in range(n):                                                     # its servo headers: 3 pins each
        px = cx + hx + 2.54 * k
        if k in plugged:
            g.append(_box((px, cy + hy, top + 2.5 + cad.PLUG_H / 2), (1.25, 3.8, cad.PLUG_H / 2), BLACK))
            continue
        for pin in range(3):
            g.append(_box((px, cy + hy + 2.54 * (pin - 1), top + 4.5), (0.32, 0.32, 3.2), (0.85, 0.75, 0.30, 1)))
    px, py = cx + cad.POWER[0], cy + cad.POWER[1]
    g.append(_box((px, py, top + 3.5), (2.5, 4.0, 3.5), CREAM))           # its 2-pin power connector
    g.append(_box((cx + 18, cy - 16, top + 1.75), (3.0, 3.0, 1.75), BLACK))   # test button
    g.append(_rod((cx + 18, cy - 16, top + 3.5), (cx + 18, cy - 16, top + 4.5), 1.75, (0.25, 0.25, 0.27, 1)))
    g.append(f'<geom class="mvisual" type="sphere" pos="{_f(*(v * MM for v in (cx + 21, cy - 21, top + 3)))}" '
             f'size="{1.6 * MM:.5f}" rgba="0.2 0.8 0.3 0.9"/>')                 # its green test LED
    g.append(_box((cx + 12, cy - 22, top + 1.8), (5.5, 2.3, 1.8), STEEL))   # its crystal (low profile)
    return ''.join(g)


def _caster():
    y = S.CASTER_Y_MM
    return (f'<geom class="mvisual" type="sphere" pos="{_f(0, y * MM, S.CASTER_R_MM * MM)}" '
            f'size="{S.CASTER_R_MM * MM:.5f}" rgba="{_f(*GLASS)}"/>')


def _wheel_visuals(s):
    """Its wheel's meshes: the printed wheel, its O-ring, its horn and its screws."""
    return ''.join(f'<geom class="mvisual" type="mesh" mesh="m_{k}_{s}" rgba="{_f(*rgba)}"/>'
                   for k, rgba in (('wheel', PRINTED), ('o_ring', RUBBER), ('horn', BLACK), ('horn_screws', STEEL)))


# ---- MuJoCo -----------------------------------------------------------------------------------------------------
def defaults_xml():
    return (f'<default class="mvisual"><geom group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="mhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="mcaster"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" condim="1" '
            'friction="0.02"/></default>'
            '<default class="mtire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" priority="1" '
            f'friction="{S.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    return ''.join(f'<mesh name="m_{k}" file="{p}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
                   for k, p in cad.build().items())


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    """Its body on the floor of a cell: its meshes and parts, its collisions, its wheels (bodies at the axle)."""
    p = prefix
    r = S.WHEEL_RADIUS_MM * MM
    wheels = ''
    for s, sign in (('l', -1), ('r', 1)):
        wheels += f'''
      <body name="{p}wheel_{s}" pos="{_f(sign * S.TRACK_MM / 2 * MM, 0, S.AXLE_Z_MM * MM)}">
        <joint name="{p}wheel_{s}" type="hinge" axis="-1 0 0" armature="{S.SERVO_ARMATURE:.3g}"/>
        <geom name="{p}tire_{s}" class="mtire" type="cylinder" size="{_f(r, S.WHEEL_OR_DIAM / 2 * MM)}" euler="0 90 0" mass="0.012"/>
        {_wheel_visuals(s)}
      </body>'''
    body = ''.join(f'<geom class="mvisual" type="mesh" mesh="m_{k}" rgba="{_f(*rgba)}"/>'
                   for k, rgba in (('printed', PRINTED), ('servos', SERVO), ('spacers', STEEL),
                                   *((f'wire_{c}', WIRE[c]) for c in WIRE)))
    lo = S.scad_to_robot(-S.REAR_C1 - S.BALL_SIZE / 2 - S.BALL_WALL - S.BALL_AIRGAP, S.REAR_C2, -S.BATTERY_C3 - S.BATTERY_TOP_GAP)
    hi = S.scad_to_robot(S.BATTERY_C1 + S.FRONT_C1, 0, S.SERVO_C1 + S.TOP_PLATE_THICKNESS)
    hull = _box([(a + b) / 2 for a, b in zip(lo, hi)], [abs(b - a) / 2 for a, b in zip(lo, hi)], (0, 0, 1, 0.1), 'mhull')
    hull = hull.replace('<geom ', f'<geom name="{p}hull" ', 1)
    x, y, z = S.SRF02_POS
    hull += _rod((x, y - S.ULTRASOUND_LENGTH, z), (x, y, z), S.ULTRASOUND_HOLE / 2, (0, 0, 1, 0.1), 'mhull')
    cx, cy, _ = S.scad_to_robot(S.SKYMEGA_X_SCAD, S.MID_Y_SCAD, 0)
    z0 = S.scad_to_robot(0, 0, S.SKYMEGA_BOTTOM_SCAD)[2]
    hull += _box((cx, cy, z0 + 4), (S.SKYMEGA_MM / 2, S.SKYMEGA_MM / 2, 4), (0, 0, 1, 0.1), 'mhull')
    caster = (f'<geom name="{p}caster" class="mcaster" type="sphere" pos="{_f(0, S.CASTER_Y_MM * MM, S.CASTER_R_MM * MM)}" '
              f'size="{S.CASTER_R_MM * MM:.5f}"/>')
    site = f'<site name="{p}srf02" pos="{_f(*(v * MM for v in S.SRF02_POS))}" size="0.002" group="4" rgba="1 0 0 0.6"/>'
    visuals = body + _srf02() + _skymega() + _caster()
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, 0.0005)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in S.COM_MM))}" mass="{S.MASS_KG - 0.024:.4f}" diaginertia="2.4e-4 2.0e-4 3.2e-4"/>
      {hull}{caster}{visuals}{site}{wheels}
    </body>'''


def scene_xml(maze):
    """The maze with Miniskybot alone, for its program's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="miniskybot_maze">
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
