"""NezumiBot (OPRobots, github.com/OPRobots/NezumiBot): Ratatron's printed chassis in black (it is based on Ratatron)
with its own cap (3d_model/stl_files/gorra.stl in its repository) and the word written on it in blue marker, as in
its photos, the two screws standing on it, its N20 wheels (Rueda.stl). Collisions: Ratatron's hull and skids.

    python -m ratasim.nezumibot.model        (builds its meshes into build/)
"""
import json
import math
import os
import struct
import urllib.request

import numpy as np

from .. import config as C, paths
from ..model import MM, ROBOT_GROUP, WALL_GROUP, _f, maze_xml
from . import config as N
from ..bulebule.model import cylinder, place

OUT = N.BUILD
REPO = N.NEZUMI_DIR
CAP_STL = os.path.join(REPO, '3d_model', 'stl_files', 'gorra.stl')
WHEEL_STL = os.path.join(REPO, '3d_model', 'stl_files', 'Rueda.stl')
WHEEL_AXIS = (207.1, 15.1)                  # Rueda.stl: its axis (x, y), along z, its outer face at z 0
FONT_URL = 'https://github.com/google/fonts/raw/main/apache/permanentmarker/PermanentMarker-Regular.ttf'
WORD, INK = 'MARINE', (30, 70, 200)
BLACK_PLA = (0.07, 0.07, 0.08, 1)
CHASSIS_TOP, CHASSIS_FRONT = 36.0, 45.0     # Ratatron's chassis in its CAD frame (mm): the cap sits there
PANEL = np.array([0.0, 0.839, 0.544])       # the normal of the cap's flat front strip, over its peak (STL)


def read_stl(path):
    d = open(path, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in d.decode(errors='ignore').splitlines()
             if ln.strip().startswith('vertex')]
        return np.array(v, float).reshape(-1, 3, 3)
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].astype(float).reshape(-1, 3, 3)


def write_stl(path, tris):
    tris = np.asarray(tris, np.float32).reshape(-1, 3, 3)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)) + rec.tobytes())


def cap():
    """The cap's triangles in Ratatron's CAD frame: its peak forward, on the chassis' top."""
    t = read_stl(CAP_STL)
    v = t.reshape(-1, 3)
    lo, hi = v.min(axis=0), v.max(axis=0)
    return t - np.array([(lo[0] + hi[0]) / 2, hi[1] - CHASSIS_FRONT, lo[2] - CHASSIS_TOP + 1.0])


def write_cap(tris, obj, png):
    """The cap as an OBJ whose flat front strip (68 x 16 mm, over its peak) carries the word, centred, read from the
    front (a planar projection on the strip); the rest white."""
    from PIL import Image, ImageDraw, ImageFont
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    panel = n @ PANEL > 0.98
    up = np.cross([1.0, 0, 0], PANEL)                                   # up the strip
    pts = tris[panel].reshape(-1, 3)
    u0, u1 = pts[:, 0].min(), pts[:, 0].max()
    w0, w1 = (pts @ up).min(), (pts @ up).max()
    font = os.path.join(OUT, 'marker.ttf')
    if not os.path.exists(font):
        urllib.request.urlretrieve(FONT_URL, font)
    W, H = 1024, int(1024 * (w1 - w0) / (u1 - u0))
    im = Image.new('RGB', (W, H), (238, 238, 236))
    g = ImageDraw.Draw(im)
    size = int(H * 0.7)
    while True:                                                         # as big as fits: 60 % of its width
        f = ImageFont.truetype(font, size)
        x0, y0, x1, y1 = g.textbbox((0, 0), WORD, font=f)
        if x1 - x0 <= 0.6 * W and y1 - y0 <= 0.62 * H:
            break
        size -= 2
    g.text((W / 2, H / 2), WORD, font=f, fill=INK, anchor='mm')
    im.save(png)
    with open(obj, 'w') as fh:
        for k, tri in enumerate(tris):
            for p in tri:
                fh.write(f'v {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n')
                if panel[k]:
                    fh.write(f'vt {(u1 - p[0]) / (u1 - u0):.5f} {(p @ up - w0) / (w1 - w0):.5f}\n')   # read from the front
                else:
                    fh.write('vt 0.002 0.002\n')                    # a corner of the image: plain white
            a = 3 * k + 1
            fh.write(f'f {a}/{a} {a + 1}/{a + 1} {a + 2}/{a + 2}\n')


def build():
    os.makedirs(OUT, exist_ok=True)
    from ..model import PARTS
    manifest = []
    for p in PARTS:                                     # Ratatron's meshes; its yellow chassis printed in black
        if p['body'] != 'robot':
            continue                                    # Ratatron's wheels: NezumiBot has its own
        rgba = BLACK_PLA if tuple(p['rgba'][:3]) == (1.0, 1.0, 0.0) else p['rgba']
        manifest.append({'file': os.path.join(C.ASSETS, p['mesh'] + '.stl'), 'body': p['body'], 'rgba': list(rgba)})
    w = read_stl(WHEEL_STL) - [WHEEL_AXIS[0], WHEEL_AXIS[1], 0.0]
    for side, s in (('l', -1), ('r', 1)):               # in its wheel body's frame: its outer face outwards
        t = np.stack([-s * w[..., 2] + s * (N.WHEEL_OUTER_X_MM - N.CONTACT_X_MM), w[..., 1], s * w[..., 0]], axis=-1)
        write_stl(os.path.join(OUT, f'wheel_{side}.stl'), t)
        manifest.append({'file': os.path.join(OUT, f'wheel_{side}.stl'), 'body': f'wheel_{side}', 'rgba': [0.09, 0.09, 0.1, 1]})
    write_cap(cap(), os.path.join(OUT, 'cap.obj'), os.path.join(OUT, 'cap.png'))
    manifest.append({'file': os.path.join(OUT, 'cap.obj'), 'body': 'robot', 'rgba': [1, 1, 1, 1], 'texture': 'cap.png'})
    screws = []
    for s in (-1, 1):                                   # the two screws standing on the cap (photos)
        x, y = s * 27.0, CHASSIS_FRONT - 52.0
        screws.append(place(cylinder(1.25, 13.0), (0, 0, 0), ('y', -90)) + [x, y, CHASSIS_TOP + 3])
        screws.append(place(cylinder(2.7, 1.6), (0, 0, 0), ('y', -90)) + [x, y, CHASSIS_TOP + 16])
    write_stl(os.path.join(OUT, 'screws.stl'), np.concatenate(screws))
    manifest.append({'file': os.path.join(OUT, 'screws.stl'), 'body': 'robot', 'rgba': [0.78, 0.78, 0.8, 1]})
    json.dump(manifest, open(os.path.join(OUT, 'manifest.json'), 'w'), indent=1)
    print(f'{len(manifest)} meshes ->', OUT)
    return OUT


def _manifest():
    m = os.path.join(OUT, 'manifest.json')
    if not os.path.exists(m):
        build()
    return json.load(open(m))


def defaults_xml():
    return (f'<default class="nvisual"><geom type="mesh" group="{ROBOT_GROUP}" contype="0" conaffinity="0" mass="0"/></default>'
            '<default class="nhull"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.1" mass="0" friction="0.3"/></default>'
            '<default class="nskid"><geom group="3" contype="2" conaffinity="1" rgba="0 0 1 0.3" mass="0" friction="0.05 0.001 0.0001"/></default>'
            '<default class="ntire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" condim="3" solref="0.004 1" '
            f'friction="{N.TIRE_FRICTION} 0.005 0.0001"/></default>')


def assets_xml():
    out = f'<texture name="n_cap" type="2d" file="{OUT}/cap.png"/><material name="n_cap" texture="n_cap" specular="0.2"/>'
    for k, e in enumerate(_manifest()):
        out += f'<mesh name="n_{k:02d}" file="{e["file"]}" scale="{_f(MM, MM, MM)}" inertia="shell"/>'
    return out


def robot_xml(start_xy=(0.5, 0.5), prefix=''):
    """As Ratatron's body (sim/model.py): its CAD frame, its skids, its tires; NezumiBot's looks."""
    p, man = prefix, _manifest()

    def visual(k, e, at=(0, 0, 0)):
        look = 'material="n_cap"' if e.get('texture') else f'rgba="{_f(*e["rgba"])}"'
        return f'<geom class="nvisual" mesh="n_{k:02d}" {look} pos="{_f(*(v * MM for v in at))}"/>'
    chassis = ''.join(visual(k, e) for k, e in enumerate(man) if e['body'] == 'robot')
    r = N.WHEEL_RADIUS_MM * MM
    wheels = ''
    for side, sign in (('l', -1), ('r', 1)):
        vis = ''.join(visual(k, e) for k, e in enumerate(man) if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="{p}wheel_{side}" pos="{_f(sign * N.CONTACT_X_MM * MM, 0, N.AXLE_Z_MM * MM)}">
        <joint name="{p}wheel_{side}" type="hinge" axis="-1 0 0" armature="{N.MOTOR_ROTOR_INERTIA * N.GEAR ** 2:.3g}"/>
        <geom name="{p}tire_{side}" class="ntire" type="cylinder" size="{_f(r, 0.001)}" euler="0 90 0" mass="0.004"/>
        {vis}
      </body>'''
    sites = ''.join(f'<site name="{p}ir_{n}" pos="{_f(*(v * MM for v in xyz))}" size="0.0015" group="4" rgba="1 0 0 0.6"/>'
                    for n, (_, _, xyz, _) in C.IR_SENSORS.items())
    skids = ''.join(f'<geom name="{p}skid_{n}" class="nskid" type="sphere" pos="{_f(0, y * MM, (C.SKID_Z_MM + C.SKID_R_MM) * MM)}" '
                    f'size="{C.SKID_R_MM * MM:.6g}"/>' for n, y in zip(('front', 'rear'), C.SKID_Y_MM))
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, -C.GROUND_Z_MM * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in C.COM_MM))}" mass="{N.MASS_KG - 0.008:.4f}" diaginertia="5e-5 4e-5 7e-5"/>
      <geom name="{p}hull" class="nhull" type="box" pos="{_f(0, 8 * MM, 15 * MM)}" size="{_f(36 * MM, 37 * MM, 20 * MM)}"/>
      {skids}
      {chassis}{sites}{wheels}
    </body>'''


def scene_xml(maze):
    """The maze (Ratatron's, sim/model.py) with NezumiBot alone, for its firmware's runs."""
    centre = maze.size * C.CELL_MM * MM / 2
    return f'''<mujoco model="nezumibot_maze">
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


if __name__ == '__main__':
    build()
