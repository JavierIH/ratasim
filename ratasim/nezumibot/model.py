"""NezumiBot (OPRobots, github.com/OPRobots/NezumiBot): Ratatron's printed chassis in black (it is based on Ratatron)
with its own cap (3d_model/stl_files/gorra.stl in its repository) and the word written on it in blue marker, as in
its photos, and the two screws standing on it. Collisions and wheels: Ratatron's for now (its N20 wheels: coming).

    python -m ratasim.nezumibot.model        (builds its meshes into build/)
"""
import json
import math
import os
import struct
import urllib.request

import numpy as np

from .. import config as C, paths
from ..model import MM, ROBOT_GROUP, _f
from ..bulebule.model import cylinder, place

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'build')
REPO = os.path.join(paths.EXTERNAL, 'NezumiBot')
CAP_STL = os.path.join(REPO, '3d_model', 'stl_files', 'gorra.stl')
FONT_URL = 'https://github.com/google/fonts/raw/main/apache/permanentmarker/PermanentMarker-Regular.ttf'
WORD, INK = 'MARINE', (30, 70, 200)
BLACK_PLA = (0.07, 0.07, 0.08, 1)
CHASSIS_TOP, CHASSIS_FRONT = 36.0, 45.0     # Ratatron's chassis in its CAD frame (mm): the cap sits there
TEXT_DIR = np.array([0.0, 0.62, 0.78])      # the cap's front panel faces forward and up


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
    """The cap as an OBJ whose front panel carries the word (planar projection along TEXT_DIR), the rest white."""
    from PIL import Image, ImageDraw, ImageFont
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    panel = (n @ TEXT_DIR > 0.55) & (n[:, 1] > 0.3) & (tris.mean(axis=1)[:, 1] > CHASSIS_FRONT - 32)
    up = np.cross(TEXT_DIR, [1.0, 0, 0])
    up = -up / np.linalg.norm(up)
    pts = tris[panel].reshape(-1, 3)
    u0, u1 = pts[:, 0].min(), pts[:, 0].max()
    w0, w1 = (pts @ up).min(), (pts @ up).max()
    font = os.path.join(OUT, 'marker.ttf')
    if not os.path.exists(font):
        urllib.request.urlretrieve(FONT_URL, font)
    W, H = 1024, int(1024 * (w1 - w0) / (u1 - u0))
    im = Image.new('RGB', (W, H), (238, 238, 236))
    g = ImageDraw.Draw(im)
    f = ImageFont.truetype(font, int(H * 0.42))
    tw, th = g.textbbox((0, 0), WORD, font=f)[2:]
    g.text(((W - tw) / 2, H * 0.12), WORD, font=f, fill=INK)          # high on the panel: its front edge curves away
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
        rgba = BLACK_PLA if tuple(p['rgba'][:3]) == (1.0, 1.0, 0.0) else p['rgba']
        manifest.append({'file': os.path.join(C.ASSETS, p['mesh'] + '.stl'), 'body': p['body'], 'rgba': list(rgba)})
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
            '<default class="ntire"><geom group="3" contype="2" conaffinity="1" rgba="0 0 0 0.3" friction="0.6 0.005 0.0001" condim="3" solref="0.004 1"/></default>')


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
    r = C.TIRE_RADIUS_MM * MM
    wheels = ''
    for side, sign in (('l', -1), ('r', 1)):
        cx = sign * C.CONTACT_X_MM
        vis = ''.join(visual(k, e, (-cx, 0, -C.AXLE_Z_MM)) for k, e in enumerate(man) if e['body'] == f'wheel_{side}')
        wheels += f'''
      <body name="{p}wheel_{side}" pos="{_f(cx * MM, 0, C.AXLE_Z_MM * MM)}">
        <joint name="{p}wheel_{side}" type="hinge" axis="-1 0 0" armature="2e-6"/>
        <geom name="{p}tire_{side}" class="ntire" type="cylinder" size="{_f(r, 0.001)}" euler="0 90 0" mass="0.004"/>
        {vis}
      </body>'''
    skids = ''.join(f'<geom name="{p}skid_{n}" class="nskid" type="sphere" pos="{_f(0, y * MM, (C.SKID_Z_MM + C.SKID_R_MM) * MM)}" '
                    f'size="{C.SKID_R_MM * MM:.6g}"/>' for n, y in zip(('front', 'rear'), C.SKID_Y_MM))
    return f'''
    <body name="{p}robot" pos="{_f(start_xy[0] * C.CELL_MM * MM, start_xy[1] * C.CELL_MM * MM, -C.GROUND_Z_MM * MM)}">
      <freejoint name="{p}robot"/>
      <inertial pos="{_f(*(v * MM for v in C.COM_MM))}" mass="0.0920" diaginertia="5e-5 4e-5 7e-5"/>
      <geom name="{p}hull" class="nhull" type="box" pos="{_f(0, 8 * MM, 15 * MM)}" size="{_f(36 * MM, 37 * MM, 20 * MM)}"/>
      {skids}
      {chassis}{wheels}
    </body>'''


if __name__ == '__main__':
    build()
