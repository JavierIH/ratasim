"""A square picture of each robot, alone in a dark studio (the README's grid of supported robots), and the README's
picture of them in a maze.

    ratasim portraits [DIR]                 (default docs/images: robots/NAME.jpg, maze.jpg)
    ratasim portraits [DIR] --only NAME...  those robots' portraits only
"""
import argparse
import math
import os

import mujoco
from PIL import Image

from .. import paths
from ..maze import Maze
from ..run import maze_path
from . import common as K

SIZE = 900
ROBOTS = {      # file: prefix, camera distance (m), azimuth, elevation[, the height it looks at (m): 0.012]
    'ratatron': ('', 0.20, 215, -30),
    'zorobot3': ('z_', 0.18, 215, -32),
    'gamusino': ('g_', 0.17, 215, -32),
    'bulebule': ('b_', 0.22, 215, -30),
    'nezumibot': ('n_', 0.20, 215, -28),
    'perenquen': ('p_', 0.20, 215, -30),
    'fujin': ('f_', 0.20, 215, -30),
    'futura': ('u_', 0.18, 215, -32),
    'ukmarsbot': ('k_', 0.22, 215, -32),
    'miniskybot': ('m_', 0.27, 215, -26, 0.03),
    'kerise': ('e_', 0.155, 215, -30),
    'ratatron2': ('r2_', 0.20, 215, -30),
    'koguma': ('kg_', 0.20, 215, -30),
}
SHADOW_CLIP = {'kerise': 0.15}   # its shadows' map over less than the scene (m): the small ones' shadows without steps
BEHIND = {'ratatron2': 0.012}    # its middle that far ahead of its origin (m): put that much behind, in the middle
SKY = 'rgb1="0.17 0.18 0.21" rgb2="0.04 0.04 0.05"'          # a dark studio
STUDIO = ('<geom name="floor" type="plane" size="4 4 0.1" rgba="0.10 0.11 0.13 1" condim="3" friction="0.05 0.001 0.0001"/>'
          # the camera looks from +x +y (azimuth 215): the key light from its upper left (the shadow falls behind the
          # robot, to the right), a soft fill from its right, a faint rim from behind to lift the edges off the dark
          '<light directional="true" pos="1 0 2" dir="-1 0.05 -1.3" diffuse="0.8 0.8 0.78" castshadow="true"/>'
          '<light directional="true" pos="0 1 2" dir="-0.3 -0.95 -0.8" diffuse="0.3 0.3 0.33" castshadow="false"/>'
          '<light directional="true" pos="-1 -1 2" dir="0.8 0.6 -0.6" diffuse="0.25 0.25 0.28" castshadow="false"/>')


MAZE_BOTS = (     # prefix, cell (x, y), heading (deg): each its own way, in the middle of the maze
    ('', 6.1, 5.5, 0),          # Ratatron east along its corridor
    ('g_', 7.5, 6.25, 80),      # Gamusino north through the gap
    ('z_', 8.8, 6.5, 0))        # ZoroBot3 east


def maze_picture(out, w=1600, h=800):
    """Ratatron, Gamusino and ZoroBot3 in the middle of the maze, each going its own way, from above (no sky): a
    dimmer headlight and a key light from the side so that the walls have a lit and a shaded face, rendered at twice
    the size and scaled down (smooth edges)."""
    xml = K.scene_xml(Maze.load(maze_path('oshwdem2026')), gamusino=True)
    xml = xml.replace('<headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>',
                      '<headlight ambient="0.22 0.22 0.24" diffuse="0.15 0.15 0.15" specular="0.05 0.05 0.05"/>')
    xml = xml.replace('</worldbody>', '<light directional="true" dir="-0.5 0.6 -1" diffuse="0.75 0.73 0.7" castshadow="true"/>'
                      '<light directional="true" dir="0.7 -0.3 -0.6" diffuse="0.2 0.22 0.26" castshadow="false"/>'
                      '</worldbody>', 1)
    m, d = K.compile(xml)
    m.vis.global_.offwidth, m.vis.global_.offheight = 2 * w, 2 * h
    m.vis.map.shadowclip = 0.8
    cell = K.C.CELL_MM * K.MM
    for prefix, x, y, heading in MAZE_BOTS:
        K.Bot(m, d, prefix).place(x * cell, y * cell, math.radians(heading))
    for _ in range(600):
        mujoco.mj_step(m, d)
    ren, cam = mujoco.Renderer(m, 2 * h, 2 * w), mujoco.MjvCamera()
    cam.lookat[:] = (7.4 * cell, 6.0 * cell, 0.02)
    cam.distance, cam.azimuth, cam.elevation = 0.66, 110, -48
    ren.update_scene(d, cam)
    Image.fromarray(ren.render()).resize((w, h), Image.LANCZOS).save(out, quality=90)
    print('->', out)


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim portraits', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dir', nargs='?', default=os.path.join(paths.REPO, 'docs', 'images'))
    p.add_argument('--only', nargs='+', choices=sorted(ROBOTS), metavar='NAME', help="only these robots' portraits")
    a = p.parse_args(argv)
    os.makedirs(os.path.join(a.dir, 'robots'), exist_ok=True)
    if not a.only:
        maze_picture(os.path.join(a.dir, 'maze.jpg'))
    xml = K.scene_xml(Maze.load(maze_path('oshwdem2026')), with_maze=False, gamusino=True, bulebule=True, nezumibot=True,
                       perenquen=True, fujin=True, futura=True, ukmarsbot=True, miniskybot=True,
                       kerise=True, ratatron2=True, koguma=True)
    xml = xml.replace('<texture name="sky" type="skybox" builtin="gradient" rgb1="0.25 0.27 0.3" rgb2="0.05 0.05 0.06"',
                      f'<texture name="sky" type="skybox" builtin="gradient" {SKY}')
    m, d = K.compile(xml.replace('</worldbody>', STUDIO + '</worldbody>', 1))
    bots = {prefix: K.Bot(m, d, prefix) for prefix, *_ in ROBOTS.values()}
    ren, cam = mujoco.Renderer(m, SIZE, SIZE), mujoco.MjvCamera()
    clip = m.vis.map.shadowclip
    for name, (prefix, dist, az, el, *z) in ROBOTS.items():
        if a.only and name not in a.only:
            continue
        m.vis.map.shadowclip = SHADOW_CLIP.get(name, clip)
        for other, bot in bots.items():
            bot.place(0.0 if other == prefix else 5.0, -BEHIND.get(name, 0.0), 1.5708)    # the others away
        for _ in range(600):                                             # settle on the floor
            mujoco.mj_step(m, d)
        cam.lookat[:] = (0.0, 0.004, z[0] if z else 0.012)
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        ren.update_scene(d, cam)
        out = os.path.join(a.dir, 'robots', f'{name}.jpg')
        Image.fromarray(ren.render()).save(out, quality=90)
        print('->', out)


if __name__ == '__main__':
    main()
