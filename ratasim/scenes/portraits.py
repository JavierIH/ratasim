"""A square picture of each robot, alone in a dark studio (the README's grid of supported robots), and the README's
picture of them in a maze.

    ratasim portraits [DIR]                 (default docs/images: robots/NAME.jpg, maze.jpg)
    ratasim portraits [DIR] --only NAME...  those robots' portraits only
"""
import argparse
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
    'kerise': ('e_', 0.15, 215, -30),
}
SHADOW_CLIP = {'kerise': 0.1}   # its shadows' map over less than the scene (m): the small ones' shadows without steps
SKY = 'rgb1="0.17 0.18 0.21" rgb2="0.04 0.04 0.05"'          # a dark studio
STUDIO = ('<geom name="floor" type="plane" size="4 4 0.1" rgba="0.10 0.11 0.13 1" condim="3" friction="0.05 0.001 0.0001"/>'
          # the camera looks from +x +y (azimuth 215): the key light from its upper left (the shadow falls behind the
          # robot, to the right), a soft fill from its right, a faint rim from behind to lift the edges off the dark
          '<light directional="true" pos="1 0 2" dir="-1 0.05 -1.3" diffuse="0.8 0.8 0.78" castshadow="true"/>'
          '<light directional="true" pos="0 1 2" dir="-0.3 -0.95 -0.8" diffuse="0.3 0.3 0.33" castshadow="false"/>'
          '<light directional="true" pos="-1 -1 2" dir="0.8 0.6 -0.6" diffuse="0.25 0.25 0.28" castshadow="false"/>')


def maze_picture(out):
    """The robots side by side in the maze's first cells, from above at three quarters."""
    m, d = K.compile(K.scene_xml(Maze.load(maze_path('oshwdem2026')), gamusino=True))
    cell = K.C.CELL_MM * K.MM
    for (x, y), prefix in zip(((0.5, 1.5), (1.5, 1.5), (2.5, 1.5)), ('', 'z_', 'g_')):
        K.Bot(m, d, prefix).place(x * cell, y * cell, 1.5708)
    for _ in range(600):
        mujoco.mj_step(m, d)
    ren, cam = mujoco.Renderer(m, 720, 1280), mujoco.MjvCamera()
    cam.lookat[:] = (1.5 * cell, 1.5 * cell, 0.012)
    cam.distance, cam.azimuth, cam.elevation = 0.47, 240, -27
    ren.update_scene(d, cam)
    Image.fromarray(ren.render()).save(out, quality=90)
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
                       kerise=True)
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
            bot.place(0.0 if other == prefix else 5.0, 0.0, 1.5708)     # the others out of the picture
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
