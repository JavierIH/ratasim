"""A square picture of each robot, alone in a plain studio (the README's grid of supported robots).

    ratasim portraits [DIR]         (default docs/images/robots)
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
ROBOTS = {      # file: prefix, camera distance (m), azimuth, elevation
    'ratatron': ('', 0.20, 215, -30),
    'zorobot3': ('z_', 0.18, 215, -32),
}
STUDIO = ('<geom name="floor" type="plane" size="4 4 0.1" rgba="0.86 0.87 0.89 1" condim="3" friction="0.05 0.001 0.0001"/>'
          '<light directional="true" pos="0 -1 2" dir="0.3 0.5 -1" diffuse="0.65 0.65 0.65" castshadow="true"/>'
          '<light directional="true" pos="0 1 2" dir="-0.3 -0.4 -1" diffuse="0.35 0.35 0.35" castshadow="false"/>')


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim portraits', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dir', nargs='?', default=os.path.join(paths.REPO, 'docs', 'images', 'robots'))
    a = p.parse_args(argv)
    os.makedirs(a.dir, exist_ok=True)
    xml = K.scene_xml(Maze.load(maze_path('oshwdem2026')), with_maze=False)
    xml = xml.replace('<texture name="sky" type="skybox" builtin="gradient" rgb1="0.25 0.27 0.3" rgb2="0.05 0.05 0.06"',
                      '<texture name="sky" type="skybox" builtin="gradient" rgb1="0.97 0.97 0.98" rgb2="0.86 0.87 0.89"')
    m, d = K.compile(xml.replace('</worldbody>', STUDIO + '</worldbody>', 1))
    bots = {prefix: K.Bot(m, d, prefix) for prefix, *_ in ROBOTS.values()}
    ren, cam = mujoco.Renderer(m, SIZE, SIZE), mujoco.MjvCamera()
    for name, (prefix, dist, az, el) in ROBOTS.items():
        for other, bot in bots.items():
            bot.place(0.0 if other == prefix else 5.0, 0.0, 1.5708)     # the others out of the picture
        for _ in range(600):                                             # settle on the floor
            mujoco.mj_step(m, d)
        cam.lookat[:] = (0.0, 0.004, 0.012)
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        ren.update_scene(d, cam)
        out = os.path.join(a.dir, f'{name}.png')
        Image.fromarray(ren.render()).save(out)
        print('->', out)


if __name__ == '__main__':
    main()
