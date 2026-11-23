"""Ratatron II in the simulator: its model only, for now (its firmware is not simulated yet). Put down in the start
cell of a maze, its motors off; its six LEDs on its tail lit as --leds asks. In a window (MuJoCo's viewer), or a
picture of it with --image.

    ratasim ratatron2 [--maze oshwdem2026] [--leds 0b111111] [--image out.jpg]
"""
import argparse
import os
import sys

if '--image' in sys.argv:
    os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco   # noqa: E402

from ..maze import Maze, maze_path   # noqa: E402
from . import model as M   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim ratatron2', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--leds', type=lambda s: int(s, 0), default=0,
                   help='its LEDs lit, as a mask: bit 0 LED1 (D2) ... bit 5 LED6 (D7), e.g. 0b111111')
    p.add_argument('--image', help='no window: a picture of it in its start cell, from behind (its LEDs)')
    a = p.parse_args(argv)
    m = mujoco.MjModel.from_xml_string(M.scene_xml(Maze.load(maze_path(a.maze)), leds=a.leds))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    if a.image:
        for _ in range(600):                          # settle on its wheels
            mujoco.mj_step(m, d)
        from PIL import Image
        w, h = 1280, 720
        ren, cam = mujoco.Renderer(m, h, w), mujoco.MjvCamera()
        x, y, _ = d.xpos[m.body('robot').id]
        cam.lookat[:] = (x, y - 0.01, 0.012)
        cam.distance, cam.azimuth, cam.elevation = 0.2, 75, -55       # over its start cell's back wall
        ren.update_scene(d, cam)
        Image.fromarray(ren.render()).save(a.image, quality=90)
        print('->', a.image)
        return 0
    from mujoco import viewer
    viewer.launch(m, d)
    return 0


if __name__ == '__main__':
    sys.exit(main())
