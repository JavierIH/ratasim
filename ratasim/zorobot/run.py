"""ZoroBot3 in the simulator: boots its firmware, picks the run in its menu (buttons), starts it with a hand in front
of a sensor as on the real robot, and runs it in a maze; optionally records a video.

    ratasim zorobot [--maze oshwdem2026] [--speed 0..5] [--explore-type 0..2] [--run]
                [--record out.mp4] [--max-s 400]

--run: EXPLORE & RUN (explore, then the speed run at --speed); else only the exploration.
"""
import argparse
import math
import os
import subprocess
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco   # noqa: E402

from ..maze import Maze   # noqa: E402
from ..run import maze_path   # noqa: E402
from .robot import ZRobot   # noqa: E402


class Recorder:
    """An mp4 (ffmpeg): a camera following the robot, the whole maze in a corner, the time and the RGB LED."""

    def __init__(self, robot, path, width=1280, height=720, fps=30, speed=1.0):
        from PIL import Image, ImageDraw, ImageFont     # noqa: F401
        self.r, self.fps, self.speed = robot, fps, speed
        m = robot.model
        self.big = mujoco.Renderer(m, height, width)
        self.small = mujoco.Renderer(m, 300, 300)
        self.cam, self.top = mujoco.MjvCamera(), mujoco.MjvCamera()
        side = robot.maze.size * 0.18
        self.top.lookat[:] = (side / 2, side / 2, 0)
        self.top.distance, self.top.elevation, self.top.azimuth = side * 1.75, -90, 90
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 0.55, -45, 90
        self.next_ms = robot.fw.time_ms()
        self.w, self.h = width, height
        self.trail = []
        self.status = ''
        self.ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                    '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-c:v', 'libx264',
                                    '-pix_fmt', 'yuv420p', '-crf', '22', path], stdin=subprocess.PIPE)

    def maybe_frame(self, fast=False):
        from PIL import Image, ImageDraw
        r = self.r
        t = r.fw.time_ms()
        if t < self.next_ms:
            return
        speed = 8.0 if fast else self.speed
        self.next_ms = t + max(1, int(1000 * speed / self.fps))
        x, y, yaw = r.pose()
        look, target = self.cam.lookat, (x / 1000, y / 1000, 0.0)
        for i in range(3):
            look[i] = 0.8 * look[i] + 0.2 * target[i] if look.any() else target[i]
        self.big.update_scene(r.data, self.cam)
        frame = Image.fromarray(self.big.render())
        self.small.update_scene(r.data, self.top)
        inset = Image.fromarray(self.small.render())
        d2 = ImageDraw.Draw(inset)
        self.trail.append((x, y))
        side = r.maze.size * 180.0
        k = 300 / (2 * 1.75 * side * math.tan(math.radians(self.r.model.vis.global_.fovy / 2)))   # px per mm
        pts = [(150 + (px - side / 2) * k, 150 - (py - side / 2) * k) for px, py in self.trail[-4000:]]
        if len(pts) > 1:
            d2.line(pts, fill=(80, 200, 255), width=2)
        frame.paste(inset, (self.w - 310, 10))
        d = ImageDraw.Draw(frame)
        d.rectangle((0, 0, 420, 64), fill=(0, 0, 0))
        d.text((10, 8), f'ZoroBot3 (OPRobots) - its firmware, simulated    t = {t / 1000:6.1f} s   x{speed:g}', fill=(255, 255, 255))
        d.text((10, 34), self.status, fill=(255, 220, 60))
        rgb = tuple(int(255 * c) for c in r.fw.rgb())
        d.ellipse((392, 34, 410, 52), fill=rgb, outline=(120, 120, 120))
        self.ff.stdin.write(frame.tobytes())

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--speed', type=int, default=1, help='run speed of its menu: 0 explore .. 5 haki')
    p.add_argument('--explore-type', type=int, default=0, help='0 to the goal, 1 and back home, 2 complete')
    p.add_argument('--floodfill', type=int, default=0, help='0 basic, 1 diagonal, 2 time')
    p.add_argument('--run', action='store_true', help='explore, then the speed run')
    p.add_argument('--record', metavar='MP4')
    p.add_argument('--record-speed', type=float, default=1.0)
    p.add_argument('--max-s', type=float, default=400.0)
    p.add_argument('--log', help='write everything the firmware prints here')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    menu = (a.speed, 0, a.explore_type, a.floodfill, 1, 0, 2)
    r = ZRobot(maze, menu=menu)
    rec = Recorder(r, a.record, speed=a.record_speed) if a.record else None
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    started = ended = False
    crash_t = -10000

    def run(ms, fast=False):
        nonlocal started, ended, crash_t
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('firmware halted')
                return False
            out = r.fw.received()
            if out:
                for line in out.splitlines():
                    if log:
                        log.write(f'[{r.fw.time_ms() / 1000:8.3f}] {line}\n')
            race = r.fw.race_started()
            started = started or race
            if started and not race and not ended:
                ended = True
                print(f'[{r.fw.time_ms() / 1000:7.2f}] race over at {[round(v) for v in r.pose()]}')
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'[{r.fw.time_ms() / 1000:7.2f}] touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=not race)
        return True

    if rec:
        rec.status = 'booting'
    run(3500, fast=True)
    r.press('mode')
    run(400, fast=True)
    r.press('up')
    run(800, fast=True)
    if rec:
        rec.status = 'a hand in front of the right front sensor: EXPLORE' + (' & RUN' if a.run else '')
    r.cover('FR', 500)
    run(900, fast=True)
    if a.run:
        r.cover('FL', 300)
    run(2500, fast=True)
    print(f'[{r.fw.time_ms() / 1000:7.2f}] started: {r.fw.race_started()}')
    if rec:
        rec.status = 'exploring' + (' then running' if a.run else '')
    while r.fw.time_ms() < a.max_s * 1000 and not (ended and r.fw.time_ms() > 0):
        if not run(500):
            break
    run(2000, fast=True)
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
