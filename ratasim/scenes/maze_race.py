"""A race in the maze: the robots' runs, each with its own firmware, side by side in one maze. Each robot runs alone,
as its runner starts it (as tests/robots.py does), its run traced (--trace, ratasim/trace.py); then the runs are put
together, each from the moment it leaves the start cell (the stopwatch's start), the robots going through each other
(they never met). A video: the maze from above at three quarters, each robot's trail in its colour, a camera on the
leader (the nearest to the goal by the maze's paths) in a corner, and the stopwatch's times to the goal.

    ratasim maze-race out.mp4 [--robots ratatron,zorobot,bulebule,fujin,ukmarsbot] [--maze oshwdem2026]
                              [--speed 4] [--max-s 200] [--fresh]
    ratasim maze-race --still out.jpg [--at 20]      # a picture of the race at 20 s

The runs are cached in build/scenes/traces (by robot and maze; --fresh runs them again). By default the robots that
search to the goal; the others (Gamusino, NezumiBot, Perenquen, Futura) run what they run, nowhere near it.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
from collections import deque

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from .. import paths
from ..maze import STEP, Maze
from ..run import maze_path
from ..stopwatch import Stopwatch
from ..trace import Trace
from . import common as K

TRACES = os.path.join(paths.SIM_BUILD, 'scenes', 'traces')
DEFAULT = 'ratatron,zorobot,bulebule,fujin,ukmarsbot'
LEAD_S = 1.5                    # before the start
RUNS = {        # its command and arguments (tests/robots.py's), {flash}: a blank file for its saved map
    'ratatron': ['ratatron', '--headless', '--speed', '0', '--no-pty', '--flash', '{flash}', '--erase-flash',
                 '--max-s', '600'],
    'zorobot': ['zorobot', '--headless', '--max-s', '400'],
    'gamusino': ['gamusino', '--headless', '--max-s', '30'],
    'bulebule': ['bulebule', '--headless', '--erase-flash', '--flash', '{flash}', '--max-s', '300'],
    'nezumibot': ['nezumibot', '--headless', '--hand', 'left', '--max-s', '60'],
    'perenquen': ['perenquen', '--headless', '--side', 'right', '--gain', '30', '--max-s', '60'],
    'fujin': ['fujin', '--headless', '--max-s', '400'],
    'futura': ['futura', '--headless', '--max-s', '60'],
    'ukmarsbot': ['ukmarsbot', '--headless', '--max-s', '400'],
}


def trace(name, maze, fresh=False):
    """Its run in the maze (a Trace), run now if not cached."""
    path = os.path.join(TRACES, f'{name}-{maze}.npz')
    if fresh or not os.path.exists(path):
        os.makedirs(TRACES, exist_ok=True)
        flash = os.path.join(tempfile.mkdtemp(), 'flash.bin')
        args = [a.format(flash=flash) for a in RUNS[name]] + ['--maze', maze, '--trace', path]
        print(f'{K.ROBOTS[name]["name"]}: its run (ratasim {" ".join(args)})', flush=True)
        out = subprocess.run([sys.executable, '-m', 'ratasim'] + args, cwd=paths.REPO, stdin=subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True).stdout
        if not os.path.exists(path):
            raise SystemExit(f'{name}: no trace\n{out[-3000:]}')
    return Trace(path)


def distances(maze):
    """Each cell's distance to the goal (cells, through the maze's open walls)."""
    dist = {c: 0 for c in maze.goal()}
    queue = deque(dist)
    while queue:
        x, y = queue.popleft()
        for d, (dx, dy) in STEP.items():
            n = (x + dx, y + dy)
            if not maze.has(x, y, d) and n not in dist:
                dist[n] = dist[(x, y)] + 1
                queue.append(n)
    return dist


class Runner:
    """A robot of the race: its run (its trace), from its start (ms of its run), its time to the goal."""

    def __init__(self, m, d, name, tr, maze):
        self.name, self.spec, self.tr = name, K.ROBOTS[name], tr
        self.bot = K.Bot(m, d, self.spec['prefix'])
        self.adr = {}
        for joint in tr.slices:
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, self.spec['prefix'] + joint)
            if jid >= 0:
                self.adr[joint] = m.jnt_qposadr[jid]
        clock, self.start, self.goal = Stopwatch(maze, say=lambda _: None), None, None
        for t, pose in zip(tr.t, tr.pose):
            clock.update(int(t), pose)
            if clock.runs and self.start is None:
                self.start = int(t)
            if clock.last is not None:
                self.goal = clock.last / 1000
                break
        if self.start is None:      # it never left its start cell
            self.start = int(tr.t[-1])

    def at(self, s):
        """Its pose s seconds into the race."""
        t = self.start + 1000 * s
        for joint, q in self.tr.at(t).items():
            if joint in self.adr:
                self.bot.d.qpos[self.adr[joint]:self.adr[joint] + len(q)] = q
        return self.tr.pose[self.tr.index(t)]

    def done(self, s):
        return self.goal is not None and s >= self.goal


def build(names, maze_name, fresh):
    maze = Maze.load(maze_path(maze_name))
    traces = {n: trace(n, maze_name, fresh) for n in names}
    m, d = K.compile(K.scene_of(maze, names))
    K.park(m, d, names)
    m.opt.gravity[:] = 0
    runners = [Runner(m, d, n, traces[n], maze) for n in names]
    return maze, m, d, runners


def trails(scene, pts, rgba, width=0.006):
    """A line through pts (m, on the floor) into the scene, as capsules."""
    for a, b in zip(pts, pts[1:]):
        if scene.ngeom >= scene.maxgeom:
            return
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), rgba)
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width, np.array([a[0], a[1], 0.001]),
                             np.array([b[0], b[1], 0.001]))
        g.category = mujoco.mjtCatBit.mjCAT_DECOR
        g.emission, g.specular, g.shininess, g.reflectance = 0.1, 0.0, 0.0, 0.0
        scene.ngeom += 1


class Race:
    def __init__(self, names, maze_name, fresh, width=1280, height=720):
        self.maze, self.m, self.d, self.runners = build(names, maze_name, fresh)
        self.dist = distances(self.maze)
        self.w, self.h = width, height
        side = self.maze.size * K.C.CELL_MM * K.MM
        self.ren = mujoco.Renderer(self.m, height, width, max_geom=20000)
        self.pw, self.ph = 320 * width // 1280, 240 * width // 1280
        self.pip = mujoco.Renderer(self.m, self.ph, self.pw)
        self.cam, self.fcam = mujoco.MjvCamera(), mujoco.MjvCamera()
        self.cam.lookat[:] = (side / 2, side / 2 + 0.04 * side, 0)       # from above, the maze between the
        self.cam.distance, self.cam.elevation, self.cam.azimuth = side * 1.42, -80, 90    # times and the camera
        self.fcam.distance, self.fcam.elevation, self.fcam.azimuth = 0.45, -50, 90
        self.paths = {r.name: [] for r in self.runners}
        self.leader = None
        self.big, self.mid, self.small = K.font(30), K.font(22), K.font(18, bold=False)

    def left(self, pose):
        """Cells to the goal from where it is (999: outside the maze)."""
        return self.dist.get((int(pose[0] // K.C.CELL_MM), int(pose[1] // K.C.CELL_MM)), 999)

    def trail(self, s):
        """Each robot's trail on to where it is s seconds into the race, until its goal."""
        for r in self.runners:
            p = r.tr.pose[r.tr.index(r.start + 1000 * s)][:2] / 1000
            path = self.paths[r.name]
            if s >= 0 and not r.done(s) and (not path or math.dist(path[-1], p) > 0.01):
                path.append(tuple(p))

    def frame(self, s, speed):
        poses = {r.name: r.at(s) for r in self.runners}
        mujoco.mj_forward(self.m, self.d)
        self.trail(s)
        order = sorted(self.runners, key=lambda r: (0, r.goal) if r.done(s) else (1, self.left(poses[r.name])))
        lead = next((r for r in order if not r.done(s)), order[0])
        x, y = poses[lead.name][:2] / 1000
        look = np.array([x, y, 0.0])
        self.fcam.lookat[:] = look if lead is not self.leader or not self.fcam.lookat.any() else \
            0.8 * self.fcam.lookat + 0.2 * look
        self.leader = lead
        self.ren.update_scene(self.d, self.cam)
        for r in self.runners:
            trails(self.ren.scene, self.paths[r.name], np.array([c / 255 for c in r.spec['colour']] + [1.0]))
        img = Image.fromarray(self.ren.render())
        self.pip.update_scene(self.d, self.fcam)
        px, py = self.w - self.pw - 10, 60
        img.paste(Image.fromarray(self.pip.render()), (px, py))
        g = ImageDraw.Draw(img)
        g.rectangle((px - 2, py - 2, px + self.pw + 1, py + self.ph + 1), outline=(255, 255, 255), width=2)
        g.rectangle((px, py + self.ph - 24, px + 230, py + self.ph), fill=(0, 0, 0))
        g.text((px + 8, py + self.ph - 22), f'en cabeza: {lead.spec["name"]}', font=self.small, fill=lead.spec['colour'])
        g.rectangle((0, 0, self.w, 50), fill=(0, 0, 0))
        g.text((16, 9), f'CARRERA EN EL LABERINTO  ·  {self.maze.size}x{self.maze.size}', font=self.big,
               fill=(255, 255, 255))
        g.text((self.w // 2 + 80, 12), 'preparados...' if s < 0 else f'{s:6.2f} s', font=self.mid,
               fill=(255, 220, 60))
        g.text((self.w - 120, 14), f'x{speed:g}', font=self.small, fill=(255, 220, 60))
        g.rectangle((10, 60, 350, 70 + 34 * len(order)), fill=(0, 0, 0))
        for i, r in enumerate(order):
            if r.done(s):
                res, col = f'{r.goal:.2f} s', (255, 255, 255)
            else:
                cells = self.left(poses[r.name])
                res, col = ('' if s < 0 else f'{cells} celdas' if cells < 999 else '-'), (170, 170, 170)
            g.text((20, 68 + 34 * i), f'{i + 1}. {r.spec["name"]}', font=self.mid, fill=r.spec['colour'])
            g.text((232, 68 + 34 * i), res, font=self.mid, fill=col)
        return img


def run(out, names, maze_name, speed, max_s, fresh):
    race = Race(names, maze_name, fresh)
    for r in race.runners:
        print(f'{r.spec["name"]:10s} ' + (f'at the goal in {r.goal:.3f} s' if r.goal is not None else 'not at the goal'))
    goals = [r.goal for r in race.runners if r.goal is not None]
    ends = [(r.tr.t[-1] - r.start) / 1000 for r in race.runners]
    end = min(max_s, (max(goals) + 2.0) if goals else max(ends))
    video = K.Video(out)
    s, fps = -LEAD_S, 30
    while s <= end:
        rate = 1.0 if s < 0 else speed
        video.write(race.frame(s, rate))
        s += rate / fps
    for _ in range(fps * 2):
        video.write(race.frame(end, speed))
    video.close()


def still(out, names, maze_name, at, fresh):
    race = Race(names, maze_name, fresh, 1920, 1080)
    for s in np.arange(0.0, at, 0.05):     # the trails up to then
        race.trail(s)
    race.frame(at, 1.0).save(out, quality=90)


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim maze-race', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out', nargs='?', help='the mp4')
    p.add_argument('--robots', default=DEFAULT, help=f'which, by name: {",".join(K.ROBOTS)} or all (default {DEFAULT})')
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--speed', type=float, default=4.0, help='the video\'s pace once they start (x real time)')
    p.add_argument('--max-s', type=float, default=200.0, help='the race\'s end at the latest (s from the start)')
    p.add_argument('--fresh', action='store_true', help='run the robots again (not their cached runs)')
    p.add_argument('--still', metavar='JPG', help='only a picture of the race, at --at')
    p.add_argument('--at', type=float, default=20.0, help='--still: seconds into the race')
    a = p.parse_args(argv)
    try:
        names = K.robot_names(a.robots)
    except ValueError as e:
        p.error(str(e))
    if a.still:
        still(a.still, names, a.maze, a.at, a.fresh)
        print('->', a.still)
    elif a.out:
        run(a.out, names, a.maze, a.speed, a.max_s, a.fresh)
        print('->', a.out)
    else:
        p.error('give the mp4, or --still JPG')


if __name__ == '__main__':
    main()
