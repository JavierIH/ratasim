"""Miniskybot 2 in the simulator, (config.py, fw/miniskybot/miniskybot.ino): switched on in the start cell, its button pressed, it goes straight on until its
ultrasonic ranger sees a wall near ahead, stops in the middle of that cell and turns, by time, to a way that is open,
trying first one side and then the other at each cell it turns in.

    ratasim miniskybot [--maze oshwdem2026] [--headless] [--record out.mp4] [--max-s N] [--echo]
"""
import argparse
import os
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from .. import config as RC   # noqa: E402
from ..maze import Maze   # noqa: E402
from ..recorder import add_view_args, make_view, time_limit   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..start import play   # noqa: E402
from ..stopwatch import Stopwatch   # noqa: E402
from . import config as S   # noqa: E402
from .robot import MRobot   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim miniskybot', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 120; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its program prints here')
    p.add_argument('--echo', action='store_true', help='print what its program prints')
    a = p.parse_args(argv)
    r = MRobot(Maze.load(maze_path(a.maze)), seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'Miniskybot 2 (Obijuan)', led=r.led, clock=clock)
    a.max_s = time_limit(a, rec, 120)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    partial = ['']
    cells = set()
    turns = {'right': 0, 'left': 0, 'back': 0, 'stuck': 0}

    def say(text):
        print(f'[{r.fw.time_ms() / 1000:7.2f}] {text}')
        if rec:
            rec.status = text

    def run(ms, fast=False):
        nonlocal crash_t
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('its program stopped')
                return False
            pose = r.pose()
            clock.update(r.fw.time_ms(), pose)
            cells.add((int(pose[0] // RC.CELL_MM), int(pose[1] // RC.CELL_MM)))
            out = r.fw.received()
            if out:
                if log:
                    log.write(out)
                *lines, partial[0] = (partial[0] + out).split('\n')
                for line in (v.strip() for v in lines):
                    if line in turns:
                        turns[line] += 1
                    if a.echo or line in ('go', 'Miniskybot: press its button'):
                        say(f'it says: {line}' + (f' at {[round(v) for v in r.pose()]}' if a.echo else ''))
            if r.toppled():
                print(f'[{r.fw.time_ms() / 1000:7.2f}] toppled at {[round(v) for v in r.pose()]}')
                return False
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'[{r.fw.time_ms() / 1000:7.2f}] touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=fast)
        return True

    play(r, S.start(), lambda ms: run(ms, fast=True), say)
    say('its program: straight on to a wall, turns to an open way')
    while r.fw.time_ms() < a.max_s * 1000:
        if not run(500):
            break
    print(f'turns: {turns["right"]} right, {turns["left"]} left, {turns["back"]} back; {turns["stuck"]} stuck; '
          f'{len(cells)} cells visited')
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, '
          f'pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
