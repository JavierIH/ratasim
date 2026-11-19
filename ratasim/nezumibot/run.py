"""NezumiBot in the simulator: boots its firmware (its OSHWDEM 2023 wall follower) and starts it as its makers do,
with a hand in front of its sensors: held in the air, the left or right sensor picks the "party" (it swaps hands
every 30 s after 45 s) and then the wall it follows; put down in the start cell, a hand in front starts it 2 s later.

    ratasim nezumibot [--maze oshwdem2026] [--hand left|right] [--party] [--headless] [--record out.mp4] [--max-s 60]
"""
import argparse
import os
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from ..maze import Maze   # noqa: E402
from ..recorder import add_view_args, make_view   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..start import play   # noqa: E402
from . import config as N   # noqa: E402
from .robot import NRobot   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim nezumibot', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--hand', choices=('left', 'right'), default='left', help='the wall it follows')
    p.add_argument('--party', action='store_true', help='swap hands every 30 s after 45 s')
    add_view_args(p)
    p.add_argument('--max-s', type=float, default=60.0)
    p.add_argument('--log', help='write everything the firmware prints here')
    a = p.parse_args(argv)
    r = NRobot(Maze.load(maze_path(a.maze)))
    rec = make_view(r, a, 'NezumiBot (OPRobots)')
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000

    def run(ms, fast=False):
        nonlocal crash_t
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('firmware halted')
                return False
            out = r.fw.received()
            if out and log:
                for line in out.splitlines():
                    log.write(f'[{r.fw.time_ms() / 1000:8.3f}] {line}\n')
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'[{r.fw.time_ms() / 1000:7.2f}] touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=fast)
        return True

    def say(text):
        print(f'[{r.fw.time_ms() / 1000:7.2f}] {text}')
        if rec:
            rec.status = text

    play(r, N.start(a.hand, a.party), lambda ms: run(ms, fast=True), say)
    say(f'following the {a.hand} wall' + (', party on' if a.party else ''))
    started = r.fw.global_bool('iniciado')
    print(f'[{r.fw.time_ms() / 1000:7.2f}] started: {started}')
    while r.fw.time_ms() < a.max_s * 1000:
        if not run(500):
            break
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
