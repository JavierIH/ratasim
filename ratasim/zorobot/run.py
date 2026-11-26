"""ZoroBot3 in the simulator: boots its firmware, picks the run in its menu (buttons), starts it with a hand in front
of a sensor as on the real robot, and runs it in a maze; optionally records a video.

    ratasim zorobot [--maze oshwdem2026] [--speed 0..5] [--explore-type 0..3] [--floodfill 0..2] [--run]
                [--solve 0|1] [--headless] [--record out.mp4] [--max-s N]

--run: EXPLORE & RUN (explore, then the speed run at --speed); else only the exploration. Its menu by default as its
makers ran it at OSHWDem 2026: explore complete, time-based floodfill, diagonals, speed 5 (haki).
"""
import argparse
import os
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from ..maze import Maze   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..recorder import add_view_args, make_view, time_limit   # noqa: E402
from ..start import play   # noqa: E402
from ..stopwatch import Stopwatch   # noqa: E402
from . import config as Z   # noqa: E402
from .robot import ZRobot   # noqa: E402


RACE_GAP_MS = 1500


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--speed', type=int, default=5, help='run speed of its menu: 0 explore .. 5 haki (its makers\' at OSHWDem 2026)')
    p.add_argument('--explore-type', type=int, default=2, choices=range(4), help='0 to the goal, 1 and back home, 2 complete (its makers\'), 3 endless (to random cells)')
    p.add_argument('--floodfill', type=int, default=2, choices=range(3), help='0 basic, 1 diagonal, 2 time (its makers\')')
    p.add_argument('--solve', type=int, default=1, help='its speed run: 1 diagonals (as its makers run it), 0 standard (turns of 90 and 180 deg)')
    p.add_argument('--run', action='store_true', help='explore, then the speed run')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 400; with a window, none)')
    p.add_argument('--log', help='write everything the firmware prints here')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    menu = (a.speed, 0, a.explore_type, a.floodfill, 1, a.solve, 2)
    r = ZRobot(maze, menu=menu)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'ZoroBot3 (OPRobots)', led=r.fw.rgb, clock=clock)
    a.max_s = time_limit(a, rec, 400)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    started = ended = False
    crash_t = -10000
    off_since = None     # its race stopped since (ms): over once stopped RACE_GAP_MS (an explore complete's end
                         # stops it while it saves its maze, then starts its speed run)

    def run(ms, fast=False):
        nonlocal started, ended, crash_t, off_since
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('firmware halted')
                return False
            clock.update(r.fw.time_ms(), r.pose())
            out = r.fw.received()
            if out:
                for line in out.splitlines():
                    if log:
                        log.write(f'[{r.fw.time_ms() / 1000:8.3f}] {line}\n')
            race = r.fw.race_started()
            started = started or race
            if race or not started:
                off_since = None
            elif off_since is None:
                off_since = r.fw.time_ms()
            if off_since is not None and r.fw.time_ms() - off_since >= RACE_GAP_MS and not ended:
                ended = True
                print(f'[{off_since / 1000:7.2f}] race over at {[round(v) for v in r.pose()]}')
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'[{r.fw.time_ms() / 1000:7.2f}] touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=not race)
        return True

    def say(text):
        print(f'[{r.fw.time_ms() / 1000:7.2f}] {text}')
        if rec:
            rec.status = text

    play(r, Z.start(a.run), lambda ms: run(ms, fast=True), say)
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
