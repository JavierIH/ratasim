"""Gamusino in the simulator: its sketch (src/gamusino/gamusino.ino) on its simulated ESP32. What it runs: 1.5 m
ahead at 1.6 m/s steering to the point (1.5, 0) by its gyro and encoders, a stop, a 180 deg turn, a rest, and back;
over and over. It reads no wall sensor: it is put down in the start cell, facing the maze's first straight (in the OSHWDEM
2026 maze, sixteen cells along its west wall) and switched on, as its maker starts it.

    ratasim gamusino [--maze oshwdem2026] [--headless] [--record out.mp4] [--max-s N] [--echo]
"""
import argparse
import os
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from ..maze import Maze   # noqa: E402
from ..recorder import add_view_args, make_view, time_limit   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..start import play   # noqa: E402
from ..stopwatch import Stopwatch   # noqa: E402
from . import config as G   # noqa: E402
from .robot import GRobot   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim gamusino', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 30; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its sketch prints here')
    p.add_argument('--echo', action='store_true', help='print what its sketch prints')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    r = GRobot(maze, seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'Gamusino (turmandreams)', led=r.led, clock=clock)
    a.max_s = time_limit(a, rec, 30)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000

    def run(ms, fast=False):
        nonlocal crash_t
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('its sketch stopped')
                return False
            clock.update(r.fw.time_ms(), r.pose())
            out = r.fw.received()
            if out:
                if log:
                    log.write(out)
                if a.echo:
                    sys.stdout.write(out)
                    sys.stdout.flush()
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

    cell = (0, 0)       # the start cell, facing north: the column along the maze's west wall (16 cells, 2.88 m)
    say(f'put down in cell {cell}, the start cell')
    play(r, G.start(cell, 0.0), lambda ms: run(ms, fast=True), say)
    say('blue: 1.5 m ahead, a 180 deg turn, back')
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
