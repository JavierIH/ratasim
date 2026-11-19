"""Gamusino in the simulator: its sketch (src/gamusino/gamusino.ino) on its simulated ESP32. What it runs: 1.5 m
ahead at 1.6 m/s steering to the point (1.5, 0) by its gyro and encoders, a stop, a 180 deg turn, a rest, and back;
over and over. It reads no wall sensor: it is put down at the start of the maze's longest straight (in the OSHWDEM
2026 maze, ten cells along its south wall) and switched on, as its maker starts it.

    ratasim gamusino [--maze oshwdem2026] [--headless] [--record out.mp4] [--max-s 30] [--echo]
"""
import argparse
import os
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from ..maze import Maze   # noqa: E402
from ..recorder import add_view_args, make_view   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..scenes.crash import longest_straight   # noqa: E402
from ..start import play   # noqa: E402
from . import config as G   # noqa: E402
from .robot import GRobot   # noqa: E402


def start_cell(maze):
    """The first cell of the maze's longest straight, facing along it: (cell, heading deg, cells)."""
    n, horizontal, k, i0, _ = longest_straight(maze)
    return ((i0, k), -90.0, n) if horizontal else ((k, i0), 0.0, n)


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim gamusino', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    add_view_args(p)
    p.add_argument('--max-s', type=float, default=30.0)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its sketch prints here')
    p.add_argument('--echo', action='store_true', help='print what its sketch prints')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    r = GRobot(maze, seed=a.seed)
    rec = make_view(r, a, 'Gamusino (turmandreams)', led=r.led)
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

    cell, heading, cells = start_cell(maze)
    say(f'put down in cell {cell}, at the start of a straight of {cells} cells ({cells * 0.18:.2f} m)')
    play(r, G.start(cell, heading), lambda ms: run(ms, fast=True), say)
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
