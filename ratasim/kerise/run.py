"""KERISE v2 in the simulator, in the classic maze (its firmware's for it): switched on in the start cell, its mode
picked with its button (0: its search) and started with a hand in front of its front sensors, as its firmware asks;
then its MazeSolver: its search to the goal and back to the start, then its fast runs. Optionally records a video.

    ratasim kerise [--maze oshwdem2026] [--headless] [--record out.mp4] [--max-s N] [--echo]
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
from ..stopwatch import Stopwatch   # noqa: E402 (its cells the classic maze's, 180 mm)
from . import config as K   # noqa: E402
from .robot import KRobot   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim kerise', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 300; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what its firmware prints')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    r = KRobot(maze, seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'KERISE v2 (kerikun11)', led=r.led, clock=clock)
    a.max_s = time_limit(a, rec, 300)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    state = {'started': False, 'back': False, 'fast': 0}

    def now():
        return f'[{r.fw.time_ms() / 1000:7.2f}]'

    def firmware_said(text):
        for line in text.splitlines():
            if log:
                log.write(f'[{r.fw.time_ms() / 1000:8.3f}] {line}\n')
            if a.echo:
                print(f'  | {line}')
            if line.startswith('start_step') and not state['started']:
                state['started'] = True
                print(f'{now()} its search started')
            elif line.startswith('maze_backup'):
                print(f'{now()} its search at the goal, at {[round(v) for v in r.pose()]}')
            elif line.startswith('agent.calcRunSequence') and not state['back']:
                state['back'] = True
                print(f'{now()} its search back to the start')
            elif line.startswith('Fast Run'):
                state['fast'] += 1
                print(f'{now()} its fast run {state["fast"]}')
            elif line.startswith(('Battery Low', 'Released Emergency')) or 'Calibration' in line:
                print(f'{now()} {line.strip()}')

    def run(ms, fast=False):
        nonlocal crash_t
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('firmware fault')
                return False
            clock.update(r.fw.time_ms(), r.pose())
            out = r.fw.received()
            if out:
                firmware_said(out)
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'{now()} touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=fast)
        return True

    def say(text):
        print(f'{now()} {text}')
        if rec:
            rec.status = text

    play(r, K.start((0, 0), 0.0), lambda ms: run(ms, fast=True), say)
    print(f'{now()} started: {state["started"]}')
    if rec:
        rec.status = 'its search: to the goal and back, then its fast runs'
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
