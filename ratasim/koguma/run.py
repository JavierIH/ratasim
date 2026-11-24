"""KOGUMA-CHAN Mk3 in the simulator: switched on in the start cell, its mode 0 number picked by turning its right
wheel and its search started with a hand in front of its sensors, as its maker does; then its search runs, to the
goal (the maze's middle) and back, and what its number adds after it (8 to 11: its fastest run). Optionally records
a video.

    ratasim koguma [--maze oshwdem2026] [--number 1] [--headless] [--record out.mp4] [--max-s N] [--echo]

--number: its mode 0 number (its README): 1 its slalom search at 500 mm/s, no fan, to the goal and back; 3 to the
goal only; 4 its search at 1000 mm/s with its fan; 8 its slalom search, then its fastest run (Dijkstra, 2 m/s turns,
6 m/s straights, its fan at 70 %); 10 its contest run: its fan's search, the unknown walls on the fastest routes
looked at, then its fastest run.
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
from . import config as K   # noqa: E402
from .robot import KRobot   # noqa: E402

NUMBERS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim koguma', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--number', type=int, default=1, choices=NUMBERS, help='its mode 0 number (default 1)')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 400; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what its firmware prints')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    r = KRobot(maze, seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'KOGUMA-CHAN Mk3 (Tokoro0917)', led=r.led, clock=clock)
    a.max_s = time_limit(a, rec, 400)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    state = {'started': None, 'out': False, 'goal': None, 'idle_ms': 0, 'over': False}

    def now():
        return r.fw.time_ms() / 1000

    def watch():
        driven = r.fw.motor(0)[0] or r.fw.motor(1)[0]
        if state['started'] is None:
            if driven:
                state['started'] = now()
                print(f'[{now():7.2f}] it goes')
            return
        target = (r.fw.var('G_Gool_X'), r.fw.var('G_Gool_Y'))
        if target != (0, 0):
            state['out'] = True                     # its search's way out (its goal set in the maze's middle)
        if state['goal'] is None and state['out'] and target == (0, 0):
            state['goal'] = now()
            print(f'[{now():7.2f}] at the goal, {now() - state["started"]:.1f} s after it went; back to the start')
        state['idle_ms'] = 0 if driven else state['idle_ms'] + 1
        if state['idle_ms'] >= 300 and not state['over']:
            state['over'] = True
            fs = ' (its fail-safe stopped it)' if r.fw.lib.Failsafe_Flag() else ''
            print(f'[{now():7.2f}] its run over{fs} at {[round(v) for v in r.pose()]}, cell '
                  f'({r.fw.var("G_Robot_MAZE_X")}, {r.fw.var("G_Robot_MAZE_Y")})')

    def run(ms, fast=False):
        nonlocal crash_t
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
                        log.write(f'[{now():8.3f}] {line}\n')
                    if a.echo:
                        print(f'  | {line}')
            watch()
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'[{now():7.2f}] touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=fast)
        return True

    def say(text):
        if text.startswith('a hand in front'):
            mode, number = r.shows()
            text += f' (its LEDs: mode {mode}, No.{number})'
            if (mode, number) != (0, a.number):
                print(f'[{now():7.2f}] its LEDs show mode {mode}, No.{number}, not mode 0, No.{a.number}')
        print(f'[{now():7.2f}] {text}')
        if rec:
            rec.status = text

    play(r, K.start((0, 0), 0.0, a.number), lambda ms: run(ms, fast=True), say)
    run(1500, fast=True)
    print(f'[{now():7.2f}] started: {state["started"] is not None}')
    if rec:
        rec.status = f'mode 0, No.{a.number}: searching'
    while r.fw.time_ms() < a.max_s * 1000 and not state['over']:
        if not run(500):
            break
        if rec and state['goal'] is not None:
            rec.status = f'mode 0, No.{a.number}: back to the start'
    run(1000, fast=True)
    seen, wrong = r.map_errors()
    print(f'its map: {seen} walls and openings seen, ' + (f'{len(wrong)} wrong: ' + ', '.join(
        f'({x},{y}) {side} {"wall" if wall else "open"}' for x, y, side, wall in wrong) if wrong else 'all right'))
    print(f'end: {now():.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
