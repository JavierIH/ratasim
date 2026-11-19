"""UKMARSBOT in the simulator: boots mazerunner-core as Peter Harrison's ORION, set for a UK contest (the goal at
(7, 7); config.py says which commit and why) and starts it as there: backed up to the start cell's back wall,
switched on with its button held (its maze map cleared), its function switches set and its button pressed, a hand
in front of its left front sensor. Function 2 (default): its search, to the goal and back to the start, mapping the
maze; 3: a left wall follower to the goal.

    ratasim ukmarsbot [--maze oshwdem2026] [--function 2] [--headless] [--record out.mp4] [--max-s N] [--echo]

--console: the terminal is its serial command line afterwards (its CLI: ? for help; Ctrl-D to stop).
"""
import argparse
import os
import re
import select
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from ..maze import Maze   # noqa: E402
from ..recorder import add_view_args, make_view, time_limit   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..start import play   # noqa: E402
from ..stopwatch import Stopwatch   # noqa: E402
from . import config as U   # noqa: E402
from .robot import URobot   # noqa: E402

FUNCTIONS = {2: 'its search: to the goal and back to the start', 3: 'its wall follower: the left wall to the goal'}


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim ukmarsbot', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--function', type=int, default=2, choices=sorted(FUNCTIONS), help='its function switches')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 240; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what its firmware prints')
    p.add_argument('--console', action='store_true', help='then type commands to it (stdin)')
    a = p.parse_args(argv)
    r = URobot(Maze.load(maze_path(a.maze)), seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'UKMARSBOT (UKMARS) running mazerunner-core', led=r.led, clock=clock)
    a.max_s = time_limit(a, rec, 240)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    printed = ''
    done = [0]                      # its runs' ends ("Arrived!")
    partial = ['']                  # what it printed after its last whole line

    def run(ms, fast=False):
        nonlocal crash_t, printed
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('its firmware stopped')
                return False
            clock.update(r.fw.time_ms(), r.pose())
            out = r.fw.received()
            if out:
                if log:
                    log.write(out)
                if a.echo:
                    sys.stdout.write(out)
                    sys.stdout.flush()
                printed = (printed + out)[-4000:]
                *lines, partial[0] = (partial[0] + out).split('\n')     # whole lines: its output comes in pieces
                for line in re.findall(r'(Off we go|Arrived!|ERR[^\r\n]*)', '\n'.join(lines)):
                    done[0] += line == 'Arrived!'
                    say({'Off we go': 'it says: Off we go...', 'Arrived!': f'it says: Arrived! ({done[0]})'}.get(line, line))
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

    play(r, U.start(a.function), lambda ms: run(ms, fast=True), say)
    say(f'function {a.function}: {FUNCTIONS[a.function]}')
    runs = 2 if a.function == 2 else 1
    while r.fw.time_ms() < a.max_s * 1000:
        if a.console and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            r.send(line.rstrip('\n'))
        if not run(100 if a.console else 500):
            break
        if done[0] >= runs and not a.console and not rec:
            run(1000)
            say('its run is over')
            break
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, '
          f'pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
