"""Futura in the simulator: its kit's firmware template (FuturaProject.zip, as its page gives it) with its page's
controller sample (controller-sample.c) and a main routine of ratasim's for it (fw/main.c), on its simulated
STM32F405. A beep, "run " on its display, then cells one after another with the sample's moveOneCell() (its speed
profile and PD controller each millisecond, its encoders and its side IR pairs' error as feedback, 0.5 m/s), to a
wall seen by its front pairs, where it stops in the cell's middle and looks for the way on: a pivot left (its
lecture's guide: the gyro's angle ends it), then right, then back. Its serial port prints each cell, wall and pivot.
In the OSHWDEM 2026 maze it goes up the west column (ten cells), round the north-west corner and along the north
rows; from 33 s on it runs to and fro between two dead ends of the north row, (1,14) and (6,13).

    ratasim futura [--maze oshwdem2026] [--headless] [--record out.mp4] [--max-s N] [--echo]
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
from . import config as F   # noqa: E402
from .robot import FRobot   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim futura', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 60; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what its firmware prints')
    a = p.parse_args(argv)
    r = FRobot(Maze.load(maze_path(a.maze)), seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'Futura (Green Ye, UCLA IEEE)', clock=clock)
    a.max_s = time_limit(a, rec, 60)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    shown = {'display': None, 'buzzer': False}
    lines, pending = 0, ''

    def say(text):
        print(f'[{r.fw.time_ms() / 1000:7.2f}] {text}')
        if rec:
            rec.status = text

    def run(ms, fast=False):
        nonlocal crash_t, lines, pending
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print(f'firmware {"reset" if r.fw.state == 1 else "halted"}')
                return False
            clock.update(r.fw.time_ms(), r.pose())
            out = r.fw.received()
            if out:
                lines += out.count('\n')
                if log:
                    log.write(out)
                if a.echo:          # whole lines, so as not to be cut by the runner's own
                    pending += out
                    done, _, pending = pending.rpartition('\n')
                    if done:
                        print(done.replace('\r', ''), flush=True)
            if r.fw.buzzing() != shown['buzzer']:
                shown['buzzer'] = r.fw.buzzing()
                say('its buzzer ' + ('on' if shown['buzzer'] else 'off'))
            text = r.fw.display()
            if text.strip() and text != shown['display']:
                shown['display'] = text
                say(f'its display: "{text}"')
            hit = r.crashed()
            if hit and r.fw.time_ms() - crash_t > 1000:
                crash_t = r.fw.time_ms()
                print(f'[{r.fw.time_ms() / 1000:7.2f}] touches {hit} at {[round(v) for v in r.pose()]}')
            if rec:
                rec.maybe_frame(fast=fast)
        return True

    play(r, F.start((0, 0), 0.0), lambda ms: run(ms, fast=True), say)
    while r.fw.time_ms() < a.max_s * 1000:
        if not run(500):
            break
    print(f'its serial port: {lines} lines')
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
