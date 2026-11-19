"""Futura in the simulator: its kit's firmware template (FuturaProject.zip, as its page gives it) on its simulated
STM32F405. What it runs (main.c): a beep, then over and over its sensors read and printed on its serial port (the
four IR pairs, the gyro, the battery, the encoders), "mous" on its display, both motors at 100 of 999 and a second's
wait. It has no start and steers by nothing: put down in the start cell, facing north, and switched on, it rolls
ahead at a tenth of its battery until the column ends (in the OSHWDEM 2026 maze, ten cells along its west wall)
and stays against the wall, its front sensors printing it.

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
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 15; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what its firmware prints')
    a = p.parse_args(argv)
    r = FRobot(Maze.load(maze_path(a.maze)), seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'Futura (Green Ye, UCLA IEEE)', clock=clock)
    a.max_s = time_limit(a, rec, 15)
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
