"""Perenquen in the simulator: boots its firmware (its last commit's: a command line over Bluetooth and a wall
follower, `start left|right <gain>`) and starts it as its maker did: in its start cell, the command typed in a terminal
(it reads its diagonal sensors there), then a hand in front of both front sensors, taken away.

    ratasim perenquen [--maze oshwdem2026] [--side right|left] [--gain 30] [--speed N] [--headless] [--record out.mp4] [--max-s N]

--speed: `traj_speed distance N` first (its units: 10 encoder counts a millisecond, 1000 = 1.96 m/s; it boots at 400,
0.78 m/s). Its notes in commands_mainboard.c: "OK speed 1000 gain 100", "OK speed 1500 gain 200"; its last commit:
"works fine with start gain 30". --console: the terminal is its command line afterwards (Ctrl-D to stop).
"""
import argparse
import os
import select
import sys
import time

os.environ.setdefault('MUJOCO_GL', 'egl')

from ..maze import Maze   # noqa: E402
from ..recorder import add_view_args, make_view, time_limit   # noqa: E402
from ..run import maze_path   # noqa: E402
from ..start import play   # noqa: E402
from . import config as P   # noqa: E402
from .robot import PRobot   # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim perenquen', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--side', choices=('left', 'right'), default='right', help='the wall it keeps to')
    p.add_argument('--gain', type=int, default=30)
    p.add_argument('--speed', type=int, help='traj_speed distance N before starting')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 60; with a window, none)')
    p.add_argument('--log', help='write everything the firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what the firmware prints')
    p.add_argument('--console', action='store_true', help='then type commands to it (stdin)')
    a = p.parse_args(argv)
    r = PRobot(Maze.load(maze_path(a.maze)))
    rec = make_view(r, a, 'Perenquen (supernudo)')
    a.max_s = time_limit(a, rec, 60)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000

    def run(ms, fast=False):
        nonlocal crash_t
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print(f'firmware {"reset" if r.fw.state == 1 else "halted"}')
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

    play(r, P.start(a.side, a.gain, a.speed), lambda ms: run(ms, fast=True), say)
    say(f'keeping to the {a.side} wall, gain {a.gain}')
    while r.fw.time_ms() < a.max_s * 1000:
        if a.console and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            r.send(line.rstrip('\n'))
        if not run(100 if a.console else 500):
            break
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
