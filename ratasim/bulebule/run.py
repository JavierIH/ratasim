"""Bulebule in the simulator: boots its firmware (mmlib's search and runs) and starts it as its makers do, with its
button and a hand (config.start): it explores to the maze's centre and back to its start, saves the run's path in its
flash (ratasim/bulebule/build/flash.bin), and with --run reuses it for a run there and back.

    ratasim bulebule [--maze oshwdem2026] [--run] [--force-steps 3] [--erase-flash] [--flash FILE] [--headless]
                     [--record out.mp4] [--max-s N]

--force-steps: short presses at its force menu (0.1 N, +0.05 a press; 3: 0.25 N, its boot's). Its exploring speed
follows from it (speed.c); its runs go up to its LINEAR_SPEED_LIMIT (2 m/s). --console: then type commands to it
(its command.c: battery, move ..., set ...; Ctrl-D to stop). Its firmware's errors (a collision it detects) are
always printed.
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
from . import config as B   # noqa: E402
from .robot import FLASH_FILE, BRobot   # noqa: E402

STILL_MS = 3000             # done: its wheels still this long after moving


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim bulebule', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--run', action='store_true', help='a run on the path its last exploration saved')
    p.add_argument('--force-steps', type=int, default=3)
    p.add_argument('--erase-flash', action='store_true', help='forget the saved path first')
    p.add_argument('--flash', default=FLASH_FILE, help=f'its flash page between runs (default {FLASH_FILE})')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 300; with a window, none)')
    p.add_argument('--log', help='write everything the firmware sends here')
    p.add_argument('--echo', action='store_true', help='print its log messages (not its data lines)')
    p.add_argument('--console', action='store_true', help='then type commands to it (stdin)')
    a = p.parse_args(argv)
    r = BRobot(Maze.load(maze_path(a.maze)), flash_file=a.flash, erase=a.erase_flash)
    rec = make_view(r, a, 'Bulebule (Bulebots)')
    a.max_s = time_limit(a, rec, 300)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    pending = ''

    def run(ms, fast=False):
        nonlocal crash_t, pending
        end = r.fw.time_ms() + ms
        while r.fw.time_ms() < end:
            if r.step():
                print('firmware halted')
                return False
            out = r.fw.received()
            if out:
                if log:
                    log.write(out)
                pending += out
                *lines, pending = pending.split('\n')
                for line in lines:
                    if ',ERROR,' in line or (a.echo and ',DATA,' not in line):
                        print('  ' + line)
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

    saved, start_pose = r.fw.maze_saved(), r.pose()
    play(r, B.start(a.run, saved, a.force_steps), lambda ms: run(ms, fast=True), say)
    say('a run' if a.run else 'exploring')
    moved, still, erases = False, 0, r.fw.flash_erases()
    while r.fw.time_ms() < a.max_s * 1000:
        if a.console and select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline()
            if not line:
                break
            r.send(line.rstrip('\n'))
        before = r.pose()
        if not run(100):
            break
        now = r.pose()
        if abs(now[0] - before[0]) + abs(now[1] - before[1]) > 0.5:
            moved, still = True, 0
        else:
            still += 100
        if r.fw.flash_erases() != erases and r.fw.maze_saved():
            say('its path saved in its flash')
            erases = r.fw.flash_erases()
        if moved and still >= STILL_MS and not a.console:
            say('done')
            break
    r.fw.save_flash()
    x, y, h = r.pose()
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(x), round(y), round(h)]}'
          f' (started at {[round(v) for v in start_pose]})')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
