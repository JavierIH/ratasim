"""Fujin in the simulator: switches it on in the start cell, picks its search with its buttons and starts it with a
hand in front of its front sensors, as its makers do; then its search runs: to the goal and back to the start, its map
in its EEPROM. With --run, then a speed run on that map, started from its menu as its makers do. Optionally records
a video.

    ratasim fujin [--maze oshwdem2026] [--mode slow|medium|fast] [--run|--known] [--run-speed S] [--run-mode M] [--backup]
                  [--headless] [--record out.mp4] [--max-s N] [--echo]

--mode: its search's speeds (slow: 0.3 m/s, no fan; medium and fast, faster, its fan on).
--run: after its search, put back in the start cell, a speed run to the goal on the map it saved; --run-speed its
speeds (custom, slow, medium, fast, super), --run-mode its moves (smooth, diagonals, time: the path its time-based
floodfill picks, hardcoded: its makers' fixed sequence), --backup the map it saved at the goal.
--known: no search, the maze written in its EEPROM (every wall known), then its run.
"""
import argparse
import os
import re
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
    p = argparse.ArgumentParser(prog='ratasim fujin', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--mode', default='slow', choices=('slow', 'medium', 'fast'))
    p.add_argument('--run', action='store_true', help='after its search, a speed run on the map it saved')
    p.add_argument('--run-speed', default='slow', choices=F.RUN_SPEEDS)
    p.add_argument('--run-mode', default='smooth', choices=F.RUN_MODES)
    p.add_argument('--backup', action='store_true', help='its run on the map it saved at the goal')
    p.add_argument('--dump-log', metavar='TXT',
                   help="after its run, its Logger's record of it (button 1 held in idle: its print_log, as its "
                        "makers' logs in external/fujin/scripts/logs) written here")
    p.add_argument('--known', action='store_true',
                   help='no search: the maze written in its EEPROM (every wall known), then its run (--run)')
    add_view_args(p)
    p.add_argument('--max-s', type=float, help='stop after these simulated seconds (default 300; with a window, none)')
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--log', help='write everything its firmware prints here')
    p.add_argument('--echo', action='store_true', help='print what its firmware prints')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    r = FRobot(maze, seed=a.seed)
    clock = Stopwatch(r.maze)
    rec = make_view(r, a, 'Fujin (Team Raiju)', led=r.led, clock=clock)
    a.max_s = time_limit(a, rec, 300)
    log = open(a.log, 'w') if a.log else None
    t0 = time.monotonic()
    crash_t = -10000
    state = {'now': None, 'searched': False, 'over': False, 'ran': False, 'run_over': False}

    dump = []

    def firmware_said(text):
        if state['run_over'] and a.dump_log:
            dump.append(text)
        for line in text.splitlines():
            if log:
                log.write(f'[{r.fw.time_ms() / 1000:8.3f}] {line}\n')
            if a.echo:
                print(f'  | {line}')
            m = re.match(r'state:(\w+)', line)
            if m:
                if m[1] == 'Search':
                    state['searched'] = True
                    print(f'[{r.fw.time_ms() / 1000:7.2f}] its search started')
                elif m[1] == 'Idle' and state['searched'] and not state['over']:
                    state['over'] = True
                    print(f'[{r.fw.time_ms() / 1000:7.2f}] its search over at {[round(v) for v in r.pose()]}')
                elif m[1] == 'Run':
                    state['ran'] = True
                    print(f'[{r.fw.time_ms() / 1000:7.2f}] its run started')
                elif m[1] == 'Idle' and state['ran'] and not state['run_over']:
                    state['run_over'] = True
                    print(f'[{r.fw.time_ms() / 1000:7.2f}] its run over at {[round(v) for v in r.pose()]}'
                          f' in {list(clock.cell(r.pose()))}')
                state['now'] = m[1]

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
                firmware_said(out)
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

    if a.known:
        a.run = state['over'] = True
        play(r, [('say', 'switched on'), ('wait', 3000)], run, say)
        r.load_maze()
        say('its maze written in its EEPROM')
    else:
        play(r, F.start((0, 0), 0.0, a.mode), lambda ms: run(ms, fast=True), say)
        print(f'[{r.fw.time_ms() / 1000:7.2f}] started: {state["searched"] or state["now"] == "Search"}')
        if rec:
            rec.status = f'searching ({a.mode}): to the goal and back'
        while r.fw.time_ms() < a.max_s * 1000 and not state['over']:
            if not run(500):
                break
        run(1000, fast=True)
    if a.run and state['over']:
        play(r, F.run_start((0, 0), 0.0, a.run_speed, a.run_mode, a.backup), lambda ms: run(ms, fast=True), say)
        print(f'[{r.fw.time_ms() / 1000:7.2f}] run started: {state["ran"]}')
        if rec:
            rec.status = f'its run ({a.run_speed}, {a.run_mode}): to the goal'
        end = r.fw.time_ms() + 60000
        while r.fw.time_ms() < end and state['ran'] and not state['run_over']:
            if not run(500):
                break
        run(1000, fast=True)
        if a.dump_log and state['run_over']:
            dump.clear()
            r.press(1, 900)
            quiet = 0
            while quiet < 3 and run(1000, fast=True):     # until it has printed it all
                quiet = 0 if dump and dump[-1] is not None else quiet + 1
                dump.append(None)
            with open(a.dump_log, 'w') as f:
                f.write(''.join(t for t in dump if t).replace('\r', ''))
            print(f'its log written to {a.dump_log}')
    print(f'end: {r.fw.time_ms() / 1000:.1f} s simulated in {time.monotonic() - t0:.0f} s, pose {[round(v) for v in r.pose()]}')
    if rec:
        rec.close()
    if log:
        log.close()
    r.fw.close()


if __name__ == '__main__':
    sys.exit(main())
