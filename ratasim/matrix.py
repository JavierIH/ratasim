#!/usr/bin/env python3
"""Robots x mazes x their runs, in parallel: which reach the goal, in what time, and where they touch a wall first.
For comparing the simulator's changes on more than one maze (a change that helps in one maze only is a fit to it).

    ratasim matrix OUT_DIR [--robots zorobot,koguma,fujin] [--mazes oshwdem2026,spiral,...] [--runs SUBSTR]
                           [--seeds 0,1] [--workers N]
    ratasim matrix OUT_DIR --table          # the table of a matrix run (or still running), from its logs

Each run is one robot's command (RUNS below, as its makers start it; ratasim ROBOT --help) with --headless, --maze and
--max-s, its log in OUT_DIR/ROBOT__MAZE__RUN[__sSEED].log; OUT_DIR/summary.csv, one row a run. The mazes default to
MAZES: the OSHWDEM 2026 one, the test mazes of ratasim/mazes (a spiral: straights and 90 deg turns between walls;
stairs: a diagonal; practice_E: an open field, turns with no walls to correct on) and two classic competition ones
(external/mazefiles: Portugal 2025's final, All Japan 2025's expert final).
"""
import argparse
import csv
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

from . import paths

MAZES = ['oshwdem2026', 'spiral', 'stairs', 'practice_E', 'Portugal-2025-Final', 'alljapan-046-2025-exp-fin']

# Each robot's runs: (name, its command's options, simulated seconds at most, takes --seed). The ones that reach the
# goal: their searches and their speed runs at each speed.
RUNS = {
    'ratatron': [('search', ['--speed', '0', '--no-pty', '--erase-flash'], 600, True)],
    'zorobot': [(f'run{s}', ['--run', '--speed', str(s)], 500, True) for s in (1, 2, 3, 4, 5)],
    'koguma': [(f'n{n}', ['--number', str(n)], 500, True) for n in (1, 4, 8, 9, 10)],
    'fujin': [(f'search-{m}', ['--mode', m], 400, True) for m in ('slow', 'medium', 'fast')]
             + [(f'known-{s}-{k}', ['--known', '--run-speed', s, '--run-mode', k], 90, True)
                for s in ('slow', 'medium', 'fast', 'super') for k in ('diagonals', 'smooth')],
    'kerise': [('search', [], 300, True)],
    'ukmarsbot': [('search', [], 400, True)],
    'bulebule': [('search', ['--erase-flash'], 300, False)],
}
FLASH = ('ratatron', 'bulebule')     # their flash in a file: one each run


def parse(text):
    """What a run's log tells: its stopwatch's runs (started, at the goal and when), its first wall touch, its end."""
    goals = re.findall(r'stopwatch: run (\d+) at the goal \((\d+), (\d+)\) in ([\d.]+) s', text)
    touch = re.search(r'\[\s*([\d.]+)\] the robot touches a wall at x=(-?\d+) y=(-?\d+) mm, heading (-?[\d.]+) deg '
                      r'\((\S+)\)', text)      # Ratatron's runner
    if touch:
        touch = (None, touch[1], touch[2], touch[3], touch[5])
    else:                                       # the others' (ratasim/recorder.py)
        touch = re.search(r'\[\s*([\d.]+)\] touches (\S+) at \[(-?\d+), (-?\d+), (-?\d+)\]', text)
        if touch:
            touch = (None, touch[1], touch[3], touch[4], touch[2])
    end = re.findall(r'^end: ([\d.]+) s simulated in (\d+) s, pose \[(-?\d+), (-?\d+), (-?\d+)\]', text, re.M)
    cell = (lambda x, y: f'({int(x) // 180},{int(y) // 180})')
    return {
        'starts': len(re.findall(r'stopwatch: run \d+ started', text)),
        'at_goal': len(goals),
        'goal_s': ' '.join(t for *_, t in goals),
        'first_touch': f'{touch[1]} s {cell(touch[2], touch[3])} {touch[4]}' if touch else '',
        'end_s': end[-1][0] if end else '',
        'end_cell': cell(*end[-1][2:4]) if end else '',
        'halted': ' '.join(re.findall(r'## firmware (\w+)', text)),
        'error': 'Traceback' if 'Traceback' in text else ('timeout' if '## TIMEOUT' in text else ''),
    }


def run_one(out, robot, maze, name, args, max_s, seed):
    tag = f'{robot}__{maze}__{name}' + ('' if seed is None else f'__s{seed}')
    log = os.path.join(out, tag + '.log')
    cmd = [sys.executable, '-m', 'ratasim', robot, '--headless', '--maze', maze, '--max-s', str(max_s), *args]
    if seed is not None:
        cmd += ['--seed', str(seed)]
    if robot in FLASH:
        cmd += ['--flash', os.path.join(out, tag + '.bin')]
    t0 = time.monotonic()
    with open(log, 'w') as f:
        f.write('$ ratasim ' + ' '.join(cmd[3:]) + '\n')
        f.flush()
        try:
            subprocess.run(cmd, cwd=paths.REPO, stdout=f, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                           env=dict(os.environ, MUJOCO_GL='egl'), timeout=max_s * 5 + 300)
        except subprocess.TimeoutExpired:
            f.write('\n## TIMEOUT\n')
    row = {'robot': robot, 'maze': maze, 'run': name, 'seed': '' if seed is None else seed,
           'wall_s': round(time.monotonic() - t0)}
    row.update(parse(open(log, errors='replace').read()))
    print(f"{robot:9} {maze:26} {name:24} {row['seed']!s:2} goal {row['goal_s'] or '-':18} "
          f"touch {row['first_touch'] or '-':28} end {row['end_cell']} {row['halted']} {row['error']}", flush=True)
    return row


def table(rows):
    """Robot and run down, mazes across: its times at the goal (its search's, then its speed run's), and x with the
    cell it first touched a wall in (or where it ended, if it reached no goal and touched nothing)."""
    mazes = list(dict.fromkeys(r['maze'] for r in rows))
    keys = list(dict.fromkeys((r['robot'], r['run']) for r in rows))
    cells = {}
    for r in rows:
        text = ' '.join(f'{float(t):.1f}' for t in r['goal_s'].split())
        if r['first_touch']:
            text += ' x' + r['first_touch'].split()[2]
        elif not text:
            text = 'x' + r['end_cell'] + ('?' if not r['end_s'] else '')
        cells.setdefault((r['robot'], r['run'], r['maze']), []).append(text.strip())
    w = max(len(f'{k[0]} {k[1]}') for k in keys) + 2
    cw = 20
    lines = [''.ljust(w) + ''.join(m[:cw - 2].ljust(cw) for m in mazes)]
    for k in keys:
        lines.append(f'{k[0]} {k[1]}'.ljust(w) + ''.join(' | '.join(cells.get(k + (m,), ['']))[:cw - 1].ljust(cw)
                                                         for m in mazes))
    return '\n'.join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out')
    p.add_argument('--robots', default='zorobot,koguma,fujin')
    p.add_argument('--mazes', default=','.join(MAZES))
    p.add_argument('--runs', default='', help='only the runs whose name has this in it')
    p.add_argument('--seeds', default='', help='seeds, comma separated (robots that take --seed)')
    p.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    p.add_argument('--table', action='store_true', help='only print the table of OUT/summary.csv')
    a = p.parse_args()
    summary = os.path.join(a.out, 'summary.csv')
    if a.table:         # from its logs (a matrix still running, or one stopped, has no summary yet)
        rows = []
        for n in sorted(os.listdir(a.out)):
            if n.endswith('.log'):
                robot, maze, name, *seed = n[:-4].split('__')
                rows.append({'robot': robot, 'maze': maze, 'run': name, 'seed': seed[0][1:] if seed else '',
                             **parse(open(os.path.join(a.out, n), errors='replace').read())})
        print(table(rows))
        return
    os.makedirs(a.out, exist_ok=True)
    seeds = [int(s) for s in a.seeds.split(',')] if a.seeds else [None]
    jobs = [(robot, maze, name, args, max_s, seed if seeded else None)
            for robot in a.robots.split(',') for maze in a.mazes.split(',')
            for name, args, max_s, seeded in RUNS[robot] if a.runs in name
            for seed in (seeds if seeded else [None])]
    jobs = list(dict.fromkeys((j[0], j[1], j[2], tuple(j[3]), j[4], j[5]) for j in jobs))
    jobs.sort(key=lambda j: -j[4])      # the long ones first
    with ThreadPoolExecutor(a.workers) as ex:
        rows = list(ex.map(lambda j: run_one(a.out, *j), jobs))
    rows.sort(key=lambda r: (r['robot'], r['run'], r['maze'], str(r['seed'])))
    with open(summary, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('\n' + table(rows))


if __name__ == '__main__':
    main()
