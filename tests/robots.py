"""Every robot in the OSHWDEM 2026 maze, each run as its makers start it, checked by what it prints. The CI runs one
robot per job, in the short check; locally, any of them or all, short or full:

    python tests/robots.py [--full] [ratatron zorobot gamusino bulebule nezumibot perenquen fujin futura ukmarsbot miniskybot kerise]

The short check (the default, the CI's) asks each robot only the basics: its firmware builds, it is started as its
makers start it (its start sequence printed: Ratatron's search, ZoroBot3's and Fujin's "started: True", Bulebule's
"exploring", UKMARSBOT's "Off we go", NezumiBot's wall following, Perenquén's "keeping to the" wall, Gamusino's
"blue:", Futura's "run " on its display, KERISE v1's "started: True"), it runs for a few simulated seconds (SHORT_S)
without its firmware halting, resetting or faulting and without a Python exception, the runner ends with its end pose,
and the robot leaves the start cell: the stopwatch starts a run (ratasim/stopwatch.py, "run 1 started (left the start
cell)"). Wall touches fail Ratatron, NezumiBot and Futura, which run clean, and are only reported for the others. A
robot that does not leave its start cell in that time fails. Its runs do not depend on what happens later in the maze
(UKMARSBOT's way back from the goal, for one, is sensitive to numerical perturbations: 3 seeds in 10 touch a wall on
it).

The full check (--full) is the whole run, as before: it fails when the firmware does not build, stops, or the robot
does not do its run (Ratatron: the search to the goal and back, then race 2.4 on its map; ZoroBot3: its exploration
to the goal; Bulebule: its exploration, to the goal and back to its start cell, its path saved; Fujin: its search to
the goal and back; UKMARSBOT: its search to the goal and back to the start; NezumiBot and Perenquén: their wall
following, away from the start; Gamusino: its 1.5 m and back; Futura: its controller sample's cells and pivots, up
to the maze's north rows; Miniskybot: through 10 cells, not toppled; KERISE v1: its search to the goal and back
to its start cell). Wall touches as in the short check. The stopwatch's times are reported;
Ratatron, ZoroBot3, Fujin, UKMARSBOT and KERISE v1, which reach the goal, fail without one.

A robot with a model and no firmware yet is checked by its model (model()): it builds and stands on the floor.
ZoroBot3's meshes need its STEP (not in the repository, README): without it only its firmware is built.
"""
import math
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
MAZE = 'oshwdem2026'
CI = os.environ.get('GITHUB_ACTIONS') == 'true'
# The short check's simulated seconds per robot: about 5 s more than its start sequence and its leaving the start
# cell take in the OSHWDEM 2026 maze (the stopwatch's start: Futura 1.4 s, Perenquén 2.9, UKMARSBOT 3.7, Gamusino 4.4,
# KERISE v1 4.5, NezumiBot 6.2, Ratatron 6.8, ZoroBot3 8.9, Fujin 11.0, Bulebule 18.0)
SHORT_S = {'ratatron': 12, 'zorobot': 14, 'gamusino': 10, 'bulebule': 23, 'nezumibot': 11, 'perenquen': 8,
           'fujin': 16, 'futura': 6, 'ukmarsbot': 9, 'miniskybot': 10, 'kerise': 10}


def note(kind, text):
    """A line in the log, and an annotation on the CI run (kind: notice, warning, error)."""
    print(f'::{kind}::{text}' if CI else f'{kind}: {text}', flush=True)


def run(args, timeout=1800):
    cmd = [sys.executable, '-m', 'ratasim'] + args
    print('$ ratasim ' + ' '.join(args), flush=True)
    out = subprocess.run(cmd, cwd=REPO, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, timeout=timeout).stdout
    print('\n'.join(line for line in out.splitlines() if not line.startswith('@')), flush=True)
    if 'Traceback' in out:    # on the run's page too (annotations: no log needed): what came before (a build's
        before, _, after = out.partition('Traceback')    # errors) and the exception
        lines = before.strip().splitlines()[-20:] + ['...'] + after.strip().splitlines()[-2:]
        note('error', ' '.join(args[:1]) + ':%0A' + '%0A'.join(lines))
    return out


def end_pose(out):
    m = re.findall(r'^end: .* pose \[(-?\d+), (-?\d+), (-?\d+)\]', out, re.M)
    return tuple(map(int, m[-1])) if m else None


def goal_times(out):
    """The stopwatch's times (s) of the runs that reached the goal (ratasim/stopwatch.py)."""
    return [float(t) for t in re.findall(r'stopwatch: run \d+ at the goal \(\d+, \d+\) in (\d+\.\d+) s', out)]


def stopwatch(out, what=''):
    """Its times on the log and the CI run's page; the times to the goal."""
    times = goal_times(out)
    starts = len(re.findall(r'stopwatch: run \d+ started', out))
    note('notice', f'stopwatch{what}: {starts} run(s) from the start cell, '
         + (f'at the goal in {", ".join(f"{t:.3f} s" for t in times)}' if times else 'none at the goal'))
    return times


def touches(out):
    return re.findall(r'touches (\S+) at', out)


def common(out, fails, clean):
    """What every runner prints: its firmware stopping, its end pose, its wall touches, its stopwatch's times."""
    for bad in ('firmware halted', 'firmware reset', 'firmware fault', 'firmware stopped', 'its sketch stopped',
                'Traceback'):
        if bad in out:
            fails.append(bad)
    stopwatch(out)
    pose = end_pose(out)
    if pose is None:
        fails.append('no end pose')
    hits = touches(out)
    if hits:
        text = f'{len(hits)} wall touches ({", ".join(sorted(set(hits)))})'
        fails.append(text) if clean else note('warning', text)
    return pose


def far_from(pose, xy, mm):
    return pose is not None and math.dist(pose[:2], xy) > mm


def left_start(out, fails):
    """The short check's point: the stopwatch started a run, the robot's centre out of the start cell."""
    if not re.search(r'stopwatch: run \d+ started', out):
        fails.append('it did not leave the start cell (no run started on the stopwatch)')


def short(args, max_s, clean, started=None):
    """The short check (the CI's): started as its makers start it, for max_s simulated seconds (its start sequence
    and a few seconds more), its firmware built and running, and out of its start cell."""
    fails = []
    out = run(args + ['--headless', '--maze', MAZE, '--max-s', str(max_s)])
    common(out, fails, clean)
    if started and started not in out:
        fails.append(f'not started (no "{started}")')
    left_start(out, fails)
    return fails


# ---- each robot: a list of what went wrong (empty: passed); full: its whole run, else the short check

def ratatron(full):
    fails = []
    flash = os.path.join(tempfile.mkdtemp(), 'flash.bin')
    base = ['ratatron', '--headless', '--speed', '0', '--no-pty', '--flash', flash, '--maze', MAZE]
    if not full:
        out = run(base + ['--erase-flash', '--max-s', str(SHORT_S['ratatron'])])
        stopwatch(out)
        for bad in ('POSE LOST', 'touches a wall', '## firmware', 'Traceback'):     # its firmware halted or faulted
            if bad in out:
                fails.append(bad)
        if '## end:' not in out:
            fails.append('no end')
        left_start(out, fails)
        return fails
    base += ['--max-s', '600']
    for name, start in (('search', ['--erase-flash']), ('race 2.4', ['--race', '4'])):     # its start (config.py)
        out = run(base + start)
        if 'End: OK' not in out:
            fails.append(f'{name}: no "End: OK"')
        if 'Goal reached' not in out:
            fails.append(f'{name}: the goal not reached')
        elif not stopwatch(out, f' ({name})'):
            fails.append(f'{name}: no time to the goal (the stopwatch)')
        for bad in ('POSE LOST', 'touches a wall', 'Traceback'):
            if bad in out:
                fails.append(f'{name}: {bad}')
    return fails


def zorobot(full):
    from ratasim.zorobot import config as Z
    from ratasim.zorobot.robot import build
    build()
    if not os.path.exists(Z.CAD_STEP):
        note('notice', 'ZoroBot3: its firmware built; no run (its STEP is not in the repository)')
        return []
    if not full:
        return short(['zorobot'], SHORT_S['zorobot'], clean=False, started='started: True')
    fails = []
    out = run(['zorobot', '--headless', '--maze', MAZE, '--max-s', '400'])
    common(out, fails, clean=False)
    if 'started: True' not in out:
        fails.append('not started')
    if 'race over' not in out:
        fails.append('its exploration did not end')
    elif not goal_times(out):
        fails.append('no time to the goal (the stopwatch)')
    return fails


def gamusino(full):
    if not full:
        return short(['gamusino'], SHORT_S['gamusino'], clean=False, started='blue:')
    fails = []
    out = run(['gamusino', '--headless', '--maze', MAZE, '--max-s', '30'])
    pose = common(out, fails, clean=False)
    m = re.search(r'put down in cell \((\d+), (\d+)\)', out)
    if not m or 'blue:' not in out:
        fails.append('its run did not start')
    elif far_from(pose, ((int(m[1]) + 0.5) * 180, (int(m[2]) + 0.5) * 180), 120):
        fails.append(f'not back at its start cell ({pose})')
    return fails


def wall_follower(args, clean, full, started):
    if not full:
        return short(args, SHORT_S[args[0]], clean, started)
    fails = []
    out = run(args + ['--headless', '--maze', MAZE, '--max-s', '60'])
    pose = common(out, fails, clean)
    if 'started: False' in out:
        fails.append('not started')
    if not far_from(pose, (90, 90), 2 * 180):
        fails.append(f'still near the start ({pose})')
    return fails


def nezumibot(full):
    return wall_follower(['nezumibot', '--hand', 'left'], clean=True, full=full, started='started: True')


def perenquen(full):
    return wall_follower(['perenquen', '--side', 'right', '--gain', '30'], clean=False, full=full, started='keeping to the')


def futura(full):
    """Its kit's template with its page's controller sample (fw/main.c): "run " on its display, cells one after
    another to the walls, a stop in the middle of the cell and pivots to look for the way on, left, right and back.
    In 60 s it runs clean, at least 25 cells and 10 pivots, up the west column and round to the maze's north rows."""
    if not full:
        return short(['futura'], SHORT_S['futura'], clean=True, started='its display: "run "')
    fails = []
    out = run(['futura', '--headless', '--echo', '--maze', MAZE, '--max-s', '60'])
    pose = common(out, fails, clean=True)
    if 'its display: "run "' not in out:
        fails.append('its display never showed "run "')
    cells = [int(v) for v in re.findall(r'^cell (\d+)', out, re.M)]
    if not cells or cells[-1] < 25:
        fails.append(f'{cells[-1] if cells else "no"} cells (at least 25)')
    pivots = re.findall(r'^pivot \d+: (-?\d+) deg', out, re.M)
    if len(pivots) < 10 or not {'90', '-180', '-90'} <= set(pivots):
        fails.append(f'{len(pivots)} pivots, {sorted(set(pivots))} deg (at least 10, left, right and back)')
    walls = [int(v) for v in re.findall(r'^wall ahead: LF (\d+)', out, re.M)]
    if not walls or min(walls) < 1000:
        fails.append(f'not stopped in the middle of the cell at a wall (LF {min(walls) if walls else "none"})')
    if pose is None or pose[1] < 12 * 180:
        fails.append(f'not up in the north rows ({pose})')
    return fails


def ukmarsbot(full):
    if not full:
        return short(['ukmarsbot'], SHORT_S['ukmarsbot'], clean=False, started='Off we go')
    fails = []
    out = run(['ukmarsbot', '--headless', '--maze', MAZE, '--max-s', '400'])
    pose = common(out, fails, clean=False)
    if 'Off we go' not in out:
        fails.append('not started')
    if 'Arrived! (1)' not in out:
        fails.append('the goal not reached')
    elif not goal_times(out):
        fails.append('no time to the goal (the stopwatch)')
    if 'Arrived! (2)' not in out:
        fails.append('not back at the start')
    elif far_from(pose, (90, 90), 90):
        fails.append(f'not in the start cell ({pose})')
    return fails


def miniskybot(full):
    """Straight on, a turn at each wall. Short check:
    it says go and leaves the start cell. Full: in 120 s it also does not topple and goes through at least 10 of the
    maze's cells."""
    if not full:
        return short(['miniskybot'], SHORT_S['miniskybot'], clean=False, started='it says: go')
    fails = []
    out = run(['miniskybot', '--headless', '--maze', MAZE, '--max-s', '120'])
    common(out, fails, clean=False)
    if 'it says: go' not in out:
        fails.append('not started')
    if not re.search(r'stopwatch: run \d+ started', out):
        fails.append('it did not leave the start cell')
    if 'toppled' in out:
        fails.append('it toppled')
    m = re.search(r'(\d+) cells visited', out)
    if not m or int(m[1]) < 10:
        fails.append(f'{m[1] if m else "no"} cells visited (at least 10)')
    return fails


def model(name):
    """A robot with no firmware yet: its model builds, and after 2 s on a floor it still stands where it was put."""
    import importlib

    import mujoco
    import numpy as np

    M = importlib.import_module(f'ratasim.{name}.model')
    robot = M.robot_xml()         # builds its meshes first (its manifest)
    eq = M.equality_xml() if hasattr(M, 'equality_xml') else ''
    m = mujoco.MjModel.from_xml_string(f'''<mujoco>
  <compiler angle="degree" inertiafromgeom="auto"/>
  <option timestep="0.0005" integrator="implicitfast" cone="elliptic"/>
  <default>{M.defaults_xml()}</default>
  <asset>{M.assets_xml()}</asset>
  <worldbody><geom type="plane" size="1 1 0.1" friction="0.7"/>{robot}</worldbody>
  {f"<equality>{eq}</equality>" if eq else ""}
</mujoco>''')
    d = mujoco.MjData(m)
    free = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    if not free:
        return ['no robot (no free joint)']
    a = m.jnt_qposadr[free[0]]
    z0 = d.qpos[a + 2]
    while d.time < 2.0:
        mujoco.mj_step(m, d)
    if not np.all(np.isfinite(d.qpos)):
        return ['its state not finite']
    q = d.qpos[a:a + 7]
    print(f'{name}: {m.nbody} bodies, after 2 s {(q[2] - z0) * 1000:+.1f} mm in z, upright {abs(q[3]):.3f}')
    return [] if abs(q[2] - z0) < 0.01 and abs(q[3]) > 0.95 else [f'not standing on the floor ({q[2] - z0:+.3f} m)']


def bulebule(full):
    fails = []
    flash = os.path.join(tempfile.mkdtemp(), 'flash.bin')
    if not full:
        return short(['bulebule', '--erase-flash', '--flash', flash], SHORT_S['bulebule'], clean=False,
                     started='exploring')
    out = run(['bulebule', '--headless', '--maze', MAZE, '--erase-flash', '--flash', flash, '--max-s', '300'])
    pose = common(out, fails, clean=False)
    if 'exploring' not in out:
        fails.append('not started')
    if 'its path saved in its flash' not in out:
        fails.append('its exploration did not end (no path saved)')
    if far_from(pose, (90, 90), 90):
        fails.append(f'not back in its start cell ({pose})')
    if 'Collision detected' in out:
        fails.append('its firmware detected a collision')
    return fails


def fujin(full):
    if not full:
        return short(['fujin'], SHORT_S['fujin'], clean=False, started='started: True')
    fails = []
    out = run(['fujin', '--headless', '--maze', MAZE, '--max-s', '400'])
    pose = common(out, fails, clean=False)
    if 'started: True' not in out:
        fails.append('not started')
    if not goal_times(out):
        fails.append('the goal not reached (no time from the stopwatch)')
    if 'its search over' not in out:
        fails.append('its search did not end')
    elif far_from(pose, (90, 90), 120):
        fails.append(f'not back at the start ({pose})')
    return fails


def kerise(full):
    if not full:
        return short(['kerise'], SHORT_S['kerise'], clean=False, started='started: True')
    fails = []
    out = run(['kerise', '--headless', '--maze', MAZE, '--max-s', '170'])
    common(out, fails, clean=False)
    if 'started: True' not in out:
        fails.append('not started')
    if not goal_times(out):
        fails.append('the goal not reached (no time from the stopwatch)')
    back = re.findall(r'its search back to the start, at \[(-?\d+), (-?\d+), (-?\d+)\]', out)
    if not back:
        fails.append('its search not back to the start')
    elif far_from(tuple(map(int, back[-1])), (90, 30), 30):
        fails.append(f'not back in its start cell ({back[-1]})')
    return fails


ROBOTS = {'ratatron': ratatron, 'zorobot': zorobot, 'gamusino': gamusino, 'bulebule': bulebule,
          'nezumibot': nezumibot, 'perenquen': perenquen, 'fujin': fujin,
          'futura': futura, 'ukmarsbot': ukmarsbot, 'miniskybot': miniskybot, 'kerise': kerise}


def main(args):
    full = '--full' in args
    names = [a for a in args if a != '--full']
    for name in names:
        if name not in ROBOTS:
            sys.exit(f'unknown robot {name!r} (robots: {" ".join(ROBOTS)})')
    failed = []
    for name in names or ROBOTS:
        print(f'==== {name}{" (full)" if full else ""}', flush=True)
        fails = ROBOTS[name](full)
        for f in fails:
            note('error', f'{name}: {f}')
        print(f'==== {name}: {"FAILED" if fails else "OK"}', flush=True)
        failed += [name] if fails else []
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
