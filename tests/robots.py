"""Every robot in the OSHWDEM 2026 maze, each run as its makers start it, checked by what it prints. The CI runs one
robot per job; locally, any of them or all:

    python tests/robots.py [ratatron zorobot gamusino bulebule nezumibot perenquen]

A check fails when the firmware does not build, stops, or the robot does not do its run (Ratatron: the search to
the goal and back, then race 2.4 on its map; ZoroBot3: its exploration to the goal; NezumiBot and Perenquén: their
wall following, away from the start; Gamusino: its 1.5 m and back). Wall touches fail the robots that run clean
(Ratatron, NezumiBot); for the others, whose touches are known (robots.md), they are only reported.
A robot with a model and no firmware yet (Bulebule) is checked by its model: it builds and stands on the floor.
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


def note(kind, text):
    """A line in the log, and an annotation on the CI run (kind: notice, warning, error)."""
    print(f'::{kind}::{text}' if CI else f'{kind}: {text}', flush=True)


def run(args, timeout=1800):
    cmd = [sys.executable, '-m', 'ratasim'] + args
    print('$ ratasim ' + ' '.join(args), flush=True)
    out = subprocess.run(cmd, cwd=REPO, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, timeout=timeout).stdout
    print('\n'.join(line for line in out.splitlines() if not line.startswith('@')), flush=True)
    return out


def end_pose(out):
    m = re.findall(r'^end: .* pose \[(-?\d+), (-?\d+), (-?\d+)\]', out, re.M)
    return tuple(map(int, m[-1])) if m else None


def touches(out):
    return re.findall(r'touches (\S+) at', out)


def common(out, fails, clean):
    """What every runner prints: its firmware stopping, its end pose, its wall touches."""
    for bad in ('firmware halted', 'firmware reset', 'its sketch stopped', 'Traceback'):
        if bad in out:
            fails.append(bad)
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


# ---- each robot: a list of what went wrong (empty: passed)

def ratatron():
    fails = []
    flash = os.path.join(tempfile.mkdtemp(), 'flash.bin')
    base = ['--headless', '--speed', '0', '--no-pty', '--flash', flash, '--maze', MAZE, '--until-idle', '--max-s', '600']
    for name, cmds in (('search', ['--erase-flash', '--cmd', 'MODE 1']), ('race 2.4', ['--cmd', 'MODE 2 4'])):
        out = run(base + cmds + ['--cmd', 'START'])
        if 'End: OK' not in out:
            fails.append(f'{name}: no "End: OK"')
        if 'Goal reached' not in out:
            fails.append(f'{name}: the goal not reached')
        for bad in ('POSE LOST', 'touches a wall', 'Traceback'):
            if bad in out:
                fails.append(f'{name}: {bad}')
    return fails


def zorobot():
    from ratasim.zorobot import config as Z
    from ratasim.zorobot.robot import build
    build()
    if not os.path.exists(Z.CAD_STEP):
        note('notice', 'ZoroBot3: its firmware built; no run (its STEP is not in the repository)')
        return []
    fails = []
    out = run(['zorobot', '--maze', MAZE, '--max-s', '400'])
    common(out, fails, clean=False)
    if 'started: True' not in out:
        fails.append('not started')
    if 'race over' not in out:
        fails.append('its exploration did not end')
    return fails


def gamusino():
    fails = []
    out = run(['gamusino', '--maze', MAZE, '--max-s', '30'])
    pose = common(out, fails, clean=False)
    m = re.search(r'put down in cell \((\d+), (\d+)\)', out)
    if not m or 'blue:' not in out:
        fails.append('its run did not start')
    elif far_from(pose, ((int(m[1]) + 0.5) * 180, (int(m[2]) + 0.5) * 180), 120):
        fails.append(f'not back at its start cell ({pose})')
    return fails


def wall_follower(args, clean):
    fails = []
    out = run(args + ['--maze', MAZE, '--max-s', '60'])
    pose = common(out, fails, clean)
    if 'started: False' in out:
        fails.append('not started')
    if not far_from(pose, (90, 90), 2 * 180):
        fails.append(f'still near the start ({pose})')
    return fails


def nezumibot():
    return wall_follower(['nezumibot', '--hand', 'left'], clean=True)


def perenquen():
    return wall_follower(['perenquen', '--side', 'right', '--gain', '30'], clean=False)


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


def bulebule():
    return model('bulebule')


ROBOTS = {'ratatron': ratatron, 'zorobot': zorobot, 'gamusino': gamusino, 'bulebule': bulebule,
          'nezumibot': nezumibot, 'perenquen': perenquen}


def main(names):
    failed = []
    for name in names or ROBOTS:
        print(f'==== {name}', flush=True)
        fails = ROBOTS[name]()
        for f in fails:
            note('error', f'{name}: {f}')
        print(f'==== {name}: {"FAILED" if fails else "OK"}', flush=True)
        failed += [name] if fails else []
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
