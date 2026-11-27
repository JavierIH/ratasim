"""Starting a robot as its makers do, the same way for every robot: its config's start sequence, an operator's
actions on the simulated robot with its firmware running between them.

    ('say', text)               what is being done (the console, the video's status line)
    ('wait', ms)                its firmware runs
    ('press', button[, ms])     a button held (100 ms)
    ('switches', value)         its function switches set (a DIP switch its firmware reads)
    ('send', line)              a line typed in a terminal connected to it (its serial port, Bluetooth)
    ('hand', sensors[, ms])     a hand in front of these sensors (300 ms; ratasim/hand.py)
    ('lift',)                   held in the air over where it stands (its sensors see over the walls)
    ('wheel', side, turns[, ms]) one of its wheels turned by hand, held up ('l' or 'r'; turns forward, 400 ms)
    ('put_down', cell, deg)     put down in a cell, facing deg (0 north, positive to the left): where in it, its
                                config's default (as its makers put it down) or --start (back_mm below)
"""
import argparse
import math

import numpy as np

from . import config as RC

# Where in its cell a robot is put down (--start, every runner): None its own default (as its makers put it down:
# ZoroBot3, Fujin, KOGUMA ... their tails against the back wall, as their firmwares count), 'wall' its tail against
# the wall behind it, 'centre' its axle on the cell's middle, or a number: its axle's mm ahead of the middle (negative:
# behind).
WHERE = None
WALL_GAP_MM = 0.3       # 'wall': its tail this far off the wall's face


class _Where(argparse.Action):
    def __call__(self, parser, namespace, value, option_string=None):
        global WHERE
        if value not in ('wall', 'centre', 'center'):
            try:
                float(value)
            except ValueError:
                parser.error(f'--start: wall, centre or a number of mm, not {value!r}')
        WHERE = 'centre' if value == 'center' else value
        setattr(namespace, self.dest, WHERE)


def add_start_arg(p):
    p.add_argument('--start', action=_Where, metavar='wall|centre|MM',
                   help='where in its start cell it is put down: its tail against the back wall, its axle on the '
                        'middle, or its axle MM ahead of the middle (negative: behind); by default as its makers put '
                        'it down')


def tail_mm(model, data, body):
    """How far behind its origin (its axle) its hull reaches: its colliding geoms' boxes, in its own frame."""
    import mujoco
    mujoco.mj_kinematics(model, data)
    rot = data.xmat[body].reshape(3, 3)
    origin = data.xpos[body]
    back = 0.0
    for g in range(model.ngeom):
        b = model.geom_bodyid[g]
        while b != body and b != 0:
            b = model.body_parentid[b]
        if b != body or not (model.geom_contype[g] or model.geom_conaffinity[g]):
            continue
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        centre, half = model.geom_aabb[g][:3], model.geom_aabb[g][3:]
        grot = data.geom_xmat[g].reshape(3, 3)
        for corner in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
            p = rot.T @ (data.geom_xpos[g] + grot @ (centre + corner * half) - origin)
            back = max(back, -p[1])
    return back / 0.001


def back_mm(model, data, body, default):
    """Its axle's mm behind its start cell's middle, as --start says (WHERE), else its default."""
    if WHERE is None:
        return default
    wall = RC.CELL_MM / 2 - RC.WALL_MM / 2 - tail_mm(model, data, body) - WALL_GAP_MM
    if WHERE == 'centre':
        return 0.0
    if WHERE == 'wall':
        return wall
    return min(-float(WHERE), wall)      # no further back than the wall lets it


def cell_pose(cell, heading_deg, back):
    """Its origin (mm) in a cell, facing heading_deg (0 north, positive to the left), back mm behind its middle."""
    h = math.radians(heading_deg)
    return ((cell[0] + 0.5) * RC.CELL_MM + back * math.sin(h), (cell[1] + 0.5) * RC.CELL_MM - back * math.cos(h))


def actions(robot, steps, say=print):
    """The steps done on the robot one by one; yields each wait (ms) for the caller to run its firmware: a runner
    with its own loop (Ratatron's, its viewer and console) takes them as they come."""
    for kind, *args in steps:
        if kind == 'say':
            say(*args)
        elif kind == 'wait':
            yield args[0]
        elif kind == 'press':
            robot.press(*args)
        elif kind == 'switches':
            robot.set_switches(*args)
        elif kind == 'send':
            robot.send(*args)
        elif kind == 'hand':
            robot.cover(*args)
        elif kind == 'lift':
            robot.lift()
        elif kind == 'wheel':
            robot.turn_wheel(*args)
        elif kind == 'put_down':
            robot.put_down(*args)
        else:
            raise ValueError(f'unknown start step {kind!r}')


def play(robot, steps, run, say=print):
    for ms in actions(robot, steps, say):
        run(ms)
