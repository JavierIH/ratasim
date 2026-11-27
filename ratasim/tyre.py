"""A robot's tyres sliding sideways in a turn, the same model for every robot that uses it.

A rubber tyre carries a sideways force only by running at a slant to where it points: its slip angle, in
proportion to the force over its load (its cornering stiffness, C per radian, the same for a load and a tyre's
make). In a turn the robot's centripetal force m v w makes its velocity point beta = m v w / (C N) outward of its
heading (N its load on the floor: its weight and its fan's downforce): its path bulges outward of the arc its gyro
and encoders follow. The micromouse community's model (docs of KOGUMA: turn_speed_research.md, "slip angle
proportional to the centrifugal force", theta = k v w; KERISE's k_slip; Ratatron's own firmware: CURVE_SLIP,
CURVE_PRE_SLIP), and what their firmwares' turns are tuned for: their turns start earlier and end later than the
arcs, as much as their tyres slide out.

Along its way the same: a tyre pushing (or braking) the robot turns a share faster (slower) than the floor goes by,
its slip ratio s = F / (C_long N), F its wheels' push. Its encoders count that much more than it goes (KOGUMA's maker:
in his runs at 2 m/s his odometry ran 13-38 mm ahead of where the posts put him; at 500 mm/s with no fan, 37 mm at
its start: docs/next_robot.md, pillar_cut_design.md).

MuJoCo's contacts hold a tyre to the floor without slipping until its grip runs out (their friction made stiff:
with MuJoCo's soft one they crept far more than a tyre slides). The slip is added on top: each step, the robot moved
sideways by its velocity times tan(beta), and back by s times its way, its wheels, encoders and gyro as they were.
"""
import math


G = 9.81


def slide(model, data, body, free_qpos, free_dof, mass_kg, load_n, cornering, dt, push_n=0.0, along=None):
    """Moves the robot sideways, outward of its turn, as its tyres slide (cornering: C, per radian), and back as
    its wheels' push (push_n, N forward) makes them slip (along: C_long, per unit of slip ratio); None: neither. Its
    frame: x right, y forward; its heading's rate the free joint's local z."""
    if not cornering and not along:
        return 0.0
    rot = data.xmat[body].reshape(3, 3)
    v = float(rot[:, 1] @ data.qvel[free_dof:free_dof + 3])      # forward speed (m/s)
    w = float(data.qvel[free_dof + 5])                            # yaw rate (rad/s), positive to the left
    load = max(load_n, 0.1)
    beta = 0.0
    if cornering:
        beta = max(-0.5, min(0.5, mass_kg * v * w / (cornering * load)))
        side = v * math.tan(beta) * dt                             # to the right when turning left
        data.qpos[free_qpos:free_qpos + 2] += (rot[:, 0] * side)[:2]
    if along:
        s = max(-0.5, min(0.5, push_n / (along * load)))
        data.qpos[free_qpos:free_qpos + 2] -= (rot[:, 1] * (s * v * dt))[:2]
    return beta
