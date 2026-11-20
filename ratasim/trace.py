"""A robot's run as it went, for the scenes that put several runs together (ratasim/scenes/maze_race.py): every
runner's --trace NPZ writes its robot's joints (their qpos) and its pose every 10 simulated ms, nothing on screen.

    ratasim fujin --headless --trace fujin.npz
"""
import numpy as np

SIZES = {0: 7, 1: 4, 2: 1, 3: 1}        # a free joint's qpos, a ball's, a slide's, a hinge's


class Tracer:
    """What --trace makes in place of a window or a video (the same calls: maybe_frame(), close(), .status, .clock).
    The robot: .model, .data, .pose() (mm, deg) and .fw.time_ms()."""

    def __init__(self, robot, path, every_ms=10):
        self.r, self.path, self.every = robot, path, every_ms
        m = robot.model
        self.joints = [(m.joint(i).name, int(m.jnt_qposadr[i]), SIZES[int(m.jnt_type[i])])
                       for i in range(m.njnt) if m.joint(i).name]
        self.t, self.pose, self.qpos = [], [], []
        self.next_ms = -1
        self.status, self.clock = '', None

    def maybe_frame(self, fast=False):
        t = self.r.fw.time_ms()
        if t < self.next_ms:
            return
        self.next_ms = t + self.every
        q = self.r.data.qpos
        self.t.append(t)
        self.pose.append(self.r.pose())
        self.qpos.append(np.concatenate([q[a:a + n] for _, a, n in self.joints]))

    def close(self):
        np.savez_compressed(self.path, t=np.array(self.t, np.int64), pose=np.array(self.pose, np.float32),
                            qpos=np.array(self.qpos, np.float64), names=np.array([j[0] for j in self.joints]),
                            sizes=np.array([j[2] for j in self.joints], np.int64))
        print(f'trace: {len(self.t)} poses to {self.path}')


class Trace:
    """A --trace file: .t (ms), .pose (x, y mm, heading deg), and each joint's qpos at those times (at())."""

    def __init__(self, path):
        z = np.load(path)
        self.t, self.pose, self.q = z['t'], z['pose'], z['qpos']
        self.slices, i = {}, 0
        for name, n in zip(z['names'], z['sizes']):
            self.slices[str(name)] = slice(i, i + int(n))
            i += int(n)

    def index(self, t_ms):
        """The last pose at or before t_ms (the first before the trace starts, the last after it ends)."""
        return int(np.clip(np.searchsorted(self.t, t_ms, side='right') - 1, 0, len(self.t) - 1))

    def at(self, t_ms):
        """{joint: qpos} at t_ms."""
        row = self.q[self.index(t_ms)]
        return {name: row[s] for name, s in self.slices.items()}
