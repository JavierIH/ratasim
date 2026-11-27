"""Miniskybot simulated: MuJoCo physics, its two hacked servos and its SRF02 around its program on a simulated
Skymega (miniskybot/fw, through ctypes). Every half millisecond: half a millisecond of its program, its servos'
torques from the pulses it sends them, one physics step; when its program starts a ranging, the SRF02's echo.

Its servos (config.py): each one's pulse every 20 ms; its control drives its motor in proportion to the pulse's
distance from its middle, its torque a DC motor's from its stall torque to its free speed.

Its SRF02: rays across its beam, level from its transducer's face; a wall gives an echo where a ray meets it within
ECHO_DEG of square (ultrasound is mirrored by a flat wall: only what faces it echoes back), the nearest such one is
its range; a small face (a post's, a wall's end: 12 mm) echoes too little to be heard but near its axis; under
16 cm it gives its minimum, beyond 6 m nothing (0).
"""
import ctypes as C
import math
import os
import random
import shutil
import subprocess
import tempfile

import mujoco
import numpy as np

from .. import config as RC
from ..model import MM, WALL_GROUP
from ..start import back_mm, cell_pose
from . import config as S
from . import model

FW_DIR = os.path.join(S.HERE, 'fw')
LIB = os.path.join(S.BUILD, 'libminiskybotfw.so')
SERVO_FRAME_MS = 20


def build():
    """Its program as a library (fw/Makefile)."""
    subprocess.run(['make', '-s', '-C', FW_DIR], check=True)
    return LIB


class MFirmware:
    """One boot of its program: its own copy of the library (its globals start as on the chip)."""

    def __init__(self, lib_path):
        self._tmp = tempfile.NamedTemporaryFile(prefix='miniskybotfw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.msim_run_until.argtypes = [C.c_uint64]
        lib.msim_run_until.restype = C.c_uint8
        lib.msim_time_ns.restype = C.c_uint64
        lib.msim_set_input.argtypes = [C.c_int, C.c_int]
        lib.msim_pin.argtypes = [C.c_int]
        lib.msim_servo_us.argtypes = [C.c_int]
        lib.msim_set_echo.argtypes = [C.c_double]
        lib.msim_serial_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.msim_serial_tx.restype = C.c_uint32
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        if not lib.msim_boot():
            raise RuntimeError('msim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def run_until(self, t_ns):
        self.state = self.lib.msim_run_until(int(t_ns))
        return self.state

    def time_ns(self):
        return self.lib.msim_time_ns()

    def time_ms(self):
        return self.lib.msim_time_ns() // 1000000

    def pin(self, n):
        return self.lib.msim_pin(n)

    def received(self):
        n = self.lib.msim_serial_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class MRobot:
    STEP_NS = 500000

    def __init__(self, maze, seed=0, battery_v=S.BATTERY_V):
        self.maze = maze
        self.model = mujoco.MjModel.from_xml_string(model.scene_xml(maze))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.rng = random.Random(seed)
        m = self.model
        self.body = m.body('robot').id
        self.free_qpos = m.joint('robot').qposadr[0]
        self.free_dof = m.joint('robot').dofadr[0]
        self.wheel_dof = {s: m.joint(f'wheel_{s}').dofadr[0] for s in 'lr'}
        self.act = {s: m.actuator(f'motor_{s}').id for s in 'lr'}
        self.site = m.site('srf02').id
        bodies = {self.body, m.body('wheel_l').id, m.body('wheel_r').id}
        self._mine = {g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies and m.geom_contype[g]}   # its collisions
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.volts = battery_v / S.BATTERY_V        # its servos' speed and torque scale with their supply
        self.button_until = -1
        self.pulse = {'l': 0, 'r': 0}
        self.next_frame_ms = 0
        self.lifted = None
        self.fw = MFirmware(build())
        self._inputs()

    # ---- the operator ----------------------------------------------------------------------------------------
    def press(self, button='button', ms=100):
        """The Skymega's test button (its only one) held for ms."""
        if button != 'button':
            raise ValueError("Miniskybot has one button: 'button'")
        self.button_until = self.fw.time_ms() + ms
        self._inputs()

    def _inputs(self):
        self.fw.lib.msim_set_input(S.BUTTON_PIN, 0 if self.fw.time_ms() < self.button_until else 1)

    def cover(self, sensors, ms=300):
        raise ValueError('Miniskybot is started with its button')

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += 0.08
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell's centre, facing heading_deg (0 north, positive to the left), still."""
        q = self.data.qpos
        a = self.free_qpos
        x, y = cell_pose(cell, heading_deg, back_mm(self.model, self.data, self.body, 0.0))
        q[a:a + 3] = (x * MM, y * MM, 0.0003)
        h = math.radians(heading_deg) / 2
        q[a + 3:a + 7] = (math.cos(h), 0, 0, math.sin(h))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- its SRF02 ---------------------------------------------------------------------------------------------
    def _face(self, geom, point):
        """The outward normal of a wall's or post's box (the maze's: upright) where a ray met it, and its face's
        width (mm)."""
        m, d = self.model, self.data
        rot = d.geom_xmat[geom].reshape(3, 3)
        rel = rot.T @ (point - d.geom_xpos[geom])
        size = m.geom_size[geom]
        k = int(np.argmax(np.abs(rel[:2]) / size[:2]))
        n = np.zeros(3)
        n[k] = math.copysign(1.0, rel[k])
        return rot @ n, 2 * size[1 - k] / MM

    def echo_mm(self):
        """What its SRF02 hears (mm from its face), or -1: nothing."""
        p = self.data.site_xpos[self.site].copy()
        rot = self.data.site_xmat[self.site].reshape(3, 3)
        best = None
        for a in np.linspace(-S.SRF02_BEAM_DEG, S.SRF02_BEAM_DEG, S.SRF02_RAYS):
            r = math.radians(a)
            v = rot @ np.array([-math.sin(r), math.cos(r), 0.0])      # its axis: the robot's +y
            v[2] = 0.0
            v /= np.linalg.norm(v)
            dist = mujoco.mj_ray(self.model, self.data, p, v, self.ray_group, 1, self.body, self.ray_geom)
            if dist < 0:
                continue
            n, width = self._face(self.ray_geom[0], p + dist * v)
            if width < S.SMALL_FACE_MM and abs(a) > S.SMALL_TARGET_DEG:
                continue
            if math.degrees(math.acos(max(-1.0, min(1.0, -float(n @ v))))) <= S.ECHO_DEG:
                best = dist if best is None else min(best, dist)
        if best is None or best / MM > S.SRF02_MAX_MM:
            return -1.0
        return max(best / MM + self.rng.gauss(0.0, S.SRF02_NOISE_MM), S.SRF02_MIN_MM)

    # ---- its servos ----------------------------------------------------------------------------------------------
    def _torque(self, s):
        us = self.pulse[s]
        w = self.data.qvel[self.wheel_dof[s]]                    # its wheel, forward positive (rad/s)
        if us <= 0:                                             # no pulse: its motor off, its gears' friction
            return -0.05 * S.SERVO_STALL_NM * math.tanh(w / 0.2)
        e = us - S.SERVO_MID_US
        u = 0.0 if abs(e) <= S.SERVO_DEADBAND_US else max(-1.0, min(1.0, e / S.SERVO_BAND_US))
        u *= S.SERVO_FORWARD[s]
        return S.SERVO_STALL_NM * self.volts * (u - w / (S.SERVO_FREE_RAD_S * self.volts))

    # ---- half a millisecond --------------------------------------------------------------------------------------
    def step(self):
        lib = self.fw.lib
        self._inputs()
        mujoco.mj_kinematics(self.model, self.data)
        if lib.msim_ranging():
            lib.msim_set_echo(self.echo_mm())
        self.fw.run_until(self.fw.time_ns() + self.STEP_NS)
        if self.fw.state:
            return self.fw.state
        t = self.fw.time_ms()
        if t >= self.next_frame_ms:                             # each servo's pulse, every 20 ms
            self.next_frame_ms = t + SERVO_FRAME_MS
            self.pulse = {s: lib.msim_servo_us(S.SERVO_PINS[s]) for s in 'lr'}
        for s in 'lr':
            self.data.ctrl[self.act[s]] = self._torque(s)
        if self.lifted is not None:
            self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
            self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        mujoco.mj_step(self.model, self.data)
        return 0

    def led(self):
        """The Skymega's test LED, for videos: green when lit."""
        return (0.1, 1.0, 0.2) if self.fw.pin(S.LED_PIN) else (0.0, 0.0, 0.0)

    def pose(self):
        """(x mm, y mm, heading deg: 0 north, positive to the left)."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7]
        w, x_, y_, z = q[3], q[4], q[5], q[6]
        yaw = math.degrees(math.atan2(2 * (w * z + x_ * y_), 1 - 2 * (y_ * y_ + z * z)))
        return q[0] / MM, q[1] / MM, yaw

    def toppled(self):
        """Over on its side or its back (its z axis more than 60 deg from up)."""
        return self.data.xmat[self.body][8] < 0.5

    def crashed(self):
        m, d = self.model, self.data
        mine = self._mine
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in mine and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
