"""NezumiBot simulated: MuJoCo physics, its N20 motors, encoders and Sharp sensors around its firmware (nezumibot/fw,
through ctypes). Every millisecond: inputs to the firmware, 1 ms of firmware (its SysTick, then its main loop until
it waits), its PWM to motor torques, two physics steps of 0.5 ms.

Its sensors are Ratatron's (the same Sharp GP2Y0A51SK0F where Ratatron has them): Ratatron's rays, hold and noise
(ratasim/robot.py), and each one's counts for a distance from Ratatron's own measured calibration (its firmware's
ir_mm_of, inverted), Ratatron's reference offsets included.
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
from ..firmware import Firmware as RatatronFirmware, build as build_ratatron
from ..hand import Hand
from ..model import MM, WALL_GROUP
from ..robot import IR_FAR_MM, IR_NOISE_MM, IR_PERIOD_MS, IR_SPREAD_DEG
from . import config as N
from . import model

FW_DIR = os.path.join(N.HERE, 'fw')
LIFT_M = 0.08                   # held over the maze: its sensors see over the walls


def build():
    subprocess.run(['make', '-s', '-C', FW_DIR, f'NEZUMIBOT_DIR={N.NEZUMI_DIR}'], check=True)
    return os.path.join(N.BUILD, 'libnezumifw.so')


def sharp_tables():
    """Each sensor's reading (mm, Ratatron's firmware's) for every count 0..4095, and its offset from the beam."""
    fw = RatatronFirmware(build_ratatron())
    counts = np.arange(4096, dtype=np.float32)
    tables = {n: np.array([fw.ir_mm_of(idx, float(c)) for c in counts]) for n, (idx, *_) in RC.IR_SENSORS.items()}
    fw.close()
    half = 0.5 * RC.FW['LANE_WIDTH_MM']
    front, skew = RC.FW['FRONT_WALL_REF_MM'], RC.FW['FRONT_SQUARE_OFFSET_MM']
    want = {'SL': RC.FW['SIDE_CENTER_L_MM'], 'SR': RC.FW['SIDE_CENTER_R_MM'], 'FL': front + skew / 2, 'FR': front - skew / 2}
    offset = {}
    for name, (_, _, (x, y, _), deg) in RC.IR_SENSORS.items():
        beam = half - y if name in ('FL', 'FR') else (half - abs(x)) / abs(math.cos(math.radians(deg)))
        offset[name] = want[name] - beam
    return tables, offset


class NFirmware:
    def __init__(self, lib_path):
        self._tmp = tempfile.NamedTemporaryFile(prefix='nezumifw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.nsim_step.restype = C.c_uint8
        lib.nsim_time_ms.restype = C.c_uint32
        lib.nsim_set_encoders.argtypes = [C.c_int32, C.c_int32]
        lib.nsim_set_sensors.argtypes = [C.c_uint16] * 4
        lib.nsim_get_motors.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16), C.POINTER(C.c_uint8)]
        lib.nsim_uart_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.nsim_uart_tx.restype = C.c_uint32
        self._l, self._r, self._en = C.c_int16(), C.c_int16(), C.c_uint8()
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        if not lib.nsim_boot():
            raise RuntimeError('nsim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def step(self, enc, counts):
        lib = self.lib
        lib.nsim_set_encoders(*enc)
        lib.nsim_set_sensors(*counts)
        self.state = lib.nsim_step()
        lib.nsim_get_motors(C.byref(self._l), C.byref(self._r), C.byref(self._en))
        return self._l.value / N.PWM_MAX, self._r.value / N.PWM_MAX, bool(self._en.value)

    def time_ms(self):
        return self.lib.nsim_time_ms()

    def global_bool(self, name):
        return bool(C.c_bool.in_dll(self.lib, name).value)

    def received(self):
        n = self.lib.nsim_uart_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class NRobot:
    def __init__(self, maze, seed=0, battery_v=N.BATTERY_V):
        self.maze = maze
        self.model = mujoco.MjModel.from_xml_string(model.scene_xml(maze))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.rng = random.Random(seed)
        m = self.model
        self.body = m.body('robot').id
        self.free_qpos = m.joint('robot').qposadr[0]
        self.free_dof = m.joint('robot').dofadr[0]
        self.wheel_qpos = {s: m.joint(f'wheel_{s}').qposadr[0] for s in 'lr'}
        self.wheel_dof = {s: m.joint(f'wheel_{s}').dofadr[0] for s in 'lr'}
        self.act = {s: m.actuator(f'motor_{s}').id for s in 'lr'}
        self.sites = {n: m.site(f'ir_{n}').id for n in N.SENSORS}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.r = N.WHEEL_RADIUS_MM * MM
        self.battery_v = battery_v
        self.tables, self.offset = sharp_tables()
        self.held = {n: IR_FAR_MM for n in N.SENSORS}
        self.phase = {n: self.rng.randrange(IR_PERIOD_MS) for n in N.SENSORS}
        self.hand = Hand(self.model, self.data)
        self.lifted = None          # the pose it is held at
        self.fw = NFirmware(build())
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}

    # ---- the operator ----------------------------------------------------------------------------------------
    def press(self, button, ms=100):
        raise ValueError('NezumiBot has no buttons: it is started with a hand')

    def cover(self, sensors, ms=300):
        """A hand in front of these sensors for ms (ratasim/hand.py)."""
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def sensor_ray(self, name):
        """A sensor's origin and direction in the world."""
        deg = math.radians(N.IR_SENSORS[name][1])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def lift(self):
        """Held in the hand over where it stands: its wheels free, its sensors over the walls."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += LIFT_M
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell's centre, facing heading_deg (0 north, positive to the left), still."""
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = ((cell[0] + 0.5) * RC.CELL_MM * MM, (cell[1] + 0.5) * RC.CELL_MM * MM, -N.GROUND_Z_MM * MM + 0.0002)
        h = math.radians(heading_deg) / 2
        q[a + 3:a + 7] = (math.cos(h), 0, 0, math.sin(h))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else IR_FAR_MM

    def beam_mm(self, name):
        deg = N.IR_SENSORS[name][1]
        sid = self.sites[name]
        p = self.data.site_xpos[sid]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        inv = 0.0
        for spread in (-IR_SPREAD_DEG, 0.0, IR_SPREAD_DEG):
            a = math.radians(deg + spread)
            inv += 1.0 / max(min(self._ray(p, rot @ np.array([math.cos(a), math.sin(a), 0.0])), IR_FAR_MM), 1.0)
        return 3.0 / inv

    def _counts(self, name, reading):
        table = self.tables[name]       # decreasing
        i = int(np.searchsorted(-table, -reading))
        if i <= 0:
            return 0
        if i >= 4096:
            return 4095
        a, b = table[i - 1], table[i]
        return int(round(i - 1 + (a - reading) / (a - b))) if a != b else i

    def sensors(self, t):
        """ADC counts in the DMA's order; each sensor measures every 16 ms and holds its output."""
        out = []
        for name in N.SENSORS:
            if (t + self.phase[name]) % IR_PERIOD_MS == 0:
                beam = self.beam_mm(name)
                self.held[name] = min(beam + self.offset[name], IR_FAR_MM) + self.rng.gauss(0.0, IR_NOISE_MM)
            out.append(self._counts(name, self.held[name]))
        return out

    def encoders(self):
        q = self.data.qpos
        mm = {s: (q[self.wheel_qpos[s]] - self.enc_zero[s]) * self.r / MM for s in 'lr'}
        return tuple(math.floor(mm[s] * 1000.0 / N.MICROMETERS_PER_TICK) for s in 'lr')

    # ---- one millisecond -------------------------------------------------------------------------------------
    def _torque(self, s, duty, enabled):
        w = self.data.qvel[self.wheel_dof[s]] * N.GEAR      # motor shaft rad/s
        if not enabled:
            return -math.copysign(N.MOTOR_FRICTION_NM, w) * N.GEAR if abs(w) > 1 else 0.0
        v = max(-1.0, min(1.0, duty)) * self.battery_v
        t = N.MOTOR_KE * (v - N.MOTOR_KE * w) / N.MOTOR_R_OHM
        if abs(w) > 1:
            t -= math.copysign(N.MOTOR_FRICTION_NM, w)
        return t * N.GEAR * N.GEAR_EFFICIENCY

    def step(self):
        t = self.fw.time_ms()
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        dl, dr, en = self.fw.step(self.encoders(), self.sensors(t))
        self.data.ctrl[self.act['l']] = self._torque('l', dl, en)
        self.data.ctrl[self.act['r']] = self._torque('r', dr, en)
        for _ in range(2):
            if self.lifted is not None:
                self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
                self.data.qvel[self.free_dof:self.free_dof + 6] = 0
            mujoco.mj_step(self.model, self.data)
        return self.fw.state

    def pose(self):
        """(x mm, y mm, heading deg: 0 north, positive to the left)."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7]
        w, x_, y_, z = q[3], q[4], q[5], q[6]
        yaw = math.degrees(math.atan2(2 * (w * z + x_ * y_), 1 - 2 * (y_ * y_ + z * z)))
        return q[0] / MM, q[1] / MM, yaw

    def crashed(self):
        m, d = self.model, self.data
        hull = m.geom('hull').id
        for i in range(d.ncon):
            c = d.contact[i]
            if hull in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == hull else c.geom1
                if m.geom_group[other] == WALL_GROUP:
                    return m.geom(other).name or 'wall'
        return None
