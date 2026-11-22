"""KERISE v2 simulated: MuJoCo physics, its motors, encoders, MPU6500, IR sensors, battery and button around its
firmware on its simulated chip (kerise/fw, through ctypes). Every half millisecond: the wheels' angles and speeds
to its encoders, its gyro and accelerometer, the light its IR receivers get back, half a millisecond of firmware
(its threads and tickers on its own clock), its motors' drive to torques, one physics step.

Its motors (config.py): a DC motor through its 4:1 gears, torque = KT * (V - KT * w) / R at the wheel, V its drive
times the battery's voltage; none when its firmware lets them coast (both inputs low).
Its IR sensors: each emitter's beam to the first wall (three rays across its spot), the light back K / d^2 in ADC
counts (config.py), its noise in; the firmware's ADC sees it only while that emitter's PWM is on.
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

from ..hand import Hand
from ..model import MM, WALL_GROUP
from . import config as K

# Its emitters' beams: five rays, their light weighted as a Gaussian of their angle off the axis (sigma
# IR_SPREAD_DEG), so that a post caught by a beam's edge sends little back
IR_RAYS = [(k * K.IR_SPREAD_DEG, math.exp(-k * k / 2)) for k in (-2, -1, 0, 1, 2)]
IR_RAYS = [(a, w / sum(w for _, w in IR_RAYS)) for a, w in IR_RAYS]
from . import model

FW_DIR = os.path.join(K.HERE, 'fw')
LIB = os.path.join(K.BUILD, 'libkerisefw.so')
STEP_NS = 500000
G = 9.80665


def build(goal):
    """Its firmware with the maze's goal (fw/Makefile: GOAL)."""
    cells = ' '.join(f'{x},{y}' for x, y in sorted(goal))
    subprocess.run(['make', '-s', '-j8', '-C', FW_DIR, f'KERISE_DIR={K.KERISE_DIR}', f'GOAL={cells}'], check=True)
    return LIB


class KFirmware:
    """One boot of its firmware: its own copy of the library (its globals start as on the chip)."""

    def __init__(self, lib_path, battery_v):
        self._tmp = tempfile.NamedTemporaryFile(prefix='kerisefw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.ksim_run_until.argtypes = [C.c_uint64]
        lib.ksim_run_until.restype = C.c_uint8
        lib.ksim_time_ns.restype = C.c_uint64
        lib.ksim_set_encoder.argtypes = [C.c_int, C.c_double, C.c_double]
        lib.ksim_set_ir.argtypes = [C.c_double] * 4
        lib.ksim_set_imu.argtypes = [C.c_double] * 6
        lib.ksim_set_battery.argtypes = [C.c_double]
        lib.ksim_button.argtypes = [C.c_int]
        lib.ksim_motor.argtypes = [C.c_int]
        lib.ksim_motor.restype = C.c_double
        lib.ksim_console.argtypes = [C.c_char_p, C.c_uint32]
        lib.ksim_console.restype = C.c_uint32
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        lib.ksim_set_battery(battery_v)
        if not lib.ksim_boot():
            raise RuntimeError('ksim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def run_until(self, t_ns):
        self.state = self.lib.ksim_run_until(int(t_ns))
        return self.state

    def time_ns(self):
        return self.lib.ksim_time_ns()

    def time_ms(self):
        return self.lib.ksim_time_ns() // 1000000

    def leds(self):
        return self.lib.ksim_leds()

    def received(self):
        n = self.lib.ksim_console(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class KRobot:
    def __init__(self, maze, seed=0, battery_v=K.BATTERY_V):
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
        self.sites = {n: m.site(f'ir_{n}').id for n in K.IR_SENSORS}
        self.hulls = {i for i in range(m.ngeom) if m.geom(i).name.startswith('hull')}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.battery_v = battery_v
        self.hand = Hand(self.model, self.data)
        self.buttons = []           # (from ms, to ms)
        self.held = False
        self.lifted = None
        self.v_prev = np.zeros(3)
        self.accel = np.array([0.0, 0.0, 1.0])
        self.fw = KFirmware(build(maze.goal()), battery_v)
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button=0, ms=100):
        t = self.fw.time_ms()
        start = max([t] + [b[1] + 50 for b in self.buttons])
        self.buttons.append((start, start + ms))

    def send(self, line):
        raise ValueError('KERISE v2 is started with its button (its serial port reads letters, not lines)')

    def cover(self, sensors, ms=300):
        """A hand in front of these sensors for ms (ratasim/hand.py)."""
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += 0.05
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell, facing heading_deg (0 north, positive to the left), its tail against the back wall."""
        h = math.radians(heading_deg)
        back = K.START_BACK_MM
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = (((cell[0] + 0.5) * K.CELL_MM + back * math.sin(h)) * MM,
                      ((cell[1] + 0.5) * K.CELL_MM - back * math.cos(h)) * MM, 0.2 * MM)
        q[a + 3:a + 7] = (math.cos(h / 2), 0, 0, math.sin(h / 2))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)
        self.v_prev = np.zeros(3)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def sensor_ray(self, name):
        """A sensor's origin and direction in the world."""
        deg = math.radians(K.IR_SENSORS[name][1])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else 1000.0

    def _incidence(self, origin, direction, d_mm):
        """The cosine between a beam and the face of the wall or post it hit (boxes: the face its hit point is on)."""
        g = self.ray_geom[0]
        if g < 0 or d_mm >= 1000.0:
            return 1.0
        hit = origin + direction * d_mm * MM
        rel = self.data.geom_xmat[g].reshape(3, 3).T @ (hit - self.data.geom_xpos[g])
        size = self.model.geom_size[g]
        axis = int(np.argmax(np.abs(rel[:2]) / size[:2]))
        normal = self.data.geom_xmat[g].reshape(3, 3)[:, axis]
        return abs(float(normal @ direction))

    def ir_mm(self):
        """Each beam's length to the wall (mm, its middle ray)."""
        return {n: self._ray(*self.sensor_ray(n)) for n in K.IR_SENSORS}

    def ir_light(self):
        """The light back at each receiver (ADC counts), noise in."""
        out = {}
        R = self.data.site_xmat
        for name, (_, deg) in K.IR_SENSORS.items():
            sid = self.sites[name]
            p = self.data.site_xpos[sid]
            rot = R[sid].reshape(3, 3)
            light = 0.0
            for spread, weight in IR_RAYS:
                a = math.radians(deg + spread)
                v = rot @ np.array([math.cos(a), math.sin(a), 0.0])
                d = self._ray(p, v)
                cos = self._incidence(p, v, d)              # a wall seen aslant sends less back (a matte face)
                light += weight * K.IR_K * cos / max(d, 5.0) ** K.IR_EXP
            out[name] = max(0.0, light * (1 + self.rng.gauss(0.0, K.IR_NOISE)))
        return out

    def _imu(self):
        """Its gyro (deg/s) and accelerometer (g) in its frame (x right, y forward, z up), noise in."""
        dof = self.free_dof
        rot = self.data.xmat[self.body].reshape(3, 3)
        w = np.degrees(self.data.qvel[dof + 3:dof + 6])             # the free joint's angular velocity: body frame
        v = self.data.qvel[dof:dof + 3].copy()                      # world frame
        a_world = (v - self.v_prev) / (STEP_NS * 1e-9)
        self.v_prev = v
        a = rot.T @ (a_world / G + np.array([0.0, 0.0, 1.0]))
        self.accel += (a - self.accel) * 0.5                        # its low-pass (about 260 Hz)
        n = self.rng.gauss
        return (w[0] + n(0, K.GYRO_NOISE_DPS), w[1] + n(0, K.GYRO_NOISE_DPS),
                w[2] + K.GYRO_BIAS_DPS + n(0, K.GYRO_NOISE_DPS),
                *(float(c) + n(0, K.ACCEL_NOISE_G) for c in self.accel))

    # ---- its motors ----------------------------------------------------------------------------------------------
    def _torque(self, s, drive):
        if math.isnan(drive):
            return 0.0
        v = max(-1.0, min(1.0, drive)) * self.battery_v
        w = self.data.qvel[self.wheel_dof[s]]
        return K.MOTOR_KT * (v - K.MOTOR_KT * w) / K.MOTOR_R_OHM

    # ---- half a millisecond --------------------------------------------------------------------------------------
    def _inputs(self):
        t = self.fw.time_ms()
        lib = self.fw.lib
        pressed = any(t0 <= t < t1 for t0, t1 in self.buttons)
        self.buttons = [b for b in self.buttons if t < b[1]]
        if pressed != self.held:
            self.held = pressed
            lib.ksim_button(int(pressed))
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        k = K.counts_per_rad()
        for i, s in enumerate('lr'):
            q = self.data.qpos[self.wheel_qpos[s]] - self.enc_zero[s]
            lib.ksim_set_encoder(i, q * k, self.data.qvel[self.wheel_dof[s]] * k * 1e-9)
        ir = self.ir_light()
        lib.ksim_set_ir(ir['SL'], ir['FL'], ir['FR'], ir['SR'])
        lib.ksim_set_imu(*self._imu())

    def step(self):
        """One millisecond: two half-millisecond steps."""
        for _ in range(2):
            self._inputs()
            t = self.fw.time_ns()
            if self.fw.run_until(t - t % STEP_NS + STEP_NS):
                return self.fw.state
            for i, s in enumerate('lr'):
                self.data.ctrl[self.act[s]] = self._torque(s, self.fw.lib.ksim_motor(i))
            if self.lifted is not None:
                self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
                self.data.qvel[self.free_dof:self.free_dof + 6] = 0
            mujoco.mj_step(self.model, self.data)
        return self.fw.state

    def led(self):
        """Its LEDs, for videos: green while any is on."""
        return (0.2, 1.0, 0.3) if self.fw.leds() else (0.0, 0.0, 0.0)

    def pose(self):
        """(x mm, y mm, heading deg: 0 north, positive to the left)."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7]
        w, x_, y_, z = q[3], q[4], q[5], q[6]
        yaw = math.degrees(math.atan2(2 * (w * z + x_ * y_), 1 - 2 * (y_ * y_ + z * z)))
        return q[0] / MM, q[1] / MM, yaw

    def crashed(self):
        m, d = self.model, self.data
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in self.hulls and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
