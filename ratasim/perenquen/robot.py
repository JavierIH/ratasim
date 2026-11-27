"""Perenquen simulated: MuJoCo physics, its motors, encoders and IR pairs around its firmware (perenquen/fw, through
ctypes). Every millisecond: inputs to the chip (encoders, the phototransistors' inputs lit and dark, the battery),
1 ms of its firmware (four Timer1 periods: its scheduler, its control system every 1 ms; its command line or wall
follower between them), its PWM to motor torques, two physics steps of 0.5 ms.

Its IR pairs: each phototransistor's input with its emitter lit is IR_K over the square of the distance to the wall
along its beam (three rays across it, their harmonic mean), with noise, over the dark input (config.py: assumed).
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
from ..hand import Hand
from ..model import MM, WALL_GROUP
from ..start import back_mm, cell_pose
from . import config as P
from . import model

FW_DIR = os.path.join(P.HERE, 'fw')
LIFT_M = 0.08


def build():
    subprocess.run(['make', '-s', '-C', FW_DIR, f'PERENQUEN_DIR={P.PERENQUEN_DIR}', f'AVERSIVE_DIR={P.AVERSIVE_DIR}'],
                   check=True)
    return os.path.join(P.BUILD, 'libperenquenfw.so')


class PFirmware:
    """One boot of its firmware: its own copy of the library (its statics start from zero, as on the chip)."""

    def __init__(self, lib_path):
        self._tmp = tempfile.NamedTemporaryFile(prefix='perenquenfw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.psim_step.restype = C.c_uint8
        lib.psim_time_ms.restype = C.c_uint32
        lib.psim_leds.restype = C.c_uint8
        lib.psim_set_ir.argtypes = [C.c_uint8, C.c_int8, C.c_uint16, C.c_uint16]
        lib.psim_set_adc.argtypes = [C.c_uint8, C.c_uint16]
        lib.psim_set_encoders.argtypes = [C.c_int32, C.c_int32]
        lib.psim_get_pwm.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16), C.POINTER(C.c_uint16)]
        lib.psim_uart_rx.argtypes = [C.c_char_p, C.c_uint32]
        lib.psim_uart_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.psim_uart_tx.restype = C.c_uint32
        self._l, self._r, self._period = C.c_int16(), C.c_int16(), C.c_uint16()
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0

    def boot(self):
        if not self.lib.psim_boot():
            raise RuntimeError('psim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def step(self, enc, ir):
        """enc: (left, right) counts; ir: {name: (lit, dark)} counts. One ms; returns the duties (-1..1)."""
        lib = self.lib
        lib.psim_set_encoders(*enc)
        for name, (lit, dark) in ir.items():
            ch, pin, _, _ = P.IR_SENSORS[name]
            lib.psim_set_ir(ch, pin, lit, dark)
        self.state = lib.psim_step()
        lib.psim_get_pwm(C.byref(self._l), C.byref(self._r), C.byref(self._period))
        period = self._period.value or P.PWM_PERIOD
        return self._l.value / period, self._r.value / period

    def time_ms(self):
        return self.lib.psim_time_ms()

    def leds(self):
        return self.lib.psim_leds()

    def send(self, text):
        data = text.encode()
        self.lib.psim_uart_rx(data, len(data))

    def received(self):
        n = self.lib.psim_uart_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class PRobot:
    def __init__(self, maze, seed=0, battery_v=P.BATTERY_V):
        self.maze = maze
        self.model = mujoco.MjModel.from_xml_string(model.scene_xml(maze))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.rng = random.Random(seed)
        m = self.model
        self.body = m.body('robot').id
        self.free_qpos = m.joint('robot').qposadr[0]
        self.free_dof = m.joint('robot').dofadr[0]
        self.wheel_qpos = {s: m.joint(f'wheel_{s}2').qposadr[0] for s in 'lr'}   # the encoders: rear axles
        self.wheel_dof = {s: m.joint(f'wheel_{s}').dofadr[0] for s in 'lr'}
        self.act = {s: m.actuator(f'motor_{s}').id for s in 'lr'}
        self.sites = {n: m.site(f'ir_{n}').id for n in P.IR_SENSORS}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.battery_v = battery_v
        self.hand = Hand(self.model, self.data)
        self.lifted = None
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}
        self.fw = PFirmware(build())
        self.fw.lib.psim_set_adc(P.ADC_BATTERY, self._counts(battery_v / 3))      # its 20k/10k divider
        self.fw.lib.psim_set_adc(P.ADC_GYRO_REF, self._counts(1.5))               # LY3200ALH, still
        self.fw.lib.psim_set_adc(P.ADC_GYRO, self._counts(1.5))
        self.fw.lib.psim_set_adc(P.ADC_FLASH, P.IR_AMBIENT)
        self.fw.boot()

    @staticmethod
    def _counts(volts):
        return int(round(min(volts, 3.3) / 3.3 * 1023))

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button, ms=100):
        raise ValueError('Perenquen is started from its command line and a hand')

    def send(self, line):
        """A line typed in the terminal connected to it over Bluetooth."""
        self.fw.send(line + '\r')

    def cover(self, sensors, ms=300):
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def sensor_ray(self, name):
        deg = math.radians(P.IR_SENSORS[name][3])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += LIFT_M
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down with its turning centre (its front axle) on a cell's centre, facing heading_deg (0 north)."""
        q = self.data.qpos
        a = self.free_qpos
        x, y = cell_pose(cell, heading_deg, back_mm(self.model, self.data, self.body, 0.0))
        q[a:a + 3] = (x * MM, y * MM, (P.WHEEL_RADIUS_MM + 0.2) * MM)
        h = math.radians(heading_deg) / 2
        q[a + 3:a + 7] = (math.cos(h), 0, 0, math.sin(h))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else P.IR_FAR_MM

    def beam_mm(self, name):
        deg = P.IR_SENSORS[name][3]
        sid = self.sites[name]
        p = self.data.site_xpos[sid]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        inv = 0.0
        for spread in (-P.IR_SPREAD_DEG, 0.0, P.IR_SPREAD_DEG):
            a = math.radians(deg + spread)
            inv += 1.0 / max(min(self._ray(p, rot @ np.array([math.cos(a), math.sin(a), 0.0])), P.IR_FAR_MM), 1.0)
        return 3.0 / inv

    def ir(self):
        out = {}
        for name in P.IR_SENSORS:
            d = self.beam_mm(name)
            lit = P.IR_AMBIENT + (P.IR_K / (d * d) if d < P.IR_FAR_MM else 0.0) + self.rng.gauss(0.0, P.IR_NOISE)
            dark = P.IR_AMBIENT + self.rng.gauss(0.0, P.IR_NOISE)
            out[name] = (max(0, min(1023, int(round(lit)))), max(0, min(1023, int(round(dark)))))
        return out

    def encoders(self):
        q = self.data.qpos
        return tuple(int(math.floor((q[self.wheel_qpos[s]] - self.enc_zero[s]) / (2 * math.pi) * P.ENCODER_COUNTS_PER_REV))
                     for s in 'lr')

    # ---- one millisecond ---------------------------------------------------------------------------------------
    def _torque(self, s, duty):
        w = self.data.qvel[self.wheel_dof[s]] * P.GEAR          # motor shaft rad/s
        v = max(-1.0, min(1.0, duty)) * self.battery_v
        t = P.MOTOR_KE * (v - P.MOTOR_KE * w) / P.MOTOR_R_OHM
        if abs(w) > 1:
            t -= math.copysign(P.MOTOR_FRICTION_NM, w)
        return t * P.GEAR * P.GEAR_EFFICIENCY

    def step(self):
        t = self.fw.time_ms()
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        dl, dr = self.fw.step(self.encoders(), self.ir())
        self.data.ctrl[self.act['l']] = self._torque('l', dl)
        self.data.ctrl[self.act['r']] = self._torque('r', dr)
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
        hulls = {m.geom('hull').id, m.geom('hull_front').id}
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in hulls and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
