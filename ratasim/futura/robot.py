"""Futura simulated: MuJoCo physics, its motors, encoders, IR pairs and gyro around its firmware (its kit's template
and its page's controller sample, futura/fw, through ctypes). Every millisecond: inputs to the chip (what the encoders
counted, the phototransistors' inputs lit and dark, the gyro, the battery), 1 ms of its firmware (main() until the
millisecond, then SysTick's interrupt), its PWM to motor torques, two physics steps of 0.5 ms.

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
from . import config as F
from . import model

FW_DIR = os.path.join(F.HERE, 'fw')
LIB = os.path.join(F.BUILD, 'libfuturafw.so')
PORTS = 'ABCD'


def build():
    """Its firmware as a library: its files downloaded first (once)."""
    F.download()
    subprocess.run(['make', '-s', '-C', FW_DIR, f'FUTURA_DIR={F.PROJECT}'], check=True)
    return LIB


class FFirmware:
    """One boot of its firmware: its own copy of the library (its statics start from zero, as on the chip)."""

    def __init__(self, lib_path):
        self._tmp = tempfile.NamedTemporaryFile(prefix='futurafw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.fsim_step.restype = C.c_uint8
        lib.fsim_time_ms.restype = C.c_uint32
        lib.fsim_leds.restype = C.c_uint16
        lib.fsim_buzzer.restype = C.c_uint8
        lib.fsim_set_ir.argtypes = [C.c_uint8, C.c_int8, C.c_uint8, C.c_uint16, C.c_uint16]
        lib.fsim_set_adc.argtypes = [C.c_uint8, C.c_uint16]
        lib.fsim_add_encoders.argtypes = [C.c_int32, C.c_int32]
        lib.fsim_get_pwm.argtypes = [C.POINTER(C.c_int32), C.POINTER(C.c_int32), C.POINTER(C.c_uint32)]
        lib.fsim_uart_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.fsim_uart_tx.restype = C.c_uint32
        self._l, self._r, self._period = C.c_int32(), C.c_int32(), C.c_uint32()
        self._buf = C.create_string_buffer(1 << 16)
        self._dots = (C.c_uint8 * 20)()
        font = (C.c_uint8 * (128 * 5)).in_dll(lib, 'fontTable')       # its display's font (matrixDisplay.c)
        self._chars = {}
        for ch in range(127, 31, -1):
            self._chars.setdefault(bytes(font[ch * 5:ch * 5 + 5]), chr(ch))
        self.state = 0
        if not lib.fsim_boot():
            raise RuntimeError('fsim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def step(self, enc, ir):
        """enc: what the wheels counted this ms (left, right); ir: {name: (lit, dark)} counts. One ms; returns the
        duties (-1..1)."""
        lib = self.lib
        lib.fsim_add_encoders(*enc)
        for name, (lit, dark) in ir.items():
            ch, (port, pin) = F.IR_CHANNELS[name]
            lib.fsim_set_ir(ch, PORTS.index(port), pin, lit, dark)
        self.state = lib.fsim_step()
        lib.fsim_get_pwm(C.byref(self._l), C.byref(self._r), C.byref(self._period))
        period = self._period.value or F.PWM_PERIOD
        return self._l.value / period, self._r.value / period

    def time_ms(self):
        return self.lib.fsim_time_ms()

    def leds(self):
        return self.lib.fsim_leds()

    def buzzing(self):
        return bool(self.lib.fsim_buzzer())

    def display(self):
        """The four characters on its HCMS-2903, read back through its font ('?' a column pattern not in it)."""
        self.lib.fsim_display(self._dots)
        dots = bytes(self._dots)
        return ''.join(self._chars.get(dots[k:k + 5], '?') for k in range(0, 20, 5))

    def received(self):
        n = self.lib.fsim_uart_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class FRobot:
    def __init__(self, maze, seed=0, battery_v=F.BATTERY_V):
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
        self.sites = {n: m.site(f'ir_{n}').id for n in F.IR_SENSORS}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.battery_v = battery_v
        self.hand = Hand(self.model, self.data)
        self.lifted = None
        self.fw = FFirmware(build())
        self.fw.lib.fsim_set_adc(F.ADC_BATTERY, int(round(battery_v * F.BATTERY_COUNTS_PER_V)))
        self.fw.lib.fsim_set_adc(F.ADC_VREF, self._counts(1.21))
        self.enc_counted = {s: 0 for s in 'lr'}
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}

    @staticmethod
    def _counts(volts):
        return int(round(min(max(volts, 0.0), 3.3) / 3.3 * F.ADC_FULL))

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button, ms=100):
        raise ValueError("Futura's firmware does nothing with its buttons (their handlers are empty)")

    def send(self, line):
        raise ValueError("Futura's firmware reads nothing from its serial port")

    def cover(self, sensors, ms=300):
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def sensor_ray(self, name):
        deg = math.radians(F.IR_SENSORS[name][1])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += 0.08
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down with the middle of its axle on a cell's centre, facing heading_deg (0 north, positive to the left)."""
        q = self.data.qpos
        a = self.free_qpos
        x, y = cell_pose(cell, heading_deg, back_mm(self.model, self.data, self.body, 0.0))
        q[a:a + 3] = (x * MM, y * MM, -F.GROUND_Z_MM * MM + 0.0002)
        h = math.radians(heading_deg) / 2
        q[a + 3:a + 7] = (math.cos(h), 0, 0, math.sin(h))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else F.IR_FAR_MM

    def beam_mm(self, name):
        deg = F.IR_SENSORS[name][1]
        sid = self.sites[name]
        p = self.data.site_xpos[sid]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        inv = 0.0
        for spread in (-F.IR_SPREAD_DEG, 0.0, F.IR_SPREAD_DEG):
            a = math.radians(deg + spread)
            inv += 1.0 / max(min(self._ray(p, rot @ np.array([math.cos(a), math.sin(a), 0.0])), F.IR_FAR_MM), 1.0)
        return 3.0 / inv

    def ir(self):
        out = {}
        for name in F.IR_SENSORS:
            d = self.beam_mm(name)
            lit = F.IR_AMBIENT + (F.IR_K / (d * d) if d < F.IR_FAR_MM else 0.0) + self.rng.gauss(0.0, F.IR_NOISE)
            dark = F.IR_AMBIENT + self.rng.gauss(0.0, F.IR_NOISE)
            out[name] = (max(0, min(F.ADC_FULL, int(round(lit)))), max(0, min(F.ADC_FULL, int(round(dark)))))
        return out

    def encoders(self):
        """What each encoder counted since the last call (forward positive)."""
        q = self.data.qpos
        out = []
        for s in 'lr':
            total = math.floor((q[self.wheel_qpos[s]] - self.enc_zero[s]) / (2 * math.pi) * F.ENCODER_COUNTS_PER_REV)
            out.append(total - self.enc_counted[s])
            self.enc_counted[s] = total
        return tuple(out)

    def gyro_counts(self):
        """Its LY3200ALH's output (its yaw rate, counter-clockwise positive), with noise."""
        dps = math.degrees(self.data.qvel[self.free_dof + 5])
        return self._counts(F.GYRO_ZERO_COUNTS / F.ADC_FULL * 3.3 + F.GYRO_V_PER_DPS * dps + self.rng.gauss(0.0, 0.001))

    # ---- one millisecond ---------------------------------------------------------------------------------------
    def _torque(self, s, duty):
        w = self.data.qvel[self.wheel_dof[s]] * F.GEAR          # motor shaft rad/s
        v = max(-1.0, min(1.0, duty)) * self.battery_v
        t = F.MOTOR_KE * (v - F.MOTOR_KE * w) / F.MOTOR_R_OHM
        if abs(w) > 1:
            t -= math.copysign(F.MOTOR_FRICTION_NM, w)
        return t * F.GEAR * F.GEAR_EFFICIENCY

    def step(self):
        t = self.fw.time_ms()
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        self.fw.lib.fsim_set_adc(F.ADC_GYRO, self.gyro_counts())
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
        hulls = {m.geom('hull').id}
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in hulls and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
