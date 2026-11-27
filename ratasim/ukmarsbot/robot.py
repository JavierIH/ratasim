"""UKMARSBOT simulated: MuJoCo physics, its motors, encoders, IR pairs, button and function switches around
mazerunner-core on a simulated Arduino Nano (ukmarsbot/fw, through ctypes). Every half millisecond: its encoders
(their counts and how fast they change: their edges fall between) and its analog inputs (each phototransistor's lit
and dark, the switches, the battery) to the chip, half a millisecond of its firmware, its motors' volts to wheel
torques, one physics step.

Its IR pairs: each phototransistor's reading with its emitter lit is the light back over the dark one, with noise:
three rays across its emitter's beam, each wall they meet a diffuse reflector, cos(incidence) IR_K / (d +
IR_OFFSET_MM)^2 (config.py: its firmware's own calibration). Its motors (config.py): volts = duty x battery, a DC
motor's torque at the wheel against its back EMF, friction.
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
from ..start import back_mm
from .. import tyre
from . import config as U
from . import model

FW_DIR = os.path.join(U.HERE, 'fw')
LIB = os.path.join(U.BUILD, 'libukmarsbotfw.so')
LIFT_M = 0.08


def build():
    """mazerunner-core as a library (as built for the contest, config.py)."""
    subprocess.run(['make', '-s', '-C', FW_DIR, f'MAZERUNNER_DIR={U.MAZERUNNER_DIR}'], check=True)
    return LIB


class UFirmware:
    """One boot of its firmware: its own copy of the library (its globals start as on the chip)."""

    def __init__(self, lib_path):
        self._tmp = tempfile.NamedTemporaryFile(prefix='ukmarsbotfw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.usim_run_until.argtypes = [C.c_uint64]
        lib.usim_run_until.restype = C.c_uint8
        lib.usim_time_ns.restype = C.c_uint64
        lib.usim_set_encoder.argtypes = [C.c_int, C.c_double, C.c_double]
        lib.usim_set_analog.argtypes = [C.c_int] * 4
        lib.usim_pin.argtypes = [C.c_int]
        lib.usim_pin.restype = C.c_double
        lib.usim_serial_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.usim_serial_tx.restype = C.c_uint32
        lib.usim_serial_rx.argtypes = [C.c_char_p, C.c_uint32]
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        if not lib.usim_boot():
            raise RuntimeError('usim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def run_until(self, t_ns):
        self.state = self.lib.usim_run_until(int(t_ns))
        return self.state

    def time_ns(self):
        return self.lib.usim_time_ns()

    def time_ms(self):
        return self.lib.usim_time_ns() // 1000000

    def pin(self, n):
        return self.lib.usim_pin(n)

    def send(self, text):
        data = text.encode()
        self.lib.usim_serial_rx(data, len(data))

    def received(self):
        n = self.lib.usim_serial_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class URobot:
    STEP_NS = 500000

    def __init__(self, maze, seed=0, battery_v=U.BATTERY_V):
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
        self.sites = {n: m.site(f'ir_{n}').id for n in U.IR_SENSORS}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.battery_v = battery_v
        self.hand = Hand(self.model, self.data)
        self.lifted = None
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}
        self.switches, self.button_until = 0, -1
        self.fw = UFirmware(build())
        self._inputs()

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button='button', ms=100):
        """Its button (its only one) held for ms."""
        if button != 'button':
            raise ValueError("UKMARSBOT has one button: 'button'")
        self.button_until = self.fw.time_ms() + ms
        self._inputs()

    def set_switches(self, value):
        """Its four function switches set to value (0..15)."""
        self.switches = int(value) & 15
        self._inputs()

    def send(self, line):
        """A line typed in a terminal on its serial port (USB, or Bluetooth on its HC-05 connector)."""
        self.fw.send(line + '\n')

    def cover(self, sensors, ms=300):
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def sensor_ray(self, name):
        deg = math.radians(U.IR_SENSORS[name][3])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += LIFT_M
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell facing heading_deg (0 north, positive to the left), backed up to the wall behind it, as its
        firmware starts (BACK_WALL_TO_CENTER: its axle 48 mm behind the cell's centre)."""
        q = self.data.qpos
        a = self.free_qpos
        h = math.radians(heading_deg)
        # its axle from the cell's centre, its tail 1 mm off the wall, or --start's
        back = back_mm(self.model, self.data, self.body, RC.CELL_MM / 2 - RC.WALL_MM / 2 + U.BACK_MM - 1.0)
        cx, cy = (cell[0] + 0.5) * RC.CELL_MM, (cell[1] + 0.5) * RC.CELL_MM
        q[a:a + 3] = ((cx + back * math.sin(h)) * MM, (cy - back * math.cos(h)) * MM, 0.0005)
        q[a + 3:a + 7] = (math.cos(h / 2), 0, 0, math.sin(h / 2))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def _ray(self, origin, direction):
        """How far a wall is along a ray (mm; IR_FAR_MM: none), and the cosine of the angle it meets it at."""
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        if d < 0 or d / MM >= U.IR_FAR_MM:
            return U.IR_FAR_MM, 0.0
        g = self.ray_geom[0]
        hit = origin + d * direction
        rot = self.data.geom_xmat[g].reshape(3, 3)
        local = rot.T @ (hit - self.data.geom_xpos[g])
        size = self.model.geom_size[g]
        if self.model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:      # its face: the one the hit is on
            axis = int(np.argmax(np.abs(local) / np.maximum(size, 1e-9)))
            normal = rot[:, axis]
        else:                                                      # rounded (the hand's fingers): from its axis
            normal = rot @ np.array([local[0], local[1], 0.0])
            normal /= max(np.linalg.norm(normal), 1e-9)
        return d / MM, abs(float(normal @ direction))

    def beam_mm(self, name):
        """How far the wall is along its beam's middle (mm)."""
        p, v = self.sensor_ray(name)
        return self._ray(p, v)[0]

    def light(self, name):
        """The light back to a phototransistor from its emitter (ADC counts)."""
        deg = U.IR_SENSORS[name][3]
        sid = self.sites[name]
        p = self.data.site_xpos[sid]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        light = 0.0
        for w, spread in zip(U.IR_WEIGHTS, (-U.IR_SPREAD_DEG, 0.0, U.IR_SPREAD_DEG)):
            a = math.radians(deg + spread)
            d, cos = self._ray(p, rot @ np.array([math.cos(a), math.sin(a), 0.0]))
            if d < U.IR_FAR_MM:
                light += w * cos * U.IR_K[name] / max(d + U.IR_OFFSET_MM, 1.0) ** 2
        return light

    def ir(self):
        """Each phototransistor's ADC counts: (lit, dark)."""
        out = {}
        for name in U.IR_SENSORS:
            dark = U.IR_DARK + self.rng.gauss(0.0, U.IR_NOISE)
            lit = dark + self.light(name) + self.rng.gauss(0.0, U.IR_NOISE)
            out[name] = tuple(max(0, min(1023, int(round(v)))) for v in (lit, dark))
        return out

    def _inputs(self):
        """The analog inputs but the IR pairs': its switches (or its button) and its battery."""
        lib = self.fw.lib
        pressed = self.fw.time_ms() < self.button_until
        sw = U.BUTTON_ADC if pressed else U.SWITCH_ADC[self.switches]
        lib.usim_set_analog(U.SWITCHES_CHANNEL, -1, sw, sw)
        v = int(round(min(self.battery_v * U.BATTERY_ADC_PER_V, 1023)))
        lib.usim_set_analog(U.BATTERY_CHANNEL, -1, v, v)

    # ---- its motors --------------------------------------------------------------------------------------------
    def volts(self, s):
        duty = self.fw.pin(U.MOTOR_PWM_PIN[s])
        forward = int(self.fw.pin(U.MOTOR_DIR_PIN[s]) > 0.5) == U.MOTOR_FORWARD_DIR[s]
        return (1 if forward else -1) * duty * self.battery_v

    def _torque(self, s):
        w = self.data.qvel[self.wheel_dof[s]]                     # rad/s, forward positive
        t = U.KE_WHEEL * (self.volts(s) - U.KE_WHEEL * w) / U.MOTOR_R_OHM
        friction = U.KE_WHEEL * U.BIAS_FF / U.MOTOR_R_OHM
        if abs(w) > 0.5:
            t -= math.copysign(friction, w)
        elif abs(t) < friction:
            t = 0.0
        else:
            t -= math.copysign(friction, t)
        return t

    # ---- one millisecond ---------------------------------------------------------------------------------------
    def step(self):
        lib = self.fw.lib
        for _ in range(2):
            t = self.fw.time_ms()
            if self.hand.sensors and t < self.hand.until:
                self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
            elif self.hand.sensors:
                self.hand.sensors = ()
                self.hand.away()
            mujoco.mj_kinematics(self.model, self.data)
            for i, s in enumerate('lr'):
                k = U.ENCODER_SEQUENCE[s] * U.COUNTS_PER_MM[s] * U.WHEEL_RADIUS_MM[s]     # its pins' steps a radian
                q = self.data.qpos[self.wheel_qpos[s]] - self.enc_zero[s]
                lib.usim_set_encoder(i, q * k, self.data.qvel[self.wheel_dof[s]] * k * 1e-9)
            for name, (lit, dark) in self.ir().items():
                ch, pin, _, _ = U.IR_SENSORS[name]
                lib.usim_set_analog(ch, pin, lit, dark)
            self._inputs()
            now = self.fw.time_ns()
            if self.fw.run_until(now - now % self.STEP_NS + self.STEP_NS):
                return self.fw.state
            for s in 'lr':
                self.data.ctrl[self.act[s]] = self._torque(s)
            if self.lifted is not None:
                self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
                self.data.qvel[self.free_dof:self.free_dof + 6] = 0
            mujoco.mj_step(self.model, self.data)
            if self.lifted is None:
                tyre.slide(self.model, self.data, self.body, self.free_qpos, self.free_dof, U.MASS_KG,
                           U.MASS_KG * tyre.G, U.TYRE_CORNERING, self.STEP_NS * 1e-9)
        return self.fw.state

    def led(self):
        """Its LED (the sensor board's), for videos: red when lit."""
        return (1.0, 0.08, 0.05) if self.fw.pin(U.LED_PIN) > 0.5 else (0.0, 0.0, 0.0)

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
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a == hull and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
