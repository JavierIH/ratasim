"""Bulebule simulated: MuJoCo physics, its motors, encoders, gyro and IR pairs around its firmware (bulebule/fw,
through ctypes). Every millisecond: inputs to the chip (encoders, each phototransistor's input dark and what its
emitter adds, the gyro's rate, the button), 1 ms of firmware (16 of its emitters' timer interrupts, SysTick, then its
main loop until it waits), its motor driver's inputs to torques, two physics steps of 0.5 ms.

Its IR pairs: the light its emitter adds is its own calibration inverted (config.py) for the distance its firmware
would give the beam's hits (five rays across the beam), with noise; its emitters lit by its own pins, so a reading
taken with the emitter off is dark.
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
from . import config as B
from . import model

FW_DIR = os.path.join(B.HERE, 'fw')
FLASH_FILE = os.path.join(B.BUILD, 'flash.bin')         # its maze's flash page between runs
RAYS = np.arange(-2, 3)                                 # its beam's rays, in sigmas of its profile
RAY_WEIGHTS = np.exp(-0.5 * RAYS ** 2) / np.exp(-0.5 * RAYS ** 2).sum()
HAND_MM = 10.0                  # its front sensors 66 mm apart look 10 deg out: a palm close enough for both


def build():
    subprocess.run(['make', '-s', '-C', FW_DIR, f'BULEBULE_DIR={B.BULEBULE_DIR}'], check=True)
    return os.path.join(B.BUILD, 'libbulebulefw.so')


class BFirmware:
    """One boot of its firmware: its own copy of the library (its statics start from zero, as on the chip)."""

    def __init__(self, lib_path, flash_file=FLASH_FILE):
        self._tmp = tempfile.NamedTemporaryFile(prefix='bulebulefw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.bsim_step.restype = C.c_uint8
        lib.bsim_time_ms.restype = C.c_uint32
        lib.bsim_leds.restype = C.c_uint8
        lib.bsim_speaker.restype = C.c_uint8
        lib.bsim_set_encoders.argtypes = [C.c_int32, C.c_int32]
        lib.bsim_set_ir.argtypes = [C.c_uint8, C.c_uint16, C.c_uint16]
        lib.bsim_set_gyro.argtypes = [C.c_int16]
        lib.bsim_set_battery.argtypes = [C.c_uint16]
        lib.bsim_set_button.argtypes = [C.c_uint8]
        lib.bsim_get_motors.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16), C.POINTER(C.c_uint8), C.POINTER(C.c_uint8)]
        lib.bsim_flash.restype = C.POINTER(C.c_uint8)
        lib.bsim_flash_bytes.restype = C.c_uint32
        lib.bsim_flash_erases.restype = C.c_uint32
        lib.bsim_uart_rx.argtypes = [C.c_char_p, C.c_uint32]
        lib.bsim_uart_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.bsim_uart_tx.restype = C.c_uint32
        self._l, self._r, self._on_l, self._on_r = C.c_int16(), C.c_int16(), C.c_uint8(), C.c_uint8()
        self._buf = C.create_string_buffer(1 << 16)
        self.flash_file = flash_file
        self.state = 0

    def boot(self):
        if not self.lib.bsim_boot():
            raise RuntimeError('bsim_boot failed (its flash page at 0x0800FC00 could not be mapped)')

    def flash(self):
        n = self.lib.bsim_flash_bytes()
        return C.cast(self.lib.bsim_flash(), C.POINTER(C.c_uint8 * n)).contents

    def load_flash(self):
        page = self.flash()
        data = open(self.flash_file, 'rb').read() if self.flash_file and os.path.exists(self.flash_file) else b''
        page[:] = list(data[:len(page)].ljust(len(page), b'\xff'))

    def save_flash(self):
        if self.flash_file:
            os.makedirs(os.path.dirname(self.flash_file), exist_ok=True)
            open(self.flash_file, 'wb').write(bytes(self.flash()))

    def maze_saved(self):
        return any(b != 0xFF for b in self.flash())

    def flash_erases(self):
        """How many times its firmware erased its maze's page (it does before each save)."""
        return self.lib.bsim_flash_erases()

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def step(self, enc, ir, gyro, button):
        """enc: (left, right) counts; ir: [(dark, lit)] by sensor id; gyro: LSB. One ms; returns the duties (-1..1)
        and whether each side's driver is on."""
        lib = self.lib
        lib.bsim_set_encoders(*enc)
        for k, (dark, lit) in enumerate(ir):
            lib.bsim_set_ir(k, dark, lit)
        lib.bsim_set_gyro(gyro)
        lib.bsim_set_button(button)
        self.state = lib.bsim_step()
        lib.bsim_get_motors(C.byref(self._l), C.byref(self._r), C.byref(self._on_l), C.byref(self._on_r))
        return (self._l.value / B.PWM_PERIOD, self._r.value / B.PWM_PERIOD,
                bool(self._on_l.value), bool(self._on_r.value))

    def time_ms(self):
        return self.lib.bsim_time_ms()

    def leds(self):
        return self.lib.bsim_leds()

    def send(self, text):
        data = text.encode()
        self.lib.bsim_uart_rx(data, len(data))

    def received(self):
        n = self.lib.bsim_uart_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class BRobot:
    def __init__(self, maze, seed=0, battery_v=B.BATTERY_V, flash_file=FLASH_FILE, erase=False):
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
        self.ir_sensors = model.ir_sensors()
        self.sites = {n: m.site(f'ir_{n}').id for n in self.ir_sensors}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.r = B.WHEEL_RADIUS * MM
        self.hand = Hand(self.model, self.data)
        self.button_until = 0
        self.put_down((0, 0), 0.0)
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}
        self.fw = BFirmware(build(), flash_file)
        self.fw.lib.bsim_set_battery(int(round(min(battery_v / 2, 3.3) / 3.3 * 4095)))    # its divider by 2
        self.fw.boot()
        if erase:
            self.fw.flash()[:] = [0xFF] * len(self.fw.flash())
        else:
            self.fw.load_flash()

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button='user', ms=150):
        """Its user button (PA12) held for ms."""
        if button != 'user':
            raise ValueError('Bulebule has one button: user')
        self.button_until = self.fw.time_ms() + ms

    def send(self, line):
        """A line typed in a terminal connected to it over Bluetooth (its commands end with a NUL)."""
        self.fw.send(line + '\0')

    def cover(self, sensors, ms=300):
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def sensor_ray(self, name):
        deg = math.radians(self.ir_sensors[name][2])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def lift(self):
        raise ValueError('Bulebule is started on the floor')

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell, centred across it, its back to the wall behind (as its firmware starts:
        MOUSE_START_SHIFT), facing heading_deg (0 north, positive to the left), still."""
        back = RC.CELL_MM / 2 - B.WALL_WIDTH / 2 - B.TAIL - 0.5     # its tail 0.5 mm off the wall's face
        h = math.radians(heading_deg)
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = ((cell[0] + 0.5) * RC.CELL_MM * MM + math.sin(h) * back * MM,
                      (cell[1] + 0.5) * RC.CELL_MM * MM - math.cos(h) * back * MM, 0.0002)
        q[a + 3:a + 7] = (math.cos(h / 2), 0, 0, math.sin(h / 2))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else B.IR_FAR_MM

    def beam_m(self, name):
        """Each of the beam's rays as the distance its firmware's calibration gives for that length of beam (m): as
        it was calibrated, the sides facing a side wall (to its face, from the robot's axis), the fronts facing a
        front wall (to its middle, from its position's point) (config.py)."""
        _, (x, y, _), deg = self.ir_sensors[name]
        if name in ('SL', 'SR'):
            base, along = abs(x), abs(math.cos(math.radians(deg)))
        else:
            base, along = y - B.REFERENCE_Y + B.WALL_WIDTH / 2, math.sin(math.radians(deg))
        sid = self.sites[name]
        p = self.data.site_xpos[sid]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        out = []
        for k in RAYS:
            a = math.radians(deg + k * B.IR_BEAM_DEG)
            length = min(self._ray(p, rot @ np.array([math.cos(a), math.sin(a), 0.0])), B.IR_FAR_MM)
            out.append(max(base + length * along, 1.0) * MM)
        return out

    def light(self, name):
        """The light its emitter adds: its calibration's for each ray's distance, the beam's mean (a wall that fills
        it reads as its calibration says, a post that fills part of it less)."""
        a, b = B.SENSOR_AB[name]
        return float(np.dot(RAY_WEIGHTS, [math.exp(min(a / (d + b), 8.4)) for d in self.beam_m(name)]))

    def distance_m(self, name):
        """What its firmware reads now, without noise (m)."""
        a, b = B.SENSOR_AB[name]
        return a / math.log(max(self.light(name), 1.0)) - b

    def ir(self):
        """[(dark, lit)] by sensor id: the input with its emitter off, and the light its emitter adds."""
        out = []
        for name, (_, _, _) in self.ir_sensors.items():
            light = self.light(name) * (1 + self.rng.gauss(0.0, B.IR_NOISE))
            dark = B.IR_DARK + self.rng.gauss(0.0, 3.0)
            out.append((max(0, min(4095, int(round(dark)))), max(0, int(round(light)))))
        return out

    def gyro(self):
        w = self.data.qvel[self.free_dof + 5]                        # rad/s about z, counter-clockwise
        v = math.degrees(w) * B.GYRO_LSB_PER_DPS + self.rng.gauss(0.0, B.GYRO_NOISE_LSB)
        return max(-32768, min(32767, int(round(v))))

    def encoders(self):
        q = self.data.qpos
        return tuple(int(math.floor((q[self.wheel_qpos[s]] - self.enc_zero[s]) / (2 * math.pi) * B.COUNTS_PER_REV * B.GEAR))
                     for s in 'lr')

    # ---- one millisecond ---------------------------------------------------------------------------------------
    def _torque(self, s, duty, on):
        w = self.data.qvel[self.wheel_dof[s]] * B.GEAR              # motor shaft rad/s
        if not on:                                                   # coasting
            t = -math.copysign(B.MOTOR_FRICTION_NM, w) if abs(w) > 1 else 0.0
        else:
            v = max(-1.0, min(1.0, duty)) * B.MOTOR_DRIVER_V
            t = B.MOTOR_KE * (v - B.MOTOR_KE * w) / B.MOTOR_R_OHM
            if abs(w) > 1:
                t -= math.copysign(B.MOTOR_FRICTION_NM, w)
        return t * B.GEAR * B.GEAR_EFFICIENCY

    def step(self):
        t = self.fw.time_ms()
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors], HAND_MM)
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        dl, dr, on_l, on_r = self.fw.step(self.encoders(), self.ir(), self.gyro(), t < self.button_until)
        self.data.ctrl[self.act['l']] = self._torque('l', dl, on_l)
        self.data.ctrl[self.act['r']] = self._torque('r', dr, on_r)
        mujoco.mj_step(self.model, self.data, nstep=2)
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
