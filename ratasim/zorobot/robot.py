"""ZoroBot3 simulated: MuJoCo physics, its motors, encoders, gyro, wall sensors, battery and menu buttons around its
firmware (zorobot/fw, through ctypes). Every millisecond: inputs to the firmware, 1 ms of firmware (its interrupts,
then its main loop until it waits), its PWM to motor torques, two physics steps of 0.5 ms."""
import ctypes as C
import math
import os
import random
import re
import shutil
import struct
import subprocess
import tempfile

import mujoco
import numpy as np

from . import config as Z
from . import model
from ..hand import Hand
from ..model import MM, WALL_GROUP

FW_DIR = os.path.join(Z.HERE, 'fw')
IR_FAR_MM = 400.0
# Its emitters (SFH 4550, +-3 deg at half power): their spot as rays (deg off its axis, weight), a Gaussian of
# 2.5 deg. In its diagonals its front sensors read the posts (12 mm) beside them: three rays read them in steps.
IR_SIGMA_DEG = float(os.environ.get('ZOROBOT_IR_SIGMA', '2.5'))
IR_RAYS = tuple((k * IR_SIGMA_DEG / 1.5, math.exp(-(k / 1.5) ** 2 / 2)) for k in (-3, -2, -1, 0, 1, 2, 3))
IR_RAYS_WEIGHT = sum(w for _, w in IR_RAYS)
ADC_LSB = 3.3 / 4096
VOLT_DIV_A = 3.96           # battery.c, robot version A (the simulated chip's UID)
BUTTON_RAW = {None: 0, 'up': 1500, 'mode': 2500, 'down': 3900}
# Firmware's distance = (a / ln(raw + c) - b) * 1000 + L0: L0 is ROBOT_FRONT_LENGTH or ROBOT_WIDTH / 2.
L0 = {'FL': Z.ROBOT_FRONT_LENGTH, 'FR': Z.ROBOT_FRONT_LENGTH, 'SL': Z.ROBOT_WIDTH / 2, 'SR': Z.ROBOT_WIDTH / 2}
# EEPROM words (include/eeprom.h): gyro offsets 3, sensor offsets 4, raw wall thresholds 4, front middle targets 4
# (mm FL, FR; raw FL, FR), wall lost distances 2, maze 256, run menu 7, RC5 5, then a checksum (all of them sum 0).
EE_SENSORS = 3
EE_MAZE = EE_SENSORS + 4 + 4 + 4 + 2
EE_MENU = EE_MAZE + 256
EE_WORDS = EE_MENU + 7 + 5
CELL, WALL = 180.0, 12.0
MIDDLE = (CELL - WALL) / 2                  # config.h MIDDLE_MAZE_DISTANCE: from its axle to a wall, centred
RAW_THRESHOLD_FACTOR = 1.5                  # config.h SENSOR_RAW_THRESHOLD_DISTANCE_FACTOR
WALL_LOST = (110, 116)                      # config.h LEFT/RIGHT_WALL_LOSS_TO_SENSING_POINT_DISTANCE


def build():
    subprocess.run(['make', '-s', '-C', FW_DIR, f'ZOROBOT3_DIR={Z.ZORO_DIR}'], check=True)
    return os.path.join(Z.BUILD, 'libzorofw.so')


def sensor_calibration(version='A'):
    """The (a, b, c) of each sensor for a robot version, read from its sensors.c."""
    text = open(os.path.join(Z.ZORO_SRC, 'src', 'sensors.c')).read()
    block = text.split(f'case ZOROBOT3_{version}:')[1].split('break;')[0]
    ids = {'FRONT_LEFT': 'FL', 'FRONT_RIGHT': 'FR', 'SIDE_LEFT': 'SL', 'SIDE_RIGHT': 'SR'}
    out = {}
    for name, k, v in re.findall(r'SENSOR_(\w+)_WALL_ID\]\.([abc]) = ([-\d.]+);', block):
        out.setdefault(ids[name], {})[k] = float(v)
    return {n: (d['a'], d['b'], d['c']) for n, d in out.items()}


def raw_at(cal, name, mm):
    """The emitter-on minus emitter-off counts its calibration (a, b, c) gives at mm from its axle."""
    a, b, c = cal[name]
    return max(0.0, math.exp(a / (max(mm - L0[name], 1.0) / 1000.0 + b)) - c)


def flash_image(menu, cal):
    """Its EEPROM sector as its calibration routines leave it on a robot whose sensors read as its (a, b, c) say
    (offsets 0): raw thresholds as front_sensors_calibration (a wall at a cell less half a wall) and
    side_sensors_calibration (centred) set them, the front middle targets as front_sensors_middle_calibration (centred,
    a wall ahead), its makers' wall lost distances, the run menu's values (speed, race, explore type, floodfill type,
    maze type, solve strategy, explore algorithm) and the checksum eeprom_load() checks."""
    words = [0] * EE_WORDS
    f2 = RAW_THRESHOLD_FACTOR ** 2
    words[EE_SENSORS + 4:EE_SENSORS + 8] = [int(raw_at(cal, 'FL', CELL - WALL / 2) / f2),
                                            int(raw_at(cal, 'FR', CELL - WALL / 2) / f2),
                                            int(raw_at(cal, 'SL', MIDDLE) / f2), int(raw_at(cal, 'SR', MIDDLE) / f2)]
    words[EE_SENSORS + 8:EE_SENSORS + 12] = [int(MIDDLE), int(MIDDLE), int(raw_at(cal, 'FL', MIDDLE)),
                                             int(raw_at(cal, 'FR', MIDDLE))]
    words[EE_MAZE - 2:EE_MAZE] = WALL_LOST
    words[EE_MENU:EE_MENU + 7] = menu
    words = [((w + 0x8000) & 0xffff) - 0x8000 for w in words]
    words.append(((-sum(words) + 0x8000) & 0xffff) - 0x8000)
    data = b''.join(struct.pack('<i', w) for w in words)
    return data + b'\xff' * (4096 - len(data))


class ZFirmware:
    def __init__(self, lib_path, flash=None):
        self._tmp = tempfile.NamedTemporaryFile(prefix='zorofw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.zsim_step.restype = C.c_uint8
        lib.zsim_time_ms.restype = C.c_uint32
        lib.zsim_set_encoders.argtypes = [C.c_int32, C.c_int32]
        lib.zsim_set_ir.argtypes = [C.c_uint8, C.c_float]
        lib.zsim_set_gyro.argtypes = [C.c_float]
        lib.zsim_set_aux.argtypes = [C.c_uint16, C.c_uint16]
        lib.zsim_get_motors.argtypes = [C.POINTER(C.c_int16), C.POINTER(C.c_int16), C.POINTER(C.c_uint8)]
        lib.zsim_oc.restype = C.c_uint32
        lib.zsim_oc.argtypes = [C.c_uint8, C.c_uint8]
        lib.zsim_gpio.restype = C.c_uint16
        lib.zsim_uart_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.zsim_uart_tx.restype = C.c_uint32
        lib.zsim_flash_load.argtypes = [C.c_char_p, C.c_uint32]
        lib.is_race_started.restype = C.c_bool
        if flash:
            lib.zsim_flash_load(flash, len(flash))
        self._l, self._r, self._en = C.c_int16(), C.c_int16(), C.c_uint8()
        self._buf = C.create_string_buffer(1 << 17)
        self.state = 0
        if not lib.zsim_boot():
            raise RuntimeError('zsim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def step(self, enc, ir, gyro_dps, battery_raw, button_raw):
        lib = self.lib
        lib.zsim_set_encoders(*enc)
        for i, s in enumerate(ir):
            lib.zsim_set_ir(i, s)
        lib.zsim_set_gyro(gyro_dps)
        lib.zsim_set_aux(battery_raw, button_raw)
        self.state = lib.zsim_step()
        lib.zsim_get_motors(C.byref(self._l), C.byref(self._r), C.byref(self._en))
        return self._l.value / 1024.0, self._r.value / 1024.0, bool(self._en.value), lib.zsim_oc(1, 0) / 1024.0

    def rgb(self):
        return tuple(min(1.0, self.lib.zsim_oc(1, ch) / 255.0) for ch in (2, 1, 3))    # TIM1: OC3 r, OC2 g, OC4 b

    def time_ms(self):
        return self.lib.zsim_time_ms()

    def race_started(self):
        return bool(self.lib.is_race_started())

    def received(self):
        n = self.lib.zsim_uart_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class ZRobot:
    def __init__(self, maze, menu=(0, 0, 0, 0, 1, 0, 2), seed=0, battery_v=Z.BATTERY_V):
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
        self.sites = {n: m.site(f'ir_{n}').id for n in Z.IR_SENSORS}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.r = Z.WHEEL_RADIUS_MM * MM
        self.cal = sensor_calibration('A')
        self.battery_raw = int(battery_v / (ADC_LSB * VOLT_DIV_A))
        self.battery_v = battery_v
        self.button, self.button_until = None, 0
        self.hand = Hand(self.model, self.data)
        self.fan = 0.0
        self.lib = build()
        self.fw = ZFirmware(self.lib, flash_image(list(menu), self.cal))
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}

    # ---- inputs --------------------------------------------------------------------------------------------
    def press(self, button, ms=100):
        self.button, self.button_until = button, self.fw.time_ms() + ms

    def cover(self, sensors, ms=500):
        """A hand in front of these sensors for ms (ratasim/hand.py; how ZoroBot3 is started)."""
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def lift(self):
        raise ValueError('ZoroBot3 is started on the floor')

    def sensor_ray(self, name):
        """A sensor's origin and direction in the world."""
        deg = math.radians(Z.IR_SENSORS[name][2])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else IR_FAR_MM

    def _hit(self, origin, direction):
        """The beam's length to the first wall or post (mm) and the cosine between it and that face's normal."""
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        if d < 0:
            return IR_FAR_MM, 1.0
        g = self.ray_geom[0]
        rot = self.data.geom_xmat[g].reshape(3, 3)
        local = rot.T @ (origin + direction * d - self.data.geom_xpos[g])
        k = int(np.argmax(np.abs(local) / self.model.geom_size[g]))       # the box's face it hits
        return d / MM, abs(float(rot[:, k] @ direction))

    def _counts(self, name, beam):
        """Emitter-on minus emitter-off counts for a wall square to its calibration's at this beam length (mm): the
        inverse of the firmware's own conversion, for the distance its calibration means (fronts: from the axle to a
        wall straight ahead; sides: from the axle to the side wall)."""
        _, (x, y), deg = Z.IR_SENSORS[name]
        a = math.radians(deg)
        dist = beam * math.sin(a) + y if name in ('FL', 'FR') else beam * abs(math.cos(a)) - abs(x)
        a_, b_, c_ = self.cal[name]
        d = max(dist - L0[name], 1.0) / 1000.0
        return max(0.0, math.exp(min(a_ / (d + b_), 9.0)) - c_)

    def ir_signals(self):
        """Each sensor's reading: its emitter's spot as IR_RAYS rays across it, each one's light at the length it
        reaches a wall or post, times the cosine of its incidence over the one its calibration had (fronts: a wall
        ahead; sides: the side wall), weighted. A post takes part of the spot, a face at a slant returns less."""
        sig = []
        R = self.data.site_xmat
        for name, (_, _, deg) in Z.IR_SENSORS.items():
            sid = self.sites[name]
            p = self.data.site_xpos[sid]
            rot = R[sid].reshape(3, 3)
            a0 = math.radians(deg)
            cal_cos = abs(math.sin(a0)) if name in ('FL', 'FR') else abs(math.cos(a0))
            total = 0.0
            for spread, w in IR_RAYS:
                a = a0 + math.radians(spread)
                beam, cos = self._hit(p, rot @ np.array([math.cos(a), math.sin(a), 0.0]))
                total += w * self._counts(name, beam) * cos / cal_cos
            sig.append(total / IR_RAYS_WEIGHT * (1.0 + self.rng.gauss(0.0, 0.004)))
        return sig

    def encoders(self):
        q = self.data.qpos
        mm = {s: (q[self.wheel_qpos[s]] - self.enc_zero[s]) * self.r / MM for s in 'lr'}
        return tuple(math.floor(mm[s] * 1000.0 / Z.MICROMETERS_PER_TICK) for s in 'lr')

    def gyro_dps(self):
        w = self.data.qvel[self.free_dof + 3:self.free_dof + 6]    # body frame
        return math.degrees(w[2]) + self.rng.gauss(0.0, 0.05)

    # ---- one millisecond -----------------------------------------------------------------------------------------
    def _torque(self, s, duty, enabled):
        w = self.data.qvel[self.wheel_dof[s]] * Z.GEAR      # motor shaft rad/s
        if not enabled:
            return -math.copysign(Z.MOTOR_FRICTION_NM, w) * Z.GEAR if abs(w) > 1 else 0.0
        v = max(-1.0, min(1.0, duty)) * self.battery_v
        t = Z.MOTOR_KE * (v - Z.MOTOR_KE * w) / Z.MOTOR_R_OHM
        if abs(w) > 1:
            t -= math.copysign(Z.MOTOR_FRICTION_NM, w)
        return t * Z.GEAR * Z.GEAR_EFFICIENCY

    def step(self):
        t = self.fw.time_ms()
        if self.button and t >= self.button_until:
            self.button = None
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        dl, dr, en, fan = self.fw.step(self.encoders(), self.ir_signals(), self.gyro_dps(), self.battery_raw,
                                       BUTTON_RAW[self.button])
        self.data.ctrl[self.act['l']] = self._torque('l', dl, en)
        self.data.ctrl[self.act['r']] = self._torque('r', dr, en)
        self.fan = fan
        self.data.xfrc_applied[self.body, 2] = -Z.FAN_DOWNFORCE_N * fan * fan
        mujoco.mj_step(self.model, self.data, nstep=2)
        return self.fw.state

    def pose(self):
        """(x mm, y mm, heading deg: 0 north, positive to the left)."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7]
        w, z = q[3], q[6]
        x_, y_ = q[4], q[5]
        yaw = math.degrees(math.atan2(2 * (w * z + x_ * y_), 1 - 2 * (y_ * y_ + z * z)))
        return q[0] / MM, q[1] / MM, yaw

    def crashed(self):
        m, d = self.model, self.data
        hulls = {m.geom('hull').id, m.geom('hull_front').id}
        for i in range(d.ncon):
            c = d.contact[i]
            if c.geom1 in hulls or c.geom2 in hulls:
                other = c.geom2 if c.geom1 in hulls else c.geom1
                if m.geom_group[other] == WALL_GROUP:
                    return m.geom(other).name or 'wall'
        return None
