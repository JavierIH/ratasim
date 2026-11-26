"""Gamusino simulated: MuJoCo physics, its motors, encoders and gyro around its sketch on a simulated ESP32 (gamusino/fw,
through ctypes). Every half millisecond: its encoders (their counts and how fast they change: the sketch reads them
at any time), its gyro and its wall sensors (the ADC, with its LEDs as they are lit) to the chip, half a millisecond
of its sketch, its H-bridge's inputs to motor forces, its suction's duty to a downforce, one physics step.

Its motors (config.py): its H-bridge's two inputs per motor, a duty each; the wheel pushes DRIVE_N_PER_MPS for each m/s
it runs below the speed its maker measured for that duty (both inputs equal: no speed, it brakes).
"""
import ctypes as C
import math
import os
import shutil
import subprocess
import tempfile
import urllib.request

import mujoco
import numpy as np

from .. import config as RC
from ..model import MM, WALL_GROUP
from . import config as G
from . import model

FW_DIR = os.path.join(G.HERE, 'fw')
LIB = os.path.join(G.BUILD, 'libgamusinofw.so')


def build():
    """Its sketch as a library; the LSM6DSR library downloaded first (once)."""
    lsm = os.path.join(G.BUILD, 'LSM6DSR')
    os.makedirs(lsm, exist_ok=True)
    for name in G.LSM6DSR_FILES:
        path = os.path.join(lsm, name)
        if not os.path.exists(path):
            urllib.request.urlretrieve(G.LSM6DSR_URL + name, path + '.part')
            os.replace(path + '.part', path)
    subprocess.run(['make', '-s', '-C', FW_DIR, f'SKETCH={G.SKETCH}', f'LSM6DSR_DIR={lsm}'], check=True)
    return LIB


class GFirmware:
    """One boot of its sketch: its own copy of the library (its globals start as on the chip)."""

    def __init__(self, lib_path, seed=0):
        self._tmp = tempfile.NamedTemporaryFile(prefix='gamusinofw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.gsim_boot.argtypes = [C.c_uint64]
        lib.gsim_run_until.argtypes = [C.c_uint64]
        lib.gsim_run_until.restype = C.c_uint8
        lib.gsim_time_ns.restype = C.c_uint64
        lib.gsim_set_encoder.argtypes = [C.c_int, C.c_double, C.c_double]
        lib.gsim_imu.argtypes = [C.c_int] + [C.c_double] * 5
        lib.gsim_set_imu.argtypes = [C.c_double] * 6
        lib.gsim_pin.argtypes = [C.c_int]
        lib.gsim_pin.restype = C.c_double
        lib.gsim_led.argtypes = [C.c_int]
        lib.gsim_led.restype = C.c_uint32
        lib.gsim_serial_tx.argtypes = [C.c_char_p, C.c_uint32]
        lib.gsim_serial_tx.restype = C.c_uint32
        lib.gsim_serial_rx.argtypes = [C.c_char_p, C.c_uint32]
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        if not lib.gsim_boot(seed):
            raise RuntimeError('gsim_boot failed')
        lib.gsim_imu(G.IMU_CS, *G.GYRO_BIAS_DPS, G.GYRO_NOISE_DPS, G.ACCEL_NOISE_G)

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def run_until(self, t_ns):
        self.state = self.lib.gsim_run_until(int(t_ns))
        return self.state

    def time_ns(self):
        return self.lib.gsim_time_ns()

    def time_ms(self):
        return self.lib.gsim_time_ns() // 1000000

    def pin(self, n):
        return self.lib.gsim_pin(n)

    def led(self, i=0):
        c = self.lib.gsim_led(i)
        return (c >> 16) & 255, (c >> 8) & 255, c & 255

    def send(self, text):
        data = text.encode()
        self.lib.gsim_serial_rx(data, len(data))

    def received(self):
        n = self.lib.gsim_serial_tx(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')


class GRobot:
    STEP_NS = 500000

    def __init__(self, maze, seed=0):
        self.maze = maze
        self.model = mujoco.MjModel.from_xml_string(model.scene_xml(maze))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        m = self.model
        self.body = m.body('robot').id
        self.free_qpos = m.joint('robot').qposadr[0]
        self.free_dof = m.joint('robot').dofadr[0]
        self.wheel_qpos = {s: m.joint(f'wheel_{s}').qposadr[0] for s in 'lr'}
        self.wheel_dof = {s: m.joint(f'wheel_{s}').dofadr[0] for s in 'lr'}
        self.act = {s: m.actuator(f'motor_{s}').id for s in 'lr'}
        self.axle_mm = model._cad()[1]['axle_height']
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}
        self.lifted = None
        self.turning = {}           # wheel: (rad a ms, until ms), turned by hand
        self.rng = np.random.default_rng(seed)
        self.sites = {n: m.site(f'ir_{n}').id for n in G.IR_SENSORS}
        self.ray_group = np.array([1 if g == WALL_GROUP else 0 for g in range(6)], dtype=np.uint8)
        self.ray_geom = np.zeros(1, dtype=np.int32)
        self.fw = GFirmware(build(), seed)

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button, ms=100):
        raise ValueError('Gamusino has no buttons its sketch reads: its menu is worked with its wheels')

    def send(self, line):
        self.fw.send(line + '\n')

    def cover(self, sensors, ms=300):
        raise ValueError('its menu is worked with its wheels, not its sensors')

    def turn_wheel(self, side, turns, ms=400):
        """Its wheel turned by hand (held up): turns of the wheel forward (negative back) in ms."""
        self.turning[side] = (2 * math.pi * turns / ms, self.fw.time_ms() + ms)

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += 0.08
        self.lifted = q

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down with its axle's middle on a cell's centre, facing heading_deg (0 north, positive to the left)."""
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = ((cell[0] + 0.5) * RC.CELL_MM * MM, (cell[1] + 0.5) * RC.CELL_MM * MM, (self.axle_mm + 0.2) * MM)
        h = math.radians(heading_deg) / 2
        q[a + 3:a + 7] = (math.cos(h), 0, 0, math.sin(h))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def _counts_per_rad(self):
        return G.WHEEL_RADIUS_M / G.M_PER_COUNT

    def ir_mm(self):
        """Each wall sensor's beam: how far its axis meets a wall (mm; IR_FAR_MM if none)."""
        out = {}
        for name, (_, _, _, deg) in G.IR_SENSORS.items():
            sid = self.sites[name]
            rot = self.data.site_xmat[sid].reshape(3, 3)
            a = math.radians(deg)
            d = mujoco.mj_ray(self.model, self.data, self.data.site_xpos[sid], rot @ np.array([-math.sin(a), math.cos(a), 0.0]),
                              self.ray_group, 1, self.body, self.ray_geom)
            out[name] = min(d / MM, G.IR_FAR_MM) if d >= 0 else G.IR_FAR_MM
        return out

    def _adc(self):
        """Its phototransistors on the ADC: the maze's light, and the light back from a wall while its LED is lit."""
        beams = self.ir_mm()
        for name, (adc, led, _, _) in G.IR_SENSORS.items():
            v = G.IR_DARK + self.rng.normal(0, G.IR_NOISE)
            if self.fw.pin(led) > 0.5:
                v += G.IR_K / max(beams[name], 1.0) ** 2
            self.fw.lib.gsim_set_adc(adc, int(min(max(v, 0), 4095)))

    def _imu(self):
        w = np.degrees(self.data.qvel[self.free_dof + 3:self.free_dof + 6])       # body frame, deg/s
        rot = self.data.xmat[self.body].reshape(3, 3)
        g = rot.T @ np.array([0.0, 0.0, 1.0])                                        # at rest: +1 g on z
        return (*w, *g)

    # ---- its motors ----------------------------------------------------------------------------------------------
    def _force(self, s):
        back, fwd = (self.fw.pin(p) for p in G.MOTOR_PINS[s])
        net = fwd - back
        duty, speed = G.PWM_TABLE[s]
        free = math.copysign(float(np.interp(abs(net) * 256, duty, speed)), net)
        v = self.data.qvel[self.wheel_dof[s]] * G.WHEEL_RADIUS_M
        return G.DRIVE_N_PER_MPS * (free - v)

    def duties(self):
        return {s: self.fw.pin(G.MOTOR_PINS[s][1]) - self.fw.pin(G.MOTOR_PINS[s][0]) for s in 'lr'}

    # ---- one millisecond ---------------------------------------------------------------------------------------
    def step(self):
        k = self._counts_per_rad()
        for _ in range(2):
            for s in 'lr':
                q = self.data.qpos[self.wheel_qpos[s]] - self.enc_zero[s]
                self.fw.lib.gsim_set_encoder(G.ENCODER_PINS[s], q * k, self.data.qvel[self.wheel_dof[s]] * k * 1e-9)
            self.fw.lib.gsim_set_imu(*self._imu())
            self._adc()
            t = self.fw.time_ns()
            if self.fw.run_until(t - t % self.STEP_NS + self.STEP_NS):
                return self.fw.state
            for s in 'lr':
                self.data.ctrl[self.act[s]] = self._force(s) * G.WHEEL_RADIUS_M
            self.data.xfrc_applied[self.body, 2] = -G.SUCTION_N * self.fw.pin(G.SUCTION_PIN) ** 2
            now = self.fw.time_ms()
            for s, (rate, until) in list(self.turning.items()):
                if now >= until:
                    del self.turning[s]
                    self.data.qvel[self.wheel_dof[s]] = 0.0
                    continue
                self.data.qpos[self.wheel_qpos[s]] += rate * self.STEP_NS * 1e-6
                self.data.qvel[self.wheel_dof[s]] = 0.0
            if self.lifted is not None:
                self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
                self.data.qvel[self.free_dof:self.free_dof + 6] = 0
            mujoco.mj_step(self.model, self.data)
        return self.fw.state

    def leds(self):
        """Its four LEDs' colours (as shown: its sketch dims them to 30/255)."""
        return [self.fw.led(i) for i in range(4)]

    def led(self):
        """The colour its LEDs show, at full brightness, for videos: the brightest of them."""
        c = max(self.leds(), key=sum)
        top = max(c)
        return tuple(v / top for v in c) if top else (0.0, 0.0, 0.0)

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

