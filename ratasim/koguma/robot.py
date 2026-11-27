"""KOGUMA simulated: MuJoCo physics, its motors, encoders, gyro, IR sensors, battery and fan around its firmware on
its simulated chip (koguma/fw, through ctypes). Every half millisecond: its encoders' shafts, its gyro, its IR
sensors' readings, half a millisecond of firmware, its motors' duties to torques, its fan's to downforce, one physics
step.

Its motors (config.py): a DC motor at the wheel, torque = K * (V - K * w) / R, V its duty times its battery's
voltage (its MP6550 drivers brake between pulses: both inputs high), its gears' and fan's losses as the wheel joint's
friction; a channel stopped, the motor coasts. Its battery: 16 V less its 0.64 ohm times what its motors and fan
draw.
Its IR sensors: each emitter's beam to the first wall (three rays across its spot); its reading what its maker
measured at that distance (config.IR_TABLE, its beam's length for each distance), interpolated as powers of the
distance and beyond its table's ends the same.
Its encoders: AS5047P on a 41-tooth gear its wheel's spur gear turns 1.7073 times a turn of the wheel; the left one
counts up going forward, the right one down (its firmware's signs).
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
from ..maze import E, N, S, W
from ..model import MM, WALL_GROUP
from .. import config as RC
from ..start import back_mm
from . import config as K
from . import model

FW_DIR = os.path.join(K.HERE, 'fw')
LIB = os.path.join(K.BUILD, 'libkogumafw.so')
# What each sensor sees: its laser diode's narrow spot, as rays (deg off its axis, weight), a Gaussian of 2 deg: a
# wall fills it, a post (12 mm) only part of it once far. Tried against its fastest run: 1 to 3 deg reach its goal
# (its wall ends where its Edge_Off expects them, 87 to 99 mm), 4 and 6 do not (its wall ends smeared over the wall)
IR_SIGMA_DEG = float(os.environ.get('KOGUMA_IR_SIGMA', '2'))
IR_RAYS = tuple((a, math.exp(-a * a / (2 * IR_SIGMA_DEG ** 2))) for a in
                (IR_SIGMA_DEG * k / 1.5 for k in (-3, -2, -1, 0, 1, 2, 3)))
IR_RAYS_WEIGHT = sum(w for _, w in IR_RAYS)
FAR_POWER = 2.5         # its readings past its table's farthest point (204 mm ahead, 133 mm aside), assumed: its
                        # table flattens there (FR 120, 112 at 192, 204 mm), and at that rate its front sensors would
                        # see, where its search reads the next cell's front wall, the one a cell beyond it
STEP_NS = 500000
HAND_MM = {('FL',): 30.0, ('L', 'R'): 12.0}     # its maker's hand: close over its side sensors


def build():
    subprocess.run(['make', '-s', '-j8', '-C', FW_DIR, f'KOGUMA_DIR={K.KOGUMA_DIR}'], check=True)
    return LIB


class KFirmware:
    """One boot of its firmware: its own copy of the library (its globals start as on the chip)."""

    def __init__(self, lib_path, seed=0):
        self._tmp = tempfile.NamedTemporaryFile(prefix='kogumafw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.ksim_boot.argtypes = [C.c_uint64]
        lib.ksim_run_until.argtypes = [C.c_uint64]
        lib.ksim_run_until.restype = C.c_uint8
        lib.ksim_time_ns.restype = C.c_uint64
        lib.ksim_set_encoder.argtypes = [C.c_int, C.c_double, C.c_double]
        lib.ksim_set_ir.argtypes = [C.c_int, C.c_double, C.c_double]
        lib.ksim_set_noise.argtypes = [C.c_double, C.c_double]
        lib.ksim_set_imu.argtypes = [C.c_double] * 4
        lib.ksim_set_battery.argtypes = [C.c_double]
        lib.ksim_set_switch.argtypes = [C.c_int]
        lib.ksim_motor.argtypes = [C.c_int, C.POINTER(C.c_double)]
        lib.ksim_fan.restype = C.c_double
        lib.ksim_buzzer.restype = C.c_double
        lib.ksim_leds.restype = C.c_uint32
        lib.ksim_console.argtypes = [C.c_char_p, C.c_uint32]
        lib.ksim_console.restype = C.c_uint32
        self._buf = C.create_string_buffer(1 << 16)
        self._duty = C.c_double()
        self.state = 0
        lib.ksim_set_noise(K.IR_NOISE, K.GYRO_NOISE_DPS)
        lib.ksim_set_battery(K.BATTERY_V)
        if not lib.ksim_boot(seed):
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

    def motor(self, side):
        """(driven, duty): its driver drives it (duty of its battery, + forward), or it coasts."""
        driven = self.lib.ksim_motor(side, C.byref(self._duty))
        return bool(driven), self._duty.value

    def received(self):
        n = self.lib.ksim_console(self._buf, len(self._buf))
        return self._buf.raw[:n].decode(errors='replace')

    def var(self, name, ctype=C.c_int):
        """One of its firmware's globals."""
        return ctype.in_dll(self.lib, name).value


def _ir_curves():
    """Each sensor's reading (counts) over its beam's length (mm): its maker's table, as log-log segments."""
    out = {}
    for name, (d, v) in K.IR_TABLE.items():
        beam = np.array([K.ir_beam_mm(name, x) for x in d])
        out[name] = (np.log(beam), np.log(np.array(v, float) * K.IR_SCALE[name]))
    return out


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
        self.curves = _ir_curves()
        self.battery_oc = battery_v
        self.battery_v = battery_v
        self.current = 0.0
        self.gyro_bias = K.GYRO_BIAS_DPS * (1 if self.rng.random() < 0.5 else -1)
        self.hand = Hand(self.model, self.data)
        self.hand_mm = 30.0
        self.lifted = None
        self.turning = {}           # wheel: (rad a ms, until ms), turned by hand
        self.switch_until = -1
        self.fw = KFirmware(build(), seed)
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button=0, ms=150):
        """Its switch S2 (only its fail-safe's release reads it)."""
        self.switch_until = self.fw.time_ms() + ms

    def send(self, line):
        raise ValueError('KOGUMA is started with its wheels and its sensors, as its maker does')

    def cover(self, sensors, ms=500):
        """A hand in front of these sensors for ms (ratasim/hand.py); close over its side sensors, which look out."""
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms
        self.hand_mm = HAND_MM.get(tuple(sorted(sensors)), HAND_MM.get(tuple(sensors), 30.0))

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += 0.08
        self.lifted = q

    def turn_wheel(self, side, turns, ms=400):
        """Its wheel turned by hand (held up): turns of the wheel forward (negative back) in ms."""
        self.turning[side] = (2 * math.pi * turns / ms, self.fw.time_ms() + ms)

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell, facing heading_deg (0 north, positive to the left), its axle START_BACK_MM behind the
        cell's centre: its tail against the back wall, as its firmware's start expects it."""
        h = math.radians(heading_deg)
        back = back_mm(self.model, self.data, self.body, K.START_BACK_MM)
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = (((cell[0] + 0.5) * RC.CELL_MM + back * math.sin(h)) * MM,
                      ((cell[1] + 0.5) * RC.CELL_MM - back * math.cos(h)) * MM, (K.WHEEL_RADIUS_MM + 0.2) * MM)
        q[a + 3:a + 7] = (math.cos(h / 2), 0, 0, math.sin(h / 2))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def sensor_ray(self, name):
        """A sensor's origin and direction in the world."""
        deg = math.radians(K.IR_SENSORS[name][2])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else 2000.0

    def ir_counts(self):
        """Each sensor's reading (lit minus dark, counts): its spot as IR_RAYS rays across it, each one what its
        maker's table gives at the distance it hits something, weighted. A wall fills the spot: its reading is its
        table's; a post (12 mm) takes only part of it, as on the robot."""
        out = {}
        R = self.data.site_xmat
        for name, (_, _, deg) in K.IR_SENSORS.items():
            sid = self.sites[name]
            p = self.data.site_xpos[sid]
            rot = R[sid].reshape(3, 3)
            total = 0.0
            for spread, w in IR_RAYS:
                a = math.radians(deg + spread)
                total += w * self.table_counts(name, self._ray(p, rot @ np.array([math.cos(a), math.sin(a), 0.0])))
            out[name] = total / IR_RAYS_WEIGHT
        return out

    def table_counts(self, name, beam):
        """A reading (lit minus dark, counts) at this beam length (mm): its maker's table."""
        xs, ys = self.curves[name]
        x = math.log(max(beam, 5.0))
        if x <= xs[0]:
            k = (ys[1] - ys[0]) / (xs[1] - xs[0])
            y = ys[0] + k * (x - xs[0])
        elif x >= xs[-1]:          # past its table, falling at least as the distance to the 2.5
            k = min((ys[-1] - ys[-2]) / (xs[-1] - xs[-2]), -FAR_POWER)
            y = ys[-1] + k * (x - xs[-1])
        else:
            y = float(np.interp(x, xs, ys))
        return min(math.exp(y), 4095.0 - K.IR_DARK)

    def _imu(self):
        w = math.degrees(self.data.qvel[self.free_dof + 5])            # its body's z rate (deg/s)
        rot = self.data.xmat[self.body].reshape(3, 3)
        return w + self.gyro_bias, float(rot[2, 2])

    # ---- its motors, battery and fan -----------------------------------------------------------------------------
    def _drive(self):
        """Its motors' torques at the wheels; what they and its fan draw from its battery."""
        amps = 0.0
        for i, s in enumerate('lr'):
            driven, duty = self.fw.motor(i)
            w = self.data.qvel[self.wheel_dof[s]]
            if driven:
                i_m = (duty * self.battery_v - K.MOTOR_K * w) / K.MOTOR_R_OHM
                self.data.ctrl[self.act[s]] = K.MOTOR_K * i_m
                amps += abs(duty * i_m)
            else:
                self.data.ctrl[self.act[s]] = 0.0
        fan_v = self.fan_v()
        amps += K.FAN_CURRENT_A * fan_v / K.FAN_RATED_V
        self.current = amps
        self.battery_v = self.battery_oc - K.BATTERY_R_OHM * amps
        self.fw.lib.ksim_set_battery(self.battery_v)
        return fan_v

    def fan_v(self):
        return self.fw.lib.ksim_fan() * self.battery_v

    # ---- half a millisecond --------------------------------------------------------------------------------------
    def _inputs(self):
        t = self.fw.time_ms()
        lib = self.fw.lib
        lib.ksim_set_switch(1 if t < self.switch_until else 0)
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors], self.hand_mm)
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        k = K.encoder_turns_per_rad() * 2 * math.pi
        for i, s in enumerate('lr'):
            q = self.data.qpos[self.wheel_qpos[s]] - self.enc_zero[s]
            w = self.data.qvel[self.wheel_dof[s]]
            sign = 1 if s == 'l' else -1
            lib.ksim_set_encoder(i, sign * q * k, sign * w * k * 1e-9)
        counts = self.ir_counts()
        for name, (index, _, _) in K.IR_SENSORS.items():
            lib.ksim_set_ir(index, counts[name], K.IR_DARK)
        gz, up = self._imu()
        lib.ksim_set_imu(gz, 0.0, 0.0, up)

    def step(self):
        """One millisecond: two half-millisecond steps."""
        for _ in range(2):
            self._inputs()
            t = self.fw.time_ns()
            if self.fw.run_until(t - t % STEP_NS + STEP_NS):
                return self.fw.state
            fan_v = self._drive()
            self.data.xfrc_applied[self.body, 2] = -K.FAN_DOWNFORCE_N * (fan_v / K.FAN_RATED_V) ** 2
            self.data.xfrc_applied[self.body, 5] = -K.YAW_DAMPING * self.data.qvel[self.free_dof + 5]
            now = self.fw.time_ms()
            for s, (rate, until) in list(self.turning.items()):
                if now >= until:
                    del self.turning[s]
                    self.data.qvel[self.wheel_dof[s]] = 0.0
                    continue
                self.data.qpos[self.wheel_qpos[s]] += rate * STEP_NS * 1e-6
                self.data.qvel[self.wheel_dof[s]] = 0.0
            if self.lifted is not None:
                self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
                self.data.qvel[self.free_dof:self.free_dof + 6] = 0
            mujoco.mj_step(self.model, self.data)
        return self.fw.state

    def leds(self):
        """Its LEDs: LED1..LED8 (bits 0-7), its front LEDs (8-11), its emitters (12-13)."""
        return self.fw.lib.ksim_leds()

    def shows(self):
        """The mode and number its LEDs show (main.c: LED1-4 its mode, LED5-8 its number, the most significant
        first)."""
        leds = self.leds()
        bits = lambda first: sum(8 >> i for i in range(4) if leds & (1 << (first + i)))
        return bits(0), bits(4)

    def led(self):
        """The colour its LEDs show, for videos: any of its eight lit."""
        return (1.0, 0.25, 0.1) if self.leds() & 0xFF else (0.0, 0.0, 0.0)

    def pose(self):
        """(x mm, y mm, heading deg: 0 north, positive to the left)."""
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7]
        w, x_, y_, z = q[3], q[4], q[5], q[6]
        yaw = math.degrees(math.atan2(2 * (w * z + x_ * y_), 1 - 2 * (y_ * y_ + z * z)))
        return q[0] / MM, q[1] / MM, yaw

    def map_errors(self):
        """Its map against the maze: (walls and openings it has seen, [those it got wrong: (x, y, side, it saw a
        wall)]). Its map (Maze.c): G_Maze_Row[y] bit x the wall south of cell (x, y), G_Maze_Column[x] bit y the one
        west of it; Maze_Row_Look and Maze_Column_Look, the ones it has seen."""
        n = self.maze.size
        arr = C.c_uint32 * (n + 1)
        row, col = arr.in_dll(self.fw.lib, 'G_Maze_Row'), arr.in_dll(self.fw.lib, 'G_Maze_Column')
        row_seen, col_seen = arr.in_dll(self.fw.lib, 'Maze_Row_Look'), arr.in_dll(self.fw.lib, 'Maze_Column_Look')
        seen, wrong = 0, []
        for k in range(n + 1):
            for j in range(n):
                for walls, look, side, truth in (
                        (row, row_seen, 'S', self.maze.has(j, k, S) if k < n else self.maze.has(j, n - 1, N)),
                        (col, col_seen, 'W', self.maze.has(k, j, W) if k < n else self.maze.has(n - 1, j, E))):
                    if not (look[k] >> j) & 1:
                        continue
                    seen += 1
                    wall = bool((walls[k] >> j) & 1)
                    if wall != truth:
                        wrong.append((j, k, side, wall) if side == 'S' else (k, j, side, wall))
        return seen, wrong

    def crashed(self):
        m, d = self.model, self.data
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in self.hulls and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
