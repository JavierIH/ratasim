"""Fujin simulated: MuJoCo physics, its motors, encoders, gyro, IR sensors, battery, fan and buttons around its
firmware on its simulated chip (fujin/fw, through ctypes). Every half millisecond (its control loop's period): the
wheels' angles and speeds to its encoders, its gyro, its IR sensors' readings, half a millisecond of firmware, its
motors' duties to torques, its fan's to downforce, one physics step.

Its motors (config.py, its firmware's own model): a DC motor at the wheel, torque = KT * (V - KT * w) / R, V its duty
times the battery's voltage; its gears' friction as the wheel joint's; its tyres' scrub turning in place a torque on its
body.
Its IR sensors: each emitter's beam to the first wall (three rays across its spot); the side ones read it scaled to
its firmware's references with the robot centred in a cell (its mm are not the beam's), the front ones the beam itself.
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
from .. import maze as MZ
from ..hand import Hand
from ..model import MM, WALL_GROUP
from . import config as F
from . import model

FW_DIR = os.path.join(F.HERE, 'fw')
LIB = os.path.join(F.BUILD, 'libfujinfw.so')
IR_SPREAD_DEG = 2.5
STEP_NS = 500000


def _version(cmd):
    try:
        out = subprocess.run([cmd, '-dumpversion'], capture_output=True, text=True).stdout.strip()
        return int(out.split('.')[0])
    except (OSError, ValueError):
        return 0


def compiler():
    """make's variables for a C++23 compiler: CXX if set, g++ 13 or later, else clang++ 15 or later on g++'s C++
    library (its constexpr functions' static variables: an extension clang accepts)."""
    if os.environ.get('CXX'):
        return []
    if _version('g++') >= 13:
        return ['CXX=g++']
    for v in range(25, 14, -1):
        clang = shutil.which(f'clang++-{v}') or (shutil.which('clang++') if _version('clang++') == v else None)
        if not clang:
            continue
        gcc = _version('g++')
        lib = subprocess.run(['g++', '-print-file-name=libstdc++.so'], capture_output=True, text=True).stdout.strip()
        flags = f'-Wno-invalid-constexpr -isystem /usr/include/c++/{gcc} -isystem /usr/include/x86_64-linux-gnu/c++/{gcc}'
        return [f'CXX={clang}', f'CXXFLAGS_EXTRA={flags}', f'LDFLAGS_EXTRA=-L{os.path.dirname(lib)}']
    raise SystemExit('Fujin: its firmware is C++23: it needs g++ 13 or later, or clang++ 15 or later')


def build():
    subprocess.run(['make', '-s', '-j8', '-C', FW_DIR, f'FUJIN_DIR={F.FUJIN_DIR}'] + compiler(), check=True)
    return LIB


class FFirmware:
    """One boot of its firmware: its own copy of the library (its globals start as on the chip)."""

    def __init__(self, lib_path, battery_v):
        self._tmp = tempfile.NamedTemporaryFile(prefix='fujinfw_', suffix='.so', delete=False)
        self._tmp.close()
        shutil.copyfile(lib_path, self._tmp.name)
        lib = self.lib = C.CDLL(self._tmp.name)
        lib.fsim_run_until.argtypes = [C.c_uint64]
        lib.fsim_run_until.restype = C.c_uint8
        lib.fsim_time_ns.restype = C.c_uint64
        lib.fsim_set_encoder.argtypes = [C.c_int, C.c_double, C.c_double]
        lib.fsim_set_ir.argtypes = [C.c_double] * 4
        lib.fsim_ir_model.argtypes = [C.c_double] * 3
        lib.fsim_set_imu.argtypes = [C.c_double, C.c_double]
        lib.fsim_set_battery.argtypes = [C.c_double]
        lib.fsim_button.argtypes = [C.c_int, C.c_int]
        lib.fsim_stripe.restype = C.c_uint32
        lib.fsim_console.argtypes = [C.c_char_p, C.c_uint32]
        lib.fsim_console.restype = C.c_uint32
        lib.fsim_eeprom_write.argtypes = [C.c_uint32, C.c_char_p, C.c_uint32]
        lib.fsim_eeprom_read.argtypes = [C.c_uint32, C.c_char_p, C.c_uint32]
        self._buf = C.create_string_buffer(1 << 16)
        self.state = 0
        lib.fsim_ir_model(*F.IR_MODEL)
        lib.fsim_set_battery(battery_v)
        if not lib.fsim_boot():
            raise RuntimeError('fsim_boot failed')

    def close(self):
        try:
            os.unlink(self._tmp.name)
        except OSError:
            pass

    def run_until(self, t_ns):
        self.state = self.lib.fsim_run_until(int(t_ns))
        return self.state

    def time_ns(self):
        return self.lib.fsim_time_ns()

    def time_ms(self):
        return self.lib.fsim_time_ns() // 1000000

    def stripe(self, i=0):
        c = self.lib.fsim_stripe(i)
        return (c >> 16) & 255, (c >> 8) & 255, c & 255

    def received(self):
        n = self.lib.fsim_console(self._buf, len(self._buf))
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
        self.hulls = {i for i in range(m.ngeom) if m.geom(i).name.startswith('hull')}
        self.ray_group = np.zeros(6, np.uint8)
        self.ray_group[WALL_GROUP] = 1
        self.ray_geom = np.zeros(1, np.int32)
        self.r = F.WHEEL_RADIUS_MM * MM
        self.battery_v = battery_v
        self.ir_scale = self._ir_scales()
        self.hand = Hand(self.model, self.data)
        self.buttons = []           # (button, from ms, to ms)
        self.lifted = None
        self.fw = FFirmware(build(), battery_v)
        self.enc_zero = {s: self.data.qpos[self.wheel_qpos[s]] for s in 'lr'}

    def _ir_scales(self):
        """Each sensor's reading over its beam: the side ones' references with the robot centred in a cell."""
        half = RC.CELL_MM / 2 - RC.WALL_MM / 2
        out = {'FL': 1.0, 'FR': 1.0}
        for name in ('L', 'R'):
            _, (x, _), deg = F.IR_SENSORS[name]
            beam = (half + abs(x)) / abs(math.cos(math.radians(deg)))
            out[name] = F.IR_SIDE_REF_MM[name] / beam
        return out

    # ---- the operator ------------------------------------------------------------------------------------------
    def press(self, button, ms=100):
        t = self.fw.time_ms()
        start = max([t] + [b[2] + 50 for b in self.buttons])
        self.buttons.append((int(button), start, start + ms))

    def send(self, line):
        raise ValueError('Fujin is started with its buttons')

    def cover(self, sensors, ms=500):
        """A hand in front of these sensors for ms (ratasim/hand.py)."""
        self.hand.sensors, self.hand.until = tuple(sensors), self.fw.time_ms() + ms

    def lift(self):
        q = self.data.qpos[self.free_qpos:self.free_qpos + 7].copy()
        q[2] += 0.08
        self.lifted = q

    def load_maze(self, maze=None):
        """Its maze in its EEPROM, every wall known, as its search saves it (services/maze.cpp: a cell in 4 bytes at
        ADDR_MAZE_START + 4 (16 x + y): its walls, N 1, W 2, S 4, E 8; its known ones; its distance, which its run's
        floodfill computes again), in both its map and its backup: a run on a maze it has not searched."""
        maze = maze or self.maze
        bits = {MZ.N: 1, MZ.W: 2, MZ.S: 4, MZ.E: 8}
        data = bytearray()
        for x in range(maze.size):
            for y in range(maze.size):
                walls = sum(b for d, b in bits.items() if maze.has(x, y, d))
                data += bytes((0, 0, 0x0F, walls))     # its u32 of (walls, known, distance, 0), big-endian
        for base in (F.EEPROM_MAZE, F.EEPROM_MAZE_BACKUP):
            self.fw.lib.fsim_eeprom_write(base, bytes(data), len(data))

    def saved_maze(self, backup=False):
        """The maze its search saved in its EEPROM (its walls as it believes them), as a Maze."""
        n = self.maze.size
        buf = C.create_string_buffer(4 * n * n)
        self.fw.lib.fsim_eeprom_read(F.EEPROM_MAZE_BACKUP if backup else F.EEPROM_MAZE, buf, len(buf))
        bits = {MZ.N: 1, MZ.W: 2, MZ.S: 4, MZ.E: 8}
        out = MZ.Maze(n)
        for x in range(n):
            for y in range(n):
                walls = buf.raw[4 * (x * n + y) + 3]
                for d, b in bits.items():
                    if walls & b:
                        out.walls[x][y] |= d
        return out

    def put_down(self, cell=(0, 0), heading_deg=0.0):
        """Down in a cell, facing heading_deg (0 north, positive to the left), its axle START_BACK_MM behind the
        cell's centre: its tail against the back wall, as its firmware's start expects it."""
        h = math.radians(heading_deg)
        back = F.START_BACK_MM
        q = self.data.qpos
        a = self.free_qpos
        q[a:a + 3] = (((cell[0] + 0.5) * RC.CELL_MM + back * math.sin(h)) * MM,
                      ((cell[1] + 0.5) * RC.CELL_MM - back * math.cos(h)) * MM, (F.WHEEL_RADIUS_MM + 0.2) * MM)
        q[a + 3:a + 7] = (math.cos(h / 2), 0, 0, math.sin(h / 2))
        self.data.qvel[self.free_dof:self.free_dof + 6] = 0
        self.lifted = None
        mujoco.mj_forward(self.model, self.data)

    # ---- sensors -----------------------------------------------------------------------------------------------
    def sensor_ray(self, name):
        """A sensor's origin and direction in the world."""
        deg = math.radians(F.IR_SENSORS[name][2])
        sid = self.sites[name]
        rot = self.data.site_xmat[sid].reshape(3, 3)
        return self.data.site_xpos[sid].copy(), rot @ np.array([math.cos(deg), math.sin(deg), 0.0])

    def _ray(self, origin, direction):
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        return d / MM if d >= 0 else 1000.0

    def _hit(self, origin, direction):
        """A ray's length to the first wall or post (mm) and the cosine between it and that face's normal."""
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.ray_group, 1, self.body, self.ray_geom)
        if d < 0:
            return 1000.0, 1.0
        g = self.ray_geom[0]
        rot = self.data.geom_xmat[g].reshape(3, 3)
        local = rot.T @ (origin + direction * d - self.data.geom_xpos[g])
        k = int(np.argmax(np.abs(local) / self.model.geom_size[g]))       # the box's face it hits
        return d / MM, abs(float(rot[:, k] @ direction))

    def ir_mm(self):
        """Each sensor's reading (its firmware's mm), noise in. F.IR_BEAM 'light': its emitter's spot as rays
        (F.IR_RAYS: angle, weight), each one's light (its firmware's raw counts at its length, through its own
        conversion) times the cosine of its incidence over its calibration's (fronts: a wall ahead; sides: the side
        wall with the robot centred), summed, back to mm: a post takes part of the spot, a face at a slant returns
        less. 'harmonic': three rays' lengths, their harmonic mean."""
        out = {}
        R = self.data.site_xmat
        a_, b_, c_ = F.IR_MODEL
        for name, (_, _, deg) in F.IR_SENSORS.items():
            sid = self.sites[name]
            p = self.data.site_xpos[sid]
            rot = R[sid].reshape(3, 3)
            if F.IR_BEAM == 'light':
                a0 = math.radians(deg)
                cal_cos = abs(math.sin(a0)) if name in ('FL', 'FR') else abs(math.cos(a0))
                raw = wsum = 0.0
                for spread, w in F.IR_RAYS:
                    a = a0 + math.radians(spread)
                    beam, cos = self._hit(p, rot @ np.array([math.cos(a), math.sin(a), 0.0]))
                    mm = beam * self.ir_scale[name]
                    raw += w * math.exp(a_ / (mm + b_)) * cos / cal_cos
                    wsum += w
                raw = max(raw / wsum - c_, 1.0001)
                reading = a_ / math.log(raw + c_) - b_ + self.rng.gauss(0.0, F.IR_NOISE_MM)
            else:
                inv = 0.0
                for spread in (-IR_SPREAD_DEG, 0.0, IR_SPREAD_DEG):
                    a = math.radians(deg + spread)
                    v = rot @ np.array([math.cos(a), math.sin(a), 0.0])
                    inv += 1.0 / max(self._ray(p, v), 1.0)
                reading = 3.0 / inv * self.ir_scale[name] + self.rng.gauss(0.0, F.IR_NOISE_MM)
            out[name] = min(max(reading, 5.0), F.IR_MAX_MM + 50)
        return out

    def _imu(self):
        w = math.degrees(self.data.qvel[self.free_dof + 5])            # body frame z, deg/s
        rot = self.data.xmat[self.body].reshape(3, 3)
        return w + F.GYRO_BIAS_DPS + self.rng.gauss(0.0, F.GYRO_NOISE_DPS), float(rot[2, 2])

    # ---- its motors and fan --------------------------------------------------------------------------------------
    def _torque(self, s, duty):
        v = max(-1.0, min(1.0, duty / 1000.0)) * self.battery_v
        w = self.data.qvel[self.wheel_dof[s]]
        drag = (F.ROLL_DRAG + F.ROLL_DRAG_PER_N * self.downforce()) / 2 * (F.WHEEL_RADIUS_MM / 1000) ** 2
        return F.MOTOR_KT * (v - F.MOTOR_KT * w) / F.MOTOR_R_OHM - drag * w

    def downforce(self):
        return F.FAN_DOWNFORCE_N * self.fan() ** 2

    def _scrub(self):
        """Its tyres' scrub turning in place: a torque against its turn, gone as it rolls forward (an arc)."""
        dof = self.free_dof
        w = self.data.qvel[dof + 5]
        rot = self.data.xmat[self.body].reshape(3, 3)
        v = abs(float(rot[:, 1] @ self.data.qvel[dof:dof + 3]))
        return -F.SCRUB_NM * math.tanh(w / 0.3) * math.exp(-v / F.SCRUB_SPEED)

    def fan(self):
        """Its fan's voltage over its rated 10 V."""
        return self.fw.lib.fsim_fan() / 999.0 * self.battery_v / 10.0

    # ---- half a millisecond --------------------------------------------------------------------------------------
    def _inputs(self):
        t = self.fw.time_ms()
        lib = self.fw.lib
        for b in list(self.buttons):
            n, t0, t1 = b
            if t0 <= t < t1 and not getattr(self, f'_held{n}', False):
                setattr(self, f'_held{n}', True)
                lib.fsim_button(n, 1)
            elif t >= t1:
                if getattr(self, f'_held{n}', False):
                    lib.fsim_button(n, 0)
                setattr(self, f'_held{n}', False)
                self.buttons.remove(b)
        if self.hand.sensors and t < self.hand.until:
            self.hand.cover([self.sensor_ray(n) for n in self.hand.sensors])
        elif self.hand.sensors:
            self.hand.sensors = ()
            self.hand.away()
        mujoco.mj_kinematics(self.model, self.data)
        k = F.counts_per_rad()
        for i, s in enumerate('lr'):
            q = self.data.qpos[self.wheel_qpos[s]] - self.enc_zero[s]
            lib.fsim_set_encoder(i, q * k, self.data.qvel[self.wheel_dof[s]] * k * 1e-9)
        ir = self.ir_mm()
        lib.fsim_set_ir(ir['R'], ir['FL'], ir['FR'], ir['L'])
        lib.fsim_set_imu(*self._imu())
        lib.fsim_sample_analog()

    def step(self):
        """One millisecond: two half-millisecond steps."""
        for _ in range(2):
            self._inputs()
            t = self.fw.time_ns()
            if self.fw.run_until(t - t % STEP_NS + STEP_NS):
                return self.fw.state
            for i, s in enumerate('lr'):
                self.data.ctrl[self.act[s]] = self._torque(s, self.fw.lib.fsim_motor(i))
            self.data.xfrc_applied[self.body, 2] = -self.downforce()
            self.data.xfrc_applied[self.body, 5] = self._scrub()
            if self.lifted is not None:
                self.data.qpos[self.free_qpos:self.free_qpos + 7] = self.lifted
                self.data.qvel[self.free_dof:self.free_dof + 6] = 0
            mujoco.mj_step(self.model, self.data)
        return self.fw.state

    def led(self):
        """The colour its two LEDs show (the first), at full brightness, for videos."""
        c = self.fw.stripe(0)
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
        for i in range(d.ncon):
            c = d.contact[i]
            for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a in self.hulls and m.geom_group[b] == WALL_GROUP:
                    return m.geom(b).name or 'wall'
        return None
