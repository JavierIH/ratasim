"""What the scenes share: one MuJoCo scene with Ratatron (unprefixed) and ZoroBot3 (prefixed z_), the other robots
added by name (ROBOTS: each its prefix), their drives without firmware (each robot's own speed profile and motor
model), and an mp4 writer."""
import math
import os
import re
import subprocess

os.environ.setdefault('MUJOCO_GL', 'egl')
import mujoco   # noqa: E402
import numpy as np   # noqa: E402
from PIL import ImageFont   # noqa: E402

from .. import config as C, model as RM   # noqa: E402
from ..robot import TAU_S   # noqa: E402
from ..zorobot import config as Z, model as ZM   # noqa: E402
from ..gamusino import config as GC, model as GM   # noqa: E402
from ..bulebule import config as BC, model as BM   # noqa: E402
from ..nezumibot import config as NC, model as NM   # noqa: E402
from ..perenquen import config as PC, model as PM   # noqa: E402
from ..fujin import config as FC, model as FM   # noqa: E402
from ..futura import config as UC, model as UM   # noqa: E402
from ..ukmarsbot import config as KC, model as KM   # noqa: E402

MM = 0.001
RAT_V, RAT_A = 0.9, 2.0                 # Ratatron's race preset: FAST_FULL_SPEED, RACE_ACCEL_MM_S2
HAKI = dict(v=6.5, a_hard=30.0, v_hard=4.5, a_soft=25.0, fan=0.90)   # ZoroBot3's SPEED_HAKI (2S fan 90 %)
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans{}.ttf'

# Every robot by its name (its command's): its prefix in the scene, scene_xml's flag that adds it (Ratatron and
# ZoroBot3 are always there), its name on screen, its colour there, its wheels' radius (m) and its mass (kg).
ROBOTS = {
    'ratatron': dict(prefix='', flag=None, name='Ratatron', colour=(255, 215, 60), r=C.TIRE_RADIUS_MM * MM, mass=None),
    'zorobot': dict(prefix='z_', flag=None, name='ZoroBot3', colour=(200, 140, 255), r=Z.TIRE_CAD_RADIUS_MM * MM,
                    mass=Z.MASS_KG),
    'gamusino': dict(prefix='g_', flag='gamusino', name='Gamusino', colour=(255, 140, 40), r=GC.WHEEL_RADIUS_M,
                     mass=GC.MASS_KG),
    'bulebule': dict(prefix='b_', flag='bulebule', name='Bulebule', colour=(255, 80, 80), r=BC.WHEEL_RADIUS * MM,
                     mass=BC.MASS_KG),
    'nezumibot': dict(prefix='n_', flag='nezumibot', name='NezumiBot', colour=(90, 220, 230),
                      r=NC.WHEEL_RADIUS_MM * MM, mass=NC.MASS_KG),
    'perenquen': dict(prefix='p_', flag='perenquen', name='Perenquén', colour=(120, 230, 110),
                      r=PC.WHEEL_RADIUS_MM * MM, mass=PC.MASS_KG),
    'fujin': dict(prefix='f_', flag='fujin', name='Fujin', colour=(90, 160, 255), r=FC.TIRE_CAD_RADIUS_MM * MM,
                  mass=FC.MASS_KG),
    'futura': dict(prefix='u_', flag='futura', name='Futura', colour=(255, 120, 200), r=UC.WHEEL_RADIUS_MM * MM,
                   mass=UC.MASS_KG),
    'ukmarsbot': dict(prefix='k_', flag='ukmarsbot', name='UKMARSBOT', colour=(235, 235, 235),
                      r=KC.AXLE_Z * MM, mass=KC.MASS_KG),
}


def robot_names(text):
    """'a,b,c' (or 'all') as a list of ROBOTS' names; a ValueError naming the ones there are."""
    names = list(ROBOTS) if text.strip() == 'all' else [n.strip().lower() for n in text.split(',') if n.strip()]
    bad = [n for n in names if n not in ROBOTS]
    if bad or not names:
        raise ValueError(f'no robot {", ".join(bad) or "given"}: the robots are {", ".join(ROBOTS)} (or all)')
    return list(dict.fromkeys(names))


def scene_of(maze, names, **kw):
    """scene_xml() with the robots of names added (Ratatron and ZoroBot3 are in it anyway: park() the others)."""
    return scene_xml(maze, **{ROBOTS[n]['flag']: True for n in names if ROBOTS[n]['flag']}, **kw)


def park(m, d, names):
    """Ratatron and ZoroBot3 out of the way (5 m off) when they are not in names."""
    for i, n in enumerate(n for n in ('ratatron', 'zorobot') if n not in names):
        Bot(m, d, ROBOTS[n]['prefix']).place(-5.0 - i, -5.0, 0.0)


def scene_xml(maze, with_maze=True, rat_mass=0.200, hull_solref=None, gamusino=False, bulebule=False, nezumibot=False,
              perenquen=False, fujin=False, futura=False,
              ukmarsbot=False):
    """Ratatron's scene (its maze, unless with_maze is False) with ZoroBot3 added (and Gamusino, prefixed g_): the
    robots' hulls hit each other (and stiffer, bouncier with hull_solref, e.g. "0.002 0.3")."""
    C.MASS_KG = rat_mass
    xml = RM.scene_xml(maze)
    if not with_maze:
        xml = re.sub(r'<light [^>]*/>', '', xml.replace(RM.maze_xml(maze), ''))
    zs = ZM.scene_xml(maze)
    zdefaults = ''.join(re.findall(r'<default class="z\w+">.*?</default>\n?', zs))
    zmeshes = ''.join(re.findall(r'<mesh name="z_[^>]*/>', zs)).replace('file="', f'file="{Z.CAD_MESHES}/')
    zrobot = re.sub(r'(name|joint)="(?!z_)', r'\1="z_', ZM.robot_xml())
    sr = f' solref="{hull_solref}"' if hull_solref else ''
    xml = xml.replace('<default class="hull"><geom group="3" contype="2"', f'<default class="hull"><geom group="3" contype="6"{sr}')
    zdefaults = zdefaults.replace('<default class="zhull"><geom group="3" contype="2" conaffinity="1"',
                                  f'<default class="zhull"><geom group="3" contype="2" conaffinity="5"{sr}')
    xml = re.sub(r'(</default>\s*<asset>)', lambda m: zdefaults + m.group(1), xml, count=1)
    xml = xml.replace('</asset>', zmeshes + '</asset>', 1)
    xml = xml.replace('</worldbody>', zrobot + '</worldbody>', 1)
    xml = xml.replace('</actuator>', '<motor name="z_motor_l" joint="z_wheel_l"/>'
                      '<motor name="z_motor_r" joint="z_wheel_r"/></actuator>')
    if gamusino:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: GM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', GM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', GM.robot_xml(prefix='g_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="g_motor_l" joint="g_wheel_l"/>'
                          '<motor name="g_motor_r" joint="g_wheel_r"/></actuator>')
    if bulebule:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: BM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', BM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', BM.robot_xml(prefix='b_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="b_motor_l" joint="b_wheel_l"/>'
                          '<motor name="b_motor_r" joint="b_wheel_r"/></actuator>'
                          f'<equality>{BM.equality_xml("b_")}</equality>')
    if nezumibot:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: NM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', NM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', NM.robot_xml(prefix='n_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="n_motor_l" joint="n_wheel_l"/>'
                          '<motor name="n_motor_r" joint="n_wheel_r"/></actuator>', 1)
    if perenquen:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: PM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', PM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', PM.robot_xml(prefix='p_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="p_motor_l" joint="p_wheel_l"/>'
                          '<motor name="p_motor_r" joint="p_wheel_r"/></actuator>', 1)
        if '<equality>' in xml:
            xml = xml.replace('</equality>', PM.equality_xml('p_') + '</equality>', 1)
        else:
            xml = xml.replace('</actuator>', f'</actuator><equality>{PM.equality_xml("p_")}</equality>', 1)
    if fujin:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: FM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', FM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', FM.robot_xml(prefix='f_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="f_motor_l" joint="f_wheel_l"/>'
                          '<motor name="f_motor_r" joint="f_wheel_r"/></actuator>', 1)
    if futura:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: UM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', UM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', UM.robot_xml(prefix='u_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="u_motor_l" joint="u_wheel_l"/>'
                          '<motor name="u_motor_r" joint="u_wheel_r"/></actuator>', 1)
    if ukmarsbot:
        xml = re.sub(r'(</default>\s*<asset>)', lambda m: KM.defaults_xml() + m.group(1), xml, count=1)
        xml = xml.replace('</asset>', KM.assets_xml() + '</asset>', 1)
        xml = xml.replace('</worldbody>', KM.robot_xml(prefix='k_') + '</worldbody>', 1)
        xml = xml.replace('</actuator>', '<motor name="k_motor_l" joint="k_wheel_l"/>'
                          '<motor name="k_motor_r" joint="k_wheel_r"/></actuator>', 1)
    return xml


def compile(xml):
    m = mujoco.MjModel.from_xml_string(xml)
    m.vis.global_.offwidth, m.vis.global_.offheight = 1920, 1080
    m.body_inertia[m.body('robot').id] *= C.MASS_KG / 0.100      # Ratatron's inertia with its mass
    return m, mujoco.MjData(m)


class Bot:
    """A robot of the scene by its prefix: its pose, its wheels and its motors."""

    def __init__(self, m, d, prefix):
        self.m, self.d, self.p = m, d, prefix
        self.body = m.body(f'{prefix}robot').id
        self.qadr = m.joint(f'{prefix}robot').qposadr[0]
        self.vadr = m.joint(f'{prefix}robot').dofadr[0]
        self.dof = {s: m.joint(f'{prefix}wheel_{s}').dofadr[0] for s in 'lr'}
        self.act = {s: m.actuator(f'{prefix}motor_{s}').id for s in 'lr'}

    def place(self, x, y, heading):
        """On the floor at (x, y) m facing heading (rad from +x)."""
        yaw = heading - math.pi / 2                 # the robots' forward is their +y
        self.d.qpos[self.qadr:self.qadr + 2] = (x, y)
        self.d.qpos[self.qadr + 3:self.qadr + 7] = (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))

    def pos(self):
        return self.d.xpos[self.body]

    def speed(self):
        return float(np.linalg.norm(self.d.qvel[self.vadr:self.vadr + 3]))

    def heading(self):
        w, x, y, z = self.d.xquat[self.body]
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) + math.pi / 2

    def tilt_deg(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, self.d.xmat[self.body][8]))))

    def zaxis(self):
        return self.d.xmat[self.body].reshape(3, 3)[:, 2]


class RatatronDrive:
    """Ratatron's motor model (robot.py): torque b * (w_target - w), b from the joint damping. Another robot's (radius
    r m, mass kg): the same speed servo, as quick (WheelServo)."""

    def __init__(self, bot, r=None, mass=None, tau=TAU_S):
        m, self.bot = bot.m, bot
        self.r = r or C.TIRE_RADIUS_MM * MM
        mass = mass or C.MASS_KG
        self.b = {}
        for s in 'lr':
            jw = m.body_inertia[m.body(f'{bot.p}wheel_{s}').id][0] + m.dof_armature[bot.dof[s]]
            self.b[s] = (0.5 * mass * self.r ** 2 + jw) / tau
        self.engage(True)

    def engage(self, on):
        """Its motors on, or free wheels (coasting)."""
        for s in 'lr':
            self.bot.m.dof_damping[self.bot.dof[s]] = self.b[s] if on else 0.0
            if not on:
                self.bot.d.ctrl[self.bot.act[s]] = 0.0

    def drive(self, v_l, v_r):
        for s, v in (('l', v_l), ('r', v_r)):
            self.bot.d.ctrl[self.bot.act[s]] = self.b[s] * v / self.r


def WheelServo(bot, name):
    """A drive for the robot of ROBOTS' name: a speed servo at its wheels' radius and its mass."""
    return RatatronDrive(bot, ROBOTS[name]['r'], ROBOTS[name]['mass'])


class ZoroDrive:
    """ZoroBot3's coreless motors (zorobot/robot.py) under a feed-forward plus P speed control, and its fan."""

    def __init__(self, bot):
        self.bot, self.r = bot, Z.WHEEL_RADIUS_MM * MM

    def torque(self, s, v_ref):
        w = self.bot.d.qvel[self.bot.dof[s]] * Z.GEAR
        w_ref = v_ref / self.r * Z.GEAR
        duty = max(-1.0, min(1.0, w_ref * Z.MOTOR_KE / Z.BATTERY_V + 0.02 * (w_ref - w)))
        tq = Z.MOTOR_KE * (duty * Z.BATTERY_V - Z.MOTOR_KE * w) / Z.MOTOR_R_OHM
        if abs(w) > 1:
            tq -= math.copysign(Z.MOTOR_FRICTION_NM, w)
        return tq * Z.GEAR * Z.GEAR_EFFICIENCY

    def drive(self, v_l, v_r):
        for s, v in (('l', v_l), ('r', v_r)):
            self.bot.d.ctrl[self.bot.act[s]] = self.torque(s, v)

    def off(self):
        for s in 'lr':
            self.bot.d.ctrl[self.bot.act[s]] = 0.0

    def fan(self, duty, down=None):
        """Its suction: FAN_DOWNFORCE_N at full duty, quadratic in it, along down (default: the robot's -z)."""
        f = Z.FAN_DOWNFORCE_N * duty * duty
        self.bot.d.xfrc_applied[self.bot.body, :3] = f * (np.asarray(down) if down is not None else -self.bot.zaxis())


def haki(t):
    """ZoroBot3's SPEED_HAKI profile from rest at t = 0 (m/s)."""
    if t <= 0:
        return 0.0
    t1 = HAKI['v_hard'] / HAKI['a_hard']
    return HAKI['a_hard'] * t if t < t1 else min(HAKI['v_hard'] + HAKI['a_soft'] * (t - t1), HAKI['v'])


def ratatron_race(t):
    """Ratatron's race preset from rest at t = 0 (m/s)."""
    return min(RAT_A * t, RAT_V) if t > 0 else 0.0


def lane_keep(bot, v, heading, offset, k_heading=0.3, k_offset=1.0):
    """(v_left, v_right): turn right when turned or moved left of the lane (offset: m to the robot's left)."""
    e = (bot.heading() - heading + math.pi) % (2 * math.pi) - math.pi
    dv = (k_heading * e + k_offset * offset) * min(1.0, v / 0.5) if v > 0 else 0.0
    return v + dv, v - dv


def project(m, ren, w, h, p):
    """World point p as (x, y) pixels of the renderer's last scene (its free camera), or None behind it."""
    c = ren.scene.camera[0]
    fwd, up = np.array(c.forward), np.array(c.up)
    right = np.cross(fwd, up)
    head = (np.array(c.pos) + np.array(ren.scene.camera[1].pos)) / 2      # its two eyes', half an ipd each side
    v = np.asarray(p) - head
    z = float(v @ fwd)
    if z <= 1e-6:
        return None
    f = h / 2 / math.tan(math.radians(m.vis.global_.fovy) / 2)     # pixels per unit at depth 1
    return w / 2 + f * float(v @ right) / z, h / 2 - f * float(v @ up) / z


def font(size, bold=True):
    return ImageFont.truetype(FONT.format('-Bold' if bold else ''), size)


class Video:
    """An mp4 through ffmpeg: write() takes PIL images or arrays (width x height, RGB)."""

    def __init__(self, path, width=1280, height=720, fps=30):
        self.w, self.h = width, height
        self.ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                    '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-c:v', 'libx264',
                                    '-pix_fmt', 'yuv420p', '-crf', '18', '-movflags', '+faststart', path],
                                   stdin=subprocess.PIPE)

    def write(self, img):
        self.ff.stdin.write(np.asarray(img, np.uint8).tobytes())

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()
