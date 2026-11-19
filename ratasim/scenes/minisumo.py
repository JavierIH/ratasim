"""Minisumo, a battle royale: the robots on a minisumo dohyo, all at once, evenly spaced round its edge facing its
centre; at the start they all charge to the centre at their top speed, then each goes for the nearest robot left;
the last one left on the dohyo wins. A video: the countdown, the charge in slow motion (x4), the rest in real time,
who is out and when, the winner; then a fade to black and the podium (podium.py): its first three on their
blocks, the camera on each from the 3rd to the 1st, the whole podium and fireworks.

    ratasim minisumo out.mp4 [--robots all]       # --robots ratatron,zorobot,fujin: any, by name
    ratasim minisumo --still out.jpg [--at 0.35]   # a picture of the fight, --at s after the start
    ratasim minisumo --still out.jpg --podium      # a picture of the podium after the bout, with its fireworks

No firmware: the robots' firmwares solve mazes and know no sumo. The scene drives them with a control of its own,
the same for every robot: full speed at its target (the centre, then the nearest robot), turning towards it, and a
"line sensor": its centre over the white border's inside and heading out, it turns back to the centre. Each robot's
top speed and acceleration are those of its drag race profile (its firmware's straights), its motors those of the
drag race (Ratatron's and ZoroBot3's models, the others a speed servo as quick as Ratatron's); their masses,
tyres and power as they are. The fans of ZoroBot3 and Fujin are off: minisumo's rules forbid suction (Fuji Soft's
"any device to fix the robot to the dohyo", vacuum in most).

The dohyo (minisumo's rules: Fuji Soft's, the Unified Sumo Robot Rules): 77 cm across including its white border,
2.5 cm wide, 2.5 cm high, black; the shikiri lines are left out (nine robots start round the edge, not behind them).
A robot is out when its centre leaves the dohyo (beyond its edge, 38.5 cm from the centre: by then it falls), or
when it lies toppled (more than 70 degrees off upright) for 0.5 s, out from when it toppled. Two out in the same
millisecond: the one farther from the centre first (if they were the last two, the nearer one stays and wins). At
the time limit (--limit, 30 s) with more than one left: the one nearest the centre wins, the rest after it by
their distance.

A robot stuck (under 8 cm/s, or tilted on another, for 1 s) backs off for 0.3 s and charges again, as sumo robots
do: without it, two robots caught on each other could circle till the limit.
"""
import argparse
import math
import re

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from ..maze import Maze
from ..run import maze_path
from . import common as K
from . import drag_race as DR
from . import podium as P

R_DOHYO, BORDER, H_DOHYO = 0.385, 0.025, 0.025      # m
R_BLACK = R_DOHYO - BORDER
LINE_R = R_BLACK - 0.045        # the line sensor: its centre beyond this, heading out
START_GAP = 0.015               # each robot's back this far inside the white border
TOPPLE_DEG, TOPPLE_S = 70.0, 0.5
COUNTDOWN = 1.5                 # s before the start (shown "3, 2, 1")
SLOW_S, SLOW = 0.8, 4.0         # the first SLOW_S s after the start at x SLOW
STUCK_V, STUCK_S, BACK_S = 0.08, 1.0, 0.3     # stuck: under 8 cm/s (or tilted) for 1 s; then 0.3 s backwards
STEP_MS = 1                     # the control's period (ms)


def top(key):
    """(top speed m/s, acceleration m/s^2) of its drag race profile."""
    p = DR.ROBOTS[key]['profile']
    return p(100.0), p(1e-3) / 1e-3


def extent(bot):
    """(front, back): how far its geoms reach ahead of and behind its body's origin (m)."""
    m, d = bot.m, bot.d
    bot.place(0.0, 0.0, 0.0)
    mujoco.mj_kinematics(m, d)
    x0, hi, lo = float(d.xpos[bot.body][0]), -1.0, 1.0
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    for g in range(m.ngeom):
        if m.body_rootid[m.geom_bodyid[g]] != bot.body or m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        pts = d.geom_xpos[g] + (c + corners * h) @ d.geom_xmat[g].reshape(3, 3).T
        hi, lo = max(hi, float(pts[:, 0].max())), min(lo, float(pts[:, 0].min()))
    return hi - x0, x0 - lo


def scenery():
    """(assets, world geoms): the floor round it, the dohyo (a black cylinder, its white border on top), lights."""
    assets = ('<material name="hall" rgba="0.36 0.37 0.39 1" specular="0.05"/>'
              '<material name="dohyo" rgba="0.03 0.03 0.03 1" specular="0.15" shininess="0.2"/>'
              '<material name="dohyo_white" rgba="0.95 0.95 0.95 1" specular="0.1"/>')
    h = H_DOHYO / 2
    world = ('<geom name="floor" type="plane" size="6 6 0.1" material="hall" condim="3" friction="0.05 0.001 0.0001"/>'
             f'<geom name="dohyo" type="cylinder" size="{R_DOHYO} {h}" pos="0 0 {h}" material="dohyo" condim="3" '
             'friction="0.05 0.001 0.0001"/>'
             f'<geom type="cylinder" size="{R_DOHYO} 0.0001" pos="0 0 {H_DOHYO + 0.0001}" material="dohyo_white" '
             'contype="0" conaffinity="0" mass="0"/>'
             f'<geom type="cylinder" size="{R_BLACK} 0.0001" pos="0 0 {H_DOHYO + 0.0003}" material="dohyo" '
             'contype="0" conaffinity="0" mass="0"/>'
             '<light directional="true" pos="0 -1 3" dir="0.2 0.4 -1" diffuse="0.7 0.7 0.7" castshadow="true"/>'
             '<light directional="true" pos="0 1 3" dir="-0.1 -0.3 -1" diffuse="0.3 0.3 0.3" castshadow="false"/>'
             '<light pos="0 0 1.5" dir="0 0 -1" diffuse="0.25 0.25 0.25" castshadow="false"/>')
    return assets, world


def hulls_collide(xml):
    """Every robot's hull hits every other's: contype bit 4 and conaffinity bit 4 on each (their floor bits kept)."""
    return re.sub(r'(<default class="[a-z]?hull"><geom group="3" contype=")[26](" conaffinity=")(\d)"',
                  lambda m: f'{m.group(1)}6{m.group(2)}{int(m.group(3)) | 4}"', xml)


class Fighter(K.Bot):
    def __init__(self, m, d, key, angle):
        super().__init__(m, d, K.ROBOTS[key]['prefix'])
        self.key, self.name, self.colour = key, K.ROBOTS[key]['name'], K.ROBOTS[key]['colour']
        self.vmax, self.acc = top(key)
        self.front, self.back = extent(self)
        r0 = R_BLACK - START_GAP - self.back
        self.place(r0 * math.cos(angle), r0 * math.sin(angle), angle + math.pi)
        d.qpos[self.qadr + 2] += H_DOHYO
        self.drive = (K.RatatronDrive(self) if key == 'ratatron' else K.ZoroDrive(self) if key == 'zorobot'
                      else K.WheelServo(self, key))
        self.v = 0.0                    # its forward speed command (m/s), ramped at its acceleration
        self.chasing = False            # after the centre: the nearest robot
        self.slow_since = self.back_until = None
        self.out_t = self.toppled_t = None
        self.place_no = None

    def r(self):
        return float(np.hypot(*self.pos()[:2]))

    def stop(self):
        if self.key == 'zorobot':
            self.drive.off()
        else:
            self.drive.engage(False)


class Bout:
    def __init__(self, keys, limit):
        self.keys, self.limit = keys, limit
        assets, world = scenery()
        xml = K.scene_of(Maze.load(maze_path('oshwdem2026')), keys, with_maze=False)
        xml = hulls_collide(xml).replace('</asset>', assets + '</asset>', 1)
        xml = xml.replace('</worldbody>', world + '</worldbody>', 1)
        self.m, self.d = K.compile(xml)
        self.m.vis.quality.numslices = 160          # a round dohyo
        n = len(keys)
        self.bots = [Fighter(self.m, self.d, k, -math.pi / 2 + 2 * math.pi * i / n) for i, k in enumerate(keys)]
        K.park(self.m, self.d, keys)
        mujoco.mj_forward(self.m, self.d)
        self.z0 = None                  # each robot's height on the dohyo, at the start (settled)
        self.root = {b.body: b for b in self.bots}
        self.t, self.k = -COUNTDOWN, 0
        self.events = []                # (t, robot, 'fuera' or 'volcado')
        self.winner = None
        self.end = None

    def standing(self):
        return [b for b in self.bots if b.out_t is None]

    def touching(self):
        """The robots touching another robot now."""
        m, d, hit = self.m, self.d, set()
        for c in d.contact[:d.ncon]:
            a, b = (self.root.get(m.body_rootid[m.geom_bodyid[g]]) for g in (c.geom1, c.geom2))
            if a and b and a is not b:
                hit.update((a.key, b.key))
        return hit

    def judge(self):
        if self.winner is not None:
            return
        t, out = self.t, []
        if self.z0 is None:
            self.z0 = {b.key: float(b.pos()[2]) for b in self.bots}
        for b in self.standing():
            if b.tilt_deg() > TOPPLE_DEG:
                b.toppled_t = b.toppled_t if b.toppled_t is not None else t
            else:
                b.toppled_t = None
            if b.r() > R_DOHYO or b.pos()[2] < self.z0[b.key] - 0.012:
                out.append((b, t, 'fuera'))
            elif b.toppled_t is not None and t - b.toppled_t >= TOPPLE_S:
                out.append((b, b.toppled_t, 'volcado'))
        out.sort(key=lambda o: -o[0].r())
        left = len(self.standing())
        if out and len(out) == left:        # the last ones out together: the nearest the centre stays, the winner
            out.pop()
        for b, when, why in out:
            b.out_t, b.place_no = when, left
            left -= 1
            b.stop()
            self.events.append((t, b, why))
            print(f'{left + 1}. {b.name:9s} out at {when:6.3f} s ({"off the dohyo" if why == "fuera" else "toppled"})')
        st = self.standing()
        if self.winner is None and (len(st) <= 1 or t >= self.limit):
            for i, b in enumerate(sorted(st, key=lambda b: b.r())):
                b.place_no = i + 1
            self.winner = min(st, key=lambda b: b.place_no)
            for b in sorted(st, key=lambda b: b.place_no)[1:]:
                print(f'{b.place_no}. {b.name:9s} still on the dohyo at the limit, {b.r() * 100:.1f} cm from the centre')
            how = 'last one left' if len(st) == 1 else f'nearest the centre at the {self.limit:g} s limit'
            print(f'1. {self.winner.name:9s} wins at {t:.3f} s ({how})')
            print('Podium:', '  '.join(f'{p}. {b.name}' for p, b in P.ranked(self.bots)))
            self.end = t

    def podium(self):
        """[(place, key, name, colour)] of the first three (or two), the 1st first."""
        return [(p, b.key, b.name, b.colour) for p, b in P.ranked(self.bots)]

    def control(self):
        t = self.t
        if t < 0 or self.winner is not None:      # before the start, after the end: still
            for b in self.standing():
                b.drive.drive(0.0, 0.0)
            return
        touching = self.touching()
        st = self.standing()
        for b in st:
            p = b.pos()[:2]
            if not b.chasing and (b.key in touching or np.hypot(*p) < 0.12):
                b.chasing = True
            others = [o for o in st if o is not b]
            target = np.zeros(2)
            if b.chasing and others:
                target = min((o.pos()[:2] for o in others), key=lambda q: float(np.hypot(*(q - p))))
            h = b.heading()
            fwd = np.array([math.cos(h), math.sin(h)])
            if np.hypot(*p) > LINE_R and float(np.dot(fwd, p)) > 0.3 * np.hypot(*p):
                target = np.zeros(2)            # the white border under it, heading out: back to the centre
            dt = STEP_MS * 1e-3
            if b.speed() < STUCK_V or b.tilt_deg() > 10:     # stuck (at a stall, caught on another): back off
                b.slow_since = t if b.slow_since is None else b.slow_since
            else:
                b.slow_since = None
            if b.back_until is None and b.slow_since is not None and t - b.slow_since > STUCK_S:
                b.back_until, b.slow_since = t + BACK_S, None
            if b.back_until is not None:
                if t < b.back_until and np.hypot(*p) < LINE_R:
                    b.v = max(-0.5 * b.vmax, b.v - b.acc * dt)
                    b.drive.drive(b.v, b.v)
                    continue
                b.back_until, b.v = None, 0.0
            want = target - p
            e = (math.atan2(want[1], want[0]) - h + math.pi) % (2 * math.pi) - math.pi
            v_goal = b.vmax * max(0.0, math.cos(e)) ** 3     # turned away: slower, so as not to circle it
            b.v = min(v_goal, b.v + b.acc * dt) if v_goal > b.v else max(v_goal, b.v - b.acc * dt)
            dv = (0.3 + 0.2 * b.vmax) * max(-1.0, min(1.0, e / (math.pi / 4)))
            b.drive.drive(b.v - dv, b.v + dv)

    def step(self):
        """One millisecond."""
        self.control()
        n = max(1, round(STEP_MS * 1e-3 / self.m.opt.timestep))
        for _ in range(n):
            mujoco.mj_step(self.m, self.d)
        if self.d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number:
            raise RuntimeError(f'the simulation went unstable at {self.t:.3f} s')
        self.t = -COUNTDOWN + (self.k + 1) * STEP_MS * 1e-3
        self.k += 1
        if self.t >= 0:
            self.judge()

    def run_to(self, t):
        while self.t < t - 1e-9:
            self.step()


class Screen:
    def __init__(self, bout, w, h):
        self.b, self.w, self.h = bout, w, h
        self.ren, self.cam = mujoco.Renderer(bout.m, h, w), mujoco.MjvCamera()
        self.cam.lookat[:] = (0.0, 0.03, H_DOHYO)
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 1.12, -58, 90
        s = h / 720
        self.big, self.mid, self.tag, self.huge = (K.font(int(n * s)) for n in (30, 22, 17, 64))
        self.small = K.font(int(20 * s), bold=False)
        self.s = s

    def frame(self, rate):
        b, w, h, s = self.b, self.w, self.h, self.s
        self.ren.update_scene(b.d, self.cam)
        img = Image.fromarray(self.ren.render())
        g = ImageDraw.Draw(img)
        for f in b.bots:                    # the names over the robots
            q = K.project(b.m, self.ren, w, h, f.pos() + (0, 0, 0.06))
            if q is None or not (0 <= q[0] < w and 0 <= q[1] < h):
                continue
            tw, th = g.textbbox((0, 0), f.name, font=self.tag)[2:]
            x, y = q[0] - tw / 2, q[1] - th - 6 * s
            fill = (0, 0, 0) if f.out_t is None else (60, 60, 60)
            g.rectangle((x - 5 * s, y - 3 * s, x + tw + 5 * s, y + th + 5 * s), fill=fill)
            g.text((x, y), f.name, font=self.tag, fill=f.colour if f.out_t is None else (150, 150, 150))
        g.rectangle((0, 0, w, 50 * s), fill=(0, 0, 0))
        g.text((16 * s, 9 * s), f'MINISUMO  ·  todos contra todos  ·  {len(b.bots)} robots', font=self.big, fill=(255, 255, 255))
        t = b.t
        clock = f'{max(0.0, t):5.2f} s'
        if rate:
            g.text((w - 330 * s, 14 * s), 'tiempo real' if rate == 1 else f'cámara lenta x{rate:g}', font=self.small, fill=(255, 220, 60))
        g.text((w - 130 * s, 12 * s), clock, font=self.mid, fill=(255, 220, 60))
        rows = sorted(b.bots, key=lambda f: (f.out_t is not None, -(f.out_t or 0), b.bots.index(f)))
        g.rectangle((10 * s, 60 * s, 330 * s, 70 * s + 32 * s * len(rows)), fill=(0, 0, 0))
        for i, f in enumerate(rows):
            y = 66 * s + 32 * s * i
            g.text((20 * s, y), f.name, font=self.mid, fill=f.colour if f.out_t is None else (130, 130, 130))
            res = 'en pie' if f.out_t is None else f'{f.place_no}º · {f.out_t:.2f} s'
            g.text((190 * s, y), res, font=self.small, fill=(255, 255, 255) if f.out_t is None else (150, 150, 150))
        if t < 0:
            n = str(int(math.ceil(-t / (COUNTDOWN / 3))))
            self.centred(g, n, h / 2 - 40 * s, self.huge, (255, 220, 60), x=0.86 * w)
        elif t < 0.5 and rate:
            self.centred(g, '¡YA!', h / 2 - 40 * s, self.huge, (120, 255, 120), x=0.86 * w)
        recent = [(et, f, why) for et, f, why in b.events if t - et < 2.0 and b.winner is None]
        for i, (et, f, why) in enumerate(recent[-3:]):
            msg = f'{f.name} {"fuera" if why == "fuera" else "volcado"} · {f.out_t:.2f} s'
            self.centred(g, msg, h - (60 + 44 * (len(recent[-3:]) - 1 - i)) * s, self.mid, f.colour, box=True)
        if b.winner is not None:
            self.centred(g, f'¡GANA {b.winner.name.upper()}!', h / 2 - 40 * s, self.huge, b.winner.colour, box=True)
        return img

    def centred(self, g, text, y, font, fill, box=False, x=None):
        tw, th = g.textbbox((0, 0), text, font=font)[2:]
        x = (self.w if x is None else 2 * x) / 2 - tw / 2
        if box:
            g.rectangle((x - 14 * self.s, y - 6 * self.s, x + tw + 14 * self.s, y + th + 12 * self.s), fill=(0, 0, 0))
        g.text((x, y), text, font=font, fill=fill)


def fight(out, keys, limit, fps=30):
    bout = Bout(keys, limit)
    video = K.Video(out)
    scr = Screen(bout, video.w, video.h)
    after = None
    while after is None or bout.t < after:
        rate = SLOW if 0 <= bout.t < SLOW_S else 1.0
        bout.run_to(bout.t + 1.0 / fps / rate)
        video.write(scr.frame(rate))
        if after is None and bout.end is not None:
            after = bout.end + 1.0
    img = scr.frame(1.0)
    for _ in range(fps * 2):
        video.write(img)
    dark = np.asarray(img, np.float32)
    for i in range(fps):                                # a fade to black, a moment of black, the podium
        video.write((dark * (1 - (i + 1) / fps)).astype(np.uint8))
    for _ in range(fps // 2):
        video.write(np.zeros_like(dark, np.uint8))
    P.ceremony(bout.podium(), video, fps)
    video.close()


def still(out, keys, at, limit, podium=False):
    bout = Bout(keys, limit)
    if podium:
        while bout.end is None:
            bout.step()
        P.picture(bout.podium(), out)
        return
    bout.run_to(at)
    scr = Screen(bout, 1280, 720)
    scr.frame(None).convert('RGB').save(out, quality=90)


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim minisumo', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out', nargs='?', help='the mp4')
    p.add_argument('--robots', default='all', help=f'which, by name, round the dohyo: {",".join(DR.ROBOTS)} (default all)')
    p.add_argument('--limit', type=float, default=30.0, help='the bout\'s time limit, s')
    p.add_argument('--still', metavar='JPG', help='only a picture of the bout, at --at')
    p.add_argument('--at', type=float, default=0.35, help='--still: seconds after the start')
    p.add_argument('--podium', action='store_true', help='--still: the podium after the bout, with its fireworks')
    a = p.parse_args(argv)
    try:
        keys = K.robot_names(a.robots)
    except ValueError as e:
        p.error(str(e))
    if len(keys) < 2:
        p.error('a bout needs two robots at least')
    if a.still:
        still(a.still, keys, a.at, a.limit, a.podium)
        print('->', a.still)
    elif a.out:
        fight(a.out, keys, a.limit)
        print('->', a.out)
    else:
        p.error('give the mp4, or --still JPG')


if __name__ == '__main__':
    main()
