"""Ratatron and ZoroBot3 through a loop-the-loop beside the maze: a helical loop (in by one lane, out by the next, a
clothoid at both ends), Ratatron shot by a launcher (its motors give 0.9 m/s, the loop needs ~3.5), ZoroBot3 behind
it with its motors and fan. By default Ratatron is too slow, lets go near the top and ZoroBot3 knocks them both off;
--meet: Ratatron makes it and ZoroBot3 catches it at the top. A slow-motion video (x10).

    ratasim loop out.mp4 [--meet]

Not as the robots are: their skids (0.3 mm over the floor) would scrape such a loop, so on the track only the tires
(as spheres) and loop skids 4 mm up touch; the hulls hit the rails and each other only. ZoroBot3 keeps its lane as
its firmware would (gyro, side sensors).
"""
import argparse
import math
import re

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from .. import config as C
from ..maze import Maze
from ..run import maze_path
from ..zorobot import config as Z
from . import common as K

SLOW, FPS, T_END = 0.1, 30, 1.0     # x10, the first simulated second
R = 0.25                            # the loop's radius at the top
K0, LT = 1 / R, 0.25                # curvature 0 -> 1/R over LT (clothoid), then 1/R, then down again
X0 = 1.90                           # the loop's bottom (m)
YC, LANE = -0.20, 0.11              # lanes: in at YC - LANE/2, out at YC + LANE/2 (south of the maze)
W, THK, RAIL_H, RAIL_T = 0.100, 0.004, 0.012, 0.003
Y_IN = YC - LANE / 2
RAT_X, ZORO_X = X0 - 0.45, X0 - 1.25    # starts
LOOP_SKID_GAP, LOOP_SKID_R = 0.004, 0.003
PRESETS = {     # Ratatron's launch (m/s, s), the hulls' damping ratio, ZoroBot3's motors off after the hit (s)
    'knock-off': dict(launch_v=4.0, launch_t=0.03, hull_dr=0.2, cut=0.3),
    'meet': dict(launch_v=5.5, launch_t=0.16, hull_dr=0.3, cut=0.3),
}


def loop_path(ds=0.008):
    """[(s, x, z)] along the riding surface from the entry (0, 0), and its length."""
    total = 2 * LT + (2 * math.pi - K0 * LT) / K0
    n = int(total / 0.0005)
    x = z = th = 0.0
    out, nxt = [], 0.0
    for i in range(n + 1):
        sv = i * total / n
        if sv >= nxt - 1e-9:
            out.append((sv, x, z))
            nxt += ds
        k = K0 * min(1.0, sv / LT, max(0.0, (total - sv) / LT))
        x += math.cos(th) * total / n
        z += math.sin(th) * total / n
        th += k * total / n
    out.append((total, x, z))
    return out, total


LOOP, LOOP_LEN = loop_path()
ALPHA = math.atan(LANE / LOOP_LEN)                  # the helix: a straight line on the unrolled loop
TOP_X = X0 + max(LOOP, key=lambda q: q[2])[1]
LOOP_PTS = [(X0 + x, Y_IN + LANE * sv / LOOP_LEN, z) for sv, x, z in LOOP]


def track_xml():
    out = []
    for (x1, y1, z1), (x2, y2, z2) in zip(LOOP_PTS, LOOP_PTS[1:]):
        ln = math.hypot(x2 - x1, z2 - z1)
        tx, tz = (x2 - x1) / ln, (z2 - z1) / ln
        nx, nz = -tz, tx                            # the riding side
        a = -math.atan2(tz, tx)
        q = f'{math.cos(a / 2):.6f} 0 {math.sin(a / 2):.6f} 0'
        cx, cy, cz = (x1 + x2) / 2, (y1 + y2) / 2, (z1 + z2) / 2
        half = ln / 2 + 0.0008
        out.append(f'<geom type="box" class="track" size="{half:.5f} {W / 2:.5f} {THK / 2:.5f}" quat="{q}" '
                   f'pos="{cx - nx * THK / 2:.5f} {cy:.5f} {cz - nz * THK / 2:.5f}" rgba="1 0.45 0.05 1"/>')
        for s in (-1, 1):
            out.append(f'<geom type="box" class="rail" size="{half:.5f} {RAIL_T / 2:.5f} {RAIL_H / 2:.5f}" quat="{q}" '
                       f'pos="{cx + nx * RAIL_H / 2:.5f} {cy + s * (W / 2 + RAIL_T / 2):.5f} {cz + nz * RAIL_H / 2:.5f}" '
                       f'rgba="0.85 0.3 0.02 1"/>')
    out.append(f'<geom type="box" contype="0" conaffinity="0" size="0.03 0.012 0.008" '
               f'pos="{RAT_X - 0.065:.4f} {Y_IN + (RAT_X - X0) * math.tan(ALPHA):.4f} 0.008" rgba="0.15 0.5 0.9 1"/>')  # the launcher
    return ''.join(out)


def loop_skids(prefix, floor_z, ys):
    return ''.join(f'<geom name="{prefix}loopskid_{i}" type="sphere" size="{LOOP_SKID_R}" contype="0" conaffinity="8" '
                   f'group="3" mass="0" friction="0.05 0.001 0.0001" pos="0 {y:.4f} {floor_z + LOOP_SKID_GAP + LOOP_SKID_R:.4f}"/>'
                   for i, y in enumerate(ys))


def scene(maze, hull_dr):
    """Collision bits: 1 floor, 2 robot, 4 hull-hull, 8 track (tires, loop skids), 16 rails (hulls)."""
    xml = K.scene_xml(maze, hull_solref=f'0.002 {hull_dr}')
    xml = re.sub(r'contype="6" (solref="[^"]*") conaffinity="1"', r'contype="22" conaffinity="16" \1', xml, count=1)
    xml = xml.replace('<default class="zhull"><geom group="3" contype="2" conaffinity="5"',
                      '<default class="zhull"><geom group="3" contype="18" conaffinity="20"')
    for cls in ('tire', 'ztire'):
        xml = xml.replace(f'<default class="{cls}"><geom group="3" contype="2" conaffinity="1"',
                          f'<default class="{cls}"><geom group="3" contype="2" conaffinity="9"')
    xml = re.sub(r'(</default>\s*<asset>)', lambda mm: (
        '<default class="track"><geom contype="8" conaffinity="8" friction="0.9 0.005 0.0001"/></default>'
        '<default class="rail"><geom contype="16" conaffinity="16" friction="0.1 0.005 0.0001"/></default>') + mm.group(1), xml, count=1)
    xml = re.sub(r'(<geom name="hull"[^>]*/>)', lambda mm: mm.group(1) + loop_skids('', C.GROUND_Z_MM * K.MM, (0.040, -0.025)), xml, count=1)
    xml = re.sub(r'(<geom name="z_hull"[^>]*/>)', lambda mm: mm.group(1) + loop_skids('z_', -Z.AXLE_HEIGHT_MM * K.MM + 0.001, (0.045, -0.036)), xml, count=1)
    xml = re.sub(r'(<geom name="(?:z_)?tire_[lr]"[^>]*?)type="cylinder" size="([\d.e-]+) [\d.e-]+"', r'\1type="sphere" size="\2"', xml)
    return xml.replace('</worldbody>', track_xml() + '</worldbody>', 1)


class LaneTracker:
    """ZoroBot3's offset from its lane (m to its left): along the loop (its progress only moves forward), else the
    floor's line."""

    def __init__(self, bot):
        self.bot, self.j = bot, 0

    def offset(self):
        x, y, z = self.bot.pos()
        if x > X0 - 0.02 or self.j > 0:
            lo, hi = self.j, min(len(LOOP_PTS), self.j + 40)
            jj = min(range(lo, hi), key=lambda q: (LOOP_PTS[q][0] - x) ** 2 + (LOOP_PTS[q][2] - z) ** 2)
            if (LOOP_PTS[jj][0] - x) ** 2 + (LOOP_PTS[jj][2] - z) ** 2 < 0.03 ** 2 and jj < len(LOOP_PTS) - 1:
                self.j = jj
                return y - LOOP_PTS[jj][1]
            if self.j >= len(LOOP_PTS) - 5:
                return y - (Y_IN + LANE + (x - X0 - LOOP[-1][1]) * math.tan(ALPHA))
        return y - (Y_IN + (x - X0) * math.tan(ALPHA))


def run(out, preset):
    p = PRESETS[preset]
    m, d = K.compile(scene(Maze.load(maze_path('oshwdem2026')), p['hull_dr']))
    rat, zoro = K.Bot(m, d, ''), K.Bot(m, d, 'z_')
    for bot, x in ((rat, RAT_X), (zoro, ZORO_X)):
        bot.place(x, Y_IN + (x - X0) * math.tan(ALPHA), ALPHA)
    rd, zd, lane = K.RatatronDrive(rat), K.ZoroDrive(zoro), LaneTracker(zoro)
    rd.engage(False)                    # free wheels: the launcher shoots it
    z_tires = {m.geom('z_tire_l').id, m.geom('z_tire_r').id}
    hull_r, hulls_z = m.geom('hull').id, {m.geom('z_hull').id, m.geom('z_hull_front').id}
    s = dict(contact=None, z_touch=-1.0, launched=False, off=None, doff=0.0)
    dt = m.opt.timestep

    def control(t):
        for c in d.contact[:d.ncon]:
            g = {c.geom1, c.geom2}
            if g & z_tires:
                s['z_touch'] = t
            if s['contact'] is None and hull_r in g and g & hulls_z:
                s['contact'] = t
                print(f'they hit at {t:.3f} s, {np.round(rat.pos(), 3)} m')
        if not s['launched'] and t >= p['launch_t']:
            s['launched'] = True
            d.qvel[rat.vadr:rat.vadr + 3] = (p['launch_v'] * math.cos(ALPHA), p['launch_v'] * math.sin(ALPHA), 0)
            for side in 'lr':
                d.qvel[rat.dof[side]] = p['launch_v'] / rd.r
        on = t >= 0 and (s['contact'] is None or t < s['contact'] + p['cut'])
        off = lane.offset()
        s['doff'] = 0.8 * s['doff'] + 0.2 * (off - (s['off'] if s['off'] is not None else off)) / dt
        s['off'] = off
        dv = max(-0.6, min(0.6, -40.0 * off - 1.2 * s['doff']))
        v = K.haki(t)
        if on:
            zd.drive(v - dv, v + dv)
        else:
            zd.off()
        zd.fan(K.HAKI['fan'] if t - s['z_touch'] < 0.03 else 0.0)    # the suction holds it to what its tires touch

    mujoco.mj_forward(m, d)
    t = -0.3
    while t < 0:
        control(t)
        mujoco.mj_step(m, d)
        t += dt
    video = K.Video(out)
    ren, cam = mujoco.Renderer(m, video.h, video.w), mujoco.MjvCamera()
    cam.lookat[:] = (TOP_X - 0.25, YC, 0.27)
    cam.distance, cam.elevation, cam.azimuth = 1.0, -8, 100
    big, small = K.font(30), K.font(22, bold=False)
    next_frame = 0.0
    while t < T_END:
        control(t)
        mujoco.mj_step(m, d)
        t += dt
        if t >= next_frame:
            next_frame += SLOW / FPS
            ren.update_scene(d, cam)
            img = Image.fromarray(ren.render())
            g = ImageDraw.Draw(img)
            g.rectangle((0, 0, video.w, 50), fill=(0, 0, 0))
            g.text((16, 9), f'Ratatron ({C.MASS_KG * 1000:.0f} g) y ZoroBot3 (70 g)  -  looping', font=big, fill=(255, 255, 255))
            g.rectangle((0, video.h - 44, video.w, video.h), fill=(0, 0, 0))
            g.text((16, video.h - 36), f'Ratatron {rat.speed():.1f} m/s (lanzado)', font=small, fill=(120, 200, 255))
            g.text((video.w // 2 - 120, video.h - 36), f'camara lenta x10   t = {t * 1000:4.0f} ms', font=small, fill=(255, 220, 60))
            g.text((video.w - 345, video.h - 36), f'ZoroBot3 {zoro.speed():.1f} m/s (ventilador)', font=small, fill=(200, 140, 255))
            video.write(img)
    video.close()


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim loop', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out', help='the mp4')
    p.add_argument('--meet', action='store_true', help='Ratatron makes it; ZoroBot3 catches it at the top')
    a = p.parse_args(argv)
    run(a.out, 'meet' if a.meet else 'knock-off')
    print('->', a.out)


if __name__ == '__main__':
    main()
