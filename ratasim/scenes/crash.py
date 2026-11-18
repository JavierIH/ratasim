"""Ratatron and ZoroBot3 head-on on the maze's longest straight, each at its own top speed (no firmware: their speed
profiles, lane keeping); with --ramp, a ramp where they meet throws them up. A slow-motion video (x10).

    ratasim crash out.mp4 [--maze oshwdem2026] [--ramp] [--rat-mass 0.2]
"""
import argparse
import math

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from ..maze import Maze, N, E
from ..run import maze_path
from . import common as K

SLOW, FPS = 0.1, 30         # x10
T0, Z_DELAY = 0.30, 0.40    # the video's start; ZoroBot3 starts later: both at top speed when they meet
RAMP_H, RAMP_L = 0.015, 0.09    # the ramp: two slopes, this high, each this long (m)


def longest_straight(maze):
    """(cells, horizontal, row or column, first, last)."""
    best = (0,)
    for horizontal in (True, False):
        for k in range(maze.size):
            i = 0
            while i < maze.size:
                j = i
                while j + 1 < maze.size and not (maze.has(j, k, E) if horizontal else maze.has(k, j, N)):
                    j += 1
                if j - i + 1 > best[0]:
                    best = (j - i + 1, horizontal, k, i, j)
                i = j + 1
    return best


def simulate(maze, rat_mass, ramp_at=None, video=None):
    """Runs the scene; the meeting point (x, y) of the hulls' first contact."""
    _, horizontal, k, i0, i1 = longest_straight(maze)
    cell = K.C.CELL_MM * K.MM

    def centre(i):
        return np.array([(i + 0.5) * cell, (k + 0.5) * cell]) if horizontal else np.array([(k + 0.5) * cell, (i + 0.5) * cell])
    a, b = centre(i0), centre(i1)
    xml = K.scene_xml(maze, rat_mass=rat_mass, hull_solref='0.002 0.3')
    if ramp_at is not None:
        verts = ' '.join(f'{x:.4f} {y:.4f} {z:.4f}' for x, z in ((-RAMP_L, 0), (RAMP_L, 0), (0, RAMP_H)) for y in (-0.075, 0.075))
        along = ramp_at[0] if horizontal else ramp_at[1]
        pos = (along, (k + 0.5) * cell) if horizontal else ((k + 0.5) * cell, along)
        xml = xml.replace('</asset>', f'<mesh name="ramp" vertex="{verts}"/></asset>', 1)
        xml = xml.replace('</worldbody>', f'<geom name="ramp" type="mesh" mesh="ramp" pos="{pos[0]:.4f} {pos[1]:.4f} 0" '
                          f'euler="0 0 {0 if horizontal else 90}" rgba="0.62 0.5 0.33 1" friction="0.8"/></worldbody>', 1)
    m, d = K.compile(xml)
    rat, zoro = K.Bot(m, d, ''), K.Bot(m, d, 'z_')
    rd, zd = K.RatatronDrive(rat), K.ZoroDrive(zoro)
    f = (b - a) / np.linalg.norm(b - a)
    for bot, start, fwd in ((rat, a, f), (zoro, b, -f)):
        bot.place(*start, math.atan2(fwd[1], fwd[0]))
        bot.start, bot.fwd = start, fwd
    hulls_z = {m.geom('z_hull').id, m.geom('z_hull_front').id}
    hull_r = m.geom('hull').id
    s = dict(contact=None, meet=None)

    def keep(bot, v):
        left = np.array([-bot.fwd[1], bot.fwd[0]])
        return K.lane_keep(bot, v, math.atan2(bot.fwd[1], bot.fwd[0]), float(np.dot(bot.pos()[:2] - bot.start, left)))

    def control(t):
        for c in d.contact[:d.ncon]:
            if s['contact'] is None and hull_r in (c.geom1, c.geom2) and {c.geom1, c.geom2} & hulls_z:
                s['contact'], s['meet'] = t, (rat.pos()[:2] + zoro.pos()[:2]) / 2
        on = s['contact'] is None or t < s['contact'] + 0.25      # their firmwares stop the motors once stuck
        rd.drive(*(keep(rat, K.ratatron_race(t)) if on else (0.0, 0.0)))
        if on:
            zd.drive(*keep(zoro, K.haki(t - Z_DELAY)))
        else:
            zd.off()
        planted = zoro.tilt_deg() < 10 and zoro.pos()[2] < s['z_rest'] + 0.003
        zd.fan(K.HAKI['fan'] if on and planted and t > Z_DELAY - 0.05 else 0.0, down=(0, 0, -1))

    mujoco.mj_forward(m, d)
    dt, t = m.opt.timestep, -0.4
    s['z_rest'] = 1.0
    while t < -0.1:                 # settle
        mujoco.mj_step(m, d)
        t += dt
    s['z_rest'] = float(zoro.pos()[2])
    ren = cam = None
    if video:
        ren, cam = mujoco.Renderer(m, video.h, video.w), mujoco.MjvCamera()
        big, small = K.font(30), K.font(22, bold=False)
    look = dist = lz = None
    next_frame = T0
    while t < T0 + 10 * SLOW:
        control(t)
        mujoco.mj_step(m, d)
        t += dt
        if video and t >= next_frame:
            next_frame += SLOW / FPS
            pr, pz = rat.pos(), zoro.pos()
            target = (pr[:2] + pz[:2]) / 2
            dd = max(0.40, 1.05 * float(np.linalg.norm(pz[:2] - pr[:2])))
            lift = max(0.0, (pr[2] + pz[2]) / 2 - 0.01)
            look = target if look is None else 0.85 * look + 0.15 * target
            dist = dd if dist is None else 0.85 * dist + 0.15 * dd
            lz = 0.015 + lift if lz is None else 0.85 * lz + 0.15 * (0.015 + lift)
            cam.lookat[:] = (look[0], look[1], lz)
            cam.distance, cam.elevation = dist, (-30 if ramp_at is not None else -52)
            cam.azimuth = math.degrees(math.atan2(-f[0], f[1])) + 180
            ren.update_scene(d, cam)
            img = Image.fromarray(ren.render())
            g = ImageDraw.Draw(img)
            g.rectangle((0, 0, video.w, 50), fill=(0, 0, 0))
            title = f'Ratatron ({rat_mass * 1000:.0f} g) vs ZoroBot3 (70 g)  -  choque frontal' + (' en rampa' if ramp_at is not None else '')
            g.text((16, 9), title, font=big, fill=(255, 255, 255))
            g.rectangle((0, video.h - 44, video.w, video.h), fill=(0, 0, 0))
            g.text((16, video.h - 36), f'Ratatron {rat.speed():.1f} m/s', font=small, fill=(120, 200, 255))
            g.text((video.w // 2 - 150, video.h - 36), f'camara lenta x10   t = {(t - T0) * 1000:4.0f} ms', font=small, fill=(255, 220, 60))
            g.text((video.w - 260, video.h - 36), f'ZoroBot3 {zoro.speed():.1f} m/s', font=small, fill=(200, 140, 255))
            video.write(img)
    return s['meet']


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim crash', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out', help='the mp4')
    p.add_argument('--maze', default='oshwdem2026')
    p.add_argument('--ramp', action='store_true', help='a ramp where they meet (found by a first run without it)')
    p.add_argument('--rat-mass', type=float, default=0.200, help='Ratatron, kg')
    a = p.parse_args(argv)
    maze = Maze.load(maze_path(a.maze))
    ramp_at = simulate(maze, a.rat_mass) if a.ramp else None
    if a.ramp and ramp_at is None:
        raise SystemExit('they never met')
    video = K.Video(a.out)
    meet = simulate(maze, a.rat_mass, ramp_at, video)
    video.close()
    print('they met at', None if meet is None else np.round(meet, 3), '->', a.out)


if __name__ == '__main__':
    main()
