"""Drag race: the robots on a straight strip, one lane each, from the start lights to a checkered finish (no
firmware: each robot's own speed profile from its firmware, lane keeping). A video: the lights in real time, the
fastest robot's run in slow motion (x4), the rest in real time; a finish camera inset and the times.

    ratasim drag-race out.mp4 [--robots all] [--length 3]       # --robots ratatron,zorobot,fujin: any, by name
    ratasim drag-race --still out.png       # the robots at the start line

Their profiles (ROBOTS): Ratatron's race preset, ZoroBot3's SPEED_HAKI (and its fan), Fujin's "super" forward
(movement_params.cpp, its fan on), UKMARSBOT's FAST_RUN_SPEED_MAX and FAST_RUN_ACCELERATION (ORION), Bulebule's
LINEAR_SPEED_LIMIT at 2 F / MOUSE_MASS (its runner's 0.25 N), Futura's 2 m/s at 6 m/s^2, Perenquen's traj_speed
1000 at its ACC_DIST, Gamusino's 1.6 m/s at 3 m/s^2, NezumiBot's 40 % PWM (velBase: no profile, its motors' step),
Miniskybot's servos at full (its program's, ratasim's: 0.14 m/s), KERISE v1's fast_speed (MoveAction.h: 1000 mm/s
unless its menu picks another) at its acceleration()'s 9000 mm/s^2, braking at its deceleration()'s 6000.
Their motors: Ratatron's and ZoroBot3's models, the others a speed servo as quick as Ratatron's (WheelServo).
"""
import argparse
import os

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from .. import paths
from ..maze import Maze
from ..run import maze_path
from . import common as K

LANE_W = 0.22
RUNOFF = 1.4


def ramp(v, a):
    """From rest at t = 0: a (m/s^2) up to v (m/s)."""
    return lambda t: min(a * t, v) if t > 0 else 0.0


NEZUMI_V = 0.40 * K.NC.BATTERY_V / K.NC.MOTOR_KE / K.NC.GEAR * K.NC.WHEEL_RADIUS_MM * K.MM     # 0.54 m/s free
ROBOTS = {      # speed profile, brake after the line (m/s^2), fan (N down, or ZoroBot3's duty)
    'ratatron': dict(profile=K.ratatron_race, brake=2.0),
    'zorobot': dict(profile=K.haki, brake=30.0, fan=K.HAKI['fan']),
    'gamusino': dict(profile=ramp(1.6, 3.0), brake=3.0),
    'bulebule': dict(profile=ramp(2.0, 2 * 0.25 / 0.11), brake=2 * 0.25 / 0.11),
    'nezumibot': dict(profile=ramp(NEZUMI_V, 10.0), brake=5.0),
    'perenquen': dict(profile=ramp(1.96, 5.9), brake=5.9),
    'fujin': dict(profile=ramp(6.5, 35.0), brake=35.0, fan=K.FC.FAN_DOWNFORCE_N),
    'futura': dict(profile=ramp(2.0, 6.0), brake=6.0),
    'ukmarsbot': dict(profile=ramp(2.5, 3.0), brake=3.0),
    'miniskybot': dict(profile=ramp(K.MC.SERVO_FREE_RAD_S * K.MC.WHEEL_RADIUS_MM * K.MM, 2.0), brake=2.0),
    'kerise': dict(profile=ramp(1.0, 9.0), brake=6.0),
}
for _k, _spec in ROBOTS.items():
    _spec.update(name=K.ROBOTS[_k]['name'], colour=K.ROBOTS[_k]['colour'], prefix=K.ROBOTS[_k]['prefix'])
TEX = os.path.join(paths.SIM_BUILD, 'scenes')


def textures(keys):
    """The banners and floor letters, drawn into build/scenes."""
    os.makedirs(os.path.join(TEX, 'mesh'), exist_ok=True)

    def text(name, s, size, bg, fg, checker=False):
        im = Image.new('RGB', size, bg)
        g = ImageDraw.Draw(im)
        if checker:
            q = size[1] // 6
            for i in range(0, size[0], q):
                for j in (0, size[1] - q):
                    g.rectangle((i, j, i + q - 1, j + q - 1), fill=(240, 240, 240) if (i // q + j // q) % 2 == 0 else (15, 15, 15))
        f = K.font(int(size[1] * (0.5 if size[0] > size[1] * 2 else 0.47)))
        w, h = g.textbbox((0, 0), s, font=f)[2:]
        g.text(((size[0] - w) / 2, (size[1] - h) / 2 - size[1] * 0.06), s, font=f, fill=fg)
        im.save(os.path.join(TEX, name))
    text('meta.png', 'META', (1024, 192), (200, 20, 20), (255, 255, 255), checker=True)
    text('salida.png', 'SALIDA', (1024, 192), (20, 60, 160), (255, 255, 255))
    for k in keys:
        text(f'lane_{k}.png', ROBOTS[k]['name'].upper(), (1024, 256), (30, 30, 32), (235, 235, 235))
    for n in (1, 2):
        text(f'mark_{n}.png', f'{n} m', (256, 256), (30, 30, 32), (235, 235, 235))


def quad(path, w, h, plane):
    """An OBJ quad w x h (m) centred on the origin with UVs: 'xz' faces -y (a banner), 'xy' faces +z (a decal)."""
    a, b = w / 2, h / 2
    v = [(-a, 0, -b), (a, 0, -b), (a, 0, b), (-a, 0, b)] if plane == 'xz' else [(-a, -b, 0), (a, -b, 0), (a, b, 0), (-a, b, 0)]
    f = 'f 1/1 2/2 3/3\nf 1/1 3/3 4/4\n' + ('f 1/1 3/3 2/2\nf 1/1 4/4 3/3\n' if plane == 'xz' else '')
    with open(path, 'w') as fh:
        fh.writelines(f'v {x} {y} {z}\n' for x, y, z in v)
        fh.write('vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\n' + f)
    return path


def scenery(lanes, length):
    """(assets, world geoms, start-light names) of the strip: lanes, start line, checkered finish, gantries, flags."""
    assets = ['<texture name="chk" type="2d" builtin="checker" rgb1="0.95 0.95 0.95" rgb2="0.05 0.05 0.05" width="256" height="256"/>',
              '<material name="chk_line" texture="chk" texuniform="true" texrepeat="20 20"/>',
              '<material name="chk_flag" texture="chk" texuniform="true" texrepeat="40 40"/>',
              '<material name="asphalt" rgba="0.12 0.12 0.13 1" specular="0.05" shininess="0.05"/>',
              '<material name="concrete" rgba="0.32 0.33 0.34 1" specular="0.05"/>']
    world = ['<geom name="floor" type="plane" size="8 8 0.1" pos="1.5 0 0" material="concrete" condim="3" friction="0.05 0.001 0.0001"/>']

    def tex_quad(name, png, w, h, plane, pos, euler='0 0 0'):
        assets.append(f'<texture name="t_{name}" type="2d" file="{os.path.join(TEX, png)}"/>'
                      f'<material name="m_{name}" texture="t_{name}" specular="0.1"/>'
                      f'<mesh name="q_{name}" inertia="shell" file="{quad(os.path.join(TEX, "mesh", name + ".obj"), w, h, plane)}"/>')
        world.append(f'<geom type="mesh" mesh="q_{name}" material="m_{name}" contype="0" conaffinity="0" mass="0" '
                     f'pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" euler="{euler}"/>')

    def box(pos, half, rgba=None, material=None, euler='0 0 0'):
        look = f'material="{material}"' if material else f'rgba="{rgba}"'
        world.append(f'<geom type="box" {look} contype="0" conaffinity="0" pos="{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f}" '
                     f'size="{half[0]:.4f} {half[1]:.4f} {half[2]:.4f}" euler="{euler}"/>')

    def post(x, y, r, h, rgba):
        world.append(f'<geom type="cylinder" rgba="{rgba}" contype="0" conaffinity="0" pos="{x:.4f} {y:.4f} {h / 2:.4f}" '
                     f'size="{r:.4f} {h / 2:.4f}"/>')

    n = len(lanes)
    half_w = n * LANE_W / 2
    x0, x1 = -0.6, length + RUNOFF
    white, grey = '0.93 0.93 0.93 1', '0.85 0.85 0.88 1'
    box(((x0 + x1) / 2, 0, 0.0001), ((x1 - x0) / 2, half_w + 0.02, 0.0001), material='asphalt')
    for s in (-1, 1):
        box(((x0 + x1) / 2, s * half_w, 0.00025), ((x1 - x0) / 2, 0.006, 0.0001), white)
    for i in range(1, n):
        for x in np.arange(0.05, x1, 0.20):
            box((x + 0.05, -half_w + i * LANE_W, 0.00025), (0.05, 0.004, 0.0001), white)
    box((0.0, 0, 0.0003), (0.012, half_w, 0.0001), white)
    box((length + 0.025, 0, 0.0003), (0.025, half_w, 0.0002), material='chk_line')
    for k, y in lanes.items():
        dw = min(0.16, 0.85 * LANE_W)
        tex_quad(f'lane_{k}', f'lane_{k}.png', dw, dw / 4, 'xy', (-0.30, y, 0.0003), '0 0 90')
    for mk in (1, 2):
        tex_quad(f'mark{mk}', f'mark_{mk}.png', 0.06, 0.06, 'xy', (mk, -half_w - 0.06, 0.0003), '0 0 90')
    for name, x in (('salida', -0.02), ('meta', length + 0.025)):      # one post on the far side, a cantilever
        py = half_w + 0.06
        post(x, py, 0.014, 0.44, grey)
        box((x, (py - half_w - 0.03) / 2, 0.43), (0.015, (py + half_w + 0.03) / 2, 0.012), grey)
        tex_quad(name, f'{name}.png', 2 * half_w, 2 * half_w * 192 / 1024, 'xz', (x, 0, 0.36), '0 0 90')
    for k, px in enumerate((length + 0.12, length + 0.30)):              # checkered flags beyond the finish
        py = half_w + 0.10 + 0.04 * k
        post(px, py, 0.006, 0.40, '0.75 0.75 0.78 1')
        box((px + 0.065, py, 0.35), (0.065, 0.002, 0.045), material='chk_flag', euler=f'0 0 {-15 + 30 * k}')
    lx, ly = -0.08, -half_w - 0.12                                       # the start lights: three red, one green
    post(lx, ly, 0.008, 0.30, '0.2 0.2 0.22 1')
    box((lx, ly, 0.27), (0.02, 0.02, 0.075), '0.08 0.08 0.09 1')
    lights = []
    for i in range(4):
        assets.append(f'<material name="light{i}" rgba="0.15 0.04 0.04 1" emission="0"/>')
        world.append(f'<geom name="light{i}" type="sphere" material="light{i}" size="0.012" contype="0" conaffinity="0" '
                     f'pos="{lx:.4f} {ly - 0.02:.4f} {0.29 - i * 0.034:.4f}"/>')
        lights.append(f'light{i}')
    world.append('<light directional="true" pos="1.5 -1.5 3" dir="-0.2 0.5 -1" diffuse="0.75 0.75 0.75" castshadow="true"/>'
                 '<light directional="true" pos="1.5 2 3" dir="0 -0.4 -1" diffuse="0.3 0.3 0.3" castshadow="false"/>')
    return ''.join(assets), ''.join(world), lights


def nose(bot):
    """How far its front is ahead of its body's origin (m), facing +x: the farthest of its geoms' boxes."""
    m, d = bot.m, bot.d
    bot.place(0.0, 0.0, 0.0)
    mujoco.mj_kinematics(m, d)
    x0, far = float(d.xpos[bot.body][0]), -1.0
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    for g in range(m.ngeom):
        if m.body_rootid[m.geom_bodyid[g]] != bot.body or m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        pts = d.geom_xpos[g] + (c + corners * h) @ d.geom_xmat[g].reshape(3, 3).T
        far = max(far, float(pts[:, 0].max()))
    return far - x0


class Racer(K.Bot):
    def __init__(self, m, d, key, y):
        spec = ROBOTS[key]
        super().__init__(m, d, spec['prefix'])
        self.key, self.spec, self.y = key, spec, y
        self.finish_t = self.finish_v = None
        self.nose = nose(self)
        self.place(-self.nose - 0.004, y, 0.0)

    def front(self):
        return float(self.pos()[0]) + self.nose

    def v_ref(self, t):
        if self.finish_t is not None:
            return max(0.0, self.finish_v - self.spec['brake'] * (t - self.finish_t))
        return self.spec['profile'](t)


def build(length, keys):
    lanes = {k: (i - (len(keys) - 1) / 2) * LANE_W for i, k in enumerate(keys)}
    textures(keys)
    assets, world, lights = scenery(lanes, length)
    xml = K.scene_of(Maze.load(maze_path('oshwdem2026')), keys, with_maze=False)
    xml = xml.replace('</asset>', assets + '</asset>', 1).replace('</worldbody>', world + '</worldbody>', 1)
    m, d = K.compile(xml)
    racers = [Racer(m, d, k, lanes[k]) for k in keys]
    K.park(m, d, keys)
    for r in racers:
        r.drive = (K.RatatronDrive(r) if r.key == 'ratatron' else K.ZoroDrive(r) if r.key == 'zorobot'
                   else K.WheelServo(r, r.key))
    return m, d, racers, lights


def control(m, d, racers, lights, t, length):
    for r in racers:
        if r.finish_t is None and t > 0 and r.front() >= length:
            r.finish_t, r.finish_v = t, r.v_ref(t)
            print(f'{r.spec["name"]:9s} {t:.3f} s, {r.speed():.2f} m/s at the line')
        r.drive.drive(*K.lane_keep(r, r.v_ref(t), 0.0, float(r.pos()[1]) - r.y))
        if r.key == 'zorobot':
            r.drive.fan(r.spec['fan'], down=(0, 0, -1))
        elif r.spec.get('fan'):
            d.xfrc_applied[r.body, :3] = (0, 0, -r.spec['fan'])
    for i, name in enumerate(lights):           # three reds come on, then out and green
        mid = m.material(name).id
        on = (-1.2 + 0.4 * i <= t < 0) if i < 3 else t >= 0
        m.mat_rgba[mid] = ((1, 0.08, 0.05, 1) if on else (0.15, 0.04, 0.04, 1)) if i < 3 else ((0.1, 1, 0.2, 1) if on else (0.03, 0.12, 0.04, 1))
        m.mat_emission[mid] = 0.9 if on else 0.0


def lane_width(keys):
    return 0.22 if len(keys) <= 3 else 0.17


def still(path, keys):
    global LANE_W
    LANE_W = 0.13 if len(keys) <= 3 else 0.15
    m, d, racers, lights = build(3.0, keys)
    t = -2.0
    while t < -1.6:
        control(m, d, racers, lights, t, 3.0)
        mujoco.mj_step(m, d)
        t += m.opt.timestep
    ren, cam = mujoco.Renderer(m, 1080, 1920), mujoco.MjvCamera()
    cam.lookat[:] = (-0.05, 0.0, 0.015)
    cam.distance, cam.elevation, cam.azimuth = (0.36, -26, 215) if len(keys) <= 3 else (0.75 * len(keys) * LANE_W, -42, 205)
    ren.update_scene(d, cam)
    Image.fromarray(ren.render()).save(path)


def race(out, length, keys):
    global LANE_W
    LANE_W = lane_width(keys)
    width = max(1.0, 0.6 * len(keys) * LANE_W / 0.44)     # the cameras farther for a wider strip (two lanes: 1)
    m, d, racers, lights = build(length, keys)
    dt, t = m.opt.timestep, -1.9
    while t < -1.6:
        control(m, d, racers, lights, t, length)
        mujoco.mj_step(m, d)
        t += dt
    video = K.Video(out)
    w, h, pw, ph = video.w, video.h, 448, 252
    ren, pip = mujoco.Renderer(m, h, w), mujoco.Renderer(m, ph, pw)
    cam, fcam = mujoco.MjvCamera(), mujoco.MjvCamera()
    fcam.lookat[:] = (length + 0.02, 0.0, 0.10)
    fcam.distance, fcam.elevation, fcam.azimuth = 1.05 * width, -14, 75
    big, mid_f, small = K.font(30), K.font(22), K.font(20, bold=False)
    fastest = max(racers, key=lambda r: r.v_ref(10.0))
    slow_until = 0.75 * length / 3.0
    look = dist = shot = None
    next_frame, end = -1.6, None
    while end is None or t < end:
        control(m, d, racers, lights, t, length)
        mujoco.mj_step(m, d)
        t += dt
        if end is None and all(r.finish_t is not None for r in racers):
            end = t + 0.8
        if t < next_frame:
            continue
        speed = 4.0 if 0 <= t < slow_until else 1.0         # video seconds per simulated second
        next_frame += 1.0 / 30 / speed
        if t < 0:
            new, target, dd, el, az = 'start', np.array([0.0, 0.0, 0.04]), 0.95 * width, -20, 125
        elif fastest.finish_t is None or t < fastest.finish_t + 0.2:
            new, target, dd, el, az = 'fastest', np.array([max(0.15, float(fastest.pos()[0])), fastest.y - 0.05, 0.03]), 0.62, -16, 108
        else:
            pending = [r for r in racers if r is not fastest and (r.finish_t is None or t - r.finish_t < 0.35)]
            xs = [float(r.pos()[0]) for r in (pending or racers)]
            ys = [r.y for r in (pending or racers)]
            new = 'pack'
            target = np.array([(min(xs) + max(xs)) / 2, float(np.mean(ys)) - 0.05, 0.03])
            dd, el, az = max(0.8, 0.8 * (max(xs) - min(xs)) + 0.55, 1.3 * (max(ys) - min(ys)) + 0.5), -22, 105
        if new != shot:
            look = dist = None                               # a cut
            shot = new
        look = target if look is None else 0.85 * look + 0.15 * target
        dist = dd if dist is None else 0.85 * dist + 0.15 * dd
        cam.lookat[:] = look
        cam.distance, cam.elevation, cam.azimuth = dist, el, az
        ren.update_scene(d, cam)
        img = Image.fromarray(ren.render())
        pip.update_scene(d, fcam)
        img.paste(Image.fromarray(pip.render()), (w - pw - 10, 60))
        g = ImageDraw.Draw(img)
        g.rectangle((w - pw - 12, 58, w - 8, 62 + ph), outline=(255, 255, 255), width=2)
        g.rectangle((w - pw - 10, 60 + ph - 26, w - pw + 150, 60 + ph), fill=(0, 0, 0))
        g.text((w - pw - 2, 60 + ph - 25), 'cámara de meta', font=small, fill=(255, 255, 255))
        g.rectangle((0, 0, w, 50), fill=(0, 0, 0))
        g.text((16, 9), f'DRAG RACE  ·  {length:g} m', font=big, fill=(255, 255, 255))
        g.text((w // 2 - 60, 12), 'preparados...' if t < 0 else f'{t:5.2f} s', font=mid_f, fill=(255, 220, 60))
        g.text((w - 230, 14), '' if t < 0 else ('cámara lenta x4' if speed > 1 else 'tiempo real'), font=small, fill=(255, 220, 60))
        order = sorted(racers, key=lambda r: (r.finish_t is None, r.finish_t or 0, -r.front()))
        g.rectangle((10, 60, 350, 70 + 36 * len(racers)), fill=(0, 0, 0))
        for i, r in enumerate(order):
            res = f'{r.finish_t:.2f} s' if r.finish_t is not None else f'{max(0.0, r.speed()):.1f} m/s'
            g.text((20, 68 + 36 * i), f'{i + 1}. {r.spec["name"]}', font=mid_f, fill=r.spec['colour'])
            g.text((250, 68 + 36 * i), res, font=mid_f, fill=(255, 255, 255) if r.finish_t else (180, 180, 180))
        video.write(img)
    video.close()


def main(argv=None):
    p = argparse.ArgumentParser(prog='ratasim drag-race', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('out', nargs='?', help='the mp4')
    p.add_argument('--length', type=float, default=3.0, help='start to finish, m')
    p.add_argument('--robots', default='all', help=f'which, by name, in their lanes\' order: {",".join(ROBOTS)} '
                   '(default all)')
    p.add_argument('--still', metavar='PNG', help='only a picture of the robots at the start line')
    a = p.parse_args(argv)
    try:
        keys = K.robot_names(a.robots)
    except ValueError as e:
        p.error(str(e))
    if a.still:
        still(a.still, keys)
        print('->', a.still)
    elif a.out:
        race(a.out, a.length, keys)
        print('->', a.out)
    else:
        p.error('give the mp4, or --still PNG')


if __name__ == '__main__':
    main()
