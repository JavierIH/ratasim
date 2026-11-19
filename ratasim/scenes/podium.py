"""The podium after a minisumo bout: its first three (or two) on a classic podium, the 1st in the middle and highest,
the 2nd on its left, the 3rd on its right, standing still (no firmware: their models as they are, settled on their
blocks); the camera on each from the 3rd to the 1st, then the whole podium and fireworks.

The fireworks are emissive spheres added to the rendered scene (not to the model: they touch nothing): rockets rise
from behind the podium and burst into sparks thrown evenly round a sphere, on ballistic paths with air drag, each
with a short streak, fading as they fall. Their launches, bursts and colours come from a fixed seed: the same
video every run.
"""
import math

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from ..maze import Maze
from ..run import maze_path
from . import common as K

# The blocks (m): their tops' heights by place, their width and depth; the 2nd on the 1st's left (the camera's).
BLOCK_W, BLOCK_D = 0.15, 0.13
BLOCK_H = {1: 0.090, 2: 0.060, 3: 0.040}
BLOCK_X = {1: 0.0, 2: -BLOCK_W, 3: BLOCK_W}
BLOCK_RGBA = {1: '0.72 0.55 0.14 1', 2: '0.55 0.57 0.61 1', 3: '0.55 0.31 0.15 1'}
MEDAL = {1: (255, 210, 60), 2: (215, 220, 230), 3: (225, 145, 80)}
SKY = 'rgb1="0.05 0.06 0.10" rgb2="0.0 0.0 0.01"'           # a night sky, for the fireworks

# The shots: (lookat, distance, azimuth, elevation); the camera at -y looking at the podium's front (+y).
WIDE = ((0.0, 0.0, 0.09), 0.68, 90.0, -12.0)
FIRE = ((0.0, 0.2, 0.27), 1.12, 90.0, -5.0)


def scenery(places=(1, 2, 3)):
    """(assets, world geoms): a dark hall, the blocks of places, lights on them."""
    assets = '<material name="podium_floor" rgba="0.06 0.06 0.075 1" specular="0.1"/>' + ''.join(
        f'<material name="podium_{p}" rgba="{rgba}" specular="0.3" shininess="0.4"/>' for p, rgba in BLOCK_RGBA.items())
    world = ['<geom name="floor" type="plane" size="6 6 0.1" material="podium_floor" condim="3" '
             'friction="0.8 0.005 0.0001"/>']
    for p in places:
        h = BLOCK_H[p]
        world.append(f'<geom name="podium_{p}" type="box" size="{BLOCK_W / 2 - 0.001} {BLOCK_D / 2} {h / 2}" '
                     f'pos="{BLOCK_X[p]} 0 {h / 2}" material="podium_{p}" '
                     'condim="3" friction="0.8 0.005 0.0001"/>')
    world.append('<light pos="0 -0.6 0.9" dir="0 0.55 -0.83" diffuse="0.5 0.5 0.47" cutoff="40" exponent="4" '
                 'castshadow="true"/>'
                 '<light directional="true" pos="0 1 2" dir="0 -0.4 -1" diffuse="0.25 0.25 0.3" castshadow="false"/>'
                 '<light directional="true" pos="0 -1 2" dir="0 0.6 -0.5" diffuse="0.25 0.25 0.25" '
                 'castshadow="false"/>')
    return assets, ''.join(world)


def top(m, d, bot):
    """The height of the robot's highest point (m), settled."""
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    z = 0.0
    for g in range(m.ngeom):
        if m.body_rootid[m.geom_bodyid[g]] != bot.body or m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        z = max(z, float((d.geom_xpos[g] + (c + corners * h) @ d.geom_xmat[g].reshape(3, 3).T)[:, 2].max()))
    return z


def ranked(bots):
    """[(place, bot)] of the first three (or as many as there are), by place."""
    return sorted(((b.place_no, b) for b in bots if b.place_no and b.place_no <= 3), key=lambda pb: pb[0])


class Podium:
    def __init__(self, podium, w, h):
        """podium: [(place, key, name, colour)], 1st first."""
        self.podium, self.w, self.h = podium, w, h
        keys = [k for _, k, _, _ in podium]
        xml = K.scene_of(Maze.load(maze_path('oshwdem2026')), keys, with_maze=False)
        sky = '<texture name="sky" type="skybox" builtin="gradient" '
        xml = xml.replace(sky + 'rgb1="0.25 0.27 0.3" rgb2="0.05 0.05 0.06"', sky + SKY)
        assets, world = scenery([p for p, *_ in podium])
        xml = xml.replace('</asset>', assets + '</asset>', 1).replace('</worldbody>', world + '</worldbody>', 1)
        self.m, self.d = K.compile(xml)
        self.bots = {}
        for p, k, _, _ in podium:
            b = K.Bot(self.m, self.d, K.ROBOTS[k]['prefix'])
            b.place(BLOCK_X[p], 0.0, -math.pi / 2)              # facing the camera
            self.d.qpos[b.qadr + 2] += BLOCK_H[p] + 0.002
            self.bots[p] = b
        K.park(self.m, self.d, keys)
        for b in self.bots.values():                            # still: their wheels held
            for s in 'lr':
                self.m.dof_damping[b.dof[s]] = 1.0
        for _ in range(int(1.0 / self.m.opt.timestep)):         # a second to settle on their blocks
            mujoco.mj_step(self.m, self.d)
        self.cx = float(np.mean([BLOCK_X[p] for p in self.bots]))
        self.top = {p: top(self.m, self.d, b) for p, b in self.bots.items()}
        self.ren, self.cam = mujoco.Renderer(self.m, h, w, max_geom=20000), mujoco.MjvCamera()
        s = h / 720
        self.s = s
        self.big, self.mid, self.tag, self.huge = (K.font(int(n * s)) for n in (30, 26, 18, 56))
        self.num = K.font(int(40 * s))
        self.fireworks = Fireworks(colours=[c for *_, c in podium])

    def shot(self, p):
        """The close shot of place p's robot: its front, a little from above."""
        b = self.bots[p]
        x, y, z = (float(v) for v in b.pos())
        return (x, y, z - 0.012), 0.36, 90.0, -13.0

    def whole(self, view):
        """WIDE or FIRE on the blocks there are (two: between them)."""
        (x, y, z), dist, az, el = view
        return (x + self.cx, y, z), dist, az, el

    def look(self, view):
        (la, dist, az, el) = view
        self.cam.lookat[:] = la
        self.cam.distance, self.cam.azimuth, self.cam.elevation = dist, az, el

    def glow(self, img, sparks):
        """The sparks' halo: each drawn as a disc on a black layer, blurred, added to the picture."""
        layer = Image.new('RGB', (self.w, self.h))
        g = ImageDraw.Draw(layer)
        f = self.h / 2 / math.tan(math.radians(self.m.vis.global_.fovy) / 2)
        for p, r, (cr, cg, cb, a) in sparks:
            q = K.project(self.m, self.ren, self.w, self.h, p)
            if q is None:
                continue
            rad = max(1.5, 2.5 * r * f / self.cam.distance)
            fill = tuple(int(255 * a * c) for c in (cr, cg, cb))
            g.ellipse((q[0] - rad, q[1] - rad, q[0] + rad, q[1] + rad), fill=fill)
        halo = np.asarray(layer.filter(ImageFilter.GaussianBlur(6 * self.s)), np.float32)
        return Image.fromarray(np.minimum(255, np.asarray(img, np.float32) + 1.6 * halo).astype(np.uint8))

    def frame(self, view, fire_t=None, caption=None, fade=1.0):
        """The podium from view; fireworks at fire_t s into them; caption (place) over the shot; fade: 0 black."""
        self.look(view)
        self.ren.update_scene(self.d, self.cam)
        sparks = self.fireworks.at(fire_t) if fire_t is not None else []
        for p, r, rgba in sparks:
            sphere(self.ren.scene, p, r, rgba)
        img = Image.fromarray(self.ren.render())
        if sparks:
            img = self.glow(img, sparks)
        g = ImageDraw.Draw(img)
        s = self.s
        for p, _, name, colour in self.podium:
            q = K.project(self.m, self.ren, self.w, self.h, (BLOCK_X[p], -BLOCK_D / 2, BLOCK_H[p] * 0.55))
            if q is not None and 0 <= q[0] < self.w and 0 <= q[1] < self.h:      # its place on its block's front
                text = f'{p}º'
                tw, th = g.textbbox((0, 0), text, font=self.num)[2:]
                g.text((q[0] - tw / 2, q[1] - th / 2), text, font=self.num, fill=(30, 25, 20))
            q = K.project(self.m, self.ren, self.w, self.h, (*self.bots[p].pos()[:2], self.top[p] + 0.012))
            if caption is None and q is not None and 0 <= q[0] < self.w and 0 <= q[1] < self.h:      # its name over it
                tw, th = g.textbbox((0, 0), name, font=self.tag)[2:]
                x, y = q[0] - tw / 2, q[1] - th - 6 * s
                g.rectangle((x - 5 * s, y - 3 * s, x + tw + 5 * s, y + th + 5 * s), fill=(0, 0, 0))
                g.text((x, y), name, font=self.tag, fill=colour)
        g.rectangle((0, 0, self.w, 50 * s), fill=(0, 0, 0))
        g.text((16 * s, 9 * s), 'MINISUMO  ·  podio', font=self.big, fill=(255, 255, 255))
        if caption is not None:
            _, _, name, colour = next(e for e in self.podium if e[0] == caption)
            text = f'{caption}º  ·  {name}'
            tw, th = g.textbbox((0, 0), text, font=self.huge)[2:]
            x, y = self.w / 2 - tw / 2, 72 * s
            g.rectangle((x - 16 * s, y - 8 * s, x + tw + 16 * s, y + th + 14 * s), fill=(0, 0, 0))
            g.text((x, y), text, font=self.huge, fill=MEDAL[caption])
        if fade < 1.0:
            img = Image.fromarray((np.asarray(img, np.float32) * max(0.0, fade)).astype(np.uint8))
        return img


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def between(a, b, u):
    """The view a fraction u (eased) of the way from a to b."""
    u = smooth(u)
    mix = lambda x, y: (1 - u) * np.asarray(x, float) + u * np.asarray(y, float)     # noqa: E731
    return tuple(mix(a[0], b[0])), float(mix(a[1], b[1])), float(mix(a[2], b[2])), float(mix(a[3], b[3]))


# The ceremony's timeline (s): fade in on the whole podium, hold; to each robot (3rd, 2nd, 1st), hold; back to the
# whole podium and up to the sky, fireworks.
FADE_IN, HOLD_WIDE, MOVE, HOLD_ONE, BACK, FIRE_S = 1.0, 1.5, 1.4, 2.2, 2.0, 7.0


def ceremony(podium, video, fps=30):
    """The ceremony's frames into video."""
    pod = Podium(podium, video.w, video.h)
    places = [p for p, *_ in podium][::-1]                 # 3rd, 2nd, 1st (or 2nd, 1st)
    wide, fire = pod.whole(WIDE), pod.whole(FIRE)
    n = lambda s: max(1, round(s * fps))                   # noqa: E731
    for i in range(n(FADE_IN)):
        video.write(pod.frame(wide, fade=(i + 1) / n(FADE_IN)))
    for _ in range(n(HOLD_WIDE)):
        video.write(pod.frame(wide))
    view = wide
    for p in places:
        to = pod.shot(p)
        for i in range(n(MOVE)):
            video.write(pod.frame(between(view, to, (i + 1) / n(MOVE)), caption=p if i > n(MOVE) / 2 else None))
        for _ in range(n(HOLD_ONE)):
            video.write(pod.frame(to, caption=p))
        view = to
    for i in range(n(BACK)):
        u = (i + 1) / n(BACK)
        video.write(pod.frame(between(view, fire, u), fire_t=-BACK * (1 - u) + 0.5))
    for i in range(n(FIRE_S)):
        video.write(pod.frame(fire, fire_t=0.5 + i / fps, fade=min(1.0, (n(FIRE_S) - i) / n(0.8))))


def picture(podium, out, w=1280, h=720, at=3.4):
    """The whole podium with its fireworks at s into them, a jpg."""
    pod = Podium(podium, w, h)
    pod.frame(pod.whole(FIRE), fire_t=at).convert('RGB').save(out, quality=90)


class Fireworks:
    """Rockets from behind the podium bursting into sparks; at(t) gives them as spheres, t s into them."""
    G, DRAG, LIFE, RISE = 0.35, 1.6, 1.7, 0.7        # m/s^2, 1/s, s a spark lives, s a rocket rises
    PALETTE = [(1.0, 0.25, 0.25), (1.0, 0.75, 0.2), (0.35, 1.0, 0.4), (0.35, 0.65, 1.0), (0.85, 0.4, 1.0),
               (1.0, 1.0, 1.0), (0.3, 1.0, 1.0)]

    def __init__(self, colours=(), seed=28, duration=9.0):
        rng = np.random.default_rng(seed)
        pal = [tuple(c / 255 for c in col) for col in colours] + self.PALETTE
        self.bursts = []
        t = 0.0
        while t < duration:
            x, y = rng.uniform(-0.45, 0.45), rng.uniform(0.25, 0.55)
            z = rng.uniform(0.38, 0.62)
            n = int(rng.integers(90, 140))
            u = rng.normal(size=(n, 3))
            u /= np.linalg.norm(u, axis=1, keepdims=True)
            v = u * rng.uniform(0.28, 0.38) * rng.uniform(0.85, 1.0, size=(n, 1))
            c = np.array(pal[int(rng.integers(len(pal)))])
            c2 = np.array(pal[int(rng.integers(len(pal)))]) if rng.random() < 0.35 else c
            cols = np.where(rng.random((n, 1)) < 0.5, c, c2)
            self.bursts.append(dict(t=t, base=np.array([x * 0.6, y, 0.0]), at=np.array([x, y, z]), v=v, c=cols,
                                    size=rng.uniform(0.005, 0.007)))
            t += rng.uniform(0.2, 0.5)

    def sparks(self, b, tau):
        """The positions of burst b's sparks tau s after it burst."""
        k = self.DRAG
        return b['at'] + b['v'] * (1 - math.exp(-k * tau)) / k - np.array([0, 0, 0.5 * self.G * tau * tau])

    def at(self, t):
        """[(position, radius, rgba)]: the rockets, sparks and flashes t s into the fireworks."""
        out = []
        put = lambda p, r, rgba: out.append((np.asarray(p, float), r, rgba))      # noqa: E731
        for b in self.bursts:
            tau = t - b['t']
            if tau < 0 or tau > self.RISE + self.LIFE:
                continue
            if tau < self.RISE:                          # the rocket rising, a spark and its tail
                u = tau / self.RISE
                for j in range(6):
                    uj = max(0.0, u - 0.04 * j)
                    p = b['base'] + (b['at'] - b['base']) * (1 - (1 - uj) ** 2)
                    put(p, 0.003 * (1 - j / 7), (1.0, 0.85, 0.55, 0.9 * (1 - j / 6)))
                continue
            tau -= self.RISE
            life = 1 - tau / self.LIFE
            glow = 1.0 if tau < 0.12 else life ** 1.3
            twinkle = 1.0 if tau < 1.0 else 0.6 + 0.4 * ((int(t * 30) % 2))
            if tau < 0.1:                                # the flash of the burst
                put(b['at'], 0.014 * (1 - tau / 0.1) + 0.004, (1, 1, 0.9, 0.4 * (1 - tau / 0.1)))
            for j, dt in enumerate((0.0, 0.035, 0.07)):  # each spark and its streak
                if tau - dt < 0:
                    break
                ps = self.sparks(b, tau - dt)
                a = glow * twinkle * (1.0, 0.55, 0.25)[j]
                r = b['size'] * (1.0, 0.75, 0.55)[j] * (0.6 + 0.4 * life)
                for p, c in zip(ps, b['c']):
                    if p[2] > 0.01:
                        put(p, r, (*c, a))
        return out


def sphere(scn, p, r, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([r, r, r]), np.asarray(p, float),
                        np.eye(3).flatten(), np.asarray(rgba, np.float32))
    g.emission, g.castshadow, g.specular = 1.0, 0, 0.0
    scn.ngeom += 1
