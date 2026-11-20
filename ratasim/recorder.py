"""Videos of a robot running its firmware in a maze (ZoroBot3, NezumiBot)."""
import math
import os
import subprocess
import sys
import threading
import time

import mujoco


class Recorder:
    """An mp4 (ffmpeg): a camera following the robot, the whole maze in a corner with its trail, the time, a status
    line, an LED and the stopwatch (.clock). The robot: .model, .data, .maze, .pose() (mm, deg) and .fw.time_ms()."""

    def __init__(self, robot, path, title, width=1280, height=720, fps=30, speed=1.0, led=None):
        self.r, self.fps, self.speed = robot, fps, speed
        m = robot.model
        self.big = mujoco.Renderer(m, height, width)
        self.small = mujoco.Renderer(m, 300, 300)
        self.cam, self.top = mujoco.MjvCamera(), mujoco.MjvCamera()
        side = robot.maze.size * 0.18
        self.top.lookat[:] = (side / 2, side / 2, 0)
        self.top.distance, self.top.elevation, self.top.azimuth = side * 1.75, -90, 90
        self.cam.distance, self.cam.elevation, self.cam.azimuth = 0.55, -45, 90
        self.next_ms = robot.fw.time_ms()
        self.w, self.h = width, height
        self.trail = []
        self.status = ''
        self.clock = None           # a Stopwatch, drawn under the title
        self.title, self.led = title, led
        self.ff = subprocess.Popen(['ffmpeg', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
                                    '-s', f'{width}x{height}', '-r', str(fps), '-i', '-', '-c:v', 'libx264',
                                    '-pix_fmt', 'yuv420p', '-crf', '22', path], stdin=subprocess.PIPE)

    def maybe_frame(self, fast=False):
        from PIL import Image, ImageDraw
        r = self.r
        t = r.fw.time_ms()
        if t < self.next_ms:
            return
        speed = 8.0 if fast else self.speed
        self.next_ms = t + max(1, int(1000 * speed / self.fps))
        x, y, yaw = r.pose()
        look, target = self.cam.lookat, (x / 1000, y / 1000, 0.0)
        for i in range(3):
            look[i] = 0.8 * look[i] + 0.2 * target[i] if look.any() else target[i]
        self.big.update_scene(r.data, self.cam)
        frame = Image.fromarray(self.big.render())
        self.small.update_scene(r.data, self.top)
        inset = Image.fromarray(self.small.render())
        d2 = ImageDraw.Draw(inset)
        self.trail.append((x, y))
        side = r.maze.size * 180.0
        k = 300 / (2 * 1.75 * side * math.tan(math.radians(self.r.model.vis.global_.fovy / 2)))   # px per mm
        pts = [(150 + (px - side / 2) * k, 150 - (py - side / 2) * k) for px, py in self.trail[-4000:]]
        if len(pts) > 1:
            d2.line(pts, fill=(80, 200, 255), width=2)
        frame.paste(inset, (self.w - 310, 10))
        d = ImageDraw.Draw(frame)
        d.rectangle((0, 0, 420, 64), fill=(0, 0, 0))
        d.text((10, 8), f'{self.title} - its firmware, simulated    t = {t / 1000:6.1f} s   x{speed:g}', fill=(255, 255, 255))
        d.text((10, 34), self.status, fill=(255, 220, 60))
        if self.led:
            rgb = tuple(int(255 * c) for c in self.led())
            d.ellipse((392, 34, 410, 52), fill=rgb, outline=(120, 120, 120))
        if self.clock:
            self.clock.draw(d, 0, 64)
        self.ff.stdin.write(frame.tobytes())

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()


class Viewer:
    """MuJoCo's window on the run, paced to the clock (the waits between runs at 8x). Same calls as Recorder."""

    def __init__(self, robot, speed=1.0):
        import mujoco.viewer
        self.r, self.speed, self.status, self.clock = robot, speed, '', None
        before = set(threading.enumerate())
        self.v = mujoco.viewer.launch_passive(robot.model, robot.data)
        self.threads = [t for t in threading.enumerate() if t not in before]     # its window's
        side = robot.maze.size * 0.18
        self.v.cam.lookat[:] = (side / 2, side / 2, 0)
        self.v.cam.distance, self.v.cam.elevation, self.v.cam.azimuth = side * 1.25, -70, 90
        self.wall0, self.sim0, self.rate, self.last = time.monotonic(), robot.fw.time_ms(), speed, 0.0

    def maybe_frame(self, fast=False):
        if not self.v.is_running():
            raise SystemExit('window closed')
        rate = 8.0 if fast else self.speed
        now, t = time.monotonic(), self.r.fw.time_ms()
        if rate != self.rate:                       # re-anchor the clock when the pace changes
            self.wall0, self.sim0, self.rate = now, t, rate
        if now - self.last > 1 / 60:
            if self.clock:
                self.v.set_texts(self.clock.overlay())
            self.v.sync()
            self.last = now
        ahead = (t - self.sim0) / 1000.0 / rate - (now - self.wall0)
        if ahead > 0.002:
            time.sleep(min(ahead, 0.05))
        elif ahead < -0.5:                          # slower than asked: do not try to catch up
            self.wall0, self.sim0 = now, t

    def close(self):
        self.v.close()
        for t in self.threads:      # let its window finish: at the interpreter's exit it would crash in glfw
            t.join(3)


def add_view_args(p):
    p.add_argument('--headless', action='store_true', help='no window (with --record there is none either)')
    p.add_argument('--record', metavar='MP4')
    p.add_argument('--record-speed', type=float, default=1.0, help='the pace of window and video (x real time)')
    p.add_argument('--trace', metavar='NPZ', help='no window: its joints every 10 ms to this file (ratasim/trace.py)')


def make_view(robot, a, title, led=None, clock=None):
    """What shows the run: a video (--record), else MuJoCo's window, else (--headless, or no screen) nothing.
    clock: a Stopwatch it shows. --trace: its run to a file (ratasim/trace.py), no window and no video."""
    if a.trace:
        from .trace import Tracer
        view = Tracer(robot, a.trace)
    elif a.record:
        view = Recorder(robot, a.record, title, speed=a.record_speed, led=led)
    elif a.headless:
        return None
    elif sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        print('no screen (DISPLAY): running headless')
        return None
    else:
        view = Viewer(robot, speed=a.record_speed)
    view.clock = clock
    return view


def time_limit(a, view, default):
    """--max-s if given; else none with a window (it runs until it is closed), else the robot's default."""
    if a.max_s is not None:
        return a.max_s
    return math.inf if isinstance(view, Viewer) else default
