"""Videos of a robot running its firmware in a maze (ZoroBot3, NezumiBot)."""
import math
import subprocess

import mujoco


class Recorder:
    """An mp4 (ffmpeg): a camera following the robot, the whole maze in a corner with its trail, the time, a status
    line and an LED. The robot: .model, .data, .maze, .pose() (mm, deg) and .fw.time_ms()."""

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
        self.ff.stdin.write(frame.tobytes())

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()
