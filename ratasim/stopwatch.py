"""The stopwatch of a run, as at a competition, the same for every robot: it starts when the robot leaves the start
cell, every time it does (from zero), and stops when it enters the goal. The cells are those of the robot's centre.

Each runner calls update() every simulated millisecond; its window and its video show text(), and the times are
printed (the tests read them):

    [  12.35] stopwatch: run 1 started (left the start cell)
    [  20.10] stopwatch: run 1 at the goal (7, 7) in 7.752 s
"""
from .config import CELL_MM


class Stopwatch:
    def __init__(self, maze, goal=None, start=(0, 0), say=print):
        self.goal = set(goal or maze.goal())
        self.start, self.say = start, say
        self.armed = False          # in the start cell: leaving it starts a run
        self.t0 = None              # the run's start (ms), while it runs
        self.last = None            # the last run's time to the goal (ms)
        self.runs = 0
        self.now = 0

    def cell(self, pose):
        return int(pose[0] // CELL_MM), int(pose[1] // CELL_MM)

    def update(self, t_ms, pose):
        """The robot's centre (pose: x, y mm) at t_ms."""
        self.now = t_ms
        c = self.cell(pose)
        if c == self.start:
            self.armed = True
        elif self.armed:
            self.armed, self.t0 = False, t_ms
            self.runs += 1
            self.say(f'[{t_ms / 1000:7.2f}] stopwatch: run {self.runs} started (left the start cell)')
        if self.t0 is not None and c in self.goal:
            self.last, self.t0 = t_ms - self.t0, None
            self.say(f'[{t_ms / 1000:7.2f}] stopwatch: run {self.runs} at the goal {c} in {self.last / 1000:.3f} s')

    def moved(self):
        """The robot put somewhere by hand: the run (if any) is over, none starts until it is in the start cell."""
        self.armed, self.t0 = False, None

    def text(self):
        """What the window and the video show: the run's time, running or at the goal, and the last one's."""
        if self.t0 is None:
            return f'run {self.runs}  {self.last / 1000:.3f} s  at the goal' if self.last is not None else 'time  0.00 s'
        text = f'run {self.runs}  {(self.now - self.t0) / 1000:.2f} s'
        return text + (f'   (last: {self.last / 1000:.3f} s)' if self.last is not None else '')

    def overlay(self):
        """On MuJoCo's window (its handle's set_texts): the time, big, in the top right corner."""
        import mujoco
        return (mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_TOPRIGHT, self.text(), None)

    def draw(self, d, x, y):
        """On a video frame (a PIL ImageDraw): a black box at (x, y), the time in it, green at the goal."""
        global _FONT
        if _FONT is None:
            from PIL import ImageFont
            try:
                _FONT = ImageFont.load_default(size=26)
            except TypeError:       # Pillow < 10.1: its bitmap font, one size
                _FONT = ImageFont.load_default()
        text = self.text()
        box = d.textbbox((x + 10, y + 8), text, font=_FONT)
        d.rectangle((x, y, box[2] + 10, box[3] + 8), fill=(0, 0, 0))
        done = self.t0 is None and self.last is not None
        d.text((x + 10, y + 8), text, font=_FONT, fill=(90, 230, 90) if done else (255, 255, 255))


_FONT = None
