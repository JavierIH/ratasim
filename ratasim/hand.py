"""A hand the robots' sensors see: how their makers start several of them. A mocap body in the robot's scene, out
of the way until it covers some sensors: their rays see it as a wall, nothing touches it, videos show it."""
import math

import numpy as np

from .model import MM, WALL_GROUP, _f

PALM_MM = (14.0, 76.0, 80.0)        # thickness (towards the sensors), width, height
FINGERS = [(-27.0, 52.0), (-9.0, 60.0), (9.0, 56.0), (27.0, 44.0)]     # across the palm (mm), length
AWAY = (0.0, 0.0, 3.0)
SKIN = 'rgba="0.93 0.74 0.62 1"'


def hand_xml():
    """Its palm towards +x, its fingers up, its thumb to the side; the body's origin at the palm's centre."""
    t, w, h = (v / 2 for v in PALM_MM)
    g = f'group="{WALL_GROUP}" contype="0" conaffinity="0" {SKIN}'
    parts = [f'<geom name="hand" type="box" size="{_f(t * MM, w * MM, h * MM)}" {g}/>']
    for y, length in FINGERS:
        parts.append(f'<geom type="capsule" size="{8.0 * MM:.4f}" fromto="{_f(0, y * MM, h * MM, 0, y * MM, (h + length) * MM)}" {g}/>')
    parts.append(f'<geom type="capsule" size="{9.0 * MM:.4f}" fromto="{_f(0, w * MM, -10 * MM, 0, (w + 28) * MM, 25 * MM)}" {g}/>')
    return f'<body name="hand" mocap="true" pos="{_f(*AWAY)}">{"".join(parts)}</body>'


class Hand:
    def __init__(self, model, data):
        self.data = data
        self.mocap = model.body('hand').mocapid[0]
        self.sensors, self.until = (), 0

    def cover(self, rays, mm=30.0):
        """Its palm mm in front of these sensors: their (origin, direction) in the world (m)."""
        p = np.mean([o for o, _ in rays], axis=0)
        d = np.mean([v for _, v in rays], axis=0)
        d[2] = 0.0
        d /= np.linalg.norm(d)
        c = p + d * (mm + PALM_MM[0] / 2) * MM
        c[2] = p[2] + (PALM_MM[2] / 2 - 15.0) * MM          # its palm from 15 mm under the sensors up
        yaw = math.atan2(d[1], d[0])
        self.data.mocap_pos[self.mocap] = c
        self.data.mocap_quat[self.mocap] = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))

    def away(self):
        self.data.mocap_pos[self.mocap] = AWAY
