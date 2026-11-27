"""An IR wall sensor's reading, the same model for every robot that uses it (Fujin, ZoroBot3, KOGUMA).

Its emitter lights a spot; what its receiver reads is the light that spot sends back. The spot is a Gaussian across
its axis (its emitter's beam times its receiver's field: the narrower one decides it), drawn as a fan of rays in the
plane of its axis (the edges that matter, a wall's end and a post, are upright; a wall fills the spot's height, and
the floor's share is in its maker's calibration). Each ray's light is what the robot's calibration gives at the
length the ray reaches a wall or a post (counts_at: the inverse of its firmware's conversion, or its maker's table),
times the cosine of its incidence on that face over its calibration's (a matt face returns light as the cosine of
its slant: Lambert), weighted by the Gaussian, summed. A wall's end or a post takes part of the spot; a face at a
slant returns less.

    beam = Beam(2.5)                                # sigma in deg: its emitter's half-intensity angle / 1.18
    counts = beam.light(robot_hits, origin, rot, axis_deg, counts_at, cal_cos)
"""
import math

import mujoco
import numpy as np

FAR_MM = 1000.0     # a ray that hits nothing


def sigma_from_half_angle(deg):
    """A Gaussian's sigma from its half-intensity half angle (a datasheet's +-deg at 50 %)."""
    return deg / math.sqrt(2 * math.log(2))


class Beam:
    """A sensor's spot as rays across its axis: offsets (deg) and Gaussian weights."""

    def __init__(self, sigma_deg, offsets=None):
        self.sigma = sigma_deg
        if offsets is None:     # seven rays out to two sigmas
            offsets = [sigma_deg * k / 1.5 for k in (-3, -2, -1, 0, 1, 2, 3)]
        self.rays = tuple((a, math.exp(-0.5 * (a / sigma_deg) ** 2)) for a in offsets)
        self.weight = sum(w for _, w in self.rays)

    def light(self, hit, origin, rot, axis_deg, counts_at, cal_cos=1.0, cosine=True):
        """The light its receiver reads (in counts_at's units): hit(origin, direction) -> (mm, cos) the first wall
        or post a ray reaches; rot its sensor's frame in the world; axis_deg its axis in that frame (deg from +x,
        counter-clockwise); cal_cos the cosine its calibration's wall had (1: square to it)."""
        a0 = math.radians(axis_deg)
        total = 0.0
        for spread, w in self.rays:
            a = a0 + math.radians(spread)
            mm, cos = hit(origin, rot @ np.array([math.cos(a), math.sin(a), 0.0]))
            total += w * counts_at(mm) * (cos / cal_cos if cosine else 1.0)
        return total / self.weight


class Hits:
    """Rays from a robot's sensors to the walls and posts (WALL_GROUP), its own body left out."""

    def __init__(self, model, data, wall_group, body, far_mm=FAR_MM):
        self.model, self.data, self.body, self.far = model, data, body, far_mm
        self.group = np.zeros(6, np.uint8)
        self.group[wall_group] = 1
        self.geom = np.zeros(1, np.int32)

    def length(self, origin, direction):
        """A ray's length to the first wall or post (mm)."""
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.group, 1, self.body, self.geom)
        return d / 0.001 if d >= 0 else self.far

    def __call__(self, origin, direction):
        """A ray's length to the first wall or post (mm) and the cosine between it and that face's normal."""
        d = mujoco.mj_ray(self.model, self.data, origin, direction, self.group, 1, self.body, self.geom)
        if d < 0:
            return self.far, 1.0
        g = self.geom[0]
        rot = self.data.geom_xmat[g].reshape(3, 3)
        local = rot.T @ (origin + direction * d - self.data.geom_xpos[g])
        k = int(np.argmax(np.abs(local) / self.model.geom_size[g]))       # the box's face it hits
        return d / 0.001, abs(float(rot[:, k] @ direction))
