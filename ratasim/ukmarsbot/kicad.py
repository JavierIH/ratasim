"""UKMARSBOT's boards read from their KiCad files (the submodule external/ukmarsbot, ecad/kicad): their outlines
(Edge.Cuts) and their footprints, in the robot's frame.

KiCad's frame: x, y in mm, y down; the boards are drawn with the robot facing +x, its wheels' axle at x 105 (the
motors' footprints, the middle of the wheels' notches) and its middle at y 105 (the sensor board is drawn on the
main board's outline). The robot's frame: x right, y forward, origin on the axle:

    x_robot = y_kicad - 105         y_robot = x_kicad - 105

A footprint's local point (u, v), at (x, y) rotated by a (degrees, counter-clockwise on the screen), is at
(x + u cos a + v sin a, y - u sin a + v cos a) on the board (the file's local points of a footprint on the bottom
are already mirrored).
"""
import math
import re

ORIGIN = (105.0, 105.0)


def parse(text):
    """An S-expression as nested lists of strings."""
    stack = [[]]
    for tok in re.findall(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+', text):
        if tok == '(':
            stack.append([])
        elif tok == ')':
            node = stack.pop()
            stack[-1].append(node)
        else:
            stack[-1].append(tok[1:-1] if tok[0] == '"' else tok)
    return stack[0][0]


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def one(node, key):
    found = find(node, key)
    return found[0] if found else None


def to_robot(kx, ky):
    return ky - ORIGIN[1], kx - ORIGIN[0]


def angle_in_robot(a_deg):
    """A footprint's local +v axis, rotated by a, as an angle in the robot's frame (deg from +x, counter-clockwise
    seen from above)."""
    a = math.radians(a_deg)
    dx, dy = math.sin(a), math.cos(a)           # on the board
    return math.degrees(math.atan2(dx, dy))      # robot: (x, y) = (dy, dx)


class Footprint:
    def __init__(self, node):
        self.node = node
        self.lib = node[1]
        self.props = {p[1]: p[2] for p in find(node, 'property')}
        self.ref = self.props.get('Reference', '')
        self.value = self.props.get('Value', '')
        at = [float(v) for v in one(node, 'at')[1:]]
        self.x, self.y = at[0], at[1]
        self.rot = at[2] if len(at) > 2 else 0.0
        self.bottom = one(node, 'layer')[1].startswith('B.')

    def point(self, u, v):
        """A local point on the board, in the robot's frame (mm)."""
        a = math.radians(self.rot)
        return to_robot(self.x + u * math.cos(a) + v * math.sin(a), self.y - u * math.sin(a) + v * math.cos(a))

    def axis_deg(self):
        return angle_in_robot(self.rot)

    def outline(self, layers=('F.CrtYd', 'B.CrtYd')):
        """Its courtyard's corners (robot frame): the points of its lines and rectangles on those layers."""
        pts = []
        for g in find(self.node, 'fp_line') + find(self.node, 'fp_rect') + find(self.node, 'fp_poly'):
            if one(g, 'layer')[1] not in layers:
                continue
            for key in ('start', 'end'):
                p = one(g, key)
                if p:
                    pts.append(self.point(float(p[1]), float(p[2])))
            pl = one(g, 'pts')
            if pl:
                pts += [self.point(float(q[1]), float(q[2])) for q in find(pl, 'xy')]
        return pts

    def bbox(self, layers=('F.CrtYd', 'B.CrtYd')):
        pts = self.outline(layers)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return min(xs), min(ys), max(xs), max(ys)


class Board:
    def __init__(self, path):
        self.root = parse(open(path, encoding='utf-8').read())
        self.thickness = float(one(one(self.root, 'general'), 'thickness')[1])
        self.footprints = [Footprint(f) for f in find(self.root, 'footprint')]

    def footprint(self, ref=None, lib=None, value=None):
        return [f for f in self.footprints if (ref is None or f.ref == ref) and (lib is None or lib in f.lib)
                and (value is None or f.value == value)]

    def edges(self):
        """Edge.Cuts' lines and arcs as polylines (robot frame), arcs sampled every 5 deg."""
        segs = []
        for g in find(self.root, 'gr_line') + find(self.root, 'gr_arc'):
            if one(g, 'layer')[1] != 'Edge.Cuts':
                continue
            s, e = (tuple(float(v) for v in one(g, k)[1:3]) for k in ('start', 'end'))
            if g[0] == 'gr_line':
                segs.append([s, e])
            else:
                segs.append(_arc(s, tuple(float(v) for v in one(g, 'mid')[1:3]), e))
        return [[to_robot(*p) for p in seg] for seg in segs]

    def loops(self):
        """Edge.Cuts' closed loops (robot frame), the outer one first, each counter-clockwise."""
        segs, loops = [list(s) for s in self.edges()], []
        close = 1e-3
        while segs:
            loop = segs.pop(0)
            while math.dist(loop[0], loop[-1]) > close:
                for i, s in enumerate(segs):
                    if math.dist(s[0], loop[-1]) < close:
                        loop += s[1:]
                    elif math.dist(s[-1], loop[-1]) < close:
                        loop += s[::-1][1:]
                    else:
                        continue
                    segs.pop(i)
                    break
                else:
                    raise ValueError('Edge.Cuts: an open outline')
            loop = loop[:-1]
            if _area(loop) < 0:
                loop.reverse()
            loops.append(loop)
        loops.sort(key=lambda lp: -abs(_area(lp)))
        return loops


def _area(poly):
    return sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1])) / 2


def _arc(s, m, e):
    """The arc through start, mid and end, sampled."""
    (x1, y1), (x2, y2), (x3, y3) = s, m, e
    d = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    ux = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / d
    uy = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / d
    r = math.dist((ux, uy), s)
    a1, a2, a3 = (math.atan2(y - uy, x - ux) for x, y in (s, m, e))

    def ccw(a, b):     # the angle from a to b, counter-clockwise, in [0, 2 pi)
        return (b - a) % (2 * math.pi)
    sweep = ccw(a1, a3)
    if ccw(a1, a2) > sweep:          # the mid point is on the other way round
        sweep -= 2 * math.pi
    n = max(2, int(abs(math.degrees(sweep)) / 5) + 1)
    return [(ux + r * math.cos(a1 + sweep * i / n), uy + r * math.sin(a1 + sweep * i / n)) for i in range(n + 1)]
