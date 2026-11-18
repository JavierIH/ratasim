"""Bulebule's board from the only copy of its design left: the screenshot of its Upverter layout in Bulebots' 2018
talk (figures/pcb.jpg, downloaded at build time). Its outline was fitted on that screenshot (below), its artwork
recoloured from it into the textures of both faces: solder mask, copper under it, tinned pads, silkscreen.

Calibration of the screenshot (the layout's half of the picture, full resolution): 24.606 px/mm from the Blue Pill's
pin rows (2.54 mm); the motors' axis at y 1900 px, midway between the mounts' screw holes (34 mm apart, CadQuery);
the centre line at x 1423 px.
"""
import math
import os
import urllib.request

import numpy as np

from . import config as B

LAYOUT_URL = ('https://raw.githubusercontent.com/Bulebots/20180216-bulebule-making-of-presentation/master/'
              'figures/pcb.jpg')
SCALE, XC, YAXIS = 24.606, 1423.0, 1900.0       # px/mm, px, px (the layout's half of the picture)
CROP = (360, 470, 2490, 2830)                   # the board, with a margin (px)

# The outline (mm, robot frame): its front an arc, shoulders, the waist where the mounts and wheels are, rear tabs.
DOME_CY, DOME_R = 11.57, 45.54                  # the front: an arc (least squares on the outline's pixels)
SHOULDER_X, SHOULDER_Y = B.SHOULDER_X, 24.9
TAB_X, TAB_TOP, TAB_BOTTOM = B.TAB_X, -25.3, -36.25
CORNER = 1.5


def layout(build=B.BUILD):
    """The layout's half of the screenshot (downloaded once), as an RGB array."""
    from PIL import Image
    path = os.path.join(build, 'pcb_layout.jpg')
    if not os.path.exists(path):
        os.makedirs(build, exist_ok=True)
        urllib.request.urlretrieve(LAYOUT_URL, path)
    im = Image.open(path).convert('RGB')
    return np.asarray(im.crop((int(im.width * 0.505), 0, im.width, im.height)))


def _arc(cx, cy, r, a0, a1, n):
    return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, n)]


def outline():
    """The board's outline, counter-clockwise from its rear middle (mm)."""
    right = []
    right += [(TAB_X - CORNER, TAB_BOTTOM)] + _arc(TAB_X - CORNER, TAB_BOTTOM + CORNER, CORNER, -math.pi / 2, 0, 5)
    right += _arc(TAB_X - CORNER, TAB_TOP - CORNER, CORNER, 0, math.pi / 2, 5)
    right += [(B.WAIST, TAB_TOP), (B.WAIST, SHOULDER_Y)]
    right += _arc(SHOULDER_X - CORNER, SHOULDER_Y + CORNER, CORNER, -math.pi / 2, 0, 5)
    a0 = math.asin((SHOULDER_X - 0.0) / DOME_R)     # the arc's ends: where it is SHOULDER_X off the centre line
    y_end = DOME_CY + DOME_R * math.cos(a0)
    right += [(SHOULDER_X, y_end)]
    dome = _arc(0, DOME_CY, DOME_R, math.pi / 2 - a0, math.pi / 2, 40)
    pts = [(0.0, TAB_BOTTOM)] + right + dome[1:]
    left = [(-x, y) for x, y in reversed(pts[1:-1])]
    return _drop_collinear(pts + left)


def _triangulate(poly):
    """Ear clipping (the outline is simple, counter-clockwise, without collinear points): index triples."""
    p = np.array(poly)

    def area(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    idx = list(range(len(poly)))
    tris = []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            if area(p[i0], p[i1], p[i2]) <= 1e-9:
                continue
            if any(area(p[i0], p[i1], p[j]) >= 0 and area(p[i1], p[i2], p[j]) >= 0 and area(p[i2], p[i0], p[j]) >= 0
                   for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise ValueError('no ear: the outline is not simple')
    return tris + [tuple(idx)]


def _drop_collinear(poly):
    out = list(poly)
    k = 0
    while k < len(out):
        a, b, c = np.array(out[k - 1]), np.array(out[k]), np.array(out[(k + 1) % len(out)])
        if abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) < 1e-6:
            out.pop(k)
        else:
            k += 1
    return out


def to_px(x, y):
    return XC + x * SCALE, YAXIS - y * SCALE


def textures(img, out):
    """top.png and bottom.png: the layout's colours turned into a board's (nearest of its palette)."""
    from PIL import Image
    x0, y0, x1, y1 = CROP
    a = img[y0:y1, x0:x1].astype(float)
    palette = np.array([       # the screenshot's colours
        (240, 240, 240), (224, 236, 244), (200, 215, 245),     # background and grid
        (240, 48, 240),                                        # pads (top and through)
        (30, 40, 230), (60, 80, 235),                          # top copper
        (150, 140, 235),                                       # bottom copper
        (245, 200, 165),                                       # copper pours, courtyards
        (90, 210, 195), (60, 190, 180),                        # silkscreen
        (130, 205, 240),                                       # the outline
        (240, 225, 30),                                        # bottom silkscreen (the logo)
        (20, 20, 20),                                          # the origin's cross
    ], float)
    mask, top_copper, tin, silk = (24, 92, 190), (40, 118, 214), (205, 205, 210), (238, 238, 238)
    top = [mask, mask, mask, tin, top_copper, top_copper, mask, top_copper, silk, silk, mask, mask, mask]
    bottom = [mask, mask, mask, tin, mask, mask, top_copper, mask, mask, mask, mask, silk, mask]
    k = np.empty(a.shape[:2], int)
    for r in range(0, a.shape[0], 128):             # in blocks: the distances to the palette are big
        k[r:r + 128] = ((a[r:r + 128, :, None, :] - palette[None, None]) ** 2).sum(axis=-1).argmin(axis=-1)
    for name, colours in (('top.png', top), ('bottom.png', bottom)):
        Image.fromarray(np.array(colours, np.uint8)[k]).save(os.path.join(out, name))


def meshes(out):
    """pcb_top.obj, pcb_bottom.obj (with UVs on the textures), pcb_side.obj, in the robot frame (mm)."""
    poly = outline()
    tris = _triangulate(poly)
    x0, y0, x1, y1 = CROP

    def uv(x, y):
        px, py = to_px(x, y)
        return (px - x0) / (x1 - x0), 1 - (py - y0) / (y1 - y0)
    for name, z, flip in (('pcb_top.obj', B.PCB_TOP, False), ('pcb_bottom.obj', B.PCB_BOTTOM, True)):
        with open(os.path.join(out, name), 'w') as f:
            for x, y in poly:
                f.write(f'v {x:.4f} {y:.4f} {z:.4f}\n')
            for x, y in poly:
                f.write('vt %.5f %.5f\n' % uv(x, y))
            for t in tris:
                a, b, c = (i + 1 for i in (t[::-1] if flip else t))
                f.write(f'f {a}/{a} {b}/{b} {c}/{c}\n')
    with open(os.path.join(out, 'pcb_side.obj'), 'w') as f:
        n = len(poly)
        for x, y in poly:
            f.write(f'v {x:.4f} {y:.4f} {B.PCB_TOP:.4f}\nv {x:.4f} {y:.4f} {B.PCB_BOTTOM:.4f}\n')
        for i in range(n):
            j = (i + 1) % n
            t0, b0, t1, b1 = 2 * i + 1, 2 * i + 2, 2 * j + 1, 2 * j + 2
            f.write(f'f {t0} {b0} {b1}\nf {t0} {b1} {t1}\n')


def build(out):
    os.makedirs(out, exist_ok=True)
    textures(layout(), out)
    meshes(out)
