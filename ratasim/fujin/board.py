"""Fujin's board's faces as its team's photos show them (raiju.team/images/fujin/pcb-1.png and pcb-2.png: its bare
board from below and from above, on a table, square to the camera): its art (the clouds, the maze round its middle
hole, 風神, the dark blue nose of its bottom face) is printed in colour at its maker's and is in none of its files
(its Altium board, its DXFs, its STEP), only in those photos. They are downloaded at build time (its site has no
licence: nothing of it is copied here) and pinned by their SHA-256.

Each photo is mapped onto the board's face by an affine map from the robot frame (mm) to its pixels, fitted to its
four mounting holes (x = +-37, y = +-16.5 in its STEP) and its middle hole (0, -15), found in it as the centroids of
their dark pixels (the table under the board): under a pixel (0.15 mm) off at every one. pcb-1.png shows its bottom
face seen from below: mirrored, its left on the right.

textures(out) writes board_top.png and board_bottom.png over FRAME, both seen from above (u = x, v = y)."""
import hashlib
import os
import urllib.request

from . import config as F

URL = 'https://raiju.team/images/fujin/'
PHOTOS = {      # face: its photo, its SHA-256, the affine map (robot mm -> its pixels): (a, b, c), (d, e, f)
    'top': ('pcb-2.png', '356d5733c2bbd03c904a7e1f25c01014496f538873263c7b7b0c077b24777000',
            ((5.5004, -0.0063, 243.392), (0.0065, -5.5256, 299.476))),
    'bottom': ('pcb-1.png', 'c07dd2263a6ad398cca9119573635b6b87ea38a7c148e81bd046e429ae194b67',
               ((-5.4818, 0.0084, 245.579), (-0.0127, -5.4435, 306.123))),
}
FRAME = (-40.5, -70.5, 40.5, 50.5)      # the textures' extent (x0, y0, x1, y1, mm): its outline and half a mm
PX_MM = 8                               # the textures' resolution (its photos: 5.5 pixels a mm)
# The simulator's lights wash its photos out (as its board's colour in cad.py, darker than in them): its board's blue
# (the median of its pixels near PHOTO_BLUE) turned to cad.py's, the rest darkened by GAIN.
PHOTO_BLUE = (140, 195, 210)
GAIN = 0.85
PHOTOS_DIR = os.path.join(F.BUILD, 'photos')


def _sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def download():
    """Its photos in PHOTOS_DIR (once), checked."""
    os.makedirs(PHOTOS_DIR, exist_ok=True)
    for name, sha, _ in PHOTOS.values():
        path = os.path.join(PHOTOS_DIR, name)
        if not os.path.exists(path):
            req = urllib.request.Request(URL + name, headers={'User-Agent': 'ratasim'})     # Python's refused
            data = urllib.request.urlopen(req, timeout=60).read()
            with open(path + '.part', 'wb') as f:
                f.write(data)
            os.replace(path + '.part', path)
        if _sha256(path) != sha:
            os.unlink(path)
            raise SystemExit(f'Fujin: {name} is not the photo pinned in ratasim/fujin/board.py (SHA-256 {sha})')


def textures(out, blue):
    """board_top.png and board_bottom.png: each face's photo resampled over FRAME, seen from above (its rows from
    y1 down to y0), its board's blue turned to blue (RGB), its pixels the nearer to it the more."""
    import numpy as np
    from PIL import Image
    download()
    x0, y0, x1, y1 = FRAME
    size = (int(round((x1 - x0) * PX_MM)), int(round((y1 - y0) * PX_MM)))
    for face, (name, _, ((a, b, c), (d, e, f))) in PHOTOS.items():
        img = Image.open(os.path.join(PHOTOS_DIR, name)).convert('RGB')
        # a texture's pixel (i, j) is at x = x0 + i / PX_MM, y = y1 - j / PX_MM; its photo's pixel, a x + b y + c...
        s = 1 / PX_MM
        coeffs = (a * s, -b * s, a * x0 + b * y1 + c, d * s, -e * s, d * x0 + e * y1 + f)
        px = np.asarray(img.transform(size, Image.AFFINE, coeffs, Image.BICUBIC), float)
        near = np.linalg.norm(px - PHOTO_BLUE, axis=2) < 40
        gain = np.array(blue) / np.median(px[near], axis=0)
        w = np.clip(1.5 - np.linalg.norm(px - np.median(px[near], axis=0), axis=2) / 40, 0, 1)[..., None]
        px = px * (w * gain + (1 - w) * GAIN)
        Image.fromarray(px.clip(0, 255).astype(np.uint8)).save(os.path.join(out, f'board_{face}.png'))


def uv(xy):
    """The textures' coordinates of points (n, 2) mm: (0, 0) at FRAME's corner x0, y0."""
    x0, y0, x1, y1 = FRAME
    return (xy - (x0, y0)) / (x1 - x0, y1 - y0)
