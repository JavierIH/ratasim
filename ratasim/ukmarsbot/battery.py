"""UKMARSBOT's battery: a PP3 (IEC 6LR61, ANSI 1604A) lying on its edge behind the motors, its long side across the
robot and its snap terminals to its right, with the snap clip its photos show on them (renders/ukmarsbot-600x526.png)
and the clip's red and black leads down to the main board's battery connector (J1, a Molex KK 2-pin header).

Its shape from the standard's drawing (IEC 60086-2, as Duracell's MN1604 and Energizer's 522 data sheets give it,
there being no 9 V battery in KiCad's 3D library): 48.5 mm overall, a 26.5 x 17.5 mm body (its four long edges
rounded) of up to 46.4 mm, its top's crimped shoulder down to 24.5 x 15.5 mm, and on it the two snaps 12.7 mm apart
along the face's long side: the positive a round stud (the smaller, male), the negative a hollow hexagon (the larger,
female), both 3 mm high, nickel-plated steel.

Its look: an alkaline battery's, black with a copper band at its top, a copper shoulder (the colours of the
best-known "copper top" alkaline cells, without their name or drawings), and "Ratasim", small, on the face looking
forward (the side its portrait sees over the motors). The body's sides are one mesh with its label as a texture,
drawn here at build time.

The robot's frame (mm, model.py): x right, y forward, z up from the floor.

    python -m ratasim.ukmarsbot.battery        # makes them again

Writes MESHES/battery_*.obj and MESHES/battery_label.png.
"""
import math
import os
import sys

from . import config as U

# The standard's sizes (mm)
WIDTH, DEPTH = 26.5, 17.5               # its body's section: standing on its edge, WIDTH is its height
CORNER_R = 2.0                          # its long edges' rounding
BODY_L = 44.5                           # its body, from its bottom to its top's shoulder
SHOULDER_L, SHOULDER_W, SHOULDER_D = 1.0, 24.5, 15.5
SNAP_H = 3.0                            # the snaps over the shoulder: 48.5 mm overall
SNAP_PITCH = 12.7
STUD_R, STUD_NECK_R = 2.85, 2.35        # the positive's round stud, narrower at its foot
HEX_R, HEX_HOLE_R = 4.0, 2.9            # the negative's hexagon (its corners' radius) and its hole
OVERALL = BODY_L + SHOULDER_L + SNAP_H

# Where it lies (robot frame): its bottom at X0, its top (and snaps) to +x, standing on the main board's top behind
# the motors, between the function switches (x < -26.5) and the battery connector J1 (x > 25.9)
X0 = -26.0
Y_MID = -25.0
Z0 = U.MAIN_TOP
Z_MID = Z0 + WIDTH / 2
X_TOP = X0 + BODY_L + SHOULDER_L        # the shoulder's face, the snaps' foot
SNAPS = {'+': Z_MID + SNAP_PITCH / 2, '-': Z_MID - SNAP_PITCH / 2}       # their height (centres)

# Its clip: a black cap pressed on the snaps (on its outside its own snaps' rivets, mirrored: a hexagon over the
# battery's positive, a round one over its negative) and its two leads, to J1's pins
CLIP_L, CLIP_W, CLIP_D = 6.0, 25.0, 16.0
CLIP_X0 = X_TOP + 0.2
J1_PINS = {'+': (28.85, -26.74), '-': (28.85, -24.20)}         # kicad.py: J1's pads, /PWR_IN and GND
J1_TOP = Z0 + 12.1                     # the header's top (its 3D model)
LEAD_R = 0.65

# Colours (RGB 0-255): a matte black body; copper as on photos of such cells; nickel-plated snaps
BLACK_RGB = (22, 22, 24)
COPPER_RGB = (184, 112, 54)            # the band
COPPER_TOP_RGB = (196, 128, 70)        # the shoulder, brighter (its metal)
TEXT_RGB = (150, 150, 152)             # "Ratasim": light grey, small
STEEL = (0.78, 0.78, 0.80, 1)
CLIP_RGBA = (0.08, 0.08, 0.09, 1)
PLUG_RGBA = (0.93, 0.93, 0.88, 1)      # the KK crimp housing, white nylon
LEAD_RGBA = {'+': (0.80, 0.06, 0.05, 1), '-': (0.05, 0.05, 0.05, 1)}
BAND_L = 13.5                          # the copper band, from the shoulder down (about its top third)
LABEL = 'Ratasim'
LABEL_H = 3.2                          # its letters' height (mm)

PX_MM = 24                             # the label's texture
CORNER_N = 8
MESHES = os.path.join(U.BUILD, 'meshes')


def rgba(rgb, a=1.0):
    return tuple(round(v / 255, 3) for v in rgb) + (a,)


# ---- geometry ------------------------------------------------------------------------------------------------------
def _rounded_rect(w, d, r, n=CORNER_N):
    """A rounded rectangle's outline, (y, z) about its centre (d along y, w along z), counter-clockwise seen from +x,
    from its bottom's +y end (the corner at +y -z first): its points and their distances along it (closed: one more)."""
    pts = []
    corners = ((d / 2 - r, -w / 2 + r, -90), (d / 2 - r, w / 2 - r, 0), (-d / 2 + r, w / 2 - r, 90),
               (-d / 2 + r, -w / 2 + r, 180))
    for cy, cz, a0 in corners:
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cy + r * math.cos(a), cz + r * math.sin(a)))
    s = [0.0]
    for p, q in zip(pts, pts[1:] + pts[:1]):
        s.append(s[-1] + math.dist(p, q))
    return pts, s


def _write_obj(path, verts, faces, uvs=None):
    with open(path, 'w') as f:
        for v in verts:
            f.write('v {:.4f} {:.4f} {:.4f}\n'.format(*v))
        if uvs:
            for u in uvs:
                f.write('vt {:.5f} {:.5f}\n'.format(*u))
        for t in faces:
            a, b, c = (i + 1 for i in t)
            f.write(f'f {a}/{a} {b}/{b} {c}/{c}\n' if uvs else f'f {a} {b} {c}\n')


def _side_obj(path):
    """The body's sides with their label's UVs: u round the section (from the corner at -y -z, by +y first), v along
    its length (0 at its bottom)."""
    pts, s = _rounded_rect(WIDTH, DEPTH, CORNER_R)
    per = s[-1]
    ring = pts + pts[:1]                 # the seam's point twice (u 0 and 1)
    verts, uvs, faces = [], [], []
    for x, v in ((X0, 0.0), (X0 + BODY_L, 1.0)):
        for (y, z), si in zip(ring, s):
            verts.append((x, Y_MID + y, Z_MID + z))
            uvs.append((si / per, v))
    n = len(ring)
    for i in range(n - 1):
        a, b, c, d = i, i + 1, n + i + 1, n + i
        faces += [(a, b, c), (a, c, d)]  # outward: the outline counter-clockwise from +x, x growing
    _write_obj(path, verts, faces, uvs)
    return per


def _cap_obj(path, x, pts, outward):
    """A flat end: the outline's fan from its centre, facing +x (outward 1) or -x (-1)."""
    verts = [(x, Y_MID, Z_MID)] + [(x, Y_MID + y, Z_MID + z) for y, z in pts]
    n = len(pts)
    faces = [(0, 1 + i, 1 + (i + 1) % n) if outward > 0 else (0, 1 + (i + 1) % n, 1 + i) for i in range(n)]
    _write_obj(path, verts, faces)


def _shoulder_obj(path):
    """The top's crimped shoulder: from the body's section to the smaller one, and its flat face."""
    big, _ = _rounded_rect(WIDTH, DEPTH, CORNER_R)
    small, _ = _rounded_rect(SHOULDER_W, SHOULDER_D, CORNER_R - 0.6)
    n = len(big)
    x1 = X0 + BODY_L
    verts = [(x1, Y_MID + y, Z_MID + z) for y, z in big] + [(X_TOP, Y_MID + y, Z_MID + z) for y, z in small]
    verts.append((X_TOP, Y_MID, Z_MID))
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [(i, j, n + j), (i, n + j, n + i), (2 * n, n + i, n + j)]
    _write_obj(path, verts, faces)


def _lathe(profile, centre, sides):
    """A solid of revolution about the x axis through centre (y, z): profile [(x, r)], from its foot; its polygon of
    `sides` (6 with its corners at radius r: a hexagon)."""
    cy, cz = centre
    verts, faces = [], []
    for x, r in profile:
        for k in range(sides):
            a = 2 * math.pi * (k + 0.5) / sides
            verts.append((x, cy + r * math.cos(a), cz + r * math.sin(a)))
    for i in range(len(profile) - 1):
        for k in range(sides):
            a, b = i * sides + k, i * sides + (k + 1) % sides
            c, d = b + sides, a + sides
            faces += [(a, b, c), (a, c, d)]
    for end, flip in ((0, True), (len(profile) - 1, False)):       # their ends' discs (an inner ring's left open)
        if profile[end][1] == 0:
            continue
        verts.append((profile[end][0], cy, cz))
        o = len(verts) - 1
        for k in range(sides):
            a, b = end * sides + k, end * sides + (k + 1) % sides
            faces.append((o, b, a) if flip else (o, a, b))
    return verts, faces


def _snaps_obj(path):
    """The battery's two snaps on its shoulder: the positive's stud (a neck, its rounded head), the negative's hollow
    hexagon (its outside, its rim, its hole's wall to a floor); the clip's rivets."""
    x = X_TOP
    stud = [(x, STUD_NECK_R), (x + 1.0, STUD_NECK_R), (x + 1.1, STUD_R), (x + SNAP_H - 0.5, STUD_R),
            (x + SNAP_H - 0.1, STUD_R - 0.5), (x + SNAP_H, STUD_R - 1.0)]
    v1, f1 = _lathe(stud, (Y_MID, SNAPS['+']), 32)
    # the hexagon: its outer wall up, its rim in, its inner wall down; the inner faces turned inwards
    hx = [(x, HEX_R), (x + SNAP_H, HEX_R)]
    v2, f2 = _lathe(hx, (Y_MID, SNAPS['-']), 6)
    v2 = v2[:12]                                       # no end discs: the rim and the hole below
    f2 = [t for t in f2 if max(t) < 12]
    rim_in = []
    for k in range(6):
        a = 2 * math.pi * (k + 0.5) / 6
        rim_in.append((x + SNAP_H, Y_MID + HEX_HOLE_R * math.cos(a), SNAPS['-'] + HEX_HOLE_R * math.sin(a)))
    for k in range(6):
        a = 2 * math.pi * (k + 0.5) / 6
        rim_in.append((x + 0.6, Y_MID + HEX_HOLE_R * math.cos(a), SNAPS['-'] + HEX_HOLE_R * math.sin(a)))
    base = len(v2)
    v2 += rim_in + [(x + 0.6, Y_MID, SNAPS['-'])]
    for k in range(6):
        j = (k + 1) % 6
        o_top, i_top, i_bot = 6 + k, base + k, base + 6 + k
        o_top2, i_top2, i_bot2 = 6 + j, base + j, base + 6 + j
        f2 += [(o_top, o_top2, i_top2), (o_top, i_top2, i_top)]              # the rim, facing +x
        f2 += [(i_top, i_top2, i_bot2), (i_top, i_bot2, i_bot)]              # the hole's wall, facing in
        f2.append((base + 12, i_bot, i_bot2))                                # its floor, facing +x
    verts, faces = [], []
    # and the clip's snaps' rivets on its cap's outside: a hexagon over the battery's +, a round one over its -
    x1 = CLIP_X0 + CLIP_L
    v3, f3 = _lathe([(x1, 3.4), (x1 + 0.6, 3.4), (x1 + 0.8, 2.8)], (Y_MID, SNAPS['+']), 6)
    v4, f4 = _lathe([(x1, 2.6), (x1 + 0.5, 2.6), (x1 + 0.8, 1.8)], (Y_MID, SNAPS['-']), 24)
    for v, f in ((v1, f1), (v2, f2), (v3, f3), (v4, f4)):
        faces += [tuple(i + len(verts) for i in t) for t in f]
        verts += v
    _write_obj(path, verts, faces)


def _label_png(path, per):
    """The sides' texture: black, the copper band at the top (the image's top rows: v 1), "Ratasim" on the face
    looking forward (+y), upright seen from there (reading from the battery's top to its bottom)."""
    from PIL import Image, ImageDraw, ImageFont
    _, s = _rounded_rect(WIDTH, DEPTH, CORNER_R)
    W, H = int(round(per * PX_MM)), int(round(BODY_L * PX_MM))
    img = Image.new('RGB', (W, H), BLACK_RGB)
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, W, int(round(BAND_L * PX_MM))), fill=COPPER_RGB)
    # the face looking forward (+y): the outline's stretch from (d/2, -w/2 + r) to (d/2, w/2 - r)
    i0 = CORNER_N                          # the first corner's last point: (d/2, -w/2 + r)
    s_lo = s[i0]                           # along u: z from -w/2 + r up
    # the text, drawn upright then turned: its baseline along the battery's length, its up +z
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', int(LABEL_H * PX_MM * 1.4))
    except OSError:
        font = ImageFont.load_default()
    box = font.getbbox(LABEL)
    tw, th = box[2] - box[0], box[3] - box[1]
    t = Image.new('L', (tw + 8, th + 8), 0)
    ImageDraw.Draw(t).text((4 - box[0], 4 - box[1]), LABEL, font=font, fill=255)
    # seen from +y the battery's top (+x) is on the left: the text reads from the top towards the bottom, so in the
    # image (rows from the top's end, columns along u = +z) its baseline runs down the rows, its up to +u
    t = t.rotate(-90, expand=True)
    zc = Z0 + WIDTH * 0.68 - Z_MID         # high on the face, above the motors seen from the front
    xc = (BODY_L - BAND_L) / 2             # the middle of the black part (mm from the bottom)
    cu = (s_lo + (zc + WIDTH / 2 - CORNER_R)) * PX_MM
    cv = (BODY_L - xc) * PX_MM
    img.paste(Image.new('RGB', t.size, TEXT_RGB), (int(cu - t.size[0] / 2), int(cv - t.size[1] / 2)), t)
    img.save(path)


def build(out=MESHES):
    os.makedirs(out, exist_ok=True)
    per = _side_obj(os.path.join(out, 'battery_side.obj'))
    pts, _ = _rounded_rect(WIDTH, DEPTH, CORNER_R)
    _cap_obj(os.path.join(out, 'battery_bottom.obj'), X0, pts, -1)
    _shoulder_obj(os.path.join(out, 'battery_top.obj'))
    _snaps_obj(os.path.join(out, 'battery_snaps.obj'))
    _label_png(os.path.join(out, 'battery_label.png'), per)
    return out


def ensure(out=MESHES):
    """The battery's meshes and label, made first if missing or older than this file or config.py."""
    f = os.path.join(out, 'battery_label.png')
    if os.path.exists(f) and os.path.getmtime(f) >= max(os.path.getmtime(__file__), os.path.getmtime(U.__file__)):
        return out
    return build(out)


# ---- MuJoCo -------------------------------------------------------------------------------------------------------
def assets_xml(mm):
    d = ensure()
    s = f'{mm} {mm} {mm}'
    return (f'<texture name="k_battery_label" type="2d" file="{d}/battery_label.png"/>'
            '<material name="k_battery_label" texture="k_battery_label" specular="0.15" shininess="0.2"/>'
            + ''.join(f'<mesh name="k_battery_{p}" file="{d}/battery_{p}.obj" scale="{s}" inertia="shell"/>'
                      for p in ('side', 'bottom', 'top', 'snaps')))


def visuals(box, rod, f):
    """Its geoms (model.py's helpers: box(lo, hi, rgba), rod(p0, p1, r, rgba, kind), f(*v) for numbers)."""
    g = ['<geom class="kvisual" type="mesh" mesh="k_battery_side" material="k_battery_label"/>',
         f'<geom class="kvisual" type="mesh" mesh="k_battery_bottom" rgba="{f(*rgba(BLACK_RGB))}"/>',
         f'<geom class="kvisual" type="mesh" mesh="k_battery_top" rgba="{f(*rgba(COPPER_TOP_RGB))}"/>',
         f'<geom class="kvisual" type="mesh" mesh="k_battery_snaps" rgba="{f(*STEEL)}"/>']
    # the clip's cap (its rivets with the snaps' mesh)
    x1 = CLIP_X0 + CLIP_L
    g.append(box((CLIP_X0, Y_MID - CLIP_D / 2, Z_MID - CLIP_W / 2), (x1, Y_MID + CLIP_D / 2, Z_MID + CLIP_W / 2),
                 CLIP_RGBA))
    # the crimp housing on J1 and the leads: out of the cap's top, in a loop over to the housing and down into it
    (px, _), ys = J1_PINS['+'], [p[1] for p in J1_PINS.values()]
    g.append(box((px - 2.3, min(ys) - 1.9, J1_TOP), (px + 2.3, max(ys) + 1.9, J1_TOP + 3.0), PLUG_RGBA))
    for k, (jx, jy) in J1_PINS.items():
        z0, y0 = Z_MID + CLIP_W / 2, Y_MID + (jy - Y_MID) * 1.2
        p0, p1 = (x1 - 2.5, y0, z0), (x1 - 2.5, y0, z0 + 8.0)
        p2, p3 = (jx + 4.0, jy, J1_TOP + 16.0), (jx, jy, J1_TOP + 3.0)
        pts = [_bezier(p0, p1, p2, p3, i / 10) for i in range(11)]
        g += [rod(a, b, LEAD_R, LEAD_RGBA[k], 'capsule') for a, b in zip(pts, pts[1:])]
    return ''.join(g)


def _bezier(p0, p1, p2, p3, t):
    u = 1 - t
    return tuple(u ** 3 * a + 3 * u * u * t * b + 3 * u * t * t * c + t ** 3 * d for a, b, c, d in zip(p0, p1, p2, p3))


if __name__ == '__main__':
    print(build(*sys.argv[1:2]))
