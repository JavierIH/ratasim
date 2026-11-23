"""Ratatron II's 3D model from its FreeCAD file (external/ratatron2/mechanics/ratatron2.fcstd), to ratatron2/assets:

    ~/programs/FreeCAD_*.AppImage freecadcmd ratasim/ratatron2/cad.py

What the file shows (as saved): every visible leaf of its tree (groups, App::Part, link groups with the visibility
of each element), and what a leaf is made from when that is visible and in no group (the sources of its mirrors),
each in its place (the paths' placements) and in its faces' colours (GuiDocument.xml; a link without its own in its
linked object's, a mirror saved in one colour in its source's faces'). Its board and parts are its KiCad export (the
`kicad` part), its drivetrain, battery and fan its designer's. Left out: "chassis" (the board's outline, the KiCad
board is on it), Body008 (the IR holder's design at the origin, inside the motors: the four on the board are the
KiCad ones) and anything far from the robot (loose parts beside it in the file).

Bodies: robot, and its four wheels (the parts of each wheel-assembly link group: rim, gear, tyre, bearings), wheel_fl,
wheel_fr, wheel_rl, wheel_rr. Its LEDs (D1 to D7, 0603, KiCad's diode model: a black body between tinned ends): their
bodies apart, one mesh each, led_1 to led_6 (D2 to D7, its firmware's LED1 to LED6) and led_pilot (D1), so that
model.py lights them.

Out (ratatron2/assets, committed: CC BY-SA 4.0, as its CAD; NOTICE): one STL per body and colour (the CAD's frame, mm)
and manifest.json: [{file, body, rgba}].
"""
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile

import FreeCAD
import Mesh
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
from ratasim.ratatron2 import config as R  # noqa: E402  (its file, its LEDs' places)

FCSTD = R.FCSTD
OUT = R.ASSETS
SKIP = ('chassis', 'Body008')
PADS = 'ratatron2_pad'                      # its board's copper (pads, vias): their tops and bottoms only
NEAR = (-50.0, 50.0, -50.0, 70.0)           # x and y (mm) of what belongs to the robot
LED_BODY = ((38, 37, 37), (176, 169, 152))  # KiCad's diode model: its body and its cathode's band
BLACK = {'DADO_ESAGONALE_AUTOBLOCCANTE': (22, 22, 24, 1.0)}   # its four axles' lock nuts, black (its maker's;
#                                                             the CAD's nylon blue and steel grey)
COARSE = ((2.0, 0.1, 1.2), (8.0, 0.05, 0.5), (1e9, 0.05, 0.2))   # faces under this diagonal (mm): this linear (mm)
#                                                                   and angular (rad) deflection


def appearances():
    """Object name -> per-face colours [(r, g, b, a)] from the ShapeAppearance lists of GuiDocument.xml."""
    z = zipfile.ZipFile(FCSTD)
    out = {}
    for vp in ET.fromstring(z.read('GuiDocument.xml')).iter('ViewProvider'):
        for p in vp.iter('Property'):
            if p.get('name') == 'ShapeAppearance' and p.find('MaterialList') is not None:
                d = z.read(p.find('MaterialList').get('file'))
                n = struct.unpack('<I', d[:4])[0]   # 24 bytes each: ambient, diffuse, specular, emissive,
                #                                         shininess, transparency
                out[vp.get('name')] = [(d[11 + 24 * i], d[10 + 24 * i], d[9 + 24 * i],
                                        1 - struct.unpack('<f', d[24 + 24 * i:28 + 24 * i])[0]) for i in range(n)]
    return out


def kids(o):
    if o.TypeId in ('App::Part', 'App::DocumentObjectGroup'):
        return o.Group
    if o.TypeId == 'App::LinkGroup':
        return o.ElementList
    return None


def leaves(doc):
    """[(path, wheel group or None)]: the visible leaves, each by its path from a root object."""
    out, seen = [], set()

    def walk(o, path, vis, wheel):
        if not vis:
            return
        k = kids(o)
        if k is None:
            if o in seen:
                return
            seen.add(o)
            out.append((path + [o], wheel))
            if o.TypeId != 'PartDesign::Body':          # a Body's features are the Body
                for c in o.OutList:                     # its sources in no group, drawn when visible
                    if hasattr(c, 'Shape') and c.getParentGeoFeatureGroup() is None and c.getParentGroup() is None:
                        walk(c, [], c.Visibility, None)
            return
        if o.TypeId == 'App::LinkGroup' and o.Label.startswith('wheel-assembly'):
            wheel = o
        for c in k:
            e = o.isElementVisible(c.Name)
            walk(c, path + [o], c.Visibility if e < 0 else bool(e), wheel)
    for r in doc.RootObjects:
        walk(r, [], r.Visibility, None)
    return out


def wheel_name(shape):
    """wheel_fl ... wheel_rr by where a part of a wheel is (+y forward, +x right)."""
    c = shape.BoundBox.Center
    return 'wheel_' + ('f' if c.y > 0 else 'r') + ('l' if c.x < 0 else 'r')


def led_name(shape):
    """led_1 ... led_6 or led_pilot for a 0603 diode on an LED's place (config.LEDS), else None."""
    c = shape.BoundBox.Center
    for name, (x, y) in R.led_places().items():
        if abs(c.x - x) < 0.6 and abs(c.y - y) < 0.6:
            return name
    return None


doc = FreeCAD.openDocument(FCSTD)
colours = appearances()
faces, done = {}, {}                              # (body, rgba) -> [faces]; (volume, bbox) -> label
for path, wheel in leaves(doc):
    o = path[-1]
    if o.Label in SKIP:
        continue
    sub = ''.join(x.Name + '.' for x in path[1:])
    s = Part.getShape(path[0], sub, needSubElement=False, transform=True)
    if s.isNull() or not s.Faces or s.Volume < 1e-3:
        continue
    b = s.BoundBox
    if not (NEAR[0] < b.Center.x < NEAR[1] and NEAR[2] < b.Center.y < NEAR[3]):
        continue
    key = (round(s.Volume, 3), tuple(round(v, 3) for v in (b.XMin, b.YMin, b.ZMin, b.XMax, b.YMax, b.ZMax)))
    if key in done:                               # the same solid twice (a link and its source both shown)
        continue
    done[key] = o.Label
    src = o.getLinkedObject(True) if o.TypeId == 'App::Link' else o
    c = colours.get(o.Name) or colours.get(src.Name) or [(204, 204, 204, 1.0)]
    for label, rgba in BLACK.items():
        if o.Label.startswith(label):
            c = [rgba]
    if o.TypeId == 'Part::Mirroring' and len(c) == 1:     # a mirror in one colour: its source's faces' (the right
        sc = colours.get(o.Source.Name, [])             # encoder's chip, the left one's mirror)
        if len(sc) == len(s.Faces) > 1:
            c = sc
    body = wheel_name(s) if wheel else 'robot'
    led = led_name(s) if o.Label.startswith('D_0603') else None
    for i, f in enumerate(s.Faces):
        if o.Label == PADS and not (f.Surface.__class__.__name__ == 'Plane' and abs(f.normalAt(0, 0).z) > 0.9):
            continue                              # its copper's 35 um edges: they do not show
        r, g, bl, a = c[i] if len(c) == len(s.Faces) else c[0]
        part = led if led and (r, g, bl) in LED_BODY else body
        rgba = (r, g, bl, round(a, 3)) if not (led and part == led) else (255, 255, 255, 1.0)
        faces.setdefault((part, rgba), []).append(f)

os.makedirs(OUT, exist_ok=True)
for f in os.listdir(OUT):
    if f.endswith('.stl'):
        os.remove(os.path.join(OUT, f))
parts = []
for n, ((body, rgba), fs) in enumerate(sorted(faces.items())):
    name = f'{body}_{n:02d}'
    mesh = Mesh.Mesh()
    lo = 0.0
    for size, linear, angular in COARSE:          # its small faces (pads, vias, the bearings' balls) coarser
        some = [f for f in fs if lo <= f.BoundBox.DiagonalLength < size]
        lo = size
        if some:
            mesh.addMesh(MeshPart.meshFromShape(Shape=Part.makeCompound(some), LinearDeflection=linear,
                                                AngularDeflection=angular))
    mesh.write(os.path.join(OUT, name + '.stl'))
    parts.append({'file': name + '.stl', 'body': body,
                  'rgba': [round(rgba[0] / 255, 3), round(rgba[1] / 255, 3), round(rgba[2] / 255, 3), rgba[3]]})
with open(os.path.join(OUT, 'manifest.json'), 'w') as fp:
    json.dump(parts, fp, indent=1)
print(f'{len(parts)} meshes in {OUT}')
for body in sorted({p['body'] for p in parts}):
    print(' ', body, sum(p['body'] == body for p in parts))
