"""Perenquen's 3D model from its FreeCAD assembly (external/perenquen/Mechanics, A2plus), to perenquen/assets:

    ~/programs/FreeCAD_*.AppImage freecadcmd ratasim/perenquen/cad.py

Its robot assembly (assemblies/robot_assembly.fcstd) holds the main board, the right encoder board and the right
motor block where its designer placed them, each one shape with its faces' colours. The motor block's shape is its
sub-assembly's parts' faces in order (motor_block_assembly.fcstd): split back into them, its two wheels (with their
gears) become bodies of their own. The main board's shape is a fusion of its KiCad StepUp export (parts/mainboard.fcstd)
in one colour: its parts are taken from that file in theirs, placed as the assembly places the board. The left side
is the right one mirrored (the assembly's `_mirror` constraints: its designer drew one side).

Out (perenquen/assets, committed: CC BY-SA 4.0, as its CAD): one STL per body and colour (assembly frame, mm) and manifest.json: [{file, body, rgba}], body in
chassis, wheel_fr, wheel_rr, wheel_fl, wheel_rl.
"""
import json
import os
import struct
import xml.etree.ElementTree as ET
import zipfile

import FreeCAD
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
MECH = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'external', 'perenquen', 'Mechanics')
OUT = os.path.join(HERE, 'assets')
ROBOT = os.path.join(MECH, 'assemblies', 'robot_assembly.fcstd')
BLOCK = os.path.join(MECH, 'assemblies', 'motor_block_assembly.fcstd')
BOARD = os.path.join(MECH, 'parts', 'mainboard.fcstd')
WHEELS = {'wheel_front_assembly_01': 'wheel_f', 'wheel_rear_assembly_01': 'wheel_r'}


def colours(path):
    """ViewProvider name -> per-face colours [(r, g, b, a)] (DiffuseColor lists, or its ShapeColor)."""
    z = zipfile.ZipFile(path)
    out = {}
    for vp in ET.fromstring(z.read('GuiDocument.xml')).iter('ViewProvider'):
        for p in vp.iter('Property'):
            name = p.get('name')
            if name == 'DiffuseColor':
                d = z.read(p.find('ColorList').get('file'))
                n = struct.unpack('<I', d[:4])[0]
                out[vp.get('name')] = [unpack(struct.unpack('<I', d[4 + 4 * i:8 + 4 * i])[0]) for i in range(n)]
            elif name == 'ShapeColor' and vp.get('name') not in out:
                c = p.find('PropertyColor')
                if c is not None:
                    out.setdefault(vp.get('name') + '#shape', [unpack(int(c.get('value')))])
    for k in [k for k in out if k.endswith('#shape')]:
        out.setdefault(k[:-6], out.pop(k))
    return out


def unpack(v):
    r, g, b, t = (v >> 24) & 255, (v >> 16) & 255, (v >> 8) & 255, v & 255
    return (r / 255, g / 255, b / 255, 1 - t / 255)


def face_colour(cols, i):
    if not cols:
        return (0.6, 0.6, 0.6, 1.0)
    return cols[i] if i < len(cols) else cols[0]


class Writer:
    def __init__(self):
        self.parts = {}         # (body, rgba) -> [faces]

    def add(self, body, face, rgba):
        key = (body, tuple(round(c, 3) for c in rgba))
        self.parts.setdefault(key, []).append(face)

    def write(self):
        os.makedirs(OUT, exist_ok=True)
        for f in os.listdir(OUT):
            os.unlink(os.path.join(OUT, f))
        manifest = []
        for i, ((body, rgba), faces) in enumerate(sorted(self.parts.items())):
            mesh = MeshPart.meshFromShape(Shape=Part.Compound(faces), LinearDeflection=0.1, AngularDeflection=0.5,
                                          Relative=False)
            name = f'{body}_{i:02d}.stl'
            mesh.write(os.path.join(OUT, name))
            manifest.append({'file': name, 'body': body, 'rgba': list(rgba), 'faces': len(faces)})
        json.dump(manifest, open(os.path.join(OUT, 'manifest.json'), 'w'), indent=1)
        print(f'{len(manifest)} meshes to {OUT}')


def main():
    w = Writer()
    mirror = FreeCAD.Matrix()
    mirror.scale(1, -1, 1)
    robot = FreeCAD.openDocument(ROBOT)
    rcols = colours(ROBOT)

    # the main board: its StepUp parts in their colours, placed as the assembly places the board
    board = FreeCAD.openDocument(BOARD)
    bcols = colours(BOARD)
    place = robot.getObject('mainboard_01').Placement
    for o in board.Objects:
        if o.TypeId != 'Part::Feature' or o.Shape.isNull():
            continue
        s = o.Shape.copy()
        s.Placement = place.multiply(s.Placement)
        cols = bcols.get(o.Name)
        for i, f in enumerate(s.Faces):
            w.add('chassis', f, face_colour(cols, i))
    FreeCAD.closeDocument(board.Name)

    # the right encoder board, both sides
    enc = robot.getObject('encoderboard_rigth_01')
    for i, f in enumerate(enc.Shape.Faces):
        c = face_colour(rcols.get(enc.Name), i)
        w.add('chassis', f, c)
        w.add('chassis', f.transformGeometry(mirror), c)

    # the motor block, split into its sub-assembly's parts, both sides
    block = FreeCAD.openDocument(BLOCK)
    counts = [(o.Name, len(o.Shape.Faces)) for o in block.Objects
              if o.TypeId == 'Part::FeaturePython' and not o.Shape.isNull() and len(o.Shape.Faces)]
    FreeCAD.closeDocument(block.Name)
    mb = robot.getObject('motor_block_assembly_01')
    faces, cols = mb.Shape.Faces, rcols.get(mb.Name)
    assert sum(n for _, n in counts) == len(faces), (counts, len(faces))
    k = 0
    for name, n in counts:
        body = WHEELS.get(name, 'chassis')
        for i in range(k, k + n):
            c = face_colour(cols, i)
            w.add(body + ('r' if body != 'chassis' else ''), faces[i], c)
            w.add(body + ('l' if body != 'chassis' else ''), faces[i].transformGeometry(mirror), c)
        k += n
    w.write()


main()
