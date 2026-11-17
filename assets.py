"""ZoroBot3's meshes for the simulator, built from its repository into zorobot/build/assets (not committed):
the main board from its KiCad PCB (kicad-cli 9: GLB with the silkscreen) and the printed parts from 3d_model/.

    python -m ratatron.sim.zorobot.assets
"""
import os
import struct
import subprocess

import numpy as np

from . import config as Z
from . import glb

ASSETS = os.path.join(Z.BUILD, 'assets')
PCB = os.path.join(Z.ZORO_DIR, 'pcb_files', 'kicad_project', 'ZoroBot3.kicad_pcb')
STL_DIR = os.path.join(Z.ZORO_DIR, '3d_model', 'stl_files')
PARTS = {   # our name: its file
    'bracket_l': 'Part Studio - Bracket Motor L.stl',
    'bracket_r': 'Part Studio - Bracket Motor R.stl',
    'fan_bracket': 'Part Studio - Bracket Ventilador.stl',
    'fan': 'Part Studio - Ventilador_fdm.stl',
    'wheel': 'Part Studio - Rueda.stl',
    'enc_gear': 'Part Studio - Spur gear (22 teeth).stl',
    'sensor_cover': 'Part Studio - Cover Sensor.stl',
}


def read_stl(path):
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:400]:
        v = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors='ignore').splitlines()
             if ln.strip().startswith('vertex')]
        return np.array(v, float).reshape(-1, 3, 3)
    n = struct.unpack('<I', data[80:84])[0]
    a = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return a['v'].astype(float).reshape(-1, 3, 3)


def write_stl(path, tris):
    tris = np.asarray(tris, np.float32).reshape(-1, 3, 3)
    rec = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'\0' * 80 + struct.pack('<I', len(tris)) + rec.tobytes())


def board():
    """The main board from the PCB: (board, silkscreen) triangles in the robot frame, mm."""
    out = os.path.join(Z.BUILD, 'ZoroBot3.glb')
    if not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(PCB):
        subprocess.run(['kicad-cli', 'pcb', 'export', 'glb', '-f', '--include-silkscreen', '-o', out, PCB],
                       check=True, stdout=subprocess.DEVNULL)
    board_t, silk_t = [], []
    for tris, _, name in glb.read(out):
        mm = tris * 1000.0      # metres, y up: KiCad (x, y) = (X, Z), height Y
        x = mm[..., 0] - Z.PCB_CENTRE_X
        y = Z.AXLE_Y_KICAD - mm[..., 2]
        z = mm[..., 1] + Z.PCB_BOTTOM_MM - Z.AXLE_HEIGHT_MM
        t = np.stack([x, y, z], axis=-1)
        cx = mm[..., 0].mean(axis=1)
        t = t[(cx > Z.PCB_KEEP_X[0]) & (cx < Z.PCB_KEEP_X[1])]
        if name.endswith('_PCB'):
            board_t.append(t)
        elif 'silkscreen' in name:
            silk_t.append(t)
    return np.concatenate(board_t), np.concatenate(silk_t)


def build():
    os.makedirs(ASSETS, exist_ok=True)
    b, s = board()
    write_stl(os.path.join(ASSETS, 'board.stl'), b)
    write_stl(os.path.join(ASSETS, 'silk.stl'), s)
    for name, f in PARTS.items():
        write_stl(os.path.join(ASSETS, name + '.stl'), read_stl(os.path.join(STL_DIR, f)))
    return ASSETS


def ensure():
    if not os.path.exists(os.path.join(ASSETS, 'board.stl')):
        build()
    return ASSETS


if __name__ == '__main__':
    print(build())
