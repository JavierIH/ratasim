"""Fujin's meshes from its STEP (boards/Fujin_V1/Fujin_Body_Full.step in its repository: its board with every
component and its mechanics): every part where the STEP puts it, tessellated with gmsh (as ZoroBot3's: zorobot/cad.py)
and merged per moving body and colour, in the robot frame (mm: x right, y forward, z up, origin on the axle between
the wheels, found from the two tyres).

    python -m ratasim.fujin.cad [STEP OUT_DIR]

The simulator calls ensure(), which needs gmsh only when the meshes are missing or older than the STEP.

Writes OUT_DIR/<body>_<n>.stl and OUT_DIR/manifest.json: [{"file", "body", "rgba", "parts"}], body in chassis,
wheel_l, wheel_r.
"""
import json
import os
import sys

import numpy as np

from . import config as F

WHEEL_PARTS = ('pneu_miniz', 'MM_wheel', 'spur_gear', 'MM_Encoder_hub')     # turn with the wheel
TYRE = 'pneu_miniz'


def body(name, tris):
    if any(k in name for k in WHEEL_PARTS):
        return 'wheel_l' if tris[..., 0].mean() < 0 else 'wheel_r'
    return 'chassis'


def build(step=F.CAD_STEP, out=F.CAD_MESHES):
    from ..zorobot.cad import tessellate, write_stl
    parts = tessellate(step)
    tyres = [t.reshape(-1, 3) for n, _, t in parts if TYRE in n]
    if len(tyres) != 2:
        raise SystemExit(f'Fujin: expected two tyres ({TYRE}) in its STEP, found {len(tyres)}')
    axle = np.mean([(v.min(0) + v.max(0)) / 2 for v in tyres], axis=0)
    if np.abs(axle - F.AXLE_IN_STEP_MM).max() > 0.5:
        raise SystemExit(f'Fujin: its tyres centred at {axle.round(2)}, not at {F.AXLE_IN_STEP_MM} (config.py)')
    groups = {}
    for name, rgb, t in parts:
        t = t - axle
        groups.setdefault((body(name, t), rgb), []).append((name, t))
    os.makedirs(out, exist_ok=True)
    manifest = []
    for i, ((b, rgb), items) in enumerate(sorted(groups.items())):
        f = f'{b}_{i:02d}.stl'
        write_stl(os.path.join(out, f), np.concatenate([t for _, t in items]))
        manifest.append({'file': f, 'body': b, 'rgba': [round(c / 255, 3) for c in rgb] + [1],
                         'parts': sorted({n.split('/')[-1] for n, _ in items})})
    json.dump(manifest, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
    print(f'{len(parts)} parts, {sum(len(t) for *_, t in parts)} triangles, {len(manifest)} meshes in {out}')
    return out


def ensure(step=F.CAD_STEP, out=F.CAD_MESHES):
    """The meshes' directory, built first if they are missing or older than the STEP."""
    m = os.path.join(out, 'manifest.json')
    if os.path.exists(m) and (not os.path.exists(step) or os.path.getmtime(m) >= os.path.getmtime(step)):
        return out
    if not os.path.exists(step):
        raise SystemExit(f'Fujin: its STEP is missing ({step}): git submodule update --init external/fujin')
    try:
        import gmsh     # noqa: F401
    except ImportError:
        raise SystemExit('Fujin: gmsh is needed to make its meshes from its STEP (pip install -e .[cad]).')
    print('Fujin: its meshes from its STEP (once, about two minutes)', flush=True)
    build(step, out)
    return out


if __name__ == '__main__':
    build(*sys.argv[1:3])
