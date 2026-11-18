"""Gamusino (turmandreams, github.com/turmandreams/gamusino) in the simulator: where its 3D model is and the physics
assumed for it. Its geometry comes from the model (cad.py, build/cad/geometry.json). Robot frame (mm): x right,
y forward, z up, origin on the axle."""
import os

from .. import paths

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')
# Its model: the Blender export in its repository (the submodule external/gamusino)
CAD_GLB = os.environ.get('GAMUSINO_GLB', os.path.join(paths.EXTERNAL, 'gamusino', 'gamusino_3d', 'gamusino.glb'))
CAD_MESHES = os.path.join(BUILD, 'cad')

# ---- Assumed physics (nothing measured) --------------------------------------------------------------------------
MASS_KG = 0.090             # two LiPo cells, two micro gearmotors, a suction motor, ESP32 board, printed chassis
COM_MM = (0.0, 2.0, 4.0)
TIRE_FRICTION = 1.0
