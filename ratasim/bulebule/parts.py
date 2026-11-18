"""The 3D models of Bulebule's components from KiCad's library (kicad-packages3D, CC BY-SA 4.0 with an exception for
the designs that use them), downloaded and tessellated at build time (gmsh, `pip install -e .[cad]`), centred in x and y, mm. KiCad's z is kept:
z = 0 is the board's surface, so through-hole parts sit on it with their legs through the board.
"""
import os
import urllib.request

import numpy as np

from . import config as B

KICAD = 'https://gitlab.com/kicad/libraries/kicad-packages3D/-/raw/master/'
PARTS = {
    'led5': 'LED_THT.3dshapes/LED_D5.0mm.step',                            # the IR emitters and receivers
    'lqfp48': 'Package_QFP.3dshapes/LQFP-48_7x7mm_P0.5mm.step',            # the Blue Pill's STM32F103C8
    'hc49': 'Crystal.3dshapes/Crystal_SMD_HC49-SD.step',
    'usb': 'Connector_USB.3dshapes/USB_Micro-B_Molex_47346-0001.step',
    'sot223': 'Package_TO_SOT_SMD.3dshapes/SOT-223.step',
    'hdr20': 'Connector_PinHeader_2.54mm.3dshapes/PinHeader_1x20_P2.54mm_Vertical.step',
    'hdr8': 'Connector_PinHeader_2.54mm.3dshapes/PinHeader_1x08_P2.54mm_Vertical.step',
    'hdr4': 'Connector_PinHeader_2.54mm.3dshapes/PinHeader_1x04_P2.54mm_Vertical.step',
    'hdr3': 'Connector_PinHeader_2.54mm.3dshapes/PinHeader_1x03_P2.54mm_Vertical.step',
    'hdr2x3': 'Connector_PinHeader_2.54mm.3dshapes/PinHeader_2x03_P2.54mm_Vertical.step',
    'sock6': 'Connector_PinSocket_2.54mm.3dshapes/PinSocket_1x06_P2.54mm_Vertical.step',
    'tl3342': 'Button_Switch_SMD.3dshapes/SW_SPST_TL3342.step',
    'pts810': 'Button_Switch_SMD.3dshapes/SW_SPST_PTS810.step',
    'ssop24': 'Package_SO.3dshapes/SSOP-24_5.3x8.2mm_P0.65mm.step',       # TB6612FNG
    'qfn24': 'Package_DFN_QFN.3dshapes/QFN-24-1EP_3x3mm_P0.4mm_EP1.75x1.6mm.step',      # MPU-6500
    'jst2': 'Connector_JST.3dshapes/JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical.step',
    'sot23': 'Package_TO_SOT_SMD.3dshapes/SOT-23.step',
    'sot23_6': 'Package_TO_SOT_SMD.3dshapes/SOT-23-6.step',
    'r0805': 'Resistor_SMD.3dshapes/R_0805_2012Metric.step',
    'r1206': 'Resistor_SMD.3dshapes/R_1206_3216Metric.step',
    'c0805': 'Capacitor_SMD.3dshapes/C_0805_2012Metric.step',
    'led0805': 'LED_SMD.3dshapes/LED_0805_2012Metric.step',
    'led1206': 'LED_SMD.3dshapes/LED_1206_3216Metric.step',
    'srn4018': 'Inductor_SMD.3dshapes/L_Bourns-SRN4018.step',
}
CACHE = os.path.join(B.BUILD, 'parts')


def tessellate(step):
    """[(rgb, triangles)] of a STEP, by its faces' colours (KiCad's models colour faces, not volumes)."""
    import gmsh
    from ..zorobot.cad import outward
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    gmsh.option.setString('Geometry.OCCTargetUnit', 'MM')
    gmsh.option.setNumber('Geometry.OCCImportLabels', 1)
    gmsh.model.occ.importShapes(step)
    gmsh.model.occ.synchronize()
    gmsh.option.setNumber('Mesh.MeshSizeMin', 0.05)
    gmsh.option.setNumber('Mesh.MeshSizeMax', 1.5)
    gmsh.option.setNumber('Mesh.MeshSizeFromCurvature', 10)
    gmsh.model.mesh.generate(2)
    tags, xyz, _ = gmsh.model.mesh.getNodes()
    index = np.zeros(int(tags.max()) + 1, int)
    index[tags.astype(int)] = np.arange(len(tags))
    xyz = xyz.reshape(-1, 3)
    by_colour = {}
    for d, v in gmsh.model.getEntities(3):
        faces, colours = [], []
        for _, sface in gmsh.model.getBoundary([(d, v)], oriented=False):
            types, _, nodes = gmsh.model.mesh.getElements(2, abs(sface))
            for ty, nd in zip(types, nodes):
                if ty == 2:
                    faces.append(index[nd.astype(int).reshape(-1, 3)])
                    colours.append(gmsh.model.getColor(2, abs(sface))[:3])
        for tris, rgb in zip(outward(faces, xyz), colours):
            by_colour.setdefault(rgb, []).append(tris)
    gmsh.finalize()
    return [(rgb, np.concatenate(t)) for rgb, t in by_colour.items()]


def load(name):
    """[(rgb, triangles (n, 3, 3) mm)] of a part, centred in x and y, z = 0 on the board."""
    npz = os.path.join(CACHE, name + '.z0.npz')
    if not os.path.exists(npz):
        os.makedirs(CACHE, exist_ok=True)
        step = os.path.join(CACHE, name + '.step')
        if not os.path.exists(step):
            urllib.request.urlretrieve(KICAD + PARTS[name], step)
        groups = tessellate(step)
        allv = np.concatenate([t.reshape(-1, 3) for _, t in groups])
        lo, hi = allv.min(axis=0), allv.max(axis=0)
        shift = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0])
        np.savez_compressed(npz, rgb=np.array([c for c, _ in groups]),
                            **{f't{i}': t - shift for i, (_, t) in enumerate(groups)})
    z = np.load(npz)
    return [(tuple(int(v) for v in z['rgb'][i]), z[f't{i}']) for i in range(len(z['rgb']))]
