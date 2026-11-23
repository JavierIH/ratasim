"""Ratatron II (JavierIH, github.com/JavierIH/ratatron2) in the simulator: where its repository is, its geometry and
the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin between its four wheels (the CAD's
origin: its motors' axes, half way between its axles), on the floor.

Its repository is the submodule external/ratatron2: its firmware (GPLv3), its board (KiCad 9) and its mechanics
(FreeCAD), both CC BY-SA 4.0. Its model: the meshes of ratatron2/assets, made from its FreeCAD file by cad.py.
Its firmware is not simulated yet (it is a test program: its LEDs, its button, a motor and fan test)."""
import os

from .. import paths

RATATRON2_DIR = os.environ.get('RATATRON2_DIR', os.path.join(paths.EXTERNAL, 'ratatron2'))
FCSTD = os.path.join(RATATRON2_DIR, 'mechanics', 'ratatron2.fcstd')
HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, 'assets')            # its meshes (cad.py), committed

# ---- Its board (hardware/ratatron2.kicad_pcb): 1.2 mm, its top at the CAD's z 0. The CAD's frame is KiCad's from
# its grid origin (200, 156), y up, moved 12 mm forward (notes' pcb_step.py: the board's outline centred in KiCad,
# its wheels' middle 12 mm behind it in the CAD).
KICAD_ORIGIN = (200.0, 156.0)
KICAD_TO_CAD_Y_MM = 12.0
PCB_MM = 1.2


def kicad_to_cad(x, y):
    return round(x - KICAD_ORIGIN[0], 3), round(KICAD_ORIGIN[1] - y + KICAD_TO_CAD_Y_MM, 3)


# Its LEDs, 0603: D2-D7 along its tail, yellow (its maker's choice; its schematic still says Blue), its firmware's
# LED1-LED6 (ui.h, board.h: PB14, PB15, PC6, PB8, PB9, PC0, each through its 470 ohm, R5-R10), and D1, the pilot,
# blue (+3V3_AUX, lit when it is on), on its nose. Their places on its board (KiCad, mm). From the left: D3 (LED2),
# D2 (LED1), D4 (LED3), D5 (LED4), D6 (LED5), D7 (LED6).
LEDS = {'led_1': ('D2', (173.965, 201.085)), 'led_2': ('D3', (168.25, 199.18)), 'led_3': ('D4', (179.68, 202.99)),
        'led_4': ('D5', (220.32, 202.99)), 'led_5': ('D6', (226.035, 201.085)), 'led_6': ('D7', (231.75, 199.18)),
        'led_pilot': ('D1', (186.792, 129.838))}
NUM_LEDS = 6


def led_places():
    """Each LED's centre in the CAD's frame (x, y mm)."""
    return {name: kicad_to_cad(*at) for name, (_ref, at) in LEDS.items()}


# ---- Its drivetrain (its FreeCAD file, measured on cad.py's meshes): four wheels, two each side, their axles 13 mm
# ahead and behind its middle, 8 mm over the board's top; each side's motor (17 mm across, 24 mm long, lying across
# the robot) with a 10-tooth pinion (module 0.5) between its two wheels' 42-tooth gears; M2 screws for axles, the
# wheels on two MR62 bearings each, their nuts outside.
TIRE_RADIUS_MM = 11.745         # its tyres: 23.5 mm across, 8.5 mm wide, their middles 29.8 mm off its middle
TIRE_WIDTH_MM = 8.5
TIRE_X_MM = 29.8
AXLE_Y_MM = 13.0
AXLE_Z_MM = 8.0                 # over the board's top
FLOOR_TO_CAD_MM = TIRE_RADIUS_MM - AXLE_Z_MM    # the board's top (the CAD's z 0) over the floor: 3.7 mm
GEAR = 42 / 10

# ---- Assumed (nothing weighed or measured yet) ----------------------------------------------------------------------
MASS_KG = 0.110                 # its board with its parts (15 g), two motors (2 x 18 g), its 2S battery (34 x 11 x
#                                 18.5 mm, 25 g), its wheels (4 x 4 g), fan and motor, printed supports, screws
WHEEL_KG = 0.004
COM_MM = (0.0, 2.0, 11.0)       # its battery behind its motors, its fan and nose ahead
INERTIA = (4.0e-5, 3.0e-5, 6.0e-5)     # kg m^2 about x, y, z
TIRE_FRICTION = 1.0
WHEEL_FRICTION_NM = 0.0005
WHEEL_ARMATURE = 0.7e-7 * GEAR ** 2 / 2  # its rotor's inertia (assumed 0.7 g cm^2) at each of a side's two wheels
