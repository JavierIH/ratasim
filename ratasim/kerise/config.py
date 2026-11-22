"""KERISE v1 (kerikun11, github.com/kerikun11/micromouse-kerise-v1) in the simulator: where its repository is, its
geometry and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the axle between the
wheels, on the floor.

Its repository has no licence; its author allows ratasim to use it: nothing of it is copied here. The submodule
external/kerise-v1 is read at build time: its firmware built from it (fw/), its board from its KiCad file, its
motors' mounts and its wheels from its STL files (cad.py).

The submodule is pinned at its firmware as it ran at the 37th All Japan Micromouse Contest (2016-11-19 and 20, its
classic Freshman class: 10th in the qualifying, 9th in the final): commit ab43c54 (2016-11-18, the day after
811b22e, "本番環境で動いた", it ran in the contest's own conditions). A classic micromouse: 180 mm cells."""
import math
import os

from .. import paths

KERISE_DIR = os.environ.get('KERISE_DIR', os.path.join(paths.EXTERNAL, 'kerise-v1'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its meshes, its firmware's library (not committed)

# ---- Its maze: classic (180 mm cells, 12 mm walls 50 mm high; its MoveAction's 180 and 90 mm moves) ---------------
CELL_MM = 180.0
WALL_MM = 12.0
WALL_H_MM = 50.0

# ---- Its board (kicad/Mouse-01.kicad_pcb): 1.0 mm thick, 74 by 72 mm: a half disc 37 mm across its nose, its wheels
# in two cut-outs (x 24 to 37 from its middle, 22 mm long), its tail's corners rounded. The robot's origin at (143,
# 99) in KiCad's frame (mm, y down), the middle of its wheels' cut-outs; its front is KiCad's -y.
PCB_ORIGIN = (143.0, 99.0)
PCB_MM = 1.0
# Its motors' mounts (3dprint/mounter-4set.stl, four plates 28 by 17.5, 4 mm thick, mm): each standing on the board
# along y over its two screws (KiCad's holes at x +-22, y 0 and 22), its wheel's axle through a hole 9.1 mm over the
# board at y 0, its motor's face in a recess on its inner face, the motor's boss through a hole 8.5 mm over the board
# at y 11.5 (cad.py measures them): the board's top is the axle's height less 9.1, 3.2 mm over the floor.
MOUNT_X_MM = 22.0               # the plates' middles (their screws), 4 mm thick
MOUNT_AXLE_MM = 9.1
MOUNT_MOTOR_MM = 8.5
MOTOR_OFFSET_MM = 11.5          # both motors ahead of the axle, on one line (its photo: one black cylinder between
#                                 the mounts, their encoders meeting in the middle, their ribbon cables to P1 and P2)

# Its motors (Faulhaber 1717T006SR with its IEH2 encoder: 17 mm across, 17 mm long, the encoder 4.5 mm more): each
# from its face in its mount's recess (2 mm deep) inwards, its encoder by the other's
MOTOR_R_MM = 8.5
MOTOR_FACE_X_MM = MOUNT_X_MM
MOTOR_LEN_MM = 17.0
ENCODER_LEN_MM = 4.5
CELL_HALF_MM = (17.0, 6.0, 1.6)         # each of its two cells' half sizes (LG325-0120), side by side on its motors

# ---- From its firmware (external/kerise-v1/firmware/config.h) ---------------------------------------------------
WHEEL_DIAMETER_MM = 24.665
GEAR_RATIO = 4.0                # WHEEL_GEER_RATIO 0.25: four turns of the motor a turn of the wheel
ENCODER_COUNTS = 1024 * 4       # IEH2-1024 on each motor, counted x4
ROTATION_RADIUS_MM = 33.0       # MACHINE_ROTATION_RADIUS: half its track
TAIL_MM = 24.0                  # its START_STEP (90 - 24 - 6): its axle from its tail (its board: 24 mm)
WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2
BOARD_Z_MM = WHEEL_RADIUS_MM - MOUNT_AXLE_MM    # the board's top over the floor
TIRE_WIDTH_MM = 5.0
TRACK_X_MM = ROTATION_RADIUS_MM                 # its tyres' middles (its cut-outs: x 24 to 37 from the middle)
MOTOR_Z_MM = BOARD_Z_MM + MOUNT_MOTOR_MM        # its motors' axes over the floor
BATTERY_Z_MM = MOTOR_Z_MM + MOTOR_R_MM          # its battery's bottom

# IR sensors: four pairs of an emitter (VSLY5850, clear) and a photodiode (SFH213FA, black) side by side, lying on the
# board (KiCad's LED-5MM_Angle and Photo-5MM_Angle: their bodies towards the footprint's -y, their lenses 6 mm out).
# Its front pair at its flanks looking ahead 15 deg out, its side pair by its nose looking sideways 15 deg ahead. The
# beam from the emitter's lens along its axis (deg from +x counter-clockwise, 90 forward). Firmware order (Reflector:
# ADC channels 10-13, its emitters PB14 for SL and FR, PB15 for SR and FL): SL, FL, FR, SR.
IR_FOOTPRINTS = {'SL': ((139.19, 58.36), 75.0), 'FL': ((112.52, 85.67), 15.0),      # D2, D8
                 'FR': ((173.48, 86.3), 345.0), 'SR': ((146.81, 58.36), 285.0)}    # D3, D9
IR_LENS_MM = 6.0


def _sensor(at, rot):
    (px, py), a = at, math.radians(rot)
    dx, dy = -math.sin(a), math.cos(a)          # the footprint's -y, rotated, in the robot's frame
    x, y = px - PCB_ORIGIN[0], PCB_ORIGIN[1] - py
    return (round(x + IR_LENS_MM * dx, 2), round(y + IR_LENS_MM * dy, 2)), round(math.degrees(math.atan2(dy, dx)), 1)


IR_SENSORS = {n: _sensor(*v) for n, v in IR_FOOTPRINTS.items()}
IR_Z_MM = BOARD_Z_MM + 2.5          # their axes: 5 mm parts lying on the board

# ---- Assumed (its README and photo: two LG325-0120 cells in series, its firmware's battery check at 7.6 V; its
# weight from its parts; the rest not measured) ----------------------------------------------------------------------
BATTERY_V = 8.0
MASS_KG = 0.060                     # two 1717 motors with encoders (about 15 g each), its board (9 g), cells (7 g),
#                                     wheels, mounts and parts
COM_MM = (0.0, 8.0, 10.0)           # its motors and most of its board ahead of the axle
YAW_INERTIA = 2.2e-5                # kg m^2: its motors (8e-6), board (8e-6), wheels (4.4e-6), cells
TIRE_FRICTION = 1.5                 # its tyres' grip (urethane)
TIRE_SINK_MM = 0.08                 # how far its tyres sink into the floor (MuJoCo's soft contact): their collision
#                                     cylinders that much bigger, so that they roll on its firmware's wheel diameter
# Faulhaber 1717T006SR (its datasheet's typical values): 3.4 ohm, 5.94 mNm/A; at the wheel through its 4:1 gears.
MOTOR_R_OHM = 3.4
MOTOR_KT = 0.00594 * GEAR_RATIO     # N m/A, V s/rad at the wheel
WHEEL_FRICTION_NM = 0.0006
WHEEL_ARMATURE = 0.6e-7 * GEAR_RATIO ** 2    # its rotor (0.6 g cm^2) through its 4:1 gears, at the wheel (kg m^2)
# Its IR sensors: the light back at a receiver in ADC counts, K / d^IR_EXP (d the beam's length to the wall, mm),
# times the cosine between the beam and the wall's face (a matte face), so that its side sensors read about 900
# centred in a cell (their beams 77 mm long); the amplifier saturates at 2048 (its zero). Its WallDetector tells
# walls from its start cell's side walls: a side wall over a third of them, a front wall over (SL + SR) * 0.56 / 3;
# it looks at a cell's walls from its middle. A hand 30 mm in front of its front left sensor saturates it (it waits
# for over 300).
IR_EXP = 2.0
IR_K = 900.0 * 77.0 ** IR_EXP
IR_SPREAD_DEG = 2.0                 # its beams' spread (sigma; its VSLY5850s: +-3 deg at half intensity)
IR_NOISE = 0.01                     # of the reading
# Its MPU6500: the real bias of its z gyro (its firmware measures it standing still) and the noise
GYRO_BIAS_DPS = 0.3
GYRO_NOISE_DPS = 0.1
ACCEL_NOISE_G = 0.01

# ---- Its start (ratasim/start.py) ---------------------------------------------------------------------------------
START_BACK_MM = CELL_MM / 2 - WALL_MM / 2 - TAIL_MM - 0.5     # its axle behind the cell's middle: its tail at the back wall


def counts_per_rad():
    """Its encoders' counts per radian of the wheel."""
    return ENCODER_COUNTS * GEAR_RATIO / (2 * math.pi)


def start(cell, heading_deg):
    """As its firmware starts a search (main.cpp): switched on in the start cell, its tail against the back wall;
    a beep and its LEDs show its battery. Its button: its speed, by turning a wheel (left still: 600 mm/s); its
    button again: its sensors on, it waits for a hand in front of its front left sensor (over 300). Then its
    MazeSolver: it calibrates its gyro and its sensors (the side walls of its start cell) and goes."""
    return [('put_down', cell, heading_deg), ('say', 'switched on'), ('wait', 1500),
            ('say', 'its button: its speed (wheels still: 600 mm/s)'), ('press', 0, 100), ('wait', 600),
            ('say', 'its button: its sensors on'), ('press', 0, 100), ('wait', 600),
            ('say', 'a hand in front of its front sensors'), ('hand', ['FL', 'FR'], 300), ('wait', 2000)]
