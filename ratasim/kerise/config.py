"""KERISE v2 (kerikun11, github.com/kerikun11/micromouse-kerise-v2) in the simulator: where its repository is, its
geometry and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the axle between the
wheels, on the floor.

Its repository has no licence: nothing of it is copied here. The submodule external/kerise is read at build time:
its firmware built from it (fw/), its board from its KiCad and Gerber files and its motors' mounts from its STL
(cad.py).

Its firmware is for a half-size maze (config.h: HALF_SIZE, 90 mm cells, 6 mm walls; its classic branch no longer
builds), so it runs in one: the same maze at half its size (model.py)."""
import math
import os

from .. import paths

KERISE_DIR = os.environ.get('KERISE_DIR', os.path.join(paths.EXTERNAL, 'kerise'))
# KERISE v1's repository (the same author's, the same motors, wheels and gears): its wheels' STL (cad.py)
KERISE_V1_DIR = os.environ.get('KERISE_V1_DIR', os.path.join(paths.EXTERNAL, 'kerise-v1'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its meshes, its firmware's library (not committed)

# ---- Its maze: half size (its config.h: SEGMENT_WIDTH 90, WALL_THICKNESS 6; the rules: walls 25 mm high) ----------
CELL_MM = 90.0
WALL_MM = 6.0
WALL_H_MM = 25.0

# ---- Its board (kicad/KERISEv2.kicad_pcb): 1.0 mm thick; the robot's origin at (143, 134) in KiCad's frame (mm,
# y down), the middle of its wheels' cut-outs. Its front is KiCad's -y.
PCB_ORIGIN = (143.0, 134.0)
PCB_MM = 1.0
# Its motors' mounts (3dcad/-4digree.stl, two plates, inches): standing on the board, their foot along y, the wheel's
# axle through one hole 10.35 mm over the board and the motor's through another 12.1 mm over it, 12 mm ahead or
# behind (cad.py measures them): the board's top is the axle's height less 10.35, 1.8 mm over the floor.
MOUNT_AXLE_MM = 10.35
MOUNT_MOTOR_MM = 12.1
MOTOR_OFFSET_MM = 12.0          # its motors' axes from the wheels': the left wheel's motor ahead, the right one's
#                                 behind (in its photos: each motor's encoder by the other wheel, its ribbon cable down
#                                 to its header, P1 ahead on the right, P2 behind on the left)

# Its motors (Faulhaber 1717T006SR with its IEH2 encoder: 17 mm across, 17 mm long, the encoder 5.5 mm more): each
# with its face in its mount's recess, 9 mm from the middle, its encoder's end by the other wheel; their axes
# MOUNT_MOTOR_MM over the board. Its battery (two LG325-0120 cells, one on the other) on them.
MOTOR_R_MM = 8.5
MOTOR_FACE_X_MM = 9.0
MOTOR_LEN_MM = 17.0
ENCODER_LEN_MM = 5.5
CELL_HALF_MM = (12.5, 17.0, 1.65)       # each cell's half sizes
# ---- From its firmware (external/kerise/firmware/config.h) ------------------------------------------------------
WHEEL_DIAMETER_MM = 24.5
GEAR_RATIO = 4.0                # WHEEL_GEER_RATIO 0.25: four turns of the motor a turn of the wheel
ENCODER_COUNTS = 1024 * 4       # IEH2-1024 on each motor, counted x4
ROTATION_RADIUS_MM = 21.5       # MACHINE_ROTATION_RADIUS: half its track
TAIL_MM = 24.0                  # MACHINE_TAIL_LENGTH: its axle from its tail
WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2
BOARD_Z_MM = WHEEL_RADIUS_MM - MOUNT_AXLE_MM    # the board's top over the floor
TIRE_WIDTH_MM = 5.0
TRACK_X_MM = ROTATION_RADIUS_MM                 # its tyres' middles (its cut-outs: x 12 to 25 from the middle)
MOTOR_Z_MM = BOARD_Z_MM + MOUNT_MOTOR_MM        # its motors' axes over the floor
BATTERY_Z_MM = MOTOR_Z_MM + MOTOR_R_MM          # its battery's bottom

# IR sensors: an emitter (VSLY5850) over a photodiode (SFH213FA), both in the same footprint, lying on the board:
# where their lenses are and their axes (deg from +x counter-clockwise, 90 forward), from their footprints (KiCad
# LED-5MM_Angle: the body towards the footprint's -y, its lens 6 mm out). Firmware order (Reflector: its ADC
# channels 10-13): SL, FL, FR, SR.
IR_FOOTPRINTS = {'SL': ((130.0, 113.0), 80.0), 'FL': ((133.5, 110.0), 5.0),
                 'FR': ((152.5, 110.0), 355.0), 'SR': ((156.0, 113.0), 280.0)}
IR_LENS_MM = 6.0


def _sensor(at, rot):
    (px, py), a = at, math.radians(rot)
    dx, dy = -math.sin(a), math.cos(a)          # the footprint's -y, rotated, in the robot's frame
    x, y = px - PCB_ORIGIN[0], PCB_ORIGIN[1] - py
    return (round(x + IR_LENS_MM * dx, 2), round(y + IR_LENS_MM * dy, 2)), round(math.degrees(math.atan2(dy, dx)), 1)


IR_SENSORS = {n: _sensor(*v) for n, v in IR_FOOTPRINTS.items()}
IR_Z_MM = BOARD_Z_MM + 4.0          # the emitter's axis: lying on the photodiode, over the board

# ---- Assumed (its README and photos: two LG325-0120-1S cells in series, its firmware's battery check at 7.6 V of
# 8.2 V; the rest not measured) -------------------------------------------------------------------------------------
BATTERY_V = 8.0
MASS_KG = 0.050                     # two 1717 motors (about 18 g each), its board, battery, wheels, mounts
COM_MM = (0.0, 0.0, 14.0)            # its motors at 12.75 mm, its battery over them
YAW_INERTIA = 1.6e-5                # kg m^2: its motors across it, near the axle
TIRE_FRICTION = 1.0
# Faulhaber 1717T006SR (its datasheet's typical values): 3.4 ohm, 5.94 mNm/A; at the wheel through its 4:1 gears.
MOTOR_R_OHM = 3.4
MOTOR_KT = 0.00594 * GEAR_RATIO     # N m/A, V s/rad at the wheel
WHEEL_FRICTION_NM = 0.0006
WHEEL_ARMATURE = 0.6e-7 * GEAR_RATIO ** 2    # its rotor (0.6 g cm^2) through its 4:1 gears, at the wheel (kg m^2)
# Its IR sensors: the light back at a receiver in ADC counts, K / d^2 (d the beam's length to the wall, mm), so
# that it reads about 900 centred between two walls (its side sensors, 23 mm from them); the amplifier saturates
# at 2048 (its zero). A hand 30 mm in front of its front sensors reads about 550 (it waits for 400).
IR_K = 900.0 * 23.5 ** 2
IR_SPREAD_DEG = 3.0
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
    a beep and its LEDs show its battery. Its button: its mode, by turning a wheel (left still: mode 0, its search);
    its button again: its sensors on, it waits for a hand in front of its front sensors (both over 400). Then it
    calibrates its gyro and its sensors (the side walls of its start cell) for a second and goes."""
    return [('put_down', cell, heading_deg), ('say', 'switched on'), ('wait', 1500),
            ('say', 'its button: the mode (wheels still: 0, its search)'), ('press', 0, 100), ('wait', 600),
            ('say', 'its button: its sensors on'), ('press', 0, 100), ('wait', 600),
            ('say', 'a hand in front of its front sensors'), ('hand', ['FL', 'FR'], 300), ('wait', 1600)]
