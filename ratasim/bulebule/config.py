"""Bulebule (Bulebots, github.com/Bulebots/bulebule) in the simulator: its geometry and the physics assumed for it.
Robot frame (mm): x right, y forward, z up, origin on the floor under the motors' axis (between its axles).

Sources: its firmware (src/setup.h, src/config.h), its CadQuery parts (3d/mount.py, 3d/rim.py), its board's layout
(a screenshot of its Upverter design in Bulebots' 2018 talk, figures/pcb.jpg: the outline fitted on it, pcb.py)
and its photos (docs/source/figures). The board's design files are gone with Upverter.
"""
import math
import os

from .. import paths

BULEBULE_DIR = os.environ.get('BULEBULE_DIR', os.path.join(paths.EXTERNAL, 'bulebule'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')

# ---- From its firmware ----------------------------------------------------------------------------------------
MASS_KG = 0.110                     # MOUSE_MASS
INERTIA_Z = 0.000125                # MOUSE_MOMENT_OF_INERTIA, kg m^2
WHEELS_SEPARATION = 65.0            # MOUSE_WHEELS_SEPARATION: left to right
AXIS_SEPARATION = 22.3              # MOUSE_AXIS_SEPARATION: its front and rear axles, each side
HEAD, TAIL = 57.0, 37.0             # MOUSE_MOTOR_HEAD, MOUSE_MOTOR_TAIL: from the motors' axis
MICROMETERS_PER_COUNT = 8.32        # config.h; IE2-512 encoders (2048 counts a turn), gears 15:60
GEAR = 60 / 15
WHEEL_RADIUS = 2048 * GEAR * MICROMETERS_PER_COUNT / 1000 / (2 * math.pi)     # 10.85

REFERENCE_Y = AXIS_SEPARATION / 2   # its position's point: MOUSE_TAIL = MOUSE_MOTOR_TAIL + MOUSE_AXIS_SEPARATION / 2
COUNTS_PER_REV = 2048               # IE2-512, both edges of both channels
PWM_PERIOD = 1024                   # DRIVER_PWM_PERIOD
MOTOR_DRIVER_V = 9.0                # MOTOR_DRIVER_INPUT_VOLTAGE: its DC-DC converters' output
WALL_WIDTH = 12.0                   # WALL_WIDTH
# Its sensors' calibration (config.h): distance (m, from its position's point) = A / ln(lit - dark) - B. Its side
# distances to the walls' faces (MIDDLE_MAZE_DISTANCE, centred: 84 mm), its front ones to the walls' middle
# (keep_front_wall_distance(CELL_DIMENSION / 2) stops it centred in a cell): its own frame.
SENSOR_AB = {'SL': (2.806, 0.287), 'SR': (2.327, 0.231), 'FL': (2.609, 0.242), 'FR': (2.713, 0.258)}
GYRO_LSB_PER_DPS = 16.4             # MPU_GYRO_SENSITIVITY_2000_DPS

# ---- From its CadQuery parts ------------------------------------------------------------------------------------
MODULE, Z_PINION, Z_GEAR = 0.3, 15, 60
MOTOR_D = 15.0                      # Faulhaber, with IE2-512 encoders
MOTOR_SHIFT = 1.5                   # the motor's axis over the wheels' axles
MOUNT = (6.0, 40.0, 15.0)           # x, y, z: on the board's waist edges, the motor through its middle
RIM_D, RIM_W, HUB_D, HUB_W = 19.0, 5.0, 14.8, 3.0
CONTACT_W = 6.0                     # the tires' collision width, centred on the firmware's wheel separation
BEARING_D = 6.2
RIM_HOLES, RIM_HOLE_D = 9, 2.5      # MINIHOLES: its face's holes, on the circle between hub and bearing
RIM_HOLE_R = (HUB_D + BEARING_D) / 4
RIM_RECESS = 1.0                    # the rim's face inside the tire (photos)
BOLT_HEAD = (5.5, 2.0)              # the wheels' M3 hex-head bolts (photos): across flats, height
WASHER_D = 7.0

# ---- From its board's layout (pcb.py) ----------------------------------------------------------------------------
PCB_THICK = 1.6
WAIST = 28.1                        # the board's waist, where the mounts stand: half width
SHOULDER_X, TAB_X = 41.8, 40.0      # the board's shoulders and rear tabs (pcb.py): half width
TIRE_X = WAIST + 1.4                # the tires' inner side, past the gears
TIRE_W = TAB_X - TIRE_X             # 10.5: the tires flush with the board's edge by them (photos)
AXLE_Z = WHEEL_RADIUS               # the wheels' axles over the floor
MOTOR_Z = AXLE_Z + MOTOR_SHIFT      # its motors' axis
PCB_TOP = MOTOR_Z - MOUNT[2] / 2    # the mounts stand on the board
PCB_BOTTOM = PCB_TOP - PCB_THICK    # 3.25 mm over the floor

# ---- Assumed (nothing measured) ---------------------------------------------------------------------------------
TIRE_FRICTION = 1.0
BATTERY = (38.0, 30.0, 8.0)         # a LiPo on its motors (photos); 1S by its firmware's limits (3.3 V)
BATTERY_V = 3.9                     # read by ADC2 through its divider by 2 (voltage.c)
# Its motors: Faulhaber 15 mm coreless with IE2-512 encoders (its docs name the maker and the encoder, not the
# motor): a 6 V one, 10 000 rpm free and 1.5 A stall at 6 V, 30 mA free; driven from its 9 V converters.
MOTOR_KE = 6.0 / (10000 * 2 * math.pi / 60)    # V s/rad
MOTOR_R_OHM = 6.0 / 1.5
MOTOR_FRICTION_NM = MOTOR_KE * 0.030
MOTOR_ROTOR_INERTIA = 6.5e-8                    # kg m^2
GEAR_EFFICIENCY = 0.8
# Its IR pairs: lit minus dark, its calibration inverted for each ray's hit (the distance its firmware would give
# it), the beam's mean: a wall filling the beam reads as its calibration says, a post less, as it fills less of it
IR_DARK = 120                       # counts, emitter off (the maze's lighting)
IR_NOISE = 0.01                     # of the light back (gaussian), and 3 counts on every reading
IR_FAR_MM = 400.0
IR_BEAM_DEG = 4.0                   # its beam's profile, gaussian: rays at 0, +-1, +-2 sigma, weighed by it
GYRO_NOISE_LSB = 2.6                # 0.01 dps/sqrt(Hz) over its 250 Hz band


# ---- Its start (main.c, mmlib's hmi.c; ratasim/start.py) ----------------------------------------------------------
SHORT_MS, LONG_MS = 150, 700        # its button: a press under 0.3 s is short, from 0.3 s long (hmi.c)


def start(run=False, saved=False, force_steps=3):
    """As its makers do, with its button and a hand: a press wakes it; with a maze saved, short reuses it (a run),
    long discards it; then long picks the classic goal (the maze's centre), short presses raise the force (0.1 N,
    +0.05 a press) and long takes it; it blinks for 2 s and waits 5 s more, its LEDs on: a hand in front, taken
    away; 2 s later it calibrates (in its start cell, its back to the wall) and goes."""
    s = [('say', 'switched on, in its start cell, its back to the wall'), ('wait', 500),
         ('say', 'its button: wake'), ('press', 'user', SHORT_MS), ('wait', 400)]
    if saved:
        if run:
            s += [('say', 'short: the saved maze, a run'), ('press', 'user', SHORT_MS), ('wait', 400)]
        else:
            s += [('say', 'long: the saved maze discarded'), ('press', 'user', LONG_MS), ('wait', 1200)]
    elif run:
        raise ValueError('no maze saved: explore first')
    if not run:
        s += [('say', 'long: the classic goal'), ('press', 'user', LONG_MS), ('wait', 3500)]
    s += [('wait', 600)]
    for _ in range(force_steps):
        s += [('press', 'user', SHORT_MS), ('wait', 400)]
    s += [('say', f'force {0.1 + 0.05 * force_steps:.2f} N: long'), ('press', 'user', LONG_MS), ('wait', 1500),
          ('say', 'it blinks, then waits'), ('wait', 7600),
          ('say', 'a hand in front, then away'), ('hand', ['FL', 'FR'], 400), ('wait', 2600)]
    return s
