"""Miniskybot 2 (Juan González-Gómez, Obijuan, and others: github.com/Obijuan/Miniskybot) in the simulator: where its
repository is, its geometry and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the
floor under its wheels' axle.

Sources: its repository (the submodule external/Miniskybot, Obijuan's last commit, October 2013; GPL or CC BY-SA,
its README): its OpenSCAD model (parts/Openscad: configuration.scad's dimensions, main.scad's assembly, show_robot())
and the STL files it ships (its chassis, battery holder, servo wheel and the Futaba S3003); its examples
(software/raw_examples: its servos' pins and how they are driven); its wiki page (iearobotics.com, Miniskybot 2: its
parts list, the SRF02 ultrasonic ranger at its front, four AAA cells) and the Skymega board's (its ATmega328 at
16 MHz, its pin table, its schematic: the test button to ground on D12, the test LED on D13, the I2C bus's 4K7
pull-ups).

Its program is ours (fw/miniskybot/miniskybot.ino): its authors publish none for a maze.
"""
import math
import os

from .. import paths

MINISKYBOT_DIR = os.environ.get('MINISKYBOT_DIR', os.path.join(paths.EXTERNAL, 'Miniskybot'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its program's library, its meshes
STL = {     # its STL files, as its repository ships them (OpenSCAD's ASCII)
    'chassis': os.path.join('parts', 'STL', 'chassis.stl'),
    'battery_holder': os.path.join('parts', 'STL', 'battery_holder.stl'),
    'wheel': os.path.join('parts', 'STL', 'Servo-wheel-4-arm-horn.stl'),
    'servo': os.path.join('parts', 'futaba3003', 'futaba.stl'),
}

# ---- From its OpenSCAD model (configuration.scad; main.scad's frame: x forward, y left, z up, mm) ----------------
SERVO_C1, SERVO_C3, SERVO_C4, SERVO_C5, SERVO_C6, SERVO_C7 = 20, 26.4, 40.3, 30.3, 10, 11.6
BATTERY_C1, BATTERY_C3, BATTERY_TOP_GAP = 49, 12, 1
WHEEL_OR_IDIAM, WHEEL_OR_DIAM, WHEEL_HEIGHT, WHEEL_GAP = 50, 3, 6, 1
SERVO_C2 = 7.5
REAR_C1, REAR_C2 = SERVO_C2 + 3, 2 * SERVO_C3
FRONT_C1, FRONT_THICKNESS = 9 + 4, 3
TOP_PLATE_THICKNESS = 4
TOP_PLATE_C1 = BATTERY_C1 + FRONT_C1 + REAR_C1                  # 72.5
BALL_SIZE, BALL_WALL, BALL_AIRGAP, ROBOT_HEIGHT = 16.4, 2.25, 0.65, 47
SKYMEGA_MM, SKYMEGA_DRILL = 51.4, 15                           # its board, its four holes +-15 mm from its middle
ULTRASOUND_HOLE, ULTRASOUND_BASE, ULTRASOUND_LENGTH = 16.5, (25, 21, 3), 12.5

# main.scad's frame to the robot's: its wheels' axle at (SERVO_C5, -, SERVO_C6), its middle at y = SERVO_C3
AXLE_X_SCAD, MID_Y_SCAD, AXLE_Z_SCAD = SERVO_C5, SERVO_C3, SERVO_C6

# Its wheels: Ø56 printed discs (WHEEL_OR_IDIAM + 2 WHEEL_OR_DIAM), a 3 mm O-ring in the V of their rim, its middle
# 1.5 mm x sqrt(2) beyond the V's bottom (26.5 mm): the O-ring's outside at 30.1 mm.
RIM_RADIUS_MM = (WHEEL_OR_IDIAM + 2 * WHEEL_OR_DIAM) / 2       # 28
WHEEL_RADIUS_MM = RIM_RADIUS_MM - WHEEL_OR_DIAM / 2 + WHEEL_OR_DIAM / 2 * math.sqrt(2) + WHEEL_OR_DIAM / 2   # 30.1
WHEEL_WIDTH_MM = WHEEL_HEIGHT
# Its wheels' middles (show_robot(): y = -SERVO_C7 - WHEEL_HEIGHT / 2 - WHEEL_GAP and its mirror): 84 mm apart
TRACK_MM = 2 * (SERVO_C3 + SERVO_C7 + WHEEL_HEIGHT / 2 + WHEEL_GAP)
AXLE_Z_MM = WHEEL_RADIUS_MM                     # its axle over the floor


def scad_to_robot(x, y, z):
    """A point of main.scad's frame in the robot's."""
    return MID_Y_SCAD - y, x - AXLE_X_SCAD, z - AXLE_Z_SCAD + AXLE_Z_MM


# Its ball caster (Sliptonic's, rear_part()): a Ø16 marble, its middle BALL_SIZE / 2 + BALL_WALL + BALL_AIRGAP behind
# the rear part's back (x = 0 in main.scad); it touches the floor when the robot stands on its wheels.
CASTER_R_MM = 8.0
CASTER_Y_MM = -(BALL_SIZE / 2 + BALL_WALL + BALL_AIRGAP) - AXLE_X_SCAD
# Its SRF02 (fake_ultrasound.scad, show_robot()): its transducer's front 12.5 mm out of its board, which sits behind
# the front part's face; its axis at main.scad's z = SERVO_C1 - ULTRASOUND_HOLE / 2 - 5.
SRF02_FRONT_SCAD_X = BATTERY_C1 + FRONT_C1 - FRONT_THICKNESS + ULTRASOUND_LENGTH + ULTRASOUND_BASE[2]   # 71.5
SRF02_Z_SCAD = SERVO_C1 - ULTRASOUND_HOLE / 2 - 5
SRF02_POS = scad_to_robot(SRF02_FRONT_SCAD_X, MID_Y_SCAD, SRF02_Z_SCAD)     # (0, 41.2, 30.8)
# Its Skymega board on the top plate's four holes (top_plate_arduino_uno()), on 10 mm spacers (its photo)
SKYMEGA_X_SCAD = TOP_PLATE_C1 / 2 - REAR_C1 - (TOP_PLATE_C1 - SKYMEGA_MM) / 2 + 3
SKYMEGA_SPACER_MM = 10.0
SKYMEGA_BOTTOM_SCAD = SERVO_C1 + TOP_PLATE_THICKNESS + SKYMEGA_SPACER_MM
PCB_MM = 1.6

# ---- Its board and sensor ------------------------------------------------------------------------------------------
SERVO_PINS = {'r': 8, 'l': 9}                  # SERVO2 (PB0) and SERVO4 (PB1), Obijuan's examples
LED_PIN, BUTTON_PIN = 13, 12
# The SRF02 (Devantech's data sheet): 40 kHz, a beam about 55 deg wide, from 16 cm (nearer it gives about that, its
# minimum) to 6 m. Its echo: from flat walls only where its beam meets them square (an ultrasonic echo is mirrored,
# not scattered): the nearest wall it sees within ECHO_DEG of square, its rays across its beam.
SRF02_MIN_MM = 160.0
SRF02_MAX_MM = 6000.0
SRF02_BEAM_DEG = 27.5                           # half its beam
SRF02_RAYS = 23
ECHO_DEG = 10.0
SMALL_FACE_MM = 20.0                            # a post's face, a wall's end: heard only near its axis
SMALL_TARGET_DEG = 8.0
SRF02_NOISE_MM = 3.0

# ---- Assumed (nothing measured) ------------------------------------------------------------------------------------
# Its Futaba S3003, hacked for continuous rotation (their pot set to the middle, their stop removed): their 0.23 s for
# 60 deg at 4.8 V (43 rpm) at their horn, 3.2 kgf cm stalled, as their data sheet gives them; their own control
# drives the motor in proportion to the pulse's distance from its middle, at full beyond SERVO_BAND_US. Their middle
# where Obijuan's examples stop them: write(90), 1472 us (Arduino's Servo maps 0..180 to 544..2400 us).
BATTERY_V = 4.8                                 # four AAA NiMH cells
SERVO_FREE_RAD_S = (60 / 0.23) * math.pi / 180  # 4.55 rad/s
SERVO_STALL_NM = 3.2 * 9.81 / 100               # 0.31 N m
SERVO_MID_US = 1472
SERVO_DEADBAND_US = 4
SERVO_BAND_US = 150
SERVO_ARMATURE = 2e-4                           # its motor and gears seen at its horn (kg m^2)
SERVO_FORWARD = {'r': -1, 'l': 1}               # a pulse longer than the middle: the left wheel forward, the right back
# Their O-rings' rubber on the maze's painted wood, its floor and its walls alike (Ratatron's tyres' 0.6; the walls'
# MuJoCo default of 1 not taken: with it, its Ø60 wheels meeting a 50 mm wall askew climb it and turn it over).
TIRE_FRICTION = 0.6
# Its mass by parts: its two S3003 (37 g each), four AAA NiMH cells (12 g), its printed chassis, wheels and holder
# (their STL's volumes in ABS, 80 % filled: 34, 11 + 11, 10 g), Skymega with its parts (18 g), SRF02 (5 g), marble
# (5 g), screws, spacers and wires (12 g).
MASS_KG = 0.230
COM_MM = (0.0, -8.0, 24.0)


def start():
    """Switched on in the start cell facing north; its button pressed: it goes a second later (its program)."""
    return [('put_down', (0, 0), 0.0), ('say', 'switched on in the start cell'), ('wait', 500),
            ('say', 'its button pressed'), ('press', 'button', 150), ('wait', 1200)]
