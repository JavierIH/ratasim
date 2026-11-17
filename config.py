"""ZoroBot3 (OPRobots, github.com/OPRobots/ZoroBot3) in the simulator: where its repository is, its geometry and
the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the axle between the wheels."""
import math
import os

from .. import config as RC
from ... import paths

# The ZoroBot3 repository, cloned next to this one (its sources are not copied here).
ZORO_DIR = os.environ.get('ZOROBOT3_DIR', os.path.join(os.path.dirname(paths.REPO), 'ZoroBot3'))
ZORO_SRC = os.path.join(ZORO_DIR, 'source_code')
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # generated meshes, the firmware library, flash images

# ---- From its firmware (include/config.h, constants.h) -------------------------------------------------------
MICROMETERS_PER_TICK = 10.494055
ROBOT_FRONT_LENGTH = 48.121
ROBOT_BACK_LENGTH = 40.706
ROBOT_WIDTH = 70.2
WHEELS_SEPARATION = 62.0
CELL_MM = RC.CELL_MM

# ---- From its PCB (pcb_files/kicad_project/ZoroBot3.kicad_pcb) --------------------------------------------------
PCB_CENTRE_X = 150.58       # KiCad x of the centre line (USB, MCU, RGB LED)
AXLE_Y_KICAD = 99.0         # KiCad y of the axle: the front edge (50.93) + ROBOT_FRONT_LENGTH
PCB_KEEP_X = (115.5, 185.5)  # the main board; the sensor boards are panelised beside it
# IR sensors: vertical boards in the sockets SEN_FRONT_I/D and SEN_PARED_I/D (emitter over receiver). The wall
# sensors are crossed: the one for the right wall stands on the left. (x, y) mm in the robot frame, direction deg
# from +x counter-clockwise (90 = forward).
IR_SENSORS = {          # firmware index: (x, y, z, direction)
    'FL': (0, (122.13 - PCB_CENTRE_X, AXLE_Y_KICAD - 84.67, 8.0), 90.0 + 7.0),
    'FR': (1, (178.98 - PCB_CENTRE_X, AXLE_Y_KICAD - 84.67, 8.0), 90.0 - 7.0),
    'SL': (2, (167.13 - PCB_CENTRE_X, AXLE_Y_KICAD - 61.42, 8.0), 90.0 + 51.8),
    'SR': (3, (133.93 - PCB_CENTRE_X, AXLE_Y_KICAD - 61.37, 8.0), 90.0 - 51.8),
}

# ---- Drive (README: 8520 coreless motors, pinion 8T, wheel gear 40T, encoder gear 22T, AS5145B) ---------------
GEAR = 40 / 8
ENCODER_COUNTS_PER_WHEEL_REV = 4096 * 40 / 22
WHEEL_RADIUS_MM = ENCODER_COUNTS_PER_WHEEL_REV * MICROMETERS_PER_TICK / 1000 / (2 * math.pi)   # 12.44
TIRE_WIDTH_MM = 7.0
AXLE_HEIGHT_MM = WHEEL_RADIUS_MM
PCB_BOTTOM_MM = 2.5         # assumed: the board's height above the floor (a suction robot rides low)
PCB_THICK_MM = 0.7

# ---- Assumed physics (nothing measured) --------------------------------------------------------------------------
MASS_KG = 0.085             # 2S 260 mAh LiPo, two 8520, a PCB chassis, printed brackets and fan
COM_MM = (0.0, 4.0, 2.0)
BATTERY_V = 8.0
# 8520 coreless at 7.4 V: ~45000 rpm free, ~1.8 A stall
MOTOR_R_OHM = 4.0
MOTOR_KE = 7.4 / (45000 * 2 * math.pi / 60)    # V s/rad = N m/A
MOTOR_ROTOR_INERTIA = 6e-9                     # kg m^2 (on the motor shaft)
GEAR_EFFICIENCY = 0.85
MOTOR_FRICTION_NM = 1.5e-4                     # at the motor shaft
TIRE_FRICTION = 0.9
FAN_DOWNFORCE_N = 3.0                           # at 100 % duty; quadratic in the duty
