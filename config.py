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
AXLE_Y_KICAD = 102.16       # KiCad y of the axle: the brackets' bearing over their screw holes (40.7 to the back edge)
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
PCB_THICK_MM = 0.71
AXLE_OVER_PCB_MM = 9.09      # the bearing's centre over the board's top face (motor brackets)
PCB_BOTTOM_MM = AXLE_HEIGHT_MM - AXLE_OVER_PCB_MM - PCB_THICK_MM     # 2.64 mm over the floor

# ---- Printed parts (3d_model/stl_files): the brackets' frame is the robot's with y - 0.07 and z from the board's top
STL_AXLE_Y = 0.07           # the bearing's y in the brackets' frame
MOTOR_AXIS = (-11.68, -2.75)  # the 8520's axis (y, z) from the axle: the brackets' motor holes (pinion 8T on 40T)
FAN_BRACKET_Z = 10.54       # the fan bracket's underside on the brackets' front tops, over the board
FAN_HOLE_KICAD = (150.57, 79.36)    # the board's suction hole (r 10): fan and its motor centred on it
GEAR_X = 26.2               # the gears' plane: wheel gear 40T, the motor's pinion 8T and the encoder gear 22T
# Encoders: a small black board stands in a slot of the main board (footprint ZoroBot3_PCBmain_Encoder, 1.2 x 7.1
# mm) beside the wheel, its AS5145B facing the magnet of a 22T gear driven by the motor's pinion (7.5 mm behind).
ENCODER_SLOT_KICAD = ((119.03, 121.28), (182.105, 121.27))
ENCODER_GEAR = (MOTOR_AXIS[0] - 7.5, MOTOR_AXIS[1])     # (y, z) from the axle

# ---- Assumed physics (nothing measured) --------------------------------------------------------------------------
MASS_KG = 0.070             # 2S 260 mAh LiPo, two 8520, a PCB chassis, printed brackets and fan
COM_MM = (0.0, 4.0, 2.0)
BATTERY_V = 8.0
# 8520 coreless at 7.4 V: ~45000 rpm free, ~3.7 A stall (R 4 ohm turned 5-16 deg too far in its explore turns)
MOTOR_R_OHM = 2.0
MOTOR_KE = 7.4 / (45000 * 2 * math.pi / 60)    # V s/rad = N m/A
MOTOR_ROTOR_INERTIA = 6e-9                     # kg m^2 (on the motor shaft)
GEAR_EFFICIENCY = 0.85
MOTOR_FRICTION_NM = 1.5e-4                     # at the motor shaft
TIRE_FRICTION = 1.1
FAN_DOWNFORCE_N = 8.0                           # at 100 % duty, quadratic in it: its explore turns (~1.2 g) slid with less
