"""ZoroBot3 (OPRobots, github.com/OPRobots/ZoroBot3) in the simulator: where its repository is, its geometry and
the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the axle between the wheels."""
import math
import os

from .. import config as RC
from .. import paths

# The ZoroBot3 repository: the submodule external/ZoroBot3 (its sources are not copied here).
ZORO_DIR = os.environ.get('ZOROBOT3_DIR', paths.ZOROBOT3)
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

# ---- From its Onshape assembly (cad.py): the "Full Assembly" exported as one STEP file into cad/ (not committed)
CAD_STEP = os.environ.get('ZOROBOT3_STEP', os.path.join(HERE, 'cad', 'Full_Assembly.step'))
CAD_MESHES = os.path.join(BUILD, 'cad')
# IR sensors: the emitters' centres (x, y mm) and axes (deg from +x counter-clockwise, 90 = forward). The wall
# sensors are crossed: the one for the right wall stands on the left.
IR_SENSORS = {          # firmware index: (x, y), direction
    'FL': (0, (-28.99, 20.77), 96.1),
    'FR': (1, (28.64, 20.78), 82.1),
    'SL': (2, (13.88, 42.67), 141.0),
    'SR': (3, (-14.10, 42.93), 37.1),
}
# Its side sensors turned this much forward of the CAD's axes: as the CAD has them they lose a wall's end 108 mm before
# the next cell at its speed runs' 2 m/s (111 mm standing); its makers measured 116 (WALL_LOSS_TO_SENSING_POINT_DISTANCE,
# found on the robot in the maze for its best centring), which 3 deg gives (114.5 at 2 m/s, 119.5 standing).
SIDE_SENSORS_FORWARD_DEG = float(os.environ.get('ZOROBOT_SIDE_FORWARD_DEG', '3.0'))
for _n, _s in (('SL', -1), ('SR', 1)):
    IR_SENSORS[_n] = IR_SENSORS[_n][:2] + (IR_SENSORS[_n][2] + _s * SIDE_SENSORS_FORWARD_DEG,)
IR_Z_MM = 2.66              # the emitters' height over the axle (11.75 over the board)
BOARD_FRONT_MM = 51.23      # the board's outline from the axle: a semicircle in front
TIRE_CAD_RADIUS_MM = 11.56  # less than the firmware's 12.44 (below): the visual tire is scaled to it

# ---- Drive (README: 8520 coreless motors, pinion 8T, wheel gear 40T, encoder gear 22T, AS5145B) ---------------
GEAR = 40 / 8
ENCODER_COUNTS_PER_WHEEL_REV = 4096 * 40 / 22
WHEEL_RADIUS_MM = ENCODER_COUNTS_PER_WHEEL_REV * MICROMETERS_PER_TICK / 1000 / (2 * math.pi)   # 12.44
AXLE_HEIGHT_MM = WHEEL_RADIUS_MM
PCB_THICK_MM = 0.80
AXLE_OVER_PCB_MM = 9.09      # the axle over the board's top face

# ---- Physics: its mass, suction and motors' resistance from its makers (OPRobots, 2026-11); the rest assumed -----
MASS_KG = 0.075             # 2S 260 mAh LiPo, two 8520, a PCB chassis, printed brackets and fan
# Its yaw inertia (kg m^2), as KOGUMA's fitted to its logs (3.7e-5 at 78 g, a 22 mm radius of gyration): a uniform
# board's 8.8e-5 made its firmware's PI (no feed-forward) overshoot every turn by 2-4 deg, and its explore's 180 deg
# turns, whose angular error its firmware then discards (reset_control_errors), left it 4-5 deg off into the walls.
YAW_INERTIA = 4.0e-5
COM_MM = (0.0, 4.0, 2.0)
BATTERY_V = 8.0
# 8520 coreless at 7.4 V, from its makers (the specification's): 50000 rpm free, 0.2 A free, 5 A stall (1.5 ohm;
# its specification's "about 2 ohm" would stall at 3.7 A)
MOTOR_R_OHM = 7.4 / 5.0
MOTOR_KE = 7.4 / (50000 * 2 * math.pi / 60)    # V s/rad = N m/A
MOTOR_ROTOR_INERTIA = 6e-9                     # kg m^2 (on the motor shaft)
GEAR_EFFICIENCY = 0.85
MOTOR_FRICTION_NM = MOTOR_KE * 0.2             # at the motor shaft: its free current's torque
TIRE_FRICTION = 1.1
# Its tyres' grip on the floor: single contact points (ellipsoids), their friction stiff (MuJoCo's solreffriction:
# its time constant, its damping ratio). With the default soft friction its tyres crept sideways at 0.23 m/s in its
# speed runs' turns (6 deg of slip at 60 % of their grip), every turn ending 20-40 mm wide of its cell's centre.
TYRE_CREEP = (0.002, 1.0)
FAN_DOWNFORCE_N = 7.5                           # at 100 % duty, quadratic in it: its makers' 350-400 g at 70 %


# ---- Its start (ratasim/start.py) ---------------------------------------------------------------------------------
def start(run=False):
    """Boots, its menu's run picked with its buttons (MODE, UP), a hand in front of its right front sensor: EXPLORE;
    another in front of its left one right after: EXPLORE & RUN."""
    steps = [('say', 'booting'), ('wait', 3500), ('press', 'mode'), ('wait', 400), ('press', 'up'), ('wait', 800),
             ('say', 'a hand in front of the right front sensor: EXPLORE' + (' & RUN' if run else '')),
             ('hand', ['FR'], 500), ('wait', 900)]
    if run:
        steps.append(('hand', ['FL'], 300))
    return steps + [('wait', 2500)]
