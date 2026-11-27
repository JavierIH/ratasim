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
# Its start. Its firmware counts its tail against the start cell's back wall (MOVE_START, set_starting_position: its
# axle ROBOT_BACK_LENGTH + WALL_WIDTH / 2 from the wall's middle; START_AXLE_MM, --start wall). By default it stands in
# the cell's middle (START_BACK_MM 0), as its side sensors' angles (SIDE_FORWARD_DEG) were fitted with it there: so its
# speed runs at 1 and 2 reach the goal in OSHWDEM 2026 (7.82, 6.92 s). There it is 43 mm further on than its firmware
# counts until a wall's end or a front wall puts it right (its first turns 43 mm late: in the stairs maze and Portugal
# 2025's final, on the wall at its first turn), and its side sensors' calibration at its exploration's start
# (floodfill.c, side_sensors_calibration) reads the next cell's walls. With --start wall its run at 2 ends on a post.
START_AXLE_MM = ROBOT_BACK_LENGTH + 12.0 / 2 + 0.3
START_BACK_MM = 0.0

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
# Its side sensors turned forward of the CAD's axes, the right one more (the CAD's left one points 1.9 deg further
# forward; its makers' wall loss distances, LEFT/RIGHT_WALL_LOSS_TO_SENSING_POINT_DISTANCE 110 and 116, have the right
# one lose a wall's end first): both 43.5 deg off the side axis. Its firmware since 2026 lets go of a wall 6-8 ms after
# its reading crosses (WALL_DETECT_RELEASE_COUNT, an EMA, a median of three: c83c6e6, 18e21af), 20-26 mm at its runs'
# 3 m/s: as the CAD has them (or 3 deg forward, as for its 2025 firmware) its speed runs' turns began 10-25 mm late.
SIDE_FORWARD_DEG = {'SL': float(os.environ.get('ZOROBOT_SL_FORWARD_DEG', '4.5')),
                    'SR': float(os.environ.get('ZOROBOT_SR_FORWARD_DEG', '6.5'))}
for _n, _s in (('SL', -1), ('SR', 1)):
    IR_SENSORS[_n] = IR_SENSORS[_n][:2] + (IR_SENSORS[_n][2] + _s * SIDE_FORWARD_DEG[_n],)
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
# Its yaw inertia (kg m^2): KOGUMA's, fitted to its logs, is 3.7e-5 at 78 g (a 22 mm radius of gyration); its own
# battery, motors and fan sit closer to its middle: 2.9e-5 (19.7 mm). A uniform board's 8.8e-5 made its firmware's PI
# (no feed-forward) overshoot every turn by 2-4 deg; at 4e-5 its speed runs' turns end 3-5 mm wide of the path its
# firmware commands, and its 1.3 m of 45 deg turns along rows 0-1 (no wall to resync on) end on the posts (9,1) and (15,2).
YAW_INERTIA = float(os.environ.get('ZOROBOT_YAW_INERTIA', '2.9e-5'))
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
        steps.append(('hand', ['FL'], 500))   # SENSOR_START_MIN_MS 350 since 2026
    return steps + [('wait', 2500)]
