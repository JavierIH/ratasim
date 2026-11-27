"""Fujin (Team Raiju, github.com/team-raiju/fujin) in the simulator: where its repository is, its geometry and the
physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the axle between the wheels.

Its repository has no licence: nothing of it is copied here. The submodule external/fujin is read at build time: its
firmware built from it (fw/), its meshes made from its STEP (cad.py)."""
import math
import os

from .. import paths

FUJIN_DIR = os.environ.get('FUJIN_DIR', os.path.join(paths.EXTERNAL, 'fujin'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its meshes, its firmware's library (not committed)

# ---- Its 3D model: the board's STEP with its mechanics (boards/Fujin_V1/Fujin_Body_Full.step: its Altium board, its
# printed frame, motors, gears, wheels and fan); the STEP's frame is the robot's, with the axle at AXLE_IN_STEP_MM.
CAD_STEP = os.environ.get('FUJIN_STEP', os.path.join(FUJIN_DIR, 'boards', 'Fujin_V1', 'Fujin_Body_Full.step'))
CAD_MESHES = os.path.join(BUILD, 'cad')
AXLE_IN_STEP_MM = (0.0, 0.0, 9.6)               # the tyres' centres (cad.py checks it)
TIRE_CAD_RADIUS_MM = 12.6                       # its CAD's tyres (Mini-Z); its firmware's wheels are bigger (below)
PCB_BOTTOM_MM = -1.5 - AXLE_IN_STEP_MM[2]       # the board, 1.5 mm thick, is its chassis: under the axle
# Its outline (for collisions with the walls; the board's, from the STEP): boxes (x half-width, y from, y to).
OUTLINE = ((40.0, -22.0, 22.0),                 # the wheels' part
           (33.0, 22.0, 42.0), (25.0, 42.0, 50.0),        # its nose, the sensors on it
           (32.0, -38.0, -22.0), (15.0, -70.0, -38.0))    # its tail (the Bluetooth module, the battery's frame)
TOP_MM = 26.5 - AXLE_IN_STEP_MM[2]              # the fan's motor
# IR sensors (emitter SFH-4550 and phototransistor pairs): the emitters' centres (x, y) and axes (deg from +x
# counter-clockwise, 90 = forward), from the STEP. The side ones are crossed: the one looking at the left wall stands
# right of the middle. Firmware index (bsp::analog_sensors::SensingDirection): R, FL, FR, L. (Its firmware's side
# sensor angle, 56.5 deg in services/config.cpp, is not the STEP's: at it they would read, where its search reads the
# walls (entering a cell), the walls of the cell after it.)
IR_SENSORS = {
    'R': (0, (-14.8, 38.0), 40.0),
    'FL': (1, (-31.9, 30.0), 94.4),
    'FR': (2, (32.1, 29.9), 85.6),
    'L': (3, (14.9, 38.2), 140.0),
}
# Its side sensors turned this much ahead of the STEP's axes. Where its runs correct their odometry at a side wall's
# end (navigation.cpp, apply_wall_break_correction: the distance there set to cells x 180 + its start_wall_break_mm,
# tuned on the robot), its makers' logs (external/fujin/scripts/logs, fan 600) find it off by -3 mm on average (-18
# to +22); the simulated one (IR_BEAM 'light' below), by +5 (-3 to +12) at the STEP's axes, -3 (-8 to +5) at 3 deg.
SIDE_FORWARD_DEG = 3.0
IR_SENSORS['R'] = IR_SENSORS['R'][:2] + (IR_SENSORS['R'][2] + SIDE_FORWARD_DEG,)
IR_SENSORS['L'] = IR_SENSORS['L'][:2] + (IR_SENSORS['L'][2] - SIDE_FORWARD_DEG,)
IR_Z_MM = 3.0 - AXLE_IN_STEP_MM[2]
START_BACK_MM = 13.5        # its tail against the start cell's back wall (its firmware counts 14.5: 6 mm walls)

# ---- From its firmware (external/fujin/firmware) ---------------------------------------------------------------
WHEEL_RADIUS_MM = 13.25     # utils/movement_params.cpp, its search's wheel radius (its tyres measured)
ENCODER_COUNTS = 1024       # bsp: AS5047P on each wheel
WHEELS_DIST_MM = 70.0       # services/config.hpp
# Its sensors' references (services/config.cpp, ir_wall_dist_ref_left/right): the side sensors' readings with the
# robot centred in a cell (mm, what its calibration gives), read from its firmware (its makers changed them, 180 and
# 165 up to its RCX 2026 tag, 185.40 and 176.37 since 086fc9e); the fronts read the distance from the emitter.


def _side_refs():
    import re
    refs = {'L': 180.0, 'R': 165.0}
    try:
        text = open(os.path.join(FUJIN_DIR, 'firmware', 'src', 'services', 'config.cpp')).read()
    except OSError:
        return refs
    for side, name in (('L', 'left'), ('R', 'right')):
        m = re.search(r'Config::ir_wall_dist_ref_%s\s*=\s*([0-9.]+)' % name, text)
        if m:
            refs[side] = float(m[1])
    return refs


IR_SIDE_REF_MM = _side_refs()
IR_MAX_MM = 270.0
EEPROM_MAZE, EEPROM_MAZE_BACKUP = 0x3000, 0x4000    # bsp/eeprom.hpp: ADDR_MAZE_START, ADDR_MAZE_BACKUP_START
GYRO_BIAS_SET_DPS = -0.4905     # its z gyro's bias (config.cpp: z_imu_bias), subtracted by its BSP

# ---- Assumed physics (from its web page: 105 g, 3S LiPo, 650 gf of suction; the rest not measured) ------------------
MASS_KG = 0.105
COM_MM = (0.0, -6.0, 6.0)
BATTERY_V = 12.0
# Its tyres' grip: what its makers' logs (fan 600) turn at without slipping (its encoders' and gyro's turn rates within
# 0.4 rad/s of each other): 55 to 80 m/s^2 sideways, slipping from about 90; at its 6.4 N of suction (below) that is
# a friction of 1.1 to 1.3.
TIRE_FRICTION = 1.2
TYRE_ELLIPSOID = False
# Its tyres' friction stiff (MuJoCo's solreffriction: time constant, damping ratio), as ZoroBot3's and KOGUMA's; with
# MuJoCo's default (its contact's 4 ms) they creep sideways in a turn.
TYRE_CREEP = (0.002, 1.0)
# Its motors at the wheel: torque = KT * (V - KT * w) / R. Fitted to its control loop's feed-forward (its search's
# GeneralParams, in volts through its own motor model, control.cpp: amps times its 2.5 ohm): 0.62 V per m/s,
# 0.0375 V per m/s^2, 0.0014 V per rad/s^2 of turn (with YAW_INERTIA below); its gears' friction, the 0.13 A
# (0.33 V) of Coulomb feed-forward it gives each wheel moving straight.
MOTOR_KT = 0.0082           # N m/A, V s/rad
MOTOR_R_OHM = 0.44
WHEEL_FRICTION_NM = 0.13 * 2.5 / MOTOR_R_OHM * MOTOR_KT
WHEEL_ARMATURE = 2e-7       # the motor's rotor through its 68:9 gears, at the wheel (kg m^2)
# Its tyres' scrub turning in place, for which its firmware gives each wheel 0.12 A more (its in-place Coulomb
# feed-forward, GeneralParams; 0.01 A in its arcs): a yaw torque against the turn, fading as it rolls forward. 0.08 A
# of it at the wheels: its turn around (a time profile of 180.7 degrees, tracked by its gyro) then turns 180.5.
SCRUB_NM = 2 * 0.08 * 2.5 / MOTOR_R_OHM * MOTOR_KT / (WHEEL_RADIUS_MM / 1000) * 0.035
SCRUB_SPEED = 0.05          # m/s
# Its yaw inertia, from its makers' logs: their turns follow their profile with no lag (at 1.25 m/s, 19 rad/s, its
# turns' voltages ramp up with the profile); at 1.35e-4 (assumed before: a uniform board's) the simulated one lagged
# 4 ms behind, its firmware's PI wound up and its turns ended 5 to 11 % past (theirs 0 to 4 %).
YAW_INERTIA = 5.5e-5
# Its losses with its fan on: its makers' logs at fan 600 need 0.93 V a wheel per m/s (0.54 to 0.6 without it, as its
# motors' model): the suction's rolling losses, viscous, a share of its downforce (N per m/s for each N). The same
# drag on each wheel resists its turns (its logs' turns' voltages).
ROLL_DRAG = 0.0             # N per m/s, without its fan
ROLL_DRAG_PER_N = 0.1       # N per m/s more for each N of its fan's downforce
# Its fan's downforce at its rated 10 V, quadratic in the voltage: its web page's 650 gf (6.4 N) at its runs' fan 600
# (6 V; fan_speed 600 of 1000 of its 10 V), as its grip in its logs needs.
FAN_DOWNFORCE_N = 6.4 / 0.6 ** 2
# Its IR sensors: a reading's noise (mm); the ADC counts of a reading (the firmware's conversion, mm =
# a / ln(raw + c) - b, both ways: fw/fbsp.cpp)
IR_NOISE_MM = 0.8
# What each sensor sees (robot.py, ir_mm): its emitter's spot as seven rays, a Gaussian of 1.5 deg, each ray's light
# at its length times the cosine of its incidence over its calibration's. Without the cosine (three rays' lengths, a
# harmonic mean: 'harmonic') a far wall at a slant read as near as a square one: with its side sensors turned 3 deg
# its search read, entering a cell with a wall ahead and none at a side, that side's wall (224-228 mm, its threshold
# 230) and went home without the goal; 2.5 deg smears its wall ends (its fast runs end on posts sooner).
IR_BEAM = 'light'
IR_RAYS = tuple((d, math.exp(-0.5 * (d / 1.5) ** 2)) for d in (-3.6, -2.4, -1.2, 0.0, 1.2, 2.4, 3.6))
IR_MODEL = (4000.0, 440.0, 0.0)
# Its LSM6DSR: the real bias off the one its firmware subtracts, the noise at its 2.1 kHz.
GYRO_BIAS_DPS = GYRO_BIAS_SET_DPS + 0.03
GYRO_NOISE_DPS = 0.1


def counts_per_rad():
    return ENCODER_COUNTS / (2 * math.pi)


# ---- Its start (ratasim/start.py) ---------------------------------------------------------------------------------
def start(cell, heading_deg, mode='slow'):
    """As its makers start a search: put down in the start cell, its tail against the back wall; button 2: search;
    button 1 held: its speeds (button 2 steps them: custom, slow, medium, fast; slow without its fan); held: its
    exploration (normal: to the goal and back); held again: it waits for a hand in front of its front sensors (white),
    which it reads for 0.4 s: two seconds' beep and it goes."""
    steps = {'slow': 1, 'medium': 2, 'fast': 3}[mode]
    out = [('put_down', cell, heading_deg), ('say', 'switched on'), ('wait', 3000),
           ('say', 'button 2: search'), ('press', 2, 200), ('wait', 500),
           ('say', 'button 1 held: its speeds'), ('press', 1, 900), ('wait', 1200)]
    out += [('press', 2, 200), ('wait', 500)] * steps
    out += [('say', f'button 1 held: search {mode}, normal exploration'), ('press', 1, 900), ('wait', 1200),
            ('press', 1, 900), ('wait', 1300),
            ('say', 'white: a hand in front of its front sensors'), ('hand', ['FL', 'FR'], 700), ('wait', 600)]
    return out


RUN_SPEEDS = ('custom', 'slow', 'medium', 'fast', 'super')
RUN_MODES = ('smooth', 'diagonals', 'time', 'hardcoded')


def run_start(cell, heading_deg, speed='slow', mode='smooth', backup=False):
    """As its makers start a speed run on the maze its search saved (fsm/states/run.cpp): put down in the start cell
    again; from idle, button 2 twice (its search's menu, then its run's); button 1 held: its run's speeds (button 2
    steps them: custom, slow, medium, fast, super); held: its moves (button 2: smooth, diagonals, time-based,
    hard-coded); held: its map (button 2: the one saved at its search's end, or its backup, saved at the goal);
    held: it waits for a hand in front of its front sensors (0.4 s), beeps two seconds and goes."""
    out = [('put_down', cell, heading_deg), ('wait', 1500),
           ('say', 'button 2 twice: its run'), ('press', 2, 200), ('wait', 500), ('press', 2, 200), ('wait', 500),
           ('say', 'button 1 held: its speeds'), ('press', 1, 900), ('wait', 1200)]
    out += [('press', 2, 200), ('wait', 500)] * RUN_SPEEDS.index(speed)
    out += [('say', f'button 1 held: run {speed}, its moves'), ('press', 1, 900), ('wait', 1200)]
    out += [('press', 2, 200), ('wait', 500)] * RUN_MODES.index(mode)
    out += [('say', f'button 1 held: {mode}, its map'), ('press', 1, 900), ('wait', 1200)]
    if backup:
        out += [('press', 2, 200), ('wait', 500)]
    out += [('say', f'button 1 held: {"its backup" if backup else "its saved"} map, waits for a hand'),
            ('press', 1, 900), ('wait', 1300),
            ('say', 'white: a hand in front of its front sensors'), ('hand', ['FL', 'FR'], 700), ('wait', 600)]
    return out
