"""UKMARSBOT (UK Micromouse and Robotics Society, github.com/ukmars/ukmarsbot) in the simulator, running
mazerunner-core (Peter Harrison, github.com/ukmars/mazerunner-core) as Peter's ORION: where its repositories are,
its geometry and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on the floor under
its wheels' axle.

Sources: its boards' KiCad files (ecad/kicad: mainboard, wall-sensor-advanced; kicad.py reads them, cad.py places
their parts' 3D models) and Gerber files (hardware/: pcb.py textures their faces from them), its firmware's
hardware and robot configurations (config-ukmarsbot.h, config-robot-orion.h: its pins, its odometry, its sensors'
calibration, its motors' measured constants), its photos and renders (renders/).

Its firmware: mazerunner-core at Peter's last commit of October 2025 (5077a4f, "adjust turn parameters for orion"),
the robot its config.h selects, ORION (Peter's UKMARSBOT: 11:1 N20 gearmotors with 18-magnet encoder discs, the
advanced wall sensor), set at build time to EVENT_UK (fw/gen_src.py): the goal at (7, 7), a cell of the 16x16 maze's
centre, and ORION's sensor calibration for UK contests. Not its tag V0.2.0 (the UK contest of 2023, OSMIUM CORE):
its stop_at_center() can miss its stop and drive into the wall ahead (its sensors reaching their reference before
its motion profile ends: the profile's end then sets its speed back to 30 mm/s), the bug that commit 1f1fd7a of
October 2025 fixed. Not its last commit either: a revert of May 2026 adds its steering's adjustment, in deg/s, to its
angle error each 2 ms tick, with gains set for it multiplied by the tick: 500 times stronger.
"""
import math
import os

from .. import paths

UKMARSBOT_DIR = os.environ.get('UKMARSBOT_DIR', os.path.join(paths.EXTERNAL, 'ukmarsbot'))
MAZERUNNER_DIR = os.environ.get('MAZERUNNER_DIR', os.path.join(paths.EXTERNAL, 'mazerunner-core'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its firmware library, its meshes
MAINBOARD = os.path.join(UKMARSBOT_DIR, 'ecad', 'kicad', 'mainboard', 'mainboard.kicad_pcb')
SENSOR_BOARD = os.path.join(UKMARSBOT_DIR, 'ecad', 'kicad', 'wall-sensor-advanced',
                            'ukmarsbot-wall-sensor-advanced.kicad_pcb')

# ---- From its firmware (config-ukmarsbot.h, config-robot-orion.h) ------------------------------------------------
COUNTS_PER_MM = {'l': 1918 / 500, 'r': 1927 / 500}     # MM_PER_COUNT_LEFT/RIGHT, measured over 500 mm
MOUSE_RADIUS_MM = 38.70                         # its wheels' contact from the middle (calibrated by turning it)
# Its encoders: 18-magnet discs (9 cycles a turn on each of two channels: 36 edges, all counted on the channels
# XOR-ed) on 11:1 gearmotors, 396 counts a turn of the wheel: each wheel's radius from its counts a mm.
COUNTS_PER_WHEEL_TURN = 36 * 11
WHEEL_RADIUS_MM = {s: COUNTS_PER_WHEEL_TURN / c / (2 * math.pi) for s, c in COUNTS_PER_MM.items()}   # 16.43, 16.35
# Its pins and wiring, as its firmware's polarities undo it: the left encoder's quadrature runs backwards going
# forward (ENCODER_LEFT_POLARITY -1), the right motor's direction pin is high going forward (MOTOR_RIGHT_POLARITY -1).
ENCODER_SEQUENCE = {'l': -1, 'r': 1}
MOTOR_DIR_PIN = {'l': 7, 'r': 8}
MOTOR_PWM_PIN = {'l': 9, 'r': 10}
MOTOR_FORWARD_DIR = {'l': 0, 'r': 1}
LED_PIN = 6                                     # LED_USER (USER_IO): the sensor board's LED
EMITTER_FRONT, EMITTER_DIAGONAL = 11, 12        # EMITTER_A, EMITTER_B
SWITCHES_CHANNEL, BATTERY_CHANNEL = 6, 7        # A6, A7
# Its function switches: a resistor ladder on A6, the ADC's reading of each setting 0..15 (adc_thesholds); its
# button pulls it up to the supply (above 800: pressed).
SWITCH_ADC = (660, 647, 630, 614, 590, 570, 545, 522, 461, 429, 385, 343, 271, 212, 128, 44)
BUTTON_ADC = 1023
# Its battery on A7 through its 10k/10k divider, a 5 V reference
BATTERY_ADC_PER_V = 1023 / 5.0 / 2
# Its motors as measured (FWD_KM, FWD_TM, BIAS_FF): mm/s for each volt, the time constant, the volts that only
# overcome friction
FWD_KM, FWD_TM, BIAS_FF = 475.0, 0.190, 0.340
ROT_TM = 0.210

# ---- From its boards (kicad.py) ---------------------------------------------------------------------------------
# Its wall sensors (wall-sensor-advanced): four 5 mm SFH4545 emitters lying on the board's top, 3 mm over it, each
# with a TEFT4300 phototransistor under the board beside it, looking the same way. Each sensor's beam from between
# their lens tips (x, y), along the emitter's axis (deg from +x, counter-clockwise); its ADC channel (its firmware's
# RFS 0, RSS 1, LSS 2, LFS 3: A0..A3 on the board's connector) and its emitter's pin (front pair EMITTER_A, the
# diagonal pair EMITTER_B).
IR_SENSORS = {          # name: ADC channel, emitter pin, (x, y), axis deg
    'RF': (0, EMITTER_FRONT, (34.04, 52.61), 80.0),
    'RD': (1, EMITTER_DIAGONAL, (24.17, 60.05), 30.0),
    'LD': (2, EMITTER_DIAGONAL, (-24.11, 60.01), 150.0),
    'LF': (3, EMITTER_FRONT, (-34.04, 52.63), 100.0),
}
PCB_THICK = 1.6
# Heights: its gearmotors lie on the main board, their axis 5 mm over it (a 10 mm high N20); the sensor board on the
# main board's 14-way connector, 11 mm over it (a pin header in a socket); the IR emitters 3 mm over its top, the
# phototransistors 3 mm under it.
MOTOR_AXIS_OVER_BOARD = 5.0
AXLE_Z = (WHEEL_RADIUS_MM['l'] + WHEEL_RADIUS_MM['r']) / 2
MAIN_TOP = AXLE_Z - MOTOR_AXIS_OVER_BOARD
MAIN_BOTTOM = MAIN_TOP - PCB_THICK
SENSOR_BOTTOM = MAIN_TOP + 11.0
SENSOR_TOP = SENSOR_BOTTOM + PCB_THICK
IR_Z_MM = ((SENSOR_TOP + 3.0) + (SENSOR_BOTTOM - 3.0)) / 2
BACK_MM = -36.84                                # the main board's back edge (its outline)

# ---- Assumed (nothing measured) ----------------------------------------------------------------------------------
# Its IR pairs: the light back from a wall, a diffuse reflector, as cos(incidence) A / (d + IR_OFFSET_MM)^2, d along
# the ray; three rays across its beam (its middle and IR_SPREAD_DEG either side), their light weighted IR_WEIGHTS.
# Each sensor's A from its firmware's calibration (EVENT_UK): its raw reading (lit - dark) backed up to a wall with
# another wall ahead (the front pair: FRONT_LEFT/RIGHT_CALIBRATION, 90 and 110, at 79.7 mm along their beams, 10 deg
# off the wall's normal), or centred in a cell with walls beside it (the diagonal pair: LEFT/RIGHT_CALIBRATION, 152
# and 192, at 69.2 mm, 30 deg off). IR_OFFSET_MM puts the front pair's sum at its FRONT_REFERENCE (850, normalised)
# centred in a cell before a wall (31.9 mm along their beams). Its turns' trigger (TURN_THRESHOLD_SS90E, 115, and 5
# for each side wall) then comes 20 mm into a cell between walls, as its config says, and 6 mm earlier with none.
IR_OFFSET_MM = 13.26
IR_SPREAD_DEG = 4.0
IR_WEIGHTS = (0.25, 0.5, 0.25)
IR_CAL = {'RF': (110, 79.74, 10.0), 'RD': (192, 69.2, 30.0), 'LD': (152, 69.2, 30.0), 'LF': (90, 79.74, 10.0)}


def _cal_light(d, off_deg):
    """The light (per unit of A) at a calibration: a flat wall, the beam's middle d away, off_deg off its normal."""
    light = 0.0
    for w, spread in zip(IR_WEIGHTS, (-IR_SPREAD_DEG, 0.0, IR_SPREAD_DEG)):
        a = math.radians(off_deg + spread)
        light += w * math.cos(a) / (d * math.cos(math.radians(off_deg)) / math.cos(a) + IR_OFFSET_MM) ** 2
    return light


IR_K = {n: raw / _cal_light(d, off) for n, (raw, d, off) in IR_CAL.items()}
IR_DARK = 25                                    # counts, emitters off (the maze's lighting)
IR_FAR_MM = 400.0
IR_NOISE = 0.7                                  # counts, gaussian (its ADC: about 1 LSB)
# A rechargeable PP3 (8.4 V), two N20 gearmotors with encoders, its boards, an Arduino Nano, printed brackets and
# skids: about 135 g; its centre of mass a little behind its axle (the battery), over its rear skid.
MASS_KG = 0.135
# Its tyres sliding sideways in its turns (ratasim/tyre.py): their cornering stiffness, per radian of slip angle.
TYRE_CORNERING = float(os.environ.get('RATASIM_TYRE_C', '0')) or None
COM_MM = (0.0, -3.0, 18.0)
BATTERY_V = 8.4
TIRE_FRICTION = 1.0
# Its motors, each a DC motor driving its wheel through its gearbox (volts across it: the duty times the
# battery's, TB6612FNG): their constants at the wheel from its measured FWD_KM and FWD_TM, its mass, its wheels'
# radius (r): back EMF KE = r / FWD_KM (V s/rad), the same torque constant, the winding's R = 2 FWD_TM KE^2 / (m r^2);
# friction as the force BIAS_FF volts give. Its turning inertia so that turns take ROT_TM to settle, as measured
# (its ROT_KM, 775 deg/s a volt, is then 703: 10% less).
KE_WHEEL = (AXLE_Z / 1000) / (FWD_KM / 1000)
MOTOR_R_OHM = 2 * FWD_TM * KE_WHEEL ** 2 / (MASS_KG * (AXLE_Z / 1000) ** 2)
INERTIA_Z = ROT_TM / FWD_TM * MASS_KG * (MOUSE_RADIUS_MM / 1000) ** 2
SKID_CLEARANCE_MM = 0.5                         # its two printed skids, front and rear, just over the floor
SKID_Y = (-30.0, 56.0)
SKID_R = 3.0


# ---- Its start (ratasim/start.py) --------------------------------------------------------------------------------
def start(function=2, cell=(0, 0)):
    """As its maker starts it at a contest: backed up to the start cell's wall, switched on with its button held
    (mazerunner-core clears its maze map then: it keeps it in memory a reset does not clear), its function switches
    set (2: search to the goal and back) and its button pressed; its LED flashes, a hand in front of its left front
    sensor (it reads which one is covered), taken away: it goes."""
    return [('put_down', cell, 0.0),
            ('say', 'switched on with its button held: its maze map cleared'), ('press', 'button', 800), ('wait', 1500),
            ('say', f'function switches at {function}, the button pressed'), ('switches', function),
            ('press', 'button', 200), ('wait', 700),
            ('say', 'a hand in front of its left front sensor, then away: GO'), ('hand', ['LF'], 500), ('wait', 600)]
