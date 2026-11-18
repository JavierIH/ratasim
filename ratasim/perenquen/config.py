"""Perenquen (Javier Baliñas, github.com/supernudo/perenquen) in the simulator: where its repository is, its geometry
and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin on its front axle at the axle's
height: the centre its firmware turns on (ROBOT_CENTER_TO_FRONT, its README: it turns on its front wheels).

Sources: its firmware (Software/perenquen-board: main.h, cs.c, sensor.c, actuator.c), its FreeCAD assembly
(Mechanics, cad.py), its KiCad board (Electronics/perenquen-board/perenquen.kicad_pcb: the board that was made; its
.net is an older export, its left and right swapped) and its design notes (Design/design_notes.ods, "motores_hp":
the column Perenquén).
"""
import math
import os

from .. import paths

PERENQUEN_DIR = os.environ.get('PERENQUEN_DIR', os.path.join(paths.EXTERNAL, 'perenquen'))
AVERSIVE_DIR = os.environ.get('AVERSIVE_DIR', os.path.join(paths.EXTERNAL, 'aversive4dspic'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its firmware library, its meshes
CAD_MESHES = os.path.join(HERE, 'assets')         # its meshes (cad.py), committed

# ---- From its firmware (main.h, actuator.c) ----------------------------------------------------------------------
WHEEL_DIAMETER_MM = 25.5                        # WHEEL_DIAMETER_MM
ENCODER_COUNTS_PER_REV = 4096                   # IMP_ENCODERS x4: AS5045B magnetic encoders on the rear wheels' axles
TRACK_MM = 66.0                                 # EXT_TRACK_MM (the tyres' centres: 68.2 in its CAD)
PWM_PERIOD = 6000                               # PTPER, HSPWM_DUTY_MAX (20 kHz)
WHEEL_RADIUS_MM = WHEEL_DIAMETER_MM / 2

# ---- From its CAD (cad.py): the assembly's frame is x forward, y left, its front axle at (15.61, ., 11.10) -------
CAD_TO_ROBOT = (15.61, 11.10)                   # robot = (-y_cad, x_cad - 15.61, z_cad - 11.10)
REAR_AXLE_Y = -31.07                            # the rear wheels' axle (x_cad -15.46), 0.5 mm higher in its CAD
TIRE_Y_MM = 34.1                                # each tyre's centre from the middle (8 mm wide Mini-Z tyres)
TIRE_WIDTH_MM = 8.0
BOARD = dict(back=-45.6, front=55.4, half_width=37.9, round_from=17.0, bottom=-11.10, top=-9.50)
TOP_MM = 14.0                                   # the motor blocks' top over the axle (collisions)

# ---- From its board (perenquen.kicad_pcb): each sensor's pair of 5 mm SFH4545 emitter and 3 mm TEFT4300
# phototransistor, lying on the board 3 mm over it (horizontal footprints): their middle, lens tips, and axis (deg
# from +x counter-clockwise). The diagonal ones look 60 deg out of the front; their two emitters are in series,
# lit by either RD4 or RD3 (the firmware lights RD4 for both).
IR_Z_MM = 4.6 - 11.10
IR_SENSORS = {          # name: ADC channel, emitter's LATD bit, (x, y), axis
    'FL': (1, 5, (-30.94, 27.44), 92.0),
    'FR': (5, 2, (30.67, 27.36), 88.0),
    'DL': (3, 4, (-20.39, 44.30), 150.0),
    'DR': (4, 4, (20.27, 44.41), 30.0),
}
ADC_BATTERY, ADC_GYRO, ADC_GYRO_REF, ADC_FLASH = 0, 30, 29, 2

# ---- Assumed (nothing measured) ----------------------------------------------------------------------------------
# The IR pairs: no calibration in its firmware. The reflected light as the inverse square of the distance along the
# beam, IR_K counts at 1 mm: its front pair crosses its obstacle threshold (350) at 100 mm, the end of its turns
# (150) at 153 mm, its start (200) at 132 mm; the side walls give its diagonals ~590 counts from a cell's centre.
IR_K = 350 * 100.0 ** 2
IR_AMBIENT = 8                                  # counts, emitters off (the maze's lighting)
IR_FAR_MM = 400.0
IR_SPREAD_DEG = 4.0
IR_NOISE = 3.0                                  # counts, gaussian
# Its design notes (the column "Perenquén"): 110 g, a Pololu micro metal gearmotor HPCB 6 V 5:1 a side (6000 rpm
# free and 1.6 A stall at 6 V, 0.12 A free), 2S; its gears as built: a 20-tooth pinion between two 45-tooth wheel
# gears (s50b20b, ds0.5-45, S50D45), so 11.2:1 (its notes asked for 10).
MASS_KG = 0.110
COM_MM = (0.0, -4.0, 0.0)                       # just behind its front axle: it turns on its front wheels
BATTERY_V = 7.6
GEAR = 4.995 * 45 / 20
MOTOR_R_OHM = 6.0 / 1.6
MOTOR_KE = 0.185e-3 * 60 / (2 * math.pi)        # V s/rad (its notes' 0.185 mV/rpm)
MOTOR_FRICTION_NM = MOTOR_KE * 0.12             # its free-running current
MOTOR_ROTOR_INERTIA = 1e-9                      # kg m^2
GEAR_EFFICIENCY = 0.8
TIRE_FRICTION = 0.7                             # its notes' friction coefficient


# ---- Its start (ratasim/start.py) --------------------------------------------------------------------------------
def start(side='right', gain=30, speed=None):
    """As its maker drove it, from a terminal over Bluetooth: in its start cell, the speed and `start <side> <gain>`
    (it reads its diagonal sensors there: the walls it will keep to); then a hand in front of both front sensors,
    taken away: it goes (strat_follow_wall, commands_mainboard.c)."""
    steps = [('say', 'booting'), ('wait', 1500)]
    if speed:
        steps += [('say', f'traj_speed distance {speed}'), ('send', f'traj_speed distance {speed}'), ('wait', 300)]
    steps += [('say', f'start {side} {gain}'), ('send', f'start {side} {gain}'), ('wait', 500),
              ('say', 'a hand in front of both front sensors, then away: GO'),
              ('hand', ['FL', 'FR'], 600), ('wait', 400)]
    return steps
