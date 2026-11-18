"""NezumiBot (OPRobots, github.com/OPRobots/NezumiBot) in the simulator: where its repository is, its geometry and
the physics assumed for it. Ratatron's chassis and CAD frame (ratasim/config.py): x right, y forward, z up, mm."""
import math
import os

from .. import config as RC
from .. import paths

NEZUMI_DIR = os.environ.get('NEZUMIBOT_DIR', os.path.join(paths.EXTERNAL, 'NezumiBot'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its meshes, its firmware library

# ---- From its firmware (include/config.h, sensors.h, motors.c) -------------------------------------------------
MICROMETERS_PER_TICK = 78.3699
PWM_MAX = 1024                                  # MOTORES_MAX_PWM
# Its Sharp GP2Y0A51SK0F where Ratatron has them (its chassis), in its DMA order (sensors_raw): front left (PA3),
# left (PA2), right (PA1), front right (PA0); Ratatron's names for them.
SENSORS = ('FL', 'SL', 'SR', 'FR')

# ---- From its 3D files (3d_model/stl_files) --------------------------------------------------------------------
# Its N20 wheels (Rueda.stl): 22.1 mm, a cup around the gearbox, 18.7 mm wide; their outer faces where Ratatron's
# tyres' are, their axle under Ratatron's so the body stands as high.
WHEEL_RADIUS_MM = 11.05
WHEEL_WIDTH_MM = 18.7
WHEEL_OUTER_X_MM = 36.0
CONTACT_X_MM = WHEEL_OUTER_X_MM - WHEEL_WIDTH_MM / 2
AXLE_Z_MM = RC.GROUND_Z_MM + WHEEL_RADIUS_MM

# ---- Assumed (nothing measured) ----------------------------------------------------------------------------------
MASS_KG = 0.100
BATTERY_V = 7.4                                 # a 2S LiPo
# N20 6 V 30:1 with a 7-pulse magnetic encoder (its 78.37 um a tick on these wheels: 886 ticks a turn, 31.6:1)
GEAR = 2 * math.pi * WHEEL_RADIUS_MM * 1000 / MICROMETERS_PER_TICK / 28
MOTOR_KE = 6.0 / (30000 * 2 * math.pi / 60)    # V s/rad: 30000 rpm free at 6 V
MOTOR_R_OHM = 10.0
MOTOR_FRICTION_NM = 1.2e-4                      # at the motor shaft
MOTOR_ROTOR_INERTIA = 1e-9                      # kg m^2
GEAR_EFFICIENCY = 0.7
TIRE_FRICTION = 0.8


# ---- Its start (main.c at its OSHWDEM 2023 tag; ratasim/start.py) ------------------------------------------------
def start(hand='left', party=False):
    """Held in the air, a hand at a side sensor picks the party (left: it swaps walls every 30 s after 45 s; right:
    no party), another the wall it follows; put down in the start cell, a hand in front: it starts 2 s later."""
    side = {'left': 'SL', 'right': 'SR'}
    return [('lift',), ('say', 'held in the air: booting'), ('wait', 1000),
            ('say', 'a hand at its ' + ('left sensor: party' if party else 'right sensor: no party')),
            ('hand', [side['left' if party else 'right']], 250), ('wait', 1200),
            ('say', f'a hand at its {hand} sensor: it follows the {hand} wall'),
            ('hand', [side[hand]], 250), ('wait', 1200),
            ('say', 'put down in the start cell; a hand in front'), ('put_down', (0, 0), 0.0), ('wait', 500),
            ('hand', ['FL', 'FR'], 300), ('wait', 2500)]
