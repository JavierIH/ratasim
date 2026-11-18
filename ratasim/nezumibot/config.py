"""NezumiBot (OPRobots, github.com/OPRobots/NezumiBot) in the simulator: where its repository is, its geometry and
the physics assumed for it. Its frame (its 3D model, assets/): x right, y forward, z up, mm, origin between the wheels
at the axle's height."""
import json
import math
import os

from .. import paths

NEZUMI_DIR = os.environ.get('NEZUMIBOT_DIR', os.path.join(paths.EXTERNAL, 'NezumiBot'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its firmware library

# ---- From its firmware (include/config.h, sensors.h, motors.c) -------------------------------------------------
MICROMETERS_PER_TICK = 78.3699
PWM_MAX = 1024                                  # MOTORES_MAX_PWM
# Its Sharp GP2Y0A51SK0F where Ratatron has them (its chassis), in its DMA order (sensors_raw): front left (PA3),
# left (PA2), right (PA1), front right (PA0); Ratatron's names for them.
SENSORS = ('FL', 'SL', 'SR', 'FR')

# ---- From its 3D model (assets/, nezumi_params.json) ------------------------------------------------------------
ASSETS = os.path.join(HERE, 'assets')
PARAMS = json.load(open(os.path.join(ASSETS, 'nezumi_params.json')))
# Its Rueda.stl rims on the N20 gearboxes, Ø30 tyres (measured) 11 mm wide, 70.4 mm apart.
WHEEL_RADIUS_MM = PARAMS['tire_radius_mm']
WHEEL_WIDTH_MM = PARAMS['tire_width_mm']
CONTACT_X_MM = PARAMS['tire_center_x_mm']
AXLE_Z_MM = PARAMS['axle_z_mm']
GROUND_Z_MM = PARAMS['ground_z_mm']             # -15
HULL_BOX_MM = PARAMS['hull_box_mm']             # base, lid and rear cover
SKIDS = PARAMS['skids']                         # two 3 mm foam pads on the floor, front and back
# Its Sharp GP2Y0A51SK0F where Ratatron has them (its base is Ratatron's chassis): the centre of each front face and
# its direction (deg from +x, counter-clockwise), 4 mm over the axle.
IR_SENSORS = {n: (tuple(v['pos_mm']), v['dir_deg']) for n, v in PARAMS['ir_sensors'].items()}
MASS_KG = PARAMS['mass_kg_estimate']            # estimated by parts (2S LiPo ~20 g, N20 ~10.5 g each, PLA by volume)
COM_MM = tuple(PARAMS['com_mm_estimate'])

# ---- Assumed (nothing measured) ----------------------------------------------------------------------------------
BATTERY_V = 7.4                                 # a 2S LiPo
# N20 6 V with a 7-pulse magnetic encoder: its 78.37 um a tick on Ø30 tyres, 1203 ticks a turn (43:1)
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
