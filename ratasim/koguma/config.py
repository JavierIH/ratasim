"""KOGUMA-CHAN Mk3 (Akihiro Suda, Tokyo University of Science Mice; github.com/Tokoro0917/KOGUMA.MkIIl) in the
simulator: where its repository is, its geometry and the physics assumed for it. Robot frame (mm): x right, y forward,
z up, origin on the axle between the wheels.

Its specification (APEC 2026 Micromouse Contest report, where it was second, with the fastest run, 6.308 s): 120 x 70 x
30 mm, 78 g, STM32F446RE at 180 MHz, 24 x 8 mm wheels, turns up to 2.4 m/s, 30 m/s^2, 6.0 m/s; IR SFH 229FA photodiodes
and OPV332 laser diodes x 4; NFP-D0816-3-3.4 motors (two to drive, one for its fan) on MP6550 drivers; a 4-cell
LiPo (15.2 V).

Its repository has no licence: its author has given his permission to use it. Nothing of it is copied here: the
submodule external/KOGUMA.MkIIl is read at build time, its firmware built from it (fw/), its meshes made from its
STEP (cad.py)."""
import math
import os

from .. import paths

KOGUMA_DIR = os.environ.get('KOGUMA_DIR', os.path.join(paths.EXTERNAL, 'KOGUMA.MkIIl'))
HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its meshes, its firmware's library (not committed)

# ---- Its 3D model: its Fusion 360 assembly as a STEP (hardware/KOGUMA.Mk3.step, every part where it is) -----------
CAD_STEP = os.environ.get('KOGUMA_STEP', os.path.join(KOGUMA_DIR, 'hardware', 'KOGUMA.Mk3.step'))
CAD_MESHES = os.path.join(BUILD, 'cad')
AXLE_IN_STEP_MM = (-6.8, 0.9, 71.3)             # its tyres' centres (cad.py checks it); its floor at z 59.3
TIRE_CAD_RADIUS_MM = 12.0                       # Mini-Z narrow tyres, 24 mm
TRACK_MM = 60.6                                 # between its tyres' middles (8 mm wide)
# Its carbon fibre bottom plate (120 x 72 mm, 1 mm, 2.5 mm over the floor in its STEP) and its board over it
PLATE_Z_MM = (61.8 - AXLE_IN_STEP_MM[2], 62.8 - AXLE_IN_STEP_MM[2])
# Its plate's outline (its STEP's, its convex hull simplified): 72 mm across its middle, tapering to its nose and tail
PLATE_OUTLINE = ((-36.0, -20.6), (-10.4, -59.4), (-9.2, -60.0), (9.2, -60.0), (10.4, -59.4), (36.0, -20.6),
                 (36.0, 21.0), (7.4, 57.0), (3.5, 59.7), (-3.5, 59.7), (-7.4, 57.0), (-36.0, 21.0))
TOP_MM = 91.3 - AXLE_IN_STEP_MM[2]              # its fan motor's cover
TAIL_MM = 60.0                                  # its plate's tail behind the axle (its firmware: 120 mm long)
START_BACK_MM = 84.0 - TAIL_MM - 0.3            # its tail 0.3 mm off the start cell's back wall: its axle 24 back

# IR sensors (3 mm OPV332 laser diodes and SFH 229FA photodiodes, through op-amps, in printed holders, each pair one
# over the other): where each beam starts (x, y: its emitter's hole in its holder's face) and its axis (deg from +x
# counter-clockwise, 90 = forward: its holder's face's normal), from its STEP's holders (cad.holder_holes). The front
# ones look 10 deg out; the side ones are crossed: the one looking at the left wall stands right of the middle, 30 deg
# ahead of square.
# Its firmware's index (g_sensor): FL 0, R 1, L 2, FR 3; its ADC channels 13, 12, 11, 10.
IR_SENSORS = {
    'FL': (0, (-27.35, 24.4), 100.0),
    'R': (1, (-9.35, 37.35), 30.0),
    'L': (2, (9.35, 37.35), 150.0),
    'FR': (3, (27.35, 24.4), 80.0),
}
# Its side sensors' axes as its robot sees with them: 5 (right) and 3 (left) deg further ahead than its STEP's 30, from
# its maker's wall-cut test (docs/pillar_cut_design.md 12; logs/20261018-16*_edge_v1000.txt: its side sensors' signal
# each mm past a lone post, a wall's end, on either side, at 1000 mm/s; at speed 0, 6 mm later, its Edge_V_K). With its
# PillarEdge's rule (half the last 30 mm's highest) on the simulated signal, robot centred, the five ends fall 3.4 mm
# (rms) from the real ones; as its STEP has them, 10 to 20 mm late (its firmware, with its Edge_Off measured on the
# robot, put the borders 20 mm further on than they were and began its turns late). Its maker suspected it: his right
# one's Edge_Off 7-12 mm more than his left one's, "its sensors' angle or mounting a few degrees off" (1 deg: 2.2 mm).
# The beam's width and the cosine of incidence hardly move them (1.5 to 4 deg). (The test's left side, as its signal
# shows it: a wall in cell 1, then lone posts; its doc says a wall in cell 3 too.)
SIDE_AHEAD_DEG = {'R': 5.0, 'L': 3.0}
# And, as its maker measured in the same test, the faster it goes the earlier its side sensors see the ends: 6 mm a
# m/s (1000, 2000, 2400 mm/s; every end, either side, their rises too: "not the way they are found: something on the
# side of the distance or the time; the cause is not known"; its firmware adds it back, Edge_V_K 0.006). Modelled as
# measured: its side sensors see from where it will be this much later (at speed 0 nothing changes).
SIDE_LEAD_S = float(os.environ.get('KOGUMA_SIDE_LEAD_S', '0.006'))
IR_SENSORS['R'] = IR_SENSORS['R'][:2] + (IR_SENSORS['R'][2] + SIDE_AHEAD_DEG['R'],)
IR_SENSORS['L'] = IR_SENSORS['L'][:2] + (IR_SENSORS['L'][2] - SIDE_AHEAD_DEG['L'],)
IR_Z_MM = 5.56                  # its emitters' axis over its axle (its photodiodes' 4.6 mm under)

# ---- From its firmware (external/KOGUMA.MkIIl/Core) --------------------------------------------------------------
WHEEL_RADIUS_MM = 23.42 / 2     # Define.h TIREDIAMETER: its tyres measured (2026-10-21)
GEAR_RATIO = 1.7073             # Define.h GEARRATIO: its encoders' gear (41 teeth) turns this much a wheel's turn
                                # (70:41; its STEP's spur gear has 71)
TREAD_MM = 55.0                 # Define.h TIREBETWEEN (its feed-forward's)
# Its wall sensors' readings (lit minus dark, ADC counts) at the distances it measured (WallDistance.c, 2026-10-06):
# square in a cell, the side sensors at D mm from its middle line to the wall (D = 84 centred), the front ones at
# D mm from its middle (its axle) to the wall ahead (D = 84 in a cell's middle); its venue's scale (WallDist_Scale,
# FL, R, L, FR) on them.
IR_TABLE = {
    'FL': ((60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204),
           (1366, 902, 645, 470, 356, 295, 230, 195, 167, 146, 132, 123, 112)),
    'R': ((37, 49, 61, 73, 85, 97, 109, 121, 133), (780, 522, 356, 262, 201, 160, 131, 110, 93)),
    'L': ((37, 49, 61, 73, 85, 97, 109, 121, 133), (910, 603, 415, 308, 237, 183, 150, 125, 105)),
    'FR': ((60, 72, 84, 96, 108, 120, 132, 144, 156, 168, 180, 192, 204),
           (1377, 940, 627, 461, 358, 281, 248, 204, 175, 155, 129, 120, 112)),
}
IR_SCALE = {'FL': 0.99, 'R': 0.96, 'L': 1.00, 'FR': 1.02}
IR_DARK = 40.0                  # a photodiode's dark reading (counts), assumed
IR_NOISE = 2.0                  # its readings' noise (counts): its docs/noise_limits.md, 1.8 to 2.8 standing
GYRO_NOISE_DPS = 0.15           # its LSM6DSR at 1.66 kHz (about one LSB)
GYRO_BIAS_DPS = 0.4             # its firmware measures it before each run (Gyro_Calibrate)

# ---- Physics: its mass (Define.h ROBOT_M; 78 g in its APEC 2026 specification), the rest fitted to its logs ---------
MASS_KG = 0.078
COM_MM = (0.0, 0.0, 2.0)
YAW_INERTIA = 3.7e-5            # kg m^2: docs/next_robot.md, from its logs (the turns' voltage)
BATTERY_V = 16.0                # 4S LiPo, 180 mAh cells: its maker runs it from 15.8 V up
BATTERY_R_OHM = 0.64            # its battery and wiring (docs/next_robot.md: 0.56 to 0.79, median 0.64)
TIRE_FRICTION = 1.2
# Its tyres' grip on the floor, single contact points (ellipsoids: a rigid cylinder's four points across its 8 mm
# scrub far more than a rubber tyre, which twists), their friction stiff (MuJoCo's solreffriction: its time
# constant, its damping ratio): what slips in a turn is a fraction of a degree, its turns' voltages as its logs'
TYRE_CREEP = (0.002, 1.0)
# Its tyres sliding sideways in its turns (ratasim/tyre.py): their cornering stiffness, per radian of slip angle.
TYRE_CORNERING = float(os.environ.get('RATASIM_TYRE_C', '0')) or None
# Its motors (NFP-D0816S coreless, 8 mm; 5.4 ohm, Define.h MOTORR) at the wheel: torque = K (V - K w) / R, its
# gears' and fan's losses as friction. Fitted to the voltage its logs need per wheel (docs/next_robot.md: V = 1.25 +
# 1.5 v + 0.35 a, v in m/s, a in m/s^2): K from the 0.35 (half its mass at the wheel), viscous friction for the rest
# of the 1.5 (its back-EMF gives 0.60), Coulomb friction for the 1.25.
MOTOR_R_OHM = 5.4
MOTOR_K = MOTOR_R_OHM * MASS_KG / 2 * (WHEEL_RADIUS_MM / 1000) / 0.35          # N m/A at the wheel
WHEEL_VISCOUS = (1.5 - MOTOR_K / (WHEEL_RADIUS_MM / 1000)) * MOTOR_K * (WHEEL_RADIUS_MM / 1000) / MOTOR_R_OHM
WHEEL_FRICTION_NM = 1.25 * MOTOR_K / MOTOR_R_OHM
# Its tyres' scrub turning: its logs' rotation needs 0.063 V a wheel per rad/s of yaw (docs/next_robot.md), its
# motors' model 0.045 (1.5 V per m/s at its 30.3 mm): the rest, 0.018 V, a yaw damping on its body.
YAW_DAMPING = 2 * (MOTOR_K * 0.018 / (MOTOR_R_OHM * WHEEL_RADIUS_MM / 1000)) * TRACK_MM / 2000    # N m s/rad
WHEEL_ARMATURE = 1e-7           # its rotor, pinion, spur gear and encoder's gear at the wheel (kg m^2), assumed: with
                                # less its wheels, light without its fan, spin and chatter against a wall
# Its fan (a coreless motor, a 32 mm acrylic impeller, a skirt): its downforce at 11 V (its runs' duty 70 % of
# 15.8 V), quadratic in the voltage, assumed: its fastest run turns at 2 m/s and 2000 deg/s (ShortParam.c: 70 m/s^2,
# docs/next_robot.md: 64 to 70 measured) with its fan at 50 % (7.9 V); at TIRE_FRICTION that needs 4.2 N there, and
# its logs found its turns short of voltage, not of grip: 9 N at 11 V gives it 4.6 N (83 m/s^2).
FAN_DOWNFORCE_N = 9.0
FAN_RATED_V = 11.0
FAN_CURRENT_A = 0.5             # docs/next_robot.md: about 0.5 A of its peak current


def encoder_turns_per_rad():
    """Its encoder shaft's turns a radian of its wheel."""
    return GEAR_RATIO / (2 * math.pi)


def ir_beam_mm(name, d_mm):
    """A sensor's beam length to a wall d_mm from its middle line (side) or its axle (front), square in a cell."""
    _, (x, y), deg = IR_SENSORS[name]
    a = math.radians(deg)
    if name in ('FL', 'FR'):
        return (d_mm - y) / math.sin(a)
    return (d_mm + abs(x)) / abs(math.cos(a))


# ---- Its start (ratasim/start.py) ----------------------------------------------------------------------------------
def start(cell, heading_deg, number=1):
    """As its maker starts a search: switched on in the start cell, its tail against the back wall; held up, its right
    wheel turned forward to pick the number of its mode 0 (each half a turn of the wheel, its encoder's gear past
    its 300 deg: one more, its LEDs 5-8 show it; what is turned past 300 deg counts towards the next); put down, a
    hand in front of its left front sensor (its buzzer), then over its two side sensors: it waits for the hand to go
    (its gyro's zero, 0.3 s) and goes. Mode 0 No.1: its slalom search at 500 mm/s, no fan, to the goal and back."""
    out = [('put_down', cell, heading_deg), ('say', 'switched on'), ('wait', 1500),
           ('say', f'held up, its right wheel turned: mode 0, No.{number}'), ('lift',), ('wait', 300)]
    out += [('wheel', 'r', 0.52, 400), ('wait', 500)] * number      # 320 deg of its encoder: one more
    out += [('put_down', cell, heading_deg), ('wait', 800),
            ('say', 'a hand in front of its left front sensor'), ('hand', ['FL'], 400), ('wait', 700),
            ('say', 'a hand over its side sensors'), ('hand', ['L', 'R'], 600), ('wait', 900)]
    return out
