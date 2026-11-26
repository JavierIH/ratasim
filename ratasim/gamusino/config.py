"""Gamusino (turmandreams, github.com/turmandreams/gamusino) in the simulator: where its 3D model is and the physics
assumed for it. Its geometry comes from the model (cad.py, build/cad/geometry.json). Robot frame (mm): x right,
y forward, z up, origin on the axle."""
import os

from .. import paths

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')
# Its model: the Blender export in its repository (the submodule external/gamusino)
CAD_GLB = os.environ.get('GAMUSINO_GLB', os.path.join(paths.EXTERNAL, 'gamusino', 'gamusino_3d', 'gamusino.glb'))
CAD_MESHES = os.path.join(BUILD, 'cad')

# ---- Assumed physics (nothing measured) --------------------------------------------------------------------------
MASS_KG = 0.090             # two LiPo cells, two micro gearmotors, a suction motor, ESP32 board, printed chassis
COM_MM = (0.0, 2.0, 4.0)
TIRE_FRICTION = 1.0

# ---- Its firmware (external/gamusino/src/gamusino/gamusino.ino, an Arduino sketch for its ESP32-WROOM-32) ---------
SKETCH = os.path.join(paths.EXTERNAL, 'gamusino', 'src', 'gamusino', 'gamusino.ino')
# The library its maker installs for its gyro (CAMBIOS.md: "STM32duino LSM6DSR"), BSD-3: downloaded at build time,
# pinned at a commit (its version 2.3.0).
LSM6DSR_COMMIT = '1cffb2e7676ba72360dad2ecea4f74ff7939b353'
LSM6DSR_URL = 'https://raw.githubusercontent.com/stm32duino/LSM6DSR/' + LSM6DSR_COMMIT + '/src/'
LSM6DSR_FILES = ('LSM6DSRSensor.h', 'LSM6DSRSensor.cpp', 'lsm6dsr_reg.h', 'lsm6dsr_reg.c')
# Its pins (the sketch): its suction motor's (a MOSFET, LEDC channel 4), each motor's two H-bridge inputs (backward, forward: its pwmmotor() drives the second one
# going forward), each encoder's A pin, the gyro's chip select.
MOTOR_PINS = {'l': (2, 13), 'r': (17, 16)}
SUCTION_PIN = 12
ENCODER_PINS = {'l': 22, 'r': 25}
IMU_CS = 5
# Its odometry: M_POR_P, 5.336 um a count on its 20 mm wheels (11776 counts a turn in full quadrature).
M_PER_COUNT = 5.336e-6
WHEEL_RADIUS_M = 0.010
# Its motors' speed for each PWM duty (of 255), measured by its maker with its wheels in the air
# (src/pruebas/pwmtovel/resultados.txt; the sketch's tables velTablaIzq/Der): nothing below 23, and its last measured
# speed above 73.
PWM_TABLE = {'l': ((0, 22, 23, 25, 28, 31, 34, 38, 42, 48, 52, 63, 71),
                   (0.0, 0.0, 0.85, 0.98, 1.09, 1.15, 1.28, 1.34, 1.44, 1.51, 1.59, 1.66, 1.67)),
             'r': ((0, 23, 24, 26, 28, 31, 34, 38, 43, 48, 54, 61, 73),
                   (0.0, 0.0, 0.82, 0.92, 1.03, 1.17, 1.31, 1.41, 1.50, 1.57, 1.65, 1.71, 1.79))}

# ---- Assumed (nothing measured) ----------------------------------------------------------------------------------
# Its H-bridge (its own, of MOSFETs): each input drives a half bridge, so both inputs low (or high) short its motor:
# it brakes. Its motors (coreless, geared down by printed spur gears): the force a wheel pushes with, DRIVE_N_PER_MPS
# for each m/s its wheel runs below the speed of its duty (the table).
DRIVE_N_PER_MPS = 0.5
# Its LSM6DSR: a zero-rate offset (its sketch measures it at boot), its noise at 1666 Hz (6 mdps/sqrt(Hz)); its axes
# the robot's: x right, y forward, z up.
GYRO_BIAS_DPS = (0.4, -0.3, 0.6)
GYRO_NOISE_DPS = 0.15
ACCEL_NOISE_G = 0.003


# Its wall sensors: four boards of an IR LED over a phototransistor (its model: IRboard0..3, TX over RX), two at the
# front looking ahead and two crossed, each looking 45 deg over the other side. Its sketch lights the LEDs with two
# pins (pinIR_I, pinIR_D) and reads each phototransistor on the ESP32's ADC, LEDs off and on (valorsin, valorcon).
# Name (its sketch's): ADC pin, LED pin, (x, y) of its lens (mm, the model's), its axis (deg from ahead, left +).
# Which board is which, assumed: IA, DA the front ones on its left and right; IW, DW the crossed ones looking at its
# left and right wall; each LED pin lights its side's two (its sketch lights both at once).
IR_SENSORS = {
    'IA': (36, 32, (-18.7, 27.7), 10.0),
    'DA': (35, 33, (18.8, 27.8), -10.0),
    'IW': (34, 32, (7.6, 34.3), 45.0),
    'DW': (39, 33, (-8.1, 34.5), -45.0),
}
IR_Z_MM = 0.25              # its phototransistors' lenses over the axle
# Their readings (12 bits), assumed: the maze's light with its LEDs off, and the light back from a wall as the
# inverse square of the beam's length, IR_K counts at 1 mm (its sketch's test lights an LED from 3500 counts over the
# dark: assumed at 50 mm of a wall), up to the ADC's top.
IR_DARK = 150
IR_K = 3500 * 50.0 ** 2
IR_NOISE = 8.0              # counts, gaussian
IR_FAR_MM = 400.0
# Its suction: a 7 mm coreless motor with a printed impeller under the board. Its downforce at full duty, assumed
# (nothing measured), quadratic in it.
SUCTION_N = 1.0


# ---- Its menu and its start (ratasim/start.py) -------------------------------------------------------------------
# Its menu (its sketch's loop()): every 200 ms it reads its encoders; its right wheel turned by hand forward or back
# (more than 1000 counts, 5.3 mm) moves to the next or the previous option, its left wheel forward runs it. Its four
# LEDs show the option plus one in binary (LED 0 the lowest bit).
OPTIONS = {
    0: 'its encoders\' test (10 s: its LEDs green as each wheel is turned)',
    1: 'its gyro\'s test: its gyro measured, then four 90 deg turns on the spot, 2 s each',
    2: 'its motors\' test: each motor 1 s forward and 1 s back at 40/255',
    3: 'its gyro measured (1.2 s)',
    4: 'its suction\'s test: 5 s at 60/255',
    5: 'its wall sensors\' test (15 s: an LED green for each sensor seeing a wall close)',
    6: '1.62 m ahead at 1.6 m/s steering by its gyro and encoders, a brake, a 180 deg turn on the spot, a stop',
}
FLICK_TURNS = 0.15          # a flick of a wheel by hand (1766 counts in 20 ms): one step of its menu
FLICK_MS = 20


def option_from_leds(leds):
    """The option its LEDs show (four colours), or None."""
    n = sum(1 << i for i, c in enumerate(leds) if any(c))
    return n - 1 if n else None


def start(cell, heading_deg, option=6):
    """As its maker starts it: switched on (red, green, blue, 0.9 s), held up, its right wheel flicked to the option
    (forward, or back past 0 to 6), its left wheel flicked forward to run it; put down while its LEDs blink and go red
    (3 s). Options 1, 3 and 6 measure its gyro then (1.2 s: not to be touched) and light blue for 1 s; 6 goes red and
    runs."""
    steps = [('say', 'switched on: red, green, blue; its menu at option 0'), ('wait', 1200), ('lift',), ('wait', 200)]
    turns = option if option <= 3 else option - 7
    steps += [('say', f'held up, its right wheel flicked {abs(turns)} time(s) {"forward" if turns > 0 else "back"}: '
                      f'option {option}')] if turns else []
    steps += [('wheel', 'r', FLICK_TURNS if turns > 0 else -FLICK_TURNS, FLICK_MS), ('wait', 400)] * abs(turns)
    steps += [('say', f'its left wheel flicked forward: option {option}, {OPTIONS[option]}'),
              ('wheel', 'l', FLICK_TURNS, FLICK_MS), ('wait', 800), ('put_down', cell, heading_deg)]
    if option in (1, 3, 6):
        steps += [('say', 'put down: red, its gyro measured after 3 s (not to be touched)'), ('wait', 5000)]
    return steps
