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
# Its pins (the sketch): each motor's two H-bridge inputs (backward, forward: its pwmmotor() drives the second one
# going forward), each encoder's A pin, the gyro's chip select.
MOTOR_PINS = {'l': (2, 13), 'r': (17, 16)}
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


# ---- Its start (ratasim/start.py) --------------------------------------------------------------------------------
def start(cell, heading_deg):
    """As its maker starts it: put down, switched on. Its sketch lights its LEDs green, waits 3 s, measures its gyro's
    offset (1.2 s: not to be moved), lights them blue and goes: 1.5 m ahead, a 180 deg turn, back."""
    return [('put_down', cell, heading_deg),
            ('say', 'switched on: green, its gyro measured in 3 s (not to be touched)'), ('wait', 4300)]
