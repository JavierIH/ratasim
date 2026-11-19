"""Futura (Green Ye, UCLA IEEE, micromouseusa.com/?page_id=1342) in the simulator: where its files come from, its
geometry and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin between its wheels at the
axle's height.

Sources: its kit's firmware template (FuturaProject.zip: user/, user_Libraries/), its schematic (Futura V1.1:
STM32F405, LY3200ALH gyro, four SFH4545 and TEFT4300 IR pairs, TC4427 and ZXMHC3F381N8 H-bridges, HCMS-2903
display), its 3D printing files (futura-3D-printing-model.zip: wheel, skate) and the photo of the kit on its page.
Its page publishes no licence: nothing of it is in this repository; its two zips are downloaded at build time
(download()), pinned by their SHA-256.
"""
import hashlib
import math
import os
import ssl
import urllib.error
import urllib.request
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')             # its downloads, its sources unzipped, its firmware library, meshes

# ---- Its files (its page's links), downloaded at build time ------------------------------------------------------
UPLOADS = 'http://micromouseusa.com/wp-content/uploads/'
FILES = {       # name: URL, SHA-256
    'FuturaProject.zip': (UPLOADS + '2014/01/FuturaProject.zip',
                          'cc30612c3a2e369499eac6885a021204f321ae20e0d8728735683bf593e0633d'),
    'futura-3D-printing-model.zip': (UPLOADS + '2014/03/futura-3D-printing-model.zip',
                                     '4db946b2dc12ea9a537e096e7bf4131c5c394aa34e36ecc0c0393432db95ef05'),
}
PROJECT = os.path.join(BUILD, 'FuturaProject')  # its firmware (FuturaProject.zip: user/, user_Libraries/)
PRINTS = os.path.join(BUILD, 'prints')          # its printed parts (STL)


def _sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def _get(url, path):
    """The file at url to path. Its site's certificate expired (2026): if it is refused for that, the file is fetched
    without checking it; the SHA-256 checked after (download()) is what vouches for its bytes."""
    try:
        data = urllib.request.urlopen(url, timeout=60).read()
    except urllib.error.URLError as e:
        if not isinstance(e.reason, ssl.SSLCertVerificationError):
            raise
        data = urllib.request.urlopen(url, timeout=60, context=ssl._create_unverified_context()).read()
    with open(path + '.part', 'wb') as f:
        f.write(data)
    os.replace(path + '.part', path)


def download():
    """Its two zips in BUILD (once; FUTURA_DOWNLOADS: a directory where they already are), checked, unzipped: its
    firmware's sources to PROJECT, its STL files to PRINTS."""
    os.makedirs(BUILD, exist_ok=True)
    local = os.environ.get('FUTURA_DOWNLOADS')
    for name, (url, sha) in FILES.items():
        path = os.path.join(BUILD, name)
        if not os.path.exists(path):
            if local and os.path.exists(os.path.join(local, name)):
                with open(os.path.join(local, name), 'rb') as src, open(path, 'wb') as dst:
                    dst.write(src.read())
            else:
                _get(url, path)
        if _sha256(path) != sha:
            os.unlink(path)
            raise RuntimeError(f'{name}: not the file pinned in ratasim/futura/config.py (SHA-256 {sha})')
    if not os.path.exists(os.path.join(PROJECT, 'user', 'main.c')):
        with zipfile.ZipFile(os.path.join(BUILD, 'FuturaProject.zip')) as z:
            z.extractall(BUILD, [n for n in z.namelist() if n.startswith(('FuturaProject/user/',
                                                                          'FuturaProject/user_Libraries/'))])
    if not os.path.exists(os.path.join(PRINTS, 'skate.stl')):
        with zipfile.ZipFile(os.path.join(BUILD, 'futura-3D-printing-model.zip')) as z:
            z.extractall(PRINTS, [n for n in z.namelist() if n.endswith('.stl')])


# ---- From its firmware (user_Libraries) --------------------------------------------------------------------------
PWM_PERIOD = 1000                               # TIM4: ARR 999, 21 kHz (pwm.c)
PWM_DEMO = 100                                  # main.c: setLeftPwm(100), setRightPwm(100)
ADC_FULL = 4095                                 # 12 bits, 3.3 V
# The IR receivers' ADC channels (adc.h) and the pin that lights each one's emitter (led.h): GPIO port, bit.
IR_CHANNELS = {'LF': (9, ('C', 7)), 'RF': (4, ('C', 1)), 'DL': (8, ('A', 7)), 'DR': (5, ('A', 7))}
ADC_BATTERY, ADC_GYRO, ADC_VREF = 3, 15, 14     # read_Vol_Meter, read_Outz, read_Vref
BATTERY_COUNTS_PER_V = 3248 / 8.09              # sensor_Function.c: voltage = volMeter * 809 / 3248 (10 mV)
GYRO_ZERO_COUNTS = 92980000 / (50000 / 20) / 20  # readGyro(): its zero-rate offset, 1859.6 counts (1.50 V)
GYRO_V_PER_DPS = 0.00067                        # LY3200ALH (its schematic): 0.67 mV/dps

# ---- Its printed parts (futura-3D-printing-model.zip) ------------------------------------------------------------
WHEEL_STL_CENTRE = (28.95, -6.005)              # "solid wheel wide.stl": 32 mm across, 9 mm wide, its axis z
SKATE_STL_CENTRE = (20.51, -0.955)              # "skate.stl": 10 x 10 x 3 mm
WHEEL_RADIUS_MM = 16.0 + 1.0                    # its printed wheel and a tyre (assumed 1 mm: the photo's rubber)
WHEEL_WIDTH_MM = 9.0

# ---- From the photo of the kit (assumed: no board file at hand): sizes by its 32 mm wheels --------------------------
# Its board: a rectangle between its wheels and a half disc for its nose (its sensors on the arc).
TRACK_MM = 74.0                                 # the wheels' centres: its two motors end to end between them
BOARD = dict(half_width=32.0, centre_y=10.0, back=-24.0, bottom=-14.0, thickness=1.6)
SKATE_Y_MM = 33.0                               # its skate under its nose, 3 mm: the board 3 mm off the floor
GROUND_Z_MM = -WHEEL_RADIUS_MM
# Each IR pair on the arc of its nose (the photo: seven parts along it; LF and RF look ahead, DL and DR 45 deg out):
# its middle (x, y) and axis (deg from +x).
IR_Z_MM = BOARD['bottom'] + BOARD['thickness'] + 3.0


def _on_arc(deg, inset=3.0):
    r = BOARD['half_width'] - inset
    return (round(r * math.cos(math.radians(deg)), 2), round(BOARD['centre_y'] + r * math.sin(math.radians(deg)), 2))


IR_SENSORS = {'LF': (_on_arc(115), 90.0), 'RF': (_on_arc(65), 90.0),
              'DL': (_on_arc(150), 135.0), 'DR': (_on_arc(30), 45.0)}

# ---- Assumed (nothing measured) ----------------------------------------------------------------------------------
# The IR pairs: the light back as the inverse square of the distance along the beam; its template prints the raw
# differences and uses no threshold: IR_K gives ~1900 counts (lit minus dark) at 40 mm, ~300 at a cell (100 mm).
IR_K = 1900 * 40.0 ** 2
IR_AMBIENT = 60                                 # counts, emitters off (the maze's lighting)
IR_FAR_MM = 400.0
IR_SPREAD_DEG = 4.0
IR_NOISE = 6.0                                  # counts, gaussian
MASS_KG = 0.085
COM_MM = (0.0, 8.0, -4.0)                       # ahead of its axle (its board's nose), on its skate
BATTERY_V = 7.6                                 # 2S 120 mAh (its kit: "3.7V 120mah 25C lipo", two in series)
# Its motors: "a 6V faulhaber motor" (pwm.c's comment), as its maker's own mice: Faulhaber 1717 006 SR (13800 rpm
# free, 3.4 ohm), its gear as theirs, 12:40 (Green Giant V2.2), its encoders 512 lines x4 a motor turn.
GEAR = 40 / 12
MOTOR_R_OHM = 3.4
MOTOR_KE = 6.0 / (13800 * 2 * math.pi / 60)    # V s/rad
MOTOR_FRICTION_NM = MOTOR_KE * 0.012            # its free-running current
MOTOR_ROTOR_INERTIA = 1e-9                      # kg m^2
GEAR_EFFICIENCY = 0.8
ENCODER_COUNTS_PER_REV = 512 * 4 * GEAR         # a wheel turn
TIRE_FRICTION = 0.8


# ---- Its start (ratasim/start.py) --------------------------------------------------------------------------------
def start(cell=(0, 0), heading_deg=0.0):
    """Its template has no start: put down, switched on, it beeps and runs its test loop (main.c)."""
    return [('put_down', cell, heading_deg), ('say', f'put down in cell {cell}, switched on'), ('wait', 10)]
