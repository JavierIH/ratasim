"""Futura (Green Ye, UCLA IEEE, micromouseusa.com/?page_id=1342) in the simulator: where its files come from, its
geometry and the physics assumed for it. Robot frame (mm): x right, y forward, z up, origin between its wheels at the
axle's height.

Sources: its kit's firmware template (FuturaProject.zip: user/, user_Libraries/), its schematic (Futura V1.1:
STM32F405, LY3200ALH gyro, four SFH4545 and TEFT4300 IR pairs, TC4427 and ZXMHC3F381N8 H-bridges, HCMS-2903
display), its board's Gerber files (Futura_Gerber.zip, V1.1: its outline, its parts' pads, its holes), its encoder
board's schematic (an AS5304/AS5306 magnetic encoder), its 3D printing files (futura-3D-printing-model.zip: wheel,
skate) and the photo of the kit on its page.
Its page publishes no licence: nothing of it is in this repository; its three zips are downloaded at build time
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
    'Futura_Gerber.zip': (UPLOADS + '2014/08/Futura_Gerber.zip',
                          'e17ace6c36edef936bd5bcc3a46c00e15087c37e28a7f58de2c0fa2310c378ff'),
}
PROJECT = os.path.join(BUILD, 'FuturaProject')  # its firmware (FuturaProject.zip: user/, user_Libraries/)
PRINTS = os.path.join(BUILD, 'prints')          # its printed parts (STL)
GERBERS = os.path.join(BUILD, 'gerbers')        # its board (Futura_Gerber.zip: V1.1.GTL, .GTO, .GML, .TXT...)


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
    """Its zips in BUILD (once; FUTURA_DOWNLOADS: a directory where they already are), checked, unzipped: its
    firmware's sources to PROJECT, its STL files to PRINTS, its Gerber files to GERBERS."""
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
    if not os.path.exists(os.path.join(GERBERS, 'V1.1.GML')):
        with zipfile.ZipFile(os.path.join(BUILD, 'Futura_Gerber.zip')) as z:
            z.extractall(GERBERS, [n for n in z.namelist() if n.startswith('V1.1.') and '/' not in n])


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

# ---- From its Gerber files (V1.1) -------------------------------------------------------------------------------
# Its board: 74 x 100 mm, its nose an arc, a notch 9 x 32 mm each side for a wheel, its back corners cut at 45 deg.
# Its Gerbers are in inches; the robot's origin there: the middle of its width, the middle of its wheels' notches.
GERBER_ORIGIN_IN = (8.8532, 5.169450)
BOARD_BOTTOM_MM = -14.0                         # assumed: 3 mm off the floor (its skate's height)
BOARD_THICKNESS_MM = 1.6                        # assumed: its Gerbers do not say
BOARD_TOP_MM = BOARD_BOTTOM_MM + BOARD_THICKNESS_MM
NOTCH_X_MM = 28.0                               # the notches' inner edge (the board: +-37 mm)
NOTCH_HALF_MM = 16.0                            # the notches: 32 mm long, centred on the axle
# Its encoder boards stand up against the notches' inner edges (1.6 mm), its wheels outside them: their centres.
TRACK_MM = 2 * (NOTCH_X_MM + 1.6 + 0.4 + WHEEL_WIDTH_MM / 2)
MOTOR_MOUNT_HOLES = [(sx * 14.85, sy * 9.0) for sx in (-1, 1) for sy in (-1, 1)]   # its four 2.3 mm holes (M2)
# Its IR pairs, each an emitter (SFH4545, 5 mm) and a phototransistor (TEFT4300, 3 mm) lying on the board, their
# leads bent down into two holes (V1.1.TXT, mm in the robot's frame): each one looks square to the line of its two
# holes, ahead (their silkscreen outlines). LF and RF at the board's edges, 7 deg out; DL and DR by its middle,
# 60 deg out to the sides.
IR_LEADS = {      # name: (emitter's holes, phototransistor's holes)
    'LF': (((-31.77, 30.99), (-29.25, 31.30)), ((-26.54, 41.51), (-24.02, 41.82))),
    'DL': (((-3.70, 54.15), (-2.43, 56.35)), ((-14.20, 50.65), (-12.93, 52.86))),
}
IR_LEADS['RF'] = tuple(tuple((-x, y) for x, y in reversed(h)) for h in IR_LEADS['LF'])
IR_LEADS['DR'] = tuple(tuple((-x, y) for x, y in reversed(h)) for h in IR_LEADS['DL'])
EMITTER_LEN_MM, RECEIVER_LEN_MM = 10.0, 6.5     # the leads' holes to the lens's tip (their silkscreen outlines)
IR_Z_MM = BOARD_TOP_MM + 2.5                    # their axes: a 5 mm emitter lying on the board


def ir_axis(holes):
    """The axis (deg from +x) of a part lying on the board, looking square to the line of its two holes, ahead."""
    (x0, y0), (x1, y1) = holes
    deg = math.degrees(math.atan2(x1 - x0, -(y1 - y0)))     # a normal of the line of its holes
    return deg % 360 if math.sin(math.radians(deg)) > 0 else (deg + 180) % 360


def lens_tip(holes, length):
    (x0, y0), (x1, y1) = holes
    a = math.radians(ir_axis(holes))
    return ((x0 + x1) / 2 + length * math.cos(a), (y0 + y1) / 2 + length * math.sin(a))


def _ir_sensor(name):
    """Each IR pair as the simulator sees it: the middle of its emitter's and phototransistor's lens tips, looking
    along its emitter."""
    em, rx = IR_LEADS[name]
    (ex, ey), (rx_, ry) = lens_tip(em, EMITTER_LEN_MM), lens_tip(rx, RECEIVER_LEN_MM)
    return (round((ex + rx_) / 2, 2), round((ey + ry) / 2, 2)), round(ir_axis(em), 1)


IR_SENSORS = {n: _ir_sensor(n) for n in ('LF', 'RF', 'DL', 'DR')}
SKATE_Y_MM = 45.0                               # assumed (no hole for it): under its nose, its 3 mm skate
GROUND_Z_MM = -WHEEL_RADIUS_MM

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
# free, 3.4 ohm), its gear as theirs, 12:40 (Green Giant V2.2). Its encoders counted as 512 lines x4 a motor turn:
# its encoder boards (encoder-V1.1.pdf) read a magnetic ring by the wheel (AS5304/AS5306), a ring its page does not
# name.
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
