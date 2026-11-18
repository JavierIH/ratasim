"""Bulebule (Bulebots, github.com/Bulebots/bulebule) in the simulator: its geometry and the physics assumed for it.
Robot frame (mm): x right, y forward, z up, origin on the floor under the motors' axis (between its axles).

Sources: its firmware (src/setup.h, src/config.h), its CadQuery parts (3d/mount.py, 3d/rim.py), its board's layout
(a screenshot of its Upverter design in Bulebots' 2018 talk, figures/pcb.jpg: the outline fitted on it, pcb.py)
and its photos (docs/source/figures). The board's design files are gone with Upverter.
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, 'build')

# ---- From its firmware ----------------------------------------------------------------------------------------
MASS_KG = 0.110                     # MOUSE_MASS
INERTIA_Z = 0.000125                # MOUSE_MOMENT_OF_INERTIA, kg m^2
WHEELS_SEPARATION = 65.0            # MOUSE_WHEELS_SEPARATION: left to right
AXIS_SEPARATION = 22.3              # MOUSE_AXIS_SEPARATION: its front and rear axles, each side
HEAD, TAIL = 57.0, 37.0             # MOUSE_MOTOR_HEAD, MOUSE_MOTOR_TAIL: from the motors' axis
MICROMETERS_PER_COUNT = 8.32        # config.h; IE2-512 encoders (2048 counts a turn), gears 15:60
GEAR = 60 / 15
WHEEL_RADIUS = 2048 * GEAR * MICROMETERS_PER_COUNT / 1000 / (2 * math.pi)     # 10.85

# ---- From its CadQuery parts ------------------------------------------------------------------------------------
MODULE, Z_PINION, Z_GEAR = 0.3, 15, 60
MOTOR_D = 15.0                      # Faulhaber, with IE2-512 encoders
MOTOR_SHIFT = 1.5                   # the motor's axis over the wheels' axles
MOUNT = (6.0, 40.0, 15.0)           # x, y, z: on the board's waist edges, the motor through its middle
RIM_D, RIM_W, HUB_D, HUB_W = 19.0, 5.0, 14.8, 3.0
CONTACT_W = 6.0                     # the tires' collision width, centred on the firmware's wheel separation
BEARING_D = 6.2
RIM_HOLES, RIM_HOLE_D = 9, 2.5      # MINIHOLES: its face's holes, on the circle between hub and bearing
RIM_HOLE_R = (HUB_D + BEARING_D) / 4
RIM_RECESS = 1.0                    # the rim's face inside the tire (photos)
BOLT_HEAD = (5.5, 2.0)              # the wheels' M3 hex-head bolts (photos): across flats, height
WASHER_D = 7.0

# ---- From its board's layout (pcb.py) ----------------------------------------------------------------------------
PCB_THICK = 1.6
WAIST = 28.1                        # the board's waist, where the mounts stand: half width
SHOULDER_X, TAB_X = 41.8, 40.0      # the board's shoulders and rear tabs (pcb.py): half width
TIRE_X = WAIST + 1.4                # the tires' inner side, past the gears
TIRE_W = TAB_X - TIRE_X             # 10.5: the tires flush with the board's edge by them (photos)
AXLE_Z = WHEEL_RADIUS               # the wheels' axles over the floor
MOTOR_Z = AXLE_Z + MOTOR_SHIFT      # its motors' axis
PCB_TOP = MOTOR_Z - MOUNT[2] / 2    # the mounts stand on the board
PCB_BOTTOM = PCB_TOP - PCB_THICK    # 3.25 mm over the floor

# ---- Assumed (nothing measured) ---------------------------------------------------------------------------------
TIRE_FRICTION = 1.0
BATTERY = (38.0, 30.0, 8.0)         # a 2S LiPo on its motors (photos)
