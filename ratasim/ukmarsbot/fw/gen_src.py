"""mazerunner-core's sketch copied for the build, changed in two places only:

1. config.h, its event: `#define EVENT EVENT_UK` (its README's way to pick one: its config.h edited), for a 16x16
   contest maze: the goal at (7, 7) and the sensor calibration its robots took to UK contests. As committed, it is
   EVENT_HOME: Peter's practice maze, the goal at (1, 0).

2. motors.h, pwm_compensated(), written as an AVR computes it:

       int pwm = MOTOR_MAX_PWM * desired_voltage / battery_voltage;

   At boot it runs before the first battery reading (motors.begin() calls stop(): 0 V over 0 V, NaN). avr-gcc
   converts a float to its 16-bit int through a 32-bit long (__fixsfsi: 0x80000000 for NaN) keeping the low 16 bits:
   0, the motors off. On x86-64 the conversion gives INT_MIN, -255 once constrained: both motors at full reverse
   until its first run. The copy converts through a long and keeps 16 bits, as the AVR.

    python3 gen_src.py SRC_DIR OUT_DIR
"""
import os
import shutil
import sys

CHANGES = {
    'config.h': [('#define EVENT EVENT_HOME', '#define EVENT EVENT_UK')],
    'motors.h': [('int pwm = MOTOR_MAX_PWM * desired_voltage / battery_voltage;',
                  'int pwm = (int16_t)(long)(MOTOR_MAX_PWM * desired_voltage / battery_voltage);')],
}


def main(src, out):
    os.makedirs(out, exist_ok=True)
    for name in os.listdir(src):
        if name.endswith(('.h', '.ino')):
            shutil.copyfile(os.path.join(src, name), os.path.join(out, name))
    for name, subs in CHANGES.items():
        path = os.path.join(out, name)
        text = open(path, encoding='utf-8').read()
        for old, new in subs:
            if text.count(old) != 1:
                sys.exit(f'{name}: not one "{old}"')
            text = text.replace(old, new)
        open(path, 'w', encoding='utf-8').write(text)


if __name__ == '__main__':
    main(*sys.argv[1:3])
