"""Starting a robot as its makers do, the same way for every robot: its config's start sequence, an operator's
actions on the simulated robot with its firmware running between them.

    ('say', text)               what is being done (the console, the video's status line)
    ('wait', ms)                its firmware runs
    ('press', button[, ms])     a button held (100 ms)
    ('hand', sensors[, ms])     a hand in front of these sensors (300 ms; ratasim/hand.py)
    ('lift',)                   held in the air over where it stands (its sensors see over the walls)
    ('put_down', cell, deg)     put down at a cell's centre, facing deg (0 north, positive to the left)
"""


def play(robot, steps, run, say=print):
    for kind, *args in steps:
        if kind == 'say':
            say(*args)
        elif kind == 'wait':
            run(*args)
        elif kind == 'press':
            robot.press(*args)
        elif kind == 'hand':
            robot.cover(*args)
        elif kind == 'lift':
            robot.lift()
        elif kind == 'put_down':
            robot.put_down(*args)
        else:
            raise ValueError(f'unknown start step {kind!r}')
