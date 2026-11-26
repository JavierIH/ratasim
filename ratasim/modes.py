"""The ways each robot can be launched: `ratasim ROBOT --help` prints them first, before its options.

MODES[robot]: (command, what it runs), one line each, as its run.py takes them.
"""

MODES = {
    'ratatron': [
        ('ratasim ratatron', 'mode 1: the search, to the goal and back, mapping (saved in its flash)'),
        ('ratasim ratatron --race 1', 'race 2.1: left wall follower'),
        ('ratasim ratatron --race 2', 'race 2.2: right wall follower'),
        ('ratasim ratatron --race 3', 'race 2.3: on the saved map, no curves'),
        ('ratasim ratatron --race 4', 'race 2.4: on the saved map, FAST 800 CURVE 300'),
        ('ratasim ratatron --race 5', 'race 2.5: on the saved map, FAST 900 CURVE 400'),
        ('ratasim ratatron --race 6', 'race 2.6: on the saved map, FAST 900 CURVE 480'),
        ('ratasim ratatron --erase-flash', 'the search with blank flash (no saved map)'),
        ('ratasim ratatron --console', 'not started: only its console (MODE, START, :select, :start ...)'),
    ],
    'zorobot': [
        ('ratasim zorobot', 'exploration only (complete, time floodfill)'),
        ('ratasim zorobot --run', 'EXPLORE & RUN: exploration, then the speed run (speed 5, haki)'),
        ('ratasim zorobot --run --speed N', 'speed run at N: 0 explore, 1 normal, 2 medium, 3 fast, 4 super, 5 haki'),
        ('  --explore-type N', '0 to the goal, 1 and back home, 2 complete (default), 3 endless'),
        ('  --floodfill N', '0 basic, 1 diagonal, 2 time (default)'),
        ('  --solve N', 'speed run: 1 diagonals (default), 0 standard (90 and 180 deg turns)'),
    ],
    'gamusino': [
        ('ratasim gamusino', 'its menu\'s option 6: 1.62 m ahead at 1.6 m/s, a brake, a 180 deg turn on the spot'),
        ('ratasim gamusino --option 0', 'its encoders\' test (held up: its LEDs as each wheel turns)'),
        ('ratasim gamusino --option 1', 'its gyro\'s test: four 90 deg turns on the spot'),
        ('ratasim gamusino --option 2', 'its motors\' test: each 1 s forward, 1 s back'),
        ('ratasim gamusino --option 3', 'its gyro measured'),
        ('ratasim gamusino --option 4', 'its suction\'s test: 5 s at 60/255'),
        ('ratasim gamusino --option 5', 'its wall sensors\' test: an LED for each one seeing a wall close'),
    ],
    'nezumibot': [
        ('ratasim nezumibot', 'left wall follower'),
        ('ratasim nezumibot --hand right', 'right wall follower'),
        ('ratasim nezumibot --party', '"party": swaps hands every 30 s after 45 s'),
    ],
    'perenquen': [
        ('ratasim perenquen', 'right wall follower, gain 30, its boot speed (400: 0.78 m/s)'),
        ('ratasim perenquen --side left', 'left wall follower'),
        ('ratasim perenquen --speed 1000 --gain 100', 'faster: 1000 = 1.96 m/s (its notes: 1000/100, 1500/200)'),
        ('ratasim perenquen --console', 'then its command line (start, traj_speed ...)'),
    ],
    'bulebule': [
        ('ratasim bulebule', 'exploration: to the centre and back, the path saved in its flash'),
        ('ratasim bulebule --run', 'a run on the saved path, there and back (up to 2 m/s)'),
        ('ratasim bulebule --force-steps N', 'its force menu: 0.1 N + 0.05 a press (3: 0.25 N, its boot\'s); '
                                             'exploring speed follows'),
        ('ratasim bulebule --erase-flash', 'exploration, the saved path forgotten first'),
        ('ratasim bulebule --console', 'then its command line (battery, move ..., set ...)'),
    ],
    'fujin': [
        ('ratasim fujin', 'search, slow: 0.3 m/s, no fan; to the goal and back'),
        ('ratasim fujin --mode medium', 'search, medium speeds, fan on'),
        ('ratasim fujin --mode fast', 'search, fast speeds, fan on'),
    ],
    'futura': [
        ('ratasim futura', 'its only program: cell by cell at 0.5 m/s, pivots at walls'),
    ],
    'ukmarsbot': [
        ('ratasim ukmarsbot', 'function 2: its search, to the goal and back, mapping'),
        ('ratasim ukmarsbot --function 3', 'function 3: left wall follower to the goal'),
        ('ratasim ukmarsbot --console', 'then its serial command line (? for help)'),
    ],
    'miniskybot': [
        ('ratasim miniskybot', 'its only program: straight on to a wall, turns by time to an open way'),
    ],
    'kerise': [
        ('ratasim kerise', 'its MazeSolver at 600 mm/s: search to the goal and back, then its fast runs'),
    ],
    'ratatron2': [
        ('ratasim ratatron2', 'its model only (no firmware yet): in the start cell, motors off'),
        ('ratasim ratatron2 --leds 0b111111', 'its tail LEDs lit as a mask'),
        ('ratasim ratatron2 --image out.jpg', 'a picture of it, no window'),
    ],
    'koguma': [
        ('ratasim koguma --number 1', 'slalom search, 500 mm/s, no fan, to the goal and back (default)'),
        ('ratasim koguma --number 2', 'slalom search, back with its pseudo full search'),
        ('ratasim koguma --number 3', 'slalom search, to the goal only'),
        ('ratasim koguma --number 4', 'fan search, 1000 mm/s, to the goal and back'),
        ('ratasim koguma --number 5', 'fan search, back with its pseudo full search'),
        ('ratasim koguma --number 6', 'fan search, then the unknown walls of the fastest route (Dijkstra)'),
        ('ratasim koguma --number 7', 'as 6, known stretches faster'),
        ('ratasim koguma --number 8', 'slalom search, then its fastest run (Dijkstra, 2 m/s turns, 6 m/s straights)'),
        ('ratasim koguma --number 9', 'fan search, then its fastest run'),
        ('ratasim koguma --number 10', 'its contest run: as 7, then its fastest run'),
        ('ratasim koguma --number 11', 'as 10, its fastest run 4 times, back to the start after each'),
        ('ratasim koguma --number 12', 'as 10, known stretches on diagonals (a toy of its maker\'s)'),
    ],
}


def text(robot):
    """The modes block of `robot`, or '' if it has none listed."""
    rows = MODES.get(robot)
    if not rows:
        return ''
    width = max(len(c) for c, _ in rows) + 3
    lines = [f'{robot}: its modes (with --maze NAME, --headless, --record OUT.mp4 ... below)', '']
    lines += ['    ' + c.ljust(width) + what for c, what in rows]
    return '\n'.join(lines) + '\n'
