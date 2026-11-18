"""ratasim: micromouse robots in MuJoCo, each with its own firmware.

    ratasim [options]               Ratatron in a maze (ratasim --help)
    ratasim batch|curves|failures|mapdiff|demo|ir-info|real-ir ...   Ratatron's studies
    ratasim zorobot [options]       ZoroBot3 in a maze
    ratasim crash|loop|drag-race OUT.mp4 [options]   scenes with both robots (videos)
    ratasim portraits [DIR]         a square picture of each robot
    ratasim help                    this
"""
import runpy
import sys

COMMANDS = {
    'batch': 'ratasim.batch',
    'curves': 'ratasim.curves',
    'failures': 'ratasim.failures',
    'mapdiff': 'ratasim.mapdiff',
    'demo': 'ratasim.demo_monitor',
    'ir-info': 'ratasim.curve_ir_info',
    'real-ir': 'ratasim.real_ir_check',
    'zorobot': 'ratasim.zorobot.run',
    'crash': 'ratasim.scenes.crash',
    'loop': 'ratasim.scenes.loop',
    'drag-race': 'ratasim.scenes.drag_race',
    'portraits': 'ratasim.scenes.portraits',
}


def main(argv=None):
    args = sys.argv[1:] if argv is None else list(argv)
    if args[:1] == ['help']:
        print(__doc__)
        return 0
    if args and args[0] in COMMANDS:
        module, name, args = COMMANDS[args[0]], 'ratasim ' + args[0], args[1:]
    else:
        module, name = 'ratasim.run', 'ratasim'
    sys.argv = [name] + args
    runpy.run_module(module, run_name='__main__')     # sys.argv[0] stays the command's name (argparse prog)
    return 0
