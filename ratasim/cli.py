"""ratasim: micromouse robots in MuJoCo, each with its own firmware.

    ratasim ratatron [options]      Ratatron in a maze
    ratasim zorobot [options]       ZoroBot3 in a maze
    ratasim gamusino [options]      Gamusino in a maze
    ratasim nezumibot [options]     NezumiBot in a maze
    ratasim perenquen [options]     Perenquen in a maze
    ratasim bulebule [options]      Bulebule in a maze
    ratasim fujin [options]         Fujin in a maze
    ratasim futura [options]        Futura in a maze
    ratasim ukmarsbot [options]     UKMARSBOT in a maze
    ratasim miniskybot [options]    Miniskybot 2 in a maze
    ratasim kerise [options]        KERISE v1 in a classic maze
    ratasim ratatron2 [options]     Ratatron II in a maze: its model only (its firmware not yet)
    ratasim koguma [options]        KOGUMA-CHAN Mk3 in a classic maze
    ratasim maze-race|drag-race OUT.mp4 [--robots a,b,c]   scenes with any robots (videos)
    ratasim minisumo OUT.mp4 [--robots a,b,c]   a minisumo bout, all at once on one dohyo, then its podium (a video)
    ratasim crash|loop OUT.mp4 [options]   scenes with Ratatron and ZoroBot3 (videos)
    ratasim portraits [DIR]         a square picture of each robot
    ratasim mazes [PATTERN] [--show NAME]   the mazes --maze takes by name (525 classic competition ones)
    ratasim help                    this

ratasim ROBOT --help: its options.
"""
import runpy
import sys

from ratasim import paths

COMMANDS = {     # batch ... real-ir: Ratatron's studies, its maker's tools (docs/simulator.md), not listed above
    'batch': 'ratasim.batch',
    'curves': 'ratasim.curves',
    'failures': 'ratasim.failures',
    'mapdiff': 'ratasim.mapdiff',
    'demo': 'ratasim.demo_monitor',
    'ir-info': 'ratasim.curve_ir_info',
    'real-ir': 'ratasim.real_ir_check',
    'ratatron': 'ratasim.run',
    'fujin': 'ratasim.fujin.run',
    'gamusino': 'ratasim.gamusino.run',
    'nezumibot': 'ratasim.nezumibot.run',
    'perenquen': 'ratasim.perenquen.run',
    'bulebule': 'ratasim.bulebule.run',
    'futura': 'ratasim.futura.run',
    'ukmarsbot': 'ratasim.ukmarsbot.run',
    'miniskybot': 'ratasim.miniskybot.run',
    'kerise': 'ratasim.kerise.run',
    'ratatron2': 'ratasim.ratatron2.run',
    'koguma': 'ratasim.koguma.run',
    'zorobot': 'ratasim.zorobot.run',
    'crash': 'ratasim.scenes.crash',
    'loop': 'ratasim.scenes.loop',
    'drag-race': 'ratasim.scenes.drag_race',
    'maze-race': 'ratasim.scenes.maze_race',
    'minisumo': 'ratasim.scenes.minisumo',
    'portraits': 'ratasim.scenes.portraits',
    'mazes': 'ratasim.mazes',
}


def main(argv=None):
    args = sys.argv[1:] if argv is None else list(argv)
    if not args or args[0] in ('help', '-h', '--help'):
        print(__doc__)
        return 0
    if args[0] not in COMMANDS:
        print(f'ratasim: no command {args[0]!r} (ratasim help: the robots and scenes)', file=sys.stderr)
        return 2
    args_command = args[0]
    module, name, args = COMMANDS[args[0]], 'ratasim ' + args[0], args[1:]
    message = paths.require(args_command)
    if message:
        print('ratasim: ' + message, file=sys.stderr)
        return 1
    sys.argv = [name] + args
    runpy.run_module(module, run_name='__main__')     # sys.argv[0] stays the command's name (argparse prog)
    return 0
