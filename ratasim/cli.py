"""ratasim: micromouse robots in MuJoCo, each with its own firmware.

    ratasim [ratatron] [options]    Ratatron in a maze, started as at a competition (ratasim --help)
    ratasim batch|curves|failures|mapdiff|demo|ir-info|real-ir ...   Ratatron's studies
    ratasim zorobot [options]       ZoroBot3 in a maze
    ratasim gamusino [options]      Gamusino in a maze
    ratasim nezumibot [options]     NezumiBot in a maze
    ratasim perenquen [options]     Perenquen in a maze
    ratasim bulebule [options]      Bulebule in a maze
    ratasim fujin [options]         Fujin in a maze
    ratasim futura [options]        Futura in a maze
    ratasim ukmarsbot [options]     UKMARSBOT in a maze
    ratasim miniskybot [options]    Miniskybot 2 in a maze (ratasim's own program)
    ratasim maze-race|drag-race OUT.mp4 [--robots a,b,c]   scenes with any robots (videos)
    ratasim minisumo OUT.mp4 [--robots a,b,c]   a minisumo bout, all at once on one dohyo, then its podium (a video)
    ratasim crash|loop OUT.mp4 [options]   scenes with Ratatron and ZoroBot3 (videos)
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
    'ratatron': 'ratasim.run',
    'fujin': 'ratasim.fujin.run',
    'gamusino': 'ratasim.gamusino.run',
    'nezumibot': 'ratasim.nezumibot.run',
    'perenquen': 'ratasim.perenquen.run',
    'bulebule': 'ratasim.bulebule.run',
    'futura': 'ratasim.futura.run',
    'ukmarsbot': 'ratasim.ukmarsbot.run',
    'miniskybot': 'ratasim.miniskybot.run',
    'zorobot': 'ratasim.zorobot.run',
    'crash': 'ratasim.scenes.crash',
    'loop': 'ratasim.scenes.loop',
    'drag-race': 'ratasim.scenes.drag_race',
    'maze-race': 'ratasim.scenes.maze_race',
    'minisumo': 'ratasim.scenes.minisumo',
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
