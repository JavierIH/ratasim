"""Where things are: the package runs from this repository (an editable install, `pip install -e .`)."""
import os

PACKAGE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(PACKAGE)
EXTERNAL = os.path.join(REPO, 'external')              # the robots' repositories (submodules)
RATATRON = os.path.join(EXTERNAL, 'ratatron')          # its firmware, its CAD, its tools (ratatron monitor)
FIRMWARE = os.path.join(RATATRON, 'firmware')
ZOROBOT3 = os.path.join(EXTERNAL, 'ZoroBot3')          # its firmware
SIM = PACKAGE
SIM_BUILD = os.path.join(SIM, 'build')                 # flash images, the simulator's serial port (not committed)
RUNTIME = os.path.join(os.environ.get('XDG_RUNTIME_DIR') or '/tmp', 'ratatron')   # shared with `ratatron connect --sim`
SIM_TTY = os.path.join(RUNTIME, 'sim-tty')             # the simulator's serial port (a link to its pseudo-terminal)


# What each command needs from the submodules (paths from the repository; one inside a submodule is initialised from
# its parent). Commands not listed need none. Never `--recursive`: perenquen's repository has a broken gitlink.
_RATATRON = ['external/ratatron']
_ALL = ['external/ratatron', 'external/ZoroBot3', 'external/gamusino', 'external/NezumiBot', 'external/bulebule',
        'external/bulebule/src/mmlib', 'external/bulebule/src/printf', 'external/perenquen',
        'external/aversive4dspic', 'external/fujin', 'external/mazerunner-core', 'external/ukmarsbot',
        'external/Miniskybot']
NEEDS = {
    'ratatron': _RATATRON, 'batch': _RATATRON, 'curves': _RATATRON, 'failures': _RATATRON, 'mapdiff': _RATATRON,
    'demo': _RATATRON, 'ir-info': _RATATRON, 'real-ir': _RATATRON,
    'zorobot': ['external/ZoroBot3'], 'crash': _RATATRON + ['external/ZoroBot3'],
    'loop': _RATATRON + ['external/ZoroBot3'],
    'gamusino': ['external/gamusino'], 'nezumibot': ['external/NezumiBot'],
    'bulebule': ['external/bulebule', 'external/bulebule/src/mmlib', 'external/bulebule/src/printf'],
    'perenquen': ['external/perenquen', 'external/aversive4dspic'], 'fujin': ['external/fujin'],
    'ukmarsbot': ['external/ukmarsbot', 'external/mazerunner-core'], 'miniskybot': ['external/Miniskybot'],
    'drag-race': _ALL, 'maze-race': _ALL, 'minisumo': _ALL, 'portraits': _ALL,
}


def _empty(path):
    full = os.path.join(REPO, path)
    return not os.path.isdir(full) or not os.listdir(full)


def require(command):
    """Makes sure the submodules `command` needs are there. If one is empty, asks (y/N) to download it with
    `git submodule update --init <it>`; without a terminal, or on no, returns the message to exit with. None if all
    is well (or after downloading)."""
    import subprocess
    import sys
    for path in NEEDS.get(command, []):
        if not _empty(path):
            continue
        # a path inside a submodule is initialised from that submodule (bulebule/src/mmlib from bulebule)
        top = '/'.join(path.split('/')[:2])
        base, rel = (top, path[len(top) + 1:]) if path != top else ('.', path)
        cmd = ['git', '-C', os.path.join(REPO, base), 'submodule', 'update', '--init', rel]
        manual = 'git submodule update --init ' + path if base == '.' else \
            'git -C %s submodule update --init %s' % (base, rel)
        if base != '.' and _empty(base):
            continue                              # its parent comes first (earlier in the list)
        if not (sys.stdin.isatty() and sys.stdout.isatty()):
            return '%s is empty. Download it with: %s' % (path, manual)
        try:
            answer = input('%s is empty. Download it now? [y/N] ' % path).strip().lower()
        except EOFError:
            answer = ''
        if answer not in ('y', 'yes'):
            return '%s is empty. Download it with: %s' % (path, manual)
        try:
            ok = subprocess.run(cmd).returncode == 0
        except OSError as e:
            return 'could not run git (%s). Download it with: %s' % (e, manual)
        if not ok or _empty(path):
            return 'git could not download %s. Try by hand: %s' % (path, manual)
    return None
