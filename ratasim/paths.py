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
