# ratasim

Micromouse robots in MuJoCo, each driven by its own firmware compiled for the host where there is one:
Ratatron (from the Ratatron simulator) and ZoroBot3 (OPRobots), the mazes, and scenes with several robots.

The robots' firmware comes from their repositories as submodules (`external/`), pinned to a commit:

    git clone --recurse-submodules <this repo>
