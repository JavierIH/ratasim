# ratasim

![Ratatron and ZoroBot3 at the start line](docs/images/start.jpg)

Micromouse robots in [MuJoCo](https://mujoco.org), each driven by **its own firmware**, compiled for the host and
unchanged, on a simulated chip (motors, encoders, IR sensors, gyro, flash, serial port). Mazes by the rules (16x16
cells of 180 mm), and scenes with several robots.

## Supported robots

| [<img src="docs/images/robots/ratatron.jpg" width="260"><br>**Ratatron**](docs/robots.md#ratatron) | [<img src="docs/images/robots/zorobot3.jpg" width="260"><br>**ZoroBot3**](docs/robots.md#zorobot3) | <img src="docs/images/robots/gamusino.jpg" width="260"><br>**Gamusino** |
|:-:|:-:|:-:|
| STM32F103 · 4 Sharp IR · no gyro | STM32F405 · 4 IR · gyro · suction fan | ESP32 · 4 IR · gyro · suction fan<br>*coming soon* |

Each robot's firmware comes from its repository, a submodule of `external/` pinned to a commit
([robots.md](docs/robots.md)).

## Getting started

Linux (the firmware runs in a `ucontext` coroutine; on Windows, WSL2), Python 3.10+, a C compiler and `make`, and
`ffmpeg` for videos:

```sh
sudo apt install git python3-venv build-essential ffmpeg
git clone --recurse-submodules https://github.com/JavierIH/ratasim
cd ratasim
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
```

### Ratatron in a maze

```sh
ratasim --maze oshwdem2026
```

A viewer opens with the maze and the robot, and the terminal becomes the robot's console, as over its Bluetooth link.
Its firmware is compiled on the first run and boots by itself. To make it move, type in the terminal:

```text
MODE 1          the search: from the start to the goal and back, mapping the maze
START
```

When it is back, `MODE 2 6` and `START` run the race (2.6, the fastest preset) on the map it saved. In the viewer,
Page Up is its SELECT button, Page Down its START and End puts it back at the start.

Without a screen (over SSH), headless, as fast as possible, with a video:

```sh
ratasim --headless --speed 0 --maze oshwdem2026 --cmd "MODE 1" --cmd START --until-idle --record search.mp4
```

![Ratatron searching the OSHWDEM 2026 maze](docs/images/maze-ratatron.jpg)

The map, the telemetry and the console come from Ratatron's own tools, in the submodule:

```sh
pip install -e external/ratatron/software
ratatron connect --sim          # in another terminal while ratasim runs
ratatron monitor                # and another
```

More: [docs/simulator.md](docs/simulator.md) (what is simulated, what was matched to the robot, the studies:
`ratasim batch`, `curves`, `failures`, `mapdiff`, `ir-info`, `real-ir`, `demo`).

### ZoroBot3 in a maze

Its 3D model is its Onshape assembly, not part of its repository: in Onshape, right-click the "Full Assembly" tab,
Export, STEP as a single file, and save it as `ratasim/zorobot/cad/Full_Assembly.step`. Then:

```sh
pip install -e .[cad]           # gmsh, to make its meshes once
ratasim zorobot --maze oshwdem2026 --record zorobot.mp4
```

It boots its firmware, picks the run in its menu with its buttons and starts it with a hand in front of its right
front sensor, as on the real robot (`--run`: explore, then the speed run).

![ZoroBot3 exploring the OSHWDEM 2026 maze](docs/images/maze-zorobot3.jpg)

### Scenes with both robots

No firmware here: each robot follows its own speed profile (Ratatron's race preset, ZoroBot3's `SPEED_HAKI`) and
keeps its lane.

```sh
ratasim drag-race race.mp4          # 3 m: start lights, lanes, a checkered finish
ratasim crash crash.mp4 --ramp      # head-on on the maze's longest straight, over a ramp
ratasim loop loop.mp4               # a loop-the-loop beside the maze
ratasim portraits                   # the pictures of the robots above
```

| ![Drag race](docs/images/drag-race.jpg) | ![Head-on on a ramp](docs/images/crash-ramp.jpg) | ![Loop-the-loop](docs/images/loop.jpg) |
|:-:|:-:|:-:|

## Layout

```text
ratasim/            the simulator: the maze, the physics, Ratatron (its model, sensors, simulated chip in fw/)
ratasim/zorobot/    ZoroBot3: its model from its STEP, its simulated chip (fw/), its runner
ratasim/scenes/     several robots: drag race, crash, loop, portraits
ratasim/mazes/      competition mazes (micromouseonline text format)
external/           the robots' repositories (submodules): ratatron, ZoroBot3
docs/               simulator.md (Ratatron in detail), robots.md
```

## Licences

ZoroBot3 is by [OPRobots](https://github.com/OPRobots/ZoroBot3) (GPLv3): its repository is a submodule and nothing of it
is copied here; its STEP and the meshes made from it stay out of git.
