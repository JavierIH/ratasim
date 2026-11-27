# Ratasim

[![CI](https://github.com/JavierIH/ratasim/actions/workflows/ci.yml/badge.svg)](https://github.com/JavierIH/ratasim/actions/workflows/ci.yml)

![Ratatron, Gamusino and ZoroBot3 in the OSHWDEM 2026 maze](docs/images/maze.jpg)

Micromouse robots in [MuJoCo](https://mujoco.org), each driven by **its own firmware**, compiled for the host and
unchanged, on a simulated chip (motors, encoders, IR sensors, gyro, flash, serial port), in mazes by the rules (16x16
cells of 180 mm).

## Robots

<table>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#ratatron"><img src="docs/images/robots/ratatron.jpg" width="250"><br><b>Ratatron</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#zorobot3"><img src="docs/images/robots/zorobot3.jpg" width="250"><br><b>ZoroBot3</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#koguma-chan-mk3"><img src="docs/images/robots/koguma.jpg" width="250"><br><b>KOGUMA-CHAN Mk3</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://github.com/JavierIH">JavierIH</a> · <a href="https://github.com/JavierIH/ratatron">repo</a></td>
<td align="center">by <a href="https://oprobots.org">OPRobots</a> · <a href="https://github.com/OPRobots/ZoroBot3">repo</a></td>
<td align="center">by <a href="https://github.com/Tokoro0917">Tokoro0917</a> · <a href="https://github.com/Tokoro0917/KOGUMA.MkIIl">repo</a></td>
</tr>
<tr>
<td align="center">STM32F103 · 4 Sharp IR · no gyro</td>
<td align="center">STM32F405 · 4 IR · gyro · fan</td>
<td align="center">STM32F446 · 4 IR · gyro · fan</td>
</tr>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#fujin"><img src="docs/images/robots/fujin.jpg" width="250"><br><b>Fujin</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#kerise-v1"><img src="docs/images/robots/kerise.jpg" width="250"><br><b>KERISE v1</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#futura"><img src="docs/images/robots/futura.jpg" width="250"><br><b>Futura</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://raiju.team">Team Raiju</a> · <a href="https://github.com/team-raiju/fujin">repo</a></td>
<td align="center">by <a href="https://github.com/kerikun11">kerikun11</a> · <a href="https://github.com/kerikun11/micromouse-kerise-v1">repo</a></td>
<td align="center">by <a href="https://micromouseusa.com">Green Ye</a> · <a href="https://micromouseusa.com/?page_id=1342">page</a></td>
</tr>
<tr>
<td align="center">STM32G474 · 4 IR · gyro · fan</td>
<td align="center">STM32F405 · 4 IR · gyro</td>
<td align="center">STM32F405 · 4 IR · gyro</td>
</tr>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#ukmarsbot"><img src="docs/images/robots/ukmarsbot.jpg" width="250"><br><b>UKMARSBOT</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#gamusino"><img src="docs/images/robots/gamusino.jpg" width="250"><br><b>Gamusino</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#bulebule"><img src="docs/images/robots/bulebule.jpg" width="250"><br><b>Bulebule</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://ukmars.org">UKMARS</a> · <a href="https://github.com/ukmars/ukmarsbot">repo</a></td>
<td align="center">by <a href="https://github.com/turmandreams">turmand<wbr>reams</a> · <a href="https://github.com/turmandreams/gamusino">repo</a></td>
<td align="center">by <a href="https://github.com/Bulebots">Bulebots</a> · <a href="https://github.com/Bulebots/bulebule">repo</a></td>
</tr>
<tr>
<td align="center">ATmega328P · 4 IR · N20 motors</td>
<td align="center">ESP32 · 4 IR · gyro · fan</td>
<td align="center">STM32F103 · 4 IR · gyro · 4 wheels</td>
</tr>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#nezumibot"><img src="docs/images/robots/nezumibot.jpg" width="250"><br><b>NezumiBot</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#perenquén"><img src="docs/images/robots/perenquen.jpg" width="250"><br><b>Perenquén</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#miniskybot"><img src="docs/images/robots/miniskybot.jpg" width="250"><br><b>Miniskybot</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://oprobots.org">OPRobots</a> · <a href="https://github.com/OPRobots/NezumiBot">repo</a></td>
<td align="center">by <a href="https://github.com/supernudo">supernudo</a> · <a href="https://github.com/supernudo/perenquen">repo</a></td>
<td align="center">by <a href="https://github.com/Obijuan">Obijuan</a> · <a href="https://github.com/Obijuan/Miniskybot">repo</a></td>
</tr>
<tr>
<td align="center">STM32F401 · 4 Sharp IR · N20 motors</td>
<td align="center">dsPIC33<wbr>EP512<wbr>MC806 · 4 IR · gyro · 4 wheels</td>
<td align="center">ATmega328 · 1 ultrasonic · hacked servos</td>
</tr>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#ratatron-ii"><img src="docs/images/robots/ratatron2.jpg" width="250"><br><b>Ratatron II</b></a></td>
<td align="center" width="33%"></td>
<td align="center" width="33%"></td>
</tr>
<tr>
<td align="center">by <a href="https://github.com/JavierIH">JavierIH</a> · <a href="https://github.com/JavierIH/ratatron2">repo</a></td>
<td align="center"></td>
<td align="center"></td>
</tr>
<tr>
<td align="center">STM32F446 · 4 IR · gyro · fan · 4 wheels</td>
<td align="center"></td>
<td align="center"></td>
</tr>
</table>

Each robot's firmware comes from its own repository, a submodule of `external/` pinned to a commit (Futura's, which
has none, from its page at build time, pinned by its hash); its authors are credited in [robots.md](docs/robots.md),
with what was taken from where.

## Getting started

Linux (the firmware runs in a `ucontext` coroutine; on Windows, WSL2), Python 3.10+, a C compiler and `make`, and
`ffmpeg` for videos:

```sh
sudo apt install git python3-venv build-essential ffmpeg
git clone https://github.com/JavierIH/ratasim
cd ratasim
git submodule update --init
python3 -m venv .venv
. .venv/bin/activate
pip install -e .[cad]            # with gmsh, to make some robots' meshes from their 3D models
```

### Mazes

Every robot takes `--maze` with a file in the micromouseonline text format or a name: the five of `ratasim/mazes`
(`oshwdem2026`, the practice and test mazes) or one of the 525 classic 16x16 competition mazes (APEC, All Japan,
the UK and many more) of [micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles), the collection
that [Peter Harrison](https://github.com/micromouseonline) has gathered from many sources over the years, here the
submodule `external/mazefiles`. Only its `classic/` folder is loaded (its half-size and training mazes are not), and
only their text is kept: the maze is built from it when the scene is made.

```sh
ratasim mazes                        # every name; ratasim mazes japan: the ones with "japan"
ratasim mazes --show japan2000       # one drawn
ratasim kerise --maze japan2000
ratasim batch out $(ratasim mazes --paths 'alljapan-*')   # Ratatron over many
ratasim matrix out --robots zorobot,koguma,fujin --mazes oshwdem2026,apec2026   # robots x mazes x their runs
```

`ratasim matrix` runs robots' searches and speed runs (at each of their speeds) in several mazes at once and makes a
table of which reach the goal, in what time, and where they first touch a wall: a change to the simulator that helps
in one maze only is a fit to that maze. Every robot is put down in its start cell as its makers put it down; `--start
wall|centre|MM` puts it elsewhere (its tail against the back wall, its axle on the cell's middle, or its axle MM ahead
of the middle).

### Running Ratatron

<img src="docs/images/maze-ratatron.jpg" alt="Ratatron searching the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim ratatron --maze oshwdem2026
ratasim ratatron --maze oshwdem2026 --race 4
```

A viewer opens and the terminal becomes its console. The firmware is compiled on the first run; the robot searches to
the goal and back, then races on the saved map with `--race`. Page Up / Page Down are its SELECT and START buttons,
End puts it back at the start.

### Running ZoroBot3

<img src="docs/images/maze-zorobot3.jpg" alt="ZoroBot3 exploring the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim zorobot --maze oshwdem2026 --run --record zorobot.mp4 [--camera fixed]
```

Export its Onshape "Full Assembly" as STEP to `ratasim/zorobot/cad/Full_Assembly.step` first. Started with a hand at
its right front sensor; `--run` explores, then speed runs. Its menu as its makers ran it at OSHWDem 2026 (explore
complete, time-based floodfill, diagonals, speed 5): its search reaches the goal in 73.3 s; its speed run reaches it at
speeds 1 and 2 (7.82 and 6.92 s, `--speed 2`), at 3-5 it ends on a post (about 5.5 s on the real robot's video at 5).

### Running KOGUMA-CHAN Mk3

<img src="docs/images/maze-koguma.jpg" alt="KOGUMA-CHAN Mk3 searching the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim koguma --maze oshwdem2026 --record koguma.mp4
```

The All Japan classic champion, Akihiro Suda's, with its contest firmware on a simulated STM32F446 (ST's HAL):
picked by turning its right wheel and started with a hand, as its maker does. Its search reaches the goal in
86.6 s and is back at 128.5 s; its fastest run, on a map of the whole maze, in 6.63 s.

### Running Fujin

<img src="docs/images/maze-fujin.jpg" alt="Fujin searching the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim fujin --maze oshwdem2026 --record fujin.mp4
```

Its RCX 2026 firmware (C++23: g++ 13 or clang++ 15) searches to the goal and back: `--mode slow` (0.3 m/s)
and medium finish (the goal in 104.9 s, 66.5 s); fast not yet. `--run` then makes its speed run on the map it saved
(`--known`: on the maze written in its EEPROM, no search), started from its menu: slow reaches the goal in 21.2 s,
medium in 9.0 s; fast and super not yet.

### Running KERISE v1

<img src="docs/images/maze-kerise.jpg" alt="KERISE v1 searching the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim kerise --maze oshwdem2026 --record kerise.mp4
```

kerikun11's 2016 classic micromouse with its All Japan contest firmware on a simulated STM32F405 (mbed OS 5.1).
It searches to the goal (84.3 s) and back (158 s), then makes its fast runs (41.4 s, 34.6 s). `--echo` prints its
console.

### Running Futura

<img src="docs/images/maze-futura.jpg" alt="Futura in the north rows of the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim futura --maze oshwdem2026 --echo --record futura.mp4
```

UCLA IEEE's kit mouse with its students' controller sample ([robots.md](docs/robots.md#futura)): it drives cell to
cell and pivots at walls, searching nothing. `--echo` prints its serial port.

### Running UKMARSBOT

<img src="docs/images/maze-ukmarsbot.jpg" alt="UKMARSBOT searching the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim ukmarsbot --maze oshwdem2026 --record ukmarsbot.mp4
```

mazerunner-core on a simulated ATmega328P: flood fill to the goal and back. `--function 3` runs its left wall
follower instead.

### Running Gamusino

<img src="docs/images/maze-gamusino.jpg" alt="Gamusino on the maze's longest straight" align="right" width="280">

```sh
ratasim gamusino --maze oshwdem2026 --record gamusino.mp4
```

Its ESP32 sketch, started from its menu with its wheels as its maker does: 1.62 m ahead on the maze's longest
straight, a brake and a 180 deg turn on the spot. `--option N` runs its menu's tests (`ratasim gamusino --help`).

### Running Bulebule

<img src="docs/images/maze-bulebule.jpg" alt="Bulebule exploring the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim bulebule --maze oshwdem2026 --record bulebule.mp4
```

Its firmware on a simulated STM32F103 explores to the centre and back, saving the path in flash; `--run` races from
there (it does not reach the goal yet: [robots.md](docs/robots.md#bulebule)).

### Running NezumiBot

<img src="docs/images/maze-nezumibot.jpg" alt="NezumiBot following the left wall" align="right" width="280">

```sh
ratasim nezumibot --maze oshwdem2026 --hand left --record nezumibot.mp4
```

Its OSHWDEM 2023 wall follower, started with hands at its sensors. `--party` swaps walls every 30 s.

### Running Perenquén

<img src="docs/images/maze-perenquen.jpg" alt="Perenquén keeping to the right wall" align="right" width="280">

```sh
ratasim perenquen --maze oshwdem2026 --side right --gain 30 --record perenquen.mp4
```

A wall follower on a simulated dsPIC, started as its maker does (`start right 30` in its terminal, then a hand).
`--speed 1000` sets 2 m/s.

### Running Miniskybot

<img src="docs/images/maze-miniskybot.jpg" alt="Miniskybot in the OSHWDEM 2026 maze" align="right" width="280">

```sh
ratasim miniskybot --maze oshwdem2026 --echo --record miniskybot.mp4
```

Obijuan's printable robot (servos, ultrasonic ranger, ATmega328): straight to a wall, then turns by time to an open
way. `--echo` prints what it says.

### Running Ratatron II

<img src="docs/images/maze-ratatron2.jpg" alt="Ratatron II in the start cell, its LEDs lit" align="right" width="280">

```sh
ratasim ratatron2 --maze oshwdem2026 --leds 0b111111
```

Ratatron's successor, being built: for now its 3D model only, from its FreeCAD file, in the start cell. Its
firmware is not simulated yet. `--leds` lights its six LEDs on its tail; `--image out.jpg` takes a picture
instead of the viewer.

## Tests and CI

`tests/robots.py` puts each robot in the OSHWDEM 2026 maze and checks what it prints. CI runs the short check, one job
per robot:

```sh
python tests/robots.py [robot ...]           # short: builds, starts, runs and leaves the start cell
python tests/robots.py --full [robot ...]    # full: each robot's whole run (slower, for local use)
python tests/robots.py --maze NAME [robot ...]   # any check in another maze
```

The full check also runs the searches of the robots that reach the goal in a second maze, OSHWDEM 2019's.

## Scenes

Several robots at once, picked by name (`--robots fujin,zorobot,ukmarsbot`, or `all`). The race in the maze replays
each robot's own traced run together in one maze. Minisumo puts them all on a dohyo until one is left, with a simple
control of its own. Drag race, crash and loop use each robot's speed profile, without its firmware.

```sh
ratasim maze-race maze-race.mp4     # Ratatron, ZoroBot3, Bulebule, Fujin, UKMARSBOT: their searches, x4
ratasim drag-race race.mp4          # 3 m, all eleven: start lights, lanes, a checkered finish
ratasim minisumo sumo.mp4           # all eleven on one dohyo, the last one left wins
ratasim minisumo --still podium.jpg --podium    # a picture of the podium after the bout
ratasim crash crash.mp4 --ramp      # Ratatron and ZoroBot3 head-on on the maze's longest straight, over a ramp
ratasim loop loop.mp4               # Ratatron and ZoroBot3 through a loop-the-loop beside the maze
ratasim portraits                   # the pictures above
```

| ![A race in the maze](docs/images/maze-race.jpg) | ![Drag race](docs/images/drag-race.jpg) |
|:-:|:-:|
| ![Minisumo](docs/images/minisumo.jpg) | ![Head-on on a ramp](docs/images/crash-ramp.jpg) |
| ![The podium after the bout](docs/images/minisumo-podium.jpg) | ![Loop-the-loop](docs/images/loop.jpg) |

## Credits and licences

Ratasim is free software under the GNU General Public License v3 or later (`LICENSE`, Copyright (C) 2026 JavierIH); each
robot keeps its authors' licence, as below, and what is a submodule or is downloaded at build time is not relicensed.

Every robot belongs to its authors (above); their repositories are submodules and nothing of them is copied here.
ZoroBot3, Bulebule and NezumiBot are GPLv3; Gamusino's, Fujin's, KERISE v1's and KOGUMA's repositories carry no licence (KERISE's and KOGUMA's authors allow Ratasim to use them); Perenquén's design is CC BY-SA
4.0 (Javier Baliñas), its firmware and its library aversive4dspic (Droids Corporation, Microb Technology, Eirbot,
Eurobotics Engineering) GPLv2+; Gamusino's gyro library, STM32duino LSM6DSR (STMicroelectronics, BSD-3), is
downloaded at build time. Futura's page (Green Ye, micromouseusa.com) gives no licence: its firmware template, its
controller sample, its board's Gerber files and its printed parts are downloaded at build time, pinned by their
SHA-256, and stay out of git; ST's library is not used
(`ratasim/futura/fw` simulates the part of its API the template calls). Fujin's team's photos of its board
(raiju.team), its art's only source, are downloaded at build time, pinned by their SHA-256, and stay out of git.
Miniskybot's repository is GPL or CC BY-SA (its README; its page CC BY-SA 3.0 ES). Ratatron II's firmware is
GPLv3, its board and mechanics CC BY-SA 4.0. The competition mazes are Peter Harrison's collection ([micromouseonline/mazefiles](https://github.com/micromouseonline/mazefiles)), gathered with its contributors from the contests' own maps; it carries no licence, so it is a submodule and none of its files is copied here. The meshes made from ZoroBot3's, Gamusino's, Bulebule's, Fujin's, Miniskybot's, KERISE v1's and KOGUMA's CAD stay out of git; NezumiBot's, Perenquén's and Ratatron II's are in `ratasim/nezumibot/assets`, `ratasim/perenquen/assets` and
`ratasim/ratatron2/assets`, under their licences.
