# ratasim

[![CI](https://github.com/JavierIH/ratasim/actions/workflows/ci.yml/badge.svg)](https://github.com/JavierIH/ratasim/actions/workflows/ci.yml)

![Gamusino, ZoroBot3 and Ratatron in the OSHWDEM 2026 maze](docs/images/maze.jpg)

Micromouse robots in [MuJoCo](https://mujoco.org), each driven by **its own firmware**, compiled for the host and
unchanged, on a simulated chip (motors, encoders, IR sensors, gyro, flash, serial port), in mazes by the rules (16x16
cells of 180 mm).

## Robots

<table>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#ratatron"><img src="docs/images/robots/ratatron.jpg" width="250"><br><b>Ratatron</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#zorobot3"><img src="docs/images/robots/zorobot3.jpg" width="250"><br><b>ZoroBot3</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#gamusino"><img src="docs/images/robots/gamusino.jpg" width="250"><br><b>Gamusino</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://github.com/JavierIH">JavierIH</a> · <a href="https://github.com/JavierIH/ratatron">repo</a></td>
<td align="center">by <a href="https://oprobots.org">OPRobots</a> · <a href="https://github.com/OPRobots/ZoroBot3">repo</a></td>
<td align="center">by <a href="https://github.com/turmandreams">turmand<wbr>reams</a> · <a href="https://github.com/turmandreams/gamusino">repo</a></td>
</tr>
<tr>
<td align="center">STM32F103 · 4 Sharp IR · no gyro</td>
<td align="center">STM32F405 · 4 IR · gyro · fan</td>
<td align="center">ESP32 · 4 IR · gyro · fan</td>
</tr>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#bulebule"><img src="docs/images/robots/bulebule.jpg" width="250"><br><b>Bulebule</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#nezumibot"><img src="docs/images/robots/nezumibot.jpg" width="250"><br><b>NezumiBot</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#perenquén"><img src="docs/images/robots/perenquen.jpg" width="250"><br><b>Perenquén</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://github.com/Bulebots">Bulebots</a> · <a href="https://github.com/Bulebots/bulebule">repo</a></td>
<td align="center">by <a href="https://oprobots.org">OPRobots</a> · <a href="https://github.com/OPRobots/NezumiBot">repo</a></td>
<td align="center">by <a href="https://github.com/supernudo">supernudo</a> · <a href="https://github.com/supernudo/perenquen">repo</a></td>
</tr>
<tr>
<td align="center">STM32F103 · 4 IR · gyro · 4 wheels</td>
<td align="center">STM32F401 · 4 Sharp IR · N20 motors</td>
<td align="center">dsPIC33<wbr>EP512<wbr>MC806 · 4 IR · gyro · 4 wheels</td>
</tr>
<tr>
<td align="center" width="33%"><a href="docs/robots.md#fujin"><img src="docs/images/robots/fujin.jpg" width="250"><br><b>Fujin</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#futura"><img src="docs/images/robots/futura.jpg" width="250"><br><b>Futura</b></a></td>
<td align="center" width="33%"><a href="docs/robots.md#ukmarsbot"><img src="docs/images/robots/ukmarsbot.jpg" width="250"><br><b>UKMARSBOT</b></a></td>
</tr>
<tr>
<td align="center">by <a href="https://raiju.team">Team Raiju</a> · <a href="https://github.com/team-raiju/fujin">repo</a></td>
<td align="center">by <a href="https://micromouseusa.com">Green Ye</a> · <a href="https://micromouseusa.com/?page_id=1342">page</a></td>
<td align="center">by <a href="https://ukmars.org">UKMARS</a> · <a href="https://github.com/ukmars/ukmarsbot">repo</a> · <a href="https://github.com/ukmars/mazerunner-core">firmware</a></td>
</tr>
<tr>
<td align="center">STM32G474 · 4 IR · gyro · fan</td>
<td align="center">STM32F405 · 4 IR · gyro · its kit's test loop</td>
<td align="center">ATmega328P · 4 IR · N20 motors</td>
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
Its firmware is compiled on the first run and boots by itself, and it is started as at a competition: switched on in
the start cell, SELECT picks mode 1 and START launches the search after a 5 s countdown, from the start to the goal
and back, mapping the maze. When it is back, the race on the map it saved:

```sh
ratasim --maze oshwdem2026 --race 4      # race 2.4 of its race menu (SELECT twice, START, SELECT 3 times, START)
```

The console stays open meanwhile: `MODE 2 6` and `START` (or `:select`, `:start`, its buttons) run the fastest race;
`--console`: not started, only the console. In the viewer, Page Up is its SELECT button, Page Down its START and
End puts it back at the start.

The other robots (`ratasim zorobot|gamusino|bulebule|nezumibot|perenquen|fujin|futura|ukmarsbot`) also open MuJoCo's window, at real time
(`--record-speed 2` for twice that); with `--headless`, or `--record out.mp4` (a video instead of the window), or no
screen, they run without it.

Without a screen (over SSH), headless, as fast as possible, with a video:

```sh
ratasim --headless --speed 0 --maze oshwdem2026 --record search.mp4
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

### Gamusino in a maze

```sh
ratasim gamusino --maze oshwdem2026 --record gamusino.mp4
```

Its sketch (`src/gamusino/gamusino.ino`) on its ESP32, under a simulated Arduino core: 1.5 m ahead at 1.6 m/s,
steering by its gyro and encoders, a stop, a 180 deg turn, a rest and back, over and over. It reads no wall sensor
yet: it is put down at the start of the maze's longest straight (in the OSHWDEM 2026 maze, ten cells along its south
wall) and switched on, as its maker starts it: green, its gyro measured, blue, it goes.

![Gamusino on the maze's longest straight](docs/images/maze-gamusino.jpg)

### Bulebule in a maze

```sh
ratasim bulebule --maze oshwdem2026 --record bulebule.mp4
```

Its firmware (libopencm3 on its STM32F103, with its library mmlib) on a simulated chip, started as its makers start
it, with its button and a hand: a press wakes it, a long one picks the classic goal (the maze's centre), short presses
and a long one its force (0.25 N, its boot's); it blinks, waits with its LEDs on for a hand in front, and 2 s after the
hand goes it calibrates and goes. It explores to the centre and on, through the cells that could make a shorter path,
back to its start, and saves the path in its flash (`ratasim/bulebule/build/flash.bin`); `--run` races it from there,
to the goal and back (it does not get there yet: [robots.md](docs/robots.md#bulebule)).

![Bulebule exploring the OSHWDEM 2026 maze](docs/images/maze-bulebule.jpg)

### NezumiBot in a maze

```sh
ratasim nezumibot --maze oshwdem2026 --hand left --record nezumibot.mp4
```

Its firmware as it ran at OSHWDEM 2023 (a wall follower), started as its makers start it: held in the air, a hand
at its right sensor (`--party`: at its left, and it swaps walls every 30 s after 45 s), a hand at the side of the
wall to follow, put down in the start cell, a hand in front; it starts 2 s later.

![NezumiBot following the left wall](docs/images/maze-nezumibot.jpg)

### Perenquén in a maze

```sh
ratasim perenquen --maze oshwdem2026 --side right --gain 30 --record perenquen.mp4
```

Its firmware's last commit on a simulated dsPIC: a command line over Bluetooth, and a wall follower. Started as its
maker drove it: in the start cell, `start right 30` typed in its terminal (it reads its diagonal sensors there, the
walls it will keep to), a hand in front of both front sensors, taken away: it goes. `--speed 1000` types
`traj_speed distance 1000` first (2 m/s; it boots at 0.78), `--console` leaves its command line in the terminal.

![Perenquén keeping to the right wall](docs/images/maze-perenquen.jpg)

### Fujin in a maze

```sh
pip install -e .[cad]           # gmsh, to make its meshes from its STEP once
ratasim fujin --maze oshwdem2026 --record fujin.mp4
```

Its firmware (its RCX 2026 tag), C++23, on a board support package of ours behind its own BSP interface: its search,
to the goal and back, its map in its EEPROM. Started as its makers start it: button 2 (search), its speeds picked
with its buttons (`--mode slow`: 0.3 m/s, no fan; medium and fast do not finish yet), its exploration, and a hand in
front of its front sensors; two seconds' beep and it goes. It needs g++ 13 or clang++ 15 or later.

![Fujin searching the OSHWDEM 2026 maze](docs/images/maze-fujin.jpg)

### Futura in a maze

```sh
ratasim futura --maze oshwdem2026 --echo --record futura.mp4
```

The kit mouse of UCLA IEEE's Project Futura, with the firmware template its students start from (downloaded from its
page on the first run): a beep, "mous" on its display, then once a second its four IR pairs, gyro, battery and
encoders printed on its serial port (`--echo`) and both motors at 100 of 999. It steers by nothing: put down in the
start cell and switched on, it rolls ahead to the wall that ends the column and stays against it, its front sensors
printing that wall.

![Futura at the end of the start column](docs/images/maze-futura.jpg)

### UKMARSBOT in a maze

```sh
ratasim ukmarsbot --maze oshwdem2026 --record ukmarsbot.mp4
```

mazerunner-core, Peter Harrison's firmware for it, as his ORION, on its Arduino Nano's ATmega328P (its registers,
timers, ADC and interrupts simulated): its search, flood fill from the start to the goal and back, mapping the maze.
Started as at a contest: backed up to the start cell's wall, switched on with its button held (its map cleared), its
function switches at 2 and its button pressed, a hand in front of its left front sensor (`--function 3`: its left
wall follower to the goal; `--console`: its serial command line afterwards).

![UKMARSBOT searching the OSHWDEM 2026 maze](docs/images/maze-ukmarsbot.jpg)

## The stopwatch

Every runner times the robot's runs as at a competition: the stopwatch starts when the robot's centre leaves the
start cell, from zero every time it does, and stops when it enters the goal (the centre 2x2 cells; Ratatron's
`--practice` build: its cell 3,2). Putting the robot somewhere by hand (Ratatron's `:place`, `:reset`) ends the run.
MuJoCo's window shows the time in its top right corner, and videos under their title, green once at the goal. The
runners print it, and `tests/robots.py` reports the times (and fails Ratatron and ZoroBot3 if they reach the goal
without one):

```text
[   5.33] stopwatch: run 1 started (left the start cell)
[  38.12] stopwatch: run 1 at the goal (8, 8) in 32.785 s
```

## Layout

```text
ratasim/            the simulator: the maze, the physics, Ratatron (its model, sensors, simulated chip in fw/)
ratasim/zorobot/    ZoroBot3: its model from its STEP, its simulated chip (fw/), its runner
ratasim/gamusino/   Gamusino: its model from the GLB in its repository, its ESP32 under an Arduino core (fw/), runner
ratasim/bulebule/   Bulebule: its board from its layout, its parts from KiCad's library, its simulated chip (fw/), runner
ratasim/nezumibot/  NezumiBot: its assembled 3D model (assets/), its simulated chip (fw/), runner
ratasim/perenquen/  Perenquén: its meshes from its FreeCAD assembly (assets/, cad.py), its simulated dsPIC (fw/), runner
ratasim/fujin/      Fujin: its model from its STEP (cad.py), its firmware on our BSP and simulated STM32G474 (fw/), runner
ratasim/futura/     Futura: its files downloaded at build time, its model, its simulated STM32F405 and ST library (fw/)
ratasim/ukmarsbot/  UKMARSBOT: its model from its KiCad boards and Gerbers (cad.py, pcb.py), its Nano's ATmega328P (fw/)
ratasim/opencm3/    the part of libopencm3 the simulated chips of ZoroBot3 and NezumiBot share
ratasim/start.py    starting a robot as its makers do (its config's sequence), with the hand of hand.py
ratasim/stopwatch.py the runs' times, from the start cell to the goal (every robot)
ratasim/scenes/     several robots: drag race, crash, loop, pictures
ratasim/mazes/      competition mazes (micromouseonline text format)
external/           the robots' repositories (submodules)
tests/robots.py    every robot in the maze, checked by what it prints (the CI: one job per robot)
docs/               robots.md (each robot, its authors), simulator.md (Ratatron in detail)
```

## Off the maze

Scenes with several robots, no firmware: each robot follows its own speed profile (Ratatron's race preset,
ZoroBot3's `SPEED_HAKI`) and keeps its lane.

```sh
ratasim drag-race race.mp4          # 3 m: start lights, lanes, a checkered finish
ratasim crash crash.mp4 --ramp      # head-on on the maze's longest straight, over a ramp
ratasim loop loop.mp4               # a loop-the-loop beside the maze
ratasim portraits                   # the pictures above
```

| ![Drag race](docs/images/drag-race.jpg) | ![Head-on on a ramp](docs/images/crash-ramp.jpg) | ![Loop-the-loop](docs/images/loop.jpg) |
|:-:|:-:|:-:|

## Credits and licences

Every robot belongs to its authors (above); their repositories are submodules and nothing of them is copied here.
ZoroBot3, Bulebule and NezumiBot are GPLv3; Gamusino's and Fujin's repositories carry no licence; Perenquén's design is CC BY-SA
4.0 (Javier Baliñas), its firmware and its library aversive4dspic (Droids Corporation, Microb Technology, Eirbot,
Eurobotics Engineering) GPLv2+; Gamusino's gyro library, STM32duino LSM6DSR (STMicroelectronics, BSD-3), is
downloaded at build time. Futura's page (Green Ye, micromouseusa.com) gives no licence: its firmware template, its
board's Gerber files and its printed parts are downloaded at build time, pinned by their SHA-256, and stay out of git; ST's library is not used
(`ratasim/futura/fw` simulates the part of its API the template calls).
The meshes made from ZoroBot3's, Gamusino's, Bulebule's and Fujin's CAD stay out of git; NezumiBot's and Perenquén's are in `ratasim/nezumibot/assets` and `ratasim/perenquen/assets`, under their licences.
