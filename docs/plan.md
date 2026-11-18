# Plan

In order; each step ends committed and pushed.

1. **README and pictures.** The README starts with the robots in the maze; each robot with its authors and a link to
   its repository; the scenes off the maze at the end. The robots' pictures on a dark studio. A slot for every robot
   coming (Gamusino, Bulebule, NezumiBot).
2. **Gamusino from its repository** ([turmandreams/gamusino](https://github.com/turmandreams/gamusino)): a submodule
   (no licence in it: nothing of it copied, its meshes made at build time), its model (`ratasim/gamusino`, from the
   ratatron repo's local branch `sim-gamusino`: the GLB in the submodule, the panel's wings cut off its PCB).
3. **Bulebule model** ([Bulebots/bulebule](https://github.com/Bulebots/bulebule), GPLv3): a submodule; its PCB from
   the screenshot of its Upverter layout (outline fitted, texture recoloured), the components of its docs from their
   published 3D models (Blue Pill, HC-05, MPU-6500 board, DRV8833 module, DC-DC converters, Faulhaber motors, 5 mm IR
   LEDs, connectors, switch), its printed parts from its CadQuery dimensions (mounts, rims, gears).
4. **NezumiBot** ([OPRobots/NezumiBot](https://github.com/OPRobots/NezumiBot), GPLv3, based on Ratatron): a
   submodule; its model (its STLs and Ratatron's parts), its firmware (libopencm3 on an STM32F401: ZoroBot3's
   simulated chip, with Sharp sensors and N20 motors) in the maze.
5. **One way to start every robot.** Each robot starts as its makers start it (buttons, a hand in front of a
   sensor), and the simulator does it the same way for all: a hand (`ratasim/hand.py`, a mocap body every robot's
   sensor rays see, drawn in videos) and buttons, moved by the robot's start sequence (`start()` in its config, steps
   `say`, `wait`, `press`, `send`, `hand`, `lift`, `put_down`) that `ratasim/start.py` plays. ZoroBot3 and NezumiBot on it;
   Ratatron keeps its console (`:start`, `:select`, its commands).
6. **Bulebule firmware**: libopencm3 on an STM32F103, its library (mmlib) through its `platform.h`; in the maze.
7. **Gamusino firmware**: its Arduino sketch for the ESP32 on a host Arduino shim (pins, PWM, ADC, SPI for the
   LSM6DSR, encoders, NeoPixel, serial); in the maze.
8. **Perenquen** ([supernudo/perenquen](https://github.com/supernudo/perenquen), by Javier Baliñas, CC BY-SA 4.0):
   a submodule; its model from its FreeCAD assemblies and its KiCad board, its firmware (dsPIC33 with aversive4dspic)
   on a simulated dsPIC; in the maze.
9. **Scenes with every robot**: the drag race, a maze race; robots added to scenes by name.
10. **The ratatron repository without its simulator** (when ratasim is validated): `software/ratatron/sim` and its
   commands removed there, its docs pointing here; the local branches `sim-zorobot` and `sim-gamusino` retired.

## Status

- [x] 1 · [x] 2 · [x] 3 · [x] 4 · [x] 5 · [ ] 6 · [ ] 7 · [x] 8 ·
  [ ] 9 · [ ] 10
