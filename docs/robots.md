# Supported robots

| [<img src="images/robots/ratatron.jpg" width="260"><br>**Ratatron**](#ratatron) | [<img src="images/robots/zorobot3.jpg" width="260"><br>**ZoroBot3**](#zorobot3) | <img src="images/robots/gamusino.jpg" width="260"><br>**Gamusino** |
|:-:|:-:|:-:|
| STM32F103 · 4 Sharp IR · no gyro | STM32F405 · 4 IR · gyro · suction fan | ESP32 · 4 IR · gyro · suction fan<br>*coming soon* |

Every robot is a package of `ratasim/` with the same parts: its **model** (meshes from its CAD, collisions from simple
shapes), its **physics** (mass, motors, tires; what was measured apart from what is assumed), its **sensors** (IR rays
through its firmware's own calibration, encoders, gyro) and its **firmware**, compiled for the host from its
repository (a submodule of `external/`) on a simulated chip. The pictures: `ratasim portraits`.

## Ratatron

- **Firmware:** [JavierIH/ratatron](https://github.com/JavierIH/ratatron) (`external/ratatron/firmware/src`), unchanged,
  on a fake STM32 HAL (`ratasim/fw/`): its application files and `gpio.c`/`infrared.c` as they are, the motors,
  encoders, UART, flash and clock simulated.
- **Model:** its FreeCAD assembly, exported to `ratasim/assets` (`ratasim/export_cad.py`).
- **Run:** `ratasim --maze oshwdem2026`, then `MODE 1` and `START` in the console. Details, calibration and the studies:
  [simulator.md](simulator.md).

## ZoroBot3

- **Firmware:** [OPRobots/ZoroBot3](https://github.com/OPRobots/ZoroBot3) (`external/ZoroBot3`, GPLv3), unchanged, on
  a fake libopencm3 chip (`ratasim/zorobot/fw/`): its `main()` in a coroutine, SysTick, its control loop (TIM5) and
  its IR emitter/ADC state machine (TIM2, 16 kHz) every simulated millisecond; the LSM6DSR on SPI, the AS5145B
  encoders, the MP6551 motor PWM, the fan, the RGB LED, the EEPROM sector, the UART.
- **Model:** its Onshape assembly (the "Full Assembly", exported as one STEP file, 297 parts where its designers
  placed them), tessellated with gmsh (`ratasim/zorobot/cad.py`, `pip install -e .[cad]`). The STEP is not in its
  repository nor in this one: put it at `ratasim/zorobot/cad/Full_Assembly.step`.
- **Physics, assumed (nothing measured):** 70 g, 8520 coreless motors (R 2 Ω, 45 000 rpm at 7.4 V), 8 N of suction at
  full fan, tire friction 1.1; the wheel radius is its firmware's (12.44 mm, the CAD's tire 11.56).
- **Run:** `ratasim zorobot --maze oshwdem2026 [--run] [--record out.mp4]`: it boots, picks the run with its menu
  buttons and starts with a hand in front of its right front sensor, as on the real robot.
- **Open:** its explore turns end 3-5 deg off, so it touches walls (the assumed motors and grip).
