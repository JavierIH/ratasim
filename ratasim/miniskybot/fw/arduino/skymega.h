// The Skymega board's servo connectors by their Arduino pins, as its wiki's pin table gives them
// (iearobotics.com/wiki, SkyMega: "Asignación de pines de la Skymega"): what Obijuan's Miniskybot examples include
// for SERVO2 and SERVO4. Ours (the original skymega.h, on iearobotics' old SVN, is no longer online).
#pragma once
#include "Arduino.h"

#define SERVO1 A0   // PC0
#define SERVO2 8    // PB0
#define SERVO3 A1   // PC1
#define SERVO4 9    // PB1
#define SERVO5 A2   // PC2
#define SERVO6 10   // PB2
#define SERVO7 A3   // PC3
#define SERVO8 11   // PB3
