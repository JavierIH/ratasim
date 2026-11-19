// Futura's main routine around its page's controller sample (controller-sample.c), in place of its kit's template's
// (user/main.c, whose loop drives both motors at 100 and prints its sensors): what the sample leaves to its students,
// the least to run it, as its lecture's guide to testing it (Guide-for-testing-controller.docx) says.
//
//   systick()     the sample's systickHandler (its comment): its IR pairs read (and in a cell's run, their error,
//                 getSensorEror()), its gyro, speedProfile(), each millisecond (SysTick_Handler, stm32f4xx_it.c,
//                 counts Millis and calls it)
//   its globals   the sample's variables, its thresholds and its switches (include/controller.h); its own values
//                 (gains, accelerations, speeds) are the sample's, but maxSpeed, brought down to moveSpeed (its
//                 guide's step 2, a low constant speed: up to 2 m/s and back in every cell, its wheels slip)
//   pivot()       its guide's step 6: targetSpeedX 0, targetSpeedW a constant, the gyro's angle to end it
//   main()        the kit's template's setup and beep, then cells one after another with the sample's
//                 moveOneCell(); at a wall, stopped in the cell's middle, it looks left (a pivot of 90 deg), then
//                 right (180), then back (90 more), until a way is open
//
// The rest is this file's own: its thresholds, measured in the simulator as its students would on the mouse (its
// IR pairs' light back a wall 78 and 168 mm ahead, its gyro's counts a degree, its track), its pivots' speed, and
// what it prints.
#include "stm32f4xx.h"
#include "delay.h"
#include "led.h"
#include "button.h"
#include <stdio.h>
#include "usart.h"
#include "SPI.h"
#include "matrixDisplay.h"
#include "pwm.h"
#include "encoder.h"
#include "buzzer.h"
#include "main.h"
#include "sensor_Function.h"
#include "adc.h"
#include "controller.h"

int32_t leftEncoder, rightEncoder, leftEncoderOld, rightEncoderOld;
int32_t leftEncoderChange, rightEncoderChange, encoderChange;
int32_t leftEncoderCount, rightEncoderCount, encoderCount_, oldEncoderCount, distanceLeft;
int32_t leftBaseSpeed, rightBaseSpeed;
// Its IR pairs' error (getSensorEror(), each ms in a cell's run) in its rotational feedback (config.py: the line its
// comment gives), over a_scale. A wall too near on its left gives a negative error (DLMiddleValue - DLSensor), which
// as it is reads as a turn to the right and turns it left, into the wall: a_scale is negative ("make sure to check
// the sign of sensor error"), its size measured.
int32_t sensorError, a_scale = -100;
// Neither "only": its encoders' feedback and its gyro's, aSpeed / gyroFeedbackRatio, whose 5700 is for another
// scale of aSpeed than readGyro()'s (0.208 a deg/s, under 100): 0, the encoders' alone, as its guide's pivots have it
bool onlyUseGyroFeedback = false, onlyUseEncoderFeedback = false;
bool useSensor, useGyro, usePID, useSensorFeedback;

int32_t oneCellDistance = 180 * COUNTS_PER_MM;  // a cell, 180 mm
// Its front pairs' light back (lit minus dark, counts): a wall 78 mm ahead of them (the robot 38 mm short of the
// cell's middle, where from moveSpeed it stops: 21 mm at decX, 17 more of its PD's lag, measured), and 168 mm (half
// a cell farther: a wall seen).
int32_t LFvalue1 = 500, RFvalue1 = 500;
int32_t LFvalue2 = 108, RFvalue2 = 108;
// Its side pairs' light back in a cell's middle with a wall on their side (60 deg out, 79 mm along their beams)
int32_t DLMiddleValue = 480, DRMiddleValue = 480;

#define ANGLE_PER_DEG 206.5f   // readGyro()'s angle a degree: aSpeed each ms, 0.208 a deg/s (LY3200ALH: 0.67 mV/dps),
                               // under that by 0.7% (measured: its integer divisions drop what is left)
#define TRACK_MM 69.0f         // its wheels' track
#define PIVOT_SPEED 30         // targetSpeedW for a pivot: right minus left counts a ms (390 deg/s)
#define PIVOT_LAG_DEG 2.6f     // how far it turns past where its braking ends it (measured: the PD's lag)

static int cells, pivots;

void systick(void) {
  if (useSensor) {
    readSensor();
    if (useSensorFeedback)
      getSensorEror();
    else
      sensorError = 0;
  }
  if (useGyro)
    readGyro();
  if (usePID)
    speedProfile();
}

void button1_interrupt(void) {
}

void button2_interrupt(void) {
}

// A wall seen ahead (moveOneCell()'s LFvalue2), and one where moveOneCell() ends at it (LFvalue1)
static int frontWall(void) {
  return LFSensor > LFvalue2 || RFSensor > RFvalue2;
}

static int atFrontWall(void) {
  return LFSensor >= LFvalue1 || RFSensor >= RFvalue1;
}

// Stopped where it is: targetSpeedX 0, then still for a while
static void stop(void) {
  useSensorFeedback = false;
  targetSpeedX = 0;
  targetSpeedW = 0;
  while (curSpeedX != 0 || curSpeedW != 0)
    delay_ms(1);
  delay_ms(150);
}

// Its guide's pivot turn (step 6): degrees left (negative right); the gyro's angle ends it, its speed brought back
// to 0 at decW when what is left of the turn is what that takes
static void pivot(int degrees) {
  float target = (abs(degrees) - PIVOT_LAG_DEG) * ANGLE_PER_DEG;
  angle = 0;
  targetSpeedX = 0;
  targetSpeedW = degrees > 0 ? PIVOT_SPEED : -PIVOT_SPEED;
  for (;;) {
    float w = curSpeedW < 0 ? -curSpeedW : curSpeedW;
    float braking = w * w / 2 / decW / COUNTS_PER_MM / TRACK_MM * 57.2958f * ANGLE_PER_DEG;
    if (abs(angle) + braking >= target)
      break;
    delay_ms(1);
  }
  stop();
  pivots++;
  printf("pivot %d: %d deg\r\n", pivots, degrees);
}

int main(void) {
  Systick_Configuration();
  LED_Configuration();
  button_Configuration();
  usart1_Configuration(9600);
  SPI_Configuration();
  TIM4_PWM_Init();
  Encoder_Configration();
  buzzer_Configuration();
  ADC_Config();
  shortBeep(2000, 8000);
  delay_ms(1000);
  resetSpeedProfile();
  maxSpeed = moveSpeed;
  enable_sensor();
  enable_gyro();
  enable_PID();
  displayMatrix("run ");
  delay_ms(100);
  printf("controller sample: running\r\n");
  while (1) {
    // cells one after another until moveOneCell() ends at a wall; it may end short of it, a reading on LFvalue2
    // itself being in none of its loop's conditions: then again, to the wall
    while (!atFrontWall()) {
      int32_t from = encoderCount_;
      distanceLeft = oneCellDistance;
      useSensorFeedback = true;
      moveOneCell();
      if (encoderCount_ - from > oneCellDistance / 2) {
        cells++;
        printf("cell %d\r\n", cells);
      }
    }
    stop();
    printf("wall ahead: LF %d RF %d\r\n", LFSensor, RFSensor);
    // stopped in the cell's middle: a look left, right, back, for the way on
    pivot(90);
    if (frontWall()) {
      pivot(-180);
      if (frontWall())
        pivot(-90);
    }
  }
}
