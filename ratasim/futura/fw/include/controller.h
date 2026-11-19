/* What Futura's controller sample (controller-sample.c, its page's "Sample Code", 2016/04) leaves to its students:
 * its globals' declarations, its units' conversions, its thresholds' names, its switches. Forced into the sample
 * (fw/Makefile: -include controller.h -DCONTROLLER_SAMPLE) and included by fw/main.c, which defines them all.
 *
 *   speed_to_counts   mm/s to encoder counts a millisecond (float: its accX's ramp, (float)(speed_to_counts(accX*2)
 *                     /100), is under one count and would round to nothing in an int); counts_to_speed the reverse
 *   encoderCount      read through fsim_poll(): moveOneCell() waits in a loop on it, which on the chip SysTick's
 *                     interrupt breaks into; in the simulator each pass takes the chip's time (as Millis, fsim.h)
 *   its misspellings  guroFeedback, LFvalues1, RFvalve2: the names it means
 *   moveOneCell       written with neither a type nor its parentheses: given them (in the sample only) */
#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <stdbool.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "pwm.h"
#include "sensor_Function.h"

// Its wheels' encoders (as ratasim/futura/config.py: 512 lines x4 a motor turn, 40:12 gears, 34 mm wheels):
// 6826.7 counts a wheel turn, 106.8 mm
#define COUNTS_PER_MM 63.91f
#define speed_to_counts(mm_per_s) ((mm_per_s) * COUNTS_PER_MM / 1000.0f)
#define counts_to_speed(counts) ((counts) * 1000.0f / COUNTS_PER_MM)

extern float curSpeedX, curSpeedW;
extern int targetSpeedX, targetSpeedW;
extern float accX, decX, accW, decW;
extern int moveSpeed, turnSpeed, stopSpeed, maxSpeed;

extern int32_t leftEncoder, rightEncoder, leftEncoderOld, rightEncoderOld;
extern int32_t leftEncoderChange, rightEncoderChange, encoderChange;
extern int32_t leftEncoderCount, rightEncoderCount, encoderCount_, oldEncoderCount, distanceLeft;
extern int32_t leftBaseSpeed, rightBaseSpeed;
extern int32_t sensorError, a_scale;
extern bool onlyUseGyroFeedback, onlyUseEncoderFeedback, useSensor, useGyro, usePID;
extern int32_t oneCellDistance, LFvalue1, LFvalue2, RFvalue1, RFvalue2, DLMiddleValue, DRMiddleValue;

volatile int32_t *fsim_poll(volatile int32_t *p);
#define encoderCount (*fsim_poll(&encoderCount_))

#define enable_sensor() (useSensor = true)
#define enable_gyro() (useGyro = true)
#define enable_PID() (usePID = true)

void speedProfile(void);
void getEncoderStatus(void);
void updateCurrentSpeed(void);
void calculateMotorPwm(void);
int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd);
void resetSpeedProfile(void);
void getSensorEror(void);
void moveOneCell(void);

#ifdef CONTROLLER_SAMPLE
#define guroFeedback gyroFeedback
#define LFvalues1 LFvalue1
#define RFvalve2 RFvalue2
#define moveOneCell void moveOneCell(void)
#endif
#endif
