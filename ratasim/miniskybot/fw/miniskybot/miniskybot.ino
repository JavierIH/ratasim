//----------------------------------------------------------------------------------------------------------------
//-- Miniskybot 2 in a micromouse maze
//--
//-- For the robot as it is: a Skymega board (Arduino, ATmega328P), its two Futaba S3003 servos hacked for
//-- continuous rotation on SERVO2 (right) and SERVO4 (left), driven as Obijuan's examples drive them
//-- (motor_test.ino), and its one sensor, the SRF02 ultrasonic ranger in its front, on the I2C bus. No encoders:
//-- its moves are timed.
//--
//-- What it does, the simplest that keeps it going round the maze:
//--   - it waits for its button (its LED blinking), then
//--   - it goes straight on, ranging ahead every 70 ms;
//--   - when the wall ahead is near (WALL_CM), it stops and squares up to it: a wall echoes back only while it
//--     faces it within its ranger's beam, so it turns slowly left to where the echo ends, then right across to
//--     where it ends on that side, and back half that: square to the wall. Its timed turns are a few degrees off
//--     each (its servos take a pulse every 20 ms), and this puts it straight again at every wall;
//--   - it goes on for the time that puts it in the middle of its cell (from the distance it measured) and stops
//--     (if it is nearer than its ranger's minimum, it backs off to where it can range first);
//--   - it turns 90 degrees, by time, and looks: if the way is clear it goes on; if not, it turns 180 degrees and
//--     looks the other way; if both are closed, 90 degrees more: back where it came from;
//--   - the side it tries first changes at every cell it turns in (right, then left, then right...), so that a
//--     corridor does not always lead it back to the same one;
//--   - with no encoders, it knows it moves only by its range changing: if in STUCK_MS its range has not changed
//--     by 3 cm (it is against a wall it meets askew, which echoes nothing back), it backs off and turns as at a
//--     wall.
//----------------------------------------------------------------------------------------------------------------
#include <Servo.h>
#include <Wire.h>
#include <skymega.h>

//-- Its motors, as Obijuan's examples set them (the servos stop at 90)
const int MOTOR_RIGHT = SERVO2;
const int MOTOR_RIGHT_FW = 0;
const int MOTOR_RIGHT_BW = 180;
const int MOTOR_LEFT = SERVO4;
const int MOTOR_LEFT_FW = 180;
const int MOTOR_LEFT_BW = 0;
const int MOTOR_STOP = 90;
const int MOTOR_STOP_US = 1472;     //-- 90 in us (Servo maps 0..180 to 544..2400)
const int SLOW_US = 40;             //-- a slow turn: its servos this far from their stop (about a quarter speed)

//-- The Skymega's test LED and button (the button to ground: low when pressed)
const int LED = 13;
const int BUTTON = 12;

//-- The SRF02 on the I2C bus: 0xE0 in its data sheet's 8 bits
const int SRF02 = 0x70;
const int SRF02_CM = 0x51;          //-- its command: a ranging, the result in cm
const int SRF02_MIN_CM = 16;        //-- nearer than this it gives about this

//-- The maze and the robot (mm, ms), measured on the robot
const long SENSOR_TO_AXLE_MM = 41;  //-- its sensor's face ahead of its wheels' axle
const long WALL_TO_CENTRE_MM = 84;  //-- a cell's middle from the faces of its walls
const long SPEED_MM_S = 137;        //-- its speed at full: its servos' 43 rpm on its 60 mm wheels
const long TURN_90_MS = 486;        //-- a 90 degree turn in place at full
const long TURN_180_MS = 966;       //-- and 180 (not twice 90: its start's lag counts once)
const long STUCK_MS = 3000;         //-- this long with the same range going on: stuck
const long BACK_OFF_MS = 300;       //-- backwards then: 4 cm
const long BACK_MAX_MS = 1500;      //-- backwards to range a wall too near: 20 cm at most
const long PAUSE_MS = 150;          //-- stopped after each move, before ranging
const long SWEEP_MAX_MS = 3000;     //-- squaring up: no sweep longer than this
const int WALL_CM = 24;             //-- a wall this near ahead: it is in the next cell's far side, or nearer
const int OPEN_CM = 20;             //-- after a turn: this far or more, the way is open

Servo rm, lm;
int side = 1;                       //-- the side it tries first: 1 right, -1 left
int last = -1;                      //-- its range when it last changed by 3 cm (cm; -1: none yet)
unsigned long since = 0;            //-- and when

void forward() {
  rm.write(MOTOR_RIGHT_FW);
  lm.write(MOTOR_LEFT_FW);
}

void backward() {
  rm.write(MOTOR_RIGHT_BW);
  lm.write(MOTOR_LEFT_BW);
}

void stop() {
  rm.write(MOTOR_STOP);
  lm.write(MOTOR_STOP);
}

void back_off() {
  backward();
  delay(BACK_OFF_MS);
  stop();
  delay(PAUSE_MS);
}

//-- In place, slowly: 1 to the right, -1 to the left (to the right the left wheel forward, the right back: both
//-- servos past their stop the same way)
void slow_turn(int dir) {
  lm.writeMicroseconds(MOTOR_STOP_US + dir * SLOW_US);
  rm.writeMicroseconds(MOTOR_STOP_US + dir * SLOW_US);
}

//-- In place: 1 to the right, -1 to the left, by one or two quarters of a turn
void turn(int dir, int quarters) {
  if (dir > 0) {
    lm.write(MOTOR_LEFT_FW);
    rm.write(MOTOR_RIGHT_BW);
  } else {
    lm.write(MOTOR_LEFT_BW);
    rm.write(MOTOR_RIGHT_FW);
  }
  delay(quarters == 2 ? TURN_180_MS : TURN_90_MS);
  stop();
  delay(PAUSE_MS);
}

//-- The SRF02's distance to what is ahead (cm), as its data sheet reads it: the command, 66 ms, registers 2 and 3.
//-- 0: no echo (nothing in its range); -1: it did not answer.
int range_cm() {
  Wire.beginTransmission(SRF02);
  Wire.write(0);
  Wire.write(SRF02_CM);
  Wire.endTransmission();
  delay(70);
  Wire.beginTransmission(SRF02);
  Wire.write(2);
  Wire.endTransmission();
  Wire.requestFrom(SRF02, 2);
  if (Wire.available() < 2) return -1;
  int high = Wire.read();
  int low = Wire.read();
  return (high << 8) | low;
}

//-- Turning slowly to dir until the wall ahead, d cm away, echoes no more (past where it echoes again, if it does
//-- not when it starts): how long it took
long sweep(int dir, int d) {
  slow_turn(dir);
  unsigned long t0 = millis();
  bool seen = false;
  while (millis() - t0 < (unsigned long)SWEEP_MAX_MS) {
    int r = range_cm();
    bool echo = r > 0 && abs(r - d) <= 2;
    if (echo) seen = true;
    else if (seen) break;
  }
  long t = millis() - t0;
  stop();
  delay(PAUSE_MS);
  return t;
}

//-- Square to the wall ahead, d cm away: to one end of its echo, across to the other, back half that
void square_up(int d) {
  long t0 = sweep(-1, d);                              //-- it echoes where it starts
  long t = sweep(1, d);
  Serial.print("square: ");
  Serial.print(t0);
  Serial.print(" ms left, ");
  Serial.print(t);
  Serial.println(" ms right");
  slow_turn(-1);
  delay(t / 2);
  stop();
  delay(PAUSE_MS);
}

bool open_ahead() {
  int d = range_cm();
  return d == 0 || d >= OPEN_CM;
}

//-- On: from here its range has to shrink
void go_on() {
  forward();
  last = -1;
  since = millis();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  rm.attach(MOTOR_RIGHT);
  lm.attach(MOTOR_LEFT);
  stop();
  Serial.println("Miniskybot: press its button");
  while (digitalRead(BUTTON) == HIGH) {
    digitalWrite(LED, (millis() / 250) % 2);
    delay(10);
  }
  digitalWrite(LED, HIGH);
  delay(1000);                      //-- the hand away
  Serial.println("go");
  go_on();
}

//-- Backwards until it can range the wall ahead (it is nearer than its minimum), BACK_MAX_MS at most: its range
int back_to_range() {
  backward();
  unsigned long t0 = millis();
  int d = range_cm();
  while (d > 0 && d <= SRF02_MIN_CM + 1 && millis() - t0 < (unsigned long)BACK_MAX_MS) d = range_cm();
  stop();
  delay(PAUSE_MS);
  return range_cm();
}

void loop() {
  int d = range_cm();
  if (last < 0 || abs(d - last) >= 3) {
    last = d;
    since = millis();
  }
  if (millis() - since > (unsigned long)STUCK_MS) {   //-- its range the same for long: stuck
    Serial.println("stuck");
    back_off();
  } else if (d <= 0 || d > WALL_CM) {
    return;                                            //-- the way ahead is clear: on
  } else {
    //-- A wall ahead: square to it, on to the middle of its cell, stop
    Serial.print("wall at ");
    Serial.print(d);
    Serial.println(" cm");
    stop();
    delay(PAUSE_MS);
    if (d <= SRF02_MIN_CM) d = back_to_range();        //-- too near to know how near
    if (d > SRF02_MIN_CM && d <= WALL_CM) {
      square_up(d);
      d = range_cm();
    }
    if (d > SRF02_MIN_CM && d <= WALL_CM) {
      forward();
      delay((d * 10L - (WALL_TO_CENTRE_MM - SENSOR_TO_AXLE_MM)) * 1000 / SPEED_MM_S);
      stop();
      delay(PAUSE_MS);
    }
  }

  //-- Its first side, then the other, then back
  turn(side, 1);
  if (open_ahead()) {
    Serial.println(side > 0 ? "right" : "left");
  } else {
    turn(-side, 2);
    if (open_ahead()) {
      Serial.println(side > 0 ? "left" : "right");
    } else {
      turn(-side, 1);
      Serial.println("back");
    }
  }
  side = -side;
  go_on();
}
