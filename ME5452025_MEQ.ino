#include "Adafruit_VL53L0X.h"
#include <Arduino.h>
#include "src/StateMachine/StateMachine.h"

// I2C addresses for the two VL53L0X
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Shutdown pins
const int SHT_LOX1 = 11;
const int SHT_LOX2 = 12;

// Sensor objects
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

// Motor pins (TB6612FNG)
const int AIN1 = 6, AIN2 = 4, PWMA = 5;
const int BIN1 = 7, BIN2 = 8, PWMB = 9;

// PID gains
const float Kp = 3.0;
const float Ki = 0.0;
const float Kd = 1.2;

// Target distance to left wall [mm]
const int targetDist = 250;
int prev_dist;
bool left_corner_mode = false;
bool right_corner_mode = false;
int reacquire_threshold = 300;
int dist1, dist2;
const float alph = 0.3;          // smoothing factor (0<α<1)
float filtDist1= targetDist;

// PID state
float prevError = 0;
float integralTerm = 0;
unsigned long prevTime = 0;

// Sampling rate
const int samplingFreq = 20;               // Hz
const unsigned long samplingInterval = 1000UL / samplingFreq;  // ms

// Camera ball tracking data (coordinate of closest ball)
int camera_x = -1;
int camera_y = -1;

// Strategy-organizing AI
StateMachine fsm = StateMachine();

// User defined functions
void setSensorIDs();
void readDualSensors(int &d1, int &d2);
void driveforward(int speedR, int speedL);
void sharpleftturn();
void leftWallFollow(int error);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  // sensor shutdown pins
  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  setSensorIDs();
  prevTime = millis();
}

void loop() {
  // TODO: Move all of this stuff to the corresponding state later
  static unsigned long lastSample = 0;
  unsigned long now = millis();

  // only run at sampling rate
  if (now - lastSample < samplingInterval) return;
  lastSample = now;
  // place these up near your globals
    // initialize to your setpoint

  // in loop, right after readDualSensors(dist1, dist2):

  // read both sensors
  readDualSensors(dist1, dist2);
  if (dist1 > 0) {
    filtDist1 = alph*dist1 + (1-alph)*filtDist1;
  }
  int useDist1 = round(filtDist1);
  //Serial.println(left_corner_mode);
  // skip if no valid reading
  if (useDist1 <= 0) return;

  // left‐wall corner detection logic
  if (!left_corner_mode) {
    // entered corner when distance jumps above threshold
    if (useDist1 > prev_dist + reacquire_threshold) {
      left_corner_mode = true;
    }
    else {
      // normal wall follow
      prev_dist = useDist1;
      leftWallFollow(targetDist - useDist1);
    }
  }
  else {
    // corner mode: spin left sharply
    sharpleftturn();

    // exit corner when we get back under threshold
    if (dist1 < reacquire_threshold) {
      left_corner_mode = false;
    }

    // keep prev_dist up to date so we don’t re‐enter immediately
    prev_dist = useDist1;
  }
  // end TODO

  // figure out how to assign the first and second distances as "wall follower" and "wall seeker"
  switch(fsm.run(camera_x, dist1, dist2)){
    case State::SEEK:
      // Code SEEK routine here
    case State::WALLFOLLOW:
      // Code WALLFOLLOW routine here
    case State::PICKUP:
      // Code PICKUP routine here
    case State::UTURN:
      // Code UTURN routine here
    case State::EXIT:
      // Code EXIT routine here
    default:
      // Double check this: you can print something to serial monitor while reading Rasp-pi right?
      Serial.println("FATAL ERROR");
      Serial.println("State code: " + static_cast<int>(fsm.getState()));
      Serial.println("Last input: " + static_cast<int>(fsm.getPrevInput()));
      break;
  }
}

void setSensorIDs() {
  // reset both sensors
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  // wake both
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // init LOX1
  digitalWrite(SHT_LOX2, LOW);  // keep #2 asleep
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot LOX1"));
    while (1);
  }
  delay(10);

  // init LOX2
  digitalWrite(SHT_LOX2, HIGH);
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot LOX2"));
    while (1);
  }
}

void readDualSensors(int &d1, int &d2) {
  lox1.rangingTest(&measure1, false);
  lox2.rangingTest(&measure2, false);

  d1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
  d2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;
}

void driveforward(int speedR, int speedL) {
  // Right motor
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, constrain(speedR, 0, 255));
  // Left motor
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, constrain(speedL, 0, 255));
}

void sharpleftturn(){
  int speedR = 150;
  int speedL = 50;
  // Right motor
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, constrain(speedR, 0, 255));
  // Left motor
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, constrain(speedL, 0, 255));
}

void leftWallFollow(int error) {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  integralTerm += error * dt;
  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = Kp * error + Ki * integralTerm + Kd * derivative;

  int speedR = 150 + output;  // slow right wheel if too close
  int speedL = 150 - output;  // speed up left wheel if too far
  Serial.print(constrain(speedR, 0, 255));
  Serial.print("  ");
  Serial.print(constrain(speedL, 0, 255));
  Serial.println("------------");
 
  driveforward(speedR, speedL);
}