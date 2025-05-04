#include "Adafruit_VL53L0X.h"
#include <Arduino.h>

// ==== CONFIG ====
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
const int SHT_LOX1 = 11;
const int SHT_LOX2 = 12;

const int AIN1 = 6, AIN2 = 4, PWMA = 5;
const int BIN1 = 7, BIN2 = 8, PWMB = 9;

const float Kp = 4.5, Ki = 0.0, Kd = 1.2;
const int targetDist = 250;
const int baseSpeed = 150;
const int samplingFreq = 20;
const unsigned long samplingInterval = 1000UL / samplingFreq;

// ========================== Sensor Manager ==========================
class DualVL53L0X {
  Adafruit_VL53L0X lox1, lox2;
  VL53L0X_RangingMeasurementData_t measure1, measure2;
public:
  void init() {
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);

    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    delay(10);

    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    digitalWrite(SHT_LOX2, LOW); // keep LOX2 off
    if (!lox1.begin(LOX1_ADDRESS)) {
      Serial.println(F("Failed to boot LOX1")); while (1);
    }
    delay(10);

    digitalWrite(SHT_LOX2, HIGH);
    if (!lox2.begin(LOX2_ADDRESS)) {
      Serial.println(F("Failed to boot LOX2")); while (1);
    }
  }

  void read(int &d1, int &d2) {
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);
    d1 = (measure1.RangeStatus != 4) ? measure1.RangeMilliMeter : -1;
    d2 = (measure2.RangeStatus != 4) ? measure2.RangeMilliMeter : -1;
  }
};

// ========================== Motor Controller ==========================
class MotorDriver {
public:
  void init() {
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  }

  void drive(int speedR, int speedL) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, constrain(speedR, 0, 255));

    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, constrain(speedL, 0, 255));
  }
};

// ========================== PID Wall Follower ==========================
class WallFollower {
  float prevError = 0;
  float integral = 0;
  unsigned long prevTime = 0;
  MotorDriver &motor;

public:
  WallFollower(MotorDriver &m): motor(m) {}

  void update(int error) {
    unsigned long now = millis();
    float dt = (now - prevTime) / 1000.0;
    prevTime = now;

    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    float output = Kp * error + Ki * integral + Kd * derivative;
    int speedR = baseSpeed + output;
    int speedL = baseSpeed - output;

    Serial.print(speedR); Serial.print("  ");
    Serial.print(speedL); Serial.println(" <-- Motor Speeds");

    motor.drive(speedR, speedL);
  }

  void resetTimer() {
    prevTime = millis();
  }
};

// ========================== Main Control ==========================
DualVL53L0X sensor;
MotorDriver motor;
WallFollower controller(motor);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  motor.init();
  sensor.init();
  controller.resetTimer();
}

void loop() {
  static unsigned long lastSample = 0;
  unsigned long now = millis();

  if (now - lastSample >= samplingInterval) {
    lastSample = now;

    int dist1, dist2;
    sensor.read(dist1, dist2);

    if (dist1 > 0) {
      int error = targetDist - dist1;
      controller.update(error);
    }
  }
}
