float d_current = 0.0;
float d_desired = 0.3;

float Kp = 200.0;
float Kd = 150.0;

float last_error = 0.0;
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);
  setupMotors(); // Your motor setup here
  last_time = millis();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int commaIndex = input.indexOf(',');
    if (commaIndex > 0) {
      d_current = input.substring(0, commaIndex).toFloat();
      d_desired = input.substring(commaIndex + 1).toFloat();
    }
  }

  float error = d_desired - d_current;

  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  float derivative = (dt > 0) ? (error - last_error) / dt : 0;

  float control = Kp * error + Kd * derivative;
  last_error = error;
  last_time = now;

  apply_control(control); // Turns based on control signal
}

void apply_control(float ctrl) {
  int base_speed = 100;
  int left_speed = base_speed - ctrl;
  int right_speed = base_speed + ctrl;

  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  analogWrite(leftMotorPWM, left_speed);
  analogWrite(rightMotorPWM, right_speed);
}
