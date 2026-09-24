#include <Servo.h>

#define TRIG_PIN 7
#define ECHO_PIN 6
#define SERVO_PIN 9

Servo servo;

// =========================
// SERVO
// =========================

const int CENTER = 90;
const int MIN_ANGLE = 60;
const int MAX_ANGLE = 120;

// =========================
// TARGET
// =========================

float setpoint = 15.0;

// =========================
// PID
// =========================

float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.1;

float previousError = 0;
float integral = 0;

unsigned long previousTime;

// =========================
// ULTRASONIC
// =========================

float getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0)
    return -1;

  float distance = duration * 0.0343 / 2.0;

  if (distance < 5 || distance > 40)
    return -1;

  return distance;
}

// =========================
// SETUP
// =========================

void setup() {

  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);

  servo.write(CENTER);

  delay(1000);

  previousTime = millis();

  // MATLAB HEADER
  Serial.println("Time,Setpoint,Distance,Error,PID,ServoAngle");
}

// =========================
// LOOP
// =========================

void loop() {

  float distance = getDistance();

  if (distance < 0)
    return;

  unsigned long currentTime = millis();

  float dt =
      (currentTime - previousTime) / 1000.0;

  if (dt < 0.03)
    return;

  previousTime = currentTime;

  // =========================
  // ERROR
  // =========================

  float error = setpoint - distance;

  // =========================
  // INTEGRAL
  // =========================

  integral += error * dt;

  integral = constrain(integral, -10, 10);

  // =========================
  // DERIVATIVE
  // =========================

  float derivative =
      (error - previousError) / dt;

  previousError = error;

  // =========================
  // PID
  // =========================

  float pid =
      Kp * error +
      Ki * integral +
      Kd * derivative;

  // Limit PID output
  pid = constrain(pid, -25, 25);

  // =========================
  // SERVO
  // =========================

  int servoAngle =
      CENTER + pid;

  servoAngle =
      constrain(
        servoAngle,
        MIN_ANGLE,
        MAX_ANGLE
      );

  servo.write(servoAngle);

  // =========================
  // MATLAB DATA
  // =========================

  Serial.print(currentTime / 1000.0, 3);
  Serial.print(",");

  Serial.print(setpoint, 3);
  Serial.print(",");

  Serial.print(distance, 3);
  Serial.print(",");

  Serial.print(error, 3);
  Serial.print(",");

  Serial.print(pid, 3);
  Serial.print(",");

  Serial.println(servoAngle);

  delay(20);
}