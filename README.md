# PID BASED CONTROLLER 

The following project is a Ball and Beam System Based PID controller.

## Code

Click the following link to directly download the file from [drive](https://drive.google.com/drive/folders/1Mt04hyl1pCcOXk2DZd40gOuwkiQ3Hsvm?usp=sharing) & upload to your arduino.

```bash
idk do something
```

## Build Quality

This shi just balances the ball.
![Ball and Beam system architecture](ballandbeam.png)

## 😝 YouTube Video

Watch this [video](https://youtu.be/h513nVM-t9g) !!

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

And I am newbie soo any sort of suggestions are welcome

## Code

Copy paste this code if you are lazy f.

```bash
#include <Servo.h>

// =====================================================
// PIN CONFIGURATION
// =====================================================

#define TRIG_PIN 7
#define ECHO_PIN 6
#define SERVO_PIN 9

Servo beamServo;

// =====================================================
// PID PARAMETERS
// =====================================================

// START WITH THESE VALUES
float Kp = 2.5;
float Ki = 0.00;
float Kd = 1.2;

// =====================================================
// BALL POSITION
// =====================================================

// Desired ball position in cm
// Change this according to your beam
float setpoint = 15.0;

// =====================================================
// SERVO SETTINGS
// =====================================================

int servoCenter = 90;

// Mechanical limits of your beam
int servoMin = 45;
int servoMax = 135;

// =====================================================
// PID VARIABLES
// =====================================================

float error = 0;
float previousError = 0;

float integral = 0;
float derivative = 0;

float pidOutput = 0;

unsigned long previousTime = 0;

// Integral protection
float integralLimit = 100;

// =====================================================
// HC-SR04 FUNCTION
// =====================================================

float readDistance()
{
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);

  // Read echo
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // If no echo
  if (duration == 0)
  {
    return -1;
  }

  // Convert time to distance
  float distance = duration * 0.0343 / 2.0;

  return distance;
}

// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Start servo at center
  beamServo.write(servoCenter);
  beamServo.attach(SERVO_PIN);

  delay(1000);

  beamServo.write(servoCenter);

  previousTime = millis();

  Serial.println("=================================");
  Serial.println("BALL AND BEAM PID CONTROLLER");
  Serial.println("=================================");
  Serial.println("System starting...");
  Serial.println();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  // ---------------------------------------------------
  // READ BALL POSITION
  // ---------------------------------------------------

  float distance = readDistance();

  // Ignore invalid measurement
  if (distance < 0)
  {
    return;
  }

  // ---------------------------------------------------
  // SIMPLE LOW-PASS FILTER
  // ---------------------------------------------------

  static float filteredDistance = distance;

  filteredDistance =
      0.7 * filteredDistance +
      0.3 * distance;

  float position = filteredDistance;

  // ---------------------------------------------------
  // TIME CALCULATION
  // ---------------------------------------------------

  unsigned long currentTime = millis();

  float dt =
      (currentTime - previousTime) / 1000.0;

  // Prevent division problems
  if (dt <= 0)
  {
    dt = 0.001;
  }

  previousTime = currentTime;

  // ---------------------------------------------------
  // ERROR
  // ---------------------------------------------------

  error = setpoint - position;

  // ---------------------------------------------------
  // INTEGRAL
  // ---------------------------------------------------

  integral += error * dt;

  // Anti-windup
  if (integral > integralLimit)
    integral = integralLimit;

  if (integral < -integralLimit)
    integral = -integralLimit;

  // ---------------------------------------------------
  // DERIVATIVE
  // ---------------------------------------------------

  derivative =
      (error - previousError) / dt;

  // ---------------------------------------------------
  // PID EQUATION
  // ---------------------------------------------------

  pidOutput =
      (Kp * error) +
      (Ki * integral) +
      (Kd * derivative);

  previousError = error;

  // ---------------------------------------------------
  // SERVO COMMAND
  // ---------------------------------------------------

  int servoAngle =
      servoCenter + (int)pidOutput;

  // Limit servo angle
  servoAngle =
      constrain(
          servoAngle,
          servoMin,
          servoMax
      );

  // Move servo
  beamServo.write(servoAngle);

  // ---------------------------------------------------
  // SERIAL MONITOR
  // ---------------------------------------------------

  Serial.print("Position: ");
  Serial.print(position);

  Serial.print(" cm | Setpoint: ");
  Serial.print(setpoint);

  Serial.print(" cm | Error: ");
  Serial.print(error);

  Serial.print(" | PID: ");
  Serial.print(pidOutput);

  Serial.print(" | Servo: ");
  Serial.println(servoAngle);

  // ---------------------------------------------------
  // LOOP SPEED
  // ---------------------------------------------------

  delay(20);
}
```

## License

Idk i have a driving license 
