#include <QTRSensors.h>

// Sensor setup
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[SensorCount] = {26, 25, 33, 32, 35, 34, 39, 36};

// PID constants
float Kp = 0.2;
float Kd = 0.1;
int baseSpeed = 80;
int maxSpeed = 160;
int lastError = 0;

// Motor pin definitions (L298N Motor Driver)
const int IN1 = 27; // Left motor direction 1
const int IN2 = 14; // Left motor direction 2
const int ENA = 12; // Left motor PWM

const int IN3 = 15; // Right motor direction 1
const int IN4 = 4;  // Right motor direction 2
const int ENB = 13; // Right motor PWM

void setup()
{
  Serial.begin(115200);

  // Sensor and motor pin setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(1000);

  Serial.println("Calibrating black...");
  for (uint8_t i = 0; i < 10; i++)
  { // Calibrate 10 times for example
    qtr.calibrate();
    delay(100); // Wait a bit for the sensors to stabilize
  }
  Serial.println("Calibrating white...");

  for (uint8_t i = 0; i < 10; i++)
  { // Calibrate 10 times for example
    qtr.calibrate();
    delay(100); // Wait a bit for the sensors to stabilize
  }
  Serial.println("Calibration complete.");
  delay(1000);
}

void loop()
{
  // Read position and sensor values
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Print raw sensor values
  Serial.print("Sensors: ");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();

  // PID error calculation
  int error = position - 3500;
  int motorSpeedAdjustment = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed = baseSpeed - motorSpeedAdjustment;
  int rightSpeed = baseSpeed + motorSpeedAdjustment;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, 0, maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, maxSpeed);

  // Set motor directions to forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Write speeds to motors
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // Print direction decision
  if (abs(leftSpeed - rightSpeed) < 10)
  {
    Serial.println("Direction: Forward");
  }
  else if (leftSpeed > rightSpeed)
  {
    Serial.println("Direction: left");
  }
  else
  {
    Serial.println("Direction: right");
  }

  delay(100);
}
