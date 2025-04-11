#include <QTRSensors.h>

// Sensor setup
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[SensorCount] = {26, 25, 33, 32, 35, 34, 39, 36};

// PID constants
float Kp = 0.2;
float Kd = 0.1;
int baseSpeed = 100;
int maxSpeed = 160;
int lastError = 0;
int lastpos;

// Motor pin definitions (L298N Motor Driver)
const int IN1 = 27; // Left motor direction 1
const int IN2 = 14; // Left motor direction 2
const int ENA = 12; // Left motor PWM pin

const int IN3 = 15; // Right motor direction 1
const int IN4 = 4;  // Right motor direction 2
const int ENB = 13; // Right motor PWM pin

// ESP32 LEDC PWM configuration
const int pwmFreq = 5000;    // PWM frequency in Hz
const int pwmResolution = 8; // PWM resolution (8 bits: 0–255)
const int leftChannel = 0;   // PWM channel for left motor
const int rightChannel = 1;  // PWM channel for right motor

void setup()
{
  Serial.begin(115200);

  // Sensor and motor pin setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup ESP32 LEDC PWM channels for the motor enable pins
  ledcSetup(leftChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, leftChannel);

  ledcSetup(rightChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ENB, rightChannel);

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
  // 1) Grab raw calibrated values
  qtr.readCalibrated(sensorValues);

  while (lastpos <= 1000 || lastpos >= 6000)
  {
    if (lastpos >= 6000)
    {
      // Serial.print("line lost turn right");
      Serial.println("Line lost! spin right");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    else
    {
      Serial.println("Line lost! spin left");
      // Serial.print("line lost turn right");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    // Use LEDC in place of analogWrite for PWM outputs.
    ledcWrite(leftChannel, 160);
    ledcWrite(rightChannel, 160);

    qtr.readCalibrated(sensorValues);
    uint16_t position = readLine(sensorValues, lastpos);
    lastpos = position;

    delay(10);
  }

  // 3) Normal PID line‑following

  qtr.readCalibrated(sensorValues);
  // uint16_t position = qtr.readLineBlack(sensorValues);

  uint16_t position = readLine(sensorValues, lastpos);

  int error = (int)position - 3700;
  int dError = error - lastError;
  lastError = error;
  lastpos = position;

  int adjust = Kp * error + Kd * dError;
  int leftSpeed = constrain(baseSpeed - adjust, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed + adjust, 0, maxSpeed);

  // Set motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Use LEDC instead of analogWrite for PWM outputs.
  ledcWrite(leftChannel, leftSpeed);
  ledcWrite(rightChannel, rightSpeed);

  // debug
  for (int i = 0; i < 8; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial.print("Pos: ");
  Serial.print(position);
  Serial.print("  Speeds L/R: ");
  Serial.print(leftSpeed);
  Serial.print("/");
  Serial.println(rightSpeed);

  delay(50);
}

int getpos(uint16_t vals[8], int lastpos)
{
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 8; i++)
  {
    sum += 1000 * i * vals[i];
    count += vals[i];
  }
  if (count == 0)
  {
    return lastpos;
  }

  return sum / count;
}

// Read raw sensorValues[8], threshold at 1000, build a binary array,
// then call getpos() to compute line position.
int readLine(uint16_t sensorValues[8], int lastpos)
{
  uint16_t vals[8];
  for (int i = 0; i < 8; i++)
  {
    vals[i] = (sensorValues[i] < 1000) ? 0 : 1;
  }
  return getpos(vals, lastpos);
}