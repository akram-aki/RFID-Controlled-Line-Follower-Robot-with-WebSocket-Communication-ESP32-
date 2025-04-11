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
int lastpos;
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
  // 1) Grab raw calibrated values
  qtr.read(sensorValues);

  for (int i = 0; i < 8; i++)
  {
    if (sensorValues[i] < 3500)
    {
      sensorValues[i] = 0;
    }
    else
    {
      sensorValues[i] = 1000;
    }
  }
  Serial.println("");
  // 2) Lost‑line detection
  while (
      sensorValues[0] == sensorValues[1] &&
      sensorValues[1] == sensorValues[2] &&
      sensorValues[2] == sensorValues[3] &&
      sensorValues[3] == sensorValues[4] &&
      sensorValues[4] == sensorValues[5] &&
      sensorValues[5] == sensorValues[6] &&
      sensorValues[6] == sensorValues[7])
  {
    bool spinCW = (lastError > 0); // spin direction based on last error
    // Serial.print(lastError);

    // spin in place until ANY sensor sees the line again
    if (lastpos > 3700)
    {
      Serial.println("Line lost! spin right");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    else
    {
      Serial.println("Line lost! spin left");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
    analogWrite(ENA, 100);
    analogWrite(ENB, 100);
    qtr.read(sensorValues);
    for (int i = 0; i < 8; i++)
    {
      if (sensorValues[i] < 3500)
      {
        sensorValues[i] = 0;
      }
      else
      {
        sensorValues[i] = 1000;
      }
    }

    delay(10);
  }

  // 3) Normal PID line‑following
  // Serial.println("Line found, resuming PID.");
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = (int)position - 3700;
  int dError = error - lastError;
  lastError = error;
  lastpos = position;

  int adjust = Kp * error + Kd * dError;
  int leftSpeed = constrain(baseSpeed - adjust, 0, maxSpeed);
  int rightSpeed = constrain(baseSpeed + adjust, 0, maxSpeed);

  // both motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  // debug
  Serial.print("Pos: ");
  Serial.print(position);
  // Serial.print("  Sum: "); Serial.print(sum);
  Serial.print("  Speeds L/R: ");
  Serial.print(leftSpeed);
  Serial.print("/");
  Serial.println(rightSpeed);

  delay(50);
}
