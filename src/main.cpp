#include <WiFi.h>
#include <WebSocketsClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <QTRSensors.h>
#include "driver/ledc.h" // Needed for ledcSetup(), ledcAttachPin(), etc.

int state = 1;

//====================
// Global Variables for RFID Trigger
//====================
// Flag to indicate that the RFID card has been set for starting the line follower.
volatile bool rfidInitialSet = false;
// Holds the initial/stored RFID UID.
String storedRFID = "";
// Flag to indicate that a matching RFID scan was detected to pause line following.
volatile bool stopLineFollowing = false;

//====================
// RFID / WebSocket Setup (Core 0)
//====================

// RFID module pins
#define SS_PIN 5   // RFID SDA pin
#define RST_PIN 22 // RFID RST pin
MFRC522 mfrc522(SS_PIN, RST_PIN);

// WiFi network credentials
const char *ssid = "TIM CHEESE";
const char *password = ""; // open network

// Create a WebSocket client instance
WebSocketsClient webSocket;

// WebSocket event callback function
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_CONNECTED:
    Serial.println("WebSocket connected");
    break;
  case WStype_DISCONNECTED:
    Serial.println("WebSocket disconnected");
    break;
  case WStype_TEXT:
    Serial.printf("Received: %s\n", payload);
    break;
  case WStype_ERROR:
    Serial.println("WebSocket error");
    break;
  default:
    break;
  }
}

// Task handling WiFi, WebSocket, and RFID reading on Core 0
void TaskWebSocketRFID(void *parameter)
{
  // Initialize Serial for debugging (if not already initialized)
  Serial.begin(115200);
  delay(1000);

  // Connect to WiFi
  Serial.printf("Connecting to WiFi network: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // Start the WebSocket connection
  webSocket.begin("192.168.24.111", 8080, "/");
  webSocket.onEvent(webSocketEvent);

  // Initialize SPI and RFID reader
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID reader initialized. Ready to scan...");

  // Main loop for this task
  for (;;)
  {
    // Keep the WebSocket connection alive
    webSocket.loop();

    // Check if a new RFID card is present
    if (mfrc522.PICC_IsNewCardPresent())
    {
      if (mfrc522.PICC_ReadCardSerial())
      {
        // Build the UID string in hexadecimal format
        String uidStr = "";
        for (byte i = 0; i < mfrc522.uid.size; i++)
        {
          if (mfrc522.uid.uidByte[i] < 0x10)
            uidStr += "0";
          uidStr += String(mfrc522.uid.uidByte[i], HEX);
        }
        Serial.print("Scanned UID: ");
        Serial.println(uidStr);

        // First time: store the UID and signal line follower to begin
        if (!rfidInitialSet)
        {
          storedRFID = uidStr;
          rfidInitialSet = true;
          Serial.print("Stored UID for run: ");
          Serial.println(storedRFID);
        }
        // If line following already started and the same UID is scanned, flag to stop.
        else if (uidStr == storedRFID)
        {
          stopLineFollowing = true;
          Serial.println("Matching UID scanned. Signaling to pause line following for 5 seconds.");
        }

        // Create the JSON payload with the scanned UID and send it via WebSocket.
        String jsonPayload = String("{ \"currentRFID\": \"") + uidStr + "\" }";
        webSocket.sendTXT(jsonPayload);
        Serial.print("Sent UID via WebSocket: ");
        Serial.println(jsonPayload);

        // Halt further processing of this card until it is removed
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
      }
    }

    delay(10);
  }
}

//====================
// Line Following Setup (Core 1)
//====================

// Sensor and motor definitions
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPins[SensorCount] = {36, 39, 34, 35, 32, 33, 25, 26};

// PID constants and variables
float Kp = 0.05;
float Kd = 0.03;
int baseSpeed = 200;
int maxSpeed = 255;
int lastError = 0;
int lastpos = 3500; // starting point

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

// Utility function: Compute position from sensor values
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

// Utility function: Process raw sensor values to a binary threshold and compute position
int readLine(uint16_t sensorValues[8], int lastpos)
{
  uint16_t vals[8];
  for (int i = 0; i < 8; i++)
  {
    vals[i] = (sensorValues[i] < 1000) ? 0 : 1;
  }
  return getpos(vals, lastpos);
}

// Task handling line following on Core 1
void TaskLineFollow(void *parameter)
{
  // Initialize Serial for debugging (if needed)
  Serial.begin(115200);
  delay(1000);

  // Wait until an RFID card has been scanned and stored before starting line following.
  while (!rfidInitialSet)
  {
    Serial.println("Waiting for RFID card to start line following...");
    delay(500);
  }
  Serial.println("RFID UID set. Starting line following.");

  // Sensor and motor pin setup
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Setup LEDC PWM channels
  ledcSetup(leftChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ENA, leftChannel);
  ledcSetup(rightChannel, pwmFreq, pwmResolution);
  ledcAttachPin(ENB, rightChannel);

  delay(1000);
  Serial.println("Calibrating sensors...");

  // Calibrate sensors
  for (uint8_t i = 0; i < 10; i++)
  {
    qtr.calibrate();
    delay(100);
  }
  Serial.println("Calibration complete.");
  delay(1000);

  // Main loop for line following
  for (;;)
  {
    // Check if RFID stop signal is triggered.
    if (stopLineFollowing)
    {
      Serial.println("Pausing line following for 5 seconds due to RFID signal...");
      // Stop motors
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      ledcWrite(leftChannel, 0);
      ledcWrite(rightChannel, 0);
      digitalWrite(16, 1);
      delay(30000); // pause for 5 seconds
      digitalWrite(16, 0);
      stopLineFollowing = false; // reset the flag
      Serial.println("Resuming line following.");
    }

    // 1) Read raw calibrated sensor values
    qtr.readCalibrated(sensorValues);

    // If line is lost: take action until a valid position is read
    while (lastpos <= 1000 || lastpos >= 6500)
    {
      if (lastpos >= 6500)
      {
        Serial.println("Line lost! Spin right");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
      else
      {
        Serial.println("Line lost! Spin left");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }
      // Keep motors at a preset speed during spin
      ledcWrite(leftChannel, 160);
      ledcWrite(rightChannel, 160);

      qtr.readCalibrated(sensorValues);
      uint16_t position = readLine(sensorValues, lastpos);
      lastpos = position;
      delay(10);
    }

    qtr.readCalibrated(sensorValues);
    while (
        sensorValues[0] < 1000 &&
        sensorValues[1] < 1000 &&
        sensorValues[2] < 1000 &&
        sensorValues[3] < 1000 &&
        sensorValues[4] < 1000 &&
        sensorValues[5] < 1000 &&
        sensorValues[6] < 1000 &&
        sensorValues[7] < 1000)
    {
      if (lastpos < 3500)
      {
        Serial.println("Line lost! Spin left");
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }
      else
      {
        Serial.println("Line lost! Spin right");
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
      // Keep motors at a preset speed during spin
      ledcWrite(leftChannel, 160);
      ledcWrite(rightChannel, 160);

      qtr.readCalibrated(sensorValues);
      uint16_t position = readLine(sensorValues, lastpos);
      lastpos = position;
      delay(10);
    }

    // 2) Normal PID line‑following
    uint16_t position = readLine(sensorValues, lastpos);
    int error = (int)position - 3500;
    int dError = error - lastError;
    lastError = error;
    lastpos = position;
    int adjust = Kp * error + Kd * dError;
    int leftSpeed = constrain(baseSpeed + adjust, 0, maxSpeed);
    int rightSpeed = constrain(baseSpeed - adjust, 0, maxSpeed);

    // Set motors forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    // Adjust motor speeds via PWM
    ledcWrite(leftChannel, leftSpeed);
    ledcWrite(rightChannel, rightSpeed);

    // Debug: print sensor values and computed speeds
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
}

//====================
// Setup and Main Loop
//====================
void setup()
{
  // Create the RFID & WebSocket task on Core 0
  xTaskCreatePinnedToCore(
      TaskWebSocketRFID,   // Task function
      "TaskWebSocketRFID", // Name of task
      15000,               // Stack size in words
      NULL,                // Parameter
      1,                   // Priority
      NULL,                // Task handle
      0                    // Pin to core 0
  );

  // Create the Line Following task on Core 1
  xTaskCreatePinnedToCore(
      TaskLineFollow,   // Task function
      "TaskLineFollow", // Name of task
      15000,            // Stack size in words
      NULL,             // Parameter
      1,                // Priority
      NULL,             // Task handle
      1                 // Pin to core 1
  );
}

void loop()
{
  // Empty. Tasks are running on both cores.
  delay(10000);
}
