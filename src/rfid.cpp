#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 5   // SDA
#define RST_PIN 22 // RST

MFRC522 mfrc522(SS_PIN, RST_PIN);

// Buffer to hold scanned UID
byte scannedUID[10]; // large enough for longest UIDs
byte scannedUIDLength = 0;

// Hardcoded target UID
byte targetUID[] = {0x04, 0xA2, 0x3F, 0x6C};
byte targetUIDLength = sizeof(targetUID);

void setup()
{
    Serial.begin(115200);
    SPI.begin();
    mfrc522.PCD_Init();
    Serial.println("Ready to scan RFID tags...");
}

void loop()
{
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial())
    {
        return;
    }

    // Copy UID to scannedUID
    scannedUIDLength = mfrc522.uid.size;
    memcpy(scannedUID, mfrc522.uid.uidByte, scannedUIDLength);

    // Print scanned UID
    Serial.print("Scanned UID: ");
    for (byte i = 0; i < scannedUIDLength; i++)
    {
        Serial.print(scannedUID[i] < 0x10 ? " 0" : " ");
        Serial.print(scannedUID[i], HEX);
    }
    Serial.println();

    // Compare scannedUID to targetUID
    if (scannedUIDLength == targetUIDLength && memcmp(scannedUID, targetUID, targetUIDLength) == 0)
    {
        Serial.println("✅ Match found. Executing deloadObject...");
        deloadObject();

        // Clear scannedUID
        memset(scannedUID, 0, sizeof(scannedUID));
        scannedUIDLength = 0;
    }

    mfrc522.PICC_HaltA();
    mfrc522.PCD_StopCrypto1();
}

// Dummy function for demonstration
void deloadObject()
{
    Serial.println("🦾 Object deloaded!");
}
