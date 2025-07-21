#include <CAN.h>

// --- CAN IDs ---
const uint16_t RSA1_ID = 0x1161;
const uint16_t RSA2_ID = 0x1162;

// --- Timer ---
unsigned long lastSendTime = 0;
const unsigned long interval = 100; // Send interval in milliseconds

void setup() {
  Serial.begin(9600);

  // Initialize CAN communication at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.println("CAN RSA Simulator Initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Send RSA1 and RSA2 periodically
  if (currentMillis - lastSendTime >= interval) {
    sendRSA1();
    sendRSA2();
    lastSendTime = currentMillis;
  }
}

void sendRSA1() {
  uint8_t data[8] = {0};

  // Set example values for RSA1 signals
  data[0] = 0x12; // TSGN1
  data[1] = 0x00; // Reserved
  data[2] = 0x34; // TSGNHLT1 and TSGNGRY1
  data[3] = 0x56; // SPDVAL1
  data[4] = 0x78; // SPLSGN2 and SPLSGN1
  data[5] = 0x9A; // TSGN2
  data[6] = 0xBC; // TSGNHLT2 and TSGNGRY2
  data[7] = 0xDE; // SPDVAL2

  // Send the CAN message
  CAN.beginPacket(RSA1_ID);
  for (int i = 0; i < 8; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();

  Serial.println("RSA1 message sent.");
}

void sendRSA2() {
  uint8_t data[8] = {0};

  // Set example values for RSA2 signals
  data[0] = 0x11; // TSGN3
  data[1] = 0x00; // Reserved
  data[2] = 0x22; // TSGNHLT3 and TSGNGRY3
  data[3] = 0x44; // SPLSGN4 and SPLSGN3
  data[4] = 0x66; // TSGN4
  data[5] = 0x88; // TSGNHLT4 and TSGNGRY4
  data[6] = 0xAA; // SGNNUMA and SGNNUMP
  data[7] = 0xCC; // SPDUNT and TSRWMSG

  // Send the CAN message
  CAN.beginPacket(RSA2_ID);
  for (int i = 0; i < 8; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();

  Serial.println("RSA2 message sent.");
}
