#include <CAN.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("CAN Simulation");

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  sendRSA1(); // 0x498
  delay(500);
  sendRSA2(); // 0x48A
  delay(1000);
}

void sendRSA1() {
  byte data[8] = {0};

  // Beispielwerte simulieren:
  byte TSGN1 = 130;       // z.B. Verkehrszeichenkennung
  byte SPDVAL1 = 80;      // Geschwindigkeit in km/h
  byte BZRRQ_P = 1;       // Buzzeranforderung Passiv
  byte SYNCID1 = 5;       // beliebiger Sync-Wert

  data[0] = TSGN1;        // TSGN1 (7|8)
  data[2] = SPDVAL1;      // SPDVAL1 (23|8)
  data[7] |= (BZRRQ_P & 0x03) << 0;    // BZRRQ_P (63|2)
  data[7] |= (SYNCID1 & 0x0F) << 2;    // SYNCID1 (59|4)

  CAN.beginPacket(0x498);
  for (int i = 0; i < 8; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();

  Serial.println("Sent RSA1 (0x498)");
}

void sendRSA2() {
  byte data[8] = {0};

  // Beispielwerte simulieren:
  byte TSGN3 = 131;
  byte SPDUNT = 1;        // km/h
  byte TSRWMSG = 2;       // Nachrichtentyp
  byte STEER_OVERRIDE = 1;
  byte STEER_INIT = 0;
  byte SYNCID2 = 7;

  data[0] |= (STEER_OVERRIDE & 0x01);           // Bit 0
  data[0] |= (STEER_INIT & 0x01) << 3;          // Bit 3
  data[0] |= TSGN3 << 0;                        // Byte 0, überdeckt andere -> optional verschieben

  data[0] = TSGN3;        // einfachere Simulation
  data[7] |= (SPDUNT & 0x03);                   // Bits 63|2
  data[7] |= (TSRWMSG & 0x03) << 2;             // Bits 61|2
  data[7] |= (SYNCID2 & 0x0F) << 4;             // Bits 59|4

  CAN.beginPacket(0x48A);
  for (int i = 0; i < 8; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();

  Serial.println("Sent RSA2 (0x48A)");
}
