#include <CAN.h>

// Toyota CAN Checksum
uint8_t can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("OpenPilot CAN Sender");

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

// Hilfsvariable für MAIN_ON Umschaltung
unsigned long lastMainOnToggle = 0;
bool mainOn = false;

void loop() {
  unsigned long now = millis();
  // MAIN_ON alle 5 Sekunden toggeln
  if (now - lastMainOnToggle >= 5000) {
    lastMainOnToggle = now;
    mainOn = !mainOn;
  }

  // PCM_CRUISE (0x1d2)
  {
    uint8_t data[8] = {0};
    // GAS_RELEASED (bit 4)
    data[0] |= (1 << 4); // 1 = released
    // CRUISE_ACTIVE (bit 5)
    data[0] |= (1 << 5); // 1 = active
    // ACC_BRAKING (bit 4 of byte 1)
    data[1] |= (0 << 4); // 0 = no braking
    // ACCEL_NET (byte 2/3, signed 16bit, -1.0...1.0 m/s²)
    int16_t accel = (int16_t)(0.5f / 0.0009765625f); // ca. 0.5 m/s²
    data[2] = (accel >> 8) & 0xFF;
    data[3] = (accel     ) & 0xFF;
    // NEUTRAL_FORCE (byte 4/5, signed 16bit)
    int16_t nforce = (int16_t)(0 / 2.0f); // 0N
    data[4] = (nforce >> 8) & 0xFF;
    data[5] = (nforce     ) & 0xFF;
    // CRUISE_STATE (bit 7-4 of byte 6)
    data[6] |= (2 << 4); // e.g. 2 = "enabled"
    // CANCEL_REQ (bit 1 of byte 6)
    data[6] |= (0 << 1); // 0 = kein Cancel
    // CHECKSUM (byte 7)
    data[7] = can_cksum(data, 7, 0x1d2);
    CAN.beginPacket(0x1D2);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PCM_CRUISE_2 (0x1d3)
  {
    uint8_t data[8] = {0};
    // BRAKE_PRESSED (bit 3)
    data[0] |= (0 << 3);
    // PCM_FOLLOW_DISTANCE (bit 5-4 of byte 1)
    data[1] |= (2 << 4); // z.B. 2 = Mittel
    // LOW_SPEED_LOCKOUT (bit 7-6 of byte 1)
    data[1] |= (0 << 6);
    // MAIN_ON (bit 7 of byte 1) - toggelt alle 5 Sekunden
    data[1] |= ((mainOn ? 1 : 0) << 7); // <<<< Umschaltung von MAIN_ON
    // SET_SPEED (byte 2)
    data[2] = 100; // 100 km/h
    // ACC_FAULTED (bit 7 of byte 5)
    data[5] |= (0 << 7);
    // CHECKSUM (byte 7)
    data[7] = can_cksum(data, 7, 0x1d3);
    CAN.beginPacket(0x1D3);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // WHEEL_SPEEDS (0xaa)
  {
    uint8_t data[8] = {0};
    uint16_t wspeed = (uint16_t)((60 + 67.67) / 0.01); // 60 km/h
    data[0] = (wspeed >> 8) & 0xFF; data[1] = wspeed & 0xFF; // FR
    data[2] = (wspeed >> 8) & 0xFF; data[3] = wspeed & 0xFF; // FL
    data[4] = (wspeed >> 8) & 0xFF; data[5] = wspeed & 0xFF; // RR
    data[6] = (wspeed >> 8) & 0xFF; data[7] = wspeed & 0xFF; // RL
    CAN.beginPacket(0xAA);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BLINKERS_STATE (0x614)
  {
    uint8_t data[8] = {0};
    data[1] |= (0 << 7);
    data[3] |= (0 << 3);
    data[3] |= (1 << 4); // 1 = links
    CAN.beginPacket(0x614);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BODY_CONTROL_STATE (0x620)
  {
    uint8_t data[8] = {0};
    data[4] |= (0 << 6);
    data[7] |= (0 << 4);
    data[7] |= (0 << 6);
    data[5] |= (0 << 5);
    data[5] |= (0 << 2);
    data[5] |= (0 << 3);
    data[5] |= (0 << 4);
    CAN.beginPacket(0x620);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // GEAR_PACKET (0x3bc)
  {
    uint8_t data[8] = {0};
    data[0] |= (0 << 2);
    data[1] |= (4 & 0x3F); // 4 = Drive
    data[4] |= (0 << 1);
    data[4] |= (0 << 4);
    data[5] |= (0 << 0);
    data[5] |= (0 << 1);
    data[5] |= (1 << 7);
    CAN.beginPacket(0x3BC);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // ESP_CONTROL (0x3b7)
  {
    uint8_t data[8] = {0};
    data[1] |= (0 << 5);
    data[1] |= (0 << 4);
    data[2] |= (0 << 2);
    data[4] |= (0 << 0);
    data[4] |= (0 << 4);
    CAN.beginPacket(0x3B7);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // STEER_ANGLE_SENSOR (0x25)
  {
    uint8_t data[8] = {0};
    int16_t angle = (int16_t)(10 / 1.5); // 10° nach rechts
    data[0] = (angle >> 4) & 0xFF;
    data[1] = ((angle & 0x0F) << 4);
    int16_t rate = (int16_t)(100 / 1.0); // 100 deg/s
    data[4] = (rate >> 4) & 0xFF;
    data[5] = ((rate & 0x0F) << 4);
    int8_t fraction = (int8_t)(0.2 / 0.1); // 0.2°
    data[4] |= ((fraction & 0x0F) << 4);
    CAN.beginPacket(0x25);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // STEER_TORQUE_SENSOR (0x260)
  {
    uint8_t data[8] = {0};
    int16_t eps = (int16_t)(100); // 100 Nm
    data[5] = (eps >> 8) & 0xFF; data[6] = eps & 0xFF;
    int16_t driver = (int16_t)(5); // 5 Nm
    data[1] = (driver >> 8) & 0xFF; data[2] = driver & 0xFF;
    int16_t angle = (int16_t)(20 / 0.0573); // 20°
    data[3] = (angle >> 8) & 0xFF; data[4] = angle & 0xFF;
    data[0] |= (0 << 3);
    data[0] |= (0 << 0);
    data[7] = can_cksum(data, 7, 0x260);
    CAN.beginPacket(0x260);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // EPS_STATUS (0x262)
  {
    uint8_t data[5] = {0};
    data[0] |= (0 & 0x0F);
    data[3] |= (1 << 1);
    data[3] |= (0 << 0);
    data[4] = can_cksum(data, 4, 0x262);
    CAN.beginPacket(0x262);
    for (int i = 0; i < 5; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  delay(100); // 10Hz Update-Rate (anpassbar)
}
