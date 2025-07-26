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
    // CRUISE_ACTIVE (bit 5)
    // Beide richten sich nach mainOn:
    data[0] |= ((mainOn ? 0 : 1) << 4); // GAS_RELEASED: false (0) wenn mainOn==true, true (1) wenn mainOn==false
    data[0] |= ((mainOn ? 1 : 0) << 5); // CRUISE_ACTIVE: true (1) wenn mainOn==true, false (0) wenn mainOn==false
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

 // 0x3bc msg GEAR_PACKET
uint8_t dat_3bc[8] = {0}; // Initialisiere alle Bytes mit 0

// SPORT_ON (Bit 2)
dat_3bc[0] |= (1 << 2);

// GEAR (Bits 13-18, also Byte 1, Bit 5 bis Byte 2, Bit 0, 6 Bit)
dat_3bc[1] |= ((4 & 0x3F) << 5);       // lower 3 Bit in byte 1, upper 3 Bit in byte 2
dat_3bc[2] |= ((4 & 0x3F) >> 3);       // restliche bits in byte 2

// SPORT_GEAR_ON (Bit 33 = Byte 4, Bit 1)
dat_3bc[4] |= (1 << 1);

// SPORT_GEAR (Bits 38–40 = Byte 4, Bit 6-7 + Byte 5, Bit 0)
dat_3bc[4] |= ((1 & 0x03) << 6);       // Bit 6-7 in Byte 4
dat_3bc[5] |= ((1 & 0x04) >> 2);       // Bit 0 in Byte 5

// ECON_ON (Bit 40 = Byte 5, Bit 0), Wert bleibt 0 -> nichts tun

// B_GEAR_ENGAGED (Bit 41 = Byte 5, Bit 1), Wert bleibt 0 -> nichts tun

// DRIVE_ENGAGED (Bit 47 = Byte 5, Bit 7)
dat_3bc[5] |= (1 << 7);

CAN.beginPacket(0x3bc);
for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3bc[ii]);
}
CAN.endPacket();

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

  // LIGHT_STALK (0x622)
  {
    uint8_t data[8] = {0};
    // AUTO_HIGH_BEAM : Bit 5 von Byte 4 (Signalstart: 37)
    data[4] |= (0 << 5); // aus
    // FRONT_FOG : Bit 3 von Byte 3 (27)
    data[3] |= (0 << 3); // aus
    // PARKING_LIGHT : Bit 4 von Byte 3 (28)
    data[3] |= (1 << 4); // an
    // LOW_BEAM : Bit 5 von Byte 3 (29)
    data[3] |= (1 << 5); // an
    // HIGH_BEAM : Bit 6 von Byte 3 (30)
    data[3] |= (0 << 6); // aus
    // DAYTIME_RUNNING_LIGHT : Bit 7 von Byte 3 (31)
    data[3] |= (1 << 7); // an
    CAN.beginPacket(0x622);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PRE_COLLISION (0x283)
  {
    static uint8_t counter = 0;
    uint8_t data[7] = {0};

    // _COUNTER : Byte 0 (7|8)
    data[0] = counter++;
    if (counter > 250) counter = 0;

    // SET_ME_X00 : Byte 1 (15|8)
    data[1] = 0x00; // typischer Wert laut DBC

    // FORCE : Byte 2 und 3 (23|16@0-)
    int16_t force = 120; // z.B. 120N, moderates Bremsmoment bei Fahrt
    data[2] = (force >> 8) & 0xFF;
    data[3] = (force     ) & 0xFF;

    // SET_ME_X002 : Byte 4 (33|8)
    data[4] = 0; // typischer Wert

    // BRAKE_STATUS : Bits 7-5 von Byte 4 (39|3)
    data[4] |= (0 << 5); // keine starke Bremsanforderung

    // STATE : Bits 4-2 von Byte 4 (36|3)
    data[4] |= (1 << 2); // Status: 1 = aktiv

    // SET_ME_X003 : Bit 0 von Byte 5 (40)
    // PRECOLLISION_ACTIVE : Bit 1 von Byte 5 (41)
    data[5] |= (0 << 0); // nicht aktiv
    data[5] |= (0 << 1); // nicht aktiv

    // CHECKSUM : Byte 6 (55|8)
    data[6] = can_cksum(data, 6, 0x283);

    CAN.beginPacket(0x283);
    for (int i = 0; i < 7; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BRAKE_MODULE (0x224)
  {
    uint8_t data[8] = {0};
    // BRAKE_PRESSURE: 43|12@0+ → Wert auf 0 setzen (kein Druck)
    uint16_t brake_pressure = 0;
    data[5] |= (brake_pressure & 0xFF) << 3;
    data[6] |= (brake_pressure >> 5) & 0x7F;
    // BRAKE_PRESSED: 5|1@0+ → Wert auf 0 setzen (Pedal nicht gedrückt)
    data[0] |= (0 << 5);

    CAN.beginPacket(0x224);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BODY_CONTROL_STATE_2 (0x610)
  {
    uint8_t data[8] = {0};
    // UI_SPEED: 23|8@0+ → Byte 2 Bit 7 bis Byte 3 Bit 0 (startbit 23, 8 bit, little endian)
    uint8_t speed = 100; // 100 km/h
    data[2] |= (speed << 7);         // niederwertiges Bit an Bit 7 in Byte 2 (little endian)
    data[3] |= (speed >> 1);         // Rest auf Byte 3

    // METER_SLIDER_BRIGHTNESS_PCT: 30|7@0+ → Byte 3 Bit 6 bis Bit 0
    uint8_t brightness = 80; // 80 %
    data[3] |= (brightness & 0x7F) << 6;      // 7 bit, an Bit 6 in Byte 3 (little endian)
    data[4] |= (brightness >> 1);             // Fortsetzung auf Byte 4

    // METER_SLIDER_LOW_BRIGHTNESS: 37|1@0+ → Byte 4 Bit 5
    data[4] |= (0 << 5); // aus

    // METER_SLIDER_DIMMED: 38|1@0+ → Byte 4 Bit 6
    data[4] |= (0 << 6); // aus

    // UNITS: 63|3@0+ → Byte 7 Bit 7 bis Bit 5
    data[7] |= (1 << 5); // 1 = km/h

    CAN.beginPacket(0x610);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

    // PCM_CRUISE_SM (0x399)
  {
    uint8_t data[8] = {0};
    // MAIN_ON : 4|1@0+ (Byte 0 Bit 4)
    data[0] |= (mainOn ? 1 : 0) << 4;
    // CRUISE_CONTROL_STATE : 11|4@0+ (Byte 1 Bit 3-0)
    data[1] |= (2 << 0); // z.B. 2 = enabled
    // DISTANCE_LINES : 14|2@0+ (Byte 1 Bit 6-5)
    data[1] |= (2 << 5); // 2 = mittlerer Abstand
    // TEMP_ACC_FAULTED : 15|1@0+ (Byte 1 Bit 7)
    data[1] |= (0 << 7); // kein Fehler
    // UI_SET_SPEED : 31|8@0+ (Byte 3)
    data[3] = 100; // 100 km/h
    CAN.beginPacket(0x399);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // ENGINE_RPM (0x1C4)
  {
    uint8_t data[8] = {0};
    // RPM : 7|16@0- (Byte 0 Bit 7 bis Byte 1 Bit 0, little endian, signed)
    // Beispiel: 2200 rpm, DBC: phys = raw * 0.78125 -> raw = phys / 0.78125
    int16_t rpm_raw = (int16_t)(2200.0 / 0.78125);
    data[0] = rpm_raw & 0xFF;
    data[1] = (rpm_raw >> 8) & 0xFF;
    // ENGINE_RUNNING : 27|1@0+ (Byte 3 Bit 3)
    data[3] |= (1 << 3); // Motor läuft
    CAN.beginPacket(0x1C4);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();


      // VSC1S07 (0x320)
  {
    uint8_t data[8] = {0};
    // Alle Schalter (FBKRLY, FVSCM, FVSCSFT, FABS, TSVSC, FVSCL): alle aus (0)
    // Byte 0: Bit 6-0
    // (Hier alles 0 – Default)
    // RQCSTBKB, PSBSTBY, P2BRXMK, MCC, RQBKB, BRSTOP, BRKON: alle aus (0)
    // Byte 1: Bit 7-0 (ebenfalls alle 0)
    // ASLP: 23|8@0- (Byte 2), Wert 0
    data[2] = 0;
    // BRKABT3,2,1: 26,25,24 (Byte 3, Bit 2-0), alle 0
    // BRTYPACC: 31|2 (Byte 3, Bit 7-6), 0
    // GVC: 39|8@0- (Byte 4), Wert 0
    data[4] = 0;
    // XGVCINV: 43|1 (Byte 5, Bit 3), 0
    // PCSBRSTA: 50|2 (Byte 6, Bit 2-1), 0
    // S07CNT: 52|1 (Byte 6, Bit 4), 0
    // VSC07SUM: 63|8 (Byte 7), 0
    // → Alle Felder Default (0), keine Flags, keine Fehler, keine Eingriffe
    CAN.beginPacket(0x320);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // LKAS_HUD (0x412)
  {
    uint8_t data[8] = {0};
    // BARRIERS: 1|2@0+ (Byte 0, Bit 1-0)
    data[0] |= (0 << 0); // Keine Barrieren
    // RIGHT_LINE: 3|2@0+ (Byte 0, Bit 3-2)
    data[0] |= (2 << 2); // Rechter Fahrstreifen erkannt
    // LEFT_LINE: 5|2@0+ (Byte 0, Bit 5-4)
    data[0] |= (2 << 4); // Linker Fahrstreifen erkannt
    // LKAS_STATUS: 7|2@0+ (Byte 0, Bit 7-6)
    data[0] |= (1 << 6); // LKAS aktiv
    // LDA_ALERT: 9|2@0+ (Byte 1, Bit 1-0)
    data[1] |= (0 << 0); // kein Alert
    // LDW_EXIST: 10|1@0+ (Byte 1, Bit 2)
    data[1] |= (1 << 2); // LDW existiert
    // TWO_BEEPS: 12|1@0+ (Byte 1, Bit 4)
    data[1] |= (0 << 4); // kein Signalton
    // ADJUSTING_CAMERA: 13|1@0+ (Byte 1, Bit 5)
    data[1] |= (0 << 5); // keine Kalibrierung
    // LDA_UNAVAILABLE_QUIET: 14|1@0+ (Byte 1, Bit 6)
    data[1] |= (0 << 6);
    // LDA_MALFUNCTION: 15|1@0+ (Byte 1, Bit 7)
    data[1] |= (0 << 7);
    // LDA_UNAVAILABLE: 16|1@0+ (Byte 2, Bit 0)
    data[2] |= (0 << 0);
    // LDA_SENSITIVITY: 18|2@0+ (Byte 2, Bit 2-1)
    data[2] |= (1 << 1); // mittlere Empfindlichkeit
    // LDA_SA_TOGGLE: 20|2@0+ (Byte 2, Bit 4-3)
    data[2] |= (1 << 3); // an
    // LDA_MESSAGES: 23|3@0+ (Byte 2, Bit 7-5)
    data[2] |= (0 << 5);
    // LDA_ON_MESSAGE: 31|2@0+ (Byte 3, Bit 7-6)
    data[3] |= (1 << 6); // LKAS auf HUD an
    // REPEATED_BEEPS: 32|1@0+ (Byte 4, Bit 0)
    data[4] |= (0 << 0);
    // LANE_SWAY_TOGGLE: 43|1@0+ (Byte 5, Bit 3)
    data[5] |= (0 << 3);
    // LANE_SWAY_SENSITIVITY: 45|2@0+ (Byte 5, Bit 5-4)
    data[5] |= (1 << 4); // Mittel
    // TAKE_CONTROL: 46|1@0+ (Byte 5, Bit 6)
    data[5] |= (0 << 6);
    // LDA_FRONT_CAMERA_BLOCKED: 47|1@0+ (Byte 5, Bit 7)
    data[5] |= (0 << 7);
    // LANE_SWAY_BUZZER: 50|2@0+ (Byte 6, Bit 2-1)
    data[6] |= (0 << 1);
    // LANE_SWAY_FLD: 53|3@0+ (Byte 6, Bit 5-3)
    data[6] |= (0 << 3);
    // LANE_SWAY_WARNING: 55|2@0+ (Byte 6, Bit 7-6)
    data[6] |= (0 << 6);
    // SET_ME_X01: 42|1@0+ (Byte 5, Bit 2)
    data[5] |= (0 << 2);
    // SET_ME_X02: 63|8@0+ (Byte 7)
    data[7] = 0;

    CAN.beginPacket(0x412);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }
  }

  delay(100); // 10Hz Update-Rate (anpassbar)
}
