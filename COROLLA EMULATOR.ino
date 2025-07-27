#include <CAN.h>


// Hilfsvariable für OP Anschalten-Umschaltung
unsigned long lastOpToggleMillis = 0;
bool OP_ANSCHALTEN = false;


// =========================
// Konfigurierbare Variablen (aus DBC)
// =========================

// -------- PCM_CRUISE (0x1D2 / 466) --------
// GAS_RELEASED: 0 = kein Gas freigegeben, 1 = Gas freigegeben
// CRUISE_ACTIVE: 0 = ACC aus, 1 = ACC aktiv
// ACC_BRAKING: 0 = kein ACC Bremsen, 1 = ACC bremst
// ACCEL_NET: [-20|20] m/s^2, Faktor: 0.0009765625, Offset: 0
// NEUTRAL_FORCE: [-65536|65534] N, Faktor: 2, Offset: 0
// CRUISE_STATE: 0=off, 1=non-adaptive engaged, 2=non-adaptive being engaged, 8=adaptive engaged, 11=timer_3sec, 7=standstill, 9/10=click up/down
// CANCEL_REQ: 0 = kein Cancel, 1 = Cancel
uint8_t PCM_CRUISE_GAS_RELEASED = 0;
uint8_t PCM_CRUISE_CRUISE_ACTIVE = 1;
uint8_t PCM_CRUISE_ACC_BRAKING = 0;
float PCM_CRUISE_ACCEL_NET = 0.5; // m/s^2
int16_t PCM_CRUISE_NEUTRAL_FORCE = 0; // N
uint8_t PCM_CRUISE_CRUISE_STATE = 2;
uint8_t PCM_CRUISE_CANCEL_REQ = 0;

// -------- PCM_CRUISE_2 (0x1D3 / 467) --------
// BRAKE_PRESSED: 0 = nicht gedrückt, 1 = gedrückt
// PCM_FOLLOW_DISTANCE: 1=far, 2=medium, 3=close
// LOW_SPEED_LOCKOUT: 0=none, 1=ok, 2=locked
// MAIN_ON: 0=aus, 1=ein
// SET_SPEED: [0|255] km/h
// ACC_FAULTED: 0=kein Fehler, 1=Fehler
uint8_t PCM_CRUISE_2_BRAKE_PRESSED = 0;
uint8_t PCM_CRUISE_2_PCM_FOLLOW_DISTANCE = 2; // medium
uint8_t PCM_CRUISE_2_LOW_SPEED_LOCKOUT = 0;
uint8_t PCM_CRUISE_2_MAIN_ON = 1;
uint8_t PCM_CRUISE_2_SET_SPEED = 100; // km/h
uint8_t PCM_CRUISE_2_ACC_FAULTED = 0;

// -------- WHEEL_SPEEDS (0xAA / 170) --------
// WHEEL_SPEED_*: [0|250] km/h, Faktor: 0.01, Offset: -67.67
float WHEEL_SPEEDS_FR = 60;
float WHEEL_SPEEDS_FL = 60;
float WHEEL_SPEEDS_RR = 60;
float WHEEL_SPEEDS_RL = 60;

// -------- BLINKERS_STATE (0x614 / 1556) --------
// BLINKER_BUTTON_PRESSED: 0=not pressed, 1=pressed
// HAZARD_LIGHT: 0=aus, 1=an
// TURN_SIGNALS: 1=links, 2=rechts, 3=none
uint8_t BLINKERS_BUTTON_PRESSED = 0;
uint8_t BLINKERS_HAZARD_LIGHT = 0;
uint8_t BLINKERS_TURN_SIGNALS = 1; // 1=links

// -------- BODY_CONTROL_STATE (0x620 / 1568) --------
// METER_DIMMED, PARKING_BRAKE, SEATBELT_DRIVER_UNLATCHED, DOOR_OPEN_*
uint8_t BCS_METER_DIMMED = 0;
uint8_t BCS_PARKING_BRAKE = 0;
uint8_t BCS_SEATBELT_DRIVER_UNLATCHED = 0;
uint8_t BCS_DOOR_OPEN_FL = 0;
uint8_t BCS_DOOR_OPEN_FR = 0;
uint8_t BCS_DOOR_OPEN_RL = 0;
uint8_t BCS_DOOR_OPEN_RR = 0;

// -------- GEAR_PACKET (0x3BC / 956) --------
// SPORT_ON: 1=an
// GEAR: 0=D, 1=S, 8=N, 16=R, 32=P
// SPORT_GEAR_ON: 1=an
// SPORT_GEAR: 1=S1...6=S6
// ECON_ON, B_GEAR_ENGAGED, DRIVE_ENGAGED: 0=aus, 1=an
uint8_t GEAR_SPORT_ON = 1;
uint8_t GEAR_GEAR = 4; // S4
uint8_t GEAR_SPORT_GEAR_ON = 1;
uint8_t GEAR_SPORT_GEAR = 1; // S1
uint8_t GEAR_ECON_ON = 0;
uint8_t GEAR_B_GEAR_ENGAGED = 0;
uint8_t GEAR_DRIVE_ENGAGED = 1;

// -------- ESP_CONTROL (0x3B7 / 951) --------
// TC_DISABLED, VSC_DISABLED, BRAKE_LIGHTS_ACC, BRAKE_HOLD_ENABLED, BRAKE_HOLD_ACTIVE
uint8_t ESP_TC_DISABLED = 0;
uint8_t ESP_VSC_DISABLED = 0;
uint8_t ESP_BRAKE_LIGHTS_ACC = 0;
uint8_t ESP_BRAKE_HOLD_ENABLED = 0;
uint8_t ESP_BRAKE_HOLD_ACTIVE = 0;

// -------- STEER_ANGLE_SENSOR (0x25 / 37) --------
// STEER_ANGLE: [-500|500] deg, Faktor: 1.5, Offset: 0
// STEER_FRACTION: [-0.7|0.7] deg, Faktor: 0.1, Offset: 0
// STEER_RATE: [-2000|2000] deg/s, Faktor: 1, Offset: 0
float SAS_ANGLE = 10;
float SAS_FRACTION = 0.2;
float SAS_RATE = 100;

// -------- STEER_TORQUE_SENSOR (0x260 / 608) --------
// STEER_TORQUE_EPS: [-32768|32767] Nm
// STEER_TORQUE_DRIVER: [-32768|32767] Nm
// STEER_ANGLE: [-500|500] deg, Faktor: 0.0573
// STEER_ANGLE_INITIALIZING, STEER_OVERRIDE
int16_t STS_TORQUE_EPS = 100;
int16_t STS_TORQUE_DRIVER = 5;
float   STS_ANGLE = 20;
uint8_t STS_ANGLE_INITIALIZING = 0;
uint8_t STS_OVERRIDE = 0;

// -------- EPS_STATUS (0x262 / 610) --------
// IPAS_STATE: 0=off, 1=disabled, 3=enabled, 5=override
// LKA_STATE: 1=standby, 5=active, 9=tmp_fault2, 25=tmp_fault
// TYPE: 0=andere, 1=Corolla
uint8_t EPS_IPAS_STATE = 0;
uint8_t EPS_LKA_STATE = 1;
uint8_t EPS_TYPE = 0;

// -------- LIGHT_STALK (0x622 / 1570) --------
uint8_t LS_AUTO_HIGH_BEAM = 0;
uint8_t LS_FRONT_FOG = 0;
uint8_t LS_PARKING_LIGHT = 1;
uint8_t LS_LOW_BEAM = 1;
uint8_t LS_HIGH_BEAM = 0;
uint8_t LS_DAYTIME_RUNNING_LIGHT = 1;

// -------- PRE_COLLISION (0x283 / 643) --------
uint8_t PRECOLL_COUNTER = 0;
uint8_t PRECOLL_SET_ME_X00 = 0x00;
int16_t PRECOLL_FORCE = 120;
uint8_t PRECOLL_SET_ME_X002 = 0;
uint8_t PRECOLL_BRAKE_STATUS = 0;
uint8_t PRECOLL_STATE = 1; // 0=normal, 1=adaptive_cc, 3=emergency_braking
uint8_t PRECOLL_SET_ME_X003 = 0;
uint8_t PRECOLL_ACTIVE = 0;

// -------- BRAKE_MODULE (0x224 / 548) --------
uint16_t BRAKE_PRESSURE = 0;
uint8_t BRAKE_PRESSED = 0;

// -------- BODY_CONTROL_STATE_2 (0x610 / 1552) --------
uint8_t BCS2_UI_SPEED = 100;
uint8_t BCS2_BRIGHTNESS_PCT = 80;
uint8_t BCS2_LOW_BRIGHTNESS = 0;
uint8_t BCS2_DIMMED = 0;
uint8_t BCS2_UNITS = 1; // 1=km/h

// -------- PCM_CRUISE_SM (0x399 / 921) --------
uint8_t PCMSM_MAIN_ON = 1;
uint8_t PCMSM_CRUISE_CONTROL_STATE = 2;
uint8_t PCMSM_DISTANCE_LINES = 2;
uint8_t PCMSM_TEMP_ACC_FAULTED = 0;
uint8_t PCMSM_UI_SET_SPEED = 100;

// -------- ENGINE_RPM (0x1C4 / 452) --------
float ENGINE_RPM = 2200.0;
uint8_t ENGINE_RUNNING = 1;

// -------- VSC1S07 (0x320 / 800) --------
// alle Felder aktuell auf 0

// -------- LKAS_HUD (0x412 / 1042) --------
// Werte für LKAS HUD (nur Beispiel)
uint8_t LKAS_BARRIERS = 0;
uint8_t LKAS_RIGHT_LINE = 2;
uint8_t LKAS_LEFT_LINE = 2;
uint8_t LKAS_LKAS_STATUS = 1;
uint8_t LKAS_LDW_EXIST = 1;

// =========================
// Hilfsfunktionen
// =========================
uint8_t can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}

// =========================
// Arduino Setup
// =========================
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("OpenPilot CAN Sender (Strukturierte Variablen)");
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

// =========================
// Arduino Loop
// =========================
void loop() {

  // OP_ANSCHALTEN alle 5 Sekunden toggeln (MAIN_ON & CRUISE_ACTIVE)
    unsigned long now = millis();
  if (now - lastOpToggleMillis >= 5000) {
    lastOpToggleMillis = now;
    OP_ANSCHALTEN = !OP_ANSCHALTEN;

    if (OP_ANSCHALTEN) {
      PCM_CRUISE_2_MAIN_ON = 1;
      PCMSM_MAIN_ON = 1;
      PCM_CRUISE_CRUISE_ACTIVE = 1;
    } else {
      PCM_CRUISE_2_MAIN_ON = 0;
      PCMSM_MAIN_ON = 0;
      PCM_CRUISE_CRUISE_ACTIVE = 0;
    }
  }
  
  // PCM_CRUISE (0x1D2)
  {
    uint8_t data[8] = {0};
    data[0] |= (PCM_CRUISE_GAS_RELEASED << 4);
    data[0] |= (PCM_CRUISE_CRUISE_ACTIVE << 5);
    data[1] |= (PCM_CRUISE_ACC_BRAKING << 4);
    int16_t accel = (int16_t)(PCM_CRUISE_ACCEL_NET / 0.0009765625f);
    data[2] = (accel >> 8) & 0xFF;
    data[3] = (accel     ) & 0xFF;
    int16_t nforce = (int16_t)(PCM_CRUISE_NEUTRAL_FORCE / 2.0f);
    data[4] = (nforce >> 8) & 0xFF;
    data[5] = (nforce     ) & 0xFF;
    data[6] |= (PCM_CRUISE_CRUISE_STATE << 4);
    data[6] |= (PCM_CRUISE_CANCEL_REQ << 1);
    data[7] = can_cksum(data, 7, 0x1d2);
    CAN.beginPacket(0x1D2);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PCM_CRUISE_2 (0x1D3)
  {
    uint8_t data[8] = {0};
    data[0] |= (PCM_CRUISE_2_BRAKE_PRESSED << 3);
    data[1] |= (PCM_CRUISE_2_PCM_FOLLOW_DISTANCE << 4);
    data[1] |= (PCM_CRUISE_2_LOW_SPEED_LOCKOUT << 6);
    data[1] |= (PCM_CRUISE_2_MAIN_ON << 7);
    data[2] = PCM_CRUISE_2_SET_SPEED;
    data[5] |= (PCM_CRUISE_2_ACC_FAULTED << 7);
    data[7] = can_cksum(data, 7, 0x1d3);
    CAN.beginPacket(0x1D3);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // WHEEL_SPEEDS (0xAA)
  {
    uint8_t data[8] = {0};
    uint16_t ws_fr = (uint16_t)((WHEEL_SPEEDS_FR + 67.67) / 0.01);
    uint16_t ws_fl = (uint16_t)((WHEEL_SPEEDS_FL + 67.67) / 0.01);
    uint16_t ws_rr = (uint16_t)((WHEEL_SPEEDS_RR + 67.67) / 0.01);
    uint16_t ws_rl = (uint16_t)((WHEEL_SPEEDS_RL + 67.67) / 0.01);
    data[0] = (ws_fr >> 8) & 0xFF; data[1] = ws_fr & 0xFF;
    data[2] = (ws_fl >> 8) & 0xFF; data[3] = ws_fl & 0xFF;
    data[4] = (ws_rr >> 8) & 0xFF; data[5] = ws_rr & 0xFF;
    data[6] = (ws_rl >> 8) & 0xFF; data[7] = ws_rl & 0xFF;
    CAN.beginPacket(0xAA);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BLINKERS_STATE (0x614)
  {
    uint8_t data[8] = {0};
    data[1] |= (BLINKERS_BUTTON_PRESSED << 7);
    data[3] |= (BLINKERS_HAZARD_LIGHT << 3);
    data[3] |= (BLINKERS_TURN_SIGNALS << 4);
    CAN.beginPacket(0x614);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BODY_CONTROL_STATE (0x620)
  {
    uint8_t data[8] = {0};
    data[4] |= (BCS_DOOR_OPEN_RL << 2);
    data[4] |= (BCS_DOOR_OPEN_RR << 3);
    data[4] |= (BCS_DOOR_OPEN_FR << 4);
    data[4] |= (BCS_DOOR_OPEN_FL << 5);
    data[5] |= (BCS_SEATBELT_DRIVER_UNLATCHED << 2);
    data[5] |= (BCS_PARKING_BRAKE << 3);
    data[5] |= (BCS_METER_DIMMED << 4);
    CAN.beginPacket(0x620);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // GEAR_PACKET (0x3BC)
  {
    uint8_t data[8] = {0};
    data[0] |= (GEAR_SPORT_ON << 2);
    data[1] |= ((GEAR_GEAR & 0x3F) << 5);
    data[2] |= ((GEAR_GEAR & 0x3F) >> 3);
    data[4] |= (GEAR_SPORT_GEAR_ON << 1);
    data[4] |= ((GEAR_SPORT_GEAR & 0x03) << 6);
    data[5] |= ((GEAR_SPORT_GEAR & 0x04) >> 2);
    data[5] |= (GEAR_DRIVE_ENGAGED << 7);
    CAN.beginPacket(0x3bc);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // ESP_CONTROL (0x3B7)
  {
    uint8_t data[8] = {0};
    data[1] |= (ESP_VSC_DISABLED << 4);
    data[1] |= (ESP_TC_DISABLED << 5);
    data[2] |= (ESP_BRAKE_LIGHTS_ACC << 2);
    data[4] |= (ESP_BRAKE_HOLD_ENABLED << 0);
    data[4] |= (ESP_BRAKE_HOLD_ACTIVE << 4);
    CAN.beginPacket(0x3B7);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // STEER_ANGLE_SENSOR (0x25)
  {
    uint8_t data[8] = {0};
    int16_t angle = (int16_t)(SAS_ANGLE / 1.5);
    data[0] = (angle >> 4) & 0xFF;
    data[1] = ((angle & 0x0F) << 4);
    int16_t rate = (int16_t)(SAS_RATE / 1.0);
    data[4] = (rate >> 4) & 0xFF;
    data[5] = ((rate & 0x0F) << 4);
    int8_t fraction = (int8_t)(SAS_FRACTION / 0.1);
    data[4] |= ((fraction & 0x0F) << 4);
    CAN.beginPacket(0x25);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // STEER_TORQUE_SENSOR (0x260)
  {
    uint8_t data[8] = {0};
    data[5] = (STS_TORQUE_EPS >> 8) & 0xFF; data[6] = STS_TORQUE_EPS & 0xFF;
    data[1] = (STS_TORQUE_DRIVER >> 8) & 0xFF; data[2] = STS_TORQUE_DRIVER & 0xFF;
    int16_t angle = (int16_t)(STS_ANGLE / 0.0573);
    data[3] = (angle >> 8) & 0xFF; data[4] = angle & 0xFF;
    data[0] |= (STS_OVERRIDE << 0);
    data[0] |= (STS_ANGLE_INITIALIZING << 3);
    data[7] = can_cksum(data, 7, 0x260);
    CAN.beginPacket(0x260);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // EPS_STATUS (0x262)
  {
    uint8_t data[5] = {0};
    data[0] |= (EPS_IPAS_STATE & 0x0F);
    data[3] |= (EPS_LKA_STATE << 1);
    data[3] |= (EPS_TYPE << 0);
    data[4] = can_cksum(data, 4, 0x262);
    CAN.beginPacket(0x262);
    for (int i = 0; i < 5; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // LIGHT_STALK (0x622)
  {
    uint8_t data[8] = {0};
    data[4] |= (LS_AUTO_HIGH_BEAM << 5);
    data[3] |= (LS_FRONT_FOG << 3);
    data[3] |= (LS_PARKING_LIGHT << 4);
    data[3] |= (LS_LOW_BEAM << 5);
    data[3] |= (LS_HIGH_BEAM << 6);
    data[3] |= (LS_DAYTIME_RUNNING_LIGHT << 7);
    CAN.beginPacket(0x622);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PRE_COLLISION (0x283)
  {
    uint8_t data[7] = {0};
    data[0] = PRECOLL_COUNTER++;
    if (PRECOLL_COUNTER > 250) PRECOLL_COUNTER = 0;
    data[1] = PRECOLL_SET_ME_X00;
    data[2] = (PRECOLL_FORCE >> 8) & 0xFF;
    data[3] = (PRECOLL_FORCE     ) & 0xFF;
    data[4] = PRECOLL_SET_ME_X002;
    data[4] |= (PRECOLL_BRAKE_STATUS << 5);
    data[4] |= (PRECOLL_STATE << 2);
    data[5] |= (PRECOLL_SET_ME_X003 << 0);
    data[5] |= (PRECOLL_ACTIVE << 1);
    data[6] = can_cksum(data, 6, 0x283);
    CAN.beginPacket(0x283);
    for (int i = 0; i < 7; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BRAKE_MODULE (0x224)
  {
    uint8_t data[8] = {0};
    data[5] |= (BRAKE_PRESSURE & 0xFF) << 3;
    data[6] |= (BRAKE_PRESSURE >> 5) & 0x7F;
    data[0] |= (BRAKE_PRESSED << 5);
    CAN.beginPacket(0x224);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BODY_CONTROL_STATE_2 (0x610)
  {
    uint8_t data[8] = {0};
    data[2] |= (BCS2_UI_SPEED << 7);
    data[3] |= (BCS2_UI_SPEED >> 1);
    data[3] |= (BCS2_BRIGHTNESS_PCT & 0x7F) << 6;
    data[4] |= (BCS2_BRIGHTNESS_PCT >> 1);
    data[4] |= (BCS2_LOW_BRIGHTNESS << 5);
    data[4] |= (BCS2_DIMMED << 6);
    data[7] |= (BCS2_UNITS << 5);
    CAN.beginPacket(0x610);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PCM_CRUISE_SM (0x399)
  {
    uint8_t data[8] = {0};
    data[0] |= (PCMSM_MAIN_ON << 4);
    data[1] |= (PCMSM_CRUISE_CONTROL_STATE << 0);
    data[1] |= (PCMSM_DISTANCE_LINES << 5);
    data[1] |= (PCMSM_TEMP_ACC_FAULTED << 7);
    data[3] = PCMSM_UI_SET_SPEED;
    CAN.beginPacket(0x399);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // ENGINE_RPM (0x1C4)
  {
    uint8_t data[8] = {0};
    int16_t rpm_raw = (int16_t)(ENGINE_RPM / 0.78125);
    data[0] = rpm_raw & 0xFF;
    data[1] = (rpm_raw >> 8) & 0xFF;
    data[3] |= (ENGINE_RUNNING << 3);
    CAN.beginPacket(0x1C4);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // VSC1S07 (0x320)
  {
    uint8_t data[8] = {0};
    CAN.beginPacket(0x320);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // LKAS_HUD (0x412)
  {
    uint8_t data[8] = {0};
    data[0] |= (LKAS_BARRIERS << 0);
    data[0] |= (LKAS_RIGHT_LINE << 2);
    data[0] |= (LKAS_LEFT_LINE << 4);
    data[0] |= (LKAS_LKAS_STATUS << 6);
    data[1] |= (LKAS_LDW_EXIST << 2);
    CAN.beginPacket(0x412);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  delay(100); // 10Hz Update-Rate
}
