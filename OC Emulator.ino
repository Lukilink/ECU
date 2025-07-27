#include <CAN.h>

// ====== SIGNAL-VARIABLEN AUS DBC ======

// PCM_CRUISE (0x1d2)
bool GAS_RELEASED = true;
bool CRUISE_ACTIVE = true; //
bool ACC_BRAKING = false; //"whether brakes are being actuated from ACC command";
float ACCEL_NET = 0.0;            // m/s^2 [-20..20] "net negative acceleration (braking) applied by the system if on flat ground";
int16_t NEUTRAL_FORCE = 0;        // N [-65536..65534] "force in newtons the engine/electric motors are applying without any acceleration commands or user input"
uint8_t CRUISE_STATE = 1;         // [0..15] //CRUISE_STATE 11 "timer_3sec" 10 "adaptive click down" 9 "adaptive click up" 8 "adaptive engaged" 7 "standstill" 6 "non-adaptive click up" 5 "non-adaptive click down" 4 "non-adaptive hold down" 3 "non-adaptive hold up" 2 "non-adaptive being engaged" 1 "non-adaptive engaged" 0 "off";
bool CANCEL_REQ = false;

// PCM_CRUISE_2 (0x1d3)
bool BRAKE_PRESSED = false;
uint8_t PCM_FOLLOW_DISTANCE = 0;  // [0..3]
uint8_t LOW_SPEED_LOCKOUT = 0;    // [0..3]
bool MAIN_ON = true;
uint8_t SET_SPEED = 10;            // km/h [0..255]
bool ACC_FAULTED = false;

// WHEEL_SPEEDS (0xaa)
float WHEEL_SPEED_FR = 10.0;       // km/h [0..250]
float WHEEL_SPEED_FL = 10.0;       // km/h [0..250]
float WHEEL_SPEED_RR = 10.0;        // km/h [0..250]
float WHEEL_SPEED_RL = 10.0;        // km/h [0..250]

// BLINKERS_STATE (0x614)
bool BLINKER_BUTTON_PRESSED = false;
bool HAZARD_LIGHT = false;
uint8_t TURN_SIGNALS = 0;         // [0..3]

// BODY_CONTROL_STATE (0x620)
bool METER_DIMMED = false;
bool PARKING_BRAKE = false;
bool SEATBELT_DRIVER_UNLATCHED = false;
bool DOOR_OPEN_FL = false;
bool DOOR_OPEN_RL = false;
bool DOOR_OPEN_RR = false;
bool DOOR_OPEN_FR = false;

// GEAR_PACKET (0x3bc)
bool SPORT_ON = false; // 0 "off" 1 "on";
uint8_t GEAR = 0;     // GEAR 0 "D" 1 "S" 8 "N" 16 "R" 32 "P";
bool SPORT_GEAR_ON = 0; // SPORT_GEAR_ON 0 "off" 1 "on";
uint8_t SPORT_GEAR = 0; // SPORT_GEAR 1 "S1" 2 "S2" 3 "S3" 4 "S4" 5 "S5" 6 "S6";
bool ECON_ON = 0; //ECON_ON 0 "off" 1 "on";
bool B_GEAR_ENGAGED = 0; //B_GEAR_ENGAGED 0 "off" 1 "on";
bool DRIVE_ENGAGED = 1; //DRIVE_ENGAGED 0 "off" 1 "on";

// ESP_CONTROL (0x3b7)
bool TC_DISABLED = false;
bool VSC_DISABLED = false;
bool BRAKE_LIGHTS_ACC = false;
bool BRAKE_HOLD_ENABLED = false;
bool BRAKE_HOLD_ACTIVE = false;

// STEER_ANGLE_SENSOR (0x25)
float STEER_ANGLE = 0.0;          // deg [-500..500]
float STEER_FRACTION = 0.0;       // deg [-0.7..0.7]
float STEER_RATE = 0.0;           // deg/s [-2000..2000]

// STEER_TORQUE_SENSOR (0x260)
int16_t STEER_TORQUE_EPS = 0;     // [-32768..32767]
int16_t STEER_TORQUE_DRIVER = 0;  // [-32768..32767]
float STEER_ANGLE_260 = 0.0;      // deg [-500..500]
bool STEER_ANGLE_INITIALIZING = false;
bool STEER_OVERRIDE = false;

// EPS_STATUS (0x262)
uint8_t IPAS_STATE = 3;           // [0..15] VAL_ 610 IPAS_STATE 5 "override" 3 "enabled" 1 "disabled";
uint8_t LKA_STATE = 5;            // [0..127] VAL_ 610 LKA_STATE 25 "temporary_fault" 9 "temporary_fault2" 5 "active" 1 "standby";
bool TYPE = 1; //"seems 1 on Corolla, 0 on all others";

// ====== CAN CHECKSUM (wie gehabt) ======
uint8_t can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}

// ====== SETUP ======
void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Toyota CAN Demo");
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

// Hilfsvariable für MAIN_ON Umschaltung (Demo)
unsigned long lastMainOnToggle = 0;

// ====== LOOP ======
void loop() {
  unsigned long now = millis();
  // Demo: MAIN_ON alle 5 Sekunden toggeln
  if (now - lastMainOnToggle >= 5000) {
    lastMainOnToggle = now;
 //   MAIN_ON = !MAIN_ON;
    CRUISE_ACTIVE = !CRUISE_ACTIVE;
  }

  // ====== PCM_CRUISE (0x1d2) ======
  {
    uint8_t d[8] = {0};
    d[0] |= (GAS_RELEASED ? 1 : 0) << 4;
    d[0] |= (CRUISE_ACTIVE ? 1 : 0) << 5;
    d[1] |= (ACC_BRAKING ? 1 : 0) << 4;
    int16_t accel_raw = (int16_t)(ACCEL_NET / 0.0009765625f);
    d[2] = (accel_raw >> 8) & 0xFF;
    d[3] = (accel_raw     ) & 0xFF;
    int16_t nforce_raw = (int16_t)(NEUTRAL_FORCE / 2.0f);
    d[4] = (nforce_raw >> 8) & 0xFF;
    d[5] = (nforce_raw     ) & 0xFF;
    d[6] |= (CRUISE_STATE & 0x0F) << 4;
    d[6] |= (CANCEL_REQ ? 1 : 0) << 1;
    d[7] = can_cksum(d, 7, 0x1d2);
    CAN.beginPacket(0x1D2);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  // ====== PCM_CRUISE_2 (0x1d3) ======
  {
    uint8_t d[8] = {0};
    d[0] |= (BRAKE_PRESSED ? 1 : 0) << 3;
    d[1] |= (PCM_FOLLOW_DISTANCE & 0x03) << 4;
    d[1] |= (LOW_SPEED_LOCKOUT & 0x03) << 6;
    d[1] |= (MAIN_ON ? 1 : 0) << 7;
    d[2] = SET_SPEED;
    d[5] |= (ACC_FAULTED ? 1 : 0) << 7;
    d[7] = can_cksum(d, 7, 0x1d3);
    CAN.beginPacket(0x1D3);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  // ====== WHEEL_SPEEDS (0xaa) ======
  {
    uint8_t d[8] = {0};
    uint16_t ws_fr = (uint16_t)((WHEEL_SPEED_FR + 67.67) / 0.01);
    uint16_t ws_fl = (uint16_t)((WHEEL_SPEED_FL + 67.67) / 0.01);
    uint16_t ws_rr = (uint16_t)((WHEEL_SPEED_RR + 67.67) / 0.01);
    uint16_t ws_rl = (uint16_t)((WHEEL_SPEED_RL + 67.67) / 0.01);
    d[0] = (ws_fr >> 8) & 0xFF; d[1] = ws_fr & 0xFF;
    d[2] = (ws_fl >> 8) & 0xFF; d[3] = ws_fl & 0xFF;
    d[4] = (ws_rr >> 8) & 0xFF; d[5] = ws_rr & 0xFF;
    d[6] = (ws_rl >> 8) & 0xFF; d[7] = ws_rl & 0xFF;
    CAN.beginPacket(0xAA);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  // ====== BLINKERS_STATE (0x614) ======
  {
    uint8_t d[8] = {0};
    d[1] |= (BLINKER_BUTTON_PRESSED ? 1 : 0) << 7;
    d[3] |= (HAZARD_LIGHT ? 1 : 0) << 3;
    d[3] |= (TURN_SIGNALS & 0x03) << 4;
    CAN.beginPacket(0x614);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

 // ====== BODY_CONTROL_STATE (0x620) ======
  {
    uint8_t d[8] = {0};
    d[4] |= (METER_DIMMED ? 1 : 0) << 6;
    d[7] |= (PARKING_BRAKE ? 1 : 0) << 4;
    d[7] |= (SEATBELT_DRIVER_UNLATCHED ? 1 : 0) << 6;
    d[5] |= (DOOR_OPEN_FL ? 1 : 0) << 2;
    d[5] |= (DOOR_OPEN_RL ? 1 : 0) << 3;
    d[5] |= (DOOR_OPEN_RR ? 1 : 0) << 4;
    d[5] |= (DOOR_OPEN_FR ? 1 : 0) << 5;
    CAN.beginPacket(0x620);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }
 
  // ====== GEAR_PACKET (0x3bc) ======
  {
    uint8_t d[8] = {0};
    d[0] |= (SPORT_ON ? 1 : 0) << 2;
    d[1] |= ((GEAR & 0x3F) << 5);
    d[2] |= ((GEAR & 0x3F) >> 3);
    d[4] |= (SPORT_GEAR_ON ? 1 : 0) << 1;
    d[4] |= (SPORT_GEAR & 0x03) << 6;
    d[5] |= ((SPORT_GEAR & 0x04) >> 2);
    d[5] |= (ECON_ON ? 1 : 0) << 0;
    d[5] |= (B_GEAR_ENGAGED ? 1 : 0) << 1;
    d[5] |= (DRIVE_ENGAGED ? 1 : 0) << 7;
    CAN.beginPacket(0x3bc);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }



  // ====== ESP_CONTROL (0x3b7) ======
  {
    uint8_t d[8] = {0};
    d[1] |= (VSC_DISABLED ? 1 : 0) << 4;
    d[1] |= (TC_DISABLED ? 1 : 0) << 5;
    d[2] |= (BRAKE_LIGHTS_ACC ? 1 : 0) << 2;
    d[4] |= (BRAKE_HOLD_ENABLED ? 1 : 0) << 0;
    d[4] |= (BRAKE_HOLD_ACTIVE ? 1 : 0) << 4;
    CAN.beginPacket(0x3B7);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  // ====== STEER_ANGLE_SENSOR (0x25) ======
  {
    uint8_t d[8] = {0};
    int16_t angle_raw = (int16_t)(STEER_ANGLE / 1.5); // 12 Bit, signed
    d[0] = (angle_raw >> 4) & 0xFF;
    d[1] = ((angle_raw & 0x0F) << 4);
    int16_t rate_raw = (int16_t)(STEER_RATE / 1.0);   // 12 Bit, signed
    d[4] = (rate_raw >> 4) & 0xFF;
    d[5] = ((rate_raw & 0x0F) << 4);
    int8_t frac_raw = (int8_t)(STEER_FRACTION / 0.1); // 4 Bit, signed
    d[4] |= ((frac_raw & 0x0F) << 4);
    CAN.beginPacket(0x25);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  // ====== STEER_TORQUE_SENSOR (0x260) ======
  {
    uint8_t d[8] = {0};
    d[0] |= (STEER_OVERRIDE ? 1 : 0) << 0;
    d[0] |= (STEER_ANGLE_INITIALIZING ? 1 : 0) << 3;
    d[1] = (STEER_TORQUE_DRIVER >> 8) & 0xFF;
    d[2] = STEER_TORQUE_DRIVER & 0xFF;
    int16_t angle_raw = (int16_t)(STEER_ANGLE_260 / 0.0573);
    d[3] = (angle_raw >> 8) & 0xFF;
    d[4] = angle_raw & 0xFF;
    d[5] = (STEER_TORQUE_EPS >> 8) & 0xFF;
    d[6] = STEER_TORQUE_EPS & 0xFF;
    d[7] = can_cksum(d, 7, 0x260);
    CAN.beginPacket(0x260);
    for (int i = 0; i < 8; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  // ====== EPS_STATUS (0x262) ======
  {
    uint8_t d[5] = {0};
    d[0] |= (IPAS_STATE & 0x0F);
    d[3] |= (LKA_STATE & 0x7F) << 1;
    d[3] |= (TYPE ? 1 : 0) << 0;
    d[4] = can_cksum(d, 4, 0x262);
    CAN.beginPacket(0x262);
    for (int i = 0; i < 5; i++) CAN.write(d[i]);
    CAN.endPacket();
  }

  delay(100); // 10Hz Update-Rate (anpassbar)
}
