#include <CAN.h>

// Globale Strukturen für die letzten Werte jeder Nachricht
struct PCM_CRUISE_t {
  bool gas_released, cruise_active, acc_braking, cancel_req;
  float accel_net, neutral_force;
  uint8_t cruise_state, checksum;
  bool valid = false;
} last_pcm_cruise;

struct PCM_CRUISE_2_t {
  bool brake_pressed, main_on, acc_faulted;
  uint8_t pcm_follow_distance, low_speed_lockout, set_speed, checksum;
  bool valid = false;
} last_pcm_cruise2;

struct WHEEL_SPEEDS_t {
  float ws_fr, ws_fl, ws_rr, ws_rl;
  bool valid = false;
} last_wheel_speeds;

struct ESP_CONTROL_t {
  bool tc_disabled, vsc_disabled, brake_lights_acc, brake_hold_enabled, brake_hold_active;
  bool valid = false;
} last_esp_control;

struct BODY_CONTROL_STATE_t {
  bool meter_dimmed, parking_brake, seatbelt_driver_unlatched;
  bool door_open_fl, door_open_rl, door_open_rr, door_open_fr;
  bool valid = false;
} last_body_control;

struct BLINKERS_STATE_t {
  bool blinker_button_pressed, hazard_light;
  uint8_t turn_signals;
  bool valid = false;
} last_blinkers;

struct STEER_ANGLE_SENSOR_t {
  float steer_angle, steer_rate, steer_fraction;
  bool valid = false;
} last_steer_angle_sensor;

struct STEER_TORQUE_SENSOR_t {
  int16_t steer_torque_eps, steer_torque_driver;
  float steer_angle;
  bool steer_angle_initializing, steer_override;
  uint8_t checksum;
  bool valid = false;
} last_steer_torque_sensor;


// Hilfsfunktion zur Ausgabe einer bool als 0/1
#define BOOL01(b) ((b)?'1':'0')

void printScreen() {
  Serial.print("\033[2J\033[H"); // Terminal löschen & Cursor Home
  Serial.println("---- Aktuelle CAN-Werte ----\n");

  if (last_pcm_cruise.valid) {
    Serial.println("PCM_CRUISE:");
    Serial.print(" GAS_RELEASED: "); Serial.print(BOOL01(last_pcm_cruise.gas_released));
    Serial.print(" CRUISE_ACTIVE: "); Serial.print(BOOL01(last_pcm_cruise.cruise_active));
    Serial.print(" ACC_BRAKING: "); Serial.print(BOOL01(last_pcm_cruise.acc_braking));
    Serial.print(" ACCEL_NET: "); Serial.print(last_pcm_cruise.accel_net, 3); Serial.print("m/s^2");
    Serial.print(" NEUTRAL_FORCE: "); Serial.print(last_pcm_cruise.neutral_force); Serial.print("N");
    Serial.print(" CRUISE_STATE: "); Serial.print(last_pcm_cruise.cruise_state);
    Serial.print(" CANCEL_REQ: "); Serial.print(BOOL01(last_pcm_cruise.cancel_req));
    Serial.print(" CHECKSUM: 0x"); Serial.println(last_pcm_cruise.checksum, HEX);
  }
  if (last_pcm_cruise2.valid) {
    Serial.println("PCM_CRUISE_2:");
    Serial.print(" BRAKE_PRESSED: "); Serial.print(BOOL01(last_pcm_cruise2.brake_pressed));
    Serial.print(" PCM_FOLLOW_DISTANCE: "); Serial.print(last_pcm_cruise2.pcm_follow_distance);
    Serial.print(" LOW_SPEED_LOCKOUT: "); Serial.print(last_pcm_cruise2.low_speed_lockout);
    Serial.print(" MAIN_ON: "); Serial.print(BOOL01(last_pcm_cruise2.main_on));
    Serial.print(" SET_SPEED: "); Serial.print(last_pcm_cruise2.set_speed); Serial.print("km/h");
    Serial.print(" ACC_FAULTED: "); Serial.print(BOOL01(last_pcm_cruise2.acc_faulted));
    Serial.print(" CHECKSUM: 0x"); Serial.println(last_pcm_cruise2.checksum, HEX);
  }
  if (last_wheel_speeds.valid) {
    Serial.println("WHEEL_SPEEDS:");
    Serial.print(" FR: "); Serial.print(last_wheel_speeds.ws_fr,2);
    Serial.print(" FL: "); Serial.print(last_wheel_speeds.ws_fl,2);
    Serial.print(" RR: "); Serial.print(last_wheel_speeds.ws_rr,2);
    Serial.print(" RL: "); Serial.println(last_wheel_speeds.ws_rl,2);
  }
  if (last_esp_control.valid) {
    Serial.println("ESP_CONTROL:");
    Serial.print(" TC_DISABLED: "); Serial.print(BOOL01(last_esp_control.tc_disabled));
    Serial.print(" VSC_DISABLED: "); Serial.print(BOOL01(last_esp_control.vsc_disabled));
    Serial.print(" BRAKE_LIGHTS_ACC: "); Serial.print(BOOL01(last_esp_control.brake_lights_acc));
    Serial.print(" BRAKE_HOLD_ENABLED: "); Serial.print(BOOL01(last_esp_control.brake_hold_enabled));
    Serial.print(" BRAKE_HOLD_ACTIVE: "); Serial.println(BOOL01(last_esp_control.brake_hold_active));
  }
  if (last_body_control.valid) {
    Serial.println("BODY_CONTROL_STATE:");
    Serial.print(" METER_DIMMED: "); Serial.print(BOOL01(last_body_control.meter_dimmed));
    Serial.print(" PARKING_BRAKE: "); Serial.print(BOOL01(last_body_control.parking_brake));
    Serial.print(" SEATBELT_DRIVER_UNLATCHED: "); Serial.print(BOOL01(last_body_control.seatbelt_driver_unlatched));
    Serial.print(" DOOR_OPEN_FL: "); Serial.print(BOOL01(last_body_control.door_open_fl));
    Serial.print(" DOOR_OPEN_RL: "); Serial.print(BOOL01(last_body_control.door_open_rl));
    Serial.print(" DOOR_OPEN_RR: "); Serial.print(BOOL01(last_body_control.door_open_rr));
    Serial.print(" DOOR_OPEN_FR: "); Serial.println(BOOL01(last_body_control.door_open_fr));
  }
  if (last_blinkers.valid) {
    Serial.println("BLINKERS_STATE:");
    Serial.print(" BLINKER_BUTTON_PRESSED: "); Serial.print(BOOL01(last_blinkers.blinker_button_pressed));
    Serial.print(" HAZARD_LIGHT: "); Serial.print(BOOL01(last_blinkers.hazard_light));
    Serial.print(" TURN_SIGNALS: "); Serial.println(last_blinkers.turn_signals);
  }
  if (last_steer_angle_sensor.valid) {
    Serial.println("STEER_ANGLE_SENSOR:");
    Serial.print(" STEER_ANGLE: "); Serial.print(last_steer_angle_sensor.steer_angle, 2); Serial.print("deg");
    Serial.print(" STEER_RATE: "); Serial.print(last_steer_angle_sensor.steer_rate, 2); Serial.print("deg/s");
    Serial.print(" STEER_FRACTION: "); Serial.print(last_steer_angle_sensor.steer_fraction, 2); Serial.println("deg");
  }
  if (last_steer_torque_sensor.valid) {
    Serial.println("STEER_TORQUE_SENSOR:");
    Serial.print(" STEER_TORQUE_EPS: "); Serial.print(last_steer_torque_sensor.steer_torque_eps);
    Serial.print(" STEER_TORQUE_DRIVER: "); Serial.print(last_steer_torque_sensor.steer_torque_driver);
    Serial.print(" STEER_ANGLE: "); Serial.print(last_steer_torque_sensor.steer_angle, 2); Serial.print("deg");
    Serial.print(" INIT: "); Serial.print(BOOL01(last_steer_torque_sensor.steer_angle_initializing));
    Serial.print(" OVERRIDE: "); Serial.print(BOOL01(last_steer_torque_sensor.steer_override));
    Serial.print(" CHECKSUM: 0x"); Serial.println(last_steer_torque_sensor.checksum, HEX);
  }
}

void updatePCM_CRUISE(uint8_t *data) {
  last_pcm_cruise.gas_released = (data[0] >> 4) & 0x1;
  last_pcm_cruise.cruise_active = (data[0] >> 5) & 0x1;
  last_pcm_cruise.acc_braking = (data[1] >> 4) & 0x1;
  int16_t accel_net_raw = (data[2] << 8) | data[3];
  last_pcm_cruise.accel_net = -(accel_net_raw) * 0.0009765625;
  int16_t neutral_force_raw = (data[4] << 8) | data[5];
  last_pcm_cruise.neutral_force = -(neutral_force_raw) * 2;
  last_pcm_cruise.cruise_state = (data[6] >> 3) & 0xF;
  last_pcm_cruise.cancel_req = (data[6] >> 1) & 0x1;
  last_pcm_cruise.checksum = data[7];
  last_pcm_cruise.valid = true;
}

void updatePCM_CRUISE_2(uint8_t *data) {
  last_pcm_cruise2.brake_pressed = (data[0] >> 3) & 0x1;
  last_pcm_cruise2.pcm_follow_distance = (data[1] >> 4) & 0x3;
  last_pcm_cruise2.low_speed_lockout = (data[1] >> 6) & 0x3;
  last_pcm_cruise2.main_on = (data[1] >> 7) & 0x1;
  last_pcm_cruise2.set_speed = data[2];
  last_pcm_cruise2.acc_faulted = (data[5] >> 7) & 0x1;
  last_pcm_cruise2.checksum = data[7];
  last_pcm_cruise2.valid = true;
}

void updateWHEEL_SPEEDS(uint8_t *data) {
  last_wheel_speeds.ws_fr = ((data[0] << 8) | data[1]) * 0.01 - 67.67;
  last_wheel_speeds.ws_fl = ((data[2] << 8) | data[3]) * 0.01 - 67.67;
  last_wheel_speeds.ws_rr = ((data[4] << 8) | data[5]) * 0.01 - 67.67;
  last_wheel_speeds.ws_rl = ((data[6] << 8) | data[7]) * 0.01 - 67.67;
  last_wheel_speeds.valid = true;
}

void updateESP_CONTROL(uint8_t *data) {
  last_esp_control.vsc_disabled = (data[1] >> 4) & 0x1;
  last_esp_control.tc_disabled = (data[1] >> 5) & 0x1;
  last_esp_control.brake_lights_acc = (data[2] >> 2) & 0x1;
  last_esp_control.brake_hold_enabled = (data[4] >> 1) & 0x1;
  last_esp_control.brake_hold_active = (data[4] >> 4) & 0x1;
  last_esp_control.valid = true;
}

void updateBODY_CONTROL_STATE(uint8_t *data) {
  last_body_control.meter_dimmed = (data[4] >> 6) & 0x1;
  last_body_control.parking_brake = (data[7] >> 4) & 0x1;
  last_body_control.seatbelt_driver_unlatched = (data[7] >> 6) & 0x1;
  last_body_control.door_open_fl = (data[5] >> 5) & 0x1;
  last_body_control.door_open_rl = (data[5] >> 2) & 0x1;
  last_body_control.door_open_rr = (data[5] >> 3) & 0x1;
  last_body_control.door_open_fr = (data[5] >> 4) & 0x1;
  last_body_control.valid = true;
}

void updateBLINKERS_STATE(uint8_t *data) {
  last_blinkers.blinker_button_pressed = (data[1] >> 7) & 0x1;
  last_blinkers.hazard_light = (data[3] >> 3) & 0x1;
  last_blinkers.turn_signals = (data[3] >> 5) & 0x3;
  last_blinkers.valid = true;
}

void updateSTEER_ANGLE_SENSOR(uint8_t *data) {
  int16_t steer_angle_raw = ((data[0] << 8) | data[1]) >> 3;
  last_steer_angle_sensor.steer_angle = -(steer_angle_raw) * 1.5;
  int16_t steer_rate_raw = ((data[4] << 8) | data[5]) >> 3;
  last_steer_angle_sensor.steer_rate = -(steer_rate_raw);
  uint8_t steer_frac_raw = data[4] & 0x0F;
  last_steer_angle_sensor.steer_fraction = -(steer_frac_raw) * 0.1;
  last_steer_angle_sensor.valid = true;
}

void updateSTEER_TORQUE_SENSOR(uint8_t *data) {
  last_steer_torque_sensor.steer_torque_eps = (data[6] << 8) | data[5];
  last_steer_torque_sensor.steer_torque_driver = (data[2] << 8) | data[1];
  int16_t steer_angle_raw = (data[4] << 8) | data[3];
  last_steer_torque_sensor.steer_angle = -(steer_angle_raw) * 0.0573;
  last_steer_torque_sensor.steer_angle_initializing = (data[0] >> 3) & 0x1;
  last_steer_torque_sensor.steer_override = (data[0] >> 0) & 0x1;
  last_steer_torque_sensor.checksum = data[7];
  last_steer_torque_sensor.valid = true;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("CAN Receiver DBC decoded (Screen-Modus)");
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  int packetSize = CAN.parsePacket();

  if (packetSize == 8) {
    uint8_t data[8];
    for (int i = 0; i < 8; i++) {
      if (CAN.available()) data[i] = CAN.read();
      else data[i] = 0;
    }
    int id = CAN.packetId();
    switch (id) {
      case 0x1d2: updatePCM_CRUISE(data); break;
      case 0x1d3: updatePCM_CRUISE_2(data); break;
      case 0xAA:  updateWHEEL_SPEEDS(data); break;
      case 0x3b7: updateESP_CONTROL(data); break;
      case 0x620: updateBODY_CONTROL_STATE(data); break;
      case 0x614: updateBLINKERS_STATE(data); break;
      case 0x25:  updateSTEER_ANGLE_SENSOR(data); break;
      case 0x260: updateSTEER_TORQUE_SENSOR(data); break;
      default: break;
    }
  }

  static unsigned long lastScreen = 0;
  if (millis() - lastScreen > 300) { // ca. 3x pro Sekunde
    printScreen();
    lastScreen = millis();
  }
}
