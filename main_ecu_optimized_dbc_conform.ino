// MAIN ECU - DBC-konforme Version für Toyota Corolla 2017
#include <CAN.h>

// --- Pin Definitions ---
const int BlinkerPinLeft = 4;
const int BlinkerPinRight = 5;
const int buttonPins[4] = {9, 6, 7, 8};  // button1 to button4
const int ClutchSwitchPin = A4;
const byte interruptPin = 3;

// --- State Variables ---
bool buttonStates[4] = {HIGH, HIGH, HIGH, HIGH};
bool lastButtonStates[4] = {HIGH, HIGH, HIGH, HIGH};
bool clutchPressed = false;

// --- CAN and Logic Values ---
bool OP_ON = false;
bool MAIN_ON = true;
uint8_t set_speed = 0;
float average = 0.0f;
bool blinker_left = false;
bool blinker_right = false;
bool BRAKE_PRESSED = false;
bool GAS_RELEASED = true;
bool hazard_light = false; // Hazard light state
uint8_t turn_signals = 0;  // 0: None, 1: Left, 2: Right, 3: Hazard
bool auto_high_beam = false;
bool front_fog = false;
bool parking_light = true; // Simulating normal light on
bool low_beam = true;      // Simulating normal light on
bool high_beam = false;
bool daytime_running_light = true; // Simulating normal light on

// --- Body Control State Variables ---
bool meter_dimmed = false;                  // METER_DIMMED
bool parking_brake = false;                 // PARKING_BRAKE
bool seatbelt_driver_unlatched = false;     // SEATBELT_DRIVER_UNLATCHED
bool door_open_fl = false;                  // Front Left Door
bool door_open_rl = false;                  // Rear Left Door
bool door_open_rr = false;                  // Rear Right Door
bool door_open_fr = false;                  // Front Right Door

// --- Smoothing Parameters ---
const int numReadings = 160;
float readings[numReadings] = {0};
int readIndex = 0;
float total = 0;

// --- VSS Sensor Variables ---
volatile int half_revolutions = 0;
int spd = 0;
unsigned long lastmillis = 0;

void rpm() {
  half_revolutions++;
}

uint8_t dbc_checksum(uint8_t *data, uint8_t len, uint16_t addr) {
  uint8_t checksum = ((addr >> 8) & 0xFF) + (addr & 0xFF) + len + 1;
  for (int i = 0; i < len; i++) checksum += data[i];
  return checksum;
}

void setup() {
  Serial.begin(9600);
  CAN.begin(500E3);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rpm, FALLING);

  for (int i = 0; i < 4; i++) pinMode(buttonPins[i], INPUT);
  pinMode(BlinkerPinLeft, INPUT_PULLUP);
  pinMode(BlinkerPinRight, INPUT_PULLUP);
  pinMode(ClutchSwitchPin, INPUT);
}

void loop() {
  if (half_revolutions >= 1) {
    noInterrupts();
    unsigned long now = micros();
    unsigned long duration = now - lastmillis;
    lastmillis = now;
    spd = half_revolutions * (0.000135 / (duration * 0.000001)) * 3600;
    half_revolutions = 0;
    interrupts();
  }

  total -= readings[readIndex];
  readings[readIndex] = spd;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;

  clutchPressed = digitalRead(ClutchSwitchPin) == LOW;
  for (int i = 0; i < 4; i++) buttonStates[i] = digitalRead(buttonPins[i]);
  blinker_left = !digitalRead(BlinkerPinLeft);
  blinker_right = !digitalRead(BlinkerPinRight);

  if (BRAKE_PRESSED || !GAS_RELEASED) OP_ON = false;

  if (buttonStates[3] != lastButtonStates[3] && buttonStates[3] == LOW) {
    OP_ON = !OP_ON;
    if (OP_ON) set_speed = average + 3;
  }
  if (buttonStates[2] != lastButtonStates[2] && buttonStates[2] == LOW) set_speed += 5;
  if (buttonStates[1] != lastButtonStates[1] && buttonStates[1] == LOW) set_speed -= 5;
  if (buttonStates[0] != lastButtonStates[0] && buttonStates[0] == LOW) OP_ON = false;
  if (set_speed > 200) set_speed = 0;

  for (int i = 0; i < 4; i++) lastButtonStates[i] = buttonStates[i];

  // PCM_CRUISE (0x1D2)
  uint8_t dat_1d2[8] = { (OP_ON << 5) | (GAS_RELEASED << 4), 0, 0, 0, 0, 0, (OP_ON << 7), 0 };
  dat_1d2[7] = dbc_checksum(dat_1d2, 7, 0x1D2);
  CAN.beginPacket(0x1D2); for (int i = 0; i < 8; i++) CAN.write(dat_1d2[i]); CAN.endPacket();

  // PCM_CRUISE_2 (0x1D3)
  uint8_t dat_1d3[8] = { 0, (MAIN_ON << 7) | 0x28, set_speed, 0, 0, 0, 0, 0 };
  dat_1d3[7] = dbc_checksum(dat_1d3, 7, 0x1D3);
  CAN.beginPacket(0x1D3); for (int i = 0; i < 8; i++) CAN.write(dat_1d3[i]); CAN.endPacket();

  // WHEEL_SPEEDS (0x170)
  uint16_t ws_kph = (uint16_t)(average * 100);
  uint8_t dat_170[8];
  for (int i = 0; i < 4; i++) {
    dat_170[i * 2] = ws_kph >> 8;
    dat_170[i * 2 + 1] = ws_kph & 0xFF;
  }
  CAN.beginPacket(0x170); for (int i = 0; i < 8; i++) CAN.write(dat_170[i]); CAN.endPacket();

  // STEERING_IPAS (0x614)
  uint8_t dat_614[8] = {0x29, 0, 0x01, (blinker_left << 5) | (blinker_right << 4), 0, 0, 0x76, 0};
  dat_614[7] = dbc_checksum(dat_614, 7, 0x614);
  CAN.beginPacket(0x614); for (int i = 0; i < 8; i++) CAN.write(dat_614[i]); CAN.endPacket();

  // BLINKERS_STATE (0x1556)
  uint8_t dat_1556[8] = {0};
  dat_1556[1] = (hazard_light << 3) | (turn_signals & 0x03); // Encode HAZARD_LIGHT and TURN_SIGNALS
  dat_1556[2] = blinker_left || blinker_right;              // Encode BLINKER_BUTTON_PRESSED
  dat_1556[7] = dbc_checksum(dat_1556, 7, 0x1556);          // Calculate checksum
  CAN.beginPacket(0x1556); for (int i = 0; i < 8; i++) CAN.write(dat_1556[i]); CAN.endPacket();

  // LIGHT_STALK (0x1570)
  uint8_t dat_1570[8] = {0};
  dat_1570[3] = (auto_high_beam << 5) | (front_fog << 3) | (parking_light << 4) | (low_beam << 5) | (high_beam << 6) | (daytime_running_light << 7);
  dat_1570[7] = dbc_checksum(dat_1570, 7, 0x1570); // Calculate checksum
  CAN.beginPacket(0x1570); for (int i = 0; i < 8; i++) CAN.write(dat_1570[i]); CAN.endPacket();

  // BODY_CONTROL_STATE (0x1568)
  uint8_t dat_1568[8] = {0};
  dat_1568[5] = (meter_dimmed << 6); // Encode METER_DIMMED
  dat_1568[6] = (door_open_rl << 2) | (door_open_rr << 1) | (door_open_fr); // Encode DOOR_OPEN_RL, DOOR_OPEN_RR, DOOR_OPEN_FR
  dat_1568[7] = (parking_brake << 4) | (seatbelt_driver_unlatched << 6) | (door_open_fl << 5); // Encode PARKING_BRAKE, SEATBELT_DRIVER_UNLATCHED, DOOR_OPEN_FL
  dat_1568[7] = dbc_checksum(dat_1568, 7, 0x1568); // Calculate checksum
  CAN.beginPacket(0x1568); for (int i = 0; i < 8; i++) CAN.write(dat_1568[i]); CAN.endPacket();
} // loop
