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
bool lastGAS_RELEASED = false;
bool lastBRAKE_PRESSED = false;

// --- CAN and Logic Values ---
bool OP_ON = false;
bool MAIN_ON = true;
uint8_t set_speed = 0;
float average = 0.0f;
bool blinker_left = false;
bool blinker_right = false;
bool BRAKE_PRESSED = false;
bool GAS_RELEASED = true;

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

int can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = ((addr >> 8) & 0xFF) + (addr & 0xFF) + len + 1;
  for (int i = 0; i < len; i++) checksum += dat[i];
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
  lastBRAKE_PRESSED = BRAKE_PRESSED;
  lastGAS_RELEASED = GAS_RELEASED;

  // PCM_CRUISE (0x1D2)
  uint8_t dat_1d2[8] = { (OP_ON << 5) | (GAS_RELEASED << 4), 0, 0, 0, 0, 0, (OP_ON << 7), 0 };
  dat_1d2[7] = can_cksum(dat_1d2, 7, 0x1D2);
  CAN.beginPacket(0x1D2); for (int i = 0; i < 8; i++) CAN.write(dat_1d2[i]); CAN.endPacket();

  // PCM_CRUISE_2 (0x1D3)
  uint8_t dat_1d3[8] = { 0, (MAIN_ON << 7) | 0x28, set_speed, 0, 0, 0, 0, 0 };
  dat_1d3[7] = can_cksum(dat_1d3, 7, 0x1D3);
  CAN.beginPacket(0x1D3); for (int i = 0; i < 8; i++) CAN.write(dat_1d3[i]); CAN.endPacket();

  // WHEEL_SPEEDS (0x170)
  uint16_t ws_kph = (uint16_t)(average * 100);
  uint8_t dat_170[8];
  for (int i = 0; i < 4; i++) {
    dat_170[i * 2] = ws_kph >> 8;
    dat_170[i * 2 + 1] = ws_kph & 0xFF;
  }
  CAN.beginPacket(0x170); for (int i = 0; i < 8; i++) CAN.write(dat_170[i]); CAN.endPacket();

  // STEERING_LEVERS (0x614)
  uint8_t dat_614[8] = {0x29, 0, 0x01, (blinker_left << 5) | (blinker_right << 4), 0, 0, 0x76, 0};
  dat_614[7] = can_cksum(dat_614, 7, 0x614);
  CAN.beginPacket(0x614); for (int i = 0; i < 8; i++) CAN.write(dat_614[i]); CAN.endPacket();

  if (CAN.parsePacket()) {
    uint16_t id = CAN.packetId();
    uint8_t data[8];
    for (int i = 0; i < 8; i++) data[i] = CAN.read();

    if (id == 0x548) {
      BRAKE_PRESSED = (data[0] >> 5) & 0x01;
    }
    else if (id == 0x705) {
      GAS_RELEASED = (data[0] >> 3) & 0x01;
    }
  }
} // loop
