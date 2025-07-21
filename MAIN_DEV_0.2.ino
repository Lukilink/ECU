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
  // --- Geschwindigkeit berechnen ---
  if (half_revolutions >= 1) {
    noInterrupts();
    unsigned long now = micros();
    unsigned long duration = now - lastmillis;
    lastmillis = now;
    spd = half_revolutions * (0.000135 / (duration * 0.000001)) * 3600;
    half_revolutions = 0;
    interrupts();
  }

  // --- Geschwindigkeit glätten ---
  total -= readings[readIndex];
  readings[readIndex] = spd;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;

  // --- Status der Tasten und Blinker ---
  clutchPressed = digitalRead(ClutchSwitchPin) == LOW;
  for (int i = 0; i < 4; i++) buttonStates[i] = digitalRead(buttonPins[i]);
  blinker_left = !digitalRead(BlinkerPinLeft);
  blinker_right = !digitalRead(BlinkerPinRight);

  // --- OP Logik ---
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

  // --- CAN-Nachrichten senden ---
  sendPCM_CRUISE();
  sendPCM_CRUISE_2();
  sendWHEEL_SPEEDS();
  sendSTEERING_IPAS();
  sendLIGHT_STALK();
  sendBLINKERS_STATE();
  sendBODY_CONTROL_STATE();
  sendBODY_CONTROL_STATE_2();
  sendESP_CONTROL();
  sendBRAKE_MODULE();
  sendPCM_CRUISE_SM();
  sendVSC1S07();
  sendENGINE_RPM();
  sendGEAR_PACKET();
  sendPRE_COLLISION_2();
}

// --- Nachrichtenfunktionen ---
void sendPCM_CRUISE() {
  uint8_t dat_1d2[8] = { (OP_ON << 5) | (GAS_RELEASED << 4), 0, 0, 0, 0, 0, (OP_ON << 7), 0 };
  dat_1d2[7] = dbc_checksum(dat_1d2, 7, 0x1D2);
  CAN.beginPacket(0x1D2); for (int i = 0; i < 8; i++) CAN.write(dat_1d2[i]); CAN.endPacket();
}

void sendPCM_CRUISE_2() {
  uint8_t dat_1d3[8] = { 0, (MAIN_ON << 7) | 0x28, set_speed, 0, 0, 0, 0, 0 };
  dat_1d3[7] = dbc_checksum(dat_1d3, 7, 0x1D3);
  CAN.beginPacket(0x1D3); for (int i = 0; i < 8; i++) CAN.write(dat_1d3[i]); CAN.endPacket();
}

void sendWHEEL_SPEEDS() {
  uint16_t ws_kph = (uint16_t)(average * 100);
  uint8_t dat_170[8];
  for (int i = 0; i < 4; i++) {
    dat_170[i * 2] = ws_kph >> 8;
    dat_170[i * 2 + 1] = ws_kph & 0xFF;
  }
  CAN.beginPacket(0x170); for (int i = 0; i < 8; i++) CAN.write(dat_170[i]); CAN.endPacket();
}

void sendSTEERING_IPAS() {
  uint8_t dat_614[8] = {0x29, 0, 0x01, (blinker_left << 5) | (blinker_right << 4), 0, 0, 0x76, 0};
  dat_614[7] = dbc_checksum(dat_614, 7, 0x614);
  CAN.beginPacket(0x614); for (int i = 0; i < 8; i++) CAN.write(dat_614[i]); CAN.endPacket();
}

void sendLIGHT_STALK() {
  uint8_t dat_1570[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00}; // Simuliert: Abblendlicht, Tagfahrlicht
  CAN.beginPacket(0x1570); for (int i = 0; i < 8; i++) CAN.write(dat_1570[i]); CAN.endPacket();
}

void sendBLINKERS_STATE() {
  uint8_t dat_1556[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (blinker_left << 1) | blinker_right};
  CAN.beginPacket(0x1556); for (int i = 0; i < 8; i++) CAN.write(dat_1556[i]); CAN.endPacket();
}

void sendBODY_CONTROL_STATE() {
  uint8_t dat_1568[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Keine offenen Türen, kein Parken
  CAN.beginPacket(0x1568); for (int i = 0; i < 8; i++) CAN.write(dat_1568[i]); CAN.endPacket();
}

void sendBODY_CONTROL_STATE_2() {
  uint8_t dat_1552[8] = {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01}; // Simuliert: Helligkeit auf Maximum
  CAN.beginPacket(0x1552); for (int i = 0; i < 8; i++) CAN.write(dat_1552[i]); CAN.endPacket();
}

void sendESP_CONTROL() {
  uint8_t dat_951[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Standard ESP-Zustand
  CAN.beginPacket(0x951); for (int i = 0; i < 8; i++) CAN.write(dat_951[i]); CAN.endPacket();
}

void sendBRAKE_MODULE() {
  uint8_t dat_548[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Keine Bremse gedrückt
  CAN.beginPacket(0x548); for (int i = 0; i < 8; i++) CAN.write(dat_548[i]); CAN.endPacket();
}

void sendPCM_CRUISE_SM() {
  uint8_t dat_921[8] = {0x00, 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Tempomat aktiv
  CAN.beginPacket(0x921); for (int i = 0; i < 8; i++) CAN.write(dat_921[i]); CAN.endPacket();
}

void sendVSC1S07() {
  uint8_t dat_800[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Stabilisierungsmodus
  CAN.beginPacket(0x800); for (int i = 0; i < 8; i++) CAN.write(dat_800[i]); CAN.endPacket();
}

void sendENGINE_RPM() {
  uint16_t rpm = 3000; // 3000 U/min
  uint8_t dat_452[8] = {rpm >> 8, rpm & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.beginPacket(0x452); for (int i = 0; i < 8; i++) CAN.write(dat_452[i]); CAN.endPacket();
}

void sendGEAR_PACKET() {
  uint8_t dat_956[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // Simuliert: Gang eingelegt
  CAN.beginPacket(0x956); for (int i = 0; i < 8; i++) CAN.write(dat_956[i]); CAN.endPacket();
}

void sendPRE_COLLISION_2() {
  uint8_t dat_836[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Normalzustand
  CAN.beginPacket(0x836); for (int i = 0; i < 8; i++) CAN.write(dat_836[i]); CAN.endPacket();
}
