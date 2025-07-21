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

// Additional DBC Variables
bool ACC_BRAKING = false;
float ACCEL_NET = 0.0; // in m/s^2
int16_t NEUTRAL_FORCE = 0; // in N
uint8_t CRUISE_STATE = 0; // 0-15
bool CANCEL_REQ = false;

// --- Smoothing Parameters ---
const int numReadings = 160;
float readings[numReadings] = {0};
int readIndex = 0;
float total = 0;

// --- VSS Sensor Variables ---
volatile int half_revolutions = 0;
int spd = 0;
unsigned long lastmillis = 0;

// --- CAN IDs to Monitor ---
const uint16_t monitoredIDs[] = {0x608, 0x37, 0x610}; // IDs to monitor
const int idCount = sizeof(monitoredIDs) / sizeof(monitoredIDs[0]);

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

  Serial.println("Monitoring CAN messages...");
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

  // --- Überwachung der CAN-Nachrichten ---
  monitorCANMessages();
}

void monitorCANMessages() {
  while (CAN.available()) {
    uint16_t messageID = CAN.packetId();
    for (int i = 0; i < idCount; i++) {
      if (messageID == monitoredIDs[i]) {
        Serial.print("Message detected on CAN bus: ID 0x");
        Serial.println(messageID, HEX);
        break;
      }
    }
    CAN.read(); // Read the message to clear it from the buffer
  }
}

// --- Nachrichtenfunktionen ---
void sendPCM_CRUISE() {
  uint8_t dat_1d2[8] = { 
    (OP_ON << 5) | (GAS_RELEASED << 4),     // Byte 0: CRUISE_ACTIVE and GAS_RELEASED
    (ACC_BRAKING << 4),                    // Byte 1: ACC_BRAKING (Bit 12)
    0,                                     // Byte 2: Unused
    (int16_t(ACCEL_NET / 0.0009765625) >> 8) & 0xFF, // Byte 3: ACCEL_NET (high byte)
    int16_t(ACCEL_NET / 0.0009765625) & 0xFF,       // Byte 4: ACCEL_NET (low byte)
    (NEUTRAL_FORCE >> 8) & 0xFF,           // Byte 5: NEUTRAL_FORCE (high byte)
    (NEUTRAL_FORCE & 0xFF) | (CANCEL_REQ << 7), // Byte 6: NEUTRAL_FORCE (low byte) and CANCEL_REQ
    0                                      // Byte 7: Checksum
  };
  
  dat_1d2[7] = dbc_checksum(dat_1d2, 7, 0x1D2); // Checksum
  CAN.beginPacket(0x1D2); 
  for (int i = 0; i < 8; i++) CAN.write(dat_1d2[i]); 
  CAN.endPacket();
}
