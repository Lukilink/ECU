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
  sendPCM_CRUISE_2();
  sendWHEEL_SPEEDS();
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
  uint8_t dat_1d2[8] = { (OP_ON << 5) | (GAS_RELEASED << 4), 0, 0, 0, 0, 0, (OP_ON << 7), 0 };
  dat_1d2[7] = dbc_checksum(dat_1d2, 7, 0x1D2);
  CAN.beginPacket(0x1D2); for (int i = 0; i < 8; i++) CAN.write(dat_1d2[i]); CAN.endPacket();
}

void sendPCM_CRUISE_2() {
  // Werte für die Signale setzen
  bool brake_pressed = BRAKE_PRESSED;   // BRAKE_PRESSED Signal, basierend auf der existierenden Logik
  uint8_t pcm_follow_distance = 2;     // Beispielwert für PCM_FOLLOW_DISTANCE (kann angepasst werden)
  uint8_t low_speed_lockout = 1;       // Beispielwert für LOW_SPEED_LOCKOUT (kann angepasst werden)
  bool main_on = MAIN_ON;              // MAIN_ON Signal
  uint8_t set_speed = ::set_speed;     // SET_SPEED Signal, basierend auf der existierenden Logik
  bool acc_faulted = false;            // Beispielwert für ACC_FAULTED (kann angepasst werden)

  // Nachricht zusammenstellen
  uint8_t dat_1d3[8] = {0};

  // Bits setzen
  dat_1d3[0] |= (brake_pressed << 3);               // BRAKE_PRESSED bei Bit 3
  dat_1d3[1] |= (pcm_follow_distance << 4);         // PCM_FOLLOW_DISTANCE bei Bits 12-13
  dat_1d3[1] |= (low_speed_lockout << 6);           // LOW_SPEED_LOCKOUT bei Bits 14-15
  dat_1d3[1] |= (main_on << 7);                     // MAIN_ON bei Bit 15
  dat_1d3[2] = set_speed;                           // SET_SPEED bei Bits 23-30
  dat_1d3[5] |= (acc_faulted << 7);                 // ACC_FAULTED bei Bit 47

  // Checksumme berechnen
  dat_1d3[7] = dbc_checksum(dat_1d3, 7, 0x1D3);     // CHECKSUM

  // Nachricht senden
  CAN.beginPacket(0x1D3);
  for (int i = 0; i < 8; i++) {
    CAN.write(dat_1d3[i]);
  }
  CAN.endPacket();
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

// Die restlichen Funktionen bleiben gleich...
