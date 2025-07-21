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
uint8_t PCM_FOLLOW_DISTANCE = 0;
uint8_t LOW_SPEED_LOCKOUT = 0;
bool ACC_FAULTED = false;

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
const uint16_t monitoredIDs[] = {0x262, 0x25, 0x260};
const int idCount = sizeof(monitoredIDs) / sizeof(monitoredIDs[0]);

// --- Timer Variables for Regular Intervals ---
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; // Interval in milliseconds

// --- Timer Variable for Smoothing ---
static unsigned long lastUpdate = 0; // Timer for smoothing

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

  // --- Geschwindigkeit glätten alle 10 ms ---
  if (millis() - lastUpdate > 10) { // Alle 10 ms
    total -= readings[readIndex];
    readings[readIndex] = spd;
    total += readings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    average = total / numReadings;

    lastUpdate = millis();
  }

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

  // --- Nachrichten in Intervallen senden ---
  unsigned long currentMillis = millis();
  if (currentMillis - lastSendTime >= sendInterval) {
    sendCANMessages();
    lastSendTime = currentMillis;
  }

  // --- Überwachung der CAN-Nachrichten ---
  monitorCANMessages();
}

// --- Nachrichten senden ---
void sendCANMessages() {
  sendPCM_CRUISE();
  waitForCANReady();
  
  sendPCM_CRUISE_2();
  waitForCANReady();
  
  sendWHEEL_SPEEDS();
  waitForCANReady();
  
  sendLIGHT_STALK();
  waitForCANReady();
  
  sendBLINKERS_STATE();
  waitForCANReady();
  
  sendBODY_CONTROL_STATE();
  waitForCANReady();
  
  sendBODY_CONTROL_STATE_2();
  waitForCANReady();
  
  sendESP_CONTROL();
  waitForCANReady();
  
  sendBRAKE_MODULE();
  waitForCANReady();
  
  sendPCM_CRUISE_SM();
  waitForCANReady();
  
  sendVSC1S07();
  waitForCANReady();
  
  sendENGINE_RPM();
  waitForCANReady();
  
  sendGEAR_PACKET();
  waitForCANReady();
  
  sendPRE_COLLISION_2();
}

// --- Warten, bis der CAN-Bus bereit ist ---
void waitForCANReady() {
  while (!CAN.availableForWrite()) {
    // Aktiv warten, bis der CAN-Bus bereit ist
  }
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
  // Werte für die Signale setzen
  bool gas_released = GAS_RELEASED;  // GAS_RELEASED Signal
  bool cruise_active = OP_ON;       // CRUISE_ACTIVE Signal
  bool acc_braking = false;         // Beispielwert für ACC_BRAKING (kann angepasst werden)
  uint16_t accel_net = 0;           // Beispielwert für ACCEL_NET (kann angepasst werden)
  int16_t neutral_force = 0;        // Beispielwert für NEUTRAL_FORCE (kann angepasst werden)
  uint8_t cruise_state = 3;         // Beispielwert für CRUISE_STATE (kann angepasst werden)
  bool cancel_req = false;          // Beispielwert für CANCEL_REQ (kann angepasst werden)

  // Nachricht zusammenstellen
  uint8_t dat_1d2[8] = {0};

  // Bits setzen
  dat_1d2[0] |= (gas_released << 4);                // GAS_RELEASED bei Bit 4
  dat_1d2[0] |= (cruise_active << 5);               // CRUISE_ACTIVE bei Bit 5
  dat_1d2[1] |= (acc_braking << 4);                 // ACC_BRAKING bei Bit 12
  dat_1d2[2] = accel_net >> 8;                      // ACCEL_NET (MSB)
  dat_1d2[3] = accel_net & 0xFF;                    // ACCEL_NET (LSB)
  dat_1d2[4] = neutral_force >> 8;                  // NEUTRAL_FORCE (MSB)
  dat_1d2[5] = neutral_force & 0xFF;                // NEUTRAL_FORCE (LSB)
  dat_1d2[6] |= (cancel_req << 1);                  // CANCEL_REQ bei Bit 49
  dat_1d2[6] |= (cruise_state & 0x0F);              // CRUISE_STATE bei Bits 55-58

  // Checksumme berechnen
  dat_1d2[7] = dbc_checksum(dat_1d2, 7, 0x1D2);     // CHECKSUM

  // Nachricht senden
  CAN.beginPacket(0x1D2);
  for (int i = 0; i < 8; i++) {
    CAN.write(dat_1d2[i]);
  }
  CAN.endPacket();
}

void sendPCM_CRUISE_2() {
  uint8_t dat_1d3[8] = {0};

  // Signale gemäß DBC setzen
  dat_1d3[0] |= BRAKE_PRESSED << 3;                     // BRAKE_PRESSED bei Bit 3
  dat_1d3[1] |= (PCM_FOLLOW_DISTANCE & 0x03) << 4;      // PCM_FOLLOW_DISTANCE bei Bits 12-13
  dat_1d3[1] |= (LOW_SPEED_LOCKOUT & 0x03) << 6;        // LOW_SPEED_LOCKOUT bei Bits 14-15
  dat_1d3[1] |= MAIN_ON << 7;                           // MAIN_ON bei Bit 15
  dat_1d3[2] = set_speed;                               // SET_SPEED bei Byte 2
  dat_1d3[5] |= ACC_FAULTED << 7;                       // ACC_FAULTED bei Bit 47

  // Checksumme berechnen
  dat_1d3[7] = dbc_checksum(dat_1d3, 7, 0x1D3);

  // Nachricht senden
  CAN.beginPacket(0x1D3);
  for (int i = 0; i < 8; i++) CAN.write(dat_1d3[i]);
  CAN.endPacket();
}

void sendWHEEL_SPEEDS() {
  uint16_t ws_kph = (uint16_t)(average * 100);
  uint8_t dat_170[8];
  for (int i = 0; i < 4; i++) {
    dat_170[i * 2] = ws_kph >> 8;
    dat_170[i * 2 + 1] = ws_kph & 0xFF;
  }
  CAN.beginPacket(0xAA); for (int i = 0; i < 8; i++) CAN.write(dat_170[i]); CAN.endPacket();
}

void sendLIGHT_STALK() {
  uint8_t dat_1570[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00}; // Simuliert: Abblendlicht, Tagfahrlicht
  CAN.beginPacket(0x622); for (int i = 0; i < 8; i++) CAN.write(dat_1570[i]); CAN.endPacket();
}

void sendBLINKERS_STATE() {
  uint8_t dat_1556[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, (blinker_left << 1) | blinker_right};
  CAN.beginPacket(0x614); for (int i = 0; i < 8; i++) CAN.write(dat_1556[i]); CAN.endPacket();
}

void sendBODY_CONTROL_STATE() {
  uint8_t dat_1568[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Keine offenen Türen, kein Parken
  CAN.beginPacket(0x620); for (int i = 0; i < 8; i++) CAN.write(dat_1568[i]); CAN.endPacket();
}

void sendBODY_CONTROL_STATE_2() {
  uint8_t dat_1552[8] = {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x01}; // Simuliert: Helligkeit auf Maximum
  CAN.beginPacket(0x610); for (int i = 0; i < 8; i++) CAN.write(dat_1552[i]); CAN.endPacket();
}

void sendESP_CONTROL() {
  uint8_t dat_951[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Standard ESP-Zustand
  CAN.beginPacket(0x3B7); for (int i = 0; i < 8; i++) CAN.write(dat_951[i]); CAN.endPacket();
}

void sendBRAKE_MODULE() {
  uint8_t dat_548[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Keine Bremse gedrückt
  CAN.beginPacket(0x224); for (int i = 0; i < 8; i++) CAN.write(dat_548[i]); CAN.endPacket();
}

void sendPCM_CRUISE_SM() {
  uint8_t dat_921[8] = {0x00, 0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Tempomat aktiv
  CAN.beginPacket(0x399); for (int i = 0; i < 8; i++) CAN.write(dat_921[i]); CAN.endPacket();
}

void sendVSC1S07() {
  uint8_t dat_800[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Stabilisierungsmodus
  CAN.beginPacket(0x320); for (int i = 0; i < 8; i++) CAN.write(dat_800[i]); CAN.endPacket();
}

void sendENGINE_RPM() {
  uint16_t rpm = 3000; // 3000 U/min
  uint8_t dat_452[8] = {rpm >> 8, rpm & 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  CAN.beginPacket(0x1C4); for (int i = 0; i < 8; i++) CAN.write(dat_452[i]); CAN.endPacket();
}

void sendGEAR_PACKET() {
  uint8_t dat_956[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}; // Simuliert: Gang eingelegt
  CAN.beginPacket(0x3BC); for (int i = 0; i < 8; i++) CAN.write(dat_956[i]); CAN.endPacket();
}

void sendPRE_COLLISION_2() {
  uint8_t dat_836[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Simuliert: Normalzustand
  CAN.beginPacket(0x344); for (int i = 0; i < 8; i++) CAN.write(dat_836[i]); CAN.endPacket();
}
