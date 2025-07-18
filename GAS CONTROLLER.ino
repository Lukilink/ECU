// GAS PEDAL CONTROL UNIT - DO NOT USE ON PUBLIC ROADS
#include <CAN.h>

//__________ CONFIGURATION VALUES
const int PERM_ERROR = 8;                   // Allowed position error
const int minPot = 86;                      // Minimum potentiometer reading
const int maxPot = 920;                     // Maximum potentiometer reading
const float maxACC_CMD = 2.5f;              // max m/s² considered (scaled from ACC_CMD)
const float minACC_CMD = 0.0f;              // min m/s² considered
const int user_input_threshold = 150;       // Threshold for user pedal override

//__________ PIN ASSIGNMENTS
const int cancel_pin = 3;                   // Cancel signal (pedal pressed by user)
const int potPin = A3;                      // Throttle potentiometer
const int M_DIR = 8;                        // Motor direction
const int M_PWM = 9;                        // Motor power
const int S_DIR = 7;                        // Solenoid direction (always LOW)
const int S_PWM = 6;                        // Solenoid power

//__________ STATE VARIABLES
int targetPosition = 0;
int potiPosition = 0;
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
bool cancel = false;
bool GAS_RELEASED = false;

void setup() {
  // Init CAN
  CAN.begin(500E3);
  CAN.filter(0x343); // ID for ACC_CONTROL (ACCEL_CMD)

  // Init pins
  pinMode(cancel_pin, INPUT_PULLUP);
  pinMode(potPin, INPUT);    
  pinMode(M_DIR, OUTPUT);
  pinMode(M_PWM, OUTPUT);
  pinMode(S_DIR, OUTPUT);
  pinMode(S_PWM, OUTPUT);
  digitalWrite(S_DIR, LOW);
}

void loop() {
  // Read cancel pin
  cancel = digitalRead(cancel_pin);

  // Read potentiometer position
  potiPosition = analogRead(potPin);

  // Read ACC_CMD from CAN (0x343)
  if (CAN.parsePacket() && CAN.packetId() == 0x343) {
    uint8_t dat[8];
    for (int i = 0; i < 8; i++) dat[i] = CAN.read();

    // Signal: ACCEL_CMD : 7|16@0- (0.001,0) [-20|20]
    // Extract Motorola 16-bit signed (big endian)
    int16_t raw = (dat[0] << 8) | dat[1];
    ACC_CMD = raw * 0.001f; // Scale to m/s²
  }

  // Calculate percentage and target position
  float clampedACC = constrain(ACC_CMD, minACC_CMD, maxACC_CMD);
  ACC_CMD_PERCENT = ((clampedACC - minACC_CMD) / (maxACC_CMD - minACC_CMD)) * 100.0f;
  targetPosition = ((ACC_CMD_PERCENT / 100.0f) * (maxPot - minPot)) + minPot;

  // Actuator logic
  if (!cancel || ACC_CMD_PERCENT == 0) {
    analogWrite(S_PWM, 0);
    analogWrite(M_PWM, 0);
  } else {
    analogWrite(S_PWM, 255);
    if (abs(potiPosition - targetPosition) >= PERM_ERROR) {
      if (potiPosition < targetPosition && potiPosition < maxPot) {
        digitalWrite(M_DIR, LOW);
        analogWrite(M_PWM, 255);
      } else if (potiPosition > targetPosition && potiPosition >= minPot) {
        digitalWrite(M_DIR, HIGH);
        analogWrite(M_PWM, 255);
      }
    } else {
      analogWrite(M_PWM, 0);
    }
  }

  // Detect user override
  GAS_RELEASED = !(potiPosition >= (targetPosition + user_input_threshold));

  // Send CAN message 0x2C1 with GAS_RELEASED bit
  sendGasPedalStatus();
}

void sendGasPedalStatus() {
  // GAS_RELEASED at bit 3 of byte 3 (startbit 27)
  uint8_t dat[8] = { 0, 0, 0, (GAS_RELEASED << 3) & 0x08, 0, 0, 0, 0 };
  CAN.beginPacket(0x2C1);
  for (int i = 0; i < 8; i++) CAN.write(dat[i]);
  CAN.endPacket();
}
