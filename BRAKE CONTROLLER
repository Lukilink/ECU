// BRAKE CONTROL UNIT - DO NOT USE ON PUBLIC ROADS
#include <CAN.h>

//__________ CONFIGURATION VALUES
const int PERM_ERROR = 2;              // Tolerance between target and current pressure
const int maxPressure = 190;           // Maximum pressure actuator can apply
const int minPressure = 100;           // Resting pressure
const float maxACC_CMD = 500.0f;       // Max command from OpenPilot (0x343)
const float minACC_CMD = 0.0f;         // Min command
const int brake_pressed_threshold = 5; // Difference to detect driver input
const int brake_light_threshold = 5;   // Pressure to activate brake light

//__________ PIN ASSIGNMENTS
const int pressurePin = A2;
const int brakelightPin = A5;
const int M_DIR = 8;
const int M_PWM = 9;
const int S_DIR = 7;
const int S_PWM = 6;

//__________ STATE VARIABLES
float targetPressure = 0;
float currentPressure = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
float ACC_CMD_PERCENT = 0;
bool BRAKE_PRESSED = true;
bool releasing_by_OP = false;
bool open_solenoid = true;
unsigned long previousMillis = 0;

void setup() {
  CAN.begin(500E3);
  CAN.filter(0x343);

  pinMode(pressurePin, INPUT);
  pinMode(M_DIR, OUTPUT);
  pinMode(M_PWM, OUTPUT);
  pinMode(S_DIR, OUTPUT);
  pinMode(S_PWM, OUTPUT);
  digitalWrite(S_DIR, HIGH);
  pinMode(brakelightPin, OUTPUT);
}

void loop() {
  // Read pressure
  currentPressure = analogRead(pressurePin);

  // Read ACC_CMD from CAN
  if (CAN.parsePacket() && CAN.packetId() == 0x343) {
    uint8_t dat[8];
    for (int i = 0; i < 8; i++) dat[i] = CAN.read();
    ACC_CMD = ((dat[0] << 8) | dat[1]) * -1; // Inverted pressure command
  }

  // Clamp and scale ACC_CMD
  ACC_CMD1 = (ACC_CMD >= minACC_CMD) ? ACC_CMD : minACC_CMD;
  ACC_CMD_PERCENT = ((100.0f / (maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));
  targetPressure = ((ACC_CMD_PERCENT / 100.0f) * (maxPressure - minPressure)) + minPressure;

  // Match actuator to target pressure
  if (abs(currentPressure - targetPressure) >= PERM_ERROR) {
    open_solenoid = false;
    analogWrite(M_PWM, 255);
    if (currentPressure < targetPressure && ACC_CMD_PERCENT > 0) {
      digitalWrite(M_DIR, HIGH); // Press pedal
      releasing_by_OP = false;
    } else {
      digitalWrite(M_DIR, LOW);  // Release pedal
      releasing_by_OP = ACC_CMD_PERCENT > 0;
    }
  } else {
    analogWrite(M_PWM, 0);
    open_solenoid = false;
    releasing_by_OP = false;
  }

  // Detect manual override
  if ((currentPressure >= (targetPressure + brake_pressed_threshold)) && !releasing_by_OP) {
    BRAKE_PRESSED = true;
    open_solenoid = true;
  } else {
    BRAKE_PRESSED = false;
  }

  // Control solenoid
  analogWrite(S_PWM, open_solenoid ? 0 : 255);

  // Brake lights
  if (millis() - previousMillis >= 200) {
    analogWrite(brakelightPin, (currentPressure >= (minPressure + brake_light_threshold)) ? 255 : 0);
    previousMillis = millis();
  }

  // Send ESP_CONTROL 0x3B7
  sendBrakeCanMessage();
}

void sendBrakeCanMessage() {
  // BRAKE_PRESSED at bit 5 of byte 0
  uint8_t dat[8] = { (BRAKE_PRESSED << 5) & 0x20, 0, 0, 0, 0, 0, 0, 0x08 };
  CAN.beginPacket(0x3B7);
  for (int i = 0; i < 8; i++) CAN.write(dat[i]);
  CAN.endPacket();
}
