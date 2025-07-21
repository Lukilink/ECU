// MAIN ECU - DBC Compliant Version
// Adapted to conform with toyota_new_mc_pt_generated.txt DBC specifications
#include <CAN.h>

//______________DBC MESSAGE IDs (decimal values from DBC)
const uint16_t ID_PCM_CRUISE       = 466;   // 0x1D2
const uint16_t ID_PCM_CRUISE_2     = 467;   // 0x1D3  
const uint16_t ID_WHEEL_SPEEDS     = 170;   // 0xAA
const uint16_t ID_STEERING_LEVERS  = 1556;  // 0x614
const uint16_t ID_BRAKE_MODULE     = 548;   // 0x224
const uint16_t ID_GAS_PEDAL        = 705;   // 0x2C1
const uint16_t ID_STEERING_IPAS_COMMA = 1553; // 0x611
const uint16_t ID_SEATS_DOORS      = 1568;  // 0x620
const uint16_t ID_GEAR_PACKET      = 956;   // 0x3BC

//______________CONTROL CONSTANTS
const int LCR_MINIMUM_SPEED = 80;
const int LCR_SPEED_DIFF = 15;
const int LCR_LEAD_DISTANCE = 100;
const uint8_t MAX_SET_SPEED = 200;

//______________PIN DEFINITIONS
const int BLINKER_PIN_LEFT = 4;
const int BLINKER_PIN_RIGHT = 5;
const int BUTTON_1 = 9;  // Cancel/Off
const int BUTTON_2 = 6;  // Speed down
const int BUTTON_3 = 7;  // Speed up  
const int BUTTON_4 = 8;  // Main/Set
const int CLUTCH_SWITCH = A4;
const int VSS_HALL_SENSOR_INTERRUPT_PIN = 3;

//______________VSS SENSOR CONFIGURATION
#define VSS_SENSOR_SMOOTHING 3
#define VSS_MAX_SPEED 160.0f
#define VSS_DISTANCE_PER_REVOLUTION 0.135f
const int VSS_RINGBUFFER_SIZE = 4;
const int VSS_REFRESH_RATE_MS = 200;

//______________STATE VARIABLES
bool clutchSwitchState = false;
bool lastGasReleased = false;
bool lastBrakePressed = false;
long lastBlinkerRight = 0;
long lastBlinkerLeft = 0;

// Control states
bool opOn = false;
bool mainOn = true;
uint8_t setSpeed = 0x0;
bool blinkerLeftOn = false;
bool blinkerRightOn = false;

// CAN received signals
bool brakePressed = true;
bool gasReleased = false;
uint16_t gasCommand = 0;
float leadLongDist = 0;
float leadRelSpeed = 0;

// VSS sensor variables
float vssRingBuffer[VSS_RINGBUFFER_SIZE];
float vssSpeedKMH = 0;
float vssSpeedSum = 0;
float vssAvgSpeedKMH = 0;
float lastValidVssSpeedKMH = 0;
int vssRingBufferIndex = 0;
unsigned long vssDuration = 0;
unsigned long lastVssRefresh = 0;
unsigned long lastValidVssSpeedTs = 0;
volatile uint8_t vssSensorRevolutions = 0;
volatile unsigned long vssLastTriggerMicros = 0;
unsigned long vssLastUnhandledTriggerMicros = 0;

// Button states
int buttonState1, lastButtonState1;
int buttonState2, lastButtonState2;
int buttonState3, lastButtonState3;
int buttonState4, lastButtonState4;

//______________VSS SENSOR INTERRUPT
void interruptVssSensor() {
  vssSensorRevolutions++;
  vssLastTriggerMicros = micros();
}

//______________TOYOTA DBC CHECKSUM CALCULATION
uint8_t toyotaDbcChecksum(uint8_t *data, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr >> 8) + (addr & 0xFF) + len + 1);
  for (int i = 0; i < len; i++) {
    checksum += data[i];
  }
  return checksum;
}

//______________CAN MESSAGE SENDER
void sendCanMessage(uint16_t id, uint8_t* data, uint8_t len) {
  CAN.beginPacket(id);
  for (uint8_t i = 0; i < len; i++) {
    CAN.write(data[i]);
  }
  CAN.endPacket();
}

//______________VSS SENSOR UPDATE (enhanced smoothing)
void updateVssSensor() {
  #if VSS_SENSOR_SMOOTHING==0 || VSS_SENSOR_SMOOTHING==1
    if (vssSensorRevolutions > 0) {
      vssDuration = (micros() - vssLastUnhandledTriggerMicros);
      uint8_t SaveSREG = SREG;
      noInterrupts();
      uint8_t tmpVssSensorRevolutions = vssSensorRevolutions;
      vssLastUnhandledTriggerMicros = vssLastTriggerMicros;
      vssSensorRevolutions -= tmpVssSensorRevolutions;
      SREG = SaveSREG;

      vssSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
      #if VSS_SENSOR_SMOOTHING==1
        vssSpeedKMH = max(min(vssSpeedKMH, vssAvgSpeedKMH+10), vssAvgSpeedKMH-10);
      #endif
    }
    else if (micros()-vssLastUnhandledTriggerMicros > 1000L*1000L) {
      vssSpeedKMH = 0;
    }
  #elif VSS_SENSOR_SMOOTHING==2 || VSS_SENSOR_SMOOTHING==3
    if (vssSensorRevolutions > 0) {
      vssDuration = (vssLastTriggerMicros - vssLastUnhandledTriggerMicros);
      uint8_t SaveSREG = SREG;
      noInterrupts();
      uint8_t tmpVssSensorRevolutions = vssSensorRevolutions;
      vssLastUnhandledTriggerMicros = vssLastTriggerMicros;
      vssSensorRevolutions -= tmpVssSensorRevolutions;
      SREG = SaveSREG;

      float tmpSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
      if (tmpSpeedKMH <= VSS_MAX_SPEED) {
        vssSpeedKMH = max(vssSpeedKMH, tmpSpeedKMH);
      }
      #if VSS_SENSOR_SMOOTHING==3
        vssSpeedKMH = max(min(vssSpeedKMH, vssAvgSpeedKMH+10), vssAvgSpeedKMH-10);
      #endif
    }
    else if (micros()-vssLastUnhandledTriggerMicros > 1000L*1000L) {
      vssSpeedKMH = 0;
    }
  #endif

  if (millis()-lastVssRefresh >= VSS_REFRESH_RATE_MS) {
    lastVssRefresh = millis();
    
    if (vssSpeedKMH > 0) {
      lastValidVssSpeedKMH = vssSpeedKMH;
      lastValidVssSpeedTs = millis();
    }
    else if (vssSpeedKMH == 0 && lastValidVssSpeedKMH > 0 && millis()-lastValidVssSpeedTs < 1000) {
      vssSpeedKMH = lastValidVssSpeedKMH;
    }

    vssSpeedSum -= vssRingBuffer[vssRingBufferIndex];
    vssSpeedSum += vssSpeedKMH;
    vssRingBuffer[vssRingBufferIndex] = vssSpeedKMH;
    vssSpeedKMH = 0;
    vssRingBufferIndex++;
    if (vssRingBufferIndex >= VSS_RINGBUFFER_SIZE) {
      vssRingBufferIndex = 0;
    }
    vssAvgSpeedKMH = vssSpeedSum / VSS_RINGBUFFER_SIZE;
  }
}

//______________SEND DBC COMPLIANT CAN MESSAGES
void sendDbcMessages() {
  // PCM_CRUISE (ID: 466, 0x1D2)
  uint8_t pcmCruise[8] = {0};
  pcmCruise[0] = (gasReleased << 4) | (opOn << 5);
  // ACCEL_NET field (16 bits at bit 16, signed, factor 0.001)
  int16_t accelNet = 0; // No acceleration command for now
  pcmCruise[2] = (accelNet >> 8) & 0xFF;
  pcmCruise[3] = accelNet & 0xFF;
  // CRUISE_STATE field (4 bits at bit 55)
  uint8_t cruiseState = opOn ? 7 : 2; // 7=ACTIVE, 2=DISABLED
  pcmCruise[6] = (cruiseState << 7) | (pcmCruise[6] & 0x0F);
  pcmCruise[7] = toyotaDbcChecksum(pcmCruise, 7, ID_PCM_CRUISE);
  sendCanMessage(ID_PCM_CRUISE, pcmCruise, 8);

  // PCM_CRUISE_2 (ID: 467, 0x1D3)
  uint8_t pcmCruise2[8] = {0};
  pcmCruise2[1] = (mainOn << 7) | 0x28; // MAIN_ON at bit 15
  pcmCruise2[2] = setSpeed; // SET_SPEED at byte 2
  pcmCruise2[7] = toyotaDbcChecksum(pcmCruise2, 7, ID_PCM_CRUISE_2);
  sendCanMessage(ID_PCM_CRUISE_2, pcmCruise2, 8);

  // WHEEL_SPEEDS (ID: 170, 0xAA) - DBC compliant format
  uint8_t wheelSpeeds[8] = {0};
  uint16_t wheelSpeedRaw = (uint16_t)(vssAvgSpeedKMH / 0.01); // factor 0.01 from DBC
  // All 4 wheels same speed for simplicity
  wheelSpeeds[0] = (wheelSpeedRaw >> 8) & 0xFF; // FL high
  wheelSpeeds[1] = wheelSpeedRaw & 0xFF;        // FL low
  wheelSpeeds[2] = (wheelSpeedRaw >> 8) & 0xFF; // FR high
  wheelSpeeds[3] = wheelSpeedRaw & 0xFF;        // FR low
  wheelSpeeds[4] = (wheelSpeedRaw >> 8) & 0xFF; // RL high
  wheelSpeeds[5] = wheelSpeedRaw & 0xFF;        // RL low
  wheelSpeeds[6] = (wheelSpeedRaw >> 8) & 0xFF; // RR high
  wheelSpeeds[7] = wheelSpeedRaw & 0xFF;        // RR low
  sendCanMessage(ID_WHEEL_SPEEDS, wheelSpeeds, 8);

  // STEERING_LEVERS (ID: 1556, 0x614)
  uint8_t steeringLevers[8] = {0x29, 0x00, 0x01, 0x00, 0x00, 0x00, 0x76, 0x00};
  // TURN_SIGNALS at bit 24 (byte 3)
  steeringLevers[3] = (blinkerLeftOn << 5) | (blinkerRightOn << 4);
  steeringLevers[7] = toyotaDbcChecksum(steeringLevers, 7, ID_STEERING_LEVERS);
  sendCanMessage(ID_STEERING_LEVERS, steeringLevers, 8);

  // STEERING_IPAS_COMMA (ID: 1553, 0x611) - New message from DBC
  uint8_t steeringIpasComma[8] = {0};
  steeringIpasComma[0] = 0x08; // STATE = standby
  steeringIpasComma[2] = 0x10; // SET_ME_X10
  steeringIpasComma[3] = 0x40; // SET_ME_X40
  steeringIpasComma[3] |= (opOn << 7); // LKA_STATE
  steeringIpasComma[4] = toyotaDbcChecksum(steeringIpasComma, 4, ID_STEERING_IPAS_COMMA);
  sendCanMessage(ID_STEERING_IPAS_COMMA, steeringIpasComma, 5);

  // SEATS_DOORS (ID: 1568, 0x620) - DBC compliant
  uint8_t seatsDoors[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  // All doors closed, seatbelt latched
  sendCanMessage(ID_SEATS_DOORS, seatsDoors, 8);

  // GEAR_PACKET (ID: 956, 0x3BC) - DBC compliant
  uint8_t gearPacket[8] = {0};
  gearPacket[1] = 32; // GEAR = D (Drive), positioned at bit 13
  sendCanMessage(ID_GEAR_PACKET, gearPacket, 8);
}

//______________PROCESS RECEIVED CAN MESSAGES
void processReceivedMessages() {
  while (CAN.parsePacket()) {
    uint16_t canId = CAN.packetId();
    uint8_t data[8] = {0};
    int i = 0;
    while (CAN.available() && i < 8) {
      data[i++] = CAN.read();
    }

    switch (canId) {
      case ID_BRAKE_MODULE: // 548 (0x224)
        // BRAKE_PRESSED signal at bit 32 according to DBC
        brakePressed = (data[4] & 0x01) != 0;
        break;

      case ID_GAS_PEDAL: // 705 (0x2C1)
        // GAS_COMMAND at bit 0, 16 bits, factor 0.005
        gasCommand = (data[0] << 8) | data[1];
        // GAS_RELEASED at bit 32
        gasReleased = (data[4] & 0x01) != 0;
        break;

      // Handle other received messages as needed
      default:
        break;
    }
  }
}

//______________ARDUINO SETUP
void setup() {
  Serial.begin(9600);
  CAN.begin(500E3);

  // Initialize pins
  pinMode(BUTTON_1, INPUT_PULLUP);
  pinMode(BUTTON_2, INPUT_PULLUP);
  pinMode(BUTTON_3, INPUT_PULLUP);
  pinMode(BUTTON_4, INPUT_PULLUP);
  pinMode(BLINKER_PIN_LEFT, INPUT_PULLUP);
  pinMode(BLINKER_PIN_RIGHT, INPUT_PULLUP);
  pinMode(CLUTCH_SWITCH, INPUT_PULLUP);

  // Initialize VSS sensor
  pinMode(VSS_HALL_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_HALL_SENSOR_INTERRUPT_PIN), interruptVssSensor, FALLING);

  // Initialize VSS ring buffer
  for (int i = 0; i < VSS_RINGBUFFER_SIZE; i++) {
    vssRingBuffer[i] = 0;
  }

  Serial.println("DBC-Compliant ECU initialized");
}

//______________MAIN LOOP
void loop() {
  // Update VSS sensor
  updateVssSensor();

  // Read input states
  clutchSwitchState = digitalRead(CLUTCH_SWITCH) == LOW;
  buttonState1 = digitalRead(BUTTON_1);
  buttonState2 = digitalRead(BUTTON_2);
  buttonState3 = digitalRead(BUTTON_3);
  buttonState4 = digitalRead(BUTTON_4);

  // Handle blinker inputs with debouncing
  bool rawBlinkerLeft = digitalRead(BLINKER_PIN_LEFT) == LOW;
  bool rawBlinkerRight = digitalRead(BLINKER_PIN_RIGHT) == LOW;
  
  if (rawBlinkerLeft) lastBlinkerLeft = millis();
  if (rawBlinkerRight) lastBlinkerRight = millis();
  
  blinkerLeftOn = (millis() - lastBlinkerLeft) < 500;
  blinkerRightOn = (millis() - lastBlinkerRight) < 500;

  // Safety: Disable OP when brake pressed or gas not released
  if (brakePressed || !gasReleased) {
    opOn = false;
  }

  // Handle button presses
  if (buttonState4 != lastButtonState4 && buttonState4 == LOW) {
    // Main/Set button
    if (!opOn) {
      opOn = true;
      setSpeed = (uint8_t)(vssAvgSpeedKMH + 3);
    } else {
      opOn = false;
    }
  }

  if (buttonState3 != lastButtonState3 && buttonState3 == LOW) {
    // Speed up
    if (setSpeed < MAX_SET_SPEED - 5) setSpeed += 5;
  }

  if (buttonState2 != lastButtonState2 && buttonState2 == LOW) {
    // Speed down
    if (setSpeed > 5) setSpeed -= 5;
  }

  if (buttonState1 != lastButtonState1 && buttonState1 == LOW) {
    // Cancel/Off
    opOn = false;
  }

  // Limit set speed
  if (setSpeed > MAX_SET_SPEED) setSpeed = 0;

  // Update button states
  lastButtonState1 = buttonState1;
  lastButtonState2 = buttonState2;
  lastButtonState3 = buttonState3;
  lastButtonState4 = buttonState4;

  // Process received CAN messages
  processReceivedMessages();

  // Send DBC compliant CAN messages
  sendDbcMessages();

  // Small delay for stability
  delay(20);
}