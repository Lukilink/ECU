// MAIN ECU
#include <CAN.h>

//______________LCR VARIABLES
const int LCR_minimum_speed = 80;
const int LCR_speed_diff = 15;
const int LCR_lead_distance = 100;

//______________BUTTONS / SWITCHES
const int BlinkerPinLeft = 4;
const int BlinkerPinRight = 5;
const int button1 = 9;
const int button2 = 6;
const int button3 = 7;
const int button4 = 8;
const int ClutchSwitch = A4;

//______________CAN CONSTANTS
const uint16_t ID_PCM_CRUISE     = 0x1D2;
const uint16_t ID_PCM_CRUISE_2   = 0x1D3;
const uint16_t ID_WHEEL_SPEEDS   = 0xAA;
const uint16_t ID_SEATS_DOORS    = 0x620;
const uint16_t ID_GEAR_PACKET    = 0x3BC;
const uint16_t ID_STEERING_LEVERS= 0x614;
const uint16_t ID_ENGINE_DATA    = 0x191;
const uint16_t ID_ACCEL_POS      = 0x2B0;
const uint16_t ID_UNKNOWN_ECU    = 0x1F5;

const uint8_t MAX_SET_SPEED = 200;

//______________STATES
bool ClutchSwitchState = false;
bool lastGAS_RELEASED = false;
bool lastBRAKE_PRESSED = false;
long last_blinker_right;
long last_blinker_left;

bool OP_ON = false;
bool MAIN_ON = true;
uint8_t set_speed = 0x0;
bool blinker_left_on = true;
bool blinker_right_on = true;
float LEAD_LONG_DIST = 0;
float LEAD_REL_SPEED = 0;
bool BRAKE_PRESSED = true;
bool GAS_RELEASED = false;

//______________VSS SENSOR
const int VSS_HALL_SENSOR_INTERRUPT_PIN = 3;
#define VSS_SENSOR_SMOOTHING 3
#define VSS_MAX_SPEED 160.0f
#define VSS_DISTANCE_PER_REVOLUTION 0.135f
const int VSS_RINGBUFFER_SIZE = 4;
const int VSS_REFRESH_RATE_MS = 200;
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

//______________BUTTON STATES
int buttonstate1, lastbuttonstate1;
int buttonstate2, lastbuttonstate2;
int buttonstate3, lastbuttonstate3;
int buttonstate4, lastbuttonstate4;

//______________INTERRUPT
void interruptVssSensor() {
  vssSensorRevolutions++;
  vssLastTriggerMicros = micros();
}

//______________CAN CHECKSUM
uint8_t can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = ((addr >> 8) + (addr & 0xFF) + len + 1);
  for (int i = 0; i < len; i++) checksum += dat[i];
  return checksum;
}

//______________CAN SENDER
void sendCanMessage(uint16_t id, uint8_t* data, uint8_t len) {
  CAN.beginPacket(id);
  for (uint8_t i = 0; i < len; i++) CAN.write(data[i]);
  CAN.endPacket();
}

void sendAllCanMessages() {
  // PCM_CRUISE
  uint8_t dat_1d2[8] = { (OP_ON << 5) & 0x20 | (GAS_RELEASED << 4) & 0x10, 0, 0, 0, 0, 0, (OP_ON << 7) & 0x80, 0 };
  dat_1d2[7] = can_cksum(dat_1d2, 7, ID_PCM_CRUISE);
  sendCanMessage(ID_PCM_CRUISE, dat_1d2, 8);

  // PCM_CRUISE_2
  uint8_t dat_1d3[8] = { 0, (MAIN_ON << 7) & 0x80 | 0x28, set_speed, 0, 0, 0, 0, 0 };
  dat_1d3[7] = can_cksum(dat_1d3, 7, ID_PCM_CRUISE_2);
  sendCanMessage(ID_PCM_CRUISE_2, dat_1d3, 8);

  // WHEEL_SPEEDS
  uint16_t ws = 0x1A6F + (uint16_t)(vssAvgSpeedKMH * 100);
  uint8_t dat_aa[8];
  for (int i = 0; i < 4; i++) {
    dat_aa[i * 2]     = (ws >> 8) & 0xFF;
    dat_aa[i * 2 + 1] = ws & 0xFF;
  }
  sendCanMessage(ID_WHEEL_SPEEDS, dat_aa, 8);

  // SEATS_DOORS (simulate all doors closed)
  uint8_t dat_620[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCanMessage(ID_SEATS_DOORS, dat_620, 8);

  // GEAR_PACKET (simulate neutral)
  uint8_t dat_3bc[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00 };
  sendCanMessage(ID_GEAR_PACKET, dat_3bc, 8);

  // STEERING_LEVERS
  uint8_t dat_614[8] = { 0x29, 0x00, 0x01, (blinker_left_on << 5) | (blinker_right_on << 4), 0, 0, 0x76, 0 };
  dat_614[7] = can_cksum(dat_614, 7, ID_STEERING_LEVERS);
  sendCanMessage(ID_STEERING_LEVERS, dat_614, 8);

  // ENGINE_DATA (simulate static RPM)
  uint8_t dat_191[8] = { 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCanMessage(ID_ENGINE_DATA, dat_191, 8);

  // ACCEL_POS (simulate pedal position 0%)
  uint8_t dat_2b0[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  sendCanMessage(ID_ACCEL_POS, dat_2b0, 8);

  // UNKNOWN_ECU (dummy traffic)
  uint8_t dat_1f5[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0x00, 0x00, 0x00, 0x00 };
  sendCanMessage(ID_UNKNOWN_ECU, dat_1f5, 8);
}

//______________LOOP
void loop() {
  loopUpdateVssSensor();

  ClutchSwitchState = digitalRead(ClutchSwitch);
  buttonstate1 = digitalRead(button1);
  buttonstate2 = digitalRead(button2);
  buttonstate3 = digitalRead(button3);
  buttonstate4 = digitalRead(button4);

  bool raw_blinker_left = digitalRead(BlinkerPinLeft);
  bool raw_blinker_right = digitalRead(BlinkerPinRight);
  if (raw_blinker_left) last_blinker_left = millis();
  if (raw_blinker_right) last_blinker_right = millis();
  blinker_left_on = millis() - last_blinker_left < 500;
  blinker_right_on = millis() - last_blinker_right < 500;

  if (BRAKE_PRESSED || !GAS_RELEASED) OP_ON = false;

  if (buttonstate4 != lastbuttonstate4 && buttonstate4 == LOW) {
    OP_ON = !OP_ON;
    if (OP_ON) set_speed = vssAvgSpeedKMH + 3;
  }
  if (buttonstate3 != lastbuttonstate3 && buttonstate3 == LOW) set_speed += 5;
  if (buttonstate2 != lastbuttonstate2 && buttonstate2 == LOW) set_speed -= 5;
  if (set_speed > MAX_SET_SPEED) set_speed = 0;
  if (buttonstate1 != lastbuttonstate1 && buttonstate1 == LOW) OP_ON = false;

  lastbuttonstate1 = buttonstate1;
  lastbuttonstate2 = buttonstate2;
  lastbuttonstate3 = buttonstate3;
  lastbuttonstate4 = buttonstate4;
  lastBRAKE_PRESSED = BRAKE_PRESSED;
  lastGAS_RELEASED = GAS_RELEASED;

  sendAllCanMessages();

  while (CAN.parsePacket()) {
    uint8_t dat[8] = {0};
    int i = 0;
    while (CAN.available() && i < 8) {
      dat[i++] = CAN.read();
    }
    switch (CAN.packetId()) {
      case 0x3b7:
        BRAKE_PRESSED = dat[0] & 0x20;
        break;
      case 0x2c1:
        GAS_RELEASED = dat[0] & 0x08;
        break;
      case 0x2e6:
        LEAD_LONG_DIST = ((dat[0] << 8) | dat[1]) * 0.005;
        LEAD_REL_SPEED = ((dat[2] << 8) | dat[3]) * 0.009;
        break;
    }
  }
}
