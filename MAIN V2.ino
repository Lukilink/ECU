// MAIN ECU V2


#include <CAN.h>

//______________VALUES READ ON CAN
// -------- BRAKE_MODULE (0x224 / 548) -------- KOMMT DANN VON BRAKE ECU 
uint16_t BRAKE_PRESSURE = 0;
uint8_t BRAKE_PRESSED = 1;


boolean GAS_RELEASED = false;

//______________BUTTONS / SWITCHES / VALUES
int BlinkerPinLeft = 4;
int BlinkerPinRight = 5;
int button4 = 8;
int button3 = 7;
int button2 = 6;
int button1 = 9;
int CluchSwitch = A4;
boolean ClutchSwitchState = false;
int buttonstate4;
int lastbuttonstate4;
int buttonstate3;
int lastbuttonstate3;
int buttonstate2;
int lastbuttonstate2;
int buttonstate1;
int lastbuttonstate1;
boolean lastGAS_RELEASED = false;
boolean lastBRAKE_PRESSED = false;
long last_blinker_right;
long last_blinker_left;

// =========================
// ALTE VARIABLEN 
// =========================

// alt boolean OP_ON = false;
// Jetzt jetzt PCM_CRUISE_CRUISE_ACTIVE

// ALT uint8_t set_speed = 0x0;
// Jetzt PCM_CRUISE_2_SET_SPEED

// ALT boolean blinker_left_on = true;
// ALT boolean blinker_right_on = true;
// JETZT uint8_t BLINKERS_TURN_SIGNALS = 3; // TURN_SIGNALS: 1=links, 2=rechts, 3=none

//______________VALUES SEND ON CAN
// boolean MAIN_ON = true; // Kann raus weil wir eh nur Statisch


// =========================
// Konfigurierbare Variablen (aus DBC)
// =========================

// -------- PCM_CRUISE (0x1D2 / 466) --------
uint8_t PCM_CRUISE_GAS_RELEASED = 1; // GAS_RELEASED: 0 = kein Gas freigegeben, 1 = Gas freigegeben
boolean PCM_CRUISE_CRUISE_ACTIVE = 1; // CRUISE_ACTIVE: 0 = ACC aus, 1 = ACC aktiv
uint8_t PCM_CRUISE_ACC_BRAKING = 0; // ACC_BRAKING: 0 = kein ACC Bremsen, 1 = ACC bremst
float PCM_CRUISE_ACCEL_NET = 0.5; // m/s^2 // ACCEL_NET: [-20|20] m/s^2, Faktor: 0.0009765625, Offset: 0
int16_t PCM_CRUISE_NEUTRAL_FORCE = 0; // N // NEUTRAL_FORCE: [-65536|65534] N, Faktor: 2, Offset: 0
uint8_t PCM_CRUISE_CRUISE_STATE = 8; // CRUISE_STATE: 0=off, 1=non-adaptive engaged, 2=non-adaptive being engaged, 8=adaptive engaged, 11=timer_3sec, 7=standstill, 9/10=click up/down
uint8_t PCM_CRUISE_CANCEL_REQ = 0; // CANCEL_REQ: 0 = kein Cancel, 1 = Cancel

// -------- PCM_CRUISE_2 (0x1D3 / 467) --------
uint8_t PCM_CRUISE_2_BRAKE_PRESSED = 0; // BRAKE_PRESSED: 0 = nicht gedrückt, 1 = gedrückt
uint8_t PCM_CRUISE_2_PCM_FOLLOW_DISTANCE = 2; // PCM_FOLLOW_DISTANCE: 1=far, 2=medium, 3=close
uint8_t PCM_CRUISE_2_LOW_SPEED_LOCKOUT = 0; // LOW_SPEED_LOCKOUT: 0=none, 1=ok, 2=locked
uint8_t PCM_CRUISE_2_MAIN_ON = 1; // MAIN_ON: 0=aus, 1=ein
uint8_t PCM_CRUISE_2_SET_SPEED = 100; // SET_SPEED: [0|255] km/h
uint8_t PCM_CRUISE_2_ACC_FAULTED = 0; // ACC_FAULTED: 0=kein Fehler, 1=Fehler

// -------- WHEEL_SPEEDS (0xAA / 170) --------
float WHEEL_SPEEDS_FR = 60; // WHEEL_SPEED_*: [0|250] km/h, Faktor: 0.01, Offset: -67.67
float WHEEL_SPEEDS_FL = 60; // WHEEL_SPEED_*: [0|250] km/h, Faktor: 0.01, Offset: -67.67
float WHEEL_SPEEDS_RR = 60; // WHEEL_SPEED_*: [0|250] km/h, Faktor: 0.01, Offset: -67.67
float WHEEL_SPEEDS_RL = 60; // WHEEL_SPEED_*: [0|250] km/h, Faktor: 0.01, Offset: -67.67

// -------- BLINKERS_STATE (0x614 / 1556) --------
uint8_t BLINKERS_BUTTON_PRESSED = 0; // BLINKER_BUTTON_PRESSED: 0=not pressed, 1=pressed
uint8_t BLINKERS_HAZARD_LIGHT = 0; // HAZARD_LIGHT: 0=aus, 1=an
uint8_t BLINKERS_TURN_SIGNALS = 3; // TURN_SIGNALS: 1=links, 2=rechts, 3=none

// -------- BODY_CONTROL_STATE (0x620 / 1568) --------
uint8_t BCS_METER_DIMMED = 0;
uint8_t BCS_PARKING_BRAKE = 0;
uint8_t BCS_SEATBELT_DRIVER_UNLATCHED = 0;
uint8_t BCS_DOOR_OPEN_FL = 0;
uint8_t BCS_DOOR_OPEN_FR = 0;
uint8_t BCS_DOOR_OPEN_RL = 0;
uint8_t BCS_DOOR_OPEN_RR = 0;

// -------- GEAR_PACKET (0x3BC / 956) --------
uint8_t GEAR_SPORT_ON = 0; // SPORT_ON: 1=an
uint8_t GEAR_GEAR = 0; // // GEAR: 0=D, 1=S, 8=N, 16=R, 32=P
uint8_t GEAR_SPORT_GEAR_ON = 0; // SPORT_GEAR_ON: 1=an
uint8_t GEAR_SPORT_GEAR = 1; // SPORT_GEAR: 1=S1...6=S6
uint8_t GEAR_ECON_ON = 0; // 0=aus, 1=an
uint8_t GEAR_B_GEAR_ENGAGED = 0; //0=aus, 1=an
uint8_t GEAR_DRIVE_ENGAGED = 1; //0=aus, 1=an

// -------- ESP_CONTROL (0x3B7 / 951) --------
uint8_t ESP_TC_DISABLED = 0;
uint8_t ESP_VSC_DISABLED = 0;
uint8_t ESP_BRAKE_LIGHTS_ACC = 0;
uint8_t ESP_BRAKE_HOLD_ENABLED = 0;
uint8_t ESP_BRAKE_HOLD_ACTIVE = 0;

/* //Kommt dann von EPS im Auto 

// -------- STEER_ANGLE_SENSOR (0x25 / 37) --------
float SAS_ANGLE = 10; // STEER_ANGLE: [-500|500] deg, Faktor: 1.5, Offset: 0
float SAS_FRACTION = 0.2; // STEER_FRACTION: [-0.7|0.7] deg, Faktor: 0.1, Offset: 0
float SAS_RATE = 100; // STEER_RATE: [-2000|2000] deg/s, Faktor: 1, Offset: 0

// -------- STEER_TORQUE_SENSOR (0x260 / 608) --------
int16_t STS_TORQUE_EPS = 1200; // STEER_TORQUE_EPS: [-32768|32767] Nm
int16_t STS_TORQUE_DRIVER = 5; // STEER_TORQUE_DRIVER: [-32768|32767] Nm
float   STS_ANGLE = 20; // STEER_ANGLE: [-500|500] deg, Faktor: 0.0573
uint8_t STS_ANGLE_INITIALIZING = 0; // STEER_ANGLE_INITIALIZING
uint8_t STS_OVERRIDE = 0; // STEER_OVERRIDE

// -------- EPS_STATUS (0x262 / 610) --------
uint8_t EPS_IPAS_STATE = 0; // IPAS_STATE: 0=off, 1=disabled, 3=enabled, 5=override
uint8_t EPS_LKA_STATE = 5; // LKA_STATE: 1=standby, 5=active, 9=tmp_fault2, 25=tmp_fault
uint8_t EPS_TYPE = 0; // TYPE: 0=andere, 1=Corolla
*/

// -------- LIGHT_STALK (0x622 / 1570) --------
uint8_t LS_AUTO_HIGH_BEAM = 0;
uint8_t LS_FRONT_FOG = 0;
uint8_t LS_PARKING_LIGHT = 1;
uint8_t LS_LOW_BEAM = 1;
uint8_t LS_HIGH_BEAM = 0;
uint8_t LS_DAYTIME_RUNNING_LIGHT = 1;

// -------- PRE_COLLISION (0x283 / 643) --------
uint8_t PRECOLL_COUNTER = 0;
uint8_t PRECOLL_SET_ME_X00 = 0x00;
int16_t PRECOLL_FORCE = 120;
uint8_t PRECOLL_SET_ME_X002 = 0;
uint8_t PRECOLL_BRAKE_STATUS = 0;
uint8_t PRECOLL_STATE = 1; // 0=normal, 1=adaptive_cc, 3=emergency_braking
uint8_t PRECOLL_SET_ME_X003 = 0;
uint8_t PRECOLL_ACTIVE = 0;


// -------- BODY_CONTROL_STATE_2 (0x610 / 1552) --------
uint8_t BCS2_UI_SPEED = 100;
uint8_t BCS2_BRIGHTNESS_PCT = 80;
uint8_t BCS2_LOW_BRIGHTNESS = 0;
uint8_t BCS2_DIMMED = 0;
uint8_t BCS2_UNITS = 1; // 1=km/h

// -------- PCM_CRUISE_SM (0x399 / 921) --------
uint8_t PCMSM_MAIN_ON = 1;
uint8_t PCMSM_CRUISE_CONTROL_STATE = 8;
uint8_t PCMSM_DISTANCE_LINES = 2;
uint8_t PCMSM_TEMP_ACC_FAULTED = 0;
uint8_t PCMSM_UI_SET_SPEED = 100;

// -------- ENGINE_RPM (0x1C4 / 452) --------
float ENGINE_RPM = 2200.0;
uint8_t ENGINE_RUNNING = 1;

// -------- VSC1S07 (0x320 / 800) --------
// alle Felder aktuell auf 0

// -------- LKAS_HUD (0x412 / 1042) --------
// Werte für LKAS HUD (nur Beispiel)
uint8_t LKAS_BARRIERS = 0;
uint8_t LKAS_RIGHT_LINE = 2;
uint8_t LKAS_LEFT_LINE = 2;
uint8_t LKAS_LKAS_STATUS = 1;
uint8_t LKAS_LDW_EXIST = 1;

// =========================
// Checksumme
// =========================

uint8_t can_cksum(uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}

// =========================
// DAYWALKER_SPEED
// =========================

const int VSS_HALL_SENSOR_INTERRUPT_PIN = 3;

#define VSS_SENSOR_SMOOTHING 3    // 0 = just ringbuffer*refresh rate smoothing (e.g. over 800ms). highest response rate for reliable sensors 
                                  // 1 = in addition to 0 accounts for debounce effects of the sensor (additional, invalid signals) by limiting the change rate to 10kmh / REFRESH_RATE, e.g. 50kmh/s
                                  // 2 = assumes the sensor might lose revolutions at higher speeds (measures the maximum speed (shortest revolution time) for each refresh rate cycle) 
                                  // 3 = in addition to 2 accounts for debounce effects of the sensor (additional, invalid signals) by limiting the change rate to 10kmh / REFRESH_RATE, e.g. 50kmh/s
#define VSS_MAX_SPEED 160.0f    // the maximum speed in kmh handled by the ECU in smoothing mode 1 & 2
#define VSS_DISTANCE_PER_REVOLUTION 0.135f // 12.5cm driving distance per sensor revolution

const int VSS_RINGBUFFER_SIZE = 4;
const int VSS_REFRESH_RATE_MS = 200;
float vssRingBuffer[VSS_RINGBUFFER_SIZE];
float vssSpeedKMH=0;
float vssSpeedSum=0;
float vssAvgSpeedKMH=0;
float lastValidVssSpeedKMH=0;

int vssRingBufferIndex=0;

unsigned long vssDuration=0;
unsigned long lastVssRefresh=0;
unsigned long lastValidVssSpeedTs=0;

volatile byte vssSensorRevolutions=0;
volatile unsigned long vssLastTriggerMicros=0;
unsigned long vssLastUnhandledTriggerMicros=0;


void interruptVssSensor() {
  vssSensorRevolutions++;
  vssLastTriggerMicros=micros();
}

// =========================
// Arduino Setup
// =========================

void setup() {
  
  //Serial.begin(9600);
CAN.begin(500E3);

  
//______________initialize pins 
pinMode(button1, INPUT);
pinMode(button2, INPUT);
pinMode(button3, INPUT);
pinMode(button4, INPUT);
pinMode(BlinkerPinLeft, INPUT_PULLUP);
pinMode(BlinkerPinRight, INPUT_PULLUP);

  pinMode(VSS_HALL_SENSOR_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_HALL_SENSOR_INTERRUPT_PIN), interruptVssSensor, FALLING);

  for (int i=0; i<VSS_RINGBUFFER_SIZE; i++)
    vssRingBuffer[i]=0;
}

//______________DAYWALKER_SPEED
/**
 * This function is called each loop and determines the current vssAvgSpeedKMH.
 * It measures the exact micros elapsed between the last handled hall sensor trigger and the latest trigger [interrupt driven].
 * The duration is is used to determine the highest current speed within each VSS_REFRESH_RATE_MS interval 
 * (highest speed because at high frequencies, the hall sensor sometimes loses revolutions [capacitance?] so we use the biggest indiviual speed)
 * The speed is averaged for VSS_RINGBUFFER_SIZE*VSS_REFRESH_RATE_MS (< 1s)
 * */
void loopUpdateVssSensor() {
  #if VSS_SENSOR_SMOOTHING==0 || VSS_SENSOR_SMOOTHING==1
    if (vssSensorRevolutions>0) {

      vssDuration = (micros() - vssLastUnhandledTriggerMicros);
      uint8_t SaveSREG = SREG;
      noInterrupts();
      byte tmpVssSensorRevolutions=vssSensorRevolutions;
      vssLastUnhandledTriggerMicros=vssLastTriggerMicros;
      vssSensorRevolutions -= tmpVssSensorRevolutions;
      SREG = SaveSREG;

      vssSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
      #if VSS_SENSOR_SMOOTHING==1
        vssSpeedKMH=max(min(vssSpeedKMH, vssAvgSpeedKMH+10), vssAvgSpeedKMH-10);
      #endif
      vssTotalSensorRevolutions += tmpVssSensorRevolutions;
    }
    else if (micros()-vssLastUnhandledTriggerMicros>1000L*1000L) { // 1 second without hall signal is interpreted as standstill
      vssSpeedKMH=0;
    }
  #elif VSS_SENSOR_SMOOTHING==2 || VSS_SENSOR_SMOOTHING==3
    if (vssSensorRevolutions>0) {
        vssDuration = (vssLastTriggerMicros - vssLastUnhandledTriggerMicros);
        uint8_t SaveSREG = SREG;
        noInterrupts();
        byte tmpVssSensorRevolutions=vssSensorRevolutions;
        vssLastUnhandledTriggerMicros=vssLastTriggerMicros;
        vssSensorRevolutions -= tmpVssSensorRevolutions;
        SREG = SaveSREG;

        float tmpSpeedKMH = tmpVssSensorRevolutions * (VSS_DISTANCE_PER_REVOLUTION / (vssDuration * 0.000001)) * 3.6;
        if (tmpSpeedKMH<=VSS_MAX_SPEED) // we cap the speed we measure to max. 150km/h (max. OP speed) because sometimes at high frequencies the hall sensor might bounce and produce incorrect, way too[...]
          vssSpeedKMH = max(vssSpeedKMH, tmpSpeedKMH);
        #if VSS_SENSOR_SMOOTHING==3
          vssSpeedKMH=max(min(vssSpeedKMH, vssAvgSpeedKMH+10), vssAvgSpeedKMH-10);
        #endif
    }
    else if (micros()-vssLastUnhandledTriggerMicros>1000L*1000L) { // 1 second without hall signal is interpreted as standstill
      vssSpeedKMH=0;
    }
  #endif

  if (millis()-lastVssRefresh>=VSS_REFRESH_RATE_MS) {
    lastVssRefresh=millis();
    
    // this allows us to measure accurate low speeds (~1.5-8 km/h)
    if (vssSpeedKMH>0) {
      lastValidVssSpeedKMH=vssSpeedKMH;
      lastValidVssSpeedTs=millis();
    }
    else if (vssSpeedKMH==0 && lastValidVssSpeedKMH>0 && millis()-lastValidVssSpeedTs<1000) {
      vssSpeedKMH=lastValidVssSpeedKMH;
    }

    vssSpeedSum-=vssRingBuffer[vssRingBufferIndex];
    vssSpeedSum+=vssSpeedKMH;
    vssRingBuffer[vssRingBufferIndex]=vssSpeedKMH;
    vssSpeedKMH=0;
    vssRingBufferIndex++;
    if (vssRingBufferIndex>=VSS_RINGBUFFER_SIZE)
      vssRingBufferIndex=0;
    vssAvgSpeedKMH = vssSpeedSum / VSS_RINGBUFFER_SIZE;
  }
  
}

// =========================
// Arduino Loop
// =========================
void loop() {
  
//______________DAYWALKER_SPEED
loopUpdateVssSensor();  
 
//______________READING BUTTONS AND SWITCHES
ClutchSwitchState = digitalRead(CluchSwitch);
buttonstate4 = digitalRead(button4);
buttonstate3 = digitalRead(button3);
buttonstate2 = digitalRead(button2);
buttonstate1 = digitalRead(button1);

//______________READING BLINKERS & LOGIC
boolean blinker_left = digitalRead(BlinkerPinLeft); // Left Blinker
  
  if (blinker_left){
    last_blinker_left = millis();
  }
  if (last_blinker_left + 500 < millis()){
      BLINKERS_TURN_SIGNALS = 3;
  }else{
      BLINKERS_TURN_SIGNALS = 1;
  }

boolean blinker_right = digitalRead(BlinkerPinRight); // Right Blinker
  
  if (blinker_right){
    last_blinker_right = millis();
  }
  if (last_blinker_right + 500 < millis()){
      BLINKERS_TURN_SIGNALS = 3;
  }else{
      BLINKERS_TURN_SIGNALS = 2;
  }
         
//______________SET OP OFF WHEN BRAKE IS PRESSED
       if (BRAKE_PRESSED == true)
       {
       PCM_CRUISE_CRUISE_ACTIVE = false;
       }
    
//______________SET OP OFF WHEN GAS IS PRESSED
       if (GAS_RELEASED == false)
       {
       PCM_CRUISE_CRUISE_ACTIVE = false;
       }
  
//______________SET BUTTON NR4
if (buttonstate4 != lastbuttonstate4)
    {
       if (buttonstate4 == LOW)
       {
          if (PCM_CRUISE_CRUISE_ACTIVE == true)
          {
          PCM_CRUISE_CRUISE_ACTIVE = false;
          }
          else
          {
          PCM_CRUISE_CRUISE_ACTIVE = true;
          PCM_CRUISE_2_SET_SPEED = (vssAvgSpeedKMH + 3);
          }
        }
     }
     
//______________SET BUTTON NR3
if (buttonstate3 != lastbuttonstate3)
    {
       if (buttonstate3 == LOW)
       {
       PCM_CRUISE_2_SET_SPEED = PCM_CRUISE_2_SET_SPEED + 5;
       }
    }

//______________SET BUTTON NR2
if (buttonstate2 != lastbuttonstate2)
   {
       if (buttonstate2 == LOW)
       {
       PCM_CRUISE_2_SET_SPEED = PCM_CRUISE_2_SET_SPEED - 5;
       }
    }
    
//______________LIMIT FOR SETSPEED
if (PCM_CRUISE_2_SET_SPEED > 200)
    { 
      PCM_CRUISE_2_SET_SPEED = 0;
    }
    
//______________SET BUTTON NR1
if (buttonstate1 != lastbuttonstate1)
   {
       if (buttonstate1 == LOW)
       {
       PCM_CRUISE_CRUISE_ACTIVE = false;
       }
   }

//______________SET CLUTCH SWITCH
if (ClutchSwitchState == LOW)
   {
  //  ("Clutch Pedal is pressed");
   }

//______________RESET BUTTONS & VALUES
lastbuttonstate1 = buttonstate1;
lastbuttonstate2 = buttonstate2;
lastbuttonstate3 = buttonstate3;
lastbuttonstate4 = buttonstate4;
lastBRAKE_PRESSED = BRAKE_PRESSED;
lastGAS_RELEASED = GAS_RELEASED;

//______________SENDING_CAN_MESSAGES
  
 // PCM_CRUISE (0x1D2)
  {
    uint8_t data[8] = {0};
    data[0] |= (PCM_CRUISE_GAS_RELEASED << 4);
    data[0] |= (PCM_CRUISE_CRUISE_ACTIVE << 5);
    data[1] |= (PCM_CRUISE_ACC_BRAKING << 4);
    int16_t accel = (int16_t)(PCM_CRUISE_ACCEL_NET / 0.0009765625f);
    data[2] = (accel >> 8) & 0xFF;
    data[3] = (accel     ) & 0xFF;
    int16_t nforce = (int16_t)(PCM_CRUISE_NEUTRAL_FORCE / 2.0f);
    data[4] = (nforce >> 8) & 0xFF;
    data[5] = (nforce     ) & 0xFF;
    data[6] |= (PCM_CRUISE_CRUISE_STATE << 4);
    data[6] |= (PCM_CRUISE_CANCEL_REQ << 1);
    data[7] = can_cksum(data, 7, 0x1d2);
    CAN.beginPacket(0x1D2);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PCM_CRUISE_2 (0x1D3)
  {
    uint8_t data[8] = {0};
    data[0] |= (PCM_CRUISE_2_BRAKE_PRESSED << 3);
    data[1] |= (PCM_CRUISE_2_PCM_FOLLOW_DISTANCE << 4);
    data[1] |= (PCM_CRUISE_2_LOW_SPEED_LOCKOUT << 6);
    data[1] |= (PCM_CRUISE_2_MAIN_ON << 7);
    data[2] = PCM_CRUISE_2_SET_SPEED;
    data[5] |= (PCM_CRUISE_2_ACC_FAULTED << 7);
    data[7] = can_cksum(data, 7, 0x1d3);
    CAN.beginPacket(0x1D3);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // WHEEL_SPEEDS (0xAA)
  {
    uint8_t data[8] = {0};
    uint16_t ws_fr = (uint16_t)((WHEEL_SPEEDS_FR + 67.67) / 0.01);
    uint16_t ws_fl = (uint16_t)((WHEEL_SPEEDS_FL + 67.67) / 0.01);
    uint16_t ws_rr = (uint16_t)((WHEEL_SPEEDS_RR + 67.67) / 0.01);
    uint16_t ws_rl = (uint16_t)((WHEEL_SPEEDS_RL + 67.67) / 0.01);
    data[0] = (ws_fr >> 8) & 0xFF; data[1] = ws_fr & 0xFF;
    data[2] = (ws_fl >> 8) & 0xFF; data[3] = ws_fl & 0xFF;
    data[4] = (ws_rr >> 8) & 0xFF; data[5] = ws_rr & 0xFF;
    data[6] = (ws_rl >> 8) & 0xFF; data[7] = ws_rl & 0xFF;
    CAN.beginPacket(0xAA);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BLINKERS_STATE (0x614)
  {
    uint8_t data[8] = {0};
    data[1] |= (BLINKERS_BUTTON_PRESSED << 7);
    data[3] |= (BLINKERS_HAZARD_LIGHT << 3);
    data[3] |= (BLINKERS_TURN_SIGNALS << 4);
    CAN.beginPacket(0x614);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // BODY_CONTROL_STATE (0x620)
  {
    uint8_t data[8] = {0};
    data[4] |= (BCS_DOOR_OPEN_RL << 2);
    data[4] |= (BCS_DOOR_OPEN_RR << 3);
    data[4] |= (BCS_DOOR_OPEN_FR << 4);
    data[4] |= (BCS_DOOR_OPEN_FL << 5);
    data[5] |= (BCS_SEATBELT_DRIVER_UNLATCHED << 2);
    data[5] |= (BCS_PARKING_BRAKE << 3);
    data[5] |= (BCS_METER_DIMMED << 4);
    CAN.beginPacket(0x620);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // GEAR_PACKET (0x3BC)
  {
    uint8_t data[8] = {0};
    data[0] |= (GEAR_SPORT_ON << 2);
    data[1] |= ((GEAR_GEAR & 0x3F) << 5);
    data[2] |= ((GEAR_GEAR & 0x3F) >> 3);
    data[4] |= (GEAR_SPORT_GEAR_ON << 1);
    data[4] |= ((GEAR_SPORT_GEAR & 0x03) << 6);
    data[5] |= ((GEAR_SPORT_GEAR & 0x04) >> 2);
    data[5] |= (GEAR_DRIVE_ENGAGED << 7);
    CAN.beginPacket(0x3bc);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

/* //FROM CAR:
 *  
  // STEER_ANGLE_SENSOR (0x25)
  {
    uint8_t data[8] = {0};
    int16_t angle = (int16_t)(SAS_ANGLE / 1.5);
    data[0] = (angle >> 4) & 0xFF;
    data[1] = ((angle & 0x0F) << 4);
    int16_t rate = (int16_t)(SAS_RATE / 1.0);
    data[4] = (rate >> 4) & 0xFF;
    data[5] = ((rate & 0x0F) << 4);
    int8_t fraction = (int8_t)(SAS_FRACTION / 0.1);
    data[4] |= ((fraction & 0x0F) << 4);
    CAN.beginPacket(0x25);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // STEER_TORQUE_SENSOR (0x260)
  {
    uint8_t data[8] = {0};
    data[5] = (STS_TORQUE_EPS >> 8) & 0xFF; data[6] = STS_TORQUE_EPS & 0xFF;
    data[1] = (STS_TORQUE_DRIVER >> 8) & 0xFF; data[2] = STS_TORQUE_DRIVER & 0xFF;
    int16_t angle = (int16_t)(STS_ANGLE / 0.0573);
    data[3] = (angle >> 8) & 0xFF; data[4] = angle & 0xFF;
    data[0] |= (STS_OVERRIDE << 0);
    data[0] |= (STS_ANGLE_INITIALIZING << 3);
    data[7] = can_cksum(data, 7, 0x260);
    CAN.beginPacket(0x260);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // EPS_STATUS (0x262)
  {
    uint8_t data[5] = {0};
    data[0] |= (EPS_IPAS_STATE & 0x0F);
    data[3] |= (EPS_LKA_STATE << 1);
    data[3] |= (EPS_TYPE << 0);
    data[4] = can_cksum(data, 4, 0x262);
    CAN.beginPacket(0x262);
    for (int i = 0; i < 5; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  */

  // LIGHT_STALK (0x622)
  {
    uint8_t data[8] = {0};
    data[4] |= (LS_AUTO_HIGH_BEAM << 5);
    data[3] |= (LS_FRONT_FOG << 3);
    data[3] |= (LS_PARKING_LIGHT << 4);
    data[3] |= (LS_LOW_BEAM << 5);
    data[3] |= (LS_HIGH_BEAM << 6);
    data[3] |= (LS_DAYTIME_RUNNING_LIGHT << 7);
    CAN.beginPacket(0x622);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PRE_COLLISION (0x283)
  {
    uint8_t data[7] = {0};
    data[0] = PRECOLL_COUNTER++;
    if (PRECOLL_COUNTER > 250) PRECOLL_COUNTER = 0;
    data[1] = PRECOLL_SET_ME_X00;
    data[2] = (PRECOLL_FORCE >> 8) & 0xFF;
    data[3] = (PRECOLL_FORCE     ) & 0xFF;
    data[4] = PRECOLL_SET_ME_X002;
    data[4] |= (PRECOLL_BRAKE_STATUS << 5);
    data[4] |= (PRECOLL_STATE << 2);
    data[5] |= (PRECOLL_SET_ME_X003 << 0);
    data[5] |= (PRECOLL_ACTIVE << 1);
    data[6] = can_cksum(data, 6, 0x283);
    CAN.beginPacket(0x283);
    for (int i = 0; i < 7; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

/* //FROM BRAKE ECU
 
  // BRAKE_MODULE (0x224)
  {
    uint8_t data[8] = {0};
    data[5] |= (BRAKE_PRESSURE & 0xFF) << 3;
    data[6] |= (BRAKE_PRESSURE >> 5) & 0x7F;
    data[0] |= (BRAKE_PRESSED << 5);
    CAN.beginPacket(0x224);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }
  
// ESP_CONTROL (0x3B7)
  {
    uint8_t data[8] = {0};
    data[1] |= (ESP_VSC_DISABLED << 4);
    data[1] |= (ESP_TC_DISABLED << 5);
    data[2] |= (ESP_BRAKE_LIGHTS_ACC << 2);
    data[4] |= (ESP_BRAKE_HOLD_ENABLED << 0);
    data[4] |= (ESP_BRAKE_HOLD_ACTIVE << 4);
    CAN.beginPacket(0x3B7);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

*/
  // BODY_CONTROL_STATE_2 (0x610)
  {
    uint8_t data[8] = {0};
    data[2] |= (BCS2_UI_SPEED << 7);
    data[3] |= (BCS2_UI_SPEED >> 1);
    data[3] |= (BCS2_BRIGHTNESS_PCT & 0x7F) << 6;
    data[4] |= (BCS2_BRIGHTNESS_PCT >> 1);
    data[4] |= (BCS2_LOW_BRIGHTNESS << 5);
    data[4] |= (BCS2_DIMMED << 6);
    data[7] |= (BCS2_UNITS << 5);
    CAN.beginPacket(0x610);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // PCM_CRUISE_SM (0x399)
  {
    uint8_t data[8] = {0};
    data[0] |= (PCMSM_MAIN_ON << 4);
    data[1] |= (PCMSM_CRUISE_CONTROL_STATE << 0);
    data[1] |= (PCMSM_DISTANCE_LINES << 5);
    data[1] |= (PCMSM_TEMP_ACC_FAULTED << 7);
    data[3] = PCMSM_UI_SET_SPEED;
    CAN.beginPacket(0x399);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // ENGINE_RPM (0x1C4)
  {
    uint8_t data[8] = {0};
    int16_t rpm_raw = (int16_t)(ENGINE_RPM / 0.78125);
    data[0] = rpm_raw & 0xFF;
    data[1] = (rpm_raw >> 8) & 0xFF;
    data[3] |= (ENGINE_RUNNING << 3);
    CAN.beginPacket(0x1C4);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // VSC1S07 (0x320)
  {
    uint8_t data[8] = {0};
    CAN.beginPacket(0x320);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }

  // LKAS_HUD (0x412)
  {
    uint8_t data[8] = {0};
    data[0] |= (LKAS_BARRIERS << 0);
    data[0] |= (LKAS_RIGHT_LINE << 2);
    data[0] |= (LKAS_LEFT_LINE << 4);
    data[0] |= (LKAS_LKAS_STATUS << 6);
    data[1] |= (LKAS_LDW_EXIST << 2);
    CAN.beginPacket(0x412);
    for (int i = 0; i < 8; i++) CAN.write(data[i]);
    CAN.endPacket();
  }
  


//______________READING CAN
  CAN.parsePacket();

/* ALT 
  //0x3b7 msg ESP_CONTROL --- WE are sending the 0x3b7 message from Brake_ECU, to reduce traffic on the can and improve safety
    if (CAN.packetId() == 0x3b7)
      {
      uint8_t dat_3b7[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_3b7[ii]  = (char) CAN.read();
        }
        BRAKE_PRESSED = (dat_3b7[0] << 5);
        }
 */ 

  if (CAN.packetId() == 0x3b7) {
  uint8_t dat_3b7[8];
  for (int ii = 0; ii < 8; ii++) {
    dat_3b7[ii] = (uint8_t) CAN.read();
  }
  ESP_TC_DISABLED = (dat_3b7[1] >> 5) & 0x01;
  ESP_VSC_DISABLED = (dat_3b7[1] >> 4) & 0x03;
  ESP_BRAKE_LIGHTS_ACC = (dat_3b7[2] >> 2) & 0x01;
  ESP_BRAKE_HOLD_ENABLED = (dat_3b7[4] >> 0) & 0x01;
  ESP_BRAKE_HOLD_ACTIVE = (dat_3b7[4] >> 4) & 0x01;
  }

if (CAN.packetId() == 0x224) {
    uint8_t dat_224[8];
    for (int ii = 0; ii < 8; ii++) {
        dat_224[ii] = (uint8_t) CAN.read();
    }
    BRAKE_PRESSED = (dat_224[0] >> 5) & 0x01;
    uint16_t pressure_raw = ((dat_224[5] >> 3) & 0x1F);          // Bits 3-7 von Byte 5 (5 Bit)
    pressure_raw |= ((uint16_t)(dat_224[6] & 0x7F)) << 5;        // Bits 0-6 von Byte 6 (7 Bit)
    BRAKE_PRESSURE = pressure_raw; 
}
  
    //0x2c1 msg GAS_PEDAL
    if (CAN.packetId() == 0x2c1)
      {
      uint8_t dat_2c1[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_2c1[ii]  = (char) CAN.read();
        }
        GAS_RELEASED = (dat_2c1[0] << 3);
        }

  
  
} //______________END OF LOOP
