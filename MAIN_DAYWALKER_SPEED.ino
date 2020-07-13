// MAIN ECU
#include <CAN.h>


//______________LCR (Lane Change Recommendation) VARIABLES
int LCR_minimum_speed = 80; // below this speed the feature is disabled
int LCR_speed_diff = 15; // at which speed differende to the leas it recommends a lane change
int LCR_lead_distance = 100; // minimum distance to the lead


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

//______________VALUES SEND ON CAN
boolean OP_ON = false;
boolean MAIN_ON = true;
uint8_t set_speed = 0x0;
boolean blinker_left_on = true;
boolean blinker_right_on = true;
float LEAD_LONG_DIST = 0;
float LEAD_REL_SPEED = 0;
float LEAD_LONG_DIST_RAW = 0;
float LEAD_REL_SPEED_RAW = 0;
boolean BRAKE_PRESSED = true;
boolean GAS_RELEASED = false;


//______________DAYWALKER_SPEED
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

//______________TOYOTA CAN CHECKSUM
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}


//______________DAYWALKER_SPEED
void interruptVssSensor() {
  vssSensorRevolutions++;
  vssLastTriggerMicros=micros();
}



void setup() {
  
Serial.begin(9600);
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
        if (tmpSpeedKMH<=VSS_MAX_SPEED) // we cap the speed we measure to max. 150km/h (max. OP speed) because sometimes at high frequencies the hall sensor might bounce and produce incorrect, way too high readings
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
      blinker_left_on = false;
  }else{
      blinker_left_on =  true;
  }

boolean blinker_right = digitalRead(BlinkerPinRight); // Right Blinker
  
  if (blinker_right){
    last_blinker_right = millis();
  }
  if (last_blinker_right + 500 < millis()){
      blinker_right_on = false;
  }else{
      blinker_right_on =  true;
  }
         
//______________SET OP OFF WHEN BRAKE IS PRESSED
       if (BRAKE_PRESSED == true)
       {
       OP_ON = false;
       }
    
//______________SET OP OFF WHEN GAS IS PRESSED
       if (GAS_RELEASED == false)
       {
       OP_ON = false;
       }
  
//______________SET BUTTON NR4
if (buttonstate4 != lastbuttonstate4)
    {
       if (buttonstate4 == LOW)
       {
          if (OP_ON == true)
          {
          OP_ON = false;
          }
          else
          {
          OP_ON = true;
          set_speed = (vssAvgSpeedKMH + 3);
          }
        }
     }
     
//______________SET BUTTON NR3
if (buttonstate3 != lastbuttonstate3)
    {
       if (buttonstate3 == LOW)
       {
       set_speed = set_speed + 5;
       }
    }

//______________SET BUTTON NR2
if (buttonstate2 != lastbuttonstate2)
   {
       if (buttonstate2 == LOW)
       {
       set_speed = set_speed - 5;
       }
    }
    
//______________LIMIT FOR SETSPEED
if (set_speed > 200)
    { 
      set_speed = 0;
    }
    
//______________SET BUTTON NR1
if (buttonstate1 != lastbuttonstate1)
   {
       if (buttonstate1 == LOW)
       {
       OP_ON = false;
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
  //0x1d2 msg PCM_CRUISE
  uint8_t dat_1d2[8];
  dat_1d2[0] = (OP_ON << 5) & 0x20 | (GAS_RELEASED << 4) & 0x10;
  dat_1d2[1] = 0x0;
  dat_1d2[2] = 0x0;
  dat_1d2[3] = 0x0;
  dat_1d2[4] = 0x0;
  dat_1d2[5] = 0x0;
  dat_1d2[6] = (OP_ON << 7) & 0x80;
  dat_1d2[7] = can_cksum(dat_1d2, 7, 0x1d2);
  CAN.beginPacket(0x1d2);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_1d2[ii]);
  }
  CAN.endPacket();

  //0x1d3 msg PCM_CRUISE_2
  uint8_t dat_1d3[8];
  dat_1d3[0] = 0x0;
  dat_1d3[1] = (MAIN_ON << 7) & 0x80 | 0x28;
  dat_1d3[2] = set_speed;
  dat_1d3[3] = 0x0;
  dat_1d3[4] = 0x0;
  dat_1d3[5] = 0x0;
  dat_1d3[6] = 0x0;
  dat_1d3[7] = can_cksum(dat_1d3, 7, 0x1d3);
  CAN.beginPacket(0x1d3);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_1d3[ii]);
  }
  CAN.endPacket();

  //0xaa msg defaults 1a 6f WHEEL_SPEEDS
  uint8_t dat_aa[8];
  uint16_t wheelspeed = 0x1a6f + (vssAvgSpeedKMH * 100);
  dat_aa[0] = (wheelspeed >> 8) & 0xFF;
  dat_aa[1] = (wheelspeed >> 0) & 0xFF;
  dat_aa[2] = (wheelspeed >> 8) & 0xFF;
  dat_aa[3] = (wheelspeed >> 0) & 0xFF;
  dat_aa[4] = (wheelspeed >> 8) & 0xFF;
  dat_aa[5] = (wheelspeed >> 0) & 0xFF;
  dat_aa[6] = (wheelspeed >> 8) & 0xFF;
  dat_aa[7] = (wheelspeed >> 0) & 0xFF;
  CAN.beginPacket(0xaa);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_aa[ii]);
  }
  CAN.endPacket();
  
/* ************we are sending this message from BRAKE ECU for safetyness
  //0x3b7 msg ESP_CONTROL
  uint8_t dat_3b7[8];
  dat_3b7[0] = 0x0;
  dat_3b7[1] = 0x0;
  dat_3b7[2] = 0x0;
  dat_3b7[3] = 0x0;
  dat_3b7[4] = 0x0;
  dat_3b7[5] = 0x0;
  dat_3b7[6] = 0x0;
  dat_3b7[7] = 0x08;
  CAN.beginPacket(0x3b7);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3b7[ii]);
  }
  CAN.endPacket();
*/
  //0x620 msg STEATS_DOORS
  uint8_t dat_620[8];
  dat_620[0] = 0x10;
  dat_620[1] = 0x0;
  dat_620[2] = 0x0;
  dat_620[3] = 0x1d;
  dat_620[4] = 0xb0;
  dat_620[5] = 0x40;
  dat_620[6] = 0x0;
  dat_620[7] = 0x0;
  CAN.beginPacket(0x620);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_620[ii]);
  }
  CAN.endPacket();

  // 0x3bc msg GEAR_PACKET
  uint8_t dat_3bc[8];
  dat_3bc[0] = 0x0;
  dat_3bc[1] = 0x0;
  dat_3bc[2] = 0x0;
  dat_3bc[3] = 0x0;
  dat_3bc[4] = 0x0;
  dat_3bc[5] = 0x80;
  dat_3bc[6] = 0x0;
  dat_3bc[7] = 0x0;
  CAN.beginPacket(0x3bc);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_3bc[ii]);
  }
  CAN.endPacket();

  //0x614 msg steering_levers
  uint8_t dat_614[8];
  dat_614[0] = 0x29;
  dat_614[1] = 0x0;
  dat_614[2] = 0x01;
  dat_614[3] = (blinker_left_on << 5) & 0x20 |(blinker_right_on << 4) & 0x10;
  dat_614[4] = 0x0;
  dat_614[5] = 0x0;
  dat_614[6] = 0x76;
  dat_614[7] = can_cksum(dat_614, 7, 0x614);
  CAN.beginPacket(0x614);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_614[ii]);
  }
  CAN.endPacket();

//______________READING CAN
  CAN.parsePacket();

  //128x2e6 msg LEAD_INFO
  if (CAN.packetId() == 0x2e6)
      {
      uint8_t dat_2e6[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_2e6[ii]  = (char) CAN.read();
        }
        LEAD_LONG_DIST_RAW = (dat_2e6[0] << 8 | dat_2e6[1] << 3); 
        LEAD_REL_SPEED_RAW = (dat_2e6[2] << 8 | dat_2e6[3] << 4);
        }
  //CONVERTING INTO RIGHT VALUE USING DBC SCALE
  LEAD_LONG_DIST = (LEAD_LONG_DIST_RAW * 0.005);
  LEAD_REL_SPEED = (LEAD_REL_SPEED_RAW * 0.009);

  
  //0x3b7 msg ESP_CONTROL --- WE are sending the 0x3b7 message from Brake_ECU, to reduce traffic on the can and improve safety
    if (CAN.packetId() == 0x3b7)
      {
      uint8_t dat_3b7[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat_3b7[ii]  = (char) CAN.read();
        }
        BRAKE_PRESSED = (dat_3b7[0] << 5);
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
  
//______________LOGIC FOR LANE CHANGE RECOMENDITION
  if ((vssAvgSpeedKMH * 100) >= LCR_minimum_speed){
  if (set_speed >= ((vssAvgSpeedKMH * 100) + LCR_speed_diff))
   {
      if (LEAD_LONG_DIST <= LCR_lead_distance)
         {    
         blinker_left_on = false;
         }
      }
   }
  
  
} //______________END OF LOOP
