// GAS - DONT USE ON PUBLIC ROAD

//________________import can libary https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>

//________________this values needs to be define for each car
int PERM_ERROR = 8; //will allow a diffrence between targetPressure and currentPressure
int minPot = 86; //measured at actuators lowest position
int maxPot = 920; //measured at actuators highest position
float maxACC_CMD = 1430; //the max Value which comes from OP on CAN ID 0x200
float minACC_CMD = 477; //the min Value which comes from OP on CAN ID 0x200

//________________define_pins
int cancel_pin = 3; //pulled to GND when pedal pressed
int potPin = A3; // connect the potentiometer of your cars throllte

int M_DIR = 8; // LOW is Left / HIGH is Right
int M_PWM = 9; // 255 is run / LOW is stopp
int S_DIR = 7; // LOW is Left / HIGH is Right
int S_PWM = 6; // 255 is run / LOW is stopp

//________________values
int targetPosition = 0;
int potiPosition = 0;
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean cancel = false;
boolean GAS_RELEASED = false;

void setup() {
//________________begin Monitor - only use it for debugging
Serial.begin(115200);

//________________begin CAN
CAN.begin(500E3);

//________________set up pin modes
pinMode(cancel_pin, INPUT_PULLUP);
pinMode(potPin, INPUT);    
pinMode(M_DIR, OUTPUT);
pinMode(M_PWM, OUTPUT);
pinMode(S_DIR, OUTPUT);
pinMode(S_PWM, OUTPUT);
digitalWrite(S_DIR, LOW);

}

void loop() {
//________________read cancel pin
cancel = (digitalRead(cancel_pin));

//________________read poti Position
int potiPosition = (analogRead(potPin));

//________________read ACC_CMD from CANbus
 CAN.parsePacket();

 if (CAN.packetId() == 0x200)
      {
      uint8_t dat[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat[ii]  = (char) CAN.read();
        }
        ACC_CMD = (dat[0] << 8 | dat[1] << 0); 
       } 

//________________calculating ACC_CMD into ACC_CMD_PERCENT
if (ACC_CMD >= minACC_CMD) {
    ACC_CMD1 = ACC_CMD;
    }
else {
    ACC_CMD1 = minACC_CMD;
    }


ACC_CMD_PERCENT = ((100/(maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));

//________________calculating ACC_CMD_PERCENT into targetPosition 
float targetPosition = (((ACC_CMD_PERCENT / 100) * (maxPot - minPot)) + minPot);

//________________do nothing if ACC_CMD is 0
if (ACC_CMD_PERCENT == 0){
   analogWrite(S_PWM, 0);  //open solenoid
   analogWrite(M_PWM, 0);  //stop Motor
}

//________________do nothing while cancel, but read if it's still cancel
  while (!cancel) {
  analogWrite(S_PWM, 0);  //open solenoid
  analogWrite(M_PWM, 0);  //stop Motor
  cancel = (digitalRead(cancel_pin));
 }
   
//________________close solenoid
   digitalWrite(S_PWM, HIGH);

//________________press or release the pedal to match targetPosition & respect endpoints
if (abs(potiPosition - targetPosition) >= PERM_ERROR)
  {
    if ((potiPosition < targetPosition) && (potiPosition < maxPot)) //if we are lower than target and not at endpoint
        { 
        analogWrite(M_PWM, 255);  //run Motor
        digitalWrite(M_DIR, LOW); //motor driection left
        }    
    else if ((potiPosition > targetPosition) && (potiPosition >= minPot)) //if we are higher than target and not at endpoint
        {       
        analogWrite(M_PWM, 255);   //run Motor
        digitalWrite(M_DIR, HIGH); //motor driection right
        }
  }  
  
//________________if we match target position, just stay here
else {
     analogWrite(M_PWM, 0);   //stop Motor
     }
 
//________________logic if gas is pressed by user
 
if (potiPosition >= (targetPosition + 150))
   {
     GAS_RELEASED = false;
     Serial.println("GAS PRESSED");
    }
else {
     GAS_RELEASED = true;
     }


    
//______________SENDING_CAN_MESSAGES

  // 0x2c1 msg GAS_PEDAL
  uint8_t dat_2c1[8];
  dat_2c1[0] = (GAS_RELEASED << 3) & 0x08;
  dat_2c1[1] = 0x0;
  dat_2c1[2] = 0x0;
  dat_2c1[3] = 0x0;
  dat_2c1[4] = 0x0;
  dat_2c1[5] = 0x0;
  dat_2c1[6] = 0x0;
  dat_2c1[7] = 0x0;
  CAN.beginPacket(0x2c1);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat_2c1[ii]);
  }
  CAN.endPacket();
     
//________________print stuff if you want to DEBUG
/*
Serial.print("ACC_CMD_");
Serial.print(ACC_CMD);
Serial.print("_____");
Serial.print(ACC_CMD_PERCENT);
Serial.print("_%");
Serial.print("_____");
Serial.print("target_");
Serial.print(targetPosition);
Serial.print("_____");
Serial.print("Position_");
Serial.print(potiPosition);
Serial.println("");
*/

}
