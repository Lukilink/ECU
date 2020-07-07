/* BRAKE
 * DONT USE ON PUBLIC ROADS!
*/

//________________import can libary https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>

//________________this values needs to be define for each car
int PERM_ERROR = 4; // will allow a diffrence between targetPressure and currentPressure
int maxPressure = 50; // the max pressure your actuator is able to aply
int minPressure = 37; //the pressure in stand still
float maxACC_CMD = 500; //the max Value which comes from OP
float minACC_CMD = 0; //the min Value which comes from OP
int brake_pressed_threshold = 10; // threshold when user input is detected
int brake_light_threshold = 5; // threshold when brakelights schould turn on


//________________define_pins
int pressurePin = A2;
int breaklightPin = A5;
int M_DIR = 8; // LOW is Left / HIGH is Right
int M_PWM = 9; // 255 is run / LOW is stopp
int S_DIR = 7; // LOW is Left / HIGH is Right
int S_PWM = 6; // 255 is run / LOW is stopp

//________________values
int targetPressure = 0;
int currentPressure;
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean cancel = true;
boolean BRAKE_PRESSED = true;
long previousMillis;


void setup() {
    
//________________begin Monitor - only use it for debugging
 Serial.begin(115200);

//________________begin CAN
CAN.begin(500E3);
CAN.filter(0x343);

//________________set up pin modes
pinMode(pressurePin, INPUT);
pinMode(M_DIR, OUTPUT);
pinMode(M_PWM, OUTPUT);
pinMode(S_DIR, OUTPUT);
pinMode(S_PWM, OUTPUT);
pinMode(breaklightPin, OUTPUT);
digitalWrite(S_DIR, HIGH);

}

void loop() {

//________________read pressure sensor 
currentPressure = (analogRead(pressurePin));


//________________light up break lights
long currentMillis = millis();
if (currentMillis - previousMillis >= 500){  
  if (currentPressure >= (minPressure + brake_light_threshold))
  {
   analogWrite(breaklightPin, 255);
  }
else 
  {
   analogWrite(breaklightPin, 0);
  }
      previousMillis = currentMillis;
}

//________________read ACC_CMD from CANbus
 CAN.parsePacket();

 if (CAN.packetId() == 0x343)
      {
      uint8_t dat[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat[ii]  = (char) CAN.read();
        }
        ACC_CMD = ((dat[0] << 8 | dat[1] << 0) * -1); 
        }

//________________calculating ACC_CMD into ACC_CMD_PERCENT
if (ACC_CMD >= minACC_CMD) {
    ACC_CMD1 = ACC_CMD;
    }
else {
    ACC_CMD1 = minACC_CMD;
    }

//Serial.println( ACC_CMD_PERCENT);
       
ACC_CMD_PERCENT = ((100/(maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));

//________________calculating tagetpressure from ACC_CMD_PERCENT
float targetPressure = (((ACC_CMD_PERCENT / 100) * (maxPressure - minPressure)) + minPressure); // conversion from ACC_CMD_PERCENT % into targetpressure
      
//________________do nothing and open solenoid if ACC_CMD_PERCENT is 0  
if (ACC_CMD_PERCENT == 0){
   analogWrite(S_PWM, 0);  //open solenoid
   analogWrite(M_PWM, 0);  //stop Motor;
   }
    
else {
    analogWrite(S_PWM, 255);
   
 
//________________press or release the pedal to match targetPressure & respect endpoints
if (abs(currentPressure - targetPressure) >= PERM_ERROR)
  {
    if (currentPressure < targetPressure)
        { 
        analogWrite(M_PWM, 255);  //run Motor
        digitalWrite(M_DIR, HIGH); //motor driection left
        }    
    else if (currentPressure > targetPressure)
        {       
        analogWrite(M_PWM, 255);   //run Motor
        digitalWrite(M_DIR, LOW); //motor driection right
        }
  }  
     
//________________if we match target position, just stay here
else {
     analogWrite(M_PWM, 0);   //stop Motor
     }  
}

//________________logic to read if brake is pressed by human
    
if (currentPressure >= (targetPressure + brake_pressed_threshold))
    {
        BRAKE_PRESSED = true;
    }
    
else {
        BRAKE_PRESSED = false;
     }

//________________send_ON_CAN-BUS 
//0x3b7 msg ESP_CONTROL
  uint8_t dat_3b7[8];
  dat_3b7[0] = (BRAKE_PRESSED << 5) & 0x20;
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

//________________print stuff if you want to DEBUG

//Serial.print("ACC_CMD_");
//Serial.print(ACC_CMD);
//Serial.print("_____");
//Serial.print(ACC_CMD_PERCENT);
//Serial.print("_%");
//Serial.print("_____");
//Serial.print("target_");
//Serial.print(targetPressure);
//Serial.print("_____");
//Serial.print("Pressure");
//Serial.println(currentPressure);
//Serial.println("");

}
