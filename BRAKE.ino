/* BRAKE
 * DONT USE ON PUBLIC ROADS!
*/

//________________import can libary https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>

//________________this values needs to be define for each car
int PERM_ERROR = 2; // will allow a diffrence between targetPressure and currentPressure
int maxPressure = 50; // the max pressure your actuator is able to aply
int minPressure = 37; //the pressure in stand still
float maxACC_CMD = 500; //the max Value which comes from OP
float minACC_CMD = 0; //the min Value which comes from OP
int brake_pressed_threshold = 5; // threshold when user input is detected
int brake_light_threshold = 2; // threshold when brakelights schould turn on

//________________define_pins
int pressurePin = A2;
int brakelightPin = A5;
int M_DIR = 8;
int M_PWM = 9;
int S_DIR = 7;
int S_PWM = 6;

//________________values
float targetPressure;
float currentPressure;
float ACC_CMD_PERCENT;
float ACC_CMD;
float ACC_CMD1;
boolean BRAKE_PRESSED = true;
boolean releasing_by_OP = false;
boolean open_solenoid = true;
long previousMillis;


void setup() {
//________________begin Monitor - only use for debugging
// Serial.begin(115200);

//________________begin CAN
CAN.begin(500E3);
CAN.filter(0x343);

//________________set up pin modes
pinMode(pressurePin, INPUT);
pinMode(M_DIR, OUTPUT); // LOW is Left / HIGH is Right
pinMode(M_PWM, OUTPUT); // 255 is run / LOW is stop
pinMode(S_DIR, OUTPUT); // LOW is Left / HIGH is Right
pinMode(S_PWM, OUTPUT); // 255 is run / LOW is stop
digitalWrite(S_DIR, HIGH); // the direction for the solenoid does not matter
pinMode(brakelightPin, OUTPUT);
}

void loop() {

//________________read pressure sensor 
currentPressure = (analogRead(pressurePin));

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
       
ACC_CMD_PERCENT = ((100/(maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));

//________________calculating targetPressure
targetPressure = (((ACC_CMD_PERCENT / 100) * (maxPressure - minPressure)) + minPressure); // conversion from ACC_CMD_PERCENT % into targetpressure
 
//________________press or release the pedal to match targetPressure
if (abs(currentPressure - targetPressure) >= PERM_ERROR)
  {
    open_solenoid = false;
    if ((currentPressure < targetPressure) && (ACC_CMD_PERCENT > 0))
        { 
        analogWrite(M_PWM, 255);  //run Motor
        digitalWrite(M_DIR, HIGH); //motor driection left | press the pedal
        releasing_by_OP = false;
        }    
    else if (currentPressure > targetPressure)
        {       
        analogWrite(M_PWM, 255);   //run Motor
        digitalWrite(M_DIR, LOW); //motor driection right | release the pedal
        releasing_by_OP = true;
        }
    else if (ACC_CMD_PERCENT == 0)
        {       
        analogWrite(M_PWM, 255);   //run Motor
        digitalWrite(M_DIR, LOW); //motor driection right | release the pedal
        releasing_by_OP = false;
        }
  }  
else 
    {
     open_solenoid = false;
     analogWrite(M_PWM, 0);   //if we match target position, just stay here
     releasing_by_OP = false;
    }  

//________________logic to read if brake is pressed by driver   
if ((currentPressure >= (targetPressure + brake_pressed_threshold)) && !releasing_by_OP)
    {
        BRAKE_PRESSED = true;
        open_solenoid = true;
    }
    
else {
        BRAKE_PRESSED = false;
     }
 
//________________operate solenoid
 if (open_solenoid)
    { 
    analogWrite(S_PWM, 0);
    }
 else
    { 
    analogWrite(S_PWM, 255);
    }

 //________________light up brakelights when brake is pressed
long currentMillis = millis();
if (currentMillis - previousMillis >= 200)
    {  
     if (currentPressure >= (minPressure + brake_light_threshold))
        {
         analogWrite(brakelightPin, 255);
        }
     else 
        {
         analogWrite(brakelightPin, 0);
        }
     previousMillis = currentMillis;
     }

//________________send CAN BUS messages
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
  /*
  Serial.print("ACC_CMD_");
  Serial.print(ACC_CMD);
  Serial.print("_____");
  Serial.print(ACC_CMD_PERCENT);
  Serial.print("_%");
  Serial.print("_____");
  Serial.print("target_");
  Serial.print(targetPressure);
  Serial.print("_____");
  Serial.print("Pressure");
  Serial.println(currentPressure);
  Serial.println("");
  */

}
