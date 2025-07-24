/* BRAKE
 * DONT USE ON PUBLIC ROADS!
*/

//________________import can libary https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>

//________________this values needs to be define for each car
int PERM_ERROR = 2; // will allow a difference between targetPressure and currentPressure
int maxPressure = 190; // the max pressure your actuator is able to apply
int minPressure = 100; // the pressure in stand still
float maxACC_CMD = 20.0;  // the max Value which comes from OP (DBC: +20 m/s^2)
float minACC_CMD = -20.0; // the min Value which comes from OP (DBC: -20 m/s^2)
float brake_force_factor = 1.0; // Skalierung der Bremskraft: 1.0 = 100%, <1 = weniger, >1 = mehr
int brake_pressed_threshold = 5; // threshold when user input is detected
int brake_light_threshold = 5;   // threshold when brakelights should turn on

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

  //________________read ACCEL_CMD from CANbus (DBC-konform)
  CAN.parsePacket();

  if (CAN.packetId() == 0x343)
  {
      uint8_t dat[8];
      for (int ii = 0; ii < 8; ii++) {
          dat[ii]  = (uint8_t) CAN.read();
      }
      // ACCEL_CMD: 16 bit signed, little endian, Startbit 7, DBC: 7|16@0-
      int16_t raw_accel_cmd = (int16_t)(dat[1] << 8 | dat[0]); // Little Endian!
      ACC_CMD = raw_accel_cmd * -0.001; // DBC: Faktor 0.001, Vorzeichenumkehr
  }

  //________________calculating ACC_CMD into ACC_CMD_PERCENT (only negative ACC_CMD relevant for braking)
  if (ACC_CMD <= 0) {
      ACC_CMD_PERCENT = (ACC_CMD / minACC_CMD) * 100.0 * brake_force_factor;
      if (ACC_CMD_PERCENT > 100.0) ACC_CMD_PERCENT = 100.0;
      if (ACC_CMD_PERCENT < 0.0) ACC_CMD_PERCENT = 0.0;
  } else {
      ACC_CMD_PERCENT = 0;
  }

  //________________calculating targetPressure
  targetPressure = (((ACC_CMD_PERCENT / 100.0) * (maxPressure - minPressure)) + minPressure); // conversion from ACC_CMD_PERCENT % into targetpressure

  //________________press or release the pedal to match targetPressure
  if (abs(currentPressure - targetPressure) >= PERM_ERROR)
  {
      open_solenoid = false;
      if ((currentPressure < targetPressure) && (ACC_CMD_PERCENT > 0))
      { 
          analogWrite(M_PWM, 255);  //run Motor
          digitalWrite(M_DIR, HIGH); //motor direction left | press the pedal
          releasing_by_OP = false;
      }    
      else if (currentPressure > targetPressure)
      {       
          analogWrite(M_PWM, 255);   //run Motor
          digitalWrite(M_DIR, LOW); //motor direction right | release the pedal
          releasing_by_OP = true;
      }
      else if (ACC_CMD_PERCENT == 0)
      {       
          analogWrite(M_PWM, 255);   //run Motor
          digitalWrite(M_DIR, LOW); //motor direction right | release the pedal
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

  //________________send CAN BUS messages, DBC-konform (ESP_CONTROL, ID 0x3b7)
  uint8_t dat_3b7[8] = {0};

  // BRAKE_LIGHTS_ACC auf Bit 18 (Byte 2, Bit 2)
  if (BRAKE_PRESSED) {
      dat_3b7[2] |= (1 << 2);
  }

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
