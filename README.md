# ECU

!!! DO NOT USE ON PUBLIC ROAD !!!

Code for Arduino based ECU: 

BRAKE.ino 
  - reads brake message on can bus (send by Openpilot / Panda)
  - Reads brake pressure from Sensor: ST749 Bremsdrucksensor, 3/8-UNF 100 bar
  - drive Motor (cruise control aktuator) which is attached to Brake Pedal

GAS.ino
  - reads gas message on can bus (send by Openpilot / Panda
  - Reads potentiometer on Throttel
  - drived motor (cruise control aktuator) which is attached to throttle.
 
MAIN.ino
  - Reads speed signal from hall sensor: https://speedpuls.de
  - sends speed message on Can bus
  - reads button state  Enabled/Disabled/increase speed / decrease speed)
  - sends message on Can bus to enable / disable openpilot
  - sends message on Can bus to adjust set speed

MAIN_DAYWALKER_SPEED.ino
   - Same like MAIN.ino but with diffrent method to read the speed signal from hall sensor (smoother/less noisy)

Evry singe code(ECU) sends messages on the can bus, which are required to enable OP. This means, if one ECU is missing incase of a failure, OP will not engage.
GAS + BRAKE uses a method to detect human input. If humaninput on Brake or Gas pedal is detected, it disables openpilot immediately. 
You need to adjust sensor values for your  specific sensors. 

HARDWARE that I use to build the ECU: 

Arduino uno: 
https://www.amazon.de/Arduino-Uno-Rev-3-Mikrocontroller-Board/dp/B008GRTSV6/ref=asc_df_B008GRTSV6/?tag=googshopde-21&linkCode=df0&hvadid=309008177512&hvpos=&hvnetw=g&hvrand=10902457370359064208&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9042366&hvtargid=pla-457497319401&psc=1&th=1&psc=1&tag=&ref=&adgrpid=65257070361&hvpone=&hvptwo=&hvadid=309008177512&hvpos=&hvnetw=g&hvrand=10902457370359064208&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9042366&hvtargid=pla-457497319401

Can bus shiel:
MCP2515 EF02037 CAN BUS Shield communication SPI Controller For Arduino

Motor Shield (not needed for Main.ino ECU): 
Cytron 10A DC Motor Treiber Arduino Shield
https://www.robotshop.com/de/de/cytron-10a-dc-motor-treiber-arduino-shield.html?gclid=CjwKCAiAz--OBhBIEiwAG1rIOtq9lzE9XKenDgZFCtUJ_VIgl4X1wVGqAu6yuw4j7MSbVEsXjBfUaRoCnGAQAvD_BwE






