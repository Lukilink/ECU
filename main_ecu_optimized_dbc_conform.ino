#include <CAN.h>

// --- CAN IDs ---
const uint16_t STEER_ANGLE_SENSOR_ID = 0x37;

// --- Variables for STEER_ANGLE_SENSOR ---
float steer_angle = 0.0;        // STEER_ANGLE: 12 bits
float steer_fraction = 0.0;     // STEER_FRACTION: 4 bits
int16_t steer_rate = 0;         // STEER_RATE: 12 bits

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  while (!Serial);

  if (!CAN.begin(500E3)) {  // Initialize CAN at 500 kbps
    Serial.println("Starting CAN failed!");
    while (1);
  }

  Serial.println("CAN initialized. Listening for STEER_ANGLE_SENSOR (0x37)...");
}

void loop() {
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // Check if the received packet matches the STEER_ANGLE_SENSOR ID
    if (CAN.packetId() == STEER_ANGLE_SENSOR_ID) {
      uint8_t data[8] = {0};

      // Read the 8 bytes of data from the CAN packet
      for (int i = 0; i < packetSize && i < 8; i++) {
        data[i] = CAN.read();
      }

      // Decode the signals from the data bytes
      steer_angle = ((int16_t)((data[0] << 4) | (data[1] >> 4))) * 1.5; // STEER_ANGLE: Bits 3-0 in byte 0 and bits 7-4 in byte 1
      steer_fraction = ((int8_t)((data[4] & 0xF0) >> 4)) * 0.1;          // STEER_FRACTION: Bits 39-36 in byte 4
      steer_rate = ((int16_t)((data[4] & 0x0F) << 8) | data[5]) * 1;     // STEER_RATE: Bits 35-24 in byte 4 and byte 5

      // Print the decoded signals to the serial monitor
      Serial.println("STEER_ANGLE_SENSOR (0x37) received:");
      Serial.print("  STEER_ANGLE: ");
      Serial.print(steer_angle);
      Serial.println(" deg");
      Serial.print("  STEER_FRACTION: ");
      Serial.print(steer_fraction);
      Serial.println(" deg");
      Serial.print("  STEER_RATE: ");
      Serial.print(steer_rate);
      Serial.println(" deg/s");
    }
  }
}
