#include <CAN.h>

// Maximale Anzahl an eindeutigen CAN-IDs, die gespeichert werden können
#define MAX_IDS 100

int uniqueIds[MAX_IDS]; // Array zur Speicherung eindeutiger CAN-IDs
int idCount = 0;        // Zähler für die Anzahl gespeicherter IDs

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Receiver - Einmalige CAN-IDs");

  // Startet den CAN-Bus mit 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starten des CAN-Bus fehlgeschlagen!");
    while (1);
  }
}

void loop() {
  // Versucht, ein Paket zu lesen
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    // Empfangene CAN-ID auslesen
    int packetId = CAN.packetId();

    // Prüfen, ob die ID bereits im Array enthalten ist
    bool found = false;
    for (int i = 0; i < idCount; i++) {
      if (uniqueIds[i] == packetId) {
        found = true;
        break;
      }
    }

    // Wenn die ID nicht gefunden wurde, füge sie hinzu und gebe sie aus
    if (!found && idCount < MAX_IDS) {
      uniqueIds[idCount++] = packetId;
      Serial.print("0x");
      Serial.println(packetId, HEX);
    }
  }
}
