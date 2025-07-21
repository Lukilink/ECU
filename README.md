1. Setup-Phase
setup() Funktion:
Startet die serielle Kommunikation mit Serial.begin(9600) (für Debugging).
Initialisiert den CAN-Bus mit einer Baudrate von 500 kbps.
Konfiguriert die Pins für:
Tasten (buttonPins), die als Eingänge definiert sind.
Blinker-Pins (BlinkerPinLeft, BlinkerPinRight), die mit INPUT_PULLUP konfiguriert werden.
Kupplungsschalter (ClutchSwitchPin).
Interrupt-Pin (interruptPin), der auf fallende Flanken reagiert und den VSS (Vehicle Speed Sensor) zählt.
Bindet die ISR (Interrupt Service Routine) rpm() an den Interrupt-Pin.
2. Hauptschleife (loop())
2.1 Geschwindigkeit messen und glätten
Wenn mindestens eine halbe Umdrehung des VSS gezählt wurde:
Interrupts deaktivieren (noInterrupts()), um sicherzustellen, dass die Berechnungen nicht gestört werden.
Berechnet die Zeit zwischen den letzten Umdrehungen.
Aktualisiert die Geschwindigkeit (spd) basierend auf der Anzahl der halben Umdrehungen.
Setzt den Zähler (half_revolutions) zurück und aktiviert die Interrupts wieder.
Geschwindigkeitsglättung:
Entfernt den ältesten Wert aus den gespeicherten Messwerten (readings).
Fügt den neuen Geschwindigkeitswert hinzu.
Berechnet den Durchschnitt (average).
2.2 Status der Tasten und Blinker
Liest den Status der Kupplung (ClutchSwitchPin).
Liest die Tastenstatus (buttonPins) und vergleicht sie mit den vorherigen Werten (lastButtonStates).
Liest die Blinkerstatus:
blinker_left: Aktiv, wenn der linke Blinker eingeschaltet ist.
blinker_right: Aktiv, wenn der rechte Blinker eingeschaltet ist.
2.3 Logik für den OP-Modus
Schaltet den OP-Modus (OP_ON) aus, wenn:
Die Bremse gedrückt ist.
Das Gaspedal nicht freigegeben ist.
Überwacht die Tasten:
Taste 4: OP-Modus aktivieren/deaktivieren. Wenn aktiviert, setzt die Geschwindigkeit (set_speed) auf den aktuellen Durchschnitt plus 3 km/h.
Taste 3: Erhöht die Geschwindigkeit um 5 km/h.
Taste 2: Verringert die Geschwindigkeit um 5 km/h.
Taste 1: Deaktiviert den OP-Modus.
Begrenzung der Geschwindigkeit:
Setzt set_speed auf 0, wenn der Wert größer als 200 km/h ist.
Aktualisiert die vorherigen Tastenstatus (lastButtonStates).
2.4 Senden von CAN-Nachrichten
Ruft die entsprechenden Funktionen auf, um die CAN-Nachrichten zu senden.
3. CAN-Nachrichtenfunktionen
Die folgenden Nachrichten werden gesendet:

PCM_CRUISE (0x1D2)

Überträgt den OP-Modus und Gaspedalstatus.
Berechnet eine Prüfsumme (dbc_checksum).
PCM_CRUISE_2 (0x1D3)

Überträgt den Hauptstatus und die eingestellte Geschwindigkeit.
WHEEL_SPEEDS (0x170)

Überträgt die berechnete Geschwindigkeit für alle vier Räder.
LIGHT_STALK (0x1570)

Simuliert den Status der Lichtsteuerung, z. B. Abblendlicht und Tagfahrlicht.
BLINKERS_STATE (0x1556)

Überträgt den Status der Blinker:
Linker Blinker: blinker_left.
Rechter Blinker: blinker_right.
BODY_CONTROL_STATE (0x1568)

Simuliert den Zustand der Türen und des Parkbremsstatus.
BODY_CONTROL_STATE_2 (0x1552)

Simuliert die Helligkeit der Anzeige und andere Kontrollwerte.
ESP_CONTROL (0x951)

Überträgt den Status des Stabilitätskontrollsystems (ESP).
BRAKE_MODULE (0x548)

Simuliert, dass die Bremse nicht gedrückt ist und kein Bremsdruck vorhanden ist.
PCM_CRUISE_SM (0x921)

Simuliert den Zustand des Tempomats.
VSC1S07 (0x800)

Überträgt normale Werte für das Stabilitätskontrollsystem.
ENGINE_RPM (0x452)

Simuliert eine Motordrehzahl von 3000 U/min.
GEAR_PACKET (0x956)

Überträgt den aktuellen Gangstatus (z. B. "Drive" eingelegt).
PRE_COLLISION_2 (0x836)

Simuliert den normalen Zustand des Pre-Collision-Systems.
