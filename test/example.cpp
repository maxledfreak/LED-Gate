#include <Wire.h>
#include <Arduino.h>

// I2C-Adresse des Empfängers
#define I2C_ADDRESS 0x08

// Timer-Handle
hw_timer_t *timer = NULL;

// Empfangspuffer
volatile bool messageReceived = false;
volatile uint8_t receivedData[32];
volatile size_t receivedLength = 0;

// Funktion, die vom Timer-Interrupt aufgerufen wird
void IRAM_ATTR onTimer() {
  // Startet die I²C-Nachrichtensendung
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write("Hello ESP32!");  // Nachricht senden
  Wire.endTransmission();
}

// Funktion, die vom I²C-Empfangs-Interrupt aufgerufen wird
void onReceive(int numBytes) {
  // Empfangene Daten speichern
  if (numBytes > 0) {
    receivedLength = numBytes;
    for (size_t i = 0; i < numBytes; i++) {
      receivedData[i] = Wire.read();
    }
    messageReceived = true;  // Empfangsflag setzen
  }
}

void setup() {
  // Serielle Kommunikation starten
  Serial.begin(115200);
  Serial.println("ESP32 I2C Interrupt Example");

  // I²C als Master initialisieren
  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22
  Wire.onReceive(onReceive); // Empfangs-Interrupt setzen

  // Timer konfigurieren (100 ms = 100000 µs)
  timer = timerBegin(0, 80, true); // Timer 0, Prescaler 80 (1 µs Tick)
  timerAttachInterrupt(timer, &onTimer, true); // Timer-Interrupt verbinden
  timerAlarmWrite(timer, 100000, true); // 100 ms Periodendauer
  timerAlarmEnable(timer); // Timer-Interrupt aktivieren
}

void loop() {
  // Falls eine Nachricht empfangen wurde, ausgeben
  if (messageReceived) {
    messageReceived = false; // Flag zurücksetzen

    Serial.print("Received message: ");
    for (size_t i = 0; i < receivedLength; i++) {
      Serial.print((char)receivedData[i]);
    }
    Serial.println();
  }

  delay(10); // Optional, um die CPU etwas zu entlasten
}
