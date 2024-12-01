#include <Arduino.h>
#include <WS2812FX.h>
#include <Wire.h>

#define LED_COUNT 10
#define LED_PIN 13
#define BUTTON_PIN 4       // Menü Auswahl
#define DEBOUNCE_DELAY 300  // 50 ms Entprellzeit

#define TIMER_MS 5000

#define SENSOR_ADDRESS 0x57  // I2C address of the sensor
int bcount; // Define counter to count bytes in response
byte distance[4]; // Define array for return data

// put function declarations here:
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

volatile unsigned long last_change = 0;        // Für Random Funktion
volatile unsigned long lastInterruptTime = 0;  // Zeitstempel des letzten Interrupts
unsigned long now = 0;
enum State {
    OFF,
    EMPTY,
    CHARGE,
    GOAL,
    RANDOM,
    CUSTOM1,
    CUSTOM2,
    CUSTOM3,
    CUSTOM4,
    CUSTOM5
};

State currentState = RANDOM;
State lastState = RANDOM;
const State firstSelectableState = RANDOM;  // Erster schaltbarer Zustand
const State lastSelectableState = CUSTOM5;  // Letzter schaltbarer Zustand

// Interrupt-Service-Routine (ISR) für den Taster
void IRAM_ATTR handleButtonPress() {
    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime > DEBOUNCE_DELAY) {
        // Zustand wechseln
        if (currentState < lastSelectableState) {
            currentState = (State)(currentState + 1);
        } else {
            currentState = firstSelectableState;
        }
        lastInterruptTime = currentTime;
    }
}

float getdistance() {
  // Start a ranging session by writing '1' to the sensor
  Wire.beginTransmission(SENSOR_ADDRESS);
  Wire.write(0x01);  // Command to start ranging
  Wire.endTransmission();
  // Request 3 bytes from the sensor
  Wire.requestFrom(SENSOR_ADDRESS, 3);
  if (Wire.available() == 3) {
    uint32_t distance_um = 0;

    // Read 3 bytes and combine them into a single value
    distance_um |= Wire.read() << 16;  // MSB
    distance_um |= Wire.read() << 8;   // Middle byte
    distance_um |= Wire.read();        // LSB

    // Convert distance from micrometers to millimeters
    float distance_mm = distance_um / 1000.0;

    // Print the distance
    Serial.print("Distance: ");
    Serial.print(distance_mm);
    Serial.println(" mm");
    return distance_mm;
  } else {
    Serial.println("Failed to read distance!");
    return -1;
  }
}

void setup() {
    Serial.begin(115200);
    // Alles für den LED Stripe

    ws2812fx.init();
    ws2812fx.setBrightness(255);
    ws2812fx.setSpeed(1000);
    ws2812fx.setColor(0x007BFF);
    ws2812fx.setMode(FX_MODE_STATIC);
    ws2812fx.start();

    // Interrupt auf fallende Flanke (LOW) konfigurieren
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Taster als Eingang mit Pull-Up-Widerstand konfigurieren
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

    Serial.println("INIT DONE");
}

void loop() {
    
    now = millis();
    float sensor_distance = getdistance();
    ws2812fx.service();
    if (currentState != lastState) {
      Serial.print("Neuer Zustand: ");
      Serial.println(currentState);
      //lastState = currentState;
    }
      switch (currentState) {
              case OFF:
                lastState = currentState;
                  break;

              case EMPTY:
                lastState = currentState;
                  break;

              case CHARGE:
                lastState = currentState;
                  break;

              case GOAL:
                lastState = currentState;
                  break;

              case RANDOM:
                    if(now - last_change > TIMER_MS) {
                    ws2812fx.setMode((ws2812fx.getMode() + 1) % ws2812fx.getModeCount());
                    last_change = now;
                    }
                    lastState = currentState;
                  break;

              case CUSTOM1:
                    ws2812fx.setMode(FX_MODE_STATIC);
                    ws2812fx.setColor(255, 0, 0);  // Alle LEDs auf Rot setzen
                    lastState = currentState;
                  break;

              case CUSTOM2:
                    if (currentState != lastState) {
                      ws2812fx.setMode(FX_MODE_DUAL_SCAN);
                      ws2812fx.setSpeed(1000);
                      lastState = currentState;
                    }
                  break;     

              case CUSTOM3:
                    ws2812fx.setMode(FX_MODE_STATIC);
                    ws2812fx.setColor(0, 0, 255);
                    lastState = currentState;
                  break;

              case CUSTOM4:
                    ws2812fx.setMode(FX_MODE_STATIC);
                    ws2812fx.setColor(255, 0, 255);
                    lastState = currentState;
                  break;

              case CUSTOM5:
                    ws2812fx.setMode(FX_MODE_STATIC);
                    ws2812fx.setColor(255, 255, 255);
                    lastState = currentState;
                  break;                      

              default:

              break;
      
      }
 




}
