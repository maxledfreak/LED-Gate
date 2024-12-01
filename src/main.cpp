#include <Arduino.h>
#include <WS2812FX.h>

#define LED_COUNT 10
#define LED_PIN 13
#define BUTTON_PIN 4       // Menü Auswahl
#define DEBOUNCE_DELAY 300  // 50 ms Entprellzeit

#define TIMER_MS 5000

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
