#include <Arduino.h>
#include <WS2812FX.h>
#include <Wire.h>

#define LED_COUNT 150
#define LED_PIN 25
#define BUTTON_PIN 26        // Menü Auswahl
#define CHARGE_PIN 35       // Ladezustand 
#define VOLTAGE_PIN 34       // Spannung 
#define DEBOUNCE_DELAY 300  // 50 ms Entprellzeit

#define I2C_ADDRESS 0x57  // I2C-Adresse des Ultraschall Sensors
#define TIMER_MS 5000

// put function declarations here:
WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

// Timer-Handle
hw_timer_t *timer = NULL;
hw_timer_t *timer_adc = NULL;  // Timer für ADC-Wert

volatile unsigned long last_change = 0;         // Für Random Funktion
volatile unsigned long lastInterruptTime = 0;   // Zeitstempel des letzten Interrupts
unsigned long now = 0;
volatile uint8_t receivedData[32];              // i²C Daten
volatile bool sendI2CFlag = false;
volatile bool goalflag = true;
volatile uint goaltime = 0;

float    distance = 0;                          // Distance data decimal value
volatile int adcValue = 0;  // ADC-Wert (global, da in der ISR aktualisiert)
volatile bool newReading = false;  // Flag, dass ein neuer Wert verfügbar ist

enum i2c_State{
  READ,
  WRITE,
};
i2c_State current_i2c_State=WRITE;

enum State {
    OFF,
    EMPTY,
    CHARGE,
    WAIT,
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
State GOALState = GOAL;
const State firstSelectableState = RANDOM;      // Erster schaltbarer Zustand
const State lastSelectableState = CUSTOM5;      // Letzter schaltbarer Zustand

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

// Funktion, die vom Timer-Interrupt aufgerufen wird
void IRAM_ATTR onTimer() {
      sendI2CFlag = true;
}

// Timer-Interrupt-Service-Routine für den ADC-Timer
void IRAM_ATTR onTimerADC() {
    adcValue = analogRead(VOLTAGE_PIN);  // Spannung einlesen
    newReading = true;  // Flag setzen, dass ein neuer Wert vorliegt
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

    // Interrupt auf fallende Flanke (LOW) konfigurieren (Menue Taster)
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Taster als Eingang mit Pull-Up-Widerstand konfigurieren
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress,FALLING);
    
    // Pin um den Ladezustand einzulesen
    pinMode(CHARGE_PIN, INPUT_PULLDOWN);

    // Pin zum einlesen der aktuellen Akkuspannung
    pinMode(VOLTAGE_PIN, INPUT);
        // ADC initialisieren
    analogReadResolution(12);  // ADC-Auflösung auf 12 Bit (ESP32) setzen (0-4095)
    analogSetAttenuation(ADC_11db);  // Einstellen der ADC-Spannung (bis zu 3,3 V)

    // I²C als Master initialisieren
    Wire.begin(32, 33); // SDA = GPIO21, SCL = GPIO22

    // Timer konfigurieren (100 ms = 100000 µs)
    timer = timerBegin(0, 80, true); // Timer 0, Prescaler 80 (1 µs Tick)
    timerAttachInterrupt(timer, &onTimer, true); // Timer-Interrupt verbinden
    timerAlarmWrite(timer, 100000, true); // 100 ms Periodendauer
    timerAlarmEnable(timer); // Timer-Interrupt aktivieren

    // Timer für das Einlesen der Spannung konfigurieren
    timer_adc = timerBegin(1, 80, true);  // Timer 0, Prescaler 80 (1 µs Tick)
    timerAttachInterrupt(timer_adc, &onTimerADC, true);  // Timer-Interrupt für ADC verbinden
    timerAlarmWrite(timer_adc, 1000000, true);  // Timer-Interrupt alle 1 Sekunde auslösen
    timerAlarmEnable(timer_adc);  // Timer aktivieren

    Serial.println("INIT DONE");
}

void loop() {
    
    now = millis();
    ws2812fx.service();
    //if (currentState != lastState) {
    //Serial.print("Neuer Zustand: ");
    //Serial.println(currentState);
    //lastState = currentState;
    
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

              case WAIT:
                lastState = currentState;
                  break;

              case GOAL:
                if (currentState != lastState) {
                  Serial.printf("GOALCASE");
                  lastState = currentState;
                  ws2812fx.setSpeed(100);
                  ws2812fx.setColor(255, 255, 255);
                  ws2812fx.setMode(FX_MODE_STROBE);
                }
                  break;

              case RANDOM:
                if(now - last_change > TIMER_MS) {
                  ws2812fx.setSpeed(1000);
                  ws2812fx.setMode((ws2812fx.getMode() + 1) % ws2812fx.getModeCount());
                  last_change = now;
                  Serial.printf("NewMode");
                  Serial.println(ws2812fx.getMode());
                  lastState = currentState;
                    }
                    
                  break;

              case CUSTOM1:
                if (currentState != lastState) {
                  ws2812fx.setMode(FX_MODE_STATIC);
                  ws2812fx.setColor(255, 0, 0);  // Alle LEDs auf Rot setzen
                  lastState = currentState;
                }
                  break;

              case CUSTOM2:
                if (currentState != lastState) {
                  ws2812fx.setMode(FX_MODE_DUAL_SCAN);
                  ws2812fx.setSpeed(1000);
                  lastState = currentState;
                }
                  break;     

              case CUSTOM3:
                if (currentState != lastState) {
                  ws2812fx.setMode(FX_MODE_STATIC);
                  ws2812fx.setColor(0, 0, 255);
                  lastState = currentState;
                }
                  break;

              case CUSTOM4:
                if (currentState != lastState) {
                  ws2812fx.setMode(FX_MODE_STATIC);
                  ws2812fx.setColor(255, 0, 255);
                  lastState = currentState;
                }
                  break;

              case CUSTOM5:
                if (currentState != lastState) {
                  ws2812fx.setMode(FX_MODE_STATIC);
                  ws2812fx.setColor(255, 255, 255);
                  lastState = currentState;
                }
                  break;                      

              default:

              break;
      
      }
 
    // I²C Daten werden abgerufen

      switch (current_i2c_State) {
          case WRITE:
            if (sendI2CFlag) {
              sendI2CFlag = false;
              Wire.beginTransmission(I2C_ADDRESS);
              Wire.write(0X01);                   // Write command 0X01, 0X01 is the start measurement command
              Wire.endTransmission();
              current_i2c_State=READ;
              timerAlarmWrite(timer, 120000, true); // 100 ms Periodendauer
              //150ms Delay, min 120 ms
            }
              break;

          case READ:
            if (sendI2CFlag) {
              sendI2CFlag = false;
              Wire.requestFrom(I2C_ADDRESS,3);
              noInterrupts(); 
              if (Wire.available()) {
                //Serial.print("Received from Slave: ");
                volatile uint8_t i = 0;
              while (Wire.available()){
                receivedData[i++] = Wire.read();
                }
              interrupts(); 
              distance=(receivedData[0]*65536+receivedData[1]*256+receivedData[2])/10000;  //Calculated as CM value  
              //Serial.print("distance：");
              //Serial.println(distance);
              current_i2c_State=WRITE;
              timerAlarmWrite(timer, 20000, true); // 100 ms Periodendauer
              //delay 20 ms
            }
              break;

          default:

          break;
      }

    }
  


    // ISR wird alle Sekunde ausgelöst. Akku Spannung wird eingelesen sowie der Status des Laders abgefragt.
    if (newReading) {
        noInterrupts();  // Unterbrechungen deaktivieren
        int value = adcValue;  // Kopiere den ADC-Wert
        newReading = false;  // Rücksetzen des Flags
        int buttonState = digitalRead(CHARGE_PIN);  // Status Lader 
        interrupts();  // Unterbrechungen wieder aktivieren

        // ADC-Wert in Spannung umrechnen (bei 3,3 V Referenz)
        float voltage = value * (3.3 / 4095.0);
        //Serial.print("Eingelesene Spannung: ");
        //Serial.print(voltage, 3);  // Spannung mit 3 Dezimalstellen
        //Serial.println(" V");

        // Pin-Zustand ausgeben
        if (buttonState == HIGH) {
            //Serial.println("Akku Laden (HIGH).");
            currentState=CHARGE;
        } else {
            //Serial.println("Akku nicht Laden (LOW).");
        }
    }

    //Wir verarbeiten die Distanz

    if((((int)distance)<70) && (((int)distance)>8) && (goalflag==true)){
      goalflag=false;
      goaltime=millis();
      Serial.printf("Treffer, entfernung:");
      Serial.println(distance);
      GOALState = currentState;
      currentState=GOAL;
    }
    else if((((int)distance)>70) && (goalflag==false) && ((millis()-goaltime)>5000)){
      goalflag=true;
      currentState=GOALState;
      Serial.printf("Reset");
    }
}
