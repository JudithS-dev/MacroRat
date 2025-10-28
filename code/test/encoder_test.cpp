/*#include <Arduino.h>
// d = 36 mm
// => u = 113.097 mm
// => 1 Umdrehung = 113.097 mm


#define COM_IS_USED true

#if COM_IS_USED
#include "communication_interface.h"
#define WIFI_SSID "TI Roboter"
#define WIFI_PASSWORD "ITRobot!"
#define MQTT_BROKER "172.16.18.48" //"172.16.8.108" //"172.16.6.148" for Raspberry Pi

CommunicationInterface *communicationInterface = nullptr;
#endif


// === PIN-Zuweisung ===
#define ENCODER_PIN_A_rechts 34  // C1
#define ENCODER_PIN_B_rechts 35  // C2

#define ENCODER_PIN_A_links 2  // C1
#define ENCODER_PIN_B_links 18  // C2

#define ENCODER_PIN_A ENCODER_PIN_A_links  // C1
#define ENCODER_PIN_B ENCODER_PIN_B_links  // C2

// === Encoder-Konfiguration ===
#define TICKS_PER_REV 300//1200  // z. B. bei Pololu #5191 mit 100:1 Getriebe

volatile long encoder_ticks = 0;
volatile uint8_t last_state = 0;

void IRAM_ATTR encoderISR() {
  bool b = digitalRead(ENCODER_PIN_B);
  encoder_ticks += (b ? +1 : -1);
}


void setup(){
  Serial.begin(115200);
  delay(500);

  #if COM_IS_USED
  communicationInterface = new CommunicationInterface(WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, 1883);
  if (communicationInterface == nullptr) {
    Serial.println("Fehler: Kommunikation konnte nicht initialisiert werden.");
    while (true);  // Dauerhalt
  }
  #endif
  
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  uint8_t c1 = digitalRead(ENCODER_PIN_A);
uint8_t c2 = digitalRead(ENCODER_PIN_B);
last_state = (c1 << 1) | c2;
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, RISING);

  
  Serial.println("Encoder-Test mit Umdrehungsausgabe gestartet.");
  #if COM_IS_USED
  communicationInterface->log("Encoder-Test mit Umdrehungsausgabe gestartet.");
  #endif
}

void loop() {
  static long last_ticks = 0;
  static unsigned long last_time = 0;
  
  if(millis() - last_time > 500){
    noInterrupts();
    long current_ticks = encoder_ticks;
    interrupts();
    
    long delta = current_ticks - last_ticks;
    last_ticks = current_ticks;
    
    float revolutions = (float)current_ticks / TICKS_PER_REV;
    float delta_rev = (float)delta / TICKS_PER_REV;
    
    Serial.print("Ticks: ");
    Serial.print(current_ticks);
    Serial.print(" | delta Ticks: ");
    Serial.print(delta);
    Serial.print(" | Umdrehungen: ");
    Serial.print(revolutions, 3);
    Serial.print(" | delta Umdr.: ");
    Serial.println(delta_rev, 3);
    #if COM_IS_USED
    communicationInterface->log("Ticks: %ld | delta Ticks: %ld | Umdrehungen: %.3f | delta Umdr.: %.3f", current_ticks, delta, revolutions, delta_rev);
    #endif
    
    last_time = millis();
  }
}*/