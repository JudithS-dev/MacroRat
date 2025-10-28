/*#include <Arduino.h>

#define COM_IS_USED true

#if COM_IS_USED
#include "communication_interface.h"
#define WIFI_SSID "TI Roboter"
#define WIFI_PASSWORD "ITRobot!"
#define MQTT_BROKER "172.16.18.48" //"172.16.8.108" //"172.16.6.148" for Raspberry Pi

CommunicationInterface *communicationInterface = nullptr;
#endif

#include "motor_driver.h"

#define ENCODER_PIN_A 34
#define ENCODER_PIN_B 35

#define TICKS_PER_REV 34.6f
#define TARGET_REVOLUTIONS 10.0f
#define BASE_SPEED 800
#define MIN_SPEED 400
#define MAX_ADJUST 200

MotorDriver motor;

// --- Globale Tickzählung (rechter Motor) ---
volatile long encoder_ticks = 0;

// ISR für Encoder C1
void IRAM_ATTR encoderISR() {
  bool b = digitalRead(ENCODER_PIN_B);
  encoder_ticks += (b ? +1 : -1);
}

// Sicheres Auslesen der Ticks
long getEncoderTicks() {
  noInterrupts();
  long ticks = encoder_ticks;
  interrupts();
  return ticks;
}

void setup() {
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, RISING);

  motor = MotorDriver();

  Serial.println("Starte Vorwärtsfahrt: 10 Umdrehungen...");
  #if COM_IS_USED
  communicationInterface->log("Starte Vorwärtsfahrt: 10 Umdrehungen...");
  #endif
}

void loop() {
  long ticks = getEncoderTicks();
  float revolutions = ticks / TICKS_PER_REV;
  float remaining = TARGET_REVOLUTIONS - revolutions;

  if (remaining > 0.05) {
    // Dynamische Basisgeschwindigkeit basierend auf Fortschritt
    float progress = revolutions / TARGET_REVOLUTIONS;
    float dynamic_base = BASE_SPEED * (1.0 - progress);
    if (dynamic_base < MIN_SPEED) dynamic_base = MIN_SPEED;

    // Noch kein linker Encoder – Differenzregelung symbolisch
    long delta_left = ticks;
    long delta_right = ticks;

    long error = delta_left - delta_right;
    int correction = constrain(error * 5, -MAX_ADJUST, MAX_ADJUST);

    uint16_t left_speed = dynamic_base + correction;
    uint16_t right_speed = dynamic_base - correction;

    motor.setLeftMotor(left_speed, MotorDirection::FORWARD);
    motor.setRightMotor(right_speed, MotorDirection::FORWARD);
  }
  else {
    motor.stopMotors();
    Serial.println("Ziel erreicht: 10 Umdrehungen vollendet.");
    #if COM_IS_USED
    communicationInterface->log("Ziel erreicht: 10 Umdrehungen vollendet.");
    #endif
    while (true);  // Dauerschleife → bleibt stehen
  }

  // Serielle Statusausgabe alle 500 ms
  static unsigned long last_time = 0;
  if (millis() - last_time > 500) {
    Serial.print("Ticks: ");
    Serial.print(ticks);
    Serial.print(" | Umdr.: ");
    Serial.print(revolutions, 2);
    Serial.print(" | Speed L: ");
    Serial.print(motor.getLeftMotorSpeed());
    Serial.print(" | Speed R: ");
    Serial.println(motor.getRightMotorSpeed());
    #if COM_IS_USED
    communicationInterface->log("Ticks: %ld | Umdr.: %.2f | Speed L: %d | Speed R: %d", ticks, revolutions, motor.getLeftMotorSpeed(), motor.getRightMotorSpeed());
    #endif
    last_time = millis();
  }
}*/