/**
 * Testet die Motorensteuerung, indem die Geschwindigkeit der Motoren
 * in regelmäßigen Abständen verringert wird, bis sie 0 erreicht.
 * 
 * Ergebnis: Ab Geschwindigkeit 400 bewegen sich die Motoren kaum noch sinnvoll.
 * => Sinnvolle Minimalgeschwindigkeit = 400
 * 
 */
/*
#include <Arduino.h>

#define COM_IS_USED true

#if COM_IS_USED
#include "communication_interface.h"
#define WIFI_SSID "TI Roboter"
#define WIFI_PASSWORD "ITRobot!"
#define MQTT_BROKER "172.16.18.48" //"172.16.8.108" //"172.16.6.148" for Raspberry Pi

CommunicationInterface *communicationInterface = nullptr;
#endif

#include "motor_driver.h"

#define START_SPEED 500
#define SPEED_STEP  50
#define MIN_SPEED   0
#define INTERVAL_MS 5000  // alle 2 Sekunden

MotorDriver motor;

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
  
  motor = MotorDriver();
  Serial.println("Starte Geschwindigkeits-Test...");
  #if COM_IS_USED
  communicationInterface->log("Starte Geschwindigkeits-Test...");
  #endif
}

void loop() {
  static uint16_t current_speed = START_SPEED;
  static unsigned long last_change = 0;

  if (millis() - last_change >= INTERVAL_MS) {
    last_change = millis();

    if (current_speed >= MIN_SPEED) {
      Serial.print("Setze Geschwindigkeit: ");
      Serial.println(current_speed);
      #if COM_IS_USED
      communicationInterface->log("Setze Geschwindigkeit: %d", current_speed);
      #endif
      motor.stopMotors();  // Stoppe die Motoren vor der Geschwindigkeitsänderung
      delay(1000);
      motor.setLeftMotor(current_speed, MotorDirection::FORWARD);
      motor.setRightMotor(current_speed, MotorDirection::FORWARD);

      current_speed -= SPEED_STEP;
    } else {
      motor.stopMotors();
      Serial.println("Test beendet: Geschwindigkeit erreicht 0.");
      #if COM_IS_USED
      communicationInterface->log("Test beendet: Geschwindigkeit erreicht 0.");
      #endif
      while (true);  // Dauerhalt
    }
  }
}
*/