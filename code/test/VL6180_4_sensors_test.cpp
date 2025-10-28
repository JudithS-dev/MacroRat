#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "error_handler.h"

#include "communication_interface.h"
#define WIFI_SSID "TI Roboter"
#define WIFI_PASSWORD "ITRobot!"
#define MQTT_BROKER "172.16.6.148" //"172.16.18.48" //"172.16.8.108" //"172.16.6.148" for Raspberry Pi

CommunicationInterface *communicationInterface = nullptr;

// define SDA and SCL pins for I2C communication (default: SDA = GPIO21, SCL = GPIO22)
#define SDA_PIN 21
#define SCL_PIN 22

// XSHUT-GPIOs for the sensors (any GPIO pins can be used)
#define XSHUT_LEFT         26
#define XSHUT_MIDDLE_LEFT  33
#define XSHUT_MIDDLE_RIGHT 25
#define XSHUT_RIGHT        32

// new VL6180X I2C addresses (any address between 0x08 and 0x77 can be used)
// VL6180X default address is 0x29
#define ADDR_LEFT          0x30
#define ADDR_MIDDLE_LEFT   0x31
#define ADDR_MIDDLE_RIGHT  0x32
#define ADDR_RIGHT         0x33

#define NR_READINGS 250

// objects for the VL6180X sensors
Adafruit_VL6180X sensorLeft = Adafruit_VL6180X();
Adafruit_VL6180X sensorMiddleLeft = Adafruit_VL6180X();
Adafruit_VL6180X sensorMiddleRight = Adafruit_VL6180X();
Adafruit_VL6180X sensorRight = Adafruit_VL6180X();

void initSensors(){
  // set XSHUT pins as OUTPUT
  pinMode(XSHUT_LEFT,         OUTPUT);
  pinMode(XSHUT_MIDDLE_LEFT,  OUTPUT);
  pinMode(XSHUT_MIDDLE_RIGHT, OUTPUT);
  pinMode(XSHUT_RIGHT,        OUTPUT);
  
  // set XSHUT pins to LOW to reset all sensors
  digitalWrite(XSHUT_LEFT,         LOW);
  digitalWrite(XSHUT_MIDDLE_LEFT,  LOW);
  digitalWrite(XSHUT_MIDDLE_RIGHT, LOW);
  digitalWrite(XSHUT_RIGHT,        LOW);
  delay(10);
  
  // set XSHUT pins to HIGH to wake up all sensors
  digitalWrite(XSHUT_LEFT,         HIGH);
  digitalWrite(XSHUT_MIDDLE_LEFT,  HIGH);
  digitalWrite(XSHUT_MIDDLE_RIGHT, HIGH);
  digitalWrite(XSHUT_RIGHT,        HIGH);
  delay(10);
  
  // reset all sensors except the left sensor
  digitalWrite(XSHUT_MIDDLE_LEFT,  LOW);
  digitalWrite(XSHUT_MIDDLE_RIGHT, LOW);
  digitalWrite(XSHUT_RIGHT,        LOW);
  delay(10);
  
  // activate left sensor and set new I2C address for it
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if(!sensorLeft.begin())
    fatalError("ERROR: sensorLeft not found!");
  sensorLeft.setAddress(ADDR_LEFT);
  delay(10);
  
  // activate middle left sensor and set new I2C address for it
  digitalWrite(XSHUT_MIDDLE_LEFT, HIGH);
  delay(10);
  if(!sensorMiddleLeft.begin())
    fatalError("ERROR: sensorMiddleLeft not found!");
  sensorMiddleLeft.setAddress(ADDR_MIDDLE_LEFT);
  delay(10);
  
  // activate middle right sensor and set new I2C address for it
  digitalWrite(XSHUT_MIDDLE_RIGHT, HIGH); 
  delay(10);
  if(!sensorMiddleRight.begin())
    fatalError("ERROR: sensorMiddleRight not found!");
  sensorMiddleRight.setAddress(ADDR_MIDDLE_RIGHT); 
  delay(10);
  
  // activate right sensor and set new I2C address for it
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if(!sensorRight.begin())
    fatalError("ERROR: sensorRight not found!");
  sensorRight.setAddress(ADDR_RIGHT);
  delay(10);
}

void setupTest() {
  // initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB devices
  }
  communicationInterface = new CommunicationInterface(WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, 1883);
  if (communicationInterface == nullptr) {
    Serial.println("Fehler: Kommunikation konnte nicht initialisiert werden.");
    while (true);  // Dauerhalt
  }
  CommunicationInterface::log("VL6180X 4 Sensors Test: Starting initialization...");

  // set SDA and SCL pins for I2C communication 
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // initialize the sensors
  initSensors();
  CommunicationInterface::log("VL6180X 4 Sensors Test: Initialization complete!");
}

/*
void printSensorData(const char* label, Adafruit_VL6180X& sensor) {
  uint16_t sum = 0;
  uint8_t min_val = 255, max_val = 0, valid = 0;
  uint8_t distance, status;
  
  for (int i = 0; i < NR_READINGS; i++){
    distance = sensor.readRange();
    status = sensor.readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
      sum += distance;
      min_val = min(min_val, distance);
      max_val = max(max_val, distance);
      valid++;
    }
    delay(10);
  }
  
  Serial.print(label);
  Serial.print(" (0x");
  Serial.print(sensor.getAddress(), HEX);
  Serial.print(") -> ");

  if (valid > 0) {
    Serial.print("Avg: ");
    Serial.print(sum / valid);
    Serial.print(" mm [min: ");
    Serial.print(min_val);
    Serial.print(" | max: ");
    Serial.print(max_val);
    Serial.println("]");
    CommunicationInterface::log("%s: Avg: %d mm [min: %d | max: %d]", label, sum / valid, min_val, max_val);
  } else {
    Serial.println("Keine gültigen Messwerte.");
    CommunicationInterface::log("%s: Keine gültigen Messwerte.", label);
  }
}*/

/*
void printSensorData(const char* label, Adafruit_VL6180X& sensor) {
  uint8_t distance = sensor.readRange();
  uint8_t status = sensor.readRangeStatus();

  Serial.print(label);
  Serial.print(" (0x");
  Serial.print(sensor.getAddress(), HEX);
  Serial.print(") -> ");

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Distanz: ");
    Serial.print(distance);
    Serial.println(" mm");
    CommunicationInterface::log("%s (0x%02X): Distanz: %d mm", label, sensor.getAddress(), distance);
  } else {
    const char* errorMsg = "Unbekannter Fehler";
    switch (status) {
      case VL6180X_ERROR_SYSERR_1:
      case VL6180X_ERROR_SYSERR_5:
        errorMsg = "Systemfehler";
        break;
      case VL6180X_ERROR_ECEFAIL:
        errorMsg = "ECE Fehler";
        break;
      case VL6180X_ERROR_NOCONVERGE:
        errorMsg = "Keine Konvergenz";
        break;
      case VL6180X_ERROR_RANGEIGNORE:
        errorMsg = "Bereich ignoriert";
        break;
      case VL6180X_ERROR_SNR:
        errorMsg = "Signal-Rausch-Verhältnis schlecht";
        break;
      case VL6180X_ERROR_RAWUFLOW:
        errorMsg = "Rohwert untergelaufen";
        break;
      case VL6180X_ERROR_RAWOFLOW:
        errorMsg = "Rohwert übergelaufen";
        break;
      case VL6180X_ERROR_RANGEUFLOW:
        errorMsg = "Messwert unter Bereich";
        break;
      case VL6180X_ERROR_RANGEOFLOW:
        errorMsg = "Messwert über Bereich";
        break;
    }

    Serial.print("Fehler: ");
    Serial.println(errorMsg);
    CommunicationInterface::log("%s (0x%02X): Fehler: %s", label, sensor.getAddress(), errorMsg);
  }
}*/

uint16_t numReadings = NR_READINGS;  // Anzahl der Messungen, die durchgeführt werden sollen
void printSensorData(const char* label, Adafruit_VL6180X& sensor) {
  uint8_t* values = new uint8_t[numReadings];  // dynamisches Array
  if (values == nullptr) {
    Serial.println("Fehler: Speicher konnte nicht zugewiesen werden.");
    CommunicationInterface::log("%s: Fehler bei der Speicherzuweisung für CSV-Ausgabe", label);
    return;
  }
  uint16_t count = 0;

  Serial.print(label);
  Serial.print(" (0x");
  Serial.print(sensor.getAddress(), HEX);
  Serial.println(") CSV-Ausgabe:");

  CommunicationInterface::log("%s (0x%02X): CSV-Ausgabe startet mit %d Messungen", label, sensor.getAddress(), numReadings);

  for (uint16_t i = 0; i < numReadings; ++i) {
    uint8_t distance = sensor.readRange();
    uint8_t status = sensor.readRangeStatus();
    if (status == VL6180X_ERROR_NONE) {
      values[count++] = distance;
    }
    delay(5);  // kurze Pause zwischen den Messungen
  }

  // CSV-Print nur für gültige Messungen
  if (count == 0) {
    Serial.println("Keine gültigen Messwerte gefunden.");
    CommunicationInterface::log("%s: Keine gültigen Messwerte gefunden.", label);
  } else {

  
  // Optional: Auch über CommunicationInterface loggen
  CommunicationInterface::log("%s CSV:", label);
  for (uint16_t i = 0; i < count; ++i) {
    CommunicationInterface::log("%d, %d", i + 1, values[i]);
  }
  
  }

  delete[] values;  // Speicher freigeben
}


void setup() {
  setupTest();
  while (1) {
    printSensorData("LEFT",          sensorLeft);
    printSensorData("MIDDLE_LEFT",   sensorMiddleLeft);
    printSensorData("MIDDLE_RIGHT",  sensorMiddleRight);
    printSensorData("RIGHT",         sensorRight);
    Serial.println();
    delay(2000);
  }
}

void loop() {
  // The main loop is empty because the sensor data is printed in the setup function
  // and the program runs indefinitely.
}
