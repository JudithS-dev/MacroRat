/*
*  VL6180X 3 Sensor Distance Test
*  To use this test, connect the VL6180X sensors to the ESP32 board as follows:
*  - VL6180X Left: GPIO17 (XSHUT), GPIO21 (SDA), GPIO22 (SCL)
*  - VL6180X Middle: GPIO18 (XSHUT), GPIO21 (SDA), GPIO22 (SCL)
*  - VL6180X Right: GPIO19 (XSHUT), GPIO21 (SDA), GPIO22 (SCL)
*  - ESP32: GPIO21 (SDA), GPIO22 (SCL)
*  The test will read the distance from each sensor and print the results to the Serial Monitor.

*  To call this test, include "./test/VL6180_3_sensor_distance_test.cpp" in your main file
*  and call runTest() in the setup() function.
*/
/*
#include <Wire.h>
#include "Adafruit_VL6180X.h"
#include "error_handler.h"
*/
/*
Results with NR_READINGS = 250:
LEFT:   Address: 30  Range: 132 mm     [min: 127 | max: 137]
MIDDLE: Address: 31  Range: 135 mm     [min: 129 | max: 141]
RIGHT:  Address: 32  Range: 141 mm     [min: 136 | max: 147]

=> +- 6 mm deviation
*/
/*
// define SDA and SCL pins for I2C communication (default: SDA = GPIO21, SCL = GPIO22)
#define SDA_PIN 21
#define SCL_PIN 22

// XSHUT-GPIOs for the sensors (any GPIO pins can be used)
#define XSHUT_LEFT   17
#define xSHUT_MIDDLE 18
#define XSHUT_RIGHT  19

// new VL6180X I2C addresses (any address between 0x08 and 0x77 can be used)
// VL6180X default address is 0x29
#define ADDR_LEFT   0x30
#define ADDR_MIDDLE 0x31
#define ADDR_RIGHT  0x32

#define NR_READINGS 250

// objects for the VL6180X sensors
Adafruit_VL6180X sensorLeft   = Adafruit_VL6180X();
Adafruit_VL6180X sensorMiddle = Adafruit_VL6180X();
Adafruit_VL6180X sensorRight  = Adafruit_VL6180X();

void initSensors(){
  // set XSHUT pins as OUTPUT
  pinMode(XSHUT_LEFT,   OUTPUT);
  pinMode(xSHUT_MIDDLE, OUTPUT);
  pinMode(XSHUT_RIGHT,  OUTPUT);
  
  // set XSHUT pins to LOW to reset all sensors
  digitalWrite(XSHUT_LEFT,   LOW);
  digitalWrite(xSHUT_MIDDLE, LOW);
  digitalWrite(XSHUT_RIGHT,  LOW);
  delay(10);

  // set XSHUT pins to HIGH to wake up all sensors
  digitalWrite(XSHUT_LEFT,   HIGH);
  digitalWrite(xSHUT_MIDDLE, HIGH);
  digitalWrite(XSHUT_RIGHT,  HIGH);
  delay(10);
  
  // aktivate left sensor and reset other sensors
  digitalWrite(XSHUT_LEFT,   HIGH);
  digitalWrite(xSHUT_MIDDLE, LOW);
  digitalWrite(XSHUT_RIGHT,  LOW);
  
  // set new I2C address for the left sensor
  if(!sensorLeft.begin())
    fatalError("ERROR: sensorLeft not found!");
  sensorLeft.setAddress(ADDR_LEFT);
  delay(10);

  // activate middle sensor and set new I2C address for it
  digitalWrite(xSHUT_MIDDLE, HIGH);
  delay(10);
  if(!sensorMiddle.begin())
    fatalError("ERROR: sensorMiddle not found!");
  sensorMiddle.setAddress(ADDR_MIDDLE);
  delay(10);
  
  // activate right sensor and set new I2C address for it
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if(!sensorRight.begin())
    fatalError("ERROR: sensorRight not found!");
  sensorRight.setAddress(ADDR_RIGHT);
  delay(10);
  
  Serial.println("All sensors successfully initialized!");
}

void setupTest(){
  Serial.begin(115200);
  while(!Serial){
    delay(10);
  }
  
  // set SDA and SCL pins for I2C communication 
  Wire.begin(21, 22);
  
  // initialize the sensors
  initSensors();
}

uint8_t toggle = 0;

void loopTest(){
  uint8_t distance = 0;
  uint8_t status = 0;

  uint16_t sum = 0;
  uint8_t valid_readings = 0;
  uint8_t min_val = 255;
  uint8_t max_val = 0;

  if(toggle == 0){
    for(uint8_t i = 0; i < NR_READINGS; i++){
      distance = sensorLeft.readRange();
      status = sensorLeft.readRangeStatus();
      if(status == VL6180X_ERROR_NONE){
        sum += distance;
        valid_readings++;
        if(distance < min_val)
          min_val = distance;
        if(distance > max_val)
          max_val = distance;
      }
      delay(10);
    }
    
  } else if(toggle == 1){
    for(uint8_t i = 0; i < NR_READINGS; i++){
      distance = sensorMiddle.readRange();
      status = sensorMiddle.readRangeStatus();
      if(status == VL6180X_ERROR_NONE){
        sum += distance;
        valid_readings++;
        if(distance < min_val)
          min_val = distance;
        if(distance > max_val)
          max_val = distance;
      }
      delay(10);
    }
  } else{
    for(uint8_t i = 0; i < NR_READINGS; i++){
      distance = sensorRight.readRange();
      status = sensorRight.readRangeStatus();
      if(status == VL6180X_ERROR_NONE){
        sum += distance;
        valid_readings++;
        if(distance < min_val)
          min_val = distance;
        if(distance > max_val)
          max_val = distance;
      }
      delay(10);
    }
  }
  
  if(toggle == 0){
    Serial.print("LEFT:   Address: ");
    Serial.print(sensorLeft.getAddress(), HEX);
  } else if(toggle == 1){
    Serial.print("MIDDLE: Address: ");
    Serial.print(sensorMiddle.getAddress(), HEX);
  } else{
    Serial.print("RIGHT:  Address: ");
    Serial.print(sensorRight.getAddress(), HEX);
  }
  if(valid_readings > 0){
    distance = sum / valid_readings; // calculate average distance
    Serial.print("  Range: ");
    Serial.print(distance);
    Serial.print(" mm   ");
    Serial.print("  [min: ");
    Serial.print(min_val);
    Serial.print(" | max: ");
    Serial.print(max_val);
    Serial.print("]");
    
    Serial.println("");
  } else{
    switch(status){
      //case VL6180X_ERROR_NONE:
      case VL6180X_ERROR_ECEFAIL:     Serial.println(" ECE failure");             break;
      case VL6180X_ERROR_NOCONVERGE:  Serial.println(" No convergence");          break;
      case VL6180X_ERROR_RANGEIGNORE: Serial.println(" Ignoring range");          break;
      case VL6180X_ERROR_SNR:         Serial.println(" Signal/Noise error");      break;
      case VL6180X_ERROR_RAWUFLOW:    Serial.println(" Raw reading underflow");   break;
      case VL6180X_ERROR_RAWOFLOW:    Serial.println(" Raw reading overflow");    break;
      case VL6180X_ERROR_RANGEUFLOW:  Serial.println(" Range reading underflow"); break;
      case VL6180X_ERROR_RANGEOFLOW:  Serial.println(" Range reading overflow");  break;
      default:  if(status >= VL6180X_ERROR_SYSERR_1 && status <= VL6180X_ERROR_SYSERR_5)
                  Serial.println(" System error");
                else
                  Serial.println(" Unknown error");
                break;
    }
  }
  
  toggle++;
  if(toggle > 2)
    toggle = 0; // reset toggle to 0
  //if(toggle == 0)
  //  delay(1000);
}

void runTest(){
  // run the test
  setupTest();
  while(1){
    loopTest();
  }
}
*/