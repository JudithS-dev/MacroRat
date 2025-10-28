#include "error_handler.h"

#include <Arduino.h>
#include <stdarg.h> 

#include "communication_interface.h"
#include "operation_mode.h"
#include "motor_driver.h"

MotorDriver* global_motor_driver = nullptr;

#define ERROR_LED_PIN LED_BUILTIN

void fatalError(const char* message, ...){
  // Stop any ongoing movement if the system is not in a simulated state
  if(!movement_simulated()){
    if(global_motor_driver != nullptr)
      // Stop the motors using the global motor driver instance
      global_motor_driver->stopMotors();
  }
  
  pinMode(ERROR_LED_PIN, OUTPUT);
  
  // Convert message to a formatted string
  char buffer[128];
  va_list args;
  va_start(args, message);
  vsnprintf(buffer, sizeof(buffer), message, args);
  va_end(args);
  
  // Print initial message (once)
  if(Serial){
    Serial.println();
    Serial.println(F("=== FATAL ERROR ==="));
    Serial.println(buffer);
    Serial.println(F("System halted."));
  }
  // Log the error message to the communication interface, but only once
  CommunicationInterface::log("================================ FATAL ERROR =================================");
  CommunicationInterface::log(buffer);
  CommunicationInterface::log("System halted.");
  
  // Enter infinite loop to indicate failure and prevent further execution
  while(true){
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(500);
    digitalWrite(ERROR_LED_PIN, LOW);
    delay(500);  // if deleted, uncomment the delay at the end of this function
    
    // Repeat message. Helpful for debugging, if USB was not connected
    if(Serial){
      Serial.println(F("[FATAL ERROR]"));
      Serial.println(buffer);
    }
    // delay(1000); // Prevent flooding of the Serial output, important if LED-code is deleted
  }
}
