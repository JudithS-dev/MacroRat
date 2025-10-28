#include "main_controller.h"
#include "Arduino.h"

// start terminal with: 'pio device monitor --raw --baud 115200'

void setup(){
  Serial.begin(115200);
  delay(1000); // Wait for Serial to initialize
  Serial.println("Starting MacroRat Robot Controller...");

  // Initialize main controller (and other components)
  MainController mainController;
  
  // Start the main controller loop
  // This will run indefinitely, handling the robot's state and navigation
  mainController.Start();
}

void loop(){
}