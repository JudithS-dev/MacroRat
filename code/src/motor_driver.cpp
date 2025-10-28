#include "motor_driver.h"

#include <Arduino.h>

#include "error_handler.h"
#include "communication_interface.h"

HardwareSerial motorSerial(1);  // UART1 for motor control

/**
 * @brief Initializes all connected motors.
 */
MotorDriver::MotorDriver()
: left_motor_speed_(0),
  right_motor_speed_(0),
  left_motor_direction_(MotorDirection::FORWARD),
  right_motor_direction_(MotorDirection::FORWARD)
{
  CommunicationInterface::log("           MotorDriver: Initializing motors...");
  motorSerial.begin(9600, SERIAL_8N1, 17, 16); // motorSerial on UART1, RX=17, TX=16
  stopMotors();
  CommunicationInterface::log("           MotorDriver: Motors initialized.");

  // Set the global motor driver instance for error handling
  CommunicationInterface::log("           MotorDriver: Setting global motor driver instance...");
  if(global_motor_driver != nullptr){
    fatalError("ERROR 'MotorDriver::MotorDriver': Global motor driver instance already set. This should not happen.");
  }
  global_motor_driver = this; // Set the global motor driver instance
  CommunicationInterface::log("           MotorDriver: Global motor driver instance set successfully.");
  CommunicationInterface::log("           MotorDriver: Initialization complete!");
}



// ---Basic Motor Control ---
/**
 * @brief Drives both motors forward at the specified speed.
 * @param speed Speed for both motors (0-3200).
 */
void MotorDriver::driveForward(uint16_t speed){
  sendMotorCommand(LEFT_MOTOR,  MotorDirection::FORWARD, speed);
  sendMotorCommand(RIGHT_MOTOR, MotorDirection::FORWARD, speed);
}

/**
 * @brief Turns the robot in a circle to the left at the specified speed.
 * @param speed Speed for both motors (0-3200).
 */
void MotorDriver::turnInCircleLeft(uint16_t speed){
  sendMotorCommand(LEFT_MOTOR, MotorDirection::BACKWARD, speed);  
  sendMotorCommand(RIGHT_MOTOR, MotorDirection::FORWARD, speed); 
}

/**
 * @brief Turns the robot in a circle to the right at the specified speed.
 * @param speed Speed for both motors (0-3200).
 */
void MotorDriver::turnInCircleRight(uint16_t speed){
  sendMotorCommand(LEFT_MOTOR, MotorDirection::FORWARD, speed);  
  sendMotorCommand(RIGHT_MOTOR, MotorDirection::BACKWARD, speed); 
}

/**
 * @brief Stops both motors immediately.
 * The directions are preserved, but speed is set to 0.
 */
void MotorDriver::stopMotors(){
  sendMotorCommand(LEFT_MOTOR,  left_motor_direction_, 0);
  sendMotorCommand(RIGHT_MOTOR, right_motor_direction_, 0);
}



// --- Advanced Motor Control ---
/**
 * @brief Sets the speed and direction of the left motor.
 * @param speed Speed for the left motor (0-3200).
 * @param direction Direction for the left motor (FORWARD or BACKWARD).
 * This function sends a command to the left motor with the specified speed and direction.
 */
void MotorDriver::setLeftMotor(uint16_t speed, MotorDirection direction){
  sendMotorCommand(LEFT_MOTOR, direction, speed);
}

/**
 * @brief Sets the speed and direction of the right motor.
 * @param speed Speed for the right motor (0-3200).
 * @param direction Direction for the right motor (FORWARD or BACKWARD).
 * This function sends a command to the right motor with the specified speed and direction.
 */
void MotorDriver::setRightMotor(uint16_t speed, MotorDirection direction){
  sendMotorCommand(RIGHT_MOTOR, direction, speed);
}

// --- Getters ---
/**
 * @brief Gets the current speed of the left motor.
 * @return Speed of the left motor (0-3200).
 */
uint16_t MotorDriver::getLeftMotorSpeed() const{
  return left_motor_speed_;
}

/**
 * @brief Gets the current speed of the right motor.
 * @return Speed of the right motor (0-3200).
 */
uint16_t MotorDriver::getRightMotorSpeed() const{
  return right_motor_speed_;
}

/**
 * @brief Gets the current direction of the left motor.
 * @return Direction of the left motor (FORWARD or BACKWARD).
 */
MotorDirection MotorDriver::getLeftMotorDirection() const{
  return left_motor_direction_;
}

/**
 * @brief Gets the current direction of the right motor.
 * @return Direction of the right motor (FORWARD or BACKWARD).
 */
MotorDirection MotorDriver::getRightMotorDirection() const{
  return right_motor_direction_;
}



// --- Internal helpers ---
/**
 * @brief Sends a command to the motor driver.
 * @param device Device ID (LEFT_MOTOR or RIGHT_MOTOR).
 * @param dir Direction to set (FORWARD or BACKWARD).
 * @param speed Speed to set (0-3200).
 * 
 * This function constructs the command byte based on the device and direction,
 * and sends it over the motorSerial interface. If the speed exceeds 3200, it
 * is capped at 3200.
 */
void MotorDriver::sendMotorCommand(byte device, MotorDirection dir, uint16_t speed){
  if(speed > 3200) speed = 3200;
  
  // Get the command byte based on device and direction
  // If direction is STOP, use the last known direction for that motor and set speed to 0
  byte command;
  if(device == LEFT_MOTOR){
    left_motor_speed_ = speed;
    left_motor_direction_ = dir;
    switch(dir){
      case MotorDirection::FORWARD:  command = 0x06; break;
      case MotorDirection::BACKWARD: command = 0x05; break;
      default: fatalError("ERROR 'MotorDriver::sendMotorCommand': Invalid direction for left motor. Use FORWARD or BACKWARD.");
    }
  } else if(device == RIGHT_MOTOR){
    right_motor_speed_ = speed;
    right_motor_direction_ = dir;
    switch(dir){
      case MotorDirection::FORWARD:  command = 0x05; break;
      case MotorDirection::BACKWARD: command = 0x06; break;
      default: fatalError("ERROR 'MotorDriver::sendMotorCommand': Invalid direction for right motor. Use FORWARD or BACKWARD.");
    }
  } else{
    fatalError("ERROR 'MotorDriver::sendMotorCommand': Invalid device ID. Use LEFT_MOTOR or RIGHT_MOTOR.");
  }
  
  // Prepare the speed bytes
  byte low = speed & 0x1F;
  byte high = (speed >> 5) & 0x7F;
  
  // Send the command to the motor
  if(!motorSerial){
    fatalError("ERROR 'MotorDriver::sendMotorCommand': Serial port not initialized or not available.");
  }
  motorSerial.write(0xAA);
  motorSerial.write(device);
  motorSerial.write(command);
  motorSerial.write(low);
  motorSerial.write(high);
}