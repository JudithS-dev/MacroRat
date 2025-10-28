#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>

/**
 * @enum MotorDirection
 * @brief Defines the possible directions for the motors.
 */
enum class MotorDirection{
  FORWARD = 0,
  BACKWARD
};

/**
 * @class MotorDriver
 * @brief Low-level driver to control left and right motors using PWM and direction pins.
 */
class MotorDriver {
public:
  // === Methods ===
  MotorDriver();
  
  // --- Basic Motor Control ---
  void driveForward(uint16_t speed);
  void turnInCircleLeft(uint16_t speed);
  void turnInCircleRight(uint16_t speed);
  
  void stopMotors();
  
  // --- Advanced Motor Control ---
  void setLeftMotor(uint16_t speed, MotorDirection direction);
  void setRightMotor(uint16_t speed, MotorDirection direction);
  
  // --- Getters ---
  uint16_t getLeftMotorSpeed() const;
  uint16_t getRightMotorSpeed() const;
  
  MotorDirection getLeftMotorDirection() const;
  MotorDirection getRightMotorDirection() const;
  
private:
  // === Constants ===
  static constexpr byte LEFT_MOTOR = 0x01;   ///< Device ID for the left motor
  static constexpr byte RIGHT_MOTOR = 0x02;  ///< Device ID for the right motor
  
  // === Attributes ===
  uint16_t left_motor_speed_;   ///< Current speed of the left motor
  uint16_t right_motor_speed_;  ///< Current speed of the right motor
  MotorDirection left_motor_direction_;   ///< Current direction of the left motor
  MotorDirection right_motor_direction_;  ///< Current direction of the right motor
  
  // === Methods ===
  // --- Internal helpers ---
  void sendMotorCommand(byte device, MotorDirection dir, uint16_t speed);
};

#endif // __MOTOR_DRIVER_H__
