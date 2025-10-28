#ifndef __SENSOR_MANAGER_H__
#define __SENSOR_MANAGER_H__

#define USE_GYROSCOPE true ///< Set to true to use the gyroscope (MPU6050) for rotation angle measurement, false to disable it

#if USE_GYROSCOPE
#include <Adafruit_MPU6050.h>
#endif // USE_GYROSCOPE


#include <Adafruit_VL6180X.h>

#include <stdint.h>
#include <Adafruit_Sensor.h>

#include "simulated_maze.h"

enum class SensorWallState{
  NO_WALL = 0,
  FOUND_WALL = 1,
  CONFLICT = -1
};

enum class NfcTagType{
  NO_TAG = 0,
  CHEESE = 1,
  TRAP = 2,
  UNKNOWN = -1
};

class SensorManager{
public:
  // === Methods ===
  SensorManager(const RatPosition& rat_pos);
  
  // --- Distance sensors (VL6180) ---
  uint8_t getDistanceLeft();
  uint8_t getDistanceRight();
  uint8_t getDistanceFrontLeft(); 
  uint8_t getDistanceFrontRight();
  
  SensorWallState isWallLeft();
  SensorWallState isWallRight();
  SensorWallState isWallFront();
  
  // --- Wheel encoders (Pololu S13V10F5) ---
  int32_t getEncoderLeft() const;
  int32_t getEncoderRight() const;
  
  void resetEncodersToZero();
  void resetEncodersToDiff();

  
  // --- Gyroscope (MPU6050) ---
  #if USE_GYROSCOPE
  bool calibrateGyroZ();
  void resetGyroZ();
  void updateGyroZAngle();
  float getGyroZRotationAngle() const;
  #endif // USE_GYROSCOPE
  
  // --- NFC reader (RFID-RC522) ---
  NfcTagType getNfcTagType();
  
  // --- Simulation ---
  void printSimulatedMaze() const;
  void printSimulatedMazeColored() const;
private:
  // === Constants ===
  // --- Distance sensors ---
  // I2C Pins
  static constexpr uint8_t SDA_PIN = 21;  ///< SDA Pin for I2C communication
  static constexpr uint8_t SCL_PIN = 22;  ///< SCL Pin for I2C communication
  
  // XSHUT Pins
  static constexpr uint8_t XSHUT_LEFT        = 26;  ///< XSHUT pin for the left distance sensor
  static constexpr uint8_t XSHUT_FRONT_LEFT  = 33;  ///< XSHUT pin for the front-left distance sensor
  static constexpr uint8_t XSHUT_FRONT_RIGHT = 25;  ///< XSHUT pin for the front-right distance sensor
  static constexpr uint8_t XSHUT_RIGHT       = 32;  ///< XSHUT pin for the right distance sensor
  
  // I2C Adressen
  static constexpr uint8_t ADDR_LEFT         = 0x30;  ///< I2C address for the left distance sensor
  static constexpr uint8_t ADDR_FRONT_LEFT   = 0x31;  ///< I2C address for the front-left distance sensor
  static constexpr uint8_t ADDR_FRONT_RIGHT  = 0x32;  ///< I2C address for the front-right distance sensor
  static constexpr uint8_t ADDR_RIGHT        = 0x33;  ///< I2C address for the right distance sensor
  
  static constexpr uint32_t NR_OF_DISTANCE_MEASUREMENTS = 3;  ///< Number of measurements to average for distance sensors
  static constexpr uint32_t WALL_THRESHOLD = 200;              ///< Threshold for wall detection in mm
  
  // === Attributes ===
  // --- Distance sensors (VL6180) ---
  Adafruit_VL6180X sensor_left_;         ///< Left distance sensor
  Adafruit_VL6180X sensor_front_left_;   ///< Front-left distance sensor
  Adafruit_VL6180X sensor_front_right_;  ///< Front-right distance sensor
  Adafruit_VL6180X sensor_right_;        ///< Right distance sensor

  // --- Gyroscope (MPU6050) ---
  #if USE_GYROSCOPE
  Adafruit_MPU6050 mpu_;
  float gyro_z_bias_;  ///< Gyroscope Z-axis bias in degrees per second
  unsigned long last_gyro_reading_time_;  ///< Last time the gyroscope was read in microseconds
  float accumulated_z_rotation_;      ///< Accumulated Z-axis rotation angle in degrees
  #endif // USE_GYROSCOPE
  
  // --- Simulated sensors ---
  SimulatedMaze simulated_maze_; ///< Simulated maze object
};

#endif // __SENSOR_MANAGER_H__
