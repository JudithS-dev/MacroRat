#include "sensor_manager.h"
#include <Arduino.h>

#include "operation_mode.h"
#include "error_handler.h"
#include "communication_interface.h"

// --- Macros for reading GPIO pins ---
/**
 * @brief Read the state of a GPIO pin (0-31).
 * @param pin GPIO pin number (0-31).
 * @return 0 if the pin is low, 1 if the pin is high.
 * 
 * Useful for reading single GPIO pins (0-31) on the ESP32 (e.g., for motor encoders)
 */
#define READ_GPIO_LOW(pin)  ((GPIO.in >> pin) & 0x1)

/**
 * @brief Read the state of a GPIO pin (32-39).
 * @param pin GPIO pin number (32-39).
 * @return 0 if the pin is low, 1 if the pin is high.
 * 
 * Useful for reading GPIO pins 32-39 on the ESP32 (e.g., for motor encoders)
 */
#define READ_GPIO_HIGH(pin)  ((GPIO.in1.data >> (pin - 32)) & 0x1)



// --- Pin assignments for motor encoders ---
static constexpr uint8_t ENCODER_MOTOR_LEFT_A_PIN  =  2;  ///< GPIO pin for the left motor encoder channel A
static constexpr uint8_t ENCODER_MOTOR_LEFT_B_PIN  = 18;  ///< GPIO pin for the left motor encoder channel B
static constexpr uint8_t ENCODER_MOTOR_RIGHT_A_PIN = 34;  ///< GPIO pin for the right motor encoder channel A
static constexpr uint8_t ENCODER_MOTOR_RIGHT_B_PIN = 35;  ///< GPIO pin for the right motor encoder channel B



// --- Encoder ticks ---
/**
 * @brief Stores the number of ticks counted by the left and right motor encoders.
 * 
 * The pololu motor encoders have 12 counts per revolution (CPR), meaning 12 edges (A and B) per revolution.
 * We count both edges of channel A and both edges of channel B, resulting in 12 counts per revolution.
 * Our motor is equipped with a 100:1 gearbox, so we multiply the counts by 100.
 * Thus, we have: 12 ticks per revolution * 100 = 1200 ticks per revolution.
 * 
 * int32_t can store values from -2,147,483,648 to 2,147,483,647.
 * Meaning it can support up to 1,789,569 full rotations in either direction
 * before it overflows or underflows.
 * Our wheels have a diameter greater than 35 mm, so the circumference is greater than 110 mm.
 * With 1,789,569 full rotations, we can cover a distance of more than 197,000 km.
 * This is more than enough for our use case.
 */
volatile int32_t encoder_ticks_left = 0;  ///< Left motor encoder ticks
volatile int32_t encoder_ticks_right = 0; ///< Right motor encoder ticks



// --- Interrupt-Service-Routines (ISR) for motor encoders ---
/**
 * @brief Interrupt handler for left motor encoder channel A.
 * This function is called when the left motor encoder channel A changes state.
 * It updates the left encoder tick count based on its state and the state of channel B.
 */
void encoderLeftA_ISR(){
  if(READ_GPIO_LOW(ENCODER_MOTOR_LEFT_B_PIN) != READ_GPIO_LOW(ENCODER_MOTOR_LEFT_A_PIN))
    encoder_ticks_left++;
  else
    encoder_ticks_left--;
}

/**
 * @brief Interrupt handler for left motor encoder channel B.
 * This function is called when the left motor encoder channel B changes state.
 * It updates the left encoder tick count based on its state and the state of channel A. 
 */
void encoderLeftB_ISR(){
  if(READ_GPIO_LOW(ENCODER_MOTOR_LEFT_A_PIN) == READ_GPIO_LOW(ENCODER_MOTOR_LEFT_B_PIN))
    encoder_ticks_left++;
  else
    encoder_ticks_left--;
}

/**
 * @brief Interrupt handler for right motor encoder channel A.
 * This function is called when the right motor encoder channel A changes state.
 * It updates the right encoder tick count based on its state and the state of channel B.
 */
void encoderRightA_ISR(){
  if(READ_GPIO_HIGH(ENCODER_MOTOR_RIGHT_B_PIN) != READ_GPIO_HIGH(ENCODER_MOTOR_RIGHT_A_PIN))
    encoder_ticks_right++;
  else
    encoder_ticks_right--;
}

/**
 * @brief Interrupt handler for right motor encoder channel B.
 * This function is called when the right motor encoder channel B changes state.
 * It updates the right encoder tick count based on its state and the state of channel A.
 */
void encoderRightB_ISR(){
  if(READ_GPIO_HIGH(ENCODER_MOTOR_RIGHT_A_PIN) == READ_GPIO_HIGH(ENCODER_MOTOR_RIGHT_B_PIN))
    encoder_ticks_right++;
  else
    encoder_ticks_right--;
}



// --- SensorManager constructor ---
/**
 * @brief Initialize all connected sensors (distance sensors, encoders, gyroscope, NFC reader).
 */
SensorManager::SensorManager(const RatPosition& rat_pos)
: sensor_left_(Adafruit_VL6180X()),
  sensor_front_left_(Adafruit_VL6180X()),
  sensor_front_right_(Adafruit_VL6180X()),
  sensor_right_(Adafruit_VL6180X()),
  simulated_maze_(rat_pos, SimMazeConfig::AVOID_TRAP_1) //AVOID_TRAP_1
#if USE_GYROSCOPE
  ,gyro_z_bias_(0.0f),
  last_gyro_reading_time_(0),
  accumulated_z_rotation_(0.0f)
#endif // USE_GYROSCOPE
{
  if(OPERATION_MODE == OperationMode::SEMI_SIMULATED || OPERATION_MODE == OperationMode::REAL_OPERATION){
    // --- Initialize motor encoders ---
    
    CommunicationInterface::log("         SensorManager: Initializing motor encoders...");
    pinMode(ENCODER_MOTOR_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_MOTOR_LEFT_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_MOTOR_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_MOTOR_RIGHT_B_PIN, INPUT_PULLUP);
    
    // Attach interrupts for motor encoders
    attachInterrupt(digitalPinToInterrupt(ENCODER_MOTOR_LEFT_A_PIN), encoderLeftA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_MOTOR_LEFT_B_PIN), encoderLeftB_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_MOTOR_RIGHT_A_PIN), encoderRightA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_MOTOR_RIGHT_B_PIN), encoderRightB_ISR, CHANGE);
    
    CommunicationInterface::log("         SensorManager: Motor encoders initialized.");
    
    
    
    // --- Initialize distance sensors (VL6180) ---
    CommunicationInterface::log("         SensorManager: Initializing distance sensors...");
    
    // Set up I2C communication
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with specified SDA and SCL pins
    
    // Set XSHUT pins as outputs
    pinMode(XSHUT_LEFT,        OUTPUT);
    pinMode(XSHUT_FRONT_LEFT,  OUTPUT);
    pinMode(XSHUT_FRONT_RIGHT, OUTPUT);
    pinMode(XSHUT_RIGHT,       OUTPUT);
    
    // Set XSHUT pins to LOW to power off the sensors
    digitalWrite(XSHUT_LEFT,        LOW);
    digitalWrite(XSHUT_FRONT_LEFT,  LOW);
    digitalWrite(XSHUT_FRONT_RIGHT, LOW);
    digitalWrite(XSHUT_RIGHT,       LOW);
    delay(10);
    
    // Set XSHUT pins to HIGH to wake up all sensors
    digitalWrite(XSHUT_LEFT,        HIGH);
    digitalWrite(XSHUT_FRONT_LEFT,  HIGH);
    digitalWrite(XSHUT_FRONT_RIGHT, HIGH);
    digitalWrite(XSHUT_RIGHT,       HIGH);
    delay(10);
    
    // Reset all sensors except the left sensor
    digitalWrite(XSHUT_FRONT_LEFT,  LOW);
    digitalWrite(XSHUT_FRONT_RIGHT, LOW);
    digitalWrite(XSHUT_RIGHT,       LOW);
    delay(10);
    
    // Activate left sensor and set new I2C address for it
    digitalWrite(XSHUT_LEFT, HIGH);
    delay(10);
    if(!sensor_left_.begin()){
      fatalError("ERROR 'SensorManager::SensorManager': Left distance sensor not found! Check wiring.");
    }
    sensor_left_.setAddress(ADDR_LEFT);
    delay(10);
    
    // Activate front-left sensor and set new I2C address for it
    digitalWrite(XSHUT_FRONT_LEFT, HIGH);
    delay(10);
    if(!sensor_front_left_.begin()){
      fatalError("ERROR 'SensorManager::SensorManager': Front-left distance sensor not found! Check wiring.");
    }
    sensor_front_left_.setAddress(ADDR_FRONT_LEFT);
    delay(10);
    
    // Activate front-right sensor and set new I2C address for it
    digitalWrite(XSHUT_FRONT_RIGHT, HIGH);
    delay(10);
    if(!sensor_front_right_.begin()){
      fatalError("ERROR 'SensorManager::SensorManager': Front-right distance sensor not found! Check wiring.");
    }
    sensor_front_right_.setAddress(ADDR_FRONT_RIGHT);
    delay(10);
    
    // Activate right sensor and set new I2C address for it
    digitalWrite(XSHUT_RIGHT, HIGH);
    delay(10);
    if(!sensor_right_.begin()){
      fatalError("ERROR 'SensorManager::SensorManager': Right distance sensor not found! Check wiring.");
    }
    sensor_right_.setAddress(ADDR_RIGHT);
    delay(10);
    
    CommunicationInterface::log("         SensorManager: Distance sensors initialized.");
    
    
    
    // --- Initialize the gyroscope (MPU6050) ---
    #if USE_GYROSCOPE
    CommunicationInterface::log("         SensorManager: Initializing MPU6050 gyroscope...");
    if(!mpu_.begin()){
      fatalError("ERROR 'SensorManager::SensorManager': MPU6050 not found! Check wiring.");
    }
    
    // Set gyro range: +-250 deg/s is lowest setting
    // This is the most sensitive setting and will give the best results for slow rotations
    // A 360° turn must be slower than ~1.5 (1.44) seconds, should be fine
    mpu_.setGyroRange(MPU6050_RANGE_250_DEG); // Set gyro range to +-250 deg/s (most sensitive)
    
    // Apply 21 Hz low-pass filter to smooth out noise
    // This is a good compromise between responsiveness and noise reduction
    // Other options are 5 Hz, 10 Hz, 21 Hz, 44 Hz, 94 Hz, 184 Hz, 260 Hz
    mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ); // Set low-pass filter to 21 Hz
    
    delay(1000); // Wait 1 ms to allow sensor to stabilize
    
    // Try to calibrate the gyroscope Z-axis
    CommunicationInterface::log("         SensorManager: Calibrating gyroscope Z-axis...");
    const int32_t calibration_attempts = 5;
    for(int32_t i = 0; i < calibration_attempts + 1; i++){
      if(i == calibration_attempts){
        fatalError("ERROR 'SensorManager::SensorManager': Failed to calibrate gyroscope Z-axis after %d attempts. Please ensure the sensor is stationary.", calibration_attempts);
      }
      if(calibrateGyroZ()){
        CommunicationInterface::log("         SensorManager: Gyroscope Z-axis calibrated successfully. Bias = %.6f deg/s", gyro_z_bias_);
        break;  // Calibration successful, exit loop
      } else{
        CommunicationInterface::log("         SensorManager: Gyroscope Z-axis calibration failed (attempt %d/%d). Retrying...", i + 1, calibration_attempts);
        delay(1000); // Wait before retrying
      }
    }
    CommunicationInterface::log("         SensorManager: Gyroscope initialized.");
    #endif // USE_GYROSCOPE
    
    CommunicationInterface::log("         SensorManager: Initialization complete!");
  }
}



// --- Distance sensors (VL6180) ---
/**
 * @brief Get distance measurement from left sensor in millimeters.
 * @return The measured distance (0-254 mm, 255 = no object detected).
 * 
 * This function performs multiple distance measurements and returns the average.
 * It uses the VL6180X sensor to measure the distance and handles errors by retrying
 * up to NR_OF_DISTANCE_MEASUREMENTS times.
 * If no valid measurement is obtained after all attempts, it logs a fatal error.
 */
uint8_t SensorManager::getDistanceLeft(){
  uint16_t sum = 0; // Sum of valid distance measurements
  uint8_t valid = 0; // Count of valid measurements
  uint8_t distance, status;
  
  for(uint32_t i = 0; i < NR_OF_DISTANCE_MEASUREMENTS; i++){
    distance = sensor_left_.readRange();
    status = sensor_left_.readRangeStatus();
    if(status == VL6180X_ERROR_NONE){ // Check if the measurement was successful
      sum += distance; // Add valid measurement to sum
      valid++; // Increment count of valid measurements
    }
    delay(10); // Wait before next measurement see datasheet of VL6180X
  }
  
  if(valid != 0){
    distance = sum / valid; // Calculate average distance
    if(distance > 254){ // Check if distance is within valid range
      return 255; // Return 255 if no object detected
    } else{
      return distance; // Return the average distance
    }
  } else{
    return 255; // If no valid measurements, return 255 (no object detected)
  }
}

/**
 * @brief Get distance measurement from right sensor in millimeters.
 * @return The measured distance (0-254 mm, 255 = no object detected).
 * 
 * This function performs multiple distance measurements and returns the average.
 * It uses the VL6180X sensor to measure the distance and handles errors by retrying
 * up to NR_OF_DISTANCE_MEASUREMENTS times.
 * If no valid measurement is obtained after all attempts, it logs a fatal error.
 */
uint8_t SensorManager::getDistanceRight(){
  uint16_t sum = 0; // Sum of valid distance measurements
  uint8_t valid = 0; // Count of valid measurements
  uint8_t distance, status;
  
  for(uint32_t i = 0; i < NR_OF_DISTANCE_MEASUREMENTS; i++){
    distance = sensor_right_.readRange();
    status = sensor_right_.readRangeStatus();
    if(status == VL6180X_ERROR_NONE){ // Check if the measurement was successful
      sum += distance; // Add valid measurement to sum
      valid++; // Increment count of valid measurements
    }
    delay(10); // Wait before next measurement see datasheet of VL6180X
  }
  
  if(valid != 0){
    distance = sum / valid; // Calculate average distance
    if(distance > 254){ // Check if distance is within valid range
      return 255; // Return 255 if no object detected
    } else{
      return distance; // Return the average distance
    }
  } else{
    return 255; // If no valid measurements, return 255 (no object detected)
  }
}

/**
 * @brief Get distance measurement from front-left sensor in millimeters.
 * @param distance Reference to store the measured distance (0-254 mm, 255 = no object detected).
 * 
 * This function performs multiple distance measurements and returns the average.
 * It uses the VL6180X sensor to measure the distance and handles errors by retrying 
 * up to NR_OF_DISTANCE_MEASUREMENTS times.
 * If no valid measurement is obtained after all attempts, it logs a fatal error.
 */
uint8_t SensorManager::getDistanceFrontLeft(){
  uint16_t sum = 0; // Sum of valid distance measurements
  uint8_t valid = 0; // Count of valid measurements
  uint8_t distance, status;
  
  for(uint32_t i = 0; i < NR_OF_DISTANCE_MEASUREMENTS; i++){
    distance = sensor_front_left_.readRange();
    status = sensor_front_left_.readRangeStatus();
    if(status == VL6180X_ERROR_NONE){ // Check if the measurement was successful
      sum += distance; // Add valid measurement to sum
      valid++; // Increment count of valid measurements
    }
    delay(10); // Wait before next measurement see datasheet of VL6180X
  }
  
  if(valid != 0){
    distance = sum / valid; // Calculate average distance
    if(distance > 254){ // Check if distance is within valid range
      return 255; // Return 255 if no object detected
    } else{
      return distance; // Return the average distance
    }
  } else{
    return 255; // If no valid measurements, return 255 (no object detected)
  }
}

/**
 * @brief Get distance measurement from front-right sensor in millimeters.
 * @param distance Reference to store the measured distance (0-254 mm, 255 = no object detected).
 * 
 * This function performs multiple distance measurements and returns the average.
 * It uses the VL6180X sensor to measure the distance and handles errors by retrying
 * up to NR_OF_DISTANCE_MEASUREMENTS times.
 * If no valid measurement is obtained after all attempts, it logs a fatal error.
 */
uint8_t SensorManager::getDistanceFrontRight(){
  uint16_t sum = 0; // Sum of valid distance measurements
  uint8_t valid = 0; // Count of valid measurements
  uint8_t distance, status;
  
  for(uint32_t i = 0; i < NR_OF_DISTANCE_MEASUREMENTS; i++){
    distance = sensor_front_right_.readRange();
    status = sensor_front_right_.readRangeStatus();
    if(status == VL6180X_ERROR_NONE){ // Check if the measurement was successful
      sum += distance; // Add valid measurement to sum
      valid++; // Increment count of valid measurements
    }
    delay(10); // Wait before next measurement see datasheet of VL6180X
  }
  
  if(valid != 0){
    distance = sum / valid; // Calculate average distance
    if(distance > 254){ // Check if distance is within valid range
      return 255; // Return 255 if no object detected
    } else{
      return distance; // Return the average distance
    }
  } else{
    return 255; // If no valid measurements, return 255 (no object detected)
  }
}

/**
 * @brief Check if there is a wall to the left side.
 * @return Detected wall state. 
 * 
 * If the left sensor detects a wall within the WALL_THRESHOLD,
 * it returns SensorWallState::FOUND_WALL.
 * If the distance is greater than WALL_THRESHOLD, it returns SensorWallState::NO_WALL
 */
SensorWallState SensorManager::isWallLeft(){
  if(sensors_simulated()){  // use simulated sensors
    WallState state = simulated_maze_.getWallLeft();
    switch(state){
      case WallState::NO_WALL:    return SensorWallState::NO_WALL;
      case WallState::WALL:       return SensorWallState::FOUND_WALL;
      default:  fatalError("ERROR 'SensorManager::isWallLeft': Unknown wall state %d", (int)state);
    }
    return SensorWallState::CONFLICT; // Unknown state
  } else{ // use real sensors
    uint8_t distance = getDistanceLeft();
    if(distance <= WALL_THRESHOLD && distance != 255){
      return SensorWallState::FOUND_WALL; // Wall detected
    } else if(distance > WALL_THRESHOLD){
      return SensorWallState::NO_WALL; // No wall detected
    } else 
    fatalError("ERROR 'SensorManager::isWallLeft': Invalid distance measurement %d", distance);
    return SensorWallState::CONFLICT; // This will never be reached, but added to avoid compiler warnings
  }
}

/**
 * @brief Check if there is a wall to the right side.
 * @return Detected wall state.
 * 
 * If the right sensor detects a wall within the WALL_THRESHOLD,
 * it returns SensorWallState::FOUND_WALL.
 * If the distance is greater than WALL_THRESHOLD, it returns SensorWallState::NO_WALL
 */
SensorWallState SensorManager::isWallRight(){
  if(sensors_simulated()){  // use simulated sensors
    WallState state = simulated_maze_.getWallRight();
    switch(state){
      case WallState::NO_WALL:    return SensorWallState::NO_WALL;
      case WallState::WALL:       return SensorWallState::FOUND_WALL;
      default:  fatalError("ERROR 'SensorManager::isWallRight': Unknown wall state %d", (int)state);
    }
    return SensorWallState::CONFLICT; // Unknown state
  } else{
    if(getDistanceRight() <= WALL_THRESHOLD){
      return SensorWallState::FOUND_WALL; // Wall detected
    } else if(getDistanceRight() > WALL_THRESHOLD){
      return SensorWallState::NO_WALL; // No wall detected
    } else
    fatalError("ERROR 'SensorManager::isWallRight': Invalid distance measurement %d", getDistanceRight());
    return SensorWallState::CONFLICT; // This will never be reached, but added to avoid compiler warnings
  }
}

/**
 * @brief Check if there is a wall in front.
 * @return Detected wall state. If front-sensors are conflicting, return CONFLICT.
 */
SensorWallState SensorManager::isWallFront(){
  if(sensors_simulated()){  // use simulated sensors
    WallState state = simulated_maze_.getWallFront();
    switch(state){
      case WallState::NO_WALL:    return SensorWallState::NO_WALL;
      case WallState::WALL:       return SensorWallState::FOUND_WALL;
      default:  fatalError("ERROR 'SensorManager::isWallFront': Unknown wall state %d", (int)state);
    }
    return SensorWallState::CONFLICT; // Unknown state
  } else{ // use real sensors
    uint8_t distance_left = getDistanceFrontLeft();
    uint8_t distance_right = getDistanceFrontRight();
    if(distance_left <= WALL_THRESHOLD && distance_right <= WALL_THRESHOLD){
      return SensorWallState::FOUND_WALL; // Wall detected
    } else if(distance_left > WALL_THRESHOLD && distance_right > WALL_THRESHOLD){
      return SensorWallState::NO_WALL; // No wall detected
    } else if(distance_left <= WALL_THRESHOLD && distance_right > WALL_THRESHOLD){
      CommunicationInterface::log("Warning: Front-left sensor detects a wall, but front-right sensor does not.");
      return SensorWallState::CONFLICT; // Conflict detected
    } else if(distance_left > WALL_THRESHOLD && distance_right <= WALL_THRESHOLD){
      CommunicationInterface::log("Warning: Front-right sensor detects a wall, but front-left sensor does not.");
      return SensorWallState::CONFLICT; // Conflict detected
    }
    fatalError("ERROR 'SensorManager::isWallFront': Invalid distance measurements: left = %d, right = %d", distance_left, distance_right);
    return SensorWallState::CONFLICT; // This will never be reached, but added to avoid compiler warnings
  }
}



// --- Wheel encoders (Pololu S13V10F5) ---
/**
 * @brief Get encoder count from the left wheel.
 * @return The number of ticks counted by the left motor encoder.
 */
int32_t SensorManager::getEncoderLeft() const{
  return encoder_ticks_left;
}

/**
 * @brief Get encoder count from the right wheel.
 * @return The number of ticks counted by the right motor encoder.
 */
int32_t SensorManager::getEncoderRight() const{
  return encoder_ticks_right;
}

/**
 * @brief Reset the encoder counts for both wheels.
 * 
 * This function resets the encoder ticks for both the left and right motors to zero.
 * It is useful when you want to start counting from a known state, e.g., after turning.
 */
void SensorManager::resetEncodersToZero(){
  encoder_ticks_left = 0;  // Reset left encoder ticks
  encoder_ticks_right = 0; // Reset right encoder ticks
  CommunicationInterface::log("         SensorManager: Encoders reset.");
}

/**
 * @brief Reset the encoder counts to the difference between left and right encoders.
 * 
 * This function resets the encoder ticks for both the left and right motors to the difference
 * between the two. This is useful to maintain the relative position of the robot after a forward movement.
 */
void SensorManager::resetEncodersToDiff(){
  int32_t min_ticks = min(abs(encoder_ticks_left), abs(encoder_ticks_right));
  encoder_ticks_left = abs(encoder_ticks_left) - min_ticks; // Reset left encoder ticks to the difference
  encoder_ticks_right = abs(encoder_ticks_right) - min_ticks; // Reset right encoder
  CommunicationInterface::log("         SensorManager: Encoders reset to difference: left = %d, right = %d", encoder_ticks_left, encoder_ticks_right);
}



// --- Gyroscope (MPU6050) ---
#if USE_GYROSCOPE
/**
 * @brief Calibrate the gyroscope Z-axis by measuring the drift.
 *        The calibration is done by averaging multiple readings and checking the standard deviation.
 * @return True if calibration was successful, false if the sensor is moving (high standard deviation).
 */
bool SensorManager::calibrateGyroZ(){
  const int32_t samples = 500;
  const float_t max_allowed_stddev = 0.05f;  // °/s used to determine if the sensor is moving
  
  float_t sum = 0.0f;
  float_t sqSum = 0.0f;
  
  sensors_event_t unused_a, gyro, unused_t;
  
  // -- Measure the Z-axis gyro drift --
  for(int32_t i = 0; i < samples; i++){
    mpu_.getEvent(&unused_a, &gyro, &unused_t);
    float_t val = gyro.gyro.z * 180.0f / PI;  // rad/s -> deg/s
    sum += val;
    sqSum += val * val;
    delay(20); // Wait for 20 ms to avoid reading too fast
  }
  
  // -- Calculate mean and standard deviation --
  float_t mean = sum / samples;
  float_t variance = (sqSum / samples) - (mean * mean);
  float_t stddev = sqrt(variance);
  
  CommunicationInterface::log("GyroZ STDDEV: %.6f, Mean: %.6f", stddev, mean);
  
  // -- Check if the standard deviation is within acceptable limits --
  // If the sensor is moving, the standard deviation will be high
  if(stddev > max_allowed_stddev){
    CommunicationInterface::log("Gyroscope Z-axis calibration failed: stddev %.6f > %.6f\n", stddev, max_allowed_stddev);
    return false;  // Calibration failed
  }
  
  // -- If successful, set the bias --
  gyro_z_bias_ = mean;
  return true;
}

/**
 * @brief Reset the accumulated Z-axis rotation angle.
 */
void SensorManager::resetGyroZ(){
  accumulated_z_rotation_ = 0.0f;
  last_gyro_reading_time_ = micros();  // Reset the last reading time to now
}


/**
 * @brief Update the accumulated Z-axis rotation angle based on the gyroscope readings.
 *        This should be called periodically to update the angle.
 */
void SensorManager::updateGyroZAngle(){
  sensors_event_t unused_a, gyro, unused_t;
  mpu_.getEvent(&unused_a, &gyro, &unused_t);
  
  // Time calculation
  unsigned long current_time = micros();
  float_t dt = (current_time - last_gyro_reading_time_) / 1000000.0f;  // Convert microseconds to seconds
  last_gyro_reading_time_ = current_time;  // Update the last reading time
  
  // Calculate the Z-axis rotation in degrees
  float_t gyro_z = gyro.gyro.z * 180.0f / PI;  // Convert rad/s to deg/s
  float_t rotation = (gyro_z - gyro_z_bias_) * dt;  // Calculate the rotation in degrees
  accumulated_z_rotation_ += rotation;
}

/**
 * @brief Get the current accumulated Z-axis rotation angle.
 * @return float The current accumulated Z-axis rotation angle in degrees.
 */
float SensorManager::getGyroZRotationAngle() const{
  return accumulated_z_rotation_;
}
#endif // USE_GYROSCOPE



// --- NFC reader (RFID-RC522) ---
/**
 * @brief Get the detected NFC tag type under the robot.
 * @return The type of the detected NFC tag.
 *         If no tag is detected, return NO_TAG.
 *         If a tag is detected, return its type (CHEESE or TRAP).
 */
NfcTagType SensorManager::getNfcTagType(){
  if(sensors_simulated()){  // use simulated sensors
    CellState cell_state = simulated_maze_.getCellState();
    switch(cell_state){
      case CellState::CHEESE: return NfcTagType::CHEESE;
      case CellState::TRAP:   return NfcTagType::TRAP;
      default:                return NfcTagType::NO_TAG;
    }
  } else{ // use real sensors
    fatalError("ERROR 'SensorManager::getNfcTagType': This function should not be called in real operation mode.");
    return NfcTagType::NO_TAG; // This will never be reached, but added to avoid compiler warnings
  }
}



// --- Simulation ---
void SensorManager::printSimulatedMaze() const{
  simulated_maze_.printMaze();
}
void SensorManager::printSimulatedMazeColored() const{
  simulated_maze_.printMazeColored();
}