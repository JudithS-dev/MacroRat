/*#include "error_handler.h"
// --- Gyroscope (MPU6050) ---
#include <stdint.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu_;
float gyro_z_bias_;  ///< Gyroscope Z-axis bias in degrees per second
unsigned long last_gyro_reading_time_;  ///< Last time the gyroscope was read in microseconds
float accumulated_z_rotation_;      ///< Accumulated Z-axis rotation angle in degrees

bool calibrateGyroZ();
void resetGyroZ();
void updateGyroZAngle();
float getGyroZRotationAngle();

#include <Arduino.h>
// d = 36 mm
// => u = 113.097 mm
// => 1 Umdrehung = 113.097 mm


#define COM_IS_USED true

#if COM_IS_USED
#include "communication_interface.h"
#define WIFI_SSID "TI Roboter"
#define WIFI_PASSWORD "ITRobot!"
#define MQTT_BROKER "172.16.18.48" //"172.16.8.108" //"172.16.6.148" for Raspberry Pi

CommunicationInterface *communicationInterface = nullptr;
#endif

#include "motor_driver.h"
MotorDriver motor;

// === PIN-Zuweisung ===
#define ENCODER_PIN_A_LEFT 2    ///< Channel A
#define ENCODER_PIN_B_LEFT 18   ///< Channel B

#define ENCODER_PIN_A_RIGHT 34  ///< Channel A
#define ENCODER_PIN_B_RIGHT 35  ///< Channel B

// === Encoder-Konfiguration ===
//
//* @brief Ticks per revolution for the encoder
//* 
//* The encoder has 12 counts per revolution (CPR), meaning 12 edges (A and B) per revolution.
//* If we only count the edges of channel A, we get half the number of counts, which is 6.
//* If we only count the rising edges of channel A, we get half of that again, which is 3.
//* Our motor is equipped with a 100:1 gearbox, so we multiply the counts by 100.
//* Thus, we have: 3 ticks per revolution * 100 = 300 ticks per revolution.
//
#define TICKS_PER_REV 300

volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

void IRAM_ATTR encoderLeftISR() {
  bool b = digitalRead(ENCODER_PIN_B_LEFT);
  encoder_ticks_left += (b ? +1 : -1);
}

void IRAM_ATTR encoderRightISR() {
  bool b = digitalRead(ENCODER_PIN_B_RIGHT);
  encoder_ticks_right += (b ? +1 : -1);
}




void setup(){
  Serial.begin(115200);
  delay(500);
  
  #if COM_IS_USED
  communicationInterface = new CommunicationInterface(WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, 1883);
  if (communicationInterface == nullptr) {
    Serial.println("Fehler: Kommunikation konnte nicht initialisiert werden.");
    while (true);  // Dauerhalt
  }
  #endif
  
  pinMode(ENCODER_PIN_A_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_PIN_A_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B_RIGHT, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A_LEFT), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A_RIGHT), encoderRightISR, RISING);

  // --- Initialize the gyroscope (MPU6050) ---
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
    mpu_.setFilterBandwidth(MPU6050_BAND_44_HZ); // Set low-pass filter to 21 Hz
    
    delay(1000); // Wait 1 ms to allow sensor to stabilize
    
    // Try to calibrate the gyroscope Z-axis
    Serial.println("SensorManager: Calibrating gyroscope Z-axis...");
    const int32_t calibration_attempts = 5;
    for(int32_t i = 0; i < calibration_attempts + 1; i++){
      if(i == calibration_attempts){
        fatalError("ERROR 'SensorManager::SensorManager': Failed to calibrate gyroscope Z-axis after %d attempts. Please ensure the sensor is stationary.", calibration_attempts);
      }
      if(calibrateGyroZ()){
        Serial.printf("SensorManager: Gyroscope Z-axis calibrated successfully. Bias = %.6f deg/s\n", gyro_z_bias_);
        break;  // Calibration successful, exit loop
      } else{
        Serial.printf("SensorManager: Gyroscope Z-axis calibration failed (attempt %d/%d). Retrying...\n", i + 1, calibration_attempts);
        delay(1000); // Wait before retrying
      }
    }
  
  
  Serial.println("Encoder-Test mit Umdrehungsausgabe gestartet.");
  #if COM_IS_USED
  communicationInterface->log("Encoder-Test mit Umdrehungsausgabe gestartet.");
  #endif
}



#define TARGET_DISTANCE_MM 1000
#define WHEEL_CIRCUMFERENCE_MM 113.097 ///< d = 36 mm 
#define TARGET_TICKS (long)((TARGET_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV) // = 2652.58 Ticks

#define MIN_SPEED 400
#define MAX_SPEED 1000

#define KP 0.5  // Proportionalfaktor für Geschwindigkeit (Tuning möglich)
#define KP_TICKS 2.0
#define KD_TICKS 0.5

float initial_heading = 0.0f;

void loop(){
  static unsigned long last_output = 0;
  
  long ticks_left = abs(encoder_ticks_left);
  long ticks_right = abs(encoder_ticks_right);
  long avg_ticks = (ticks_left + ticks_right) / 2;
  
  #if COM_IS_USED
    if(millis() - last_output > 0){
      communicationInterface->log("Ticks L: %ld | Ticks R: %ld | Avg Ticks: %ld", ticks_left, ticks_right, avg_ticks);
    }
  #endif
  
  long remaining_ticks = TARGET_TICKS - avg_ticks;
  
  if(remaining_ticks <= 0){
    motor.stopMotors();
    
    Serial.println("1 Meter erreicht!");
    #if COM_IS_USED
    communicationInterface->log("1 Meter erreicht!");
    #endif
    while(1);  // Dauerhalt
  }
  
  // --- Geschwindigkeit durch P-Regler ---
  int speed = (int)(remaining_ticks * KP + 500);
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  
  // --- Gyroskop zur Richtungsregelung verwenden ---
  updateGyroZAngle();  // Z-Achse aktualisieren
  float current_heading = getGyroZRotationAngle();  // aktuelle Abweichung (°)
  float heading_error = initial_heading - current_heading;

  // Begrenze Fehler auf ±180°
  if (heading_error > 180.0f) heading_error -= 360.0f;
  if (heading_error < -180.0f) heading_error += 360.0f;

  
  // => nur mit correction ohne bias, eiert der roboter zu stark
  //=> Gyro-basierte Korrektur macht keinen Sinn, da das Gyro zu schlecht misst, bei einem Durchlauf von 1 Meter und
  // einer Abweichung von 5mm sagte das gyro -10
  
  //Ticks L: 2648 | Ticks R: 2652 | Avg Ticks: 2650
  //Ticks L: 2648 | Ticks R: 2652 | Avg Ticks: 2650 | Gyro: -10.17° | Speed L: 390 | Speed R: 611 | Correction: -40
  //Ticks L: 2664 | Ticks R: 2670 | Avg Ticks: 2667
  //1 Meter erreicht!

  

  int tick_diff = ticks_left - ticks_right;
  int correction = tick_diff * 10; // Tuningfaktor 10 
  
  int left_speed = (speed*0.7) - correction; 
  int right_speed = (speed*1.3) + correction;



  
  #if COM_IS_USED
    if(millis() - last_output > 0){
      last_output = millis();
      //communicationInterface->log ("%2f, %d, %d, %d", current_heading, left_speed, right_speed, correction);
      //communicationInterface->log("Gyro: %.2f° | Speed L: %d | Speed R: %d | Correction: %d", current_heading, left_speed, right_speed, correction);
      communicationInterface->log("Ticks L: %ld | Ticks R: %ld | Avg Ticks: %ld | Gyro: %.2f° | Speed L: %d | Speed R: %d | Correction: %d", 
                                  ticks_left, ticks_right, avg_ticks, current_heading, left_speed, right_speed, correction);
    }
  #endif
  
  
  // Begrenzen
  left_speed = constrain(left_speed, MIN_SPEED, MAX_SPEED);
  right_speed = constrain(right_speed, MIN_SPEED, MAX_SPEED);
  
  // Ansteuern
  motor.setLeftMotor((uint16_t)left_speed, MotorDirection::FORWARD);
  motor.setRightMotor((uint16_t)right_speed, MotorDirection::FORWARD);
  
  delay(500);  // Entlastung
}



















// --- Gyroscope (MPU6050) ---

bool calibrateGyroZ(){
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
  
  Serial.print("GyroZ STDDEV: ");
  Serial.println(stddev, 6);
  Serial.print("GyroZ Mean: ");
  Serial.println(mean, 6);
  
  // -- Check if the standard deviation is within acceptable limits --
  // If the sensor is moving, the standard deviation will be high
  if(stddev > max_allowed_stddev){
    Serial.printf("Gyroscope Z-axis calibration failed: stddev %.6f > %.6f\n", stddev, max_allowed_stddev);
    return false;  // Calibration failed
  }
  
  // -- If successful, set the bias --
  gyro_z_bias_ = mean;
  return true;
}


void resetGyroZ(){
  accumulated_z_rotation_ = 0.0f;
  last_gyro_reading_time_ = micros();  // Reset the last reading time to now
}



void updateGyroZAngle(){
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


float getGyroZRotationAngle(){
  return accumulated_z_rotation_;
} */
