/*
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// Accumulated rotation around Z-axis (degrees)
float_t totalAngle = 0.0f;

// Gyro drift correction (Z-axis)
float_t gyroZDrift = 0.0f;

// Timestamp for delta time calculation
uint32_t lastTime = 0;

// Measures gyro drift on Z-axis (sensor must be stationary)
float_t calibrateGyroZ(){
  float_t sum = 0.0f;
  float_t sqSum = 0.0f;
  const int32_t samples = 500;
  sensors_event_t a, g, t;

  for (int32_t i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &t);
    float_t val = g.gyro.z * 180.0f / PI;
    sum += val;
    sqSum += val * val;
    delay(20);
  }

  float_t mean = sum / samples;
  float_t variance = (sqSum / samples) - (mean * mean);
  float_t stddev = sqrt(variance);

  Serial.print("Drift STDDEV: ");
  Serial.println(stddev); // gibt dir ein Gefühl für die Stabilität

  return mean;
}

void setup(){
  Serial.begin(115200);
  Wire.begin(21, 22); // I2C pins: SDA (GPIO21), SCL (GPIO22)
  
  if(!mpu.begin()){
    Serial.println("MPU6050 not found! Check wiring.");
    while(1) delay(10);
  }
  
  Serial.println("MPU6050 connected. Hold still for calibration...");
  
  // Set gyro range: +-250 deg/s is lowest setting
  // This is the most sensitive setting and will give the best results for slow rotations
  // A 360° turn must be slower than ~1.5 (1.44) seconds, should be fine
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  // Apply 21 Hz low-pass filter to smooth out noise
  // This is a good compromise between responsiveness and noise reduction
  // Other options are 5 Hz, 10 Hz, 21 Hz, 44 Hz, 94 Hz, 184 Hz, 260 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(1000); // Wait 1 ms to allow sensor to stabilize
  
  gyroZDrift = calibrateGyroZ();
  Serial.print("Gyro Z Drift (deg/s): ");
  Serial.println(gyroZDrift);
  
  lastTime = millis(); // Set initial timestamp
}

void loop(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Time delta calculation
  uint32_t now = millis();
  float_t dt = (now - lastTime) / 1000.0f;  // convert milliseconds to seconds
  lastTime = now;
  
  // Convert Z-axis gyro data from rad/s => deg/s
  float_t gyroZ_deg = g.gyro.z * 180.0f / PI;
  
  // Subtract previously calibrated drift
  gyroZ_deg -= gyroZDrift;
  
  // Integrate angular velocity over time
  totalAngle += gyroZ_deg * dt;
  
  Serial.print("Z-Axis Drift [°]: ");
  Serial.print(gyroZDrift);
  Serial.print("; Z-Axis Rotation [°]: ");
  Serial.println(totalAngle);
  
  if(fabs(totalAngle) >= 360.0f){
    Serial.println("=> 360 degree turn detected!");
    // stop the robot or perform any action
    // todo: implement stop action
    // then reset the total angle
    totalAngle = 0.0f;
    delay(1000); // Debounce for test without robot
  }
  delay(10); // ~100 Hz sampling
}
*/