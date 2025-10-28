#include <Arduino.h>
#include "error_handler.h"

#define COM_IS_USED true

#if COM_IS_USED
  #include "communication_interface.h"
  #define WIFI_SSID "TI Roboter"
  #define WIFI_PASSWORD "ITRobot!"
  #define MQTT_BROKER "172.16.18.48" //"172.16.8.108" //"172.16.6.148" for Raspberry Pi
  
  CommunicationInterface *communicationInterface = nullptr;
  
  #define MAX_LOG_ENTRIES 300
  
  struct EncoderLogEntry {
    long ticks_left;
    long ticks_right;
    long avg_ticks;
    int speed_left;
    int speed_right;
    int tick_diff;
    int correction;
  };
  
  EncoderLogEntry logBuffer[MAX_LOG_ENTRIES];
  int logIndex = 0;
#endif

#include "motor_driver.h"
MotorDriver motor;

// === PIN-Zuweisung ===
#define ENCODER_PIN_A_LEFT 2    ///< Channel A
#define ENCODER_PIN_B_LEFT 18   ///< Channel B

#define ENCODER_PIN_A_RIGHT 34  ///< Channel A
#define ENCODER_PIN_B_RIGHT 35  ///< Channel B


#define TICKS_PER_REV 1200

// Pins 0–31
#define READ_GPIO_LOW(pin)  ((GPIO.in >> pin) & 0x1)
// Pins 32–39
#define READ_GPIO_HIGH(pin)  ((GPIO.in1.data >> (pin - 32)) & 0x1)


volatile long encoder_ticks_left = 0;
volatile long encoder_ticks_right = 0;

void encoderLeftA_ISR(){
  if(READ_GPIO_LOW(ENCODER_PIN_B_LEFT) != READ_GPIO_LOW(ENCODER_PIN_A_LEFT))
    encoder_ticks_left++;
  else
    encoder_ticks_left--;
}

void encoderLeftB_ISR(){
  if(READ_GPIO_LOW(ENCODER_PIN_A_LEFT) == READ_GPIO_LOW(ENCODER_PIN_B_LEFT))
    encoder_ticks_left++;
  else
    encoder_ticks_left--;
}

void encoderRightA_ISR(){
  if(READ_GPIO_HIGH(ENCODER_PIN_B_RIGHT) != READ_GPIO_HIGH(ENCODER_PIN_A_RIGHT))
    encoder_ticks_right++;
  else
    encoder_ticks_right--;
}

void encoderRightB_ISR(){
  if(READ_GPIO_HIGH(ENCODER_PIN_A_RIGHT) == READ_GPIO_HIGH(ENCODER_PIN_B_RIGHT))
    encoder_ticks_right++;
  else
    encoder_ticks_right--;
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

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A_LEFT),encoderLeftA_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B_LEFT),encoderLeftB_ISR,CHANGE);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A_RIGHT),encoderRightA_ISR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B_RIGHT),encoderRightB_ISR,CHANGE);
  
  
  Serial.println("Encoder-Test mit Umdrehungsausgabe gestartet.");
  #if COM_IS_USED
  communicationInterface->log("Encoder-Test mit Umdrehungsausgabe gestartet.");
  #endif
}

#define TARGET_DISTANCE_MM 1000
#define WHEEL_CIRCUMFERENCE_MM 115.64 ///< d = 35.2 mm 
#define TARGET_TICKS (long)((TARGET_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV)

#define MIN_SPEED 500
#define AVG_SPEED 1000
#define MAX_SPEED 1400

  //#define LEFT_ENCODER_SCALE 1.03// 5 messwerte auf 2m: 5cm rechts, 2cm rechts, 2cm links 4cm rechts, 2cm links
  #define LEFT_ENCODER_SCALE 1.005 //1.015 ganz leicht zu hoch
  #define RIGHT_ENCODER_SCALE 1.00

  #define KP_TICK 5.0f   // Proportional 2.5f 4.8
  #define KI_TICK 0.00//0.30f // Integral (klein halten!) 0.06f
  #define KD_TICK 0.0f
  #define MAX_CORRECTION 250
  #define MAX_INTEGRAL 1000 // Anti-Windup

void loop(){
  long ticks_left, ticks_right, avg_ticks, remaining_ticks;
  
  // get encoder ticks
  ticks_left = abs(encoder_ticks_left);
  ticks_right = abs(encoder_ticks_right);
  
  // calculate remaining ticks
  avg_ticks = (ticks_left + ticks_right) / 2;
  remaining_ticks = TARGET_TICKS - avg_ticks;
  
  // Wenn an Ziel angekommen
  if(remaining_ticks <= 0){
    motor.stopMotors();
    
    #if COM_IS_USED
      for(int i = 0; i < logIndex; i++){
        communicationInterface->log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d | Correction: %d",
                                    logBuffer[i].ticks_left, logBuffer[i].ticks_right, logBuffer[i].avg_ticks, logBuffer[i].speed_left, logBuffer[i].speed_right,
                                    logBuffer[i].tick_diff,  logBuffer[i].correction);
        }
      communicationInterface->log("1 Meter erreicht!");
      communicationInterface->log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d",
                                   ticks_left, ticks_right, avg_ticks, motor.getLeftMotorSpeed(), motor.getRightMotorSpeed(), (int) ((ticks_left * LEFT_ENCODER_SCALE) - (ticks_right * RIGHT_ENCODER_SCALE)));
    #endif
    while(1);  // Dauerhalt
  }
  
  // Set base speed
  static int32_t speed = 600;  // Startgeschwindigkeit
  if(speed < MIN_SPEED) speed = MIN_SPEED;  // Sicherstellen, dass die Startgeschwindigkeit nicht unter dem Minimum liegt
  if(speed < AVG_SPEED) speed += 25;        // Erhöhe die Geschwindigkeit um 50, wenn sie unter dem Durchschnitt liegt
  if(speed > MAX_SPEED) speed = MAX_SPEED;  // Sicherstellen, dass die Geschwindigkeit nicht über dem Maximum liegt
  
  
  int tick_diff, tick_diff_delta, correction;
  static int32_t last_tick_diff = 0;
  static float tick_integral = 0.0f;
  
  int left_speed, right_speed;
  
  // PID-Regler für die Korrektur
  tick_diff = ticks_left * LEFT_ENCODER_SCALE - ticks_right * RIGHT_ENCODER_SCALE;
  tick_diff_delta = tick_diff - last_tick_diff;
  last_tick_diff = tick_diff;
  tick_integral += tick_diff;
  tick_integral = constrain(tick_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  
  correction = constrain((int)(KP_TICK * tick_diff + KI_TICK * tick_integral + KD_TICK * tick_diff_delta), -MAX_CORRECTION, MAX_CORRECTION);
  
  const float BASE_CORRECTION = 0.125; //0.150;
  left_speed  = constrain(speed - correction - BASE_CORRECTION * speed, MIN_SPEED, MAX_SPEED);
  right_speed = constrain(speed + correction + BASE_CORRECTION * speed, MIN_SPEED, MAX_SPEED);
  
  // Debug-Ausgabe
  #if COM_IS_USED
    if(logIndex < MAX_LOG_ENTRIES){
      logBuffer[logIndex++] = {
        .ticks_left = ticks_left,
        .ticks_right = ticks_right,
        .avg_ticks = avg_ticks,
        .speed_left = left_speed,
        .speed_right = right_speed,
        .tick_diff = tick_diff,
        .correction = correction
      };
    }
  #endif
  
  // Ansteuern
  motor.setLeftMotor((uint16_t)left_speed, MotorDirection::FORWARD);
  motor.setRightMotor((uint16_t)right_speed, MotorDirection::FORWARD);
  
  delay(50);  // Entlastung
}
