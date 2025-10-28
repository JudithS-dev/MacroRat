/*#include "error_handler.h"

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
//*
// @brief Ticks per revolution for the encoder
// 
// The encoder has 12 counts per revolution (CPR), meaning 12 edges (A and B) per revolution.
// If we only count the edges of channel A, we get half the number of counts, which is 6.
// If we only count the rising edges of channel A, we get half of that again, which is 3.
// Our motor is equipped with a 100:1 gearbox, so we multiply the counts by 100.
// Thus, we have: 3 ticks per revolution * 100 = 300 ticks per revolution.
///

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

//
// @brief radumfang anpassen
// auf 2m 4cm zu wenig gefahren
// => 2m - 4cm = 1960 mm = 21254 ticks
// => 2m = 2000 mm = 21687 ticks
// => 21687 / 1200 = 18.0725 Umdrehungen
// => 2m / 18.0725 Umdrehungen = 110.665 mm pro Umdrehung
// => WHEEL_CIRCUMFERENCE_MM = 110.665 mm statt davor 113.097 mm
// => d = 35.2 mm statt davor d = 36 mm
// 
///
//
//
// @brief Radumfang anpassen die zweite:
// auf 25 cm 6 mm zu viel gefahren
// ROboter wollte 2710 ticks fahren ist aber 2732 ticks gefahren => 
// 2732 Ticks = 256 mm
// 1200 Ticks = 112.445 mm statt davor 110.665 mm
// => d = 35,79 mm
// 
// 
//
//
//
// @brief Radumfang anpassen die dritte:
// auf 25 cm 1cm zu viel gefahren
// Roboter ist 2698 Ticks gefahren
// => 2698 Ticks = 260 mm
// => 1200 Ticks = 115.64 mm statt davor 112.445 mm
// 
// 
///
#define TARGET_DISTANCE_MM 1000
#define WHEEL_CIRCUMFERENCE_MM 115.64 ///< d = 35.2 mm 
#define TARGET_TICKS (long)((TARGET_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV) // = 

#define MIN_SPEED 500
#define AVG_SPEED 1000
#define MAX_SPEED 1400

  //#define LEFT_ENCODER_SCALE 1.03// 5 messwerte auf 2m: 5cm rechts, 2cm rechts, 2cm links 4cm rechts, 2cm links
  #define LEFT_ENCODER_SCALE 1.010 //1.015 ganz leicht zu hoch
  #define RIGHT_ENCODER_SCALE 1.00

  #define KP_TICK 5.0f   // Proportional 2.5f 4.8
  #define KI_TICK 0.00//0.30f // Integral (klein halten!) 0.06f
  #define KD_TICK 0.0f
  #define MAX_CORRECTION 250
  #define MAX_INTEGRAL 1000 // Anti-Windup


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




void loop(){
  static unsigned long last_output = 0;
  
  long ticks_left = abs(encoder_ticks_left);
  long ticks_right = abs(encoder_ticks_right);
  long avg_ticks = (ticks_left + ticks_right) / 2;
  
  long remaining_ticks = TARGET_TICKS - avg_ticks;
  
  if(remaining_ticks <= 0){
    motor.stopMotors();
    
    Serial.println("1 Meter erreicht!");
    #if COM_IS_USED
    for (int i = 0; i < logIndex; i++) {
  communicationInterface->log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d | Correction: %d",
                              logBuffer[i].ticks_left,
                              logBuffer[i].ticks_right,
                              logBuffer[i].avg_ticks,
                              logBuffer[i].speed_left,
                              logBuffer[i].speed_right,
                              logBuffer[i].tick_diff,
                              logBuffer[i].correction);
}

    communicationInterface->log("1 Meter erreicht!");
    communicationInterface->log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d",
                                  ticks_left, ticks_right, avg_ticks, motor.getLeftMotorSpeed(), motor.getRightMotorSpeed(), (int) ((ticks_left * LEFT_ENCODER_SCALE) - (ticks_right * RIGHT_ENCODER_SCALE)));
    #endif
    while(1);  // Dauerhalt
  }
  
 


  static int32_t speed = 600;  // Startgeschwindigkeit
  if(speed < MIN_SPEED) speed = MIN_SPEED;  // Sicherstellen, dass die Startgeschwindigkeit nicht unter dem Minimum liegt
  if(speed < AVG_SPEED) speed += 25;  // Erhöhe die Geschwindigkeit um 50, wenn sie unter dem Durchschnitt liegt
  if(speed > MAX_SPEED) speed = MAX_SPEED;  // Sicherstellen, dass die Geschwindigkeit nicht über dem Maximum liegt



  static int32_t last_tick_diff = 0;
  static float tick_integral = 0.0f;

  //
  // @brief Testergebnisse mit LEFT_ENCODER_SCALE = 1.0125 und RIGHT_ENCODER_SCALE = 1.00
  // Abweichung auf 2m Strecke:
  // 1. Versuch 8 cm nach rechts
  // 2. Versuch 5 cm nach links
  // 3. Versuch 1 cm nach rechts
  // 4. Versuch 16 cm nach links
  // 5. Versuch 9 cm nach links
  // => Mittelwert: 7.8 cm nach links => öfter nach links als nach rechts => LEFT_ENCODER_SCALE ist leicht zu hoch
  // 
  // @brief Testergebnisse mit LEFT_ENCODER_SCALE = 1.0123 und RIGHT_ENCODER_SCALE = 1.00
  // Abweichung auf 2m Strecke:
  // 1. Versuch 4.5 cm nach rechts
  // 2. Versuch 5.5 cm nach rechts
  // 3. Versuch 3.5 cm nach rechts
  // 4. Versuch 4 cm nach rechts
  // 5. Versuch 4.5 cm nach rechts
  // => Mittelwert: 4.2 cm nach rechts => LEFT_ENCODER_SCALE ist etwas zu niedrig
  // 
  // 
  // @brief Testergebnisse mit LEFT_ENCODER_SCALE = 1.01235 und RIGHT_ENCODER_SCALE = 1.00
  // Abweichung auf 2m Strecke:
  // 1. Versuch 17 cm nach rechts
  // 2. Versuch 16 cm nach rechts
  // 3. Versuch 7 cm nach rechts
  // 4. Versuch 23 cm nach rechts
  // => Mittelwert: 14.25 cm nach rechts => LEFT_ENCODER_SCALE ist zu niedrig
  ///

  int tick_diff = ticks_left * LEFT_ENCODER_SCALE - ticks_right * RIGHT_ENCODER_SCALE;


  int tick_diff_delta = tick_diff - last_tick_diff;
  last_tick_diff = tick_diff;

  // Integral-Term aufaddieren
  tick_integral += tick_diff;
  tick_integral = constrain(tick_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  
  int correction = (int)(KP_TICK * tick_diff + KI_TICK * tick_integral + KD_TICK * tick_diff_delta);


// Begrenzen
correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  const float BASE_CORRECTION = 0.125; //0.150;

  int left_speed = speed - correction - BASE_CORRECTION * speed; 
  int right_speed = speed + correction + BASE_CORRECTION * speed;

  left_speed  = constrain(left_speed,  MIN_SPEED, MAX_SPEED);
  right_speed = constrain(right_speed, MIN_SPEED, MAX_SPEED);


  
  
  //#if COM_IS_USED
  //  if(millis() - last_output > 0){
  //    last_output = millis();
  //    //communicationInterface->log ("%2f, %d, %d, %d", current_heading, left_speed, right_speed, correction);
  //    //communicationInterface->log("Gyro: %.2f° | Speed L: %d | Speed R: %d | Correction: %d", current_heading, left_speed, right_speed, correction);
  //    communicationInterface->log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d | Correction: %d", 
  //                                ticks_left, ticks_right, avg_ticks, left_speed, right_speed, tick_diff, correction);
  //  }
  //#endif

  if (logIndex < MAX_LOG_ENTRIES) {
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

  
  // Ansteuern
  motor.setLeftMotor((uint16_t)left_speed, MotorDirection::FORWARD);
  motor.setRightMotor((uint16_t)right_speed, MotorDirection::FORWARD);
  
  delay(50);  // Entlastung
}
*/