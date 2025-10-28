#ifndef __MOVEMENT_CONTROLLER_H__
#define __MOVEMENT_CONTROLLER_H__

#include "maze_map.h"
#include "movement_action.h"
#include "sensor_manager.h"
#include "motor_driver.h"

class MovementController{
public:
  // === Methods ===
  MovementController(MazeMap &map);
  
  // --- Movement and Mapping ---
  void mapStartCell();
  void map3WallsCurrentCell();
  void executeAction(MovementAction action);
  
  // --- Special behavior ---
  void performTrapDelay();                   ///< Perform 360° turn after detecting a trap
  
  // --- Simulation ---
  void printSimulatedMaze() const;
  void printSimulatedMazeColored() const;
private:
  // === Constants ===
  //--- Motor encoders ---
  /**
   * @brief Number of encoder ticks per revolution.
   * 
   * The Pololu motor encoders have 12 counts per revolution (CPR), meaning 12 edges (A and B) per revolution.
   * We count both edges of channel A and both edges of channel B, resulting in 12 counts per revolution.
   * Our motor is equipped with a 100:1 gearbox, so we multiply the counts by 100.
   * Thus, we have: 12 ticks per revolution * 100 = 1200 ticks per revolution.
   */
  static constexpr uint32_t TICKS_PER_REV = 1200;
  
  /**
   * @brief The circumference of the wheel in millimeters.
   * 
   * The wheel has a diameter of about 36-37 mm.
   * Through tests on a 2 m long track, we determined that the wheel circumference is approximately 115.64 mm.
   * Thus the wheels diameter is about 36.8 mm.
   */
  static constexpr float WHEEL_CIRCUMFERENCE_MM = 115.64f;
  
  /**
   * @brief The distance in millimeters that the robot moves per cell.
   * 
   * This is the cell size + the width of the wall separating cells.
   */
  static constexpr float CELL_DISTANCE_MM = 253.0f; // Cell size is 250 mm, wall width is 3 mm
  
  /**
   * @brief The target number of encoder ticks to move one cell forward.
   * 
   * This is based on the formula:
   * TARGET_TICKS = (TARGET_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV
   */
  static constexpr uint32_t TARGET_TICKS = static_cast<uint32_t>((CELL_DISTANCE_MM / WHEEL_CIRCUMFERENCE_MM) * TICKS_PER_REV);
  
  /**
   * @brief The minimum speed for the motors.
   * 
   * This is the minimum speed at which the robot should move forward, to avoid stalling.
   */
  static constexpr uint16_t MIN_SPEED = 500;
  
  /**
   * @brief The average speed for the motors.
   * 
   * This is the average speed at which the robot should move forward.
   */
  static constexpr uint16_t AVG_SPEED = 1000;
  
  /**
   * @brief The maximum speed for the motors.
   * 
   * This is the maximum speed at which the robot should move forward, to avoid skidding of the wheels.
   */
  static constexpr uint16_t MAX_SPEED = 1400;
  
  /**
   * @brief The scale factor for the left encoder.
   * 
   * The left encoder is slightly faster than the right encoder due to mechanical differences.
   * Proably due to slightly different wheel diameters or friction.
   */
  static constexpr float LEFT_ENCODER_SCALE = 1.005f;
  
  /**
   * @brief The scale factor for the right encoder.
   * 
   * The right encoder is slightly slower than the left encoder due to mechanical differences.
   * Proably due to slightly different wheel diameters or friction.
   */
  static constexpr float RIGHT_ENCODER_SCALE = 1.00f;
  
  // --- Wall detection ---
  /**
   * @brief The desired distance to stop in front of a wall.
   * 
   * This is the distance in millimeters that the robot should stop in front of a wall.
   * It is used to prevent the robot from crashing into walls.
   * This must be less than 255 (invalid value for the VL6180X distance sensor).
   */
  static constexpr uint8_t DESIRED_FRONT_WALL_DISTANCE_MM = 50;
  
  // --- PID Control Constants ---
  static constexpr float KP_TICK = 5.0f;   ///< Proportional constant for speed correction based on encoder ticks
  static constexpr float KI_TICK = 0.00f;  ///< Integral constant for speed correction (kept small to avoid overshooting)
  static constexpr float KD_TICK = 0.0f;   ///< Derivative constant for speed correction (not used in this implementation)
  static constexpr float BASE_CORRECTION = 0.0f;//0.125f; ///< Base correction factor for speed adjustment, to ensure smooth movement, even at start of movement
  static constexpr int32_t MAX_CORRECTION = 250;   ///< Maximum speed correction to prevent excessive speed differences
  static constexpr int32_t MAX_INTEGRAL = 1000;    ///< Maximum integral value to prevent windup
  
  // === Attributes ===
  MazeMap &maze_map_;             ///< Reference to the map to modify
  SensorManager sensor_manager_;  ///< Sensor manager instance
  MotorDriver motor_driver_;      ///< Motor driver instance
  
  enum class Degree{
    DEG_90 = 0,
    DEG_180,
    DEG_270,
    DEG_360
  };
  // === Methods ===
  // --- Internal helpers ---
  void moveCellForward();
  void alignToFrontWall();
  void turnLeft(Degree deg);
  void turnRight(Degree deg);
};

#endif // __MOVEMENT_CONTROLLER_H__
