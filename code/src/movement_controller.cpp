#include "movement_controller.h"
#include <Arduino.h>

#include "operation_mode.h"
#include "error_handler.h"

/**
 * @brief Construct a new Movement Controller:: Movement Controller object
 * 
 * @param map Reference to the MazeMap object
 */
MovementController::MovementController(MazeMap &map)
: maze_map_(map), 
  sensor_manager_(map.rat_pos), 
  motor_driver_()
{
  CommunicationInterface::log("    MovementController: Initialization complete!");
}



// --- Movement and Mapping ---
/**
 * @brief Maps the start cell by checking the walls in all directions.
 * @note The rat will turn 90° to check the wall behind it.
 */
void MovementController::mapStartCell(){
  map3WallsCurrentCell();
  turnLeft(Degree::DEG_90); // Rotate 90° to check the wall behind the rat
  map3WallsCurrentCell();
}

/**
 * @brief Maps the current cell by checking the walls in the left, front, and right directions.
 */
void MovementController::map3WallsCurrentCell() {
  // Check left wall
  SensorWallState current_wall = sensor_manager_.isWallLeft();
  switch(current_wall){
    case SensorWallState::NO_WALL: maze_map_.setLeftWallState(WallState::NO_WALL); break;
    case SensorWallState::FOUND_WALL: maze_map_.setLeftWallState(WallState::WALL); break;
    case SensorWallState::CONFLICT: fatalError("ERROR 'MovementController::mapCurrentCell': Conflict detected while checking left wall."); break;
    default: fatalError("ERROR 'MovementController::mapCurrentCell': Unknown wall state while checking left wall."); break;
  }
  
  // Check front wall
  current_wall = sensor_manager_.isWallFront();
  switch(current_wall){
    case SensorWallState::NO_WALL: maze_map_.setFrontWallState(WallState::NO_WALL); break;
    case SensorWallState::FOUND_WALL: maze_map_.setFrontWallState(WallState::WALL); break;
    case SensorWallState::CONFLICT: fatalError("ERROR 'MovementController::mapCurrentCell': Conflict detected while checking front wall."); break;
    default: fatalError("ERROR 'MovementController::mapCurrentCell': Unknown wall state while checking front wall."); break;
  }
  
  // Check right wall
  current_wall = sensor_manager_.isWallRight();
  switch(current_wall){
    case SensorWallState::NO_WALL: maze_map_.setRightWallState(WallState::NO_WALL); break;
    case SensorWallState::FOUND_WALL: maze_map_.setRightWallState(WallState::WALL); break;
    case SensorWallState::CONFLICT: fatalError("ERROR 'MovementController::mapCurrentCell': Conflict detected while checking right wall."); break;
    default: fatalError("ERROR 'MovementController::mapCurrentCell': Unknown wall state while checking right wall."); break;
  }
  
  // Update maze cell state
  if(sensors_simulated()){ // if simulated just call getNfcTagType to get simulated tag-data
    NfcTagType tag_type;
    tag_type = sensor_manager_.getNfcTagType();
    switch(tag_type){
      case NfcTagType::NO_TAG: maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::EMPTY);  break;
      case NfcTagType::CHEESE: maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::CHEESE); break;
      case NfcTagType::TRAP:   maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::TRAP);   break;
      default: fatalError("ERROR 'MovementController::mapCurrentCell': Unknown NFC tag type.");
    }
  } else{
    // nothing to do here, as nfc tag detection is done in the moveForward method, but set the cell state to EMPTY to mark it as visited
    CellState current_cell_state = maze_map_.maze.getCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY());
    switch(current_cell_state){
      case CellState::NON_VISITED:  maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::EMPTY);
                                    break;
      case CellState::EMPTY:        // If the cell is EMPTY, CHEESE or TRAP, we keep its state
      case CellState::TRAP:
      case CellState::CHEESE:       break;
      case CellState::UNREACHABLE:  fatalError("ERROR 'MovementController::mapCurrentCell': Cell state UNREACHABLE is not allowed while mapping current cell. This is impossible");
      default:
        fatalError("ERROR 'MovementController::mapCurrentCell': Unknown cell state while mapping current cell.");
    }
  }
}

/**
 * @brief Executes the specified movement action.
 * 
 * @param action The movement action to perform (e.g., MOVE_FORWARD, TURN_LEFT_90, etc.)
 */
void MovementController::executeAction(MovementAction action){  
  // --- Perform movement based on direction ---
  // get current orientation of the rat
  Direction current_orientation = maze_map_.rat_pos.getOrientation();
  switch(action){
    case MovementAction::MOVE_FORWARD:   moveCellForward();          break;
    case MovementAction::TURN_LEFT_90:   turnLeft(Degree::DEG_90);   break;
    case MovementAction::TURN_LEFT_180:  turnLeft(Degree::DEG_180);  break;
    case MovementAction::TURN_LEFT_270:  turnLeft(Degree::DEG_270);  break;
    case MovementAction::TURN_LEFT_360:  turnLeft(Degree::DEG_360);  break;
    case MovementAction::TURN_RIGHT_90:  turnRight(Degree::DEG_90);  break;
    case MovementAction::TURN_RIGHT_180: turnRight(Degree::DEG_180); break;
    case MovementAction::TURN_RIGHT_270: turnRight(Degree::DEG_270); break;
    case MovementAction::TURN_RIGHT_360: turnRight(Degree::DEG_360); break;
    case MovementAction::NO_ACTION: break; // No action needed
    default: fatalError("ERROR 'MovementController::moveInDirection': Unknown movement action.");
  }
}



// --- Simulation ---
void MovementController::printSimulatedMaze() const {
  sensor_manager_.printSimulatedMaze();
}
void MovementController::printSimulatedMazeColored() const {
  sensor_manager_.printSimulatedMazeColored();
}

#define FORWARD_LOGGING false
#if FORWARD_LOGGING
#define MAX_LOG_ENTRIES 300

struct EncoderLogEntry {
  long ticks_left;
  long ticks_right;
  long avg_ticks;
  long remaining_ticks;
  uint32_t speed_left;
  uint32_t speed_right;
  int tick_diff;
  int correction;
};

EncoderLogEntry logBuffer[MAX_LOG_ENTRIES];
int logIndex = 0;
#endif // FORWARD_LOGGING

// --- Internal helpers ---
/**
 * @brief Moves the robot one cell forward in the current direction.
 *
 * The movement is typically performed using timed or sensor-controlled motion.
 * The robot will stop when it detects a wall or reaches the end of the cell.
 */
void MovementController::moveCellForward(){
  // --- Ceck if it is possible to move forward ---
  if(sensor_manager_.isWallFront() == SensorWallState::FOUND_WALL){
    fatalError("ERROR 'MovementController::moveCellForward': Wall detected in front of the rat.");
  }
  
  // --- Perform motor movement if not simulated ---
  if(OPERATION_MODE == OperationMode::SEMI_SIMULATED || OPERATION_MODE == OperationMode::REAL_OPERATION){
    if(OPERATION_MODE == OperationMode::REAL_OPERATION){
      //TODO: implement NFC tag detection during movement, if real operation mode
    }
    
    bool wall_too_close = false;
    int32_t ticks_left, ticks_right, avg_ticks, remaining_ticks;
    int32_t tick_diff, tick_diff_delta, correction;
    int32_t last_tick_diff = 0;
    float tick_integral = 0.0f;
    
    uint32_t left_speed, right_speed;
    uint32_t base_speed = 575; // Initial speed for the motors = 600 (575 + 25)
    
    // Loop until the rat has moved forward enough
    uint8_t wall_dist_frontleft = 255;
    uint8_t wall_dist_frontright = 255; // Initialize with maximum distance
    bool check_wall_frontleft = true;
    uint32_t lastReadingTime = millis();
    while(true){
      // Get wall distances only every 1000 ms to avoid excessive sensor reads
      if(millis() - lastReadingTime > 500){
        if(check_wall_frontleft){
          wall_dist_frontleft  = sensor_manager_.getDistanceFrontLeft();
        } else{
          wall_dist_frontright = sensor_manager_.getDistanceFrontRight();
        }
        lastReadingTime = millis();
        check_wall_frontleft = !check_wall_frontleft; // Alternate between checking front-left and front-right
      }
      wall_too_close = (wall_dist_frontleft < DESIRED_FRONT_WALL_DISTANCE_MM) || (wall_dist_frontright < DESIRED_FRONT_WALL_DISTANCE_MM);
      
      // If a wall is too close, stop the motors and exit the loop
      if(wall_too_close){
        motor_driver_.stopMotors();
        //CommunicationInterface::log("MovementController: Wall detected in front of the rat, starting to align with the wall.");
        //CommunicationInterface::log("MovementController: Wall distance front-left: %d mm, front-right: %d mm, desired distance: %d mm",
        //                            wall_dist_frontleft, wall_dist_frontright, DESIRED_FRONT_WALL_DISTANCE_MM);
        break;
      }
      
      // Get current encoder ticks
      ticks_left = abs(sensor_manager_.getEncoderLeft());
      ticks_right = abs(sensor_manager_.getEncoderRight());
      
      // Calculate remaining ticks to move forward
      avg_ticks = (LEFT_ENCODER_SCALE * ticks_left + RIGHT_ENCODER_SCALE * ticks_right) / 2;
      remaining_ticks = TARGET_TICKS - avg_ticks;
      
      // Check if rat moved already enough
      if(remaining_ticks <= 0){
        motor_driver_.stopMotors();
        break;
      }
      
      // Adjust motor speed for gradual acceleration
      if(base_speed < MIN_SPEED) base_speed = MIN_SPEED; // Ensure minimum speed
      if(base_speed < AVG_SPEED) base_speed += 25; // Increase speed gradually, to avoid skidding
      if(base_speed > MAX_SPEED) base_speed = MAX_SPEED; // Ensure maximum speed
      
      // Calculate PID correction for motor speed
      tick_diff = LEFT_ENCODER_SCALE * ticks_left - RIGHT_ENCODER_SCALE * ticks_right;
      tick_diff_delta = tick_diff - last_tick_diff;
      last_tick_diff = tick_diff;
      tick_integral += tick_diff;
      tick_integral = constrain(tick_integral, -MAX_INTEGRAL, MAX_INTEGRAL);
      
      // Calculate correction based on PID values
      correction = constrain(static_cast<int32_t>(KP_TICK * tick_diff + KI_TICK * tick_integral + KD_TICK * tick_diff_delta), -MAX_CORRECTION, MAX_CORRECTION);
      
      // Adjust motor speeds based on correction
      left_speed  = constrain(base_speed - correction - BASE_CORRECTION * base_speed, MIN_SPEED, MAX_SPEED);
      right_speed = constrain(base_speed + correction + BASE_CORRECTION * base_speed, MIN_SPEED, MAX_SPEED);
      
      // Set motor speeds
      motor_driver_.setLeftMotor(static_cast<uint16_t>(left_speed), MotorDirection::FORWARD);
      motor_driver_.setRightMotor(static_cast<uint16_t>(right_speed), MotorDirection::FORWARD);
      
      #if FORWARD_LOGGING
      if(logIndex < MAX_LOG_ENTRIES){
        logBuffer[logIndex++] = {
          .ticks_left = ticks_left,
          .ticks_right = ticks_right,
          .avg_ticks = avg_ticks,
          .remaining_ticks = remaining_ticks,
          .speed_left = left_speed,
          .speed_right = right_speed,
          .tick_diff = tick_diff,
          .correction = correction
        };
      }
      #endif // FORWARD_LOGGING
    }
    #if FORWARD_LOGGING
    for(int i = 0; i < logIndex; i++){
        CommunicationInterface::log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Remaining Tick: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d | Correction: %d",
                                    logBuffer[i].ticks_left, logBuffer[i].ticks_right, logBuffer[i].avg_ticks, logBuffer[i].remaining_ticks, logBuffer[i].speed_left, logBuffer[i].speed_right,
                                    logBuffer[i].tick_diff,  logBuffer[i].correction);
        }
    CommunicationInterface::log("25 cm erreicht!");
    CommunicationInterface::log("Ticks L: %5ld | Ticks R: %5ld | Avg Ticks: %5ld | Remaining Tick: %5ld | Speed L: %4d | Speed R: %4d | Tick-Diff: %3d",
                                 ticks_left, ticks_right, avg_ticks, remaining_ticks, motor_driver_.getLeftMotorSpeed(), motor_driver_.getRightMotorSpeed(), (int) ((ticks_left * LEFT_ENCODER_SCALE) - (ticks_right * RIGHT_ENCODER_SCALE)));
    #endif // FORWARD_LOGGING
    
    // Reset encoders to difference after forward movement
    sensor_manager_.resetEncodersToDiff();
    
    // --- Try to align with front wall ---
    alignToFrontWall();
  }
  
  // --- Update internal rat position ---
  maze_map_.rat_pos.moveForward();
  
  // --- Check for wall detection ---
  //map3WallsCurrentCell();

  // Update maze cell state
  if(sensors_simulated()){ // if simulated just call getNfcTagType to get simulated tag-data
    NfcTagType tag_type;
    tag_type = sensor_manager_.getNfcTagType();
    switch(tag_type){
      case NfcTagType::NO_TAG: maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::EMPTY);  break;
      case NfcTagType::CHEESE: maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::CHEESE); break;
      case NfcTagType::TRAP:   maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::TRAP);   break;
      default: fatalError("ERROR 'MovementController::mapCurrentCell': Unknown NFC tag type.");
    }
  } else{
    // nothing to do here, as nfc tag detection is done in the moveForward method, but set the cell state to EMPTY to mark it as visited
    CellState current_cell_state = maze_map_.maze.getCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY());
    switch(current_cell_state){
      case CellState::NON_VISITED:  maze_map_.maze.setCellState(maze_map_.rat_pos.getX(), maze_map_.rat_pos.getY(), CellState::EMPTY);
                                    break;
      case CellState::EMPTY:        // If the cell is EMPTY, CHEESE or TRAP, we keep its state
      case CellState::TRAP:
      case CellState::CHEESE:       break;
      case CellState::UNREACHABLE:  fatalError("ERROR 'MovementController::mapCurrentCell': Cell state UNREACHABLE is not allowed while mapping current cell. This is impossible");
      default:
        fatalError("ERROR 'MovementController::mapCurrentCell': Unknown cell state while mapping current cell.");
    }
  }
}

void MovementController::alignToFrontWall() {
  uint8_t wall_dist_frontleft, wall_dist_frontright;
  
  // Check if a wall can be detected in front of both sensors
  wall_dist_frontleft  = sensor_manager_.getDistanceFrontLeft();
  wall_dist_frontright = sensor_manager_.getDistanceFrontRight();
  
  if(wall_dist_frontleft == 255 && wall_dist_frontright == 255) {
    // No wall detected in front of both sensors, no alignment needed
    //CommunicationInterface::log("MovementController: No wall detected in front of the rat, no alignment needed.");
    return;
  }
  
  const int16_t ALIGNMENT_THRESHOLD = 5; ///< Threshold for front alignment in mm
  uint32_t base_speed = 600; // Base speed for alignment
  int16_t diff = static_cast<int16_t>(wall_dist_frontleft) - static_cast<int16_t>(wall_dist_frontright);
  
  while(abs(diff) > ALIGNMENT_THRESHOLD){
    if(diff > 0){ // Left sensor is closer to the wall
      // Turn right to align with the wall
      motor_driver_.setLeftMotor(base_speed, MotorDirection::FORWARD);
      motor_driver_.setRightMotor(base_speed, MotorDirection::BACKWARD);
    } else { // Right sensor is closer to the wall
      // Turn left to align with the wall
      motor_driver_.setLeftMotor(base_speed, MotorDirection::BACKWARD);
      motor_driver_.setRightMotor(base_speed, MotorDirection::FORWARD);
    }
    
    delay(100);
    motor_driver_.stopMotors(); // Stop motors to check distances again
    delay(100); // Wait for a short time to allow sensors to stabilize
    
    // Re-check wall distances
    wall_dist_frontleft  = sensor_manager_.getDistanceFrontLeft();
    wall_dist_frontright = sensor_manager_.getDistanceFrontRight();
    
    if(wall_dist_frontleft == 255 && wall_dist_frontright == 255) {
      // No wall detected in front of both sensors, exit alignment
      fatalError("ERROR 'MovementController::alignToFrontWall': No wall detected in front of the rat during alignment.");
    }
    
    diff = static_cast<int16_t>(wall_dist_frontleft) - static_cast<int16_t>(wall_dist_frontright);
  }
  
  //CommunicationInterface::log("MovementController: Aligned with front wall. Left distance: %d mm, Right distance: %d mm", wall_dist_frontleft, wall_dist_frontright);
}


/**
 * @brief Rotates the robot deg degrees to the left (counter-clockwise).
 *
 * During the rotation, the NFC reader remains active to detect any trap tags.
 * The rotation is typically performed by driving the left motor backward and
 * the right motor forward using timed or sensor-controlled motion.
 */
void MovementController::turnLeft(Degree deg) {
  // --- Perform motor rotation if not simulated ---
  float angle_bias = 3.0f; // Bias for angle correction, if needed
  if(OPERATION_MODE == OperationMode::SEMI_SIMULATED || OPERATION_MODE == OperationMode::REAL_OPERATION){
    // Get target rotation angle
    float target_angle = 0.0f;
    switch(deg){
      case Degree::DEG_90:  target_angle = 90.0f  - 2 * angle_bias; break;
      case Degree::DEG_180: target_angle = 180.0f - 4 * angle_bias; break;
      case Degree::DEG_270: target_angle = 270.0f - 6 * angle_bias; break;
      case Degree::DEG_360: return; // 360° is equivalent to 0°
      default: fatalError("ERROR 'MovementController::turnLeft': Unknown degree of rotation.");
    }
    
    // Perform rotation
    #if USE_GYROSCOPE
    sensor_manager_.resetGyroZ(); // Reset gyroscope before rotation
    motor_driver_.turnInCircleLeft(800); // Start turning left
    
    const float angle_tolerance = 1.0f; // Tolerance for angle measurement
    while(true){
      sensor_manager_.updateGyroZAngle(); // Update gyroscope angle
      float current_angle = sensor_manager_.getGyroZRotationAngle();
      
      // Check if the target angle is reached
      float needed_angle = target_angle - current_angle; // Calculate needed angle to reach target
      if(needed_angle < angle_tolerance){
        break; // Exit loop when target angle is reached
      }
      
      delay(10); // Wait before next check
    }
    motor_driver_.stopMotors(); // Stop after rotation
    #else
    motor_driver_.turnInCircleLeft(800);
    switch(deg){
      case Degree::DEG_90:  delay(1000); break;
      case Degree::DEG_180: delay(2000); break;
      case Degree::DEG_270: delay(3000); break;
      case Degree::DEG_360: break; // 360° is equivalent to 0°
      default: fatalError("ERROR 'MovementController::turnLeft': Unknown degree of rotation.");
    }
    motor_driver_.stopMotors(); // Stop after rotation
    #endif // USE_GYROSCOPE
    sensor_manager_.resetEncodersToZero(); // Reset encoders after rotation
  }
  
  // --- Update internal rat orientation ---
  switch(deg){
    case Degree::DEG_90:  maze_map_.rat_pos.turnLeft(); break;
    case Degree::DEG_180: maze_map_.rat_pos.turnAround(); break;
    case Degree::DEG_270: maze_map_.rat_pos.turnRight(); break;
    case Degree::DEG_360: break; // 360° is equivalent to 0°
    default: fatalError("ERROR 'MovementController::turnLeft': Unknown degree of rotation.");
  }
}

/**
 * @brief Rotates the robot deg degrees to the right (clockwise).
 *
 * During the rotation, the NFC reader remains active to detect any trap tags.
 * The rotation is typically performed by driving the left motor forward and
 * the right motor backward using timed or sensor-controlled motion.
 */
void MovementController::turnRight(Degree deg){
  // --- Perform motor rotation if not simulated ---
  float angle_bias = 3.0f; // Bias for angle correction, if needed
  if(OPERATION_MODE == OperationMode::SEMI_SIMULATED || OPERATION_MODE == OperationMode::REAL_OPERATION){
    // Get target rotation angle
    float target_angle = 0.0f;
    switch(deg){
      case Degree::DEG_90:  target_angle = -90.0f  + 2 * angle_bias; break;
      case Degree::DEG_180: target_angle = -180.0f + 4 * angle_bias; break;
      case Degree::DEG_270: target_angle = -270.0f + 6 * angle_bias; break;
      case Degree::DEG_360: return; // 360° is equivalent to 0°
      default: fatalError("ERROR 'MovementController::turnRight': Unknown degree of rotation.");
    }
    
    // Perform rotation
    #if USE_GYROSCOPE
    sensor_manager_.resetGyroZ(); // Reset gyroscope before rotation
    motor_driver_.turnInCircleRight(800); // Start turning right
    
    const float angle_tolerance = 1.0f; // Tolerance for angle measurement
    while(true){
      sensor_manager_.updateGyroZAngle(); // Update gyroscope angle
      float current_angle = sensor_manager_.getGyroZRotationAngle();
      
      // Check if the target angle is reached
      float needed_angle = current_angle - target_angle; // Calculate needed angle to reach target
      if(needed_angle < angle_tolerance){
        break; // Exit loop when target angle is reached
      }
      
      delay(10); // Wait before next check
    }
    motor_driver_.stopMotors(); // Stop after rotation
    #else
    motor_driver_.turnInCircleRight(800);
    switch(deg){
      case Degree::DEG_90:  delay(1000); break;
      case Degree::DEG_180: delay(2000); break;
      case Degree::DEG_270: delay(3000); break;
      case Degree::DEG_360: break; // 360° is equivalent to 0°
      default: fatalError("ERROR 'MovementController::turnLeft': Unknown degree of rotation.");
    }
    motor_driver_.stopMotors(); // Stop after rotation
    #endif // USE_GYROSCOPE
    sensor_manager_.resetEncodersToZero(); // Reset encoders after rotation
  }
  
  // --- Update internal rat orientation ---
  switch(deg){
    case Degree::DEG_90:  maze_map_.rat_pos.turnRight(); break;
    case Degree::DEG_180: maze_map_.rat_pos.turnAround(); break;
    case Degree::DEG_270: maze_map_.rat_pos.turnLeft(); break;
    case Degree::DEG_360: break; // 360° is equivalent to 0°
    default: fatalError("ERROR 'MovementController::turnRight': Unknown degree of rotation.");
  }
}