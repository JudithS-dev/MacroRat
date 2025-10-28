#ifndef __MOVEMENT_ACTION_H__
#define __MOVEMENT_ACTION_H__


/**
 * @enum MovementAction
 * @brief Possible actions for the rat in the maze.
 */
enum class MovementAction {
  MOVE_FORWARD = 0,  ///< Move forward
  TURN_LEFT_90,      ///< Turn left (90°)
  TURN_LEFT_180,     ///< Turn left (180°)
  TURN_LEFT_270,     ///< Turn left (270°)
  TURN_LEFT_360,     ///< Turn left (360°)
  TURN_RIGHT_90,     ///< Turn right (90°)
  TURN_RIGHT_180,    ///< Turn right (180°)
  TURN_RIGHT_270,    ///< Turn right (270°)
  TURN_RIGHT_360,    ///< Turn right (360°)
  NO_ACTION,         ///< No action
};

#endif // __MOVEMENT_ACTION_H__