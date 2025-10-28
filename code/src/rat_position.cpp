#include "rat_position.h"

#include <Arduino.h>
#include <stdlib.h>

#include "maze.h"
#include "error_handler.h"

/**
 * @brief Constructs a RatPosition object starting at (0,0) facing NORTH.
 * @param maze Reference to the maze object.
 */
RatPosition::RatPosition(const Maze &maze, CommunicationInterface& comm_interface)
: comm_interface_(comm_interface),
  x_(0),
  y_(0),
  direction_(Direction::NORTH),
  maze_(maze)
{
  CommunicationInterface::log("           RatPosition: Initialization complete!");
}



// --- Getters ---
/**
 * @brief Gets the current x-coordinate of the rat.
 * @return The x-coordinate.
 */
int16_t RatPosition::getX() const{
  return x_;
}

/**
 * @brief Gets the current y-coordinate of the rat.
 * @return The y-coordinate.
 */
int16_t RatPosition::getY() const{
  return y_;
}

/**
 * @brief Gets the current orientation of the rat.
 * @return The direction the rat is facing.
 */
Direction RatPosition::getOrientation() const{
  return direction_;
}



// --- Movement ---
/**
 * @brief Moves the rat one step forward in the current direction.
 * @note The rat's position is updated, and bounds are checked.
 */
void RatPosition::moveForward(){
  switch(direction_){
    case Direction::NORTH: y_++; break;
    case Direction::EAST:  x_++; break;
    case Direction::SOUTH: y_--; break;
    case Direction::WEST:  x_--; break;
  }
  checkBounds(x_, y_); // Check new position
  
  // Send the new rat position to the communication interface
  comm_interface_.sendRatPosition(x_, y_, direction_);
}

/**
 * @brief Sets the direction of the rat 90° to the left (counter-clockwise).
 */
void RatPosition::turnLeft(){
  switch(direction_){
    case Direction::NORTH: direction_ = Direction::WEST;  break;
    case Direction::EAST:  direction_ = Direction::NORTH; break;
    case Direction::SOUTH: direction_ = Direction::EAST;  break;
    case Direction::WEST:  direction_ = Direction::SOUTH; break;
  }
  // Send the new rat position to the communication interface
  comm_interface_.sendRatPosition(x_, y_, direction_);
}

/**
 * @brief Sets the direction of the rat 90° to the right (clockwise).
 */
void RatPosition::turnRight(){
  switch(direction_){
    case Direction::NORTH: direction_ = Direction::EAST;  break;
    case Direction::EAST:  direction_ = Direction::SOUTH; break;
    case Direction::SOUTH: direction_ = Direction::WEST;  break;
    case Direction::WEST:  direction_ = Direction::NORTH; break;
  }
  // Send the new rat position to the communication interface
  comm_interface_.sendRatPosition(x_, y_, direction_);
}

/**
 * @brief Sets the direction of the rat to the opposite (180° turn).
 */
void RatPosition::turnAround(){
  switch(direction_){
    case Direction::NORTH: direction_ = Direction::SOUTH; break;
    case Direction::EAST:  direction_ = Direction::WEST;  break;
    case Direction::SOUTH: direction_ = Direction::NORTH; break;
    case Direction::WEST:  direction_ = Direction::EAST;  break;
  }
  // Send the new rat position to the communication interface
  comm_interface_.sendRatPosition(x_, y_, direction_);
}



// --- Internal helpers ---
/**
 * @brief Checks if the given coordinates are within the explored maze bounds (+1).
 * @param x The x-coordinate to check.
 * @param y The y-coordinate to check.
 * @note If the coordinates are out of bounds, a fatal error is triggered. 
 *       The system will enter an infinite loop.
 */
void RatPosition::checkBounds(int16_t x, int16_t y) const{
  if(x < maze_.getMinReachableX() || x > maze_.getMaxReachableX() || y < maze_.getMinReachableY() || y > maze_.getMaxReachableY())
    fatalError("ERROR 'RatPosition::checkBounds': Coordinates (%d, %d) out of bounds of reachable maze (%d, %d, %d, %d)", x, y, maze_.getMinReachableX(), maze_.getMaxReachableX(), maze_.getMinReachableY(), maze_.getMaxReachableY());
  if(x < maze_.getMinExploredX() - 1 || x > maze_.getMaxExploredX() + 1 || y < maze_.getMinExploredY() - 1|| y > maze_.getMaxExploredY() + 1)
    fatalError("ERROR 'RatPosition::checkBounds': Coordinates (%d, %d) out of bounds of explored maze (%d, %d, %d, %d)", x, y, maze_.getMinExploredX(), maze_.getMaxExploredX(), maze_.getMinExploredY(), maze_.getMaxExploredY());
  if(abs(x) > maze_.MAZE_MAX_X || abs(y) > maze_.MAZE_MAX_Y)
    fatalError("ERROR 'RatPosition::checkBounds': Coordinates out of bounds of maze (%d, %d)", x, y);
}