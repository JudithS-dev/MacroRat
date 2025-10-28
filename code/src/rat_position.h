#ifndef __RAT_POSITION_H__
#define __RAT_POSITION_H__

#include <stdint.h>

#include "direction_enum.h" // Direction

// Forward declarations
class Maze;
class CommunicationInterface;


/**
 * @class RatPosition
 * @brief Represents the position and orientation of the rat in the maze.
 *
 * Coordinates are logical coordinates, centered at (0,0).
 */
class RatPosition {
public:
  // === Methods ===
  RatPosition(const Maze &map, CommunicationInterface& comm_interface);
  
  // --- Getters ---
  int16_t getX() const;
  int16_t getY() const;
  Direction getOrientation() const;
  
  // --- Movement ---
  void moveForward();
  void turnLeft();
  void turnRight();
  void turnAround();
  
private:
  // === Attributes ===
  // --- Communication interface ---
  CommunicationInterface& comm_interface_;  ///< Reference to the communication interface for sending wall data
  
  // --- Position and orientation ---
  int16_t x_;            ///< Logical X coordinate (can be negative)
  int16_t y_;            ///< Logical Y coordinate (can be negative)
  Direction direction_;  ///< Current facing direction
  
  // --- Maze reference ---
  const Maze &maze_;     ///< Reference to the maze object (for bounds checking)
  
  // === Methods ===
  // --- Internal helpers ---
  void checkBounds(int16_t x, int16_t y) const;
};

#endif // __RAT_POSITION_H__
