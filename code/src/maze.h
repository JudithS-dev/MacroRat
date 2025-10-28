#ifndef __MAZE_H__
#define __MAZE_H__

#include <stdint.h>

#include "communication_interface.h"

#include "maze_enums.h" // CellState, WallState

/**
 * @class Maze
 * @brief Stores the structure and state of the maze using logical (x, y) coordinates.
 *
 * The Maze class manages cell states and wall information in fixed-size arrays, 
 * centered around (0,0). Negative and positive coordinates are supported.
 *
 * Walls are shared between adjacent cells. For example, the north wall of (0,0) 
 * is the same as the south wall of (0,1).
 * 
 * It tracks explored boundaries, which can be queried to efficiently process 
 * only the known parts of the maze (e.g. for NavigationEngine and CommunicationEngine).
 */
class Maze {
public:
  // === Constants ===
  static constexpr int16_t MAZE_WIDTH  = 15; ///< Total number of columns (must be odd to center around x = 0)
  static constexpr int16_t MAZE_HEIGHT = 15; ///< Total number of rows    (must be odd to center around y = 0)
  
  static constexpr int16_t MAZE_MAX_X = MAZE_WIDTH / 2;  ///< Maximum absolute logical X-coordinate (range: -MAX_X to +MAX_X)
  static constexpr int16_t MAZE_MAX_Y = MAZE_HEIGHT / 2; ///< Maximum absolute logical Y-coordinate (range: -MAX_Y to +MAX_Y)
  
  // === Methods ===
  Maze(CommunicationInterface& comm_interface);
  
  // --- Cell state access ---
  void setCellState(int16_t x, int16_t y, CellState state);
  CellState getCellState(int16_t x, int16_t y) const;
  
  void setUnreachableCells();
  bool hasUnvisitedCells() const;
  
  bool doesCheeseExist(int16_t& cheese_x, int16_t& cheese_y) const;
  
  // --- Wall state access ---
  void setHorizontalWallNorth(int16_t x, int16_t y, WallState state);
  void setHorizontalWallSouth(int16_t x, int16_t y, WallState state);
  void setVerticalWallEast(int16_t x, int16_t y, WallState state);
  void setVerticalWallWest(int16_t x, int16_t y, WallState state);
  
  WallState getHorizontalWallNorth(int16_t x, int16_t y) const;
  WallState getHorizontalWallSouth(int16_t x, int16_t y) const;
  WallState getVerticalWallEast(int16_t x, int16_t y) const;
  WallState getVerticalWallWest(int16_t x, int16_t y) const;
  
  bool isWallNorth(int16_t x, int16_t y) const;
  bool isWallEast(int16_t x, int16_t y) const;
  bool isWallSouth(int16_t x, int16_t y) const;
  bool isWallWest(int16_t x, int16_t y) const;
  bool isWallBetween(int16_t x1, int16_t y1, int16_t x2, int16_t y2) const;
  
  // --- Version tracking ---
  uint8_t getVersion() const;
  bool hasChangedSince(uint8_t oldVersion) const;
  
  // --- Bounds tracking ---
  void checkMazeBounds(int16_t x, int16_t y) const;
  
  int16_t getMinExploredX() const;
  int16_t getMaxExploredX() const;
  int16_t getMinExploredY() const;
  int16_t getMaxExploredY() const;
  
  int16_t getMinReachableX() const;
  int16_t getMaxReachableX() const;
  int16_t getMinReachableY() const;
  int16_t getMaxReachableY() const;
  
  // --- Public helpers ---
  int16_t logical2ArrayX(int16_t x) const;
  int16_t logical2ArrayY(int16_t y) const;
private:
  // === Attributes ===
  // --- Communication interface ---
  CommunicationInterface& comm_interface_;  ///< Reference to the communication interface for sending wall data

  // --- Maze structure ---
  CellState cell_states_[MAZE_HEIGHT][MAZE_WIDTH];           ///< Cell states, initialized to NON_VISITED
  WallState vertical_walls_[MAZE_HEIGHT][MAZE_WIDTH + 1];    ///< west vertical wall of cells, initialized to UNKNOWN/OUTER_WALL
  WallState horizontal_walls_[MAZE_HEIGHT + 1][MAZE_WIDTH];  ///< south horizontal wall of cells, initialized to UNKNOWN/OUTER_WALL
  uint8_t version_counter_;                                  ///< Version counter to track changes (255 is invalid)
  
  // --- Explored area bounds ---
  int16_t min_explored_x_; ///< Minimum explored logical X coordinate
  int16_t max_explored_x_; ///< Maximum explored logical X coordinate
  int16_t min_explored_y_; ///< Minimum explored logical Y coordinate
  int16_t max_explored_y_; ///< Maximum explored logical Y coordinate
  
  // --- Reachable area bounds ---
  int16_t min_reachable_x_; ///< Minimum reachable logical X coordinate
  int16_t max_reachable_x_; ///< Maximum reachable logical X coordinate
  int16_t min_reachable_y_; ///< Minimum reachable logical Y coordinate
  int16_t max_reachable_y_; ///< Maximum reachable logical Y coordinate
  
  // === Methods ===
  // --- Internal helpers ---
  void incrementVersion();
  bool isCellInMaze(int16_t x, int16_t y) const;
  void updateExploredBounds(int16_t x, int16_t y);
};

#endif // __MAZE_H__