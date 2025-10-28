#ifndef __NAVIGATION_ENGINE_H__
#define __NAVIGATION_ENGINE_H__

#include <stdint.h>
#include "maze_map.h"
#include "movement_action.h"

/**
 * @enum Algorithm
 * @brief Enumeration of algorithms used for exploration and pathfinding.
 */
enum class Algorithm{
  FOLLOW_LEFT_WALL = 0,  ///< Algorithm to follow the left wall
  FLOOD_FILL             ///< Algorithm to use Flood-Fill for pathfinding
};

/**
 * @brief Handles navigation tasks: exploration planning, pathfinding to goals, and route calculation avoiding traps.
 */
class NavigationEngine {
public:
  // === Methods ===
  NavigationEngine(MazeMap& map, Algorithm used_algorithm);
  
  // --- Movement handling ---
  MovementAction getNextMovementForExploration();
  MovementAction getNextMovementToTarget();
  
  // --- Target handling ---
  void setTargetXY(uint16_t target_cell_x, uint16_t target_cell_y);
  bool isTargetReachable();
  bool isTargetReached() const;
  
  // --- Exploration handling ---
  bool isMapComplete();
private:
  // === Constants ===
  static constexpr uint8_t STEP_COST_NORMAL = 1;  ///< Cost of moving into a normal cell
  static constexpr uint8_t STEP_COST_TRAP = 20;   ///< Cost of moving into a trap cell
  
  // === Attributes ===
  // --- Map and Target ---
  MazeMap& map_;
  int16_t target_cell_x_; 
  int16_t target_cell_y_; ///< Target cell coordinates
  
  // --- Algorithm ---
  const Algorithm used_algorithm_ ; ///< Algorithm used for pathfinding
  bool recalculate_route_; ///< Flag to indicate if the route needs to be recalculated
  uint16_t maze_route_[Maze::MAZE_HEIGHT][Maze::MAZE_WIDTH]; ///< Distance map (Flood-Fill result)
  
  // === Methods ===
  // --- Internal helpers ---
  void handleMapChanges();
  MovementAction getTurnAction(Direction from, Direction to);
  
  //--- Simple algorithm implementations ---
  MovementAction getNextMovementFollowLeftWall();
  MovementAction getNextMovementFollowRightWall();
  
  // --- Flood-Fill algorithm implementations ---
  MovementAction getNextMovementForExplorationFloodFill();
  MovementAction getNextMovementToTargetFloodFill();
  
  void calculateFloodFillForExploration();
  void calculateFloodFillToTarget();
  void calculateFloodFillGeneric(int16_t start_x, int16_t start_y, int16_t min_x, int16_t max_x, int16_t min_y, int16_t max_y);
  
  void printFloodFillRoute() const;
};

#endif // __NAVIGATION_ENGINE_H__
