#include "navigation_engine.h"

#include <Arduino.h>
#include <queue>

#include "error_handler.h"

/**
 * @brief Initialize the NavigationEngine with a reference to the MazeMap.
 * @param map Pointer to the MazeMap.
 */
NavigationEngine::NavigationEngine(MazeMap& map, Algorithm used_algorithm)
: map_(map),
  target_cell_x_(0),
  target_cell_y_(0),
  used_algorithm_(used_algorithm),
  recalculate_route_(true)
{
  // Initialize maze_route_ with UINT16_MAX
  for(int y = 0; y < static_cast<int>(Maze::MAZE_HEIGHT); ++y){
    for(int x = 0; x < static_cast<int>(Maze::MAZE_WIDTH); ++x){
        maze_route_[y][x] = UINT16_MAX;
    }
  }
  CommunicationInterface::log("      NavigationEngine: Initialization complete!");
}



// --- Movement handling ---
/**
 * @brief Get the next movement action for exploration.
 * @param next_action Reference to store the next movement action.
 * @return True if a valid movement action is found, false if the target is unreachable.
 */
MovementAction NavigationEngine::getNextMovementForExploration(){
  switch(used_algorithm_){
    case Algorithm::FOLLOW_LEFT_WALL:   return getNextMovementFollowLeftWall();
    case Algorithm::FLOOD_FILL:         return getNextMovementForExplorationFloodFill();
    default:  fatalError("ERROR 'NavigationEngine::getNextMovementForExploration': Unknown algorithm.");
              return MovementAction::NO_ACTION;
  }
}

/**
 * @brief Return the next movement direction towards the target cell.
 * 
 * This function checks if the target cell has changed or if the map version has changed.
 * If so, it recalculates the flood fill to find the next movement direction.
 * It also checks if the target cell is reachable.
 * 
 * @param next_direction Reference to store the next movement direction.
 * @return True if a valid movement direction is found, false if the target is unreachable.
 */
MovementAction NavigationEngine::getNextMovementToTarget(){
  switch(used_algorithm_){
    case Algorithm::FOLLOW_LEFT_WALL:   return getNextMovementFollowLeftWall();
    case Algorithm::FLOOD_FILL:         return getNextMovementToTargetFloodFill();
    default:  fatalError("ERROR 'NavigationEngine::getNextMovementForExploration': Unknown algorithm.");
              return MovementAction::NO_ACTION;
  }
}



// --- Target handling ---
/**
 * @brief Set the target cell to navigate towards (e.g., cheese or start).
 * @param target_cell_x logical X coordinate of the target cell.
 * @param target_cell_y logical Y coordinate of the target cell.
 * @note Calls checkBounds and isTargetReachable to ensure the target cell is valid.
 */
void NavigationEngine::setTargetXY(uint16_t target_cell_x, uint16_t target_cell_y){
  map_.maze.checkMazeBounds(target_cell_x, target_cell_y);
  
  if(target_cell_x == target_cell_x_ && target_cell_y == target_cell_y_)
    return; // No change in target
  
  target_cell_x_ = target_cell_x;
  target_cell_y_ = target_cell_y;
  recalculate_route_ = true;
  
  if(!isTargetReachable())
    fatalError("ERROR 'NavigationEngine::setTarget': Target cell (%d, %d) is unreachable.", target_cell_x_, target_cell_y_);
}

/**
 * @brief Check if the target cell is reachable (not unreachable).
 * @return True if the target cell is reachable, false otherwise.
 * @note This function checks the maze version and sets unreachable cells if necessary.
 */
bool NavigationEngine::isTargetReachable(){
  handleMapChanges();
  
  if(map_.maze.getCellState(target_cell_x_, target_cell_y_) == CellState::UNREACHABLE)
    return false;
  return true;
}

/**
 * @brief Check if the robot has already reached the current target cell.
 * @return True if the target is reached.
 */
bool NavigationEngine::isTargetReached() const{
  if(target_cell_x_ == map_.rat_pos.getX() && target_cell_y_ == map_.rat_pos.getY()){
    return true;
  }
  return false;
}



//--- Exploration handling ---
/**
 * @brief Check if the exploration is complete (all reachable cells visited).
 * @return True if exploration is complete.
 * @note This function checks the maze version and sets unreachable cells if necessary.
 */
bool NavigationEngine::isMapComplete(){
  handleMapChanges();
  return !map_.maze.hasUnvisitedCells();
}



// --- Internal helpers ---
/**
 * @brief Handle changes in the maze map (e.g. new walls, traps, cheese).
 * 
 * This function checks if the maze version has changed since the last calculation.
 * If so, it sets unreachable cells and updates the saved maze version.
 * It also sets the recalculate_route_ flag to true.
 */
void NavigationEngine::handleMapChanges(){
  static uint8_t saved_map_version_ = 255; // Initialize value to an invalid version
  
  // Check if the maze version has changed
  if(map_.maze.hasChangedSince(saved_map_version_)){
    map_.maze.setUnreachableCells();
    saved_map_version_ = map_.maze.getVersion();
    recalculate_route_ = true;
  }
}

/**
 * @brief Get the turn action to change direction from one direction to another.
 * 
 * This function calculates the turn action needed to change from one direction to another.
 * It uses a switch-case structure to determine the correct turn action based on the current
 * and target directions.
 * 
 * @param from The current direction.
 * @param to The target direction.
 * @return The movement action representing the turn.
 */
MovementAction NavigationEngine::getTurnAction(Direction from, Direction to){
  if(from == to)
    return MovementAction::NO_ACTION; // No turn needed
  
  switch(from){
    case Direction::NORTH:
      switch(to){
        case Direction::EAST:  return MovementAction::TURN_RIGHT_90;
        case Direction::SOUTH: return MovementAction::TURN_RIGHT_180;
        case Direction::WEST:  return MovementAction::TURN_LEFT_90;
        default: fatalError("ERROR 'NavigationEngine::getTurnAction': Invalid target direction.");
      }
    case Direction::EAST:
      switch(to){
        case Direction::NORTH: return MovementAction::TURN_LEFT_90;
        case Direction::SOUTH: return MovementAction::TURN_RIGHT_90;
        case Direction::WEST:  return MovementAction::TURN_RIGHT_180;
        default: fatalError("ERROR 'NavigationEngine::getTurnAction': Invalid target direction.");
      }
    case Direction::SOUTH:
      switch(to){
        case Direction::NORTH: return MovementAction::TURN_RIGHT_180;
        case Direction::EAST:  return MovementAction::TURN_LEFT_90;
        case Direction::WEST:  return MovementAction::TURN_RIGHT_90;
        default: fatalError("ERROR 'NavigationEngine::getTurnAction': Invalid target direction.");
      }
    case Direction::WEST:
      switch(to){
        case Direction::NORTH: return MovementAction::TURN_RIGHT_90;
        case Direction::EAST:  return MovementAction::TURN_RIGHT_180;
        case Direction::SOUTH: return MovementAction::TURN_LEFT_90;
        default: fatalError("ERROR 'NavigationEngine::getTurnAction': Invalid target direction.");
      }
    default:
      fatalError("ERROR 'NavigationEngine::getTurnAction': Invalid current direction.");
  }
  return MovementAction::NO_ACTION; // This line will never be reached
}



//--- Simple algorithm implementations ---
/**
 * @brief Implementation of the left wall-following algorithm.
 * 
 * This function determines the next movement action based on the principle of always
 * keeping the left wall close to the rat. It handles three main cases:
 * 
 * 1. **Initial Exploration**: If the rat has not yet detected any walls in its surroundings,
 *    it will move straight ahead, until it encounters the first wall on any side.
 * 
 * 2. **Initial Wall Encounter**: Once a wall is detected (at any side), the rat will
 *    perform an initial orientation such that the detected wall is on its left side.
 *    This orientation is only executed once. After that, the algorithm will use the third case.
 * 
 * 3. **Following the Left Wall**: The rat will continuously check its surrounding walls and  
 *    it follows the left wall by adjusting its movement direction accordingly.
 *    - If there is no left wall, the rat turns left (once) and then moves forward in the next step.
 *    - If there is a left wall but no front wall, the rat moves forward.
 *    - If there is a left wall and a front wall but no right wall, the rat turns right.
 *    - If there is a left wall, a front wall, and a right wall but no back wall, the rat turns around.
 *    - If there are walls on all sides, the rat cannot move and will not perform any action.
 */
MovementAction NavigationEngine::getNextMovementFollowLeftWall(){
  // --- Check if the target cell is reachable ---
  if(!isTargetReachable())
    return MovementAction::NO_ACTION; // Target is not reachable
  
  // --- Get information about the current cell ---
  int16_t current_cell_x = map_.rat_pos.getX();
  int16_t current_cell_y = map_.rat_pos.getY();
  Direction current_direction = map_.rat_pos.getOrientation();
  bool wall_left = map_.isWallLeft();
  bool wall_front = map_.isWallFront();
  bool wall_right = map_.isWallRight();
  bool wall_back = map_.isWallBack();
  
  // --- Follow the left wall algorithm ---
  static bool following_left_wall = false;  ///< Flag to indicate if the left wall is being followed
  
  // === Step 1: If at start no wall is found, move straight ahead ===
  if(!following_left_wall){
    if(!wall_left && !wall_front && !wall_right && !wall_back){
      return MovementAction::MOVE_FORWARD;
    }
  }
  
  // === Step 2: If at start a wall is found, orientate to the left ===
  if(!following_left_wall){
    if(wall_left){ // Walls: left
      following_left_wall = true; 
      // just use the algorithm below (Step 3)
    } else if(wall_front){
      if(wall_right){
        if(wall_back){ // Walls: top, right, bottom
          following_left_wall = true;
          return MovementAction::TURN_LEFT_90;
        } else{ //Walls: top, right
          following_left_wall = true;
          return MovementAction::TURN_RIGHT_180;
        }
      } else{
        if(wall_back){ // Walls: top, bottom
          following_left_wall = true;
          return MovementAction::TURN_LEFT_90;
        } else{ //Walls top
          following_left_wall = true;
          return MovementAction::TURN_RIGHT_90;
        }
      }
    } else if(wall_right){
      if(wall_back){ // wall right and back
        following_left_wall = true;
        return MovementAction::TURN_LEFT_90;
      } else{ // wall right
        following_left_wall = true;
        return MovementAction::TURN_RIGHT_180;
      }
    } else if(wall_back){ // wall back
      following_left_wall = true;
      return MovementAction::TURN_LEFT_90;
    } else{ 
      fatalError("ERROR 'NavigationEngine::getNextMovementFollowLeftWall': this should not happen.");
    }
  }
  
  // === Step 3: Follow the left wall ===
  if(following_left_wall){
    static bool was_turning = false; // Flag to indicate if the last movement was a left turn
    if(!wall_left){ // walls: no left
      if(was_turning){
        was_turning = false;
        return MovementAction::MOVE_FORWARD;
      } else{
        was_turning = true;
        return MovementAction::TURN_LEFT_90;
      }
    } else if(!wall_front){ // walls: left, no front
      was_turning = false;
      return MovementAction::MOVE_FORWARD;
    } else if(!wall_right){ // walls: left, front, no right
      was_turning = false;
      return MovementAction::TURN_RIGHT_90;
    } else if(!wall_back){ // walls: left, front, right, no back
      was_turning = false;
      return MovementAction::TURN_RIGHT_180;
    } else{ // walls: left, front, right, back
      was_turning = false;
      return MovementAction::NO_ACTION; // No movement possible
    }
  }
  
  fatalError("ERROR 'NavigationEngine::getNextMovementFollowLeftWall': this should not happen.");
  return MovementAction::NO_ACTION; // Line will never be reached
}



// --- Flood-Fill algorithm implementations ---
/**
 * @brief Determines the next movement action during maze exploration using the Flood-Fill algorithm.
 * 
 * This function selects the nearest unvisited (NON_VISITED) cell based on the current flood-fill cost map.
 * It uses a static flag to avoid unnecessary recalculations if the current target is still valid.
 * 
 * Steps:
 *  - Checks if the map has changed (via handleMapChanges()) and triggers a route recalculation if needed.
 *  - Recalculates the flood-fill distances from the rat position if necessary.
 *  - If a new exploration target is needed (due to map update or goal reached), finds the best unvisited cell.
 *  - Sets the selected cell as the new exploration target.
 *  - Delegates the movement decision to getNextMovementToTargetFloodFill() using the updated target.
 * 
 * @return MovementAction to execute (e.g., MOVE_FORWARD, TURN_LEFT_90, etc.)
 *         or NO_ACTION if no unvisited cells remain (handled as fatalError for now).
 */
MovementAction NavigationEngine::getNextMovementForExplorationFloodFill(){
  // --- Flag to indicate if a new exploration target should be found ---
  static bool find_new_target = true; // Flag to indicate if a new target should be found
  
  // --- Check if map has changed ---
  handleMapChanges();
  
  // --- Check if the route needs to be recalculated ---
  if(recalculate_route_){
    calculateFloodFillForExploration(); // Calculate the Flood-Fill distancfes
    recalculate_route_ = false; // Reset the flag
    find_new_target = true; // Set the flag to find a new target
  }
  
  // --- If a new target should be found, search for the best unvisited cell ---
  if(find_new_target || isTargetReached()){
    find_new_target = false; // Reset the flag
    
    // --- Search for the best unvisited cell in the explored area + 1 ---
    int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X, map_.maze.getMinExploredX() - 1);
    int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X, map_.maze.getMaxExploredX() + 1);
    int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y, map_.maze.getMinExploredY() - 1);
    int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y, map_.maze.getMaxExploredY() + 1);
    
    uint16_t best_cost = UINT16_MAX;
    int16_t best_x = map_.rat_pos.getX();
    int16_t best_y = map_.rat_pos.getY();
    
    for(int16_t y = min_y; y <= max_y; y++){
      for(int16_t x = min_x; x <= max_x; x++){
        if(map_.maze.getCellState(x, y) != CellState::NON_VISITED)
          continue;
        
        uint16_t cost = maze_route_[map_.maze.logical2ArrayY(y)][map_.maze.logical2ArrayX(x)];
        if (cost < best_cost){
          best_cost = cost;
          best_x = x;
          best_y = y;
        }
      }
    }
    
    // Check if a best unvisited cell was found, should be at least one cell, because the rat is still in exploration mode
    if(best_cost == UINT16_MAX)
      fatalError("ERROR 'NavigationEngine::getNextMovementForExplorationFloodFill': No unvisited cells found in the explored area.");
    
    // --- Set the target cell to the best/nearest unvisited cell found ---
    target_cell_x_ = best_x;
    target_cell_y_ = best_y;
    recalculate_route_ = true; // Reset flag to recalculate route with getNextMovementToTargetFloodFill()
  }
  return getNextMovementToTargetFloodFill();
}

/**
 * @brief Determines the next movement action to reach the currently set target cell using Flood-Fill pathfinding.
 * 
 * This function uses precomputed flood-fill distances (from the target to all reachable cells)
 * to select the best neighboring cell from the rat's current position. It chooses the neighbor
 * with the lowest cost and returns the appropriate MovementAction to get closer to the target.
 * 
 * Steps:
 *  - Checks if the target cell is still reachable; returns NO_ACTION if not.
 *  - Recalculates the Flood-Fill cost map from the target if flagged.
 *  - Evaluates all four possible movement directions (N, E, S, W) from the current rat position.
 *  - Ignores directions blocked by walls or out-of-bounds.
 *  - Selects the direction with the lowest cost in the maze_route_ array.
 *  - Compares the best direction to the rat's current orientation and returns:
 *      - MOVE_FORWARD if already facing the best direction,
 *      - a TURN_* action otherwise (via getTurnAction()).
 * 
 * @return MovementAction to reduce distance to the target (or NO_ACTION if stuck or already at best cell).
 */
MovementAction NavigationEngine::getNextMovementToTargetFloodFill(){
  // --- Check if the target cell is reachable ---
  if(!isTargetReachable())
    return MovementAction::NO_ACTION; // Target is not reachable
  
  // --- Check if the route needs to be recalculated ---
  if(recalculate_route_){
    calculateFloodFillToTarget(); // Calculate the Flood-Fill distances
    recalculate_route_ = false; // Reset the flag
  }
  
  // --- Check which direction to move based on the Flood-Fill distances ---
  // Get the current rat position
  int16_t rat_x = map_.rat_pos.getX();
  int16_t rat_y = map_.rat_pos.getY();
  
  int16_t best_x = rat_x; // Initialize best x to rat x
  int16_t best_y = rat_y; // Initialize best y to rat y
  int16_t min_cost = maze_route_[map_.maze.logical2ArrayY(rat_y)][map_.maze.logical2ArrayX(rat_x)]; // Initialize minimum cost to maximum value
  Direction next_direction = Direction::NORTH; // Default direction (will be updated)
  
  // Direction vectors for moving in the maze (north, east, south, west)
  const int16_t dx[4] = {0, 1,  0, -1}; // Change in x for each direction
  const int16_t dy[4] = {1, 0, -1,  0}; // Change in y for each direction
  
  for(int dir = 0; dir < 4; dir++){
    int16_t next_x = rat_x + dx[dir];
    int16_t next_y = rat_y + dy[dir];
    
    // Check if the next cell is within the bounds of the maze
    if(next_x < -Maze::MAZE_MAX_X || next_x > Maze::MAZE_MAX_X || next_y < -Maze::MAZE_MAX_Y || next_y > Maze::MAZE_MAX_Y)
      continue; // Skip if out of bounds
    
    // Check if there is a wall between the current cell and the next cell
    if(map_.maze.isWallBetween(rat_x, rat_y, next_x, next_y))
      continue; // Skip if there is a wall
    
    // Get the cost of moving to the next cell
    uint16_t cost = maze_route_[map_.maze.logical2ArrayY(next_y)][map_.maze.logical2ArrayX(next_x)];
    
    // If this direction has a lower cost, update the minimum cost and action
    if(cost < min_cost){
      min_cost = cost;
      best_x = next_x; // Update best x
      best_y = next_y; // Update best y
      switch(dir){ // rat must move in opposite direction of route
        case 0: next_direction = Direction::NORTH; break;
        case 1: next_direction = Direction::EAST;  break;
        case 2: next_direction = Direction::SOUTH; break;
        case 3: next_direction = Direction::WEST;  break;
      }
    }
  }
  
  // --- Convert the best direction to a movement action ---
  if(best_x == rat_x && best_y == rat_y){
    return MovementAction::NO_ACTION; // No movement needed, can't get to better cell
  }
  // Determine the movement action based on current direction and best direction
  Direction current_direction = map_.rat_pos.getOrientation();
  if(next_direction == current_direction){
    return MovementAction::MOVE_FORWARD; // Move forward in the current direction
  }
  // Get the turn action to change from current direction to next direction
  return getTurnAction(current_direction, next_direction);
}



/**
 * @brief Calculates Flood-Fill distances from the rat position to all reachable cells
 *        in the explored maze area (plus 1 margin).
 * 
 * This function sets the bounds for the flood fill to the explored area plus a margin of 1 cell.
 * It then calls the generic Flood-Fill algorithm with the rat position as the start point.
 */
void NavigationEngine::calculateFloodFillForExploration(){
  // Set the bounds to for flood fill (inside maze bounds, but only to explored area + 1)
  int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X,  map_.maze.getMinExploredX() - 1);
  int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X,  map_.maze.getMaxExploredX() + 1);
  int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y,  map_.maze.getMinExploredY() - 1);
  int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y,  map_.maze.getMaxExploredY() + 1);
  
  // Call the generic Flood-Fill algorithm with the rat position as start point
  int16_t rat_x = map_.rat_pos.getX();
  int16_t rat_y = map_.rat_pos.getY();
  calculateFloodFillGeneric(rat_x, rat_y, min_x, max_x, min_y, max_y);
}

/**
 * @brief Calculates Flood-Fill distances from the target cell to all reachable cells
 *        in the explored maze area (plus 1 margin).
 * 
 * This function sets the bounds for the flood fill to the explored area plus a margin of 1 cell 
 * or to the target cell coordinates.
 * It then calls the generic Flood-Fill algorithm with the target cell as the start point.
 */
void NavigationEngine::calculateFloodFillToTarget(){
  // Set the bounds to for flood fill (inside maze bounds, but only to explored area + 1 or target cell)
  int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X,  std::min<int16_t>(map_.maze.getMinExploredX() - 1, target_cell_x_));
  int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X,  std::max<int16_t>(map_.maze.getMaxExploredX() + 1, target_cell_x_));
  int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y,  std::min<int16_t>(map_.maze.getMinExploredY() - 1, target_cell_y_));
  int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y,  std::max<int16_t>(map_.maze.getMaxExploredY() + 1, target_cell_y_));
  
  // Call the generic Flood-Fill algorithm with the target cell as start point
  calculateFloodFillGeneric(target_cell_x_, target_cell_y_, min_x, max_x, min_y, max_y);
}

/**
 * @brief Performs a generic Flood-Fill algorithm from a given start cell over a bounded area of the maze.
 * 
 * This function calculates the minimum cost to reach each cell in the specified bounding box
 * starting from (start_x, start_y). Costs are stored in the internal maze_route_ array.
 * It respects walls and unreachable cells, and assigns higher step costs to TRAP cells.
 * 
 * @param start_x   Logical x-coordinate of the start cell
 * @param start_y   Logical y-coordinate of the start cell
 * @param min_x     Minimum logical x bound to consider
 * @param max_x     Maximum logical x bound to consider
 * @param min_y     Minimum logical y bound to consider
 * @param max_y     Maximum logical y bound to consider
 */
void NavigationEngine::calculateFloodFillGeneric(int16_t start_x, int16_t start_y, int16_t min_x, int16_t max_x, int16_t min_y, int16_t max_y){
  // Set all cells in bounds to UINT16_MAX (unreachable)
  for(int16_t y = min_y; y <= max_y; y++){
    for(int16_t x = min_x; x <= max_x; x++){
      maze_route_[map_.maze.logical2ArrayY(y)][map_.maze.logical2ArrayX(x)] = UINT16_MAX;
    }
  }
  
  // Initialize the start of Flood-Fill algorithm to lowest value (0)
  maze_route_[map_.maze.logical2ArrayY(start_y)][map_.maze.logical2ArrayX(start_x)] = 0;
  
  // Create a queue for Flood-Fill algorithm
  std::queue<std::pair<int16_t, int16_t>> queue; // Queue for cells to check in the flood fill
  queue.push(std::make_pair(start_x, start_y));  // Start from the start cell
  
  // Direction vectors for moving in the maze (north, east, south, west)
  // These vectors represent the change in x and y coordinates for each direction
  const int16_t dx[4] = {0, 1,  0, -1};
  const int16_t dy[4] = {1, 0, -1,  0};
  
  // Start flood fill algorithm
  uint16_t cur_cost, step_cost, new_cost;
  while(!queue.empty()){
    auto cell = queue.front();
    queue.pop();
    int16_t cur_x = cell.first;
    int16_t cur_y = cell.second;
    
    cur_cost = maze_route_[map_.maze.logical2ArrayY(cur_y)][map_.maze.logical2ArrayX(cur_x)];
    
    // Check if neighboring cells are within bounds and are reachable from the current cell
    for(int dir = 0; dir < 4; dir++){
      int16_t next_x = cur_x + dx[dir];
      int16_t next_y = cur_y + dy[dir];
      
      // Check if the next cell is within the bounds of flood fill (and in maze bounds)
      if(next_x < min_x || next_x > max_x || next_y < min_y || next_y > max_y)
        continue;
      
      // Check if there is a wall between the current cell and the next cell
      if(map_.maze.isWallBetween(cur_x, cur_y, next_x, next_y))
        continue; // Skip if there is a wall
      
      // Set step cost based on the cell state
      if(map_.maze.getCellState(next_x, next_y) == CellState::UNREACHABLE)
        fatalError("ERROR 'NavigationEngine::calculateFloodFillToTarget': Unreachable cell found in flood fill.");
      if(map_.maze.getCellState(next_x, next_y) == CellState::TRAP)
        step_cost = STEP_COST_TRAP; // High cost for traps
      else
        step_cost = STEP_COST_NORMAL; // Normal cost for empty cells
      
      new_cost = cur_cost + step_cost;
      
      // Set the new cost, if it is lower than the current cost
      int16_t next_ix = map_.maze.logical2ArrayX(next_x);
      int16_t next_iy = map_.maze.logical2ArrayY(next_y);
      if(new_cost < maze_route_[next_iy][next_ix]){
        maze_route_[next_iy][next_ix] = new_cost;
        // Add the next cell to the queue for further processing (to check if other values have changed)
        queue.push({next_x, next_y});
      }
    }
  }
}

/**
 * @brief Prints the calculated flood fill maze (including walls) to the serial console.
 * 
 * 
 * 
 * The walls are represented as follows:
 * - WALL: (horizontal) --- or (vertical) |
 * - OUTER_WALL: (horizontal) ~~~ or (vertical) \
 * - UNKNOWN: ?
 * - NO_WALL: (space)
 *  
 */
void NavigationEngine::printFloodFillRoute() const{

  int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X,  std::min<int16_t>(map_.maze.getMinExploredX() - 1, target_cell_x_));
  int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X,  std::max<int16_t>(map_.maze.getMaxExploredX() + 1, target_cell_x_));
  int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y,  std::min<int16_t>(map_.maze.getMinExploredY() - 1, target_cell_y_));
  int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y,  std::max<int16_t>(map_.maze.getMaxExploredY() + 1, target_cell_y_));

  uint16_t current_value;
  WallState current_wall = WallState::UNKNOWN;
  // From the top to the bottom, so y decreases
  for(int16_t y = max_y; y >= min_y; y--){
    // Top of the row: north walls
    for(int16_t x = min_x; x <= max_x; x++){
      current_wall = map_.maze.getHorizontalWallNorth(x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.printf("+---"); break;
        case WallState::OUTER_WALL: Serial.printf("+~~~"); break;
        case WallState::UNKNOWN:    Serial.printf("+ ? "); break;
        case WallState::NO_WALL:    Serial.printf("+   "); break;
        default:                    fatalError("ERROR 'NavigationEngine::printFloodFillRoute': Unknown wall state %d", (int)current_wall);
      }
    }
    Serial.printf("+\n"); // End of the row
    
    // Content of the row: walls and values
    for(int16_t x = min_x; x <= max_x; x++){
      current_wall = map_.maze.getVerticalWallWest(x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.printf("|");  break;
        case WallState::OUTER_WALL: Serial.printf("\\"); break;
        case WallState::UNKNOWN:    Serial.printf("?");  break;
        case WallState::NO_WALL:    Serial.printf(" ");  break;
        default:                    fatalError("ERROR 'NavigationEngine::printFloodFillRoute': Unknown wall state %d", (int)current_wall);
      }
      
      // Get current cell value
      current_value = maze_route_[map_.maze.logical2ArrayY(y)][map_.maze.logical2ArrayX(x)];
      
      if(current_value == UINT16_MAX){ // Unreachable cell
        Serial.printf(" X ");
      } else if(current_value < 10){ // Single digit value
        Serial.printf(" %d ", current_value);
      } else if(current_value <100){ // Two digit value
        Serial.printf("%2d ", current_value);
      } else{ 
        Serial.printf("%3d", current_value);
      }
    }
    // right wall of the row
    current_wall = map_.maze.getVerticalWallEast(max_x, y);
    switch(current_wall){
      case WallState::WALL:       Serial.printf("|");  break;
      case WallState::OUTER_WALL: Serial.printf("\\"); break;
      case WallState::UNKNOWN:    Serial.printf("?");  break;
      case WallState::NO_WALL:    Serial.printf(" ");  break;
      default:                    fatalError("ERROR 'NavigationEngine::printFloodFillRoute': Unknown wall state %d", (int)current_wall);
    }
    Serial.printf("\n");
  }
  
  // Bottom of the maze: south walls
  for(int16_t x = min_x; x <= max_x; x++){
    current_wall = map_.maze.getHorizontalWallSouth(x, min_y);
    switch(current_wall){
      case WallState::WALL:       Serial.printf("+---"); break;
      case WallState::OUTER_WALL: Serial.printf("+~~~"); break;
      case WallState::UNKNOWN:    Serial.printf("+ ? "); break;
      case WallState::NO_WALL:    Serial.printf("+   "); break;
      default:                    fatalError("ERROR 'NavigationEngine::printFloodFillRoute': Unknown wall state %d", (int)current_wall);
    }
  }
  Serial.printf("+\n"); // End of the row
}