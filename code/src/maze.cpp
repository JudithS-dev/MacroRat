#include "maze.h"

#include <Arduino.h>
#include <stdlib.h>
#include <queue>

#include "maze_enums.h"
#include "error_handler.h"

/**
 * @brief Constructs a Maze object and initializes its state.
 *        All cells are set to NON_VISITED, 
 *        inner walls to UNKNOWN, and outer walls to OUTER_WALL.
 */
Maze::Maze(CommunicationInterface& comm_interface)
: comm_interface_(comm_interface),
  version_counter_(0),
  min_explored_x_(0), // Initialize explored area bounds (starting at (0,0))
  max_explored_x_(0),
  min_explored_y_(0),
  max_explored_y_(0),
  min_reachable_x_(-MAZE_MAX_X), // Initialize reachable area bounds (whole maze)
  max_reachable_x_(MAZE_MAX_X),
  min_reachable_y_(-MAZE_MAX_Y),
  max_reachable_y_(MAZE_MAX_Y)
{
  // --- Initialize cell states to NON_VISITED ---
  for(uint16_t row = 0; row < MAZE_HEIGHT; row++){
    for(uint16_t col = 0; col < MAZE_WIDTH; col++){
      cell_states_[row][col] = CellState::NON_VISITED;
    }
  }
  
  // --- Initialize all wall states to UNKNOWN ---
  for(uint16_t row = 0; row < MAZE_HEIGHT; row++){
    for(uint16_t col = 0; col < MAZE_WIDTH + 1; col++){
      vertical_walls_[row][col] = WallState::UNKNOWN;
    }
  }
  
  for(uint16_t row = 0; row < MAZE_HEIGHT + 1; row++){
    for(uint16_t col = 0; col < MAZE_WIDTH; col++){
      horizontal_walls_[row][col] = WallState::UNKNOWN;
    }
  }
  // --- Set outer walls to OUTER_WALL ---
  // Set top horizontal walls
  for(int16_t x = -MAZE_MAX_X; x <= MAZE_MAX_X; x++)
    horizontal_walls_[logical2ArrayY(MAZE_MAX_Y + 1)][logical2ArrayX(x)] = WallState::OUTER_WALL;
  
  // Set bottom horizontal walls
  for(int16_t x = -MAZE_MAX_X; x <= MAZE_MAX_X; x++)
    horizontal_walls_[0][logical2ArrayX(x)] = WallState::OUTER_WALL;   
  
  // Set left vertical walls
  for(int16_t y = -MAZE_MAX_Y; y <= MAZE_MAX_Y; y++)
    vertical_walls_[logical2ArrayY(y)][0] = WallState::OUTER_WALL;
  
  // Set right vertical walls
  for(int16_t y = -MAZE_MAX_Y; y <= MAZE_MAX_Y; y++)
    vertical_walls_[logical2ArrayY(y)][logical2ArrayX(MAZE_MAX_X + 1)] = WallState::OUTER_WALL;
  
  CommunicationInterface::log("                  Maze: Initialization complete!");
}



// --- Cell state access ---
/**
 * @brief Sets the state of a cell in the maze and updates explored bounds.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @param state The new state of the cell.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
void Maze::setCellState(int16_t x, int16_t y, CellState state){
  checkMazeBounds(x, y);
  cell_states_[logical2ArrayY(y)][logical2ArrayX(x)] = state;
  incrementVersion();
  
  // only update explored bounds if the cell is not UNREACHABLE => not from function setUnreachableCells()
  if(state != CellState::UNREACHABLE)
    updateExploredBounds(x, y);
  
  // Send the cell state to the communication interface
  switch(state){
    case CellState::NON_VISITED:
      fatalError("ERROR 'Maze::setCellState': Cell state should not be set to NON_VISITED.");
      break;
    case CellState::UNREACHABLE:
      // No need to send UNREACHABLE state to the communication interface, it won't be displayed
      break;
    case CellState::EMPTY:
      // No need to send EMPTY state to the communication interface, it is the default state in the web interface
      break;
    case CellState::TRAP:
      comm_interface_.sendTrap(x, y);
      break;
    case CellState::CHEESE:
      comm_interface_.sendCheese(x, y);
      break;
    default:
      fatalError("ERROR 'Maze::setCellState': Invalid cell state %d for cell (%d, %d)", (int)state, x, y);
      break;
  }
}

/**
 * @brief Gets the state of a cell in the maze.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return The state of the cell.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
CellState Maze::getCellState(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  return cell_states_[logical2ArrayY(y)][logical2ArrayX(x)];
}

/**
 * @brief Updates the maze by marking all unreachable cells as UNREACHABLE.
 * 
 * Checks if there are cells in the maze that are not reachable from the starting point (0,0).
 * If a cell is unreachable, it is marked as UNREACHABLE.
 * Sets min_reachable_x_, max_reachable_x_, min_reachable_y_, and max_reachable_y_ to the bounds of the reachable area.
 */
void Maze::setUnreachableCells(){
  // Create a array to save which cells where already checked
  bool checked_cells[MAZE_HEIGHT][MAZE_WIDTH] = {false}; // Initialize all cells as unchecked
  
  // Create a queue for Breadth-First Search (BFS)
  std::queue<std::pair<int16_t, int16_t>> queue; // Queue for cells to check if any of its neighbors are reachable
  queue.push(std::make_pair(0, 0)); // Start from the center of the maze
  
  // Set the bounds to check for unreachable cells to explored bounds + 1 (and make sure they are within bounds)
  int16_t min_x, max_x, min_y, max_y;
  min_x = std::max<int16_t>(-MAZE_MAX_X, min_explored_x_ - 1);
  max_x = std::min<int16_t>( MAZE_MAX_X, max_explored_x_ + 1);
  min_y = std::max<int16_t>(-MAZE_MAX_Y, min_explored_y_ - 1);
  max_y = std::min<int16_t>( MAZE_MAX_Y, max_explored_y_ + 1);
  
  // reset reachable bounds
  min_reachable_x_ = MAZE_MAX_X;
  max_reachable_x_ = -MAZE_MAX_X;
  min_reachable_y_ = MAZE_MAX_Y;
  max_reachable_y_ = -MAZE_MAX_Y;
  
  while(!queue.empty()){
    auto cell = queue.front();
    queue.pop();
    int16_t x = cell.first;
    int16_t y = cell.second;
    
    // Skip if already checked or out of bounds
    if(!isCellInMaze(x, y) || checked_cells[logical2ArrayY(y)][logical2ArrayX(x)] || x < min_x || x > max_x || y < min_y || y > max_y)
      continue;
    
    // Mark the cell as checked
    checked_cells[logical2ArrayY(y)][logical2ArrayX(x)] = true;
    
    // Update reachable bounds
    if(x < min_reachable_x_) min_reachable_x_ = x;
    if(x > max_reachable_x_) max_reachable_x_ = x;
    if(y < min_reachable_y_) min_reachable_y_ = y;
    if(y > max_reachable_y_) max_reachable_y_ = y;
    
    // Add neighbors to the queue
    if(!isWallNorth(x, y))  queue.push({x,     y + 1});
    if(!isWallEast(x, y))   queue.push({x + 1, y    });
    if(!isWallSouth(x, y))  queue.push({x,     y - 1});
    if(!isWallWest(x, y))   queue.push({x - 1, y    });
  }
  
  // Set all unchecked cells to UNREACHABLE
  uint8_t old_version = version_counter_;
  bool found_unreachable = false;
  
  for(int16_t y = min_y; y <= max_y; y++){
    for(int16_t x = min_x; x <= max_x; x++){
      if(!checked_cells[logical2ArrayY(y)][logical2ArrayX(x)]){
        // Check current state of the cell
        CellState current_state = getCellState(x, y);
        if(current_state == CellState::NON_VISITED){  // cell is NON_VISITED, mark it as UNREACHABLE
          setCellState(x, y, CellState::UNREACHABLE);
          found_unreachable = true;
        } else if(current_state == CellState::UNREACHABLE){  // already marked as UNREACHABLE
          // do nothing
        } else{
          // If the cell is EMPTY, TRAP, or CHEESE, it should not be UNREACHABLE
          fatalError("ERROR 'Maze::setUnreachableCells': Cell (%d, %d) is not reachable but has a state other than NON_VISITED or UNREACHABLE (%d)", x, y, (int)current_state);
        }
      }
    }
  }

  if(found_unreachable){
    version_counter_ = old_version + 1; // Increment version only once if there are unreachable cells
    if(version_counter_ > 254)
      version_counter_ = 0; // Reset version counter if it exceeds 254
  }
  
  // If BFS reached the edges of the inspected area, expand reachable bounds to full maze area
  min_reachable_x_ = (min_reachable_x_ <= min_x) ? -MAZE_MAX_X : min_reachable_x_;
  max_reachable_x_ = (max_reachable_x_ >= max_x) ?  MAZE_MAX_X : max_reachable_x_;
  min_reachable_y_ = (min_reachable_y_ <= min_y) ? -MAZE_MAX_Y : min_reachable_y_;
  max_reachable_y_ = (max_reachable_y_ >= max_y) ?  MAZE_MAX_Y : max_reachable_y_;
}

/**
 * @brief Checks if there are any unvisited cells in the maze (explored bounds + 1).
 * @return True if there are unvisited cells, false otherwise.
 * @warning This function does not call setUnreachableCells().
 * It may return true even if the maze is fully explored.
 * It is recommended to call setUnreachableCells() before using this function.
 */
bool Maze::hasUnvisitedCells() const{
  // Set the bounds to check for unvisited cells to explored bounds + 1 (and make sure they are within bounds)
  int16_t min_x = std::max<int16_t>(-MAZE_MAX_X, min_explored_x_ - 1);
  int16_t max_x = std::min<int16_t>( MAZE_MAX_X, max_explored_x_ + 1);
  int16_t min_y = std::max<int16_t>(-MAZE_MAX_Y, min_explored_y_ - 1);
  int16_t max_y = std::min<int16_t>( MAZE_MAX_Y, max_explored_y_ + 1);
  
  // Check if there are any unvisited cells in the bounds
  for(int16_t y = min_y; y <= max_y; y++){
    for(int16_t x = min_x; x <= max_x; x++){
      if(getCellState(x, y) == CellState::NON_VISITED){
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Checks if there is cheese in the maze (explored bounds + 1).
 * @param[out] cheese_x The x-coordinate of the cheese if found.
 * @param[out] cheese_y The y-coordinate of the cheese if found.
 * @return True if cheese exists, false otherwise.
 */
bool Maze::doesCheeseExist(int16_t& cheese_x, int16_t& cheese_y) const{
  // Check if the min/max reachable bounds are "more"/"less" than the explored bounds + 1 and inside the maze bounds
  int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X,  std::min<int16_t>(min_reachable_x_, min_explored_x_ - 1));
  int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X,  std::max<int16_t>(max_reachable_x_, max_explored_x_ + 1));
  int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y,  std::min<int16_t>(min_reachable_y_, min_explored_y_ - 1));
  int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y,  std::max<int16_t>(max_reachable_y_, max_explored_y_ + 1));
  
  for(int16_t y = min_y; y <= max_y; y++){
    for(int16_t x = min_x; x <= max_x; x++){
      if(getCellState(x, y) == CellState::CHEESE){
        cheese_x = x;
        cheese_y = y;
        return true;
      }
    }
  }
  return false;
}



// --- Wall state access ---
/**
 * @brief Sets the state of the horizontal wall to the north of a cell and updates explored bounds.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @param state The new state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
void Maze::setHorizontalWallNorth(int16_t x, int16_t y, WallState state){
  checkMazeBounds(x, y);
  horizontal_walls_[logical2ArrayY(y + 1)][logical2ArrayX(x)] = state;
  incrementVersion();
  updateExploredBounds(x, y);
  
  // Send wall data to the communication interface
  // Note: The datastructure of the web interface only stores north or west walls,
  // so we send the current cell coordinates (x, y) for the north wall.
  switch(state){
    case WallState::WALL:
      comm_interface_.sendWallData(x, y, true, true); // Send wall data for north wall
      break;
    case WallState::NO_WALL:
      comm_interface_.sendWallData(x, y, true, false); // Send no wall data for north wall
      break;
    case WallState::OUTER_WALL:
      fatalError("ERROR 'Maze::setHorizontalWallNorth': Wall should not be set to OUTER_WALL.");
      break;
    case WallState::UNKNOWN:
      fatalError("ERROR 'Maze::setHorizontalWallNorth': Wall should not be set to UNKNOWN.");
      break;
    default:
      fatalError("ERROR 'Maze::setHorizontalWallNorth': Unknown wall state %d", (int)state);
  }
}

/**
 * @brief Sets the state of the horizontal wall to the south of a cell and updates explored bounds.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @param state The new state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
void Maze::setHorizontalWallSouth(int16_t x, int16_t y, WallState state){
  checkMazeBounds(x, y);
  horizontal_walls_[logical2ArrayY(y)][logical2ArrayX(x)] = state;
  incrementVersion();
  updateExploredBounds(x, y);
  
  // Send wall data to the communication interface
  // Note: The datastructure of the web interface only stores north or west walls,
  // so we send the cell coordinates of the cell below the current cell => (x, y - 1).
  switch(state){
    case WallState::WALL:
      comm_interface_.sendWallData(x, y - 1, true, true); // Send wall data for south wall
      break;
    case WallState::NO_WALL:
      comm_interface_.sendWallData(x, y - 1, true, false); // Send no wall data for south wall
      break;
    case WallState::OUTER_WALL:
      fatalError("ERROR 'Maze::setHorizontalWallSouth': Wall should not be set to OUTER_WALL.");
      break;
    case WallState::UNKNOWN:
      fatalError("ERROR 'Maze::setHorizontalWallSouth': Wall should not be set to UNKNOWN.");
      break;
    default:
      fatalError("ERROR 'Maze::setHorizontalWallSouth': Unknown wall state %d", (int)state);
  }
}

/**
 * @brief Sets the state of the vertical wall to the east of a cell and updates explored bounds.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @param state The new state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
void Maze::setVerticalWallEast(int16_t x, int16_t y, WallState state){
  checkMazeBounds(x, y);
  vertical_walls_[logical2ArrayY(y)][logical2ArrayX(x + 1)] = state;
  incrementVersion();
  updateExploredBounds(x, y);
  
  // Send wall data to the communication interface
  // Note: The datastructure of the web interface only stores north or west walls,
  // so we send the cell coordinates of the current cell => (x + 1, y).
  switch(state){
    case WallState::WALL:
      comm_interface_.sendWallData(x + 1, y, false, true); // Send wall data for east wall
      break;
    case WallState::NO_WALL:
      comm_interface_.sendWallData(x + 1, y, false, false); // Send no wall data for east wall
      break;
    case WallState::OUTER_WALL:
      fatalError("ERROR 'Maze::setVerticalWallEast': Wall should not be set to OUTER_WALL.");
      break;
    case WallState::UNKNOWN:
      fatalError("ERROR 'Maze::setVerticalWallEast': Wall should not be set to UNKNOWN.");
      break;
    default:
      fatalError("ERROR 'Maze::setVerticalWallEast': Unknown wall state %d", (int)state);
  }
}

/**
 * @brief Sets the state of the vertical wall to the west of a cell and updates explored bounds.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @param state The new state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
void Maze::setVerticalWallWest(int16_t x, int16_t y, WallState state){
  checkMazeBounds(x, y);
  vertical_walls_[logical2ArrayY(y)][logical2ArrayX(x)] = state;
  incrementVersion();
  updateExploredBounds(x, y);
  
  // Send wall data to the communication interface
  // Note: The datastructure of the web interface only stores north or west walls,
  // so we send the cell coordinates of the current cell => (x, y).
  switch(state){
    case WallState::WALL:
      comm_interface_.sendWallData(x, y, false, true); // Send wall data for west wall
      break;
    case WallState::NO_WALL:
      comm_interface_.sendWallData(x, y, false, false); // Send no wall data for west wall
      break;
    case WallState::OUTER_WALL:
      fatalError("ERROR 'Maze::setVerticalWallWest': Wall should not be set to OUTER_WALL.");
      break;
    case WallState::UNKNOWN:
      fatalError("ERROR 'Maze::setVerticalWallWest': Wall should not be set to UNKNOWN.");
      break;
    default:
      fatalError("ERROR 'Maze::setVerticalWallWest': Unknown wall state %d", (int)state);
  }
}



/**
 * @brief Gets the state of the horizontal wall to the north of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return The state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
WallState Maze::getHorizontalWallNorth(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  return horizontal_walls_[logical2ArrayY(y + 1)][logical2ArrayX(x)];
}

/**
 * @brief Gets the state of the horizontal wall to the south of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return The state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
WallState Maze::getHorizontalWallSouth(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  return horizontal_walls_[logical2ArrayY(y)][logical2ArrayX(x)];
}

/**
 * @brief Gets the state of the vertical wall to the east of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return The state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
WallState Maze::getVerticalWallEast(int16_t x, int16_t y) const {
  checkMazeBounds(x, y);
  return vertical_walls_[logical2ArrayY(y)][logical2ArrayX(x + 1)];
}

/**
 * @brief Gets the state of the vertical wall to the west of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return The state of the wall.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
WallState Maze::getVerticalWallWest(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  return vertical_walls_[logical2ArrayY(y)][logical2ArrayX(x)];
}

/**
 * @brief Checks if there is a wall (WALL or OUTER_WALL) to the north of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return True if there is a wall, false otherwise.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
bool Maze::isWallNorth(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  WallState wall_state = horizontal_walls_[logical2ArrayY(y + 1)][logical2ArrayX(x)];
  if(wall_state == WallState::WALL || wall_state == WallState::OUTER_WALL)
    return true;
  return false;
}

/**
 * @brief Checks if there is a wall (WALL or OUTER_WALL) to the east of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return True if there is a wall, false otherwise.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
bool Maze::isWallEast(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  WallState wall_state = vertical_walls_[logical2ArrayY(y)][logical2ArrayX(x + 1)];
  if(wall_state == WallState::WALL || wall_state == WallState::OUTER_WALL)
    return true;
  return false;
}

/**
 * @brief Checks if there is a wall (WALL or OUTER_WALL) to the south of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return True if there is a wall, false otherwise.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
bool Maze::isWallSouth(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  WallState wall_state = horizontal_walls_[logical2ArrayY(y)][logical2ArrayX(x)];
  if(wall_state == WallState::WALL || wall_state == WallState::OUTER_WALL)
    return true;
  return false;
}

/**
 * @brief Checks if there is a wall (WALL or OUTER_WALL) to the west of a cell.
 * @param x The x-coordinate of the cell.
 * @param y The y-coordinate of the cell.
 * @return True if there is a wall, false otherwise.
 * @note Calls checkBounds to check if the coordinates are out of bounds.
 */
bool Maze::isWallWest(int16_t x, int16_t y) const{
  checkMazeBounds(x, y);
  WallState wall_state = vertical_walls_[logical2ArrayY(y)][logical2ArrayX(x)];
  if(wall_state == WallState::WALL || wall_state == WallState::OUTER_WALL)
    return true;
  return false;
}

/**
 * @brief Checks if there is a wall between two cells.
 * @param x1 The x-coordinate of the first cell.
 * @param y1 The y-coordinate of the first cell.
 * @param x2 The x-coordinate of the second cell.
 * @param y2 The y-coordinate of the second cell.
 * @return True if there is a wall between the cells, false otherwise.
 * @note Checks if cells are adjacent and calls isWall{North, South, East, West} accordingly.
 */
bool Maze::isWallBetween(int16_t x1, int16_t y1, int16_t x2, int16_t y2) const{
  // Check if the cells are adjacent
  if(abs(x1 - x2) + abs(y1 - y2) != 1){
    fatalError("ERROR 'Maze::isWallBetween': Cells (%d, %d) and (%d, %d) are not adjacent", x1, y1, x2, y2);
    return false; // This line is never be reached
  }
  
  // Check which wall to check
  if(x1 == x2){ // Same column
    return (y1 < y2) ? isWallNorth(x1, y1) : isWallSouth(x1, y1);
  } else{ // Same row
    return (x1 < x2) ? isWallEast(x1, y1) : isWallWest(x1, y1);
  }
}



// --- Version tracking ---
/**
 * @brief Gets the current version of the maze.
 * @return The version counter.
 */
uint8_t Maze::getVersion() const{
  return version_counter_;
}
/**
 * @brief Checks if the maze has changed since a given version.
 * @param oldVersion The version to compare against.
 * @return True if the maze has changed, false otherwise.
 */
bool Maze::hasChangedSince(uint8_t oldVersion) const{
  return version_counter_ != oldVersion;
}



// --- Bounds tracking ---
/**
 * @brief Checks if the given logical coordinates are within the bounds of the maze.
 * @param x The x-coordinate to check.
 * @param y The y-coordinate to check.
 * @note If the coordinates are out of bounds, a fatal error is triggered. 
 *       The system will enter an infinite loop.
 * @see  isCellInMaze() is a non-fatal check that returns a boolean value.
 */
void Maze::checkMazeBounds(int16_t x, int16_t y) const{
  if(abs(x) > MAZE_MAX_X || abs(y) > MAZE_MAX_Y){
    fatalError("ERROR 'Maze::checkBounds': Coordinates out of bounds. (%d, %d) not in (+-%d, +-%d)", x, y, MAZE_MAX_X, MAZE_MAX_Y);
  }
}

/**
 * @brief Gets the minimum explored x-coordinate of the maze.
 * @return The minimum explored x-coordinate.
 */
int16_t Maze::getMinExploredX() const{
  return min_explored_x_;
}

/**
 * @brief Gets the maximum explored x-coordinate of the maze.
 * @return The maximum explored x-coordinate.
 */
int16_t Maze::getMaxExploredX() const {
  return max_explored_x_;
}

/**
 * @brief Gets the minimum explored y-coordinate of the maze.
 * @return The minimum explored y-coordinate.
 */
int16_t Maze::getMinExploredY() const{
  return min_explored_y_;
}

/**
 * @brief Gets the maximum explored y-coordinate of the maze.
 * @return The maximum explored y-coordinate.
 */
int16_t Maze::getMaxExploredY() const{
  return max_explored_y_;
}

/**
 * @brief Gets the maximum reachable x-coordinate of the maze, according to setUnreachableCells().
 * 
 * @return The maximum reachable x-coordinate.
 * 
 * @warning This function does not call setUnreachableCells() to update the maze bounds.
 *          It is the caller's responsibility to ensure that the maze bounds are up to date.
 */
int16_t Maze::getMinReachableX() const{
  return min_reachable_x_;
}

/**
 * @brief Gets the maximum reachable x-coordinate of the maze, according to setUnreachableCells().
 * 
 * @return The maximum reachable x-coordinate.
 * 
 * @warning This function does not call setUnreachableCells() to update the maze bounds.
 *          It is the caller's responsibility to ensure that the maze bounds are up to date.
 */
int16_t Maze::getMaxReachableX() const{
  return max_reachable_x_;
}

/**
 * @brief Gets the minimum reachable y-coordinate of the maze, according to setUnreachableCells().
 * 
 * @return The minimum reachable y-coordinate.
 * 
 * @warning This function does not call setUnreachableCells() to update the maze bounds.
 *          It is the caller's responsibility to ensure that the maze bounds are up to date.
 */
int16_t Maze::getMinReachableY() const{
  return min_reachable_y_;
}

/**
 * @brief Gets the maximum reachable y-coordinate of the maze, according to setUnreachableCells().
 * 
 * @return The maximum reachable y-coordinate.
 * 
 * @warning This function does not call setUnreachableCells() to update the maze bounds.
 *          It is the caller's responsibility to ensure that the maze bounds are up to date.
 */
int16_t Maze::getMaxReachableY() const{
  return max_reachable_y_;
}



// --- Public helpers ---
/**
 * @brief Converts logical coordinates to array indices.
 * @param x The x-coordinate to convert.
 * @return The corresponding array index.
 */
int16_t Maze::logical2ArrayX(int16_t x) const{
  return MAZE_MAX_X + x;
}

/**
 * @brief Converts logical coordinates to array indices.
 * @param y The y-coordinate to convert.
 * @return The corresponding array index.
 */
int16_t Maze::logical2ArrayY(int16_t y) const{
  return MAZE_MAX_Y + y;
}



// --- Internal helpers ---
/**
 * @brief Increments the version counter of the maze.
 * 
 * This function makes sure that the version counter is reset to 0 after reaching 254.
 * 255 is reserved for invalid version.
 */
void Maze::incrementVersion(){
  version_counter_++;
  if(version_counter_ > 254) // 255 represents invalid version
    version_counter_ = 0; // Reset version counter
}

/**
 * @brief Checks if the given logical coordinates are within the bounds of the maze.
 * @param x The x-coordinate to check.
 * @param y The y-coordinate to check.
 * @return True if the coordinates are within bounds, false otherwise.
 * 
 * @see checkMazeBounds() for a fatal check that triggers an error if the coordinates are out of bounds.
 */
bool Maze::isCellInMaze(int16_t x, int16_t y) const{
  if(abs(x) > MAZE_MAX_X || abs(y) > MAZE_MAX_Y){
    return false;
  }
  return true;
}

/**
 * @brief Updates the explored bounds of the maze based on the given coordinates.
 * @param x The x-coordinate to update.
 * @param y The y-coordinate to update.
 */
void Maze::updateExploredBounds(int16_t x, int16_t y){
  if(x < min_explored_x_) min_explored_x_ = x;
  if(x > max_explored_x_) max_explored_x_ = x;
  if(y < min_explored_y_) min_explored_y_ = y;
  if(y > max_explored_y_) max_explored_y_ = y;
}