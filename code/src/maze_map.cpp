#include "maze_map.h"

#include <Arduino.h>

#include "maze.h"
#include "rat_position.h"
#include "error_handler.h"
#include "communication_interface.h"


/**
 * @brief Constructs a MazeMap object, initializing the maze and rat position.
 */
MazeMap::MazeMap(CommunicationInterface& comm_interface)
: maze(comm_interface), 
  rat_pos(maze, comm_interface)
{
  CommunicationInterface::log("               MazeMap: Initialization complete!");
}



/// --- Cell state access ---
/**
 * @brief Sets the wall state of the left wall of the rat's current position.
 *
 * If the current wall is UNKNOWN or OUTER_WALL, the given state will be set.
 * If the wall is already WALL or NO_WALL and the new state differs, an error will be logged.
 *
 * @param state The new WallState to set (WALL or NO_WALL).
 */
void MazeMap::setLeftWallState(WallState state){
  // check if the parameter state is valid
  if(state != WallState::WALL && state != WallState::NO_WALL)
    fatalError("ERROR 'MazeMap::setLeftWallState': Invalid wall state %d", (int)state);
  
  // retrieve current orientation and position of the rat
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  WallState current_state;
  
  // determine the current state of the wall to the left of the rat
  switch(orientation){
    case Direction::NORTH:  current_state = maze.getVerticalWallWest(x, y);    break;
    case Direction::EAST:   current_state = maze.getHorizontalWallNorth(x, y); break;
    case Direction::SOUTH:  current_state = maze.getVerticalWallEast(x, y);    break;
    case Direction::WEST:   current_state = maze.getHorizontalWallSouth(x, y); break;
    default:  fatalError("ERROR 'MazeMap::setLeftWallState': Unknown rat orientation %d", (int)orientation);
  }
  
  // decide action based on current wall state
  switch(current_state){
    case WallState::UNKNOWN: // Unknown: Safe to set new wall
    case WallState::OUTER_WALL: // Outer wall: Safe to set new wall
      switch(orientation){
        case Direction::NORTH:  maze.setVerticalWallWest(x, y, state);    break;
        case Direction::EAST:   maze.setHorizontalWallNorth(x, y, state); break;
        case Direction::SOUTH:  maze.setVerticalWallEast(x, y, state);    break;
        case Direction::WEST:   maze.setHorizontalWallSouth(x, y, state); break;
        default:  fatalError("ERROR 'MazeMap::setLeftWallState': Unknown rat orientation %d", (int)orientation);
      }
      //CommunicationInterface::log("[Info] Wall set successfully (was UNKNOWN).");
      break;
    default: // Already set: Check for conflict
      if(current_state != state){ // Conflict detected
        fatalError("ERROR 'MazeMap::setLeftWallState': Conflict detected! Trying to overwrite existing wall state %d with %d", (int)current_state, (int)state);
      } else{  // Already correctly set
        //CommunicationInterface::log("[Info] Wall already matches desired state; no update needed.");
      }
  }
}

/**
 * @brief Sets the wall state of the front wall of the rat's current position.
 *
 * If the current wall is UNKNOWN or OUTER_WALL, the given state will be set.
 * If the wall is already WALL or NO_WALL and the new state differs, an error will be logged.
 *
 * @param state The new WallState to set (WALL or NO_WALL).
 */
void MazeMap::setFrontWallState(WallState state){
  // check if the parameter state is valid
  if(state != WallState::WALL && state != WallState::NO_WALL)
    fatalError("ERROR 'MazeMap::setFrontWallState': Invalid wall state %d", (int)state);
  
  // retrieve current orientation and position of the rat
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  WallState current_state;
  
  // determine the current state of the wall in front of the rat
  switch(orientation){
    case Direction::NORTH:  current_state = maze.getHorizontalWallNorth(x, y); break;
    case Direction::EAST:   current_state = maze.getVerticalWallEast(x, y);    break;
    case Direction::SOUTH:  current_state = maze.getHorizontalWallSouth(x, y); break;
    case Direction::WEST:   current_state = maze.getVerticalWallWest(x, y);    break;
    default:  fatalError("ERROR 'MazeMap::setFrontWallState': Unknown rat orientation %d", (int)orientation);
  }
  
  // decide action based on current wall state
  switch(current_state){
    case WallState::UNKNOWN: // Unknown: Safe to set new wall
    case WallState::OUTER_WALL: // Outer wall: Safe to set new wall
      switch(orientation){
        case Direction::NORTH:  maze.setHorizontalWallNorth(x, y, state); break;
        case Direction::EAST:   maze.setVerticalWallEast(x, y, state);    break;
        case Direction::SOUTH:  maze.setHorizontalWallSouth(x, y, state); break;
        case Direction::WEST:   maze.setVerticalWallWest(x, y, state);    break;
        default:  fatalError("ERROR 'MazeMap::setFrontWallState': Unknown rat orientation %d", (int)orientation);
      }
      //CommunicationInterface::log("[Info] Wall set successfully (was UNKNOWN).");
      break;
    default: // Already set: Check for conflict
      if(current_state != state){ // Conflict detected
        fatalError("ERROR 'MazeMap::setFrontWallState': Conflict detected! Trying to overwrite existing wall state %d with %d", (int)current_state, (int)state);
      } else{  // Already correctly set
        //CommunicationInterface::log("[Info] Wall already matches desired state; no update needed.");
      }
  }
}

/**
 * @brief Sets the wall state of the right wall of the rat's current position.
 *
 * If the current wall is UNKNOWN or OUTER_WALL, the given state will be set.
 * If the wall is already WALL or NO_WALL and the new state differs, an error will be logged.
 *
 * @param state The new WallState to set (WALL or NO_WALL).
 */
void MazeMap::setRightWallState(WallState state){
  // check if the parameter state is valid
  if(state != WallState::WALL && state != WallState::NO_WALL)
    fatalError("ERROR 'MazeMap::setRightWallState': Invalid wall state %d", (int)state);
  
  // retrieve current orientation and position of the rat
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  WallState current_state;
  
  // determine the current state of the wall to the right of the rat
  switch(orientation){
    case Direction::NORTH:  current_state = maze.getVerticalWallEast(x, y);    break;
    case Direction::EAST:   current_state = maze.getHorizontalWallSouth(x, y); break;
    case Direction::SOUTH:  current_state = maze.getVerticalWallWest(x, y);    break;
    case Direction::WEST:   current_state = maze.getHorizontalWallNorth(x, y); break;
    default:  fatalError("ERROR 'MazeMap::setRightWallState': Unknown rat orientation %d", (int)orientation);
  }
  
  // decide action based on current wall state
  switch(current_state){
    case WallState::UNKNOWN: // Unknown: Safe to set new wall
    case WallState::OUTER_WALL: // Outer wall: Safe to set new wall
      switch(orientation){
        case Direction::NORTH:  maze.setVerticalWallEast(x, y, state);    break;
        case Direction::EAST:   maze.setHorizontalWallSouth(x, y, state); break;
        case Direction::SOUTH:  maze.setVerticalWallWest(x, y, state);    break;
        case Direction::WEST:   maze.setHorizontalWallNorth(x, y, state); break;
        default:  fatalError("ERROR 'MazeMap::setRightWallState': Unknown rat orientation %d", (int)orientation);
      }
      //CommunicationInterface::log("[Info] Wall set successfully (was UNKNOWN).");
      break;
    default: // Already set: Check for conflict
      if(current_state != state){ // Conflict detected
        fatalError("ERROR 'MazeMap::setRightWallState': Conflict detected! Trying to overwrite existing wall state %d with %d", (int)current_state, (int)state);
      } else{  // Already correctly set
        //CommunicationInterface::log("[Info] Wall already matches desired state; no update needed.");
      }
  }
}



// --- Wall state access ---
/**
 * @brief Checks if there is a wall to the left of the rat's current position.
 * 
 * @return True if there is a wall (WALL or OUTER_WALL), false otherwise.
 * @see Maze::isWallNorth, Maze::isWallEast, Maze::isWallSouth, Maze::isWallWest
 */
bool MazeMap::isWallLeft() const{
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  switch(orientation){
    case Direction::NORTH: return maze.isWallWest(x, y);
    case Direction::EAST:  return maze.isWallNorth(x, y);
    case Direction::SOUTH: return maze.isWallEast(x, y);
    case Direction::WEST:  return maze.isWallSouth(x, y);
    default: fatalError("ERROR 'MazeMap::isWallLeft': Unknown rat orientation %d", (int)orientation);
  }
  return false; // This line should never be reached
}

/**
 * @brief Checks if there is a wall in front of the rat's current position.
 * 
 * @return True if there is a wall (WALL or OUTER_WALL), false otherwise.
 * @see Maze::isWallNorth, Maze::isWallEast, Maze::isWallSouth, Maze::isWallWest
 */
bool MazeMap::isWallFront() const{
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  switch(orientation){
    case Direction::NORTH: return maze.isWallNorth(x, y);
    case Direction::EAST:  return maze.isWallEast(x, y);
    case Direction::SOUTH: return maze.isWallSouth(x, y);
    case Direction::WEST:  return maze.isWallWest(x, y);
    default: fatalError("ERROR 'MazeMap::isWallFront': Unknown rat orientation %d", (int)orientation);
  }
  return false; // This line should never be reached
}

/**
 * @brief Checks if there is a wall to the right of the rat's current position.
 * 
 * @return True if there is a wall (WALL or OUTER_WALL), false otherwise.
 * @see Maze::isWallNorth, Maze::isWallEast, Maze::isWallSouth, Maze::isWallWest
 */
bool MazeMap::isWallRight() const{
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  switch(orientation){
    case Direction::NORTH: return maze.isWallEast(x, y);
    case Direction::EAST:  return maze.isWallSouth(x, y);
    case Direction::SOUTH: return maze.isWallWest(x, y);
    case Direction::WEST:  return maze.isWallNorth(x, y);
    default: fatalError("ERROR 'MazeMap::isWallRight': Unknown rat orientation %d", (int)orientation);
  }
  return false; // This line should never be reached
}

/**
 * @brief Checks if there is a wall behind the rat's current position.
 * 
 * @return True if there is a wall (WALL or OUTER_WALL), false otherwise.
 * @see Maze::isWallNorth, Maze::isWallEast, Maze::isWallSouth, Maze::isWallWest
 */
bool MazeMap::isWallBack() const{
  Direction orientation = rat_pos.getOrientation();
  int16_t x = rat_pos.getX();
  int16_t y = rat_pos.getY();
  
  switch(orientation){
    case Direction::NORTH: return maze.isWallSouth(x, y);
    case Direction::EAST:  return maze.isWallWest(x, y);
    case Direction::SOUTH: return maze.isWallNorth(x, y);
    case Direction::WEST:  return maze.isWallEast(x, y);
    default: fatalError("ERROR 'MazeMap::isWallBack': Unknown rat orientation %d", (int)orientation);
  }
  return false; // This line should never be reached
}



// --- Output ---
/**
 * @brief Prints the maze map (including walls, rat position and cell state) to the serial console.
 * 
 * The walls are represented as follows:
 * - WALL: (horizontal) --- or (vertical) |
 * - OUTER_WALL: (horizontal) ~~~ or (vertical) \
 * - UNKNOWN: ?
 * - NO_WALL: (space)
 *  
 * The cell states are represented as follows: 
 * - NON_VISITED: N
 * - UNREACHABLE: X
 * - EMPTY: (space)
 * - TRAP: T
 * - CHEESE: C
 * 
 * The rat's orientation represented as an arrow:
 * - ^ (north)
 * - > (east)
 * - v (south)
 * - < (west)
 */
void MazeMap::printMaze() const{
  // Debug output of bounds of maze
  Serial.printf("Maze reachable bounds: (%d, %d) to (%d, %d)\n", maze.getMinReachableX(), maze.getMinReachableY(), maze.getMaxReachableX(), maze.getMaxReachableY());
  Serial.printf("Maze explored  bounds: (%d, %d) to (%d, %d)\n", maze.getMinExploredX(), maze.getMinExploredY(), maze.getMaxExploredX(), maze.getMaxExploredY());
  Serial.printf("Maze bounds: (%d, %d) to (%d, %d)\n", -maze.MAZE_MAX_X, -maze.MAZE_MAX_Y, maze.MAZE_MAX_X, maze.MAZE_MAX_Y);
  
  // Bounds for printing the maze
  int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X,  maze.getMinExploredX() - 1);
  int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X,  maze.getMaxExploredX() + 1);
  int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y,  maze.getMinExploredY() - 1);
  int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y,  maze.getMaxExploredY() + 1);
  
  WallState current_wall = WallState::UNKNOWN;
  CellState current_cell = CellState::NON_VISITED;
  // From the top to the bottom, so y decreases
  for(int16_t y = max_y; y >= min_y; y--){
  
    // Top of the row: north walls
    for(int16_t x = min_x; x <= max_x; x++){
      current_wall = maze.getHorizontalWallNorth(x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.printf("+---"); break;
        case WallState::OUTER_WALL: Serial.printf("+~~~"); break;
        case WallState::UNKNOWN:    Serial.printf("+ ? "); break;
        case WallState::NO_WALL:    Serial.printf("+   "); break;
        default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
      }
    }
    Serial.printf("+\n"); // End of the row
    
    // content of the row: walls and rat
    for(int16_t x = min_x; x <= max_x; x++){
      current_wall = maze.getVerticalWallWest(x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.printf("|");  break;
        case WallState::OUTER_WALL: Serial.printf("\\"); break;
        case WallState::UNKNOWN:    Serial.printf("?");  break;
        case WallState::NO_WALL:    Serial.printf(" ");  break;
        default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      // get current cell state
      current_cell = maze.getCellState(x, y);
      
      // print cell state and rat position
      if(rat_pos.getX() == x && rat_pos.getY() == y){ // rat is in current cell
        if(current_cell == CellState::EMPTY){ // only print rat if cell is empty
          switch(rat_pos.getOrientation()){
            case Direction::NORTH: Serial.printf(" ^ "); break;
            case Direction::EAST:  Serial.printf(" > "); break;
            case Direction::SOUTH: Serial.printf(" v "); break;
            case Direction::WEST:  Serial.printf(" < "); break;
            default:               fatalError("ERROR 'MazeMap::printMaze': Unknown rat orientation %d", (int)rat_pos.getOrientation());
          }
        }
        else{ // print cell state left of the rat
          switch(current_cell){ 
            case CellState::NON_VISITED: Serial.printf(" N"); break;
            case CellState::UNREACHABLE: Serial.printf(" X"); break;
            case CellState::TRAP:        Serial.printf(" T"); break;
            case CellState::CHEESE:      Serial.printf(" C"); break;
            default:                     fatalError("ERROR 'MazeMap::printMaze': Unknown cell state %d", (int)current_cell);
          }
          switch(rat_pos.getOrientation()){
            case Direction::NORTH: Serial.printf("^"); break;
            case Direction::EAST:  Serial.printf(">"); break;
            case Direction::SOUTH: Serial.printf("v"); break;
            case Direction::WEST:  Serial.printf("<"); break;
            default:               fatalError("ERROR 'MazeMap::printMaze': Unknown rat orientation %d", (int)rat_pos.getOrientation());
          }
        }
        } else{ // no rat in current cell
          switch(current_cell){ 
            case CellState::NON_VISITED: Serial.printf(" N "); break;
            case CellState::UNREACHABLE: Serial.printf(" X "); break;
            case CellState::EMPTY:       Serial.printf("   "); break;
            case CellState::TRAP:        Serial.printf(" T "); break;
            case CellState::CHEESE:      Serial.printf(" C "); break;
            default:                     fatalError("ERROR 'MazeMap::printMaze': Unknown cell state %d", (int)current_cell);
          }
        }
      }
      
      // right wall of the row
      current_wall = maze.getVerticalWallEast(max_x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.printf("|");  break;
        case WallState::OUTER_WALL: Serial.printf("\\"); break;
        case WallState::UNKNOWN:    Serial.printf("?");  break;
        case WallState::NO_WALL:    Serial.printf(" ");  break;
        default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      Serial.printf("\n");
  }
  
  // Bottom of the maze: south walls
  for(int16_t x = min_x; x <= max_x; x++){
    current_wall = maze.getHorizontalWallSouth(x, min_y);
    switch(current_wall){
      case WallState::WALL:       Serial.printf("+---"); break;
      case WallState::OUTER_WALL: Serial.printf("+~~~"); break;
      case WallState::UNKNOWN:    Serial.printf("+ ? "); break;
      case WallState::NO_WALL:    Serial.printf("+   "); break;
      default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
    }
  }
  Serial.printf("+\n"); // End of the row
}

void MazeMap::printMazeColored() const{
  // Debug output of bounds of maze
  Serial.printf("Maze reachable bounds: (%d, %d) to (%d, %d)\n", maze.getMinReachableX(), maze.getMinReachableY(), maze.getMaxReachableX(), maze.getMaxReachableY());
  Serial.printf("Maze explored  bounds: (%d, %d) to (%d, %d)\n", maze.getMinExploredX(), maze.getMinExploredY(), maze.getMaxExploredX(), maze.getMaxExploredY());
  Serial.printf("Maze bounds: (%d, %d) to (%d, %d)\n", -maze.MAZE_MAX_X, -maze.MAZE_MAX_Y, maze.MAZE_MAX_X, maze.MAZE_MAX_Y);
  
  // Bounds for printing the maze
  int16_t min_x = std::max<int16_t>(-Maze::MAZE_MAX_X,  maze.getMinExploredX() - 1);
  int16_t max_x = std::min<int16_t>( Maze::MAZE_MAX_X,  maze.getMaxExploredX() + 1);
  int16_t min_y = std::max<int16_t>(-Maze::MAZE_MAX_Y,  maze.getMinExploredY() - 1);
  int16_t max_y = std::min<int16_t>( Maze::MAZE_MAX_Y,  maze.getMaxExploredY() + 1);

  WallState current_wall = WallState::UNKNOWN;
  CellState current_cell = CellState::NON_VISITED;
  // From the top to the bottom, so y decreases
  for(int16_t y = max_y; y >= min_y; y--){
  
    // Top of the row: north walls
    for(int16_t x = min_x; x <= max_x; x++){
      current_wall = maze.getHorizontalWallNorth(x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.print("\033[90m+\033[94m---\033[0m"); break;
        case WallState::OUTER_WALL: Serial.print("\033[90m+~~~\033[0m"); break;
        case WallState::UNKNOWN:    Serial.print("\033[90m+ ? \033[0m"); break;
        case WallState::NO_WALL:    Serial.print("\033[90m+   \033[0m"); break;
        default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
      }
    }
    Serial.printf("\033[90m+\033[0m\n"); // End of the row
    
    // content of the row: walls and rat
    for(int16_t x = min_x; x <= max_x; x++){
      current_wall = maze.getVerticalWallWest(x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.print("\033[94m|\033[0m");  break;
        case WallState::OUTER_WALL: Serial.print("\033[90m\\\033[0m"); break;
        case WallState::UNKNOWN:    Serial.print("\033[90m?\033[0m");  break;
        case WallState::NO_WALL:    Serial.print(" ");  break;
        default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      // get current cell state
      current_cell = maze.getCellState(x, y);
      
      // print cell state and rat position
      if(rat_pos.getX() == x && rat_pos.getY() == y){ // rat is in current cell
        if(current_cell == CellState::EMPTY){ // only print rat if cell is empty
          switch(rat_pos.getOrientation()){
            case Direction::NORTH: Serial.print("\033[37m ^ \033[0m"); break;
            case Direction::EAST:  Serial.print("\033[37m > \033[0m"); break;
            case Direction::SOUTH: Serial.print("\033[37m v \033[0m"); break;
            case Direction::WEST:  Serial.print("\033[37m < \033[0m"); break;
            default:               fatalError("ERROR 'MazeMap::printMaze': Unknown rat orientation %d", (int)rat_pos.getOrientation());
          }
        }
        else{ // print cell state left of the rat
          switch(current_cell){ 
            case CellState::NON_VISITED: Serial.print("\033[90m N\033[0m"); break;
            case CellState::UNREACHABLE: Serial.print("\033[31m X\033[0m"); break;
            case CellState::TRAP:        Serial.print("\033[91m T\033[0m"); break;
            case CellState::CHEESE:      Serial.print("\033[33m C\033[0m"); break;
            default:                     fatalError("ERROR 'MazeMap::printMaze': Unknown cell state %d", (int)current_cell);
          }
          switch(rat_pos.getOrientation()){
            case Direction::NORTH: Serial.print("\033[37m^\033[0m"); break;
            case Direction::EAST:  Serial.print("\033[37m>\033[0m"); break;
            case Direction::SOUTH: Serial.print("\033[37mv\033[0m"); break;
            case Direction::WEST:  Serial.print("\033[37m<\033[0m"); break;
            default:               fatalError("ERROR 'MazeMap::printMaze': Unknown rat orientation %d", (int)rat_pos.getOrientation());
          }
        }
        } else{ // no rat in current cell
          switch(current_cell){ 
            case CellState::NON_VISITED: Serial.print("\033[90m N \033[0m"); break;
            case CellState::UNREACHABLE: Serial.print("\033[31m X \033[0m"); break;
            case CellState::EMPTY:       Serial.print("   "); break;
            case CellState::TRAP:        Serial.print("\033[91m T \033[0m"); break;
            case CellState::CHEESE:      Serial.print("\033[33m C \033[0m"); break;
            default:                     fatalError("ERROR 'MazeMap::printMaze': Unknown cell state %d", (int)current_cell);
          }
        }
      }
      
      // right wall of the row
      current_wall = maze.getVerticalWallEast(max_x, y);
      switch(current_wall){
        case WallState::WALL:       Serial.print("\033[94m|\033[0m");  break;
        case WallState::OUTER_WALL: Serial.print("\033[90m\\\033[0m"); break;
        case WallState::UNKNOWN:    Serial.print("\033[90m?\033[0m");  break;
        case WallState::NO_WALL:    Serial.print(" ");  break;
        default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      Serial.print("\n");
  }
  
  // Bottom of the maze: south walls
  for(int16_t x = min_x; x <= max_x; x++){
    current_wall = maze.getHorizontalWallSouth(x, min_y);
    switch(current_wall){
      case WallState::WALL:       Serial.printf("\033[90m+\033[94m---\033[0m"); break;
      case WallState::OUTER_WALL: Serial.printf("\033[90m+~~~\033[0m"); break;
      case WallState::UNKNOWN:    Serial.printf("\033[90m+ ? \033[0m"); break;
      case WallState::NO_WALL:    Serial.printf("\033[90m+   \033[0m"); break;
      default:                    fatalError("ERROR 'MazeMap::printMaze': Unknown wall state %d", (int)current_wall);
    }
  }
  Serial.printf("\033[90m+\033[0m\n"); // End of the row
}