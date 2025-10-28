#include "simulated_maze.h"

#include <Arduino.h>

#include "error_handler.h"
#include "communication_interface.h"

/**
 * @brief Constructs a SimulatedMaze object and initializes its state.
 * 
 * The constructor creates a simulated maze with the specified rat position,
 * depending on maze_type.
 * 
 * @param rat_pos Reference to the RatPosition object representing the rat's position.
 * @param maze_type The type of maze to be created (empty, basic, or complex).
 */
SimulatedMaze::SimulatedMaze(const RatPosition& rat_pos, SimMazeConfig maze_type)
: rat_pos_(rat_pos)
{
  // set all cell states to EMPTY
  for(uint16_t row = 0; row < SIM_HEIGHT; row++){
    for(uint16_t col = 0; col < SIM_WIDTH; col++){
      cell_states_[row][col] = CellState::EMPTY;
    }
  }
  // set all wall states to NO_WALL
  for(uint16_t row = 0; row < SIM_HEIGHT; row++){
    for(uint16_t col = 0; col < SIM_WIDTH + 1; col++){
      vertical_walls_[row][col] = WallState::NO_WALL;
    }
  }
  for(uint16_t row = 0; row < SIM_HEIGHT + 1; row++){
    for(uint16_t col = 0; col < SIM_WIDTH; col++){
      horizontal_walls_[row][col] = WallState::NO_WALL;
    }
  }
  // set outer walls to WALL
  // Set top and bottom horizontal walls
  for(int16_t col = 0; col < SIM_WIDTH; col++){
    horizontal_walls_[0][col] = WallState::WALL;
    horizontal_walls_[SIM_HEIGHT][col] = WallState::WALL;
  }
  // Set left and right vertical walls
  for(int16_t row = 0; row < SIM_HEIGHT; row++){
    vertical_walls_[row][0] = WallState::WALL;
    vertical_walls_[row][SIM_WIDTH] = WallState::WALL;
  }
  
  // set specific walls, cells and starting position for the simulated maze
  switch(maze_type){
    case SimMazeConfig::EMPTY:         setMazeEmpty();       break;
    case SimMazeConfig::BASIC_1:       setMazeBasic_1();     break;
    case SimMazeConfig::BASIC_2:       setMazeBasic_2();     break;
    case SimMazeConfig::COMPLEX_1:     setMazeComplex_1();   break;
    case SimMazeConfig::COMPLEX_2:     setMazeComplex_2();   break;
    case SimMazeConfig::AVOID_TRAP_1:  setMazeAvoidTrap_1(); break;
    case SimMazeConfig::AVOID_TRAP_2:  setMazeAvoidTrap_2(); break;
    case SimMazeConfig::USE_TRAP_1:    setMazeUseTrap_1();   break;
    default: fatalError("ERROR 'SimulatedMaze::SimulatedMaze': Unknown maze type");
  }
  
  CommunicationInterface::log("         SimulatedMaze: Initialization complete!");
}



// --- Cell state access ---
/**
 * @brief Get the state of the cell where the rat is currently located.
 * @return The state of the cell.
 */
CellState SimulatedMaze::getCellState() const{
  return cell_states_[getGlobalYIndex()][getGlobalXIndex()];
}



// --- Wall state access ---
/**
 * @brief Get the wall state to the left of the rat, based on its orientation.
 * @return Wall state on the left side.
 */
WallState SimulatedMaze::getWallLeft() const{
  int16_t x = getGlobalXIndex();
  int16_t y = getGlobalYIndex();

  switch(rat_pos_.getOrientation()){
    case Direction::NORTH: return vertical_walls_[y][x];
    case Direction::EAST:  return horizontal_walls_[y + 1][x];
    case Direction::SOUTH: return vertical_walls_[y][x + 1];
    case Direction::WEST:  return horizontal_walls_[y][x];
    default:  fatalError("ERROR 'SimulatedMaze::getWallLeft': Unknown direction");
              return WallState::UNKNOWN; // This line will never be reached
  }
}

/**
 * @brief Get the wall state in front of the rat, based on its orientation.
 * @return Wall state in front of the rat.
 */
WallState SimulatedMaze::getWallFront() const{
  int16_t x = getGlobalXIndex();
  int16_t y = getGlobalYIndex();

  switch(rat_pos_.getOrientation()){
    case Direction::NORTH: return horizontal_walls_[y + 1][x];
    case Direction::EAST:  return vertical_walls_[y][x + 1];
    case Direction::SOUTH: return horizontal_walls_[y][x];
    case Direction::WEST:  return vertical_walls_[y][x];
    default:  fatalError("ERROR 'SimulatedMaze::getWallFront': Unknown direction");
              return WallState::UNKNOWN; // This line will never be reached
  }
}

/**
 * @brief Get the wall state to the right of the rat, based on its orientation.
 * @return Wall state on the right side.
 */
WallState SimulatedMaze::getWallRight() const{
  int16_t x = getGlobalXIndex();
  int16_t y = getGlobalYIndex();

  switch(rat_pos_.getOrientation()){
    case Direction::NORTH: return vertical_walls_[y][x + 1];
    case Direction::EAST:  return horizontal_walls_[y][x];
    case Direction::SOUTH: return vertical_walls_[y][x];
    case Direction::WEST:  return horizontal_walls_[y + 1][x];
    default:  fatalError("ERROR 'SimulatedMaze::getWallRight': Unknown direction");
              return WallState::UNKNOWN; // This line will never be reached
  }
}



// --- Simulation output ---
/**
 * @brief Prints the simulated maze (including walls, rat position, and cell states) to the serial console.
 *
 * The walls are represented as follows:
 * - WALL:         +--- (horizontal), | (vertical)
 * - NO_WALL:      +    (horizontal),   (vertical)
 * - OUTER_WALL / UNKNOWN: fatal error (should not appear in simulation)
 * 
 * The cell states are represented as follows: 
 * - EMPTY:        (space)
 * - TRAP:         T
 * - CHEESE:       C
 * - NON_VISITED / UNREACHABLE: fatal error (not allowed in simulation)
 * 
 * The rat's orientation represented as an arrow:
 * - ^ (north)
 * - > (east)
 * - v (south)
 * - < (west)
 */
void SimulatedMaze::printMaze() const{
  // Debug output of bounds of maze
  Serial.printf("Simulated Maze bounds: (0, 0) to (%d, %d)\n", SIM_WIDTH - 1, SIM_HEIGHT - 1);
  
  WallState current_wall = WallState::UNKNOWN;
  CellState current_cell = CellState::NON_VISITED;
  // From the top to the bottom, so y decreases
  for(int16_t y = SIM_HEIGHT - 1; y >= 0; y--){
  
    // Top of the row: north walls
    for(int16_t x = 0; x < SIM_WIDTH; x++){
      current_wall = horizontal_walls_[y + 1][x];
      switch(current_wall){
        case WallState::WALL:       Serial.printf("+---"); break;
        case WallState::NO_WALL:    Serial.printf("+   "); break;
        case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL in simulated maze (%d, %d)", x, y);
                                    break;
        case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state in simulated maze (%d, %d)", x, y);
                                    break;
        default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
      }
    }
    Serial.printf("+\n"); // End of the row
    
    // content of the row: walls and rat
    for(int16_t x = 0; x < SIM_WIDTH; x++){
      current_wall = vertical_walls_[y][x];
      switch(current_wall){
        case WallState::WALL:       Serial.printf("|");  break;
        case WallState::NO_WALL:    Serial.printf(" ");  break;
        case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL in simulated maze (%d, %d)", x, y);
                                    break;
        case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state in simulated maze (%d, %d)", x, y);
                                    break;
        default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      // get current cell state
      current_cell = cell_states_[y][x];
      
      // print cell state and rat position
      if(getGlobalXIndex() == x && getGlobalYIndex() == y){ // rat is in current cell
        if(current_cell == CellState::EMPTY){ // only print rat if cell is empty
          switch(rat_pos_.getOrientation()){
            case Direction::NORTH: Serial.printf(" ^ "); break;
            case Direction::EAST:  Serial.printf(" > "); break;
            case Direction::SOUTH: Serial.printf(" v "); break;
            case Direction::WEST:  Serial.printf(" < "); break;
            default:               fatalError("ERROR 'SimulatedMaze::printMaze': Unknown rat orientation %d", (int)rat_pos_.getOrientation());
          }
        }
        else{ // print cell state left of the rat
          switch(current_cell){ 
            case CellState::TRAP:         Serial.printf(" T"); break;
            case CellState::CHEESE:       Serial.printf(" C"); break;
            case CellState::NON_VISITED:  fatalError("ERROR 'SimulatedMaze::printMaze': Found NON_VISITED cell in simulated maze (%d, %d)", x, y);
                                          break;
            case CellState::UNREACHABLE:  fatalError("ERROR 'SimulatedMaze::printMaze': Found UNREACHABLE cell in simulated maze (%d, %d)", x, y);
                                          break;
            default:                      fatalError("ERROR 'SimulatedMaze::printMaze': Unknown cell state %d", (int)current_cell);
          }
          switch(rat_pos_.getOrientation()){
            case Direction::NORTH: Serial.printf("^"); break;
            case Direction::EAST:  Serial.printf(">"); break;
            case Direction::SOUTH: Serial.printf("v"); break;
            case Direction::WEST:  Serial.printf("<"); break;
            default:               fatalError("ERROR 'SimulatedMaze::printMaze': Unknown rat orientation %d", (int)rat_pos_.getOrientation());
          }
        }
        } else{ // no rat in current cell
          switch(current_cell){ 
            case CellState::EMPTY:        Serial.printf("   "); break;
            case CellState::TRAP:         Serial.printf(" T "); break;
            case CellState::CHEESE:       Serial.printf(" C "); break;
            case CellState::NON_VISITED:  fatalError("ERROR 'SimulatedMaze::printMaze': Found NON_VISITED cell in simulated maze (%d, %d)", x, y);
                                          break;
            case CellState::UNREACHABLE:  fatalError("ERROR 'SimulatedMaze::printMaze': Found UNREACHABLE cell in simulated maze (%d, %d)", x, y);
                                          break;
            default:                      fatalError("ERROR 'SimulatedMaze::printMaze': Unknown cell state %d", (int)current_cell);
          }
        }
      }
      
      // right wall of the row
      current_wall = vertical_walls_[y][SIM_WIDTH];
      switch(current_wall){
        case WallState::WALL:       Serial.printf("|");  break;
        case WallState::NO_WALL:    fatalError("ERROR 'SimulatedMaze::printMaze': Found NO_WALL on edge of simulated maze (%d, %d)", SIM_WIDTH, y);
                                    break;
        case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL on edge of simulated maze (%d, %d)", SIM_WIDTH, y);
                                    break;
        case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state on edge of simulated maze (%d, %d)", SIM_WIDTH, y);
                                    break;
        default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      Serial.printf("\n");
  }
  
  // Bottom of the maze: south walls
  for(int16_t x = 0; x < SIM_WIDTH; x++){
    current_wall = horizontal_walls_[0][x];
    switch(current_wall){
      case WallState::WALL:       Serial.printf("+---"); break;
      case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL in simulated maze (%d, %d)", x, 0);
                                  break;
      case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state in simulated maze (%d, %d)", x, 0);
                                  break;
      case WallState::NO_WALL:    fatalError("ERROR 'SimulatedMaze::printMaze': Found NO_WALL on edge of simulated maze (%d, %d)", x, 0);
                                  break;
      default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
    }
  }
  Serial.printf("+\n"); // End of the row
}


void SimulatedMaze::printMazeColored() const{
  // Debug output of bounds of maze
  Serial.printf("Simulated Maze bounds: (0, 0) to (%d, %d)\n", SIM_WIDTH - 1, SIM_HEIGHT - 1);
  
  WallState current_wall = WallState::UNKNOWN;
  CellState current_cell = CellState::NON_VISITED;
  // From the top to the bottom, so y decreases
  for(int16_t y = SIM_HEIGHT - 1; y >= 0; y--){
  
    // Top of the row: north walls
    for(int16_t x = 0; x < SIM_WIDTH; x++){
      current_wall = horizontal_walls_[y + 1][x];
      switch(current_wall){
        case WallState::WALL:       Serial.printf("\033[90m+---\033[0m"); break;
        case WallState::NO_WALL:    Serial.printf("\033[90m+\033[0m   "); break;
        case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL in simulated maze (%d, %d)", x, y);
                                    break;
        case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state in simulated maze (%d, %d)", x, y);
                                    break;
        default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
      }
    }
    Serial.printf("\033[90m+\033[0m\n"); // End of the row
    
    // content of the row: walls and rat
    for(int16_t x = 0; x < SIM_WIDTH; x++){
      current_wall = vertical_walls_[y][x];
      switch(current_wall){
        case WallState::WALL:       Serial.printf("\033[90m|\033[0m");  break;
        case WallState::NO_WALL:    Serial.printf(" ");  break;
        case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL in simulated maze (%d, %d)", x, y);
                                    break;
        case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state in simulated maze (%d, %d)", x, y);
                                    break;
        default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      // get current cell state
      current_cell = cell_states_[y][x];
      
      // print cell state and rat position
      if(getGlobalXIndex() == x && getGlobalYIndex() == y){ // rat is in current cell
        if(current_cell == CellState::EMPTY){ // only print rat if cell is empty
          switch(rat_pos_.getOrientation()){
            case Direction::NORTH: Serial.printf(" \033[37m^\033[0m "); break;
            case Direction::EAST:  Serial.printf(" \033[37m>\033[0m "); break;
            case Direction::SOUTH: Serial.printf(" \033[37mv\033[0m "); break;
            case Direction::WEST:  Serial.printf(" \033[37m<\033[0m "); break;
            default:               fatalError("ERROR 'SimulatedMaze::printMaze': Unknown rat orientation %d", (int)rat_pos_.getOrientation());
          }
        }
        else{ // print cell state left of the rat
          switch(current_cell){ 
            case CellState::TRAP:         Serial.printf(" \033[31mT\033[0m"); break;
            case CellState::CHEESE:       Serial.printf(" \033[33mC\033[0m"); break;
            case CellState::NON_VISITED:  fatalError("ERROR 'SimulatedMaze::printMaze': Found NON_VISITED cell in simulated maze (%d, %d)", x, y);
                                          break;
            case CellState::UNREACHABLE:  fatalError("ERROR 'SimulatedMaze::printMaze': Found UNREACHABLE cell in simulated maze (%d, %d)", x, y);
                                          break;
            default:                      fatalError("ERROR 'SimulatedMaze::printMaze': Unknown cell state %d", (int)current_cell);
          }
          switch(rat_pos_.getOrientation()){
            case Direction::NORTH: Serial.printf("\033[37m^\033[0m"); break;
            case Direction::EAST:  Serial.printf("\033[37m>\033[0m"); break;
            case Direction::SOUTH: Serial.printf("\033[37mv\033[0m"); break;
            case Direction::WEST:  Serial.printf("\033[37m<\033[0m"); break;
            default:               fatalError("ERROR 'SimulatedMaze::printMaze': Unknown rat orientation %d", (int)rat_pos_.getOrientation());
          }
        }
        } else{ // no rat in current cell
          switch(current_cell){ 
            case CellState::EMPTY:        Serial.printf("   "); break;
            case CellState::TRAP:         Serial.print(" \033[31mT\033[0m "); break;
            case CellState::CHEESE:       Serial.print(" \033[33mC\033[0m "); break;
            case CellState::NON_VISITED:  fatalError("ERROR 'SimulatedMaze::printMaze': Found NON_VISITED cell in simulated maze (%d, %d)", x, y);
                                          break;
            case CellState::UNREACHABLE:  fatalError("ERROR 'SimulatedMaze::printMaze': Found UNREACHABLE cell in simulated maze (%d, %d)", x, y);
                                          break;
            default:                      fatalError("ERROR 'SimulatedMaze::printMaze': Unknown cell state %d", (int)current_cell);
          }
        }
      }
      
      // right wall of the row
      current_wall = vertical_walls_[y][SIM_WIDTH];
      switch(current_wall){
        case WallState::WALL:       Serial.printf("\033[90m|\033[0m");  break;
        case WallState::NO_WALL:    fatalError("ERROR 'SimulatedMaze::printMaze': Found NO_WALL on edge of simulated maze (%d, %d)", SIM_WIDTH, y);
                                    break;
        case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL on edge of simulated maze (%d, %d)", SIM_WIDTH, y);
                                    break;
        case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state on edge of simulated maze (%d, %d)", SIM_WIDTH, y);
                                    break;
        default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
      }
      
      Serial.printf("\n");
  }
  
  // Bottom of the maze: south walls
  for(int16_t x = 0; x < SIM_WIDTH; x++){
    current_wall = horizontal_walls_[0][x];
    switch(current_wall){
      case WallState::WALL:       Serial.printf("\033[90m+---\033[0m"); break;
      case WallState::OUTER_WALL: fatalError("ERROR 'SimulatedMaze::printMaze': Found OUTER_WALL in simulated maze (%d, %d)", x, 0);
                                  break;
      case WallState::UNKNOWN:    fatalError("ERROR 'SimulatedMaze::printMaze': Found UNKNOWN wall state in simulated maze (%d, %d)", x, 0);
                                  break;
      case WallState::NO_WALL:    fatalError("ERROR 'SimulatedMaze::printMaze': Found NO_WALL on edge of simulated maze (%d, %d)", x, 0);
                                  break;
      default:                    fatalError("ERROR 'SimulatedMaze::printMaze': Unknown wall state %d", (int)current_wall);
    }
  }
  Serial.printf("\033[90m+\033[0m\n"); // End of the row
}


// --- Internal helpers ---
/**
 * @brief Compute the global X index of the rat in the simulated maze.
 * @return Global X index within the simulated maze.
 */
int16_t SimulatedMaze::getGlobalXIndex() const{
  int16_t global_x = rat_pos_.getX() + global_rat_start_x_;
  if(global_x < 0 || global_x >= SIM_WIDTH){
    fatalError("ERROR 'SimulatedMaze::getGlobalXIndex': Global X coordinate out of bounds");
  }
  return global_x;
}

/**
 * @brief Compute the global Y index of the rat in the simulated maze.
 * @return Global Y index within the simulated maze.
 */
int16_t SimulatedMaze::getGlobalYIndex() const{
  int16_t global_y = rat_pos_.getY() + global_rat_start_y_;
  if(global_y < 0 || global_y >= SIM_HEIGHT){
    fatalError("ERROR 'SimulatedMaze::getGlobalYIndex': Global Y coordinate out of bounds");
  }
  return global_y;
}



// --- Maze configurations ---
/**
 * @brief Initializes an empty 4x4 maze.
 * 
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+
 *   3 |               |
 *     +   +   +   +   +
 *   2 |               |
 *     +   +   +   +   +
 *   1 |     ^         |
 *     +   +   +   +   +
 *   0 |               |
 *     +---+---+---+---+
 *       0   1   2   3   x
 */
void SimulatedMaze::setMazeEmpty(){
  // Check if the maze size is correct
  
  // Set start position (global coords at logical 1,2)
  global_rat_start_x_ = 1;
  global_rat_start_y_ = 1;
  
  // Place cheese
  
  // Inner horizontal walls from top to bottom
  
  // Inner vertical walls from left to right
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a basic 4x4 maze with walls and a cheese but no traps.
 * 
 * This maze is designed to be solvable by following the walls.
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+
 *   3 |   |   |       |
 *     +   +   +---+   +
 *   2 |     ^ | C |   |
 *     +   +   +   +   +
 *   1 |   |   |       |
 *     +   +   +---+   +
 *   0 |   |           |
 *     +---+---+---+---+
 *       0   1   2   3   x
 */
void SimulatedMaze::setMazeBasic_1(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 4 || SIM_HEIGHT != 4) {
    fatalError("ERROR 'SimulatedMaze::setMazeBasic_1': Maze size must be 4x4");
  }
  
  // Set start position (global coords at logical 1,2)
  global_rat_start_x_ = 1;
  global_rat_start_y_ = 2;
  
  // Place cheese
  cell_states_[2][2] = CellState::CHEESE;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[3][2] = WallState::WALL;
  horizontal_walls_[1][2] = WallState::WALL;
  
  // Inner vertical walls from left to right
  vertical_walls_[0][1] = WallState::WALL;
  vertical_walls_[1][1] = WallState::WALL;
  vertical_walls_[3][1] = WallState::WALL;
  
  vertical_walls_[1][2] = WallState::WALL;
  vertical_walls_[2][2] = WallState::WALL;
  vertical_walls_[3][2] = WallState::WALL;
  
  vertical_walls_[2][3] = WallState::WALL;
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a basic 4x4 maze with walls and a cheese but no traps and unreachable cells.
 * 
 * This maze is designed to be solvable by following the walls.
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+
 *   3 |               |
 *     +   +---+---+   +
 *   2 |   | ^ | C     |
 *     +   +   +---+   +
 *   1 |       |   |   |
 *     +   +   +   +   +
 *   0 |       |   |   |
 *     +---+---+---+---+
 *       0   1   2   3   x
 */
void SimulatedMaze::setMazeBasic_2(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 4 || SIM_HEIGHT != 4) {
    fatalError("ERROR 'SimulatedMaze::setMazeBasic_2': Maze size must be 4x4");
  }
  
  // Set start position (global coords at logical 1,2)
  global_rat_start_x_ = 1;
  global_rat_start_y_ = 2;
  
  // Place cheese
  cell_states_[2][2] = CellState::CHEESE;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[3][1] = WallState::WALL;
  horizontal_walls_[3][2] = WallState::WALL;
  horizontal_walls_[2][2] = WallState::WALL;
  
  // Inner vertical walls from left to right
  vertical_walls_[2][1] = WallState::WALL;

  vertical_walls_[0][2] = WallState::WALL;
  vertical_walls_[1][2] = WallState::WALL;
  vertical_walls_[2][2] = WallState::WALL;
  
  vertical_walls_[0][3] = WallState::WALL;
  vertical_walls_[1][3] = WallState::WALL;
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a complex 4x4 maze with walls and a cheese but no traps and unreachable cells.
 * 
 * This maze is designed to be not easily solvable by following the walls.
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+
 *   3 |               |
 *     +   +   +   +   +
 *   2 |         C     |
 *     +   +---+   +   +
 *   1 |         ^     |
 *     +   +   +   +   +
 *   0 |               |
 *     +---+---+---+---+
 *       0   1   2   3   x
 */
void SimulatedMaze::setMazeComplex_1(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 4 || SIM_HEIGHT != 4) {
    fatalError("ERROR 'SimulatedMaze::setMazeComplex_1': Maze size must be 4x4");
  }
  
  // Set start position (global coords at logical 1,2)
  global_rat_start_x_ = 2;
  global_rat_start_y_ = 1;
  
  // Place cheese
  cell_states_[2][2] = CellState::CHEESE;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[2][1] = WallState::WALL;
  
  // Inner vertical walls from left to right
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a complex 4x4 maze with walls and a cheese but no traps and unreachable cells.
 * 
 * This maze is designed to be not easily solvable by following the walls.
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+
 *   3 |             C |
 *     +   +   +   +   +
 *   2 |               |
 *     +   +---+   +   +
 *   1 |   |     ^     |
 *     +   +---+   +   +
 *   0 |               |
 *     +---+---+---+---+
 *       0   1   2   3   x
 */
void SimulatedMaze::setMazeComplex_2(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 4 || SIM_HEIGHT != 4) 
    fatalError("ERROR 'SimulatedMaze::setMazeComplex_2': Maze size must be 4x4");
  
  // Set start position (global coords at logical 1,2)
  global_rat_start_x_ = 2;
  global_rat_start_y_ = 1;
  
  // Place cheese
  cell_states_[3][3] = CellState::CHEESE;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[2][1] = WallState::WALL;
  horizontal_walls_[1][1] = WallState::WALL;
  
  // Inner vertical walls from left to right
  vertical_walls_[1][1] = WallState::WALL;
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a complex 4x5 maze with walls, traps and a cheese but no unreachable cells.
 * 
 * This maze is designed to be solvable by following the walls, but not without some traps.
 * It's possible to reach the cheese without hitting a trap, but it requires careful navigation.
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+---+
 *   3 |       |           |
 *     +   +   +---+       +
 *   2 |     T   C     |   |
 *     +   +   +---+---    +
 *   1 |           | T     |
 *     +   +---+   +       +
 *   0 | ^   T             |
 *     +---+---+---+---+---+
 *       0   1   2   3   4   x
 */
void SimulatedMaze::setMazeAvoidTrap_1(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 5 || SIM_HEIGHT != 4) 
    fatalError("ERROR 'SimulatedMaze::setMazeAvoidTrap_1': Maze size must be 4x5");
  
  // Set start position (global coords at logical 0,0)
  global_rat_start_x_ = 0;
  global_rat_start_y_ = 0;
  
  // Place cheese
  cell_states_[2][2] = CellState::CHEESE;
  
  // Place traps
  cell_states_[2][1] = CellState::TRAP;
  cell_states_[1][3] = CellState::TRAP;
  cell_states_[0][1] = CellState::TRAP;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[3][2] = WallState::WALL;
  horizontal_walls_[2][2] = WallState::WALL;
  horizontal_walls_[2][3] = WallState::WALL;
  horizontal_walls_[1][1] = WallState::WALL;
  
  // Inner vertical walls from left to right
  vertical_walls_[3][2] = WallState::WALL;
  vertical_walls_[2][4] = WallState::WALL;
  vertical_walls_[1][3] = WallState::WALL;
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a complex 7x6 maze with walls, traps and a cheese but no unreachable cells.
 * 
 * This maze is exactly the same as setMazeUseTrap_1, but without the second trap.
 * The rat solves the maze by avoiding all traps, which is possible.
 * This maze is designed to be not solvable by following the walls.
 * 
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+---+---+
 *   6 |           |           |
 *     +   +---+   +   +---+   +
 *   5 |   | C |       |   |   |
 *     +   +   +---+   +---+   +
 *   4 |       |   |   |       |
 *     +   +   +   +   +   +---+
 *   3 |         T |   |       |
 *     +---+---+   +   +---+   +
 *   2 |                   |   |
 *     +   +   +---+---+   +   +
 *   1 |   |           | <     |
 *     +   +---+---+---+---+---+
 *   0 |           |           |
 *     +---+---+---+---+---+---+
 *       0   1   2   3   4   5   x
 */
void SimulatedMaze::setMazeAvoidTrap_2(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 6 || SIM_HEIGHT != 7) 
    fatalError("ERROR 'SimulatedMaze::setMazeAvoidTrap_2': Maze size must be 7x6");
  
  // Set start position (global coords at logical 4,1)
  global_rat_start_x_ = 4;
  global_rat_start_y_ = 1;
  
  // Place cheese
  cell_states_[5][1] = CellState::CHEESE;
  
  // Place traps
  cell_states_[3][2] = CellState::TRAP;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[6][1] = WallState::WALL;
  horizontal_walls_[6][4] = WallState::WALL;
  horizontal_walls_[5][2] = WallState::WALL;
  horizontal_walls_[5][4] = WallState::WALL;
  horizontal_walls_[4][5] = WallState::WALL;
  horizontal_walls_[3][0] = WallState::WALL;
  horizontal_walls_[3][1] = WallState::WALL;
  horizontal_walls_[3][4] = WallState::WALL;
  horizontal_walls_[2][2] = WallState::WALL;
  horizontal_walls_[2][3] = WallState::WALL;
  horizontal_walls_[1][1] = WallState::WALL;
  horizontal_walls_[1][2] = WallState::WALL;
  horizontal_walls_[1][3] = WallState::WALL;
  horizontal_walls_[1][4] = WallState::WALL;
  horizontal_walls_[1][5] = WallState::WALL;
  
  // Inner vertical walls from left to right
  vertical_walls_[5][1] = WallState::WALL;
  vertical_walls_[1][1] = WallState::WALL;
  vertical_walls_[5][2] = WallState::WALL;
  vertical_walls_[4][2] = WallState::WALL;
  vertical_walls_[6][3] = WallState::WALL;
  vertical_walls_[4][3] = WallState::WALL;
  vertical_walls_[3][3] = WallState::WALL;
  vertical_walls_[0][3] = WallState::WALL;
  vertical_walls_[5][4] = WallState::WALL;
  vertical_walls_[4][4] = WallState::WALL;
  vertical_walls_[3][4] = WallState::WALL;
  vertical_walls_[1][4] = WallState::WALL;
  vertical_walls_[5][5] = WallState::WALL;
  vertical_walls_[2][5] = WallState::WALL;
  
  // Outer walls are assumed already set in the constructor
}

/**
 * @brief Initializes a complex 7x6 maze with walls, traps and a cheese but no unreachable cells.
 * 
 * This maze is exactly the same as setMazeAvoidTrap_2, but with the second trap.
 * The rat solves the maze by using one of the traps, eventhough it is not necessary.
 * This maze is designed to be not solvable by following the walls.
 *
 * The maze layout is as follows:
 *   y
 *     +---+---+---+---+---+---+
 *   6 |           |           |
 *     +   +---+   +   +---+   +
 *   5 |   | C |       |   |   |
 *     +   +   +---+   +---+   +
 *   4 |       |   | T |       |
 *     +   +   +   +   +   +---+
 *   3 |         T |   |       |
 *     +---+---+   +   +---+   +
 *   2 |                   |   |
 *     +   +   +---+---+   +   +
 *   1 |   |           | <     |
 *     +   +---+---+---+---+---+
 *   0 |           |           |
 *     +---+---+---+---+---+---+
 *       0   1   2   3   4   5   x
 */
void SimulatedMaze::setMazeUseTrap_1(){
  // Check if the maze size is correct
  if(SIM_WIDTH != 6 || SIM_HEIGHT != 7) 
    fatalError("ERROR 'SimulatedMaze::setMazeUseTrap_1': Maze size must be 7x6");
  
  // Set start position (global coords at logical 4,1)
  global_rat_start_x_ = 4;
  global_rat_start_y_ = 1;
  
  // Place cheese
  cell_states_[5][1] = CellState::CHEESE;
  
  // Place traps
  cell_states_[3][2] = CellState::TRAP;
  cell_states_[4][3] = CellState::TRAP;
  
  // Inner horizontal walls from top to bottom
  horizontal_walls_[6][1] = WallState::WALL;
  horizontal_walls_[6][4] = WallState::WALL;
  horizontal_walls_[5][2] = WallState::WALL;
  horizontal_walls_[5][4] = WallState::WALL;
  horizontal_walls_[4][5] = WallState::WALL;
  horizontal_walls_[3][0] = WallState::WALL;
  horizontal_walls_[3][1] = WallState::WALL;
  horizontal_walls_[3][4] = WallState::WALL;
  horizontal_walls_[2][2] = WallState::WALL;
  horizontal_walls_[2][3] = WallState::WALL;
  horizontal_walls_[1][1] = WallState::WALL;
  horizontal_walls_[1][2] = WallState::WALL;
  horizontal_walls_[1][3] = WallState::WALL;
  horizontal_walls_[1][4] = WallState::WALL;
  horizontal_walls_[1][5] = WallState::WALL;
  
  // Inner vertical walls from left to right
  vertical_walls_[5][1] = WallState::WALL;
  vertical_walls_[1][1] = WallState::WALL;
  vertical_walls_[5][2] = WallState::WALL;
  vertical_walls_[4][2] = WallState::WALL;
  vertical_walls_[6][3] = WallState::WALL;
  vertical_walls_[4][3] = WallState::WALL;
  vertical_walls_[3][3] = WallState::WALL;
  vertical_walls_[0][3] = WallState::WALL;
  vertical_walls_[5][4] = WallState::WALL;
  vertical_walls_[4][4] = WallState::WALL;
  vertical_walls_[3][4] = WallState::WALL;
  vertical_walls_[1][4] = WallState::WALL;
  vertical_walls_[5][5] = WallState::WALL;
  vertical_walls_[2][5] = WallState::WALL;
  
  // Outer walls are assumed already set in the constructor
}