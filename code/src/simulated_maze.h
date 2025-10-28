#ifndef __SIMULATED_MAZE_H__
#define __SIMULATED_MAZE_H__

#include "maze_enums.h"
#include "rat_position.h"

/**
 * @enum MazeType
 * @brief Enumeration of different simulated mazes.
 */
enum class SimMazeConfig{
  EMPTY = 0,    ///< Empty maze with no walls, traps, or cheese
  BASIC_1,      ///< Basic mazes can be solved by following the walls
  BASIC_2,      ///< Basic mazes can be solved by following the walls
  COMPLEX_1,    ///< Complex mazes can't be solved by following the walls
  COMPLEX_2,    ///< Complex mazes can't be solved by following the walls
  AVOID_TRAP_1, ///< Maze with traps that the rat should avoid
  AVOID_TRAP_2, ///< Maze with traps that the rat should avoid
  USE_TRAP_1,   ///< Maze with traps that the rat will use, even if it is not necessary
};

class SimulatedMaze{
public:
  // === Methods ===
  SimulatedMaze(const RatPosition& rat_pos, SimMazeConfig maze_type);
  
  // --- Cell state access ---
  CellState getCellState() const;
  
  // --- Wall state access ---
  WallState getWallLeft() const;
  WallState getWallFront() const;
  WallState getWallRight() const;

  // --- Simulation output ---
  void printMaze() const;
  void printMazeColored() const;
private:
  // === Constants ===
  static constexpr int16_t SIM_WIDTH  = 5; ///< Total number of columns in the simulated maze
  static constexpr int16_t SIM_HEIGHT = 4; ///< Total number of rows in the simulated maze
  
  // === Attributes ===
  // --- Maze structure ---
  CellState cell_states_[SIM_HEIGHT][SIM_WIDTH];           ///< Cell states, of simulated maze
  WallState vertical_walls_[SIM_HEIGHT][SIM_WIDTH + 1];    ///< west vertical wall of cells of simulated maze
  WallState horizontal_walls_[SIM_HEIGHT + 1][SIM_WIDTH];  ///< south horizontal wall of cells of simulated maze
  
  // --- Rat position ---
  int16_t global_rat_start_x_; ///< Starting global X position of the rat in the simulated maze
  int16_t global_rat_start_y_; ///< Starting global Y position of the rat in the simulated maze
  const RatPosition &rat_pos_; ///< Reference to the rat position object
  
  // === Methods ===
  // --- Internal helpers ---
  int16_t getGlobalXIndex() const;
  int16_t getGlobalYIndex() const;
  
  // --- Maze configurations ---
  void setMazeEmpty();
  void setMazeBasic_1();
  void setMazeBasic_2();
  void setMazeComplex_1();
  void setMazeComplex_2();
  void setMazeAvoidTrap_1();
  void setMazeAvoidTrap_2();
  void setMazeUseTrap_1();
};

#endif // __SIMULATED_MAZE_H__