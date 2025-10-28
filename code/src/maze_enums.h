#ifndef __MAZE_ENUMS_H__
#define __MAZE_ENUMS_H__

/**
 * @enum CellState
 * @brief Defines all possible states of a cell in the maze.
 */
enum class CellState {
  NON_VISITED = 0, ///< Unvisited cell, initial state
  UNREACHABLE,     ///< Marked as unreachable (only by navigation engine)
  EMPTY,           ///< Visited cell without trap or cheese
  TRAP,            ///< Visited cell with a trap
  CHEESE           ///< Visited cell with cheese
};

/**
 * @enum WallState
 * @brief Defines all possible states of a (vertical or horizontal) wall between two cells.
 */
enum class WallState {
  UNKNOWN = 0, ///< Non-explored wall state
  NO_WALL,     ///< Explored: no wall present
  WALL,        ///< Explored: wall present
  OUTER_WALL,  ///< Used to mark the end of reachable space, could be a wall or a non-wall, rat won't be able to cross it
};

#endif // __MAZE_ENUMS_H__