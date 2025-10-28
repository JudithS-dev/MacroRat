#ifndef __DIRECTION_ENUM_H__
#define __DIRECTION_ENUM_H__

/**
 * @enum Direction
 * @brief Possible orientations of the rat in the maze.
 */
enum class Direction {
  NORTH = 0,  ///< Facing in maze upwards (towards increasing y)
  EAST,       ///< Facing in maze right (towards increasing x)
  SOUTH,      ///< Facing in maze downwards (towards decreasing y)
  WEST        ///< Facing in maze left (towards decreasing x)
};

#endif // __DIRECTION_ENUM_H__