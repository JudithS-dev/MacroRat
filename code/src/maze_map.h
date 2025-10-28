#ifndef __MAZEMAP_H__
#define __MAZEMAP_H__

#include "maze.h"
#include "rat_position.h"

class CommunicationInterface; // Forward declaration

/**
 * @class MazeMap
 * @brief Combines the maze structure and the rat's current position.
 *
 * The maze is represented as a grid of cells. This structure allows tracking of 
 * explored areas and recording the position of walls, traps, and cheese within the maze.
 *  
 * To track the rat in the maze, the structure includes the current logical position 
 * (X, Y) and orientation (facing direction) of the rat.
 */
class MazeMap{
public:
  // == Attributes ==
  Maze maze;             ///< The complete maze layout (walls and cell states)
  RatPosition rat_pos;   ///< The current position and orientation of the rat
  
  // == Methods ==
  MazeMap(CommunicationInterface& comm_interface);
  
  // --- Cell state access ---
  void setLeftWallState(WallState state);
  void setFrontWallState(WallState state);
  void setRightWallState(WallState state);
  
  // --- Wall state access ---
  bool isWallLeft() const;
  bool isWallFront() const;
  bool isWallRight() const;
  bool isWallBack() const;
  
  // --- Output ---
  void printMaze() const;
  void printMazeColored() const;
};

#endif // __MAZEMAP_H__