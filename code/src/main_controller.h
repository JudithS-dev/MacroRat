#ifndef __MAIN_CONTROLLER_H__
#define __MAIN_CONTROLLER_H__

#include "maze_map.h"
#include "movement_controller.h"
#include "navigation_engine.h"
#include "communication_interface.h"

enum class RatState;

class MainController{
public:
  MainController(); // in init serial beginn für error handler
  
  void Start();

  void MainControllerLoop(); // Main loop for the controller
private:
  RatState currentState_;
  
  CommunicationInterface commInterface_; // Communication interface for sending messages, has to be initialized first
  
  MazeMap map_;
  MovementController movementController_;
  NavigationEngine navigationEngine_;
};

#endif // __MAIN_CONTROLLER_H__