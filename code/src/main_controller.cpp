#include "main_controller.h"

#include <Arduino.h>

#include "error_handler.h"
#include "movement_action.h"
#include "operation_mode.h"

#define WIFI_SSID "TI Roboter"
#define WIFI_PASSWORD "ITRobot!"
#define MQTT_BROKER "172.16.6.148" // "172.16.18.48" //for me //"172.16.8.108" for Tim //"172.16.6.148" for Raspberry Pi

/**
 * @brief Represents the different states of the rat during operation.
 * 
 */
enum class RatState{
  INIT = 0,         ///< Initial state, rat is being set up and checks all walls in current cell (rotates 90° to the left to check wall behind it)
  EXPLORE,
  RETURN_TO_START_1,
  GO_TO_CHEESE,
  RETURN_TO_START_2,
  FINISHED,
  ERROR
};

MainController::MainController()
: currentState_(RatState::INIT),
  commInterface_(WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, 1883), // Initialize communication interface with WiFi and MQTT credentials
  map_(commInterface_),
  movementController_(map_), 
  navigationEngine_(map_, Algorithm::FLOOD_FILL)
{
  CommunicationInterface::log("        MainController: Initialization complete!");
}

// "Idle" state
void MainController::Start(){
  // Initialize the communication interface
  commInterface_.log("Starting MainController. Please enter a command...");
  
  bool idling = true;
  
  while(true)
  {
    commInterface_.loop();
    
    auto command = commInterface_.getCommand();
    if(command.length() > 0)
    {
      if(command == "start")
      {
        currentState_ = RatState::INIT; // Set the initial state to INIT
        MainControllerLoop();
        CommunicationInterface::log("Finished processing command: %s. Please enter a new command...", command.c_str());
      }
      else if(command == "turnLeft")
      {
        movementController_.executeAction(MovementAction::TURN_LEFT_90);
        CommunicationInterface::log("Finished processing command: %s. Please enter a new command...", command.c_str());
      }
      else if (command == "turnRight")
      {
        movementController_.executeAction(MovementAction::TURN_RIGHT_90);
        CommunicationInterface::log("Finished processing command: %s. Please enter a new command...", command.c_str());
      }
      else if (command == "forward")
      {
        movementController_.executeAction(MovementAction::MOVE_FORWARD);
        CommunicationInterface::log("Finished processing command: %s. Please enter a new command...", command.c_str());
      } 
      else if(command == "changeOPMode"){
        switch(OPERATION_MODE){
          case OperationMode::FULLY_SIMULATED:
            CommunicationInterface::log("Changing operation mode to SEMI_SIMULATED");
            OPERATION_MODE = OperationMode::SEMI_SIMULATED;
            break;
          case OperationMode::SEMI_SIMULATED:
            CommunicationInterface::log("Changing operation mode to REAL_OPERATION");
            OPERATION_MODE = OperationMode::REAL_OPERATION;
            break;
          case OperationMode::REAL_OPERATION:
            CommunicationInterface::log("Changing operation mode to FULLY_SIMULATED");
            OPERATION_MODE = OperationMode::FULLY_SIMULATED;
            break;
          default:
            CommunicationInterface::log("Unknown operation mode, no change made.");
            break;
        }
        CommunicationInterface::log("Finished processing command: %s. Please enter a new command...", command.c_str());
      } else if(command == "map"){
        movementController_.map3WallsCurrentCell();
        CommunicationInterface::log("Finished processing command: %s. Please enter a new command...", command.c_str());
      } 
    }
    
    delay(10);
  }
}

void MainController::MainControllerLoop(){
  CommunicationInterface::log("============================= STARTING MAIN LOOP =============================");
  while(1){   
    // print simulated maze map
    /*if(sensors_simulated()){
      Serial.println("Simulated Maze Map:");
      movementController_.printSimulatedMazeColored();
      Serial.println("Simulated Maze Map End");
    }*/
    
    // print explored maze map
    /*Serial.println("Maze Map:");
    map_.printMazeColored();
    Serial.println("Maze Map End");*/
    
    switch(currentState_){
      case RatState::INIT:  // Rat checks all walls in current cell (rotates 90° to the left to check wall behind it)
                            CommunicationInterface::log("State: INIT");
                            movementController_.mapStartCell();
                            CommunicationInterface::log("State: EXPLORE");
                            currentState_ = RatState::EXPLORE;
                            break;
      case RatState::EXPLORE: // Rat explores the maze
                            movementController_.map3WallsCurrentCell();
                            int16_t cheese_x, cheese_y;
                            if(map_.maze.doesCheeseExist(cheese_x, cheese_y)){
                              delay(1000); // Wait before returning to start
                              navigationEngine_.setTargetXY(0,0);
                              currentState_ = RatState::RETURN_TO_START_1;
                              CommunicationInterface::log("State: RETURN_TO_START_1");
                            } else if(navigationEngine_.isMapComplete()){
                              delay(1000); // Wait before returning to start
                              navigationEngine_.setTargetXY(0, 0);
                              currentState_ = RatState::RETURN_TO_START_1;
                              CommunicationInterface::log("State: RETURN_TO_START_1");
                            } else{
                              MovementAction nextAction = navigationEngine_.getNextMovementForExploration();
                              if(nextAction != MovementAction::NO_ACTION){
                                movementController_.executeAction(nextAction);
                                movementController_.map3WallsCurrentCell(); // Map the current cell after moving
                              } else{
                                CommunicationInterface::log("No valid movement direction found.");
                                currentState_ = RatState::ERROR;
                              }
                            }
                            break;
      case RatState::RETURN_TO_START_1: // Rat returns to start
                            movementController_.map3WallsCurrentCell();
                            if(navigationEngine_.isTargetReached()){
                              int16_t cheese_x, cheese_y;
                              if(map_.maze.doesCheeseExist(cheese_x, cheese_y)){
                                delay(2000); // Wait before going to cheese
                                navigationEngine_.setTargetXY(cheese_x, cheese_y);
                                currentState_ = RatState::GO_TO_CHEESE;
                                CommunicationInterface::log("State: GO_TO_CHEESE");
                              } else{
                                CommunicationInterface::log("No cheese found. Finished!");
                                currentState_ = RatState::FINISHED;
                              }
                            } else{
                              MovementAction nextAction = navigationEngine_.getNextMovementToTarget();
                              if(nextAction != MovementAction::NO_ACTION){
                                movementController_.executeAction(nextAction);
                                movementController_.map3WallsCurrentCell(); // Map the current cell after moving
                              } else{
                                CommunicationInterface::log("No valid movement direction found.");
                                currentState_ = RatState::ERROR;
                              }
                            }
                            break;
      case RatState::GO_TO_CHEESE: // Rat goes to cheese
                            movementController_.map3WallsCurrentCell();
                            if(navigationEngine_.isTargetReached()){
                              delay(2000); // Wait before returning to start
                              navigationEngine_.setTargetXY(0, 0); // Set target to start position
                              currentState_ = RatState::RETURN_TO_START_2;
                              CommunicationInterface::log("State: RETURN_TO_START_2");
                              commInterface_.pathDrawing(true);
                            } else{
                              MovementAction nextAction = navigationEngine_.getNextMovementToTarget();
                              if(nextAction != MovementAction::NO_ACTION){
                                movementController_.executeAction(nextAction);
                                movementController_.map3WallsCurrentCell(); // Map the current cell after moving
                              } else{
                                CommunicationInterface::log("No valid movement direction found.");
                                currentState_ = RatState::ERROR;
                              }
                            }
                            break;
      case RatState::RETURN_TO_START_2: // Rat returns to start
                            movementController_.map3WallsCurrentCell();
                            if(navigationEngine_.isTargetReached()){
                              CommunicationInterface::log("Start reached! Finished.");
                              currentState_ = RatState::FINISHED;
                              commInterface_.pathDrawing(false);
                            } else{
                              MovementAction nextAction = navigationEngine_.getNextMovementToTarget();
                              if(nextAction != MovementAction::NO_ACTION){
                                movementController_.executeAction(nextAction);
                                movementController_.map3WallsCurrentCell(); // Map the current cell after moving
                              } else{
                                CommunicationInterface::log("No valid movement direction found.");
                                currentState_ = RatState::ERROR;
                              }
                            }
                            break;
      case RatState::FINISHED: // Rat finished
                            CommunicationInterface::log("State: FINISHED");
                            return;
      case RatState::ERROR: // Error state
                            CommunicationInterface::log("State: ERROR");
                            fatalError("ERROR 'MainController::MainControllerLoop': An error occurred.");
                            break;
      default: fatalError("ERROR 'MainController::MainControllerLoop': Unknown state.");      
    }
    
    if(OPERATION_MODE == OperationMode::FULLY_SIMULATED)
      delay(500); // Use less delay in fully simulated mode to speed up the simulation
    else
      delay(1000);
  }
}