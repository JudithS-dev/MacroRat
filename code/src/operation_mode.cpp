#include "operation_mode.h"

OperationMode OPERATION_MODE = OperationMode::REAL_OPERATION; ///< Operation mode of the system

/**
 * @brief Check if the sensors are simulated.
 * @return True if the sensors are simulated, false otherwise.
 */
bool sensors_simulated() {
  return OPERATION_MODE == OperationMode::FULLY_SIMULATED || OPERATION_MODE == OperationMode::SEMI_SIMULATED;
}

/**
 * @brief Check if the movement is simulated.
 * @return True if the movement is simulated, false otherwise.
 */
bool movement_simulated() {
  return OPERATION_MODE == OperationMode::FULLY_SIMULATED;
}