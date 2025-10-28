#ifndef __OPERATION_MODE_H__
#define __OPERATION_MODE_H__

enum class OperationMode {
  FULLY_SIMULATED = 0, ///< sensor data and movement are simulated
  SEMI_SIMULATED,      ///< wall sensor data is simulated, gyroscope sensor data and movement is real
  REAL_OPERATION,      ///< sensor data and movement are real
};

extern OperationMode OPERATION_MODE; ///< Global variable to store the current operation mode

bool sensors_simulated();
bool movement_simulated();

#endif // __OPERATION_MODE_H__