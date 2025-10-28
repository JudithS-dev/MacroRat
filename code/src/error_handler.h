#ifndef __ERROR_HANDLER_H__
#define __ERROR_HANDLER_H__

class MotorDriver;

/**
 * @brief global_motor_driver
 * 
 * This global pointer holds the instance of the MotorDriver class.
 * It is used to control the motors and stop them in case of a fatal error.
 */
extern MotorDriver* global_motor_driver;

/**
 * @brief Triggers a fatal system error.
 * 
 * Stops the robot, outputs an error message (if Serial is active),
 * and blinks an LED to visually indicate failure. The system halts
 * in an infinite loop until a manual reset is performed.
 * 
 * @warning This function won't return as it enters an infinite loop.
 * 
 * @note The final formatted message will be truncated after 127 characters
 *       (plus null terminator) if it exceeds the internal buffer size.
 * 
 * @param message A printf-style format string describing the error.
 *                The total message length must not exceed 128 characters.
 * @param ... Optional arguments corresponding to the format string.
 * 
 * @code
 * // Examples:
 * fatalError("ERROR 'some_func': Unexpected null pointer in controller");
 * fatalError("ERROR 'a_func': Maze::setCellState: Invalid coordinates (%d, %d)", row, col);
 * @endcode
 */
void fatalError(const char* message, ...);

#endif // __ERROR_HANDLER_H__