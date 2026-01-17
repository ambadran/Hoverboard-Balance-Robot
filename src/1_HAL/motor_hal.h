#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// -- Data Structures --

/**
 * @brief Raw Feedback structure matching the Hoverboard UART protocol.
 * All fields are raw integers directly from the line.
 */
typedef struct {
    int16_t cmd1;           // Commanded Speed (echoed)
    int16_t cmd2;           // Commanded Steer (echoed)
    int16_t speedR_meas;    // Measured Speed Right
    int16_t speedL_meas;    // Measured Speed Left
    int16_t batVoltage;     // Battery Voltage (ADC value, usually Volts * 100)
    int16_t boardTemp;      // Board Temperature (ADC/Decidegrees, usually DegC * 10)
    uint16_t cmdLed;        // LED command state
} HoverMotionData_t;

// -- API Functions --

/**
 * @brief Initializes the UART connection to the motor controller.
 * @return true if initialization successful (Serial port opened), false otherwise.
 */
bool HAL_Motor_Init(void);

/**
 * @brief Sends speed and steer commands to the motor controller.
 * 
 * @param speed_cmd Desired speed (-1000 to 1000).
 * @param steer_cmd Desired steer/turn rate (-1000 to 1000).
 */
void HAL_Motor_SetControl(int16_t speed_cmd, int16_t steer_cmd);

/**
 * @brief Processing function to be called periodically (e.g., in the control loop).
 * Reads bytes from the UART buffer and updates the internal feedback state.
 */
void HAL_Motor_Process(void);

/**
 * @brief Get the full feedback structure.
 * @return Copy of the latest valid HoverMotionData_t.
 */
HoverMotionData_t HAL_Motor_GetFeedback(void);

/**
 * @brief Get the latest battery voltage in Volts.
 * @return Voltage as float (e.g., 36.5).
 */
float HAL_Motor_GetVoltage(void);

/**
 * @brief Get the latest board temperature in Celsius.
 * @return Temperature as float (e.g., 40.2).
 */
float HAL_Motor_GetTemperature(void);

/**
 * @brief Get the average measured speed (raw units).
 * @return Average of Left and Right measured speeds.
 */
int16_t HAL_Motor_GetAverageSpeed(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_HAL_H
