#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "definitions.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the PID controller.
 * 
 * 1. Loads PID parameters from EEPROM.
 * 2. Runs a Zero-Calibration routine (averages IMU readings) to establish a baseline.
 * 3. Configures the QuickPID instance (Mode, Limits, AntiWindup).
 */
void MID_CONTROL_PID_Init(void);

/**
 * @brief Executes one step of the PID control loop.
 * 
 * @param target_setpoint The desired setpoint (usually 0.0 for balancing).
 * @param pid_output_ptr Pointer to store the calculated PID output.
 * @return True if a new value was computed, false otherwise.
 */
bool MID_CONTROL_PID_Step(float target_setpoint, float *pid_output_ptr);

/**
 * @brief Dynamic Gain Scheduling/Tuning.
 * 
 * Allows changing PID gains during runtime.
 * 
 * @param kp Proportional Gain.
 * @param ki Integral Gain.
 * @param kd Derivative Gain.
 */
void MID_CONTROL_PID_SetGains(float kp, float ki, float kd);

/**
 * @brief Get the latest status of the PID controller.
 * 
 * @return PID_Status_t structure containing current internals (Error, Terms, etc.).
 */
PID_Status_t MID_CONTROL_PID_GetStatus(void);

/**
 * @brief Set the PID Controller Mode.
 * 
 * @param mode The desired mode (MANUAL, AUTOMATIC, TIMER).
 */
void MID_CONTROL_PID_SetMode(MID_PID_Mode_t mode);

/**
 * @brief Set the PID Sample Time.
 * 
 * @param us Sample time in microseconds.
 */
void MID_CONTROL_PID_SetSampleTime(uint32_t us);

/**
 * @brief Get the current PID Sample Time.
 * 
 * @return Sample time in microseconds.
 */
uint32_t MID_CONTROL_PID_GetSampleTime(void);

/**
 * @brief Resets the PID internal state.
 * 
 * Clears the integral term and resets the previous input to the current input.
 * Call this when transitioning from IDLE to BALANCING to prevent derivative kick
 * or integral windup from previous states.
 */
void MID_CONTROL_PID_Reset(void);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
