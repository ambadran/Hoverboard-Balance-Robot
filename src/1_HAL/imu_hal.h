#ifndef IMU_HAL_H
#define IMU_HAL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MPU6050, I2C bus, and DMP.
 * @return true if initialization and DMP setup were successful, false otherwise.
 */
bool HAL_IMU_Init(void);

/**
 * @brief Polls the MPU6050 DMP FIFO.
 *        Should be called frequently (e.g., in the control loop).
 *        If a full packet is received, it updates the internal orientation state.
 * @return true if new data was read and state updated, false otherwise.
 */
bool HAL_IMU_Update(void);

/**
 * @brief Get the latest known Pitch angle.
 * @return Pitch in Degrees.
 */
float HAL_IMU_GetPitch(void);

/**
 * @brief Get the latest known Roll angle.
 * @return Roll in Degrees.
 */
float HAL_IMU_GetRoll(void);

/**
 * @brief Get both Pitch and Roll simultaneously.
 * @param pitch Pointer to float to store pitch (Degrees).
 * @param roll Pointer to float to store roll (Degrees).
 */
void HAL_IMU_GetOrientation(float* pitch, float* roll);

#ifdef __cplusplus
}
#endif

#endif // IMU_HAL_H
