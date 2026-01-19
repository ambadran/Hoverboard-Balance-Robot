#ifndef EEPROM_HAL_H
#define EEPROM_HAL_H

#include <stdbool.h>
#include <stdint.h>
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

// EEPROM Address Map
// Each float takes 4 bytes.
#define EEPROM_ADDR_KP  0x00
#define EEPROM_ADDR_KI  0x04
#define EEPROM_ADDR_KD  0x08

/**
 * @brief Initialize the EEPROM (Flash-based) storage.
 * @return true if initialization was successful, false otherwise.
 */
bool HAL_EEPROM_Init(void);

/**
 * @brief Save a PID parameter structure to EEPROM and Commit to Flash.
 * @param params Pointer to the PID parameters to save.
 */
void HAL_EEPROM_SavePID(const HAL_PID_Params_t* params);

/**
 * @brief Load a PID parameter structure from EEPROM.
 * @param params Pointer to the structure where the loaded parameters will be stored.
 */
void HAL_EEPROM_LoadPID(HAL_PID_Params_t* params);

// --- Granular Accessors ---

/**
 * @brief Write and Commit the Kp value to EEPROM.
 * @param val The new Kp value.
 */
void HAL_EEPROM_SetKp(float val);

/**
 * @brief Read the Kp value from EEPROM.
 * @return The current Kp value.
 */
float HAL_EEPROM_GetKp(void);

/**
 * @brief Write and Commit the Ki value to EEPROM.
 * @param val The new Ki value.
 */
void HAL_EEPROM_SetKi(float val);

/**
 * @brief Read the Ki value from EEPROM.
 * @return The current Ki value.
 */
float HAL_EEPROM_GetKi(void);

/**
 * @brief Write and Commit the Kd value to EEPROM.
 * @param val The new Kd value.
 */
void HAL_EEPROM_SetKd(float val);

/**
 * @brief Read the Kd value from EEPROM.
 * @return The current Kd value.
 */
float HAL_EEPROM_GetKd(void);

#ifdef __cplusplus
}
#endif

#endif // EEPROM_HAL_H