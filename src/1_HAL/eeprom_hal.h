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
#define EEPROM_ADDR_KP         0x00
#define EEPROM_ADDR_KI         0x04
#define EEPROM_ADDR_KD         0x08
#define EEPROM_ADDR_MIN_POWER  0x0C
#define EEPROM_ADDR_DEADBAND   0x10
#define EEPROM_ADDR_LPF_ALPHA  0x14

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

/**
 * @brief Write and Commit the Min Power value to EEPROM.
 * @param val The new Min Power value.
 */
void HAL_EEPROM_SetMinPower(float val);

/**
 * @brief Read the Min Power value from EEPROM.
 * @return The current Min Power value.
 */
float HAL_EEPROM_GetMinPower(void);

/**
 * @brief Write and Commit the Deadband value to EEPROM.
 * @param val The new Deadband value.
 */
void HAL_EEPROM_SetDeadband(float val);

/**
 * @brief Read the Deadband value from EEPROM.
 * @return The current Deadband value.
 */
float HAL_EEPROM_GetDeadband(void);

/**
 * @brief Write and Commit the LPF Alpha value to EEPROM.
 * @param val The new LPF Alpha value.
 */
void HAL_EEPROM_SetLpfAlpha(float val);

/**
 * @brief Read the LPF Alpha value from EEPROM.
 * @return The current LPF Alpha value.
 */
float HAL_EEPROM_GetLpfAlpha(void);

#ifdef __cplusplus
}
#endif

#endif // EEPROM_HAL_H