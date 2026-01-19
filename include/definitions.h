#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Global structure to hold PID tuning parameters.
 *        Used by HAL (EEPROM) and Middleware (Control Logic).
 */
typedef struct {
    float kp;
    float ki;
    float kd;
} HAL_PID_Params_t;

/**
 * @brief Global structure to report PID internal status.
 *        Used for monitoring via telemetry.
 */
typedef struct {
    float setpoint;
    float input;
    float output;
    float error;
    float p_term;
    float i_term;
    float d_term;
} PID_Status_t;

/**
 * @brief PID Operation Modes (C-Compatible Wrapper)
 */
typedef enum {
    MID_PID_MODE_MANUAL = 0,    // Off / Manual Output
    MID_PID_MODE_AUTOMATIC = 1, // On / Compute on every call
    MID_PID_MODE_TIMER = 2      // On / Compute based on internal timer
} MID_PID_Mode_t;

#ifdef __cplusplus
}
#endif

#endif // DEFINITIONS_H
