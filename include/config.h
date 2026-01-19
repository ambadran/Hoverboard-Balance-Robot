#ifndef CONFIG_H
#define CONFIG_H

// --- MPU6050 Gyro/Accel Calibration Offsets ---
// I got these values by running the re-implemented test_calibrate_imu which is a mock IMU_Zero from the electonicscats MPU6050 library.
#define MPU_OFF_X_GYRO      115
#define MPU_OFF_Y_GYRO      76
#define MPU_OFF_Z_GYRO      68
#define MPU_OFF_X_ACCEL     -211
#define MPU_OFF_Y_ACCEL     -831
#define MPU_OFF_Z_ACCEL     1053

// --- PID Configuration ---
// Default Tunings (If EEPROM is empty)
#define PID_DEFAULT_KP      15.0f
#define PID_DEFAULT_KI      0.0f
#define PID_DEFAULT_KD      0.5f

// Output Limits (Motor Command Range, typically -1000 to 1000 or similar)
// Assuming normalized output or direct PWM scale. Let's start with safe limits.
// Hoverboard firmware usually takes -1000 to 1000.
#define PID_OUTPUT_MIN      -400.0f
#define PID_OUTPUT_MAX      400.0f

// Calibration
#define PID_CALIBRATION_SAMPLES 100 // Number of samples to average for zero-offset

// Input Mapping
// This macro maps the logical PID input to a HAL function.
#include "../src/1_HAL/imu_hal.h" // Include header to make the function visible if needed, strictly we just need the name here
#define PID_INPUT_FUNC      HAL_IMU_GetPitch

// Advanced PID Modes
// Proportional on Error (0) or Measurement (1)
// Derivative on Error (0) or Measurement (1)
#define PID_P_MODE          0 // pOnError
#define PID_D_MODE          1 // dOnMeas

// Anti-Windup Mode
// iAwCondition (0) - Conditional Integration (Smart, prevents windup when output saturated)
// iAwClamp (1) - Standard Clamping
// iAwOff (2) - Off
#define PID_AW_MODE         0 

// Loop Timing
#define PID_SAMPLE_TIME_US  5000 // 200Hz

#endif // CONFIG_H