#ifndef TUNER_CONFIG_H
#define TUNER_CONFIG_H

// --- Tuning Configuration Defaults ---

// Input (IMU Pitch) Span
#define TUNER_INPUT_SPAN_MIN  -45.0f
#define TUNER_INPUT_SPAN_MAX   45.0f

// Output (Motor Power) Span
// Assuming +/- 1000 is the max raw PWM/Speed command accepted by Motor HAL
#define TUNER_OUTPUT_SPAN_MIN -1000.0f
#define TUNER_OUTPUT_SPAN_MAX  1000.0f

// Tuning Sequence Parameters
#define TUNER_OUTPUT_START     0.0f
#define TUNER_OUTPUT_STEP      300.0f   // 20% of 1000
#define TUNER_TEST_TIME_SEC    1        // Robot falls in ~1s
#define TUNER_SETTLE_TIME_SEC  0.5f     // Wait 0.5s before step
#define TUNER_SAMPLES          200      // Match loop frequency (200Hz)

// Default Initial Settings
#define TUNER_DEFAULT_RULE     TUNER_Mixed_PID
#define TUNER_DEFAULT_ACTION   TUNER_ACTION_REVERSE
#define TUNER_DEFAULT_SERIAL   TUNER_SERIAL_ALL

#endif // TUNER_CONFIG_H
