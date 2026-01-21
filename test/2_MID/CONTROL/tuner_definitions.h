#ifndef TUNER_DEFINITIONS_H
#define TUNER_DEFINITIONS_H

#include <Arduino.h>

// Tuning Rules (Methods)
// These map to sTune::TuningMethod (usually defined in the class)
// We define our own to map to integer commands easily.
typedef enum {
    TUNER_ZN_PID = 0,
    TUNER_DampedOsc_PID = 1,
    TUNER_NoOvershoot_PID = 2,
    TUNER_CohenCoon_PID = 3,
    TUNER_Mixed_PID = 4,
    TUNER_ZN_PI = 5,
    TUNER_DampedOsc_PI = 6,
    TUNER_NoOvershoot_PI = 7,
    TUNER_CohenCoon_PI = 8,
    TUNER_Mixed_PI = 9
} Tuner_Rule_t;

// Serial Modes
typedef enum {
    TUNER_SERIAL_OFF = 0,
    TUNER_SERIAL_ALL = 1,
    TUNER_SERIAL_SUMMARY = 2,
    TUNER_SERIAL_DEBUG = 3
} Tuner_SerialMode_t;

// Controller Action
typedef enum {
    TUNER_ACTION_DIRECT = 0, // DirectIP
    TUNER_ACTION_DIRECT_5T = 1,
    TUNER_ACTION_REVERSE = 2, // ReverseIP
    TUNER_ACTION_REVERSE_5T = 3
} Tuner_Action_t;

#endif // TUNER_DEFINITIONS_H
