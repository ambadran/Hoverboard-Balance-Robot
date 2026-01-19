#ifndef PLATFORM_ABSTRACTION_H
#define PLATFORM_ABSTRACTION_H

#ifdef ARDUINO
    // --- Target: ESP32 (Arduino Framework) ---
    #include <Arduino.h>
#else
    // --- Host: Linux / Unit Tests ---
    #include <unistd.h> // for usleep
    #include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 
 * Platform Abstraction Layer
 * 
 * This file isolates the Middleware from platform-specific APIs (like Arduino).
 * It enables the Middleware to be compiled and tested on both the Host (Linux/PC)
 * and the Target (ESP32/Arduino) without code changes.
 */

#ifdef ARDUINO
    /**
     * @brief Blocking delay in milliseconds.
     * @note Only allowed in Initialization phases. NEVER in Control Loops.
     */
    #define PLATFORM_DELAY_MS(ms)   delay(ms)

#else
    /**
     * @brief Blocking delay in milliseconds (Host implementation).
     */
    #define PLATFORM_DELAY_MS(ms)   usleep((ms) * 1000)

#endif

#ifdef __cplusplus
}
#endif

#endif // PLATFORM_ABSTRACTION_H