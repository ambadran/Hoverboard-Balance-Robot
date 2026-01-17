#include <Arduino.h>
#include "imu_hal.h"

// State variables for the test
static bool stream_data = false;
static unsigned long last_print_time = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // 10Hz print rate for readability

void print_help() {
    Serial.println("\n--- IMU HAL Test Brain ---");
    Serial.println("Commands:");
    Serial.println("  h: Print this help menu");
    Serial.println("  s: Toggle CSV data streaming (Pitch, Roll)");
    Serial.println("  r: Reset/Re-init IMU");
    Serial.println("--------------------------");
}

void setup() {

    delay(2000);

    // 1. Initialize Serial for Test Interface
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for serial
    Serial.println("\n[INFO] Starting IMU HAL Test...");

    // 2. Initialize the Module Under Test
    if (HAL_IMU_Init()) {
        Serial.println("[INFO] HAL_IMU_Init() SUCCESS.");
    } else {
        Serial.println("[ERROR] HAL_IMU_Init() FAILED. Check wiring/power.");
        while(1); // Stop here if hardware fails
    }

    print_help();
}

void loop() {
    // 1. Core Logic: Must call Update frequently
    HAL_IMU_Update();

    // 2. Command Parsing
    if (Serial.available()) {
        char cmd = Serial.read();
        // Flush remaining characters (like newlines)
        while(Serial.available()) Serial.read();

        switch (cmd) {
            case 'h':
                print_help();
                break;
            case 's':
                stream_data = !stream_data;
                Serial.printf("[INFO] Streaming %s\n", stream_data ? "ENABLED" : "DISABLED");
                if (stream_data) {
                    Serial.println("DATA,Timestamp,Pitch,Roll");
                }
                break;
            case 'r':
                Serial.println("[INFO] Re-initializing...");
                if (HAL_IMU_Init()) {
                    Serial.println("[INFO] Re-init SUCCESS.");
                } else {
                    Serial.println("[ERROR] Re-init FAILED.");
                }
                break;
            default:
                // Ignore newlines/garbage
                if (cmd != '\n' && cmd != '\r') {
                    Serial.printf("[WARN] Unknown command: %c\n", cmd);
                }
                break;
        }
    }

    // 3. Feedback / Data Streaming
    if (stream_data) {
        unsigned long current_time = millis();
        if (current_time - last_print_time >= PRINT_INTERVAL_MS) {
            last_print_time = current_time;
            
            float p, r;
            HAL_IMU_GetOrientation(&p, &r);
            
            // CSV Format: DATA,Timestamp,Pitch,Roll
            // Using \r to refresh line if you were running a terminal that supports it,
            // but for logging/plotting, newlines are usually better.
            // The directive says: "use the '\r' character to refresh the printed values... This would also mean that other print statements need to start with '\n'"
            // Let's follow the directive for the "dashboard" feel, but standard CSV often wants newlines.
            // "if a test will constantly print in terminal, it should use the '\r' character"
            
            // We'll use printf with \r to overwrite the line
            Serial.printf("DATA,%lu,%.2f,%.2f        \r", current_time, p, r);
        }
    }
}
