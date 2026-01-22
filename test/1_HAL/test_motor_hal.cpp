#include <Arduino.h>
#include "motor_hal.h"

// -- State Variables --
static int16_t target_speed = 0;
static int16_t target_steer = 0;
static unsigned long last_process_time = 0;
static unsigned long last_telemetry_time = 0;

// -- Input Buffer --
static char input_buffer[32];
static int input_idx = 0;

void setup() {
    delay(1500);
    // 1. Initialize PC Serial
    Serial.begin(115200);
    while (!Serial) { delay(10); } // Wait for USB
    Serial.println("[TEST] Starting Motor HAL Test Brain...");

    // 2. Initialize Module under test
    if (HAL_Motor_Init()) {
        Serial.println("[INFO] HAL_Motor_Init success");
    } else {
        Serial.println("[ERROR] HAL_Motor_Init failed");
        while(1); // Halt
    }

    // 3. Enable Auto-Send (Keepalive)
    // This allows the HAL to automatically resend the last command inside HAL_Motor_Process
    HAL_Motor_SetAutoSend(true);
    Serial.println("[INFO] Auto-Send Enabled");

    Serial.println("[HELP] Commands: v<val> (Speed), d<val> (Steer/Dir), x (Stop)");
    Serial.println("[HELP] Example: v100 -> Speed 100 | d50 -> Steer 50 | x -> Stop");
}

void loop() {
    unsigned long now = millis();

    // 1. Process HAL (RX + Auto-TX) @ 200Hz (5ms)
    // Simulates the production control loop frequency.
    // This handles both reading telemetry AND sending the keepalive command.
    if (now - last_process_time >= 5) {
        last_process_time = now;
        HAL_Motor_Process();
    }

    // 2. Non-Blocking Command Parser
    while (Serial.available()) {
        char ch = Serial.read();

        // Handle end of line (Execute command)
        if (ch == '\n' || ch == '\r') {
            if (input_idx > 0) {
                input_buffer[input_idx] = '\0'; // Null-terminate
                char cmd = input_buffer[0];
                int val = atoi(&input_buffer[1]); // Parse integer starting after command char

                bool updated = false;

                if (cmd == 'v') {
                    target_speed = (int16_t)val;
                    Serial.printf("\n[CMD] Set Speed: %d\n", target_speed);
                    updated = true;
                } 
                else if (cmd == 'd') {
                    target_steer = (int16_t)val;
                    Serial.printf("\n[CMD] Set Steer: %d\n", target_steer);
                    updated = true;
                } 
                else if (cmd == 'x') {
                    target_speed = 0;
                    target_steer = 0;
                    Serial.println("\n[CMD] STOP\n");
                    updated = true;
                }
                
                if (updated) {
                    // Update the HAL. It will send immediately AND cache the value 
                    // for the auto-send loop.
                    HAL_Motor_SetControl(target_speed, target_steer);
                }
                
                input_idx = 0; // Reset buffer
            }
        }
        // Handle normal characters
        else {
            if (input_idx < sizeof(input_buffer) - 1) {
                input_buffer[input_idx++] = ch;
            }
        }
    }

    // 3. Print Telemetry (10Hz / 100ms)
    if (now - last_telemetry_time >= 100) {
        last_telemetry_time = now;
        
        HoverMotionData_t feed = HAL_Motor_GetFeedback();
        
        // CSV Format: DATA,CmdSpeed,CmdSteer,MeasSpeedL,MeasSpeedR,Volts,Temp
        Serial.printf("DATA,%d,%d,%d,%d,%.2f,%.1f\r", 
            feed.cmd1, 
            feed.cmd2, 
            feed.speedL_meas, 
            feed.speedR_meas,
            HAL_Motor_GetVoltage(),
            HAL_Motor_GetTemperature()
        );
    }
}