#include <Arduino.h>
#include "motor_hal.h"

// -- State Variables --
static int16_t target_speed = 0;
static int16_t target_steer = 0;
static unsigned long last_control_time = 0;
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

    Serial.println("[HELP] Commands: v<val> (Speed), d<val> (Steer/Dir), x (Stop)");
    Serial.println("[HELP] Example: v100 -> Speed 100 | d50 -> Steer 50 | x -> Stop");
}

void loop() {
    // 1. Process Incoming Data from Hoverboard
    HAL_Motor_Process();

    // 2. Non-Blocking Command Parser
    while (Serial.available()) {
        char ch = Serial.read();

        // Handle end of line (Execute command)
        if (ch == '\n' || ch == '\r') {
            if (input_idx > 0) {
                input_buffer[input_idx] = '\0'; // Null-terminate
                char cmd = input_buffer[0];
                int val = atoi(&input_buffer[1]); // Parse integer starting after command char

                if (cmd == 'v') {
                    target_speed = (int16_t)val;
                    Serial.printf("\n[CMD] Set Speed: %d\n", target_speed);
                } 
                else if (cmd == 'd') {
                    target_steer = (int16_t)val;
                    Serial.printf("\n[CMD] Set Steer: %d\n", target_steer);
                } 
                else if (cmd == 'x') {
                    target_speed = 0;
                    target_steer = 0;
                    Serial.println("\n[CMD] STOP\n");
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

    unsigned long now = millis();

    // 3. Send Control Packets (Keepalive @ 50Hz / 20ms)
    // CRITICAL: This must run constantly. If we stop sending for > ~500ms, 
    // the hoverboard firmware will trigger a safety timeout and stop.
    if (now - last_control_time >= 20) {
        last_control_time = now;
        HAL_Motor_SetControl(target_speed, target_steer);
    }

    // 4. Print Telemetry (10Hz / 100ms)
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
