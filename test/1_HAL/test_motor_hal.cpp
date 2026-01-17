#include <Arduino.h>
#include "motor_hal.h"

// -- State Variables --
static int16_t target_speed = 0;
static int16_t target_steer = 0;
static unsigned long last_control_time = 0;
static unsigned long last_telemetry_time = 0;

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

    // 2. Parse Commands from PC
    if (Serial.available()) {
        char cmd = Serial.read();
        
        // Handle value commands
        if (cmd == 'v' || cmd == 'd') {
            int val = Serial.parseInt(); // Blocking read of integer
            if (cmd == 'v') {
                target_speed = (int16_t)val;
                Serial.printf("\n[CMD] Set Speed: %d\n\n", target_speed);
            } else if (cmd == 'd') {
                target_steer = (int16_t)val;
                Serial.printf("\n[CMD] Set Steer: %d\n\n", target_steer);
            }
        } 
        // Handle immediate actions
        else if (cmd == 'x') {
            target_speed = 0;
            target_steer = 0;
            Serial.println("\n[CMD] STOP\n\n");
        }
        
        // Flush newline characters often left by terminals
        while(Serial.peek() == '\n' || Serial.peek() == '\r') {
            Serial.read();
        }
    }

    unsigned long now = millis();

    // 3. Send Control Packets (Keepalive @ 50Hz / 20ms)
    if (now - last_control_time >= 20) {
        last_control_time = now;
        HAL_Motor_SetControl(target_speed, target_steer);
    }

    // 4. Print Telemetry (10Hz / 100ms)
    if (now - last_telemetry_time >= 100) {
        last_telemetry_time = now;
        
        HoverMotionData_t feed = HAL_Motor_GetFeedback();
        
        // CSV Format: DATA,CmdSpeed,CmdSteer,MeasSpeedL,MeasSpeedR,Volts,Temp
        Serial.printf("DATA,%d,%d,%d,%d,%.2f,%.1f\r ", 
            feed.cmd1, 
            feed.cmd2, 
            feed.speedL_meas, 
            feed.speedR_meas,
            HAL_Motor_GetVoltage(),
            HAL_Motor_GetTemperature()
        );
    }
}
