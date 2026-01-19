#include <Arduino.h>
#include "pid_controller.h"
#include "../../../src/1_HAL/imu_hal.h"
#include "../../../src/1_HAL/eeprom_hal.h"

// --- Test Globals ---
static float target_setpoint = 0.0f;
static bool auto_run = false;
static uint32_t last_step_time = 0;
static uint32_t step_interval_us = 5000; // Default 5ms (200Hz)

// --- Test Logic ---

void setup() {
    Serial.begin(115200);
    // Give time for terminal to connect
    delay(2000); 
    
    Serial.println("--- PID Controller Unit Test (Real HAL) ---");
    Serial.println("Commands:");
    Serial.println("  kP,I,D  : Set Gains (e.g., k10.5,0.1,0.5)");
    Serial.println("  tVAL    : Set Target Setpoint (e.g., t0.0)");
    Serial.println("  mMODE   : Set Mode (m0=MANUAL, m1=AUTO, m2=TIMER)");
    Serial.println("  stVAL   : Set Sample Time in us (e.g. st5000)");
    Serial.println("  ar0/1   : Toggle Auto-Run at Sample Time frequency");
    Serial.println("  s       : Step PID (Single Shot)");
    Serial.println("  r       : Re-Initialize PID");
    Serial.println("  ?       : Print Status");
    
    // Initialize Hardware
    if (!HAL_EEPROM_Init()) {
        Serial.println("[ERROR] EEPROM Init Failed");
    } else {
        Serial.println("[INFO] EEPROM Initialized");
    }

    if (!HAL_IMU_Init()) {
        Serial.println("[ERROR] IMU Init Failed");
    } else {
        Serial.println("[INFO] IMU Initialized");
    }

    // Initialize PID
    MID_CONTROL_PID_Init();
    // Default to Automatic for our simulated loop
    MID_CONTROL_PID_SetMode(MID_PID_MODE_AUTOMATIC);
    
    Serial.println("[INFO] PID Initialized. Default Mode: AUTOMATIC. Auto-Run: OFF");
}

void print_status_csv() {
    PID_Status_t status = MID_CONTROL_PID_GetStatus();
    Serial.printf("STEP,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r", 
        status.setpoint, status.input, status.output, status.error,
        status.p_term, status.i_term, status.d_term);
}

void loop() {
    // Always update IMU so FIFO doesn't overflow
    HAL_IMU_Update();

    // Handle Auto-Run Logic
    // We simulate the RTOS task frequency here using micros()
    if (auto_run) {
        if (micros() - last_step_time >= step_interval_us) {
            last_step_time = micros();
            MID_CONTROL_PID_Step(target_setpoint);
            print_status_csv();
        }
    }

    // Handle Serial Commands
    if (Serial.available()) {
        String cmdStr = Serial.readStringUntil('\n');
        cmdStr.trim();
        
        if (cmdStr.length() == 0) return; 
        
        char cmd = cmdStr.charAt(0);
        String valStr = cmdStr.substring(1);
        
        // Handle 2-char commands
        if (cmdStr.startsWith("st")) {
            uint32_t us = valStr.substring(1).toInt(); // Skip 't'
            if (us > 0) {
                step_interval_us = us;
                MID_CONTROL_PID_SetSampleTime(us);
                Serial.printf("[INFO] Sample Time Set: %u us\n", us);
            }
            return;
        }
        
        if (cmdStr.startsWith("ar")) {
            int state = valStr.substring(1).toInt(); // Skip 'r'
            auto_run = (state == 1);
            Serial.printf("[INFO] Auto-Run: %s\n", auto_run ? "ON" : "OFF");
            return;
        }

        switch (cmd) {
            case 'k': // Set Gains kP,I,D
            {
                // Parse CSV
                int firstComma = valStr.indexOf(',');
                int secondComma = valStr.indexOf(',', firstComma + 1);
                
                if (firstComma > 0 && secondComma > 0) {
                    float kp = valStr.substring(0, firstComma).toFloat();
                    float ki = valStr.substring(firstComma + 1, secondComma).toFloat();
                    float kd = valStr.substring(secondComma + 1).toFloat();
                    
                    MID_CONTROL_PID_SetGains(kp, ki, kd);
                    Serial.printf("[INFO] Gains Set: Kp=%.2f, Ki=%.2f, Kd=%.2f\n", kp, ki, kd);
                } else {
                    Serial.println("[ERROR] Invalid format. Use kP,I,D");
                }
                break;
            }
            
            case 't': // Set Target
                target_setpoint = valStr.toFloat();
                Serial.printf("[INFO] Target Set: %.2f\n", target_setpoint);
                break;
            
            case 'm': // Set Mode
            {
                int mode = valStr.toInt();
                if (mode >= 0 && mode <= 2) {
                    MID_CONTROL_PID_SetMode((MID_PID_Mode_t)mode);
                    Serial.printf("[INFO] Mode Set: %d (0=MAN, 1=AUTO, 2=TMR)\n", mode);
                }
                break;
            }

            case 's': // Step (Manual Single Shot)
            {
                // Optional target override: s<Target>
                if (valStr.length() > 0) target_setpoint = valStr.toFloat();
                
                MID_CONTROL_PID_Step(target_setpoint);
                print_status_csv();
                Serial.println(); // Newline for manual step to see history
                break;
            }
            
            case 'r': // Reset
                MID_CONTROL_PID_Init();
                Serial.println("[INFO] PID Re-Initialized (Calibration Ran)");
                break;
                
            case '?': // Status
            {
                PID_Status_t status = MID_CONTROL_PID_GetStatus();
                Serial.printf("STATUS: Tgt=%.2f In=%.2f Out=%.2f Err=%.2f | P=%.2f I=%.2f D=%.2f\n", 
                    status.setpoint, status.input, status.output, status.error,
                    status.p_term, status.i_term, status.d_term);
                break;
            }
            
            default:
                Serial.println("[ERROR] Unknown Command");
                break;
        }
    }
}