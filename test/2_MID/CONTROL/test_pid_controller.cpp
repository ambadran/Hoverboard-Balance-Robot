#include <Arduino.h>
#include "pid_controller.h"
#include "../../../src/1_HAL/imu_hal.h"
#include "../../../src/1_HAL/eeprom_hal.h"
#include "../../../src/1_HAL/motor_hal.h"

// --- Test Globals ---
static float target_setpoint = 0.0f;
static bool auto_run = false;
static bool enable_motors = false;

// --- Test Logic ---

void print_help() {
    Serial.println("\n--- PID Controller HIL Test (Real HAL) ---");
    Serial.println("Logic Commands:");
    Serial.println("  kP,I,D  : Set PID Gains (Kp, Ki, Kd)");
    Serial.println("  tVAL    : Set persistent Target Setpoint (e.g. t0.0)");
    Serial.println("  dVAL    : Manual Motor Drive (e.g. d200). Only when Auto-Run OFF.");
    Serial.println("  mMODE   : PID Internal Mode (0=MANUAL, 1=AUTO, 2=TIMER)");
    Serial.println("  stVAL   : PID Sample Time in microseconds (e.g. st5000)");
    Serial.println("");
    Serial.println("Execution Commands:");
    Serial.println("  s       : Single Step (Try to run PID once)");
    Serial.println("  ar<0|1> : Auto-Run Toggle (Poll PID continuously)");
    Serial.println("  dr<0|1> : Motor Drive Toggle (1 = Enable PID->Motor, 0 = Disable)");
    Serial.println("  r       : Re-Initialize PID (Full Recalibration)");
    Serial.println("  z       : Zero/Reset PID State & Motors (Fast Stop)");
    Serial.println("  ?       : Print current internal state");
    Serial.println("  h       : Print this help menu");
}

void setup() {
    Serial.begin(115200);
    // Give time for terminal to connect
    delay(2000); 
    
    print_help();
    
    // Initialize Hardware (According to the wiring map)
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

    if (!HAL_Motor_Init()) {
        Serial.println("[ERROR] Motor Init Failed");
    } else {
        Serial.println("[INFO] Motor Initialized");
    }

    // Initialize PID
    MID_CONTROL_PID_Init();
    // Default to Automatic (Timer Mode in implementation) for our simulated loop
    MID_CONTROL_PID_SetMode(MID_PID_MODE_AUTOMATIC);
    
    Serial.println("[INFO] PID Initialized. Default Mode: AUTOMATIC. Auto-Run: OFF. Motors: OFF");
}

void print_status_csv() {
    PID_Status_t status = MID_CONTROL_PID_GetStatus();
    Serial.printf("STEP,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r", 
        status.setpoint, status.input, status.output, status.error,
        status.p_term, status.i_term, status.d_term);
}

void loop() {
    // Always update HALs
    HAL_IMU_Update();
    HAL_Motor_Process(); // Keep UART feedback buffer clean

    // Handle Auto-Run Logic
    // In Auto-Run, we constantly poll the PID. It decides when to compute based on its internal timer.
    if (auto_run) {
        float output_val = 0.0f;
        if (MID_CONTROL_PID_Step(target_setpoint, &output_val)) {
            // A new PID step was computed!
            
            // Actuate Motors if Enabled
            if (enable_motors) {
                int16_t speed_cmd = (int16_t)output_val;
                HAL_Motor_SetControl(speed_cmd, 0); // 0 Steer for balancing
            } 
            
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
                MID_CONTROL_PID_SetSampleTime(us);
                Serial.printf("[INFO] Sample Time Set: %u us\n", us);
            }
            return;
        }
        
        if (cmdStr.startsWith("ar")) {
            int state = valStr.substring(1).toInt(); // Skip 'r'
            auto_run = (state == 1);
            if (!auto_run) {
                // STOP Logic: When stopping the loop, clear the motor command
                HAL_Motor_SetControl(0, 0);
            }
            Serial.printf("[INFO] Auto-Run: %s. Motors Zeroed.\n", auto_run ? "ON" : "OFF");
            return;
        }

        if (cmdStr.startsWith("dr")) {
            int state = valStr.substring(1).toInt(); // Skip 'r'
            enable_motors = (state == 1);
            if (!enable_motors) {
                // Failsafe: Stop immediately when disabling
                HAL_Motor_SetControl(0, 0);
            }
            Serial.printf("[INFO] Motor Drive: %s. Motors Zeroed.\n", enable_motors ? "ENABLED" : "DISABLED");
            return;
        }

        switch (cmd) {
            case 'h':
                print_help();
                break;

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

            case 'd': // Manual Drive
                if (auto_run) {
                    Serial.println("[ERROR] Cannot drive manually while Auto-Run is ON!");
                } else {
                    int16_t manual_speed = (int16_t)valStr.toInt();
                    HAL_Motor_SetControl(manual_speed, 0);
                    Serial.printf("[INFO] Manual Motor Drive: %d\n", manual_speed);
                }
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

            case 's': // Step (Single Shot)
            {
                // Optional target override: s<Target>
                if (valStr.length() > 0) target_setpoint = valStr.toFloat();
                
                float output_val = 0.0f;
                bool computed = MID_CONTROL_PID_Step(target_setpoint, &output_val);
                
                if (computed) {
                    print_status_csv();
                    // Also drive if enabled, even in single step
                    if (enable_motors) {
                        HAL_Motor_SetControl((int16_t)output_val, 0);
                    }
                    Serial.println(" [COMPUTED]");
                } else {
                    PID_Status_t status = MID_CONTROL_PID_GetStatus();
                    Serial.printf("SKIPPED (Wait for Timer). Last Out: %.2f\n", status.output);
                }
                break;
            }
            
            case 'r': // Reset Full (Re-Init)
                HAL_Motor_SetControl(0, 0); // Safety
                MID_CONTROL_PID_Init();
                Serial.println("[INFO] PID Re-Initialized (Calibration Ran)");
                break;

            case 'z': // Zero/Reset State
                MID_CONTROL_PID_Reset();
                HAL_Motor_SetControl(0, 0); // Safety
                target_setpoint = 0.0f;
                Serial.println("[INFO] PID State Reset (Integral=0). Motors Zeroed. Target=0.");
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
                Serial.printf("[ERROR] Unknown Command: %c\n", cmd);
                break;
        }
    }
}
