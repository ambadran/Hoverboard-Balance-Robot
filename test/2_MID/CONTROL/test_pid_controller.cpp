#include <Arduino.h>
#include "pid_controller.h"
#include "../../../src/1_HAL/imu_hal.h"
#include "../../../src/1_HAL/eeprom_hal.h"
#include "../../../src/1_HAL/motor_hal.h"

// --- Test Globals ---
static float target_setpoint = 0.0f;
static bool auto_run = false;
static bool enable_motors = false;
static uint32_t last_step_time = 0;
static uint32_t step_interval_us = 5000; // Default 5ms (200Hz)

// --- Test Logic ---

void print_help() {
    Serial.println("\n--- PID Controller HIL Test (Real HAL) ---");
    Serial.println("Logic Commands:");
    Serial.println("  kP,I,D  : Set PID Gains (Kp, Ki, Kd)");
    Serial.println("  tVAL    : Set persistent Target Setpoint (e.g. t0.0)");
    Serial.println("  mMODE   : PID Internal Mode (0=MANUAL, 1=AUTO, 2=TIMER)");
    Serial.println("  stVAL   : PID Sample Time in microseconds (e.g. st5000)");
    Serial.println("");
    Serial.println("Execution Commands:");
    Serial.println("  s       : Single Step (Run PID once and stop)");
    Serial.println("  ar<0|1> : Auto-Run Toggle (1 = Start 200Hz Loop, 0 = Stop)");
    Serial.println("  dr<0|1> : Motor Drive Toggle (1 = Enable PID->Motor, 0 = Disable)");
    Serial.println("  r       : Reset/Re-calibrate PID");
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
    // Default to Automatic for our simulated loop
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
    // We simulate the RTOS task frequency here using micros()
    if (auto_run) {
        if (micros() - last_step_time >= step_interval_us) {
            last_step_time = micros();
            
            // Execute Control Logic
            float output_f = MID_CONTROL_PID_Step(target_setpoint);
            
            // Actuate Motors if Enabled
            if (enable_motors) {
                // Safety Clamp typically happens in PID limits, but good to be explicit for int16 conversion
                // PID output is configured to +/- 400 in config.h
                int16_t speed_cmd = (int16_t)output_f;
                HAL_Motor_SetControl(speed_cmd, 0); // 0 Steer for balancing
            } else {
                // Ensure motors are stopped if flag is off but loop is running
                // (Optional: Depends if we want to coast or brake. Usually 0 is safer here)
                // However, we shouldn't spam 0 if we assume they are off.
                // But to be safe, if we toggled off, we might want to send 0 once.
                // For this simple test, we just don't send commands.
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

        if (cmdStr.startsWith("dr")) {
            int state = valStr.substring(1).toInt(); // Skip 'r'
            enable_motors = (state == 1);
            if (!enable_motors) {
                // Failsafe: Stop immediately when disabling
                HAL_Motor_SetControl(0, 0);
            }
            Serial.printf("[INFO] Motor Drive: %s\n", enable_motors ? "ENABLED" : "DISABLED");
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
                
                float output = MID_CONTROL_PID_Step(target_setpoint);
                print_status_csv();
                
                // Also drive if enabled, even in single step
                if (enable_motors) {
                    HAL_Motor_SetControl((int16_t)output, 0);
                }
                
                Serial.println(); // Newline
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
                Serial.printf("[ERROR] Unknown Command: %c\n", cmd);
                break;
        }
    }
}
