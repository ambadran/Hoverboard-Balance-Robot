#include <Arduino.h>
#include "pid_controller.h"
#include "../../../src/1_HAL/imu_hal.h"
#include "../../../src/1_HAL/eeprom_hal.h"
#include "../../../src/1_HAL/motor_hal.h"

// --- Test Globals ---
static float target_setpoint = 0.0f;
static bool auto_run = false;
static bool enable_motors = false;

// PID Gains State (Mirrored from Controller/EEPROM for Saving)
static float current_kp = 0.0f;
static float current_ki = 0.0f;
static float current_kd = 0.0f;

// Safety Timeout
static unsigned long auto_run_stop_time = 0;
static unsigned long run_duration = 5000; // Default 5 seconds
static unsigned long start_delay_ms = 2000; // Default 3 seconds

// Output Format
static bool use_csv_format = false;

// Jitter Monitor Globals
static uint32_t max_loop_time_us = 0;
static uint32_t min_loop_time_us = 0xFFFFFFFF;
static uint32_t loop_count = 0;
static uint64_t total_loop_time_us = 0;
static uint32_t last_step_micros = 0;

// --- Test Logic ---

void print_help() {
    Serial.println("\n--- PID Controller HIL Test (Real HAL) ---");
    Serial.println("Logic Commands:");
    Serial.println("  kpVAL   : Set Proportional Gain (e.g. kp1.5)");
    Serial.println("  kiVAL   : Set Integral Gain (e.g. ki0.1)");
    Serial.println("  kdVAL   : Set Derivative Gain (e.g. kd0.05)");
    Serial.println("  kP,I,D  : Set all PID Gains at once (Legacy)");
    Serial.println("  tVAL    : Set persistent Target Setpoint (e.g. t0.0)");
    Serial.println("  trVAL   : Set Auto-Run Duration in seconds (e.g. tr2.5)");
    Serial.println("  sdVAL   : Set START Delay in seconds (e.g. sd3.0)");
    Serial.println("  dVAL    : Manual Motor Drive (e.g. d200). Only when Auto-Run OFF.");
    Serial.println("  mMODE   : PID Internal Mode (0=MANUAL, 1=AUTO, 2=TIMER)");
    Serial.println("  stVAL   : PID Sample Time in microseconds (e.g. st5000)");
    Serial.println("");
    Serial.println("Execution Commands:");
    Serial.println("  s       : Single Step (Try to run PID once)");
    Serial.println("  ar      : START Auto-Run (Runs for 'tr' duration, then Stops)");
    Serial.println("  dr<0|1> : Motor Drive Toggle (1 = Enable PID->Motor, 0 = Disable)");
    Serial.println("  r       : Re-Initialize PID (Full Recalibration)");
    Serial.println("  S       : SAVE current PID gains to EEPROM");
    Serial.println("  L       : LOAD/VIEW PID gains currently in EEPROM");
    Serial.println("  v       : Toggle Output Format (CSV <-> Human Readable)");
    Serial.println("  b       : Print Battery Voltage");
    Serial.println("  z       : Zero/Reset PID State & Motors (Fast Stop)");
    Serial.println("  ?       : Print current internal state");
    Serial.println("  h       : Print this help menu");
}

void setup() {
    Serial.begin(115200);
    while(!Serial) { delay(10); }; // Wait for serial to be ready
    delay(1000);
 
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
    
    // Sync Local Gains with EEPROM (what Init loaded)
    current_kp = HAL_EEPROM_GetKp();
    current_ki = HAL_EEPROM_GetKi();
    current_kd = HAL_EEPROM_GetKd();

    // Default to Automatic (Timer Mode in implementation) for our simulated loop
    MID_CONTROL_PID_SetMode(MID_PID_MODE_AUTOMATIC);
    
    Serial.println("[INFO] PID Initialized. Default Mode: AUTOMATIC. Auto-Run: OFF. Motors: OFF");
}

void print_status() {
    PID_Status_t status = MID_CONTROL_PID_GetStatus();
    if (use_csv_format) {
        Serial.printf("STEP,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", 
            status.setpoint, status.input, status.output, status.error,
            status.p_term, status.i_term, status.d_term);
    } else {
        Serial.printf("Tgt:%6.2f | In:%6.2f | Out:%6.2f \t| Err:%6.2f | P:%5.2f I:%5.2f D:%5.2f\r\n", 
            status.setpoint, status.input, status.output, status.error,
            status.p_term, status.i_term, status.d_term);
    }
}

void loop() {
    // Always update HALs
    HAL_IMU_Update();
    HAL_Motor_Process(); // Keep UART feedback buffer clean

    // Handle Auto-Run Logic
    // In Auto-Run, we constantly poll the PID. It decides when to compute based on its internal timer.
    if (auto_run) {
        // Safety Timeout Check
        if (millis() > auto_run_stop_time) {
            auto_run = false;
            HAL_Motor_SetControl(0, 0);
            Serial.println("\n[INFO] Auto-Run Timeout Reached. Motors Stopped.");
        } else {
            float output_val = 0.0f;
            if (MID_CONTROL_PID_Step(target_setpoint, &output_val)) {
                // A new PID step was computed!
                uint32_t now = micros();
                uint32_t dt = now - last_step_micros;
                last_step_micros = now;

                // Update Stats (Skip first loop to avoid initialization noise)
                if (loop_count > 0) {
                    if (dt > max_loop_time_us) max_loop_time_us = dt;
                    if (dt < min_loop_time_us) min_loop_time_us = dt;
                    total_loop_time_us += dt;
                }
                loop_count++;
                
                // Actuate Motors if Enabled
                if (enable_motors) {
                    int16_t speed_cmd = (int16_t)output_val;
                    HAL_Motor_SetControl(speed_cmd, 0); // 0 Steer for balancing
                } 
                
                // Downsample Serial Output to avoid blocking
                // PID Loop = 200Hz. Print at ~10Hz (Every 20th step).
                // 115200 baud cannot sustain 200Hz printing of long strings.
                static uint8_t print_divider = 0;
                if (++print_divider >= 20) {
                    print_divider = 0;
                    print_status();
                }
            }
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
                Serial.printf("[INFO] Sample Time Set: %u us\r\n", us);
            }
            return;
        }

        if (cmdStr.startsWith("kp")) {
            current_kp = valStr.substring(1).toFloat();
            MID_CONTROL_PID_SetGains(current_kp, current_ki, current_kd);
            Serial.printf("[INFO] Kp Set: %.2f (Current: P=%.2f I=%.2f D=%.2f)\r\n", current_kp, current_kp, current_ki, current_kd);
            return;
        }

        if (cmdStr.startsWith("ki")) {
            current_ki = valStr.substring(1).toFloat();
            MID_CONTROL_PID_SetGains(current_kp, current_ki, current_kd);
            Serial.printf("[INFO] Ki Set: %.2f (Current: P=%.2f I=%.2f D=%.2f)\r\n", current_ki, current_kp, current_ki, current_kd);
            return;
        }

        if (cmdStr.startsWith("kd")) {
            current_kd = valStr.substring(1).toFloat();
            MID_CONTROL_PID_SetGains(current_kp, current_ki, current_kd);
            Serial.printf("[INFO] Kd Set: %.2f (Current: P=%.2f I=%.2f D=%.2f)\r\n", current_kd, current_kp, current_ki, current_kd);
            return;
        }
        
        if (cmdStr.startsWith("tr")) {
            String val = valStr.substring(1); // Everything after 'tr'
            if (val.length() == 0) {
                Serial.printf("[INFO] Current Auto-Run Duration: %.2f seconds (%lu ms)\r\n", (float)run_duration / 1000.0f, run_duration);
            } else {
                float duration_sec = val.toFloat();
                if (duration_sec > 0) {
                    run_duration = (unsigned long)(duration_sec * 1000.0f);
                    Serial.printf("[INFO] Auto-Run Duration Set: %.2f seconds (%lu ms)\r\n", duration_sec, run_duration);
                } else {
                     Serial.println("[ERROR] Duration must be > 0");
                }
            }
            return;
        }

        if (cmdStr.startsWith("sd")) {
            String val = valStr.substring(1); // Everything after 'sd'
            if (val.length() == 0) {
                Serial.printf("[INFO] Current Start Delay: %.2f seconds (%lu ms)\r\n", (float)start_delay_ms / 1000.0f, start_delay_ms);
            } else {
                float delay_sec = val.toFloat();
                if (delay_sec >= 0) {
                    start_delay_ms = (unsigned long)(delay_sec * 1000.0f);
                    Serial.printf("[INFO] Start Delay Set: %.2f seconds (%lu ms)\r\n", delay_sec, start_delay_ms);
                } else {
                     Serial.println("[ERROR] Delay must be >= 0");
                }
            }
            return;
        }
        
        if (cmdStr.startsWith("ar")) {
            // Countdown Sequence
            if (start_delay_ms > 0) {
                Serial.printf("[INFO] Starting in %.1f seconds...\r\n", (float)start_delay_ms / 1000.0f);
                unsigned long start_wait = millis();
                unsigned long remaining = start_delay_ms;
                
                while (millis() - start_wait < start_delay_ms) {
                    // Update HALs during wait to keep buffers clean
                    HAL_IMU_Update();
                    HAL_Motor_Process();
                    
                    // Simple integer countdown for visibility
                    unsigned long elapsed = millis() - start_wait;
                    unsigned long current_remaining = start_delay_ms - elapsed;
                    
                    if (current_remaining < remaining && (current_remaining % 1000 == 0)) {
                         Serial.printf("... %lu\r\n", current_remaining / 1000 + 1);
                         remaining = current_remaining;
                    }
                }
                Serial.println("GO!");
            }

            // START Logic: Reset PID internals (Integral=0) for a clean run
            MID_CONTROL_PID_Reset();

            // Reset Jitter Metrics
            max_loop_time_us = 0;
            min_loop_time_us = 0xFFFFFFFF;
            loop_count = 0;
            total_loop_time_us = 0;
            last_step_micros = micros();
            
            auto_run = true;
            auto_run_stop_time = millis() + run_duration;
            
            Serial.printf("[INFO] Auto-Run STARTED. Duration: %lu ms. PID Reset.\r\n", run_duration);
            return;
        }

        if (cmdStr.startsWith("dr")) {
            int state = valStr.substring(1).toInt(); // Skip 'r'
            enable_motors = (state == 1);
            if (!enable_motors) {
                // Failsafe: Stop immediately when disabling
                HAL_Motor_SetControl(0, 0);
            }
            Serial.printf("[INFO] Motor Drive: %s. Motors Zeroed.\r\n", enable_motors ? "ENABLED" : "DISABLED");
            return;
        }

        switch (cmd) {
            case 'h':
                print_help();
                break;

            case 'v': // Toggle Output Format
                use_csv_format = !use_csv_format;
                Serial.printf("[INFO] Output Format: %s\r\n", use_csv_format ? "CSV" : "HUMAN READABLE");
                break;
            
            case 'b': // Battery Voltage
                Serial.printf("[INFO] Battery Voltage: %.2f V\r\n", HAL_Motor_GetVoltage());
                break;

            case 'S': // Save to EEPROM
            {
                HAL_PID_Params_t params;
                params.kp = current_kp;
                params.ki = current_ki;
                params.kd = current_kd;
                HAL_EEPROM_SavePID(&params);
                Serial.printf("[INFO] Saved Tunings to EEPROM: P=%.2f I=%.2f D=%.2f\r\n", 
                    params.kp, params.ki, params.kd);
                break;
            }

            case 'L': // Load/View from EEPROM
            {
                HAL_PID_Params_t stored;
                HAL_EEPROM_LoadPID(&stored);
                
                // Get truth from the PID controller instance
                float active_kp = MID_CONTROL_PID_GetKp();
                float active_ki = MID_CONTROL_PID_GetKi();
                float active_kd = MID_CONTROL_PID_GetKd();

                Serial.println("[INFO] PID Gains Transparency Check:");
                Serial.printf("  ACTIVE (RAM): Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", active_kp, active_ki, active_kd);
                Serial.printf("  STORED (ROM): Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", stored.kp, stored.ki, stored.kd);
                break;
            }

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
                    
                    // Update Local Shadow Copies
                    current_kp = kp;
                    current_ki = ki;
                    current_kd = kd;
                    
                    Serial.printf("[INFO] Gains Set: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
                } else {
                    Serial.println("[ERROR] Invalid format. Use kP,I,D");
                }
                break;
            }
            
            case 't': // Set/Get Target
                if (valStr.length() == 0) {
                    Serial.printf("[INFO] Current Target Setpoint: %.2f\r\n", target_setpoint);
                } else {
                    target_setpoint = valStr.toFloat();
                    Serial.printf("[INFO] Target Set: %.2f\r\n", target_setpoint);
                }
                break;

            case 'd': // Manual Drive
                if (auto_run) {
                    Serial.println("[ERROR] Cannot drive manually while Auto-Run is ON!");
                } else {
                    int16_t manual_speed = (int16_t)valStr.toInt();
                    HAL_Motor_SetControl(manual_speed, 0);
                    Serial.printf("[INFO] Manual Motor Drive: %d\r\n", manual_speed);
                }
                break;
            
            case 'm': // Set/Get Mode
            {
                if (valStr.length() == 0) {
                    MID_PID_Mode_t mode = MID_CONTROL_PID_GetMode();
                    Serial.printf("[INFO] Current PID Mode: %d (%s)\r\n", 
                        (int)mode, 
                        (mode == 0) ? "MANUAL" : (mode == 1 ? "AUTOMATIC" : "TIMER"));
                } else {
                    int mode = valStr.toInt();
                    if (mode >= 0 && mode <= 2) {
                        MID_CONTROL_PID_SetMode((MID_PID_Mode_t)mode);
                        Serial.printf("[INFO] Mode Set: %d (%s)\r\n", 
                            mode, 
                            (mode == 0) ? "MANUAL" : (mode == 1 ? "AUTOMATIC" : "TIMER"));
                    } else {
                        Serial.println("[ERROR] Invalid Mode. Use 0, 1, or 2\r\n");
                    }
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
                    print_status();
                    // Also drive if enabled, even in single step
                    if (enable_motors) {
                        HAL_Motor_SetControl((int16_t)output_val, 0);
                    }
                    Serial.println(" [COMPUTED]");
                } else {
                    PID_Status_t status = MID_CONTROL_PID_GetStatus();
                    Serial.printf("SKIPPED (Wait for Timer). Last Out: %.2f\r\n", status.output);
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
                
            case '?': // Status & Jitter Metrics
            {
                PID_Status_t status = MID_CONTROL_PID_GetStatus();
                uint32_t avg = (loop_count > 1) ? (uint32_t)(total_loop_time_us / (loop_count - 1)) : 0;
                
                Serial.printf("\r\n--- Jitter Monitor Stats ---\r\n");
                Serial.printf("  Sample Count: %lu\r\n", (unsigned long)loop_count);
                Serial.printf("  Target dt   : %lu us (200Hz)\r\n", (unsigned long)MID_CONTROL_PID_GetSampleTime());
                Serial.printf("  Max dt      : %lu us\r\n", (unsigned long)max_loop_time_us);
                Serial.printf("  Min dt      : %lu us\r\n", (unsigned long)min_loop_time_us);
                Serial.printf("  Avg dt      : %lu us\r\n", (unsigned long)avg);
                Serial.printf("---------------------------\r\n");
                Serial.printf("STATUS: Tgt=%.2f In=%.2f Out=%.2f Err=%.2f | P=%.2f I=%.2f D=%.2f\r\n", 
                    status.setpoint, status.input, status.output, status.error,
                    status.p_term, status.i_term, status.d_term);
                break;
            }
            
            default:
                Serial.printf("[ERROR] Unknown Command: %c\r\n", cmd);
                break;
        }
    }
}
