#include <Arduino.h>
#include <sTune.h>
#include "tuner_config.h"
#include "tuner_definitions.h"
#include "../../../src/1_HAL/imu_hal.h"
#include "../../../src/1_HAL/eeprom_hal.h"
#include "../../../src/1_HAL/motor_hal.h"
#include "../../../include/config.h"

// --- Globals ---
float input_pitch = 0.0f;
float output_motor = 0.0f;
float imu_offset = 0.0f;
bool is_tuning_active = false;
int16_t manual_speed_cmd = 0; // Persistent manual command
uint32_t last_motor_write_time = 0; // For 200Hz Motor Pacing

sTune tuner = sTune(&input_pitch, &output_motor, tuner.Mixed_PID, tuner.directIP, tuner.printALL);

// --- Helper Functions ---

void calibrate_imu() {
    Serial.println("\n[INFO] Calibrating IMU... KEEP ROBOT STILL!");
    
    float sum = 0.0f;
    int samples = PID_CALIBRATION_SAMPLES;
    int count = 0;
    
    while (count < samples) {
        if (HAL_IMU_Update()) {
            sum += HAL_IMU_GetPitch();
            count++;
            if (count % 20 == 0) Serial.print("."); 
        }
        delay(5); // ~200Hz
    }
    
    imu_offset = sum / (float)samples;
    Serial.printf("\n[INFO] Calibration Complete. Samples: %d, Offset: %.2f\n", samples, imu_offset);
}

void apply_tuning_method(uint8_t method_idx) {
    switch(method_idx) {
        case TUNER_ZN_PID: tuner.SetTuningMethod(tuner.ZN_PID); break;
        case TUNER_DampedOsc_PID: tuner.SetTuningMethod(tuner.DampedOsc_PID); break;
        case TUNER_NoOvershoot_PID: tuner.SetTuningMethod(tuner.NoOvershoot_PID); break;
        case TUNER_CohenCoon_PID: tuner.SetTuningMethod(tuner.CohenCoon_PID); break;
        case TUNER_Mixed_PID: tuner.SetTuningMethod(tuner.Mixed_PID); break;
        case TUNER_ZN_PI: tuner.SetTuningMethod(tuner.ZN_PI); break;
        case TUNER_DampedOsc_PI: tuner.SetTuningMethod(tuner.DampedOsc_PI); break;
        case TUNER_NoOvershoot_PI: tuner.SetTuningMethod(tuner.NoOvershoot_PI); break;
        case TUNER_CohenCoon_PI: tuner.SetTuningMethod(tuner.CohenCoon_PI); break;
        case TUNER_Mixed_PI: tuner.SetTuningMethod(tuner.Mixed_PI); break;
        default: Serial.println("[ERROR] Invalid Method Index"); return;
    }
    Serial.printf("[INFO] Tuning Method Set: %d\n", method_idx);
}

void apply_serial_mode(uint8_t mode_idx) {
    switch(mode_idx) {
        case TUNER_SERIAL_OFF: tuner.SetSerialMode(tuner.printOFF); break;
        case TUNER_SERIAL_ALL: tuner.SetSerialMode(tuner.printALL); break;
        case TUNER_SERIAL_SUMMARY: tuner.SetSerialMode(tuner.printSUMMARY); break;
        case TUNER_SERIAL_DEBUG: tuner.SetSerialMode(tuner.printDEBUG); break;
        default: Serial.println("[ERROR] Invalid Serial Mode"); return;
    }
    Serial.printf("[INFO] Serial Mode Set: %d\n", mode_idx);
}

void apply_action(uint8_t action_idx) {
    switch(action_idx) {
        case TUNER_ACTION_DIRECT: tuner.SetControllerAction(tuner.directIP); break;
        case TUNER_ACTION_DIRECT_5T: tuner.SetControllerAction(tuner.direct5T); break;
        case TUNER_ACTION_REVERSE: tuner.SetControllerAction(tuner.reverseIP); break;
        case TUNER_ACTION_REVERSE_5T: tuner.SetControllerAction(tuner.reverse5T); break;
        default: Serial.println("[ERROR] Invalid Action Index"); return;
    }
    Serial.printf("[INFO] Controller Action Set: %d\n", action_idx);
}

void start_tuning_sequence() {
    Serial.println("\n******************************************");
    Serial.println("* [ACTION] Starting PID Auto-Tuning...   *");
    Serial.println("* PLEASE HOLD THE ROBOT VERTICAL         *");
    Serial.println("* ENABLING MOTORS IN 2 SECONDS...        *");
    Serial.println("******************************************");
    delay(2000);

    // Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples)
    tuner.Configure(
        TUNER_INPUT_SPAN_MAX - TUNER_INPUT_SPAN_MIN, 
        TUNER_OUTPUT_SPAN_MAX - TUNER_OUTPUT_SPAN_MIN, 
        TUNER_OUTPUT_START, 
        TUNER_OUTPUT_STEP, 
        TUNER_TEST_TIME_SEC, 
        TUNER_SETTLE_TIME_SEC, 
        TUNER_SAMPLES
    );
    
    is_tuning_active = true;
    Serial.println("[INFO] Tuner Active: Settling Phase Started.");
}

void print_help() {
    Serial.println("\n--- PID Auto-Tuner (sTune) ---");
    Serial.println("Execution Commands:");
    Serial.println("  c       : Start Tuning Sequence (Hold robot!)");
    Serial.println("  w       : Save Tuned Gains to EEPROM");
    Serial.println("  x       : Emergency Stop / Reset");
    Serial.println("");
    Serial.println("IMU & Motor Commands:");
    Serial.println("  i       : Print IMU (Raw vs Offset-Corrected)");
    Serial.println("  dVAL    : Drive Motor Manually (Continuous) (e.g. d200)");
    Serial.println("  tVAL    : Pulse Motor (100ms Kick) (e.g. t400)");
    Serial.println("");
    Serial.println("Query Commands:");
    Serial.println("  r[0-11] : Read Parameter");
    Serial.println("            0:Kp 1:Ki 2:Kd 3:Ti 4:Td 5:ProcGain");
    Serial.println("            6:DeadTime 7:Tau 8:Action 9:SerMode 10:Method");
    Serial.println("            11:AutoTunings(All)");
    Serial.println("");
    Serial.println("Configuration Commands:");
    Serial.println("  m[0-9]  : Get/Set Tuning Method");
    Serial.println("  s[0-3]  : Get/Set Serial Mode");
    Serial.println("  a[0-3]  : Get/Set Action (Direct/Reverse)");
    Serial.println("  h       : Help");
}

void setup() {
    Serial.begin(115200);
    while(!Serial) { delay(10); }; // Wait for serial to be ready
    delay(1000);
    
    print_help();

    // Init HAL
    if (!HAL_EEPROM_Init()) Serial.println("[ERROR] EEPROM Init Failed");
    if (!HAL_IMU_Init()) Serial.println("[ERROR] IMU Init Failed");
    if (!HAL_Motor_Init()) Serial.println("[ERROR] Motor Init Failed");

    // Calibration
    calibrate_imu();

    // Defaults
    apply_tuning_method(TUNER_DEFAULT_RULE);
    apply_serial_mode(TUNER_DEFAULT_SERIAL);
    apply_action(TUNER_DEFAULT_ACTION);
    
    Serial.println("[INFO] System Ready. Send 'c' to start.");
}

void loop() {
    // 1. Hardware Update (Always run fast)
    HAL_Motor_Process();
    HAL_IMU_Update();
    // Invert reading: Fall Forward must be Positive for standard PID logic
    input_pitch = -1.0f * (HAL_IMU_GetPitch() - imu_offset);

    // 2. Control Logic
    if (is_tuning_active) {
        // Run sTune FAST so its state machine transitions properly
        uint8_t state = tuner.Run();
        
        if (state == 2) { // Tunings Ready
            is_tuning_active = false;
            manual_speed_cmd = 0;
            Serial.println("\n******************************************");
            Serial.println("* [SUCCESS] Auto-Tuning Completed!       *");
            Serial.println("******************************************");
            tuner.printResults();
        } 
        else {
            // Safety Check
            if (abs(input_pitch) > 60.0f) {
                is_tuning_active = false;
                manual_speed_cmd = 0;
                Serial.println("\n[ERROR] TILT LIMIT EXCEEDED 60deg - TUNING ABORTED");
            } 
            else {
                // Throttle Motor Writes to 200Hz (5ms)
                if (micros() - last_motor_write_time >= 5000) {
                    last_motor_write_time = micros();
                    HAL_Motor_SetControl((int16_t)output_motor, 0);
                }
            }
        }
    } else {
        // Idle / Manual Mode
        if (micros() - last_motor_write_time >= 5000) {
            last_motor_write_time = micros();
            HAL_Motor_SetControl(manual_speed_cmd, 0);
        }
    }

    // 3. Serial Command Parsing
    if (Serial.available()) {
        String inputStr = Serial.readStringUntil('\n');
        inputStr.trim();
        if (inputStr.length() == 0) return;

        char cmdChar = inputStr.charAt(0);
        String valStr = inputStr.substring(1);
        bool hasVal = (valStr.length() > 0);
        int val = valStr.toInt();

        switch (cmdChar) {
            case 'h': print_help(); break;
            
            case 'i': // IMU Monitoring
                Serial.printf("IMU -> Raw: %.2f | Offset: %.2f | Inverted+Corrected: %.2f\n", 
                    HAL_IMU_GetPitch(), imu_offset, input_pitch);
                break;

            case 'd': // Manual Drive
                if (is_tuning_active) {
                    Serial.println("[ERROR] Cannot drive manually during tuning!");
                } else {
                    manual_speed_cmd = (int16_t)val;
                    Serial.printf("[INFO] Manual Drive Set: %d\n", manual_speed_cmd);
                }
                break;
            
            case 't': // Pulse Test (100ms kick)
                if (is_tuning_active) {
                    Serial.println("[ERROR] Cannot pulse during tuning!");
                } else {
                    if(val == 0) { val=400; }
                    Serial.printf("[INFO] Pulsing Motor: %d for 100ms\n", val);
                    HAL_Motor_SetControl((int16_t)val, 0);
                    delay(100);
                    HAL_Motor_SetControl(0, 0);
                }
                break;

            case 'x': 
                is_tuning_active = false;
                manual_speed_cmd = 0;
                Serial.println("[WARN] Emergency Stop / Reset Triggered");
                break;

            case 'c':
                if (!is_tuning_active) {
                    start_tuning_sequence();
                } else {
                    Serial.println("[WARN] Tuning already in progress!");
                }
                break;

            case 'w': // Write to EEPROM
            {
                HAL_PID_Params_t old_params;
                HAL_PID_Params_t new_params;
                
                // Load existing
                HAL_EEPROM_LoadPID(&old_params);
                
                // Get new from tuner
                tuner.GetAutoTunings(&new_params.kp, &new_params.ki, &new_params.kd);
                
                Serial.println("\n--- EEPROM UPDATE ---");
                Serial.println("Metric | Existing | New (Tuned)");
                Serial.println("-------------------------------");
                Serial.printf("Kp     | %.5f | %.5f\n", old_params.kp, new_params.kp);
                Serial.printf("Ki     | %.5f | %.5f\n", old_params.ki, new_params.ki);
                Serial.printf("Kd     | %.5f | %.5f\n", old_params.kd, new_params.kd);
                
                if (new_params.kp == 0 && new_params.ki == 0 && new_params.kd == 0) {
                    Serial.println("[ERROR] Tuned values are all zero. Run 'c' first!");
                } else {
                    HAL_EEPROM_SavePID(&new_params);
                    Serial.println("[SUCCESS] New values saved to EEPROM.");
                }
                break;
            }

            case 'r':
                if (!hasVal) break;
                switch(val) {
                    case 0: Serial.printf("Kp: %.5f\n", tuner.GetKp()); break;
                    case 1: Serial.printf("Ki: %.5f\n", tuner.GetKi()); break;
                    case 2: Serial.printf("Kd: %.5f\n", tuner.GetKd()); break;
                    case 3: Serial.printf("Ti: %.5f\n", tuner.GetTi()); break;
                    case 4: Serial.printf("Td: %.5f\n", tuner.GetTd()); break;
                    case 5: Serial.printf("ProcessGain: %.5f\n", tuner.GetProcessGain()); break;
                    case 6: Serial.printf("DeadTime: %.5f\n", tuner.GetDeadTime()); break;
                    case 7: Serial.printf("Tau: %.5f\n", tuner.GetTau()); break;
                    case 8: Serial.printf("Action: %d\n", tuner.GetControllerAction()); break;
                    case 9: Serial.printf("SerialMode: %d\n", tuner.GetSerialMode()); break;
                    case 10: Serial.printf("Method: %d\n", tuner.GetTuningMethod()); break;
                    case 11: 
                        float kP, kI, kD;
                        tuner.GetAutoTunings(&kP, &kI, &kD);
                        Serial.printf("AutoTunings -> Kp:%.5f Ki:%.5f Kd:%.5f\n", kP, kI, kD);
                        break;
                    default: Serial.println("[ERROR] Invalid Read Index");
                }
                break;

            case 'm': // Tuning Method
                if (hasVal) {
                    apply_tuning_method(val);
                } else {
                    Serial.printf("Current Method: %d\n", tuner.GetTuningMethod());
                }
                break;

            case 's': // Serial Mode
                if (hasVal) {
                    apply_serial_mode(val);
                } else {
                    Serial.printf("Current Serial Mode: %d\n", tuner.GetSerialMode());
                }
                break;

            case 'a': // Action
                if (hasVal) {
                    apply_action(val);
                } else {
                    Serial.printf("Current Action: %d\n", tuner.GetControllerAction());
                }
                break;

            default:
                Serial.printf("[ERROR] Unknown Command: %c\n", cmdChar);
                break;
        }
    }
}
