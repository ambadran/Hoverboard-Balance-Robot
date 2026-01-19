#include <Arduino.h>
#include "eeprom_hal.h"

void print_help() {
    Serial.println("\n--- EEPROM HAL Test Brain ---");
    Serial.println("Commands:");
    Serial.println("  h          : Print this help menu");
    Serial.println("  i          : Re-initialize EEPROM");
    Serial.println("  S<p>,<i>,<d>: Save PID struct (e.g., S1.5,0.1,0.05)");
    Serial.println("  L          : Load and print PID struct");
    Serial.println("  p<v>       : Set Kp (e.g. p1.2)");
    Serial.println("  k<v>       : Set Ki (e.g. k0.01)");
    Serial.println("  d<v>       : Set Kd (e.g. d0.5)");
    Serial.println("  g          : Get all individual values");
    Serial.println("-----------------------------");
}

void setup() {
    delay(2000);
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    Serial.println("\n[INFO] Starting EEPROM HAL Test...");

    if (HAL_EEPROM_Init()) {
        Serial.println("[INFO] HAL_EEPROM_Init() SUCCESS.");
    } else {
        Serial.println("[ERROR] HAL_EEPROM_Init() FAILED.");
    }

    print_help();
}

void parse_serial_command() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        
        switch (cmd) {
            case 'h':
                print_help();
                break;
                
            case 'i':
                if (HAL_EEPROM_Init()) {
                    Serial.println("\n[INFO] Re-init SUCCESS.");
                } else {
                    Serial.println("\n[ERROR] Re-init FAILED.");
                }
                break;

            case 'S': { // Upper case S for Struct Save
                HAL_PID_Params_t params;
                // Expecting format: S1.5,0.1,0.05
                params.kp = Serial.parseFloat();
                if (Serial.read() == ',') params.ki = Serial.parseFloat();
                if (Serial.read() == ',') params.kd = Serial.parseFloat();
                
                HAL_EEPROM_SavePID(&params);
                Serial.printf("\n[INFO] Saved PID Struct: P=%.4f, I=%.4f, D=%.4f\n", params.kp, params.ki, params.kd);
                break;
            }

            case 'L': { // Upper case L for Struct Load
                HAL_PID_Params_t params = {0};
                HAL_EEPROM_LoadPID(&params);
                Serial.printf("\n[INFO] Loaded PID Struct: P=%.4f, I=%.4f, D=%.4f\n", params.kp, params.ki, params.kd);
                break;
            }

            case 'p': {
                float val = Serial.parseFloat();
                HAL_EEPROM_SetKp(val);
                Serial.printf("\n[INFO] Set Kp=%.4f\n", val);
                break;
            }

            case 'k': {
                float val = Serial.parseFloat();
                HAL_EEPROM_SetKi(val);
                Serial.printf("\n[INFO] Set Ki=%.4f\n", val);
                break;
            }

            case 'd': {
                float val = Serial.parseFloat();
                HAL_EEPROM_SetKd(val);
                Serial.printf("\n[INFO] Set Kd=%.4f\n", val);
                break;
            }

            case 'g': {
                float p = HAL_EEPROM_GetKp();
                float i = HAL_EEPROM_GetKi();
                float d = HAL_EEPROM_GetKd();
                Serial.printf("\n[INFO] Get Individual: P=%.4f, I=%.4f, D=%.4f\n", p, i, d);
                break;
            }

            case '\n':
            case '\r':
                break;

            default:
                Serial.printf("\n[WARN] Unknown command: %c\n", cmd);
                break;
        }
    }
}

void loop() {
    parse_serial_command();
}