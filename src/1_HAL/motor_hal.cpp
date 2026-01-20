#include "motor_hal.h"
#include "pin_map.h"
#include <Arduino.h>
#include <HardwareSerial.h>

// -- Protocol Constants --
#define START_FRAME         0xABCD
#define FRAME_LENGTH        18      // Total bytes in receive frame

// -- Internal State --
static HoverMotionData_t _feedback; // Stores latest valid data
static uint8_t _rx_buffer[FRAME_LENGTH];
static int _rx_idx = 0;
static unsigned long _last_read_time = 0;

// -- Implementation --

extern "C" {

bool HAL_Motor_Init(void) {
    // Initialize Serial2 for Hoverboard communication
    Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_SERIAL_RX, HOVER_SERIAL_TX);
    
    // Clear buffer
    while(Serial2.available()) {
        Serial2.read();
    }
    
    // Reset internal state
    _rx_idx = 0;
    memset(&_feedback, 0, sizeof(HoverMotionData_t));
    
    return true; // Serial.begin always "succeeds" on ESP32 unless pins are invalid
}

void HAL_Motor_SetControl(int16_t speed_cmd, int16_t steer_cmd) {
    uint16_t start = START_FRAME;
    uint16_t checksum = (uint16_t)(start ^ steer_cmd ^ speed_cmd);

    // Write to UART buffer (non-blocking usually, as buffer is 256 bytes)
    // Send Low Byte first, then High Byte (Little Endian)
    Serial2.write((uint8_t)(start & 0xFF));
    Serial2.write((uint8_t)((start >> 8) & 0xFF));
    
    Serial2.write((uint8_t)(speed_cmd & 0xFF));
    Serial2.write((uint8_t)((speed_cmd >> 8) & 0xFF));
    
    Serial2.write((uint8_t)(steer_cmd & 0xFF));
    Serial2.write((uint8_t)((steer_cmd >> 8) & 0xFF));
    
    Serial2.write((uint8_t)(checksum & 0xFF));
    Serial2.write((uint8_t)((checksum >> 8) & 0xFF));
}

void HAL_Motor_Process(void) {
    while (Serial2.available()) {
        uint8_t ch = Serial2.read();

        // 1. Sliding window / State machine to find header
        if (_rx_idx == 0) {
            // Looking for first byte of Start (0xCD) - Little Endian logic for 0xABCD
            // 0xABCD in Little Endian is: Byte0=0xCD, Byte1=0xAB
            if (ch == (uint8_t)(START_FRAME & 0xFF)) {
                _rx_buffer[_rx_idx++] = ch;
            }
        } 
        else if (_rx_idx == 1) {
            // Looking for second byte of Start (0xAB)
            if (ch == (uint8_t)((START_FRAME >> 8) & 0xFF)) {
                _rx_buffer[_rx_idx++] = ch;
            } else {
                // Mismatch, reset to index 0. 
                // However, if this byte was 0xCD, it might be the start of a new frame, 
                // so we re-evaluate it.
                 _rx_idx = 0;
                 if (ch == (uint8_t)(START_FRAME & 0xFF)) {
                     _rx_buffer[_rx_idx++] = ch;
                 }
            }
        }
        else {
            // Reading payload
            _rx_buffer[_rx_idx++] = ch;

            // Frame Complete?
            if (_rx_idx >= FRAME_LENGTH) {
                // Validate Checksum
                uint16_t start_rx = (uint16_t)_rx_buffer[0] | ((uint16_t)_rx_buffer[1] << 8);
                int16_t cmd1_rx = (int16_t)_rx_buffer[2] | ((int16_t)_rx_buffer[3] << 8);
                int16_t cmd2_rx = (int16_t)_rx_buffer[4] | ((int16_t)_rx_buffer[5] << 8);
                int16_t speedR_rx = (int16_t)_rx_buffer[6] | ((int16_t)_rx_buffer[7] << 8);
                int16_t speedL_rx = (int16_t)_rx_buffer[8] | ((int16_t)_rx_buffer[9] << 8);
                int16_t bat_rx = (int16_t)_rx_buffer[10] | ((int16_t)_rx_buffer[11] << 8);
                int16_t temp_rx = (int16_t)_rx_buffer[12] | ((int16_t)_rx_buffer[13] << 8);
                uint16_t cmdLed_rx = (uint16_t)_rx_buffer[14] | ((uint16_t)_rx_buffer[15] << 8);
                uint16_t checksum_rx = (uint16_t)_rx_buffer[16] | ((uint16_t)_rx_buffer[17] << 8);

                uint16_t calc_checksum = start_rx ^ cmd1_rx ^ cmd2_rx ^ speedR_rx ^ speedL_rx ^ bat_rx ^ temp_rx ^ cmdLed_rx;

                if (calc_checksum == checksum_rx) {
                    // Checksum Match - Update Global State
                    _feedback.cmd1 = cmd1_rx;
                    _feedback.cmd2 = cmd2_rx;
                    _feedback.speedR_meas = speedR_rx;
                    _feedback.speedL_meas = speedL_rx;
                    _feedback.batVoltage = bat_rx;
                    _feedback.boardTemp = temp_rx;
                    _feedback.cmdLed = cmdLed_rx;
                } else {
                    // Checksum Failed - Ignore frame
                }

                // Reset for next frame
                _rx_idx = 0;
            }
        }
    }
}

HoverMotionData_t HAL_Motor_GetFeedback(void) {
    // Returns a copy of the structure (Atomic read for struct? No, but reasonably safe on single core logic if called from same task. 
    // If multithreaded (Core 1 vs Core 0), this needs a lock, but HAL is usually called by one Worker.)
    return _feedback;
}

float HAL_Motor_GetVoltage(void) {
    // ADC Value is usually Volts * 100
    return (float)_feedback.batVoltage / 100.0f;
}

float HAL_Motor_GetTemperature(void) {
    // ADC Value is usually DegC * 10
    return (float)_feedback.boardTemp / 10.0f;
}

int16_t HAL_Motor_GetAverageSpeed(void) {
    return (_feedback.speedL_meas + _feedback.speedR_meas) / 2;
}

} // extern "C"
