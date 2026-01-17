#ifndef PIN_MAP_H
#define PIN_MAP_H

// UART Connection to Hoverboard Mainboard
// ESP32 UART2 (Serial2) is standard for this.
#define HOVER_SERIAL_TX     17
#define HOVER_SERIAL_RX     16
#define HOVER_SERIAL_BAUD   115200

// IMU MPU6050 (I2C)
#define MPU6050_I2C_SDA     22
#define MPU6050_I2C_SCL     21
#define MPU6050_INT         23

#endif // PIN_MAP_H
