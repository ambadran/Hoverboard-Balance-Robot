#include "imu_hal.h"
#include "pin_map.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

static MPU6050 mpu;

// MPU Control/Status
static bool dmp_ready = false;
static uint8_t dev_status;
static uint16_t packet_size;
static uint8_t fifo_buffer[64];

// Orientation/Motion
static Quaternion q;
static VectorFloat gravity;
static float ypr[3];

// Cached Output (Degrees)
static float current_pitch = 0.0f;
static float current_roll = 0.0f;

bool HAL_IMU_Init(void) {
    // Initialize I2C
    // ESP32 Wire.begin(sda, scl, frequency)
    Wire.begin(MPU6050_I2C_SDA, MPU6050_I2C_SCL, 400000);

    // Initialize Device
    mpu.initialize();
    
    // Verify Connection
    if (!mpu.testConnection()) {
        return false;
    }

    // Initialize DMP
    dev_status = mpu.dmpInitialize();

    // Supply gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(MPU_OFF_X_GYRO);
    mpu.setYGyroOffset(MPU_OFF_Y_GYRO);
    mpu.setZGyroOffset(MPU_OFF_Z_GYRO);
    mpu.setXAccelOffset(MPU_OFF_X_ACCEL);
    mpu.setYAccelOffset(MPU_OFF_Y_ACCEL);
    mpu.setZAccelOffset(MPU_OFF_Z_ACCEL);

    // dev_status 0 = success
    if (dev_status == 0) {
        // Turn on the DMP
        mpu.setDMPEnabled(true);

        // Packet Size
        packet_size = mpu.dmpGetFIFOPacketSize();
        
        dmp_ready = true;
        return true;
    } else {
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        return false;
    }
}

void HAL_IMU_Update(void) {
    if (!dmp_ready) return;

    // dmpGetCurrentFIFOPacket checks for FIFO count and reads if ready
    // It returns true if a packet was successfully read
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
        
        // Extract Data
        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert Radians to Degrees and Store
        // ypr[0] = yaw, ypr[1] = pitch, ypr[2] = roll
        current_pitch = ypr[1] * 180.0f / PI;
        current_roll  = ypr[2] * 180.0f / PI;
    }
}

float HAL_IMU_GetPitch(void) {
    return current_pitch;
}

float HAL_IMU_GetRoll(void) {
    return current_roll;
}

void HAL_IMU_GetOrientation(float* pitch, float* roll) {
    if (pitch) *pitch = current_pitch;
    if (roll)  *roll  = current_roll;
}
