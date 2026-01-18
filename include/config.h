#ifndef CONFIG_H
#define CONFIG_H

// MPU6050 Gyro/Accel Calibration Offsets
// I got these values by running the re-implemented test_calibrate_imu which is a mock IMU_Zero from the electonicscats MPU6050 library.
#define MPU_OFF_X_GYRO      115
#define MPU_OFF_Y_GYRO      76
#define MPU_OFF_Z_GYRO      68
#define MPU_OFF_X_ACCEL     -211
#define MPU_OFF_Y_ACCEL     -831
#define MPU_OFF_Z_ACCEL     1053

#endif // CONFIG_H
