#ifndef MPU_H_
#define MPU_H_
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <math.h>
#include <esp_log.h>
#include <esp_timer.h>
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_ADDR 0x68

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);
void gyro_signals(void);
void mpu_read(void);
void mpu_init(void);
extern const char *TAG;
extern float RateRoll, RatePitch, RateYaw;
extern float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
extern int RateCalibrationNumber;
extern float AccX, AccY, AccZ;
extern float AngleRoll, AnglePitch;
extern uint64_t LoopTimer;
extern float KalmanAngleRoll, KalmanUncertaintyAngleRoll;
extern float KalmanAnglePitch, KalmanUncertaintyAnglePitch;
extern float Kalman1DOutput[2];

#endif // MPU_H_